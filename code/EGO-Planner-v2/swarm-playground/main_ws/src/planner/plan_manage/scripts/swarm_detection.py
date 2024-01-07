#!/usr/bin/env python3

# from math import nan
import rospy
from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import Header
from std_msgs.msg import String
import csv  
from std_msgs.msg import Int16


from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np

import tf
# tf2_ros
from tf import TransformListener

import cv2
from cv_bridge import CvBridge, CvBridgeError
bounding_boxes = []
yolo_img_pub, poseArray_pub, nms_bbox_pub = None, None, None
box_z = {}


def non_max_suppression_slow(boxes, overlapThresh):
    # if there are no boxes, return an empty list
    if len(boxes) == 0:
        return []
    # initialize the list of picked indexes
    pick = []
    x1 = np.array([x.xmin for x in boxes])
    y1 = np.array([x.ymin for x in boxes])
    x2 = np.array([x.xmax for x in boxes])
    y2 = np.array([x.ymax for x in boxes])
    scores = np.array([x.probability for x in boxes])

    # compute the area of the bounding boxes and sort the bounding
    # boxes by the bottom-right y-coordinate of the bounding box
    area = (x2 - x1 + 1) * (y2 - y1 + 1)
    # idxs = np.argsort(y2)
    idxs = np.argsort(area)
    # keep looping while some indexes still remain in the indexes
    # list
    while len(idxs) > 0:
        # grab the last index in the indexes list, add the index
        # value to the list of picked indexes, then initialize
        # the suppression list (i.e. indexes that will be deleted)
        # using the last index
        last = len(idxs) - 1
        i = idxs[last]
        pick.append(i)
        suppress = [last]
        # loop over all indexes in the indexes list
        for pos in range(0, last):  
            # grab the current index
            j = idxs[pos]
            '''
                Let say the new bounding box formed by intersection is b3, then to find area of b3 we need to find it parameters.
                x_sort = sort(b1.xmin, b1.xmax, b2.xmin, b2.xmax)
                y_sort = sort(b1.ymin, b1.ymax, b2.ymin, b2.ymax)
                b3.xmin, b3.xmax = x_sort[1], x_sort[2] #i.e 2nd & 3rd element
                b3.ymin, b3.ymax = y_sort[1], y_sort[2] #i.e 2nd & 3rd element
                b3.area = (b3.xmax - b3.xmin)*(b3.ymax-b3.ymin)
                Below is a alterate way to find b3 parameter
            '''
            # find the largest (x, y) coordinates for the start of
            # the bounding box and the smallest (x, y) coordinates
            # for the end of the bounding box
            xx1 = max(x1[i], x1[j]) #x_sort[1] = maxima of b1.xmin & b2.xmin i.e 2nd element of sort(b1.xmin, b1.xmax, b2.xmin, b2.xmax)
            yy1 = max(y1[i], y1[j]) #y_sort[1] = maxima of b1.ymin & b2.ymin i.e 2nd element of sort(b1.ymin, b1.ymax, b2.ymin, b2.ymax)
            xx2 = min(x2[i], x2[j]) #x_sort[2] = minima of b1.xmax & b2.xmax i.e 3rd element of sort(b1.xmin, b1.xmax, b2.xmin, b2.xmax)
            yy2 = min(y2[i], y2[j]) #y_sort[2] = minima of b1.ymax & b2.ymax i.e 3rd element of sort(b1.ymin, b1.ymax, b2.ymin, b2.ymax)
            # compute the width and height of the bounding box
            w = max(0, xx2 - xx1) # + 1)
            h = max(0, yy2 - yy1) # + 1)
            # compute the ratio of overlap between the computed
            # bounding box and the bounding box in the area list

            ''' We are not using iou'''
            # iou = float(w * h) / (area[i] + area[j] - float(w * h))
            
            ''' Instead we are using (intersection_area / area_of_smallest_box)'''
            overlap = float(w * h) / area[j]
           
            if overlap > overlapThresh:
                suppress.append(pos)
      
        idxs = np.delete(idxs, suppress)
    
    return [boxes[i] for i in pick]

#============================ Bounding Box data Callback ==========================#
def callback_boundingBox(data):
    global bounding_boxes
    global pc2_data
    global nms_bbox_pub
    bounding_boxes = data.bounding_boxes
    
    input_nms = bounding_boxes

    pick = non_max_suppression_slow(input_nms, 0.5) 
    bounding_boxes = pick 
    bbox_msg = BoundingBoxes()
    bbox_msg.header.frame_id = "detection"
    bbox_msg.header.stamp = rospy.Time.now()
    bbox_msg.image_header.frame_id = "stereo_camera2_link"
    bbox_msg.image_header.stamp = rospy.Time.now()
    bbox_msg.bounding_boxes = bounding_boxes
    nms_bbox_pub.publish(bbox_msg)


    

pc2_data = None
temp_drone_x = None
temp_drone_y = None
temp_drone_z = None

#============================ PointCloud data Callback ==========================#
def callback_pointcloud(data):
    global pc2_data
    global temp_drone_x, temp_drone_y, temp_drone_z
    pc2_data = data
def get_z(x,y):
    global pc2_data
    assert isinstance(pc2_data, PointCloud2)
    gen = point_cloud2.read_points(pc2_data, field_names=("x", "y", "z"), skip_nans=False, uvs=[(x, y)])
    next_gen = next(gen)
    temp_drone_x = next_gen[0]
    temp_drone_y = next_gen[1]
    temp_drone_z = next_gen[2]
    return (temp_drone_x, temp_drone_y, temp_drone_z)


#============================ Image data Callback ==========================#
def image_callback(img_msg):
    try:
        # Initialize the CvBridge class
        bridge = CvBridge()
        # cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Show the converted image
    show_image(cv_image)
    
#============================ Function to show/publish image ==========================#
def show_image(bgr_img):
    global bounding_boxes
    global actual_steps
    global previous_actual_steps
    global temp_drone_x, temp_drone_y, temp_drone_z
    global yolo_img_pub, poseArray_pub
    image = bgr_img

    font = cv2.FONT_HERSHEY_SIMPLEX
    org = (00, 185) # org
    fontScale = 1 # fontScale
    color = (0, 0, 255) # Red color in BGR
    thickness = 2 # Line thickness of 2 px
   

    poseArray = PoseArray()
    poseArray.header.frame_id = "world"
    poseArray.header.stamp = rospy.Time.now()
    with open('/home/u20/swarm_ws/csv/Detection.csv', 'a+', encoding='UTF8') as f:
        writer = csv.writer(f)
       
        for i, box in enumerate(bounding_boxes):
            image = cv2.rectangle(image, (box.xmin, box.ymin), (box.xmax, box.ymax), (36,255,12), 2)
            text = box.Class
            if text == "aeroplane" or text == "bird":
                text = "drone"
            image = cv2.putText(image, text, (box.xmin, box.ymin - 10), font, fontScale, color, thickness, cv2.LINE_AA, False) # Using cv2.putText() method
            
            if pc2_data == None:
                temp_drone_x, temp_drone_y, temp_drone_z = None, None, None
            else:
                (temp_drone_x, temp_drone_y, temp_drone_z) = get_z(int(round((box.xmin+box.xmax)/2)),int(round((box.ymin+box.ymax)/2)))
          
            if np.isnan(temp_drone_x)!=1:
                p1 = PoseStamped()
                p1.header.frame_id = "stereo_camera2_link" #world
                p1.pose.position.x,p1.pose.position.y, p1.pose.position.z  = temp_drone_x, temp_drone_y, temp_drone_z
                p1.pose.orientation.z = -0.7071068
                p1.pose.orientation.w = 0.7071068
                listener = tf.TransformListener()
                listener.waitForTransform("/world", "/stereo_camera2_link", rospy.Time(), rospy.Duration(4.0))
                p_in_base = listener.transformPose("/world", p1)
                if actual_steps != previous_actual_steps:
                    print("Step: {}, X: {}, Y: {}, Z: {} Probability: {}".format(actual_steps, round(p_in_base.pose.position.x,5),round(p_in_base.pose.position.y,5), round(p_in_base.pose.position.z,5), box.probability))
                    writer.writerow([actual_steps, round(p_in_base.pose.position.x,4)*100,round(p_in_base.pose.position.y,4)*100, round(p_in_base.pose.position.z,4)*100, box.probability])
                pose = Pose()
                pose = p_in_base.pose
                poseArray.poses.append( pose )
            else:
                print("None found!!!!")
    previous_actual_steps = actual_steps
    poseArray_pub.publish( poseArray )
    print("------------------------------")
    yolo_img_pub.publish(CvBridge().cv2_to_imgmsg(image, "bgr8"))
    bounding_boxes = []

actual_steps = 0
previous_actual_steps = -1
def step_callback(data):
    global actual_steps
    # print("=============> data:",data.data)
    actual_steps = data.data

def main():
    with open('Detection.csv', 'w', encoding='UTF8') as f:
        writer = csv.writer(f)
        writer.writerow(["steps","X","Y","Z","Probability"])
    global yolo_img_pub, poseArray_pub, nms_bbox_pub
    global bounding_boxes
    rospy.init_node('yolo_img_pub', anonymous=False)
    rospy.Subscriber("/stereo_camera/left/image_raw", Image, image_callback)
    rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes , callback_boundingBox)
    rospy.Subscriber('/stereo_camera/points2', PointCloud2, callback_pointcloud)
    rospy.Subscriber("/steps_done", Int16, step_callback)
    yolo_img_pub = rospy.Publisher("/yolo_img_pub/image_raw", Image, queue_size=50) #Publish Final Processed image
    poseArray_pub = rospy.Publisher("/yolo_poseArray", PoseArray, queue_size=50) #Publish poseArray
    nms_bbox_pub = rospy.Publisher('/darknet_ros/bounding_boxes_nms', BoundingBoxes, queue_size=10) #Publish BoundingBoxes
    
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try :
        main()
    except rospy.ROSInterruptException:
        pass
