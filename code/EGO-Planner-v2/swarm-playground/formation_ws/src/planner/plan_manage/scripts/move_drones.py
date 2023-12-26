import rospy
# from gazebo_msgs.srv import SpawnModel
# from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SpawnModel, SetModelState
from std_msgs.msg import Int16, Bool

def move_cb(msg):
    if (msg.header.frame_id == "null"):
        return
    temp_pose = Pose()
    temp_pose=msg.pose.pose
    model_state = ModelState()
    model_state.model_name = "drone_0"
    model_state.pose=temp_pose
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    resp = set_state(model_state)
    if not resp:
        print("Object cannot be moved")
    
    # rospy.loginfo(rospy.get_caller_id() + "I heard", msg.header.frame_id)
    
def main():
    rospy.init_node('move_drones',log_level=rospy.INFO)

    sub_odom =rospy.Subscriber('/drone_0_visual_slam/odom', Odometry, move_cb)
    # sub_odom =rospy.Subscriber('/drone_1_visual_slam/odom', Odometry, move_cb)
    # sub_odom =rospy.Subscriber('/drone_2_visual_slam/odom', Odometry, move_cb)
    # sub_odom =rospy.Subscriber('/drone_3_visual_slam/odom', Odometry, move_cb)
    

    rospy.spin()


    # model_state = ModelState()
    # model_state.model_name = "drone0"

    # initial_pose = Pose()
    # initial_pose.position.x = 3
    # initial_pose.position.y = 3
    # initial_pose.position.z = 1

    # model_state.pose=initial_pose
    # rospy.wait_for_service('/gazebo/set_model_state')
    # set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    # resp = set_state(model_state)
    # if not resp:
    #     print("Object cannot be moved")

    # /drone_0_visual_slam/odom 

    # while(1):
    #     # initial_pose = Pose()
    #     initial_pose.position.x += 0.01
    #     # initial_pose.position.y = 3
    #     # initial_pose.position.z = 1

    #     model_state.pose=initial_pose
    #     rospy.wait_for_service('/gazebo/set_model_state')
    #     set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    #     resp = set_state(model_state)
    #     if not resp:
    #         print("Object cannot be moved")   
            
        
if __name__ == '__main__':
    main()