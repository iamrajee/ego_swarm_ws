import rospy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SetModelState, GetModelState
from std_msgs.msg import Int16, Bool
import sys


import math

def generate_circular_trajectory(radius, num_points):
  theta = 0
  points = []
  for i in range(num_points):
    x = radius * math.cos(theta)
    y = radius * math.sin(theta)
    points.append((x, y))
    theta += 2 * math.pi / num_points  # Increment angle for next point
  return points

# def generate_zigzag_circular_trajectory(radius, num_points):
#     points = []
#     direction = 1  # Start with positive direction
#     for i in range(num_points):
#         theta = 2 * math.pi * i / num_points  # Calculate angle
#         x = radius * math.cos(theta)
#         y = radius * math.sin(theta) * direction  # Apply direction switch
#         points.append((x, y))
#         direction *= -1  # Switch direction for next point
#     return points

def generate_zigzag_circular_trajectory(radius, num_points):
    points = []
    for i in range(num_points):
        theta = 2 * math.pi * i / num_points
        x = radius * math.cos(theta)
        y = radius * math.sin(theta) * math.sin(2 * math.pi * i / 5)  # Sinusoidal zig-zag
        points.append((x, y))
    return points

# def generate_zigzag_circular_trajectory(radius, num_points):
#     points = []
#     segment_length = 10  # Adjust as needed
#     for i in range(0, num_points, segment_length):
#         slope = 1 if i % (2 * segment_length) == 0 else -1  # Alternate slope
#         for j in range(segment_length):
#             theta = 2 * math.pi * (i + j) / num_points
#             x = radius * math.cos(theta)
#             y = radius * math.sin(theta) * slope
#             points.append((x, y))
#     return points

radius = 3
num_points = 100  # Adjust for desired number of points
trajectory = generate_circular_trajectory(radius, num_points)
lenfactor = 5 
trajectory = [[lenfactor,lenfactor], [lenfactor,-lenfactor], [-lenfactor,-lenfactor], [-lenfactor,lenfactor] ]
# trajectory = generate_zigzag_circular_trajectory(radius, num_points)
trajectory_itr=0
num_points = len(trajectory)



drone_id = sys.argv[1]

pub = rospy.Publisher('/move_command', Pose, queue_size=1)  # Create publisher for move_cb

drone_cords_offset = [[0.5,0.5,0.5],[0.5,-0.5,0.5],[-0.5,0.5,0.5],[-0.5,-0.5,0.5]]

#drone1->drone2->drone3->drone4->drone1...

number_of_drones=4

# Ring
drone_id_of_interest=(int(drone_id)%number_of_drones) + 1

# star
# drone_id_of_interest= 1

def update_position():
    
    global num_points
    global trajectory
    global trajectory_itr
    global lenfactor
    # if msg.header.frame_id == "null":
    #     return

    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    
    model_name_of_interest = "drone_" + str(drone_id_of_interest)
    model_state_of_interest = get_model_state(model_name_of_interest, "")
    
    model_state_of_interest.pose.position.x += (drone_cords_offset[int(drone_id) - 1][0] - drone_cords_offset[drone_id_of_interest - 1][0])
    model_state_of_interest.pose.position.y += (drone_cords_offset[int(drone_id) - 1][1] - drone_cords_offset[drone_id_of_interest - 1][1])
    model_state_of_interest.pose.position.z=0.5#+= model_state_of_interest.pose.position.z + (drone_cords_offset[int(drone_id) - 1][2] - drone_cords_offset[drone_id_of_interest - 1][2])

    model_state = ModelState()
    model_state.model_name = "drone_" + drone_id
    
    if int(drone_id)!=1:
        model_state.pose = model_state_of_interest.pose #msg.pose.pose
        model_state.twist = model_state_of_interest.twist
    # model_state.pose.position.z=0.5
    
        
    # model_state.twist.linear.x = 0.1
    # model_state.twist.linear.y = 0.1   
        
    if int(drone_id)==1:
        print("trajectory_itr:",trajectory_itr)
        current_model_state = get_model_state(model_state.model_name, "")
        currx, curry = current_model_state.pose.position.x, current_model_state.pose.position.y
        trajectory_itr=trajectory_itr%num_points
        x,y = trajectory[trajectory_itr]
        xvel=x-currx
        yvel=y-curry
    
        vfactor=0.1*lenfactor
        
        model_state.pose.position.x = currx#x
        model_state.pose.position.y = curry#y
        model_state.pose.position.z = 0.5
        model_state.twist.linear.x = xvel/(xvel**2+yvel**2)**(1/2)*vfactor
        model_state.twist.linear.y = yvel/(xvel**2+yvel**2)**(1/2)*vfactor
        
        if xvel**2 < 0.25 and yvel**2 < 0.25:
            trajectory_itr+=1    
            
        
        # trajectory_itr=trajectory_itr%num_points
        # x,y = trajectory[trajectory_itr]
        # model_state.pose.position.x = x
        # model_state.pose.position.y = y
        # model_state.pose.position.z = 0.5
        # print(x,y)  
        # trajectory_itr+=1      
        # xvel,yvel = trajectory[(trajectory_itr+1)%num_points]
        # xvel-=x
        # yvel-=y
        # vfactor=1*lenfactor #0.05
        # model_state.twist.linear.x = xvel/(xvel**2+yvel**2)**(1/2)*vfactor
        # model_state.twist.linear.y = yvel/(xvel**2+yvel**2)**(1/2)*vfactor
        
        # model_state.pose.orientation.w = 1

    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    resp = set_state(model_state)

    if not resp:
        print("Object cannot be moved")

    # pub_odom.publish(msg.pose.pose)  # Publish the pose


def main():
    rospy.init_node('move_' + drone_id + "_in_gazebo", log_level=rospy.INFO)
    # spawn_drone()

    # pub_odom = rospy.Publisher('/drone_' + drone_id + '_visual_slam/odom', Odometry)
    while not rospy.is_shutdown():
        print("--------------drone:", drone_id)
        update_position()
        rospy.sleep(0.2)
        
    rospy.spin()

if __name__ == '__main__':
    main()
