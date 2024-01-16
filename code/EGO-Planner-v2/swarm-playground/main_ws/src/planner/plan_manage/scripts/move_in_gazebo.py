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

radius = 2
num_points = 2500  # Adjust for desired number of points
trajectory = generate_circular_trajectory(radius, num_points)
trajectory_itr=0



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
    model_state.pose = model_state_of_interest.pose #msg.pose.pose
        
    if int(drone_id)==1:
        trajectory_itr=trajectory_itr%num_points
        x,y = trajectory[trajectory_itr]
        model_state.pose.position.x = x
        model_state.pose.position.y = y
        model_state.pose.position.z = 0.5
        print(x,y)  
        trajectory_itr+=1      

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
        # rospy.sleep(0.1)
        
    rospy.spin()

if __name__ == '__main__':
    main()
