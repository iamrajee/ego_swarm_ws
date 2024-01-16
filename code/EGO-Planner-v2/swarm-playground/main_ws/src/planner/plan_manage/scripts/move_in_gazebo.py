import rospy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SpawnModel, SetModelState, GetModelState
from std_msgs.msg import Int16, Bool
import sys

drone_id = sys.argv[1]

pub = rospy.Publisher('/move_command', Pose, queue_size=1)  # Create publisher for move_cb

drone_cords_offset = [[0.5,0.5,0.5],[0.5,-0.5,0.5],[-0.5,0.5,0.5],[-0.5,-0.5,0.5]]

#drone1->drone2->drone3->drone4->drone1...
number_of_drones=4
drone_id_of_interest=(int(drone_id)%number_of_drones) + 1


def update_position():
    # if msg.header.frame_id == "null":
    #     return

    rospy.wait_for_service('/gazebo/get_model_state')
    get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    model_name_of_interest = "drone_" + str(drone_id_of_interest)
    model_state_of_interest = get_model_state(model_name_of_interest, "")
    
    model_state_of_interest.pose.position.x += model_state_of_interest.pose.position.x + (drone_cords_offset[int(drone_id) - 1][0] - drone_cords_offset[drone_id_of_interest - 1][0])
    model_state_of_interest.pose.position.y += model_state_of_interest.pose.position.y + (drone_cords_offset[int(drone_id) - 1][1] - drone_cords_offset[drone_id_of_interest - 1][1])
    model_state_of_interest.pose.position.z += model_state_of_interest.pose.position.z + (drone_cords_offset[int(drone_id) - 1][2] - drone_cords_offset[drone_id_of_interest - 1][2])

    model_state = ModelState()
    model_state.model_name = "drone_" + drone_id
    model_state.pose = model_state_of_interest.pose #msg.pose.pose

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
        update_position()
        rospy.sleep(5)

    rospy.spin()

if __name__ == '__main__':
    main()
