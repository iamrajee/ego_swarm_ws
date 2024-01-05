import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


rospy.init_node('spawn_drones',log_level=rospy.INFO)

initial_pose = Pose()
initial_pose.position.x = 0
initial_pose.position.y = 0
initial_pose.position.z = 1

f = open('/home/rajendra/ego_swarm_ws/code/EGO-Planner-v2/swarm-playground/formation_ws/src/planner/plan_manage/scripts/drone.sdf','r')
sdff = f.read()

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
spawn_model_prox("drone0", sdff, "robotos_name_space", initial_pose, "world")
initial_pose.position.x, initial_pose.position.y = 0, 1
spawn_model_prox("drone1", sdff, "robotos_name_space", initial_pose, "world")
initial_pose.position.x, initial_pose.position.y = 1, 0
spawn_model_prox("drone2", sdff, "robotos_name_space", initial_pose, "world")
initial_pose.position.x, initial_pose.position.y = 1, 1
spawn_model_prox("drone3", sdff, "robotos_name_space", initial_pose, "world")