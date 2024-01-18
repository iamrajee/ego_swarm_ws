import rospy
# from gazebo_msgs.srv import SpawnModel
# from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SpawnModel, SetModelState
from std_msgs.msg import Int16, Bool
import sys
drone_id=sys.argv[1]
# drone_id = str(0)
# device = rospy.get_param('/node_name/mode')
# drone_id=str(rospy.get_param(rospy.get_namespace()+"/drone_in_gazebo/"+"drone_id"))
# print("================>>>>> ",drone_id, " <<<<<==============")
# drone_id=rospy.resolve_name("drone_id")
# drone_id=str(rospy.get_param("drone_id"))

def move_cb(msg):
    if (msg.header.frame_id == "null"):
        return
    # temp_pose = Pose()
    # temp_pose=msg.pose.pose
    model_state = ModelState()
    model_state.model_name = "drone_"+drone_id
    model_state.pose=msg.pose.pose #temp_pose
    model_state.twist=msg.twist.twist
    rospy.wait_for_service('/gazebo/set_model_state')
    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    resp = set_state(model_state)
    if not resp:
        print("Object cannot be moved")
    
    # rospy.loginfo(rospy.get_caller_id() + "I heard", msg.header.frame_id)
    
def spawn_drone():
    initial_pose = Pose()
    initial_pose.position.x = 0
    initial_pose.position.y = 0
    initial_pose.position.z = 1

    f = open('/home/rajendra/ego_swarm_ws/code/EGO-Planner-v2/swarm-playground/formation_ws/src/planner/plan_manage/scripts/drone.sdf','r')
    sdff = f.read()

    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_prox("drone_" + drone_id, sdff, "robotos_name_space", initial_pose, "world")
    
def main():
    rospy.init_node('drone_'+drone_id+"_in_gazebo",log_level=rospy.INFO)
    spawn_drone()

    sub_odom =rospy.Subscriber('/drone_'+drone_id+'_visual_slam/odom', Odometry, move_cb, queue_size=1)

    rospy.spin() 
            
        
if __name__ == '__main__':
    main()