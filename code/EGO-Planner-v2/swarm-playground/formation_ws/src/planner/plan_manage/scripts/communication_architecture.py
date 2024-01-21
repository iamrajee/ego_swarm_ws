#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from traj_utils.msg import MINCOTraj #?
from quadrotor_msgs.msg import GoalSet


no_of_drone = 4
traj_pub = []


def traj_sub_cb(msg):
    global no_of_drone
    global traj_pub
    
    print("------- > drone_id:",msg.drone_id)
    
    # # ring
    # id_of_interest1 = (msg.drone_id+1)%no_of_drone
    # id_of_interest2 = (msg.drone_id-1)%no_of_drone
    # traj_pub[id_of_interest1].publish(msg)
    # traj_pub[msg.drone_id].publish(msg) #? self loop
    # traj_pub[id_of_interest2].publish(msg) #since bidirectional
    
    
    
    # #star
    # if msg.drone_id==0:
    #     for i in range(no_of_drone):
    #         traj_pub[i].publish(msg) #outward and self loop from 0
    # else:
    #       traj_pub[msg.drone_id].publish(msg) #? selfloop of leaf 
    #       traj_pub[0].publish(msg) #inward to 0 from all leaf
            
    #mesh
    for i in range(no_of_drone):
        traj_pub[i].publish(msg) #to all
    
    

def main():
    global no_of_drone
    global traj_pub
    rospy.init_node("com_arch") 
    # nh = rospy.get_param("~")

    # drone_id = rospy.get_param("drone_id", 0) #-1)

    traj_sub = rospy.Subscriber("/broadcast_traj_to_planner", MINCOTraj, traj_sub_cb, tcp_nodelay=True)
    for i in range(no_of_drone):
        traj_pub.append(rospy.Publisher("/broadcast_traj_to_planner_"+str(i), MINCOTraj, queue_size=100))

    # rospy.sleep(0.1)

    rospy.spin()
    
    
if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        rospy.loginfo("Ctrl+C detected. Shutting down...")
        rospy.signal_shutdown("Ctrl+C detected")