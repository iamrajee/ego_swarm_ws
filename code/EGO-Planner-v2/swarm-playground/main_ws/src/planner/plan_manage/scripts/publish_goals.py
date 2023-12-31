#!/usr/bin/env python3
import rospy
from quadrotor_msgs.msg import GoalSet
from std_msgs.msg import Header
import time

def publish_goal():
    rospy.init_node('goal_publisher', anonymous=True)
    goal_pub = rospy.Publisher('/goal_with_id', GoalSet, queue_size=10)
    rate = rospy.Rate(1)  # 10 Hz

    goal_msg = GoalSet()
    goal_msg.drone_id = 3
    goal_msg.goal = [2.0, 2.0, 0.0]
    
    while goal_pub.get_num_connections() == 0:
        rospy.loginfo("Waiting for subscribers to connect")
        rospy.sleep(1)
        
    # # if rospy.is_shutdown()!=1:
    while not rospy.is_shutdown():
        goal_pub.publish(goal_msg)
        rospy.loginfo("published once!")#print("published")
        rate.sleep()
        break

    print("close")
    
    # goal_msg = GoalSet()
    # goal_msg.drone_id = 0
    # goal_msg.goal = [1.0, 0.0, 0.0]
    # goal_pub.publish(goal_msg)
    # goal_pub.publish(goal_msg)
    
    # time.sleep(1)
    # rospy.sleep(1000)
    # rospy.

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass
