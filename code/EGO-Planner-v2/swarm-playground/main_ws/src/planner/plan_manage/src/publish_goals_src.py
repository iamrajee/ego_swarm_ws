import rospy
from quadrotor_msgs.msg import GoalSet
from std_msgs.msg import Header

def publish_goal():
    rospy.init_node('goal_publisher', anonymous=True)
    goal_pub = rospy.Publisher('/goal_with_id', GoalSet, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        goal_msg = GoalSet()
        goal_msg.drone_id = 5
        # goal_msg.header = Header()
        # goal_msg.header.stamp = rospy.Time.now()
        goal_msg.goal = [5.0, 0.0, 0.0]

        goal_pub.publish(goal_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_goal()
    except rospy.ROSInterruptException:
        pass
