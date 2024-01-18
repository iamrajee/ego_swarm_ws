#!/usr/bin/env python3
import rospy
from quadrotor_msgs.msg import GoalSet
from std_msgs.msg import Header
import time

delay_time =0#1    
delay_between_formation_time =10
  
x0,y0=0.5,0.0
x1,y1=0.0,0.5
x2,y2=-0.5,0.0
x3,y3=0.0,-0.5
x4,y4=0.5,0.5
x5,y5=-0.5,0.5
x6,y6=-0.5,-0.5
x7,y7=0.5,-0.5

# x0,y0=0.5,0.0
# x1,y1=0.0,0.5
# x2,y2=-0.5,0.0
# x3,y3=0.0,-0.5
# x4,y4=0.25,0.25
# x5,y5=-0.25,0.25
# x6,y6=-0.25,-0.25
# x7,y7=0.25,-0.25

# x0,y0=1.0,0.0
# x1,y1=0.0,1.0
# x2,y2=-1.0,0.0
# x3,y3=0.0,-1.0
# x4,y4=0.50,0.50
# x5,y5=-0.50,0.50
# x6,y6=-0.50,-0.50
# x7,y7=0.50,-0.50


goalx,goaly=-2.0,-2.0
    
def move(id, x, y, z, goal_pub, rate):
    while not rospy.is_shutdown():
        goal_msg = GoalSet()
        goal_msg.drone_id = id
        goal_msg.goal = [x, y, z]
        goal_pub.publish(goal_msg)
        rospy.loginfo("published once!")#print("published")
        rate.sleep()
        break
    time.sleep(delay_time)
    
def move_formation(goal_pub, rate):
    move(0, goalx + x0, goaly + y0, 0, goal_pub, rate)
    move(1, goalx + x1, goaly + y1, 0, goal_pub, rate)
    move(2, goalx + x2, goaly + y2, 0, goal_pub, rate)
    move(3, goalx + x3, goaly + y3, 0, goal_pub, rate)
    move(4, goalx + x4, goaly + y4, 0, goal_pub, rate)
    move(5, goalx + x5, goaly + y5, 0, goal_pub, rate)
    move(6, goalx + x6, goaly + y6, 0, goal_pub, rate)
    move(7, goalx + x7, goaly + y7, 0, goal_pub, rate)
    time.sleep(delay_between_formation_time)
    print("--------- Next formation ----------")
    
def publish_goal():
    global goalx
    global goaly
    rospy.init_node('goal_publisher', anonymous=True)
    goal_pub = rospy.Publisher('/goal_with_id', GoalSet, queue_size=10)
    rate = rospy.Rate(1)  # 10 Hz
    
    while goal_pub.get_num_connections() == 0:
        rospy.loginfo("Waiting for subscribers to connect")
        rospy.sleep(1)
    
    goalx,goaly=2.0,2.0
    move_formation(goal_pub, rate)
    goalx,goaly=2.0,-2.0
    move_formation(goal_pub, rate)
    goalx,goaly=-2.0,-2.0
    move_formation(goal_pub, rate)
    goalx,goaly=-2.0,2.0
    move_formation(goal_pub, rate)
    # goalx,goaly=0.0,0.0
    # move_formation(goal_pub, rate)

    print("close")

if __name__ == '__main__':
    try:
        time.sleep(10)
        publish_goal()
    except KeyboardInterrupt:
        rospy.loginfo("Ctrl+C detected. Shutting down...")
        rospy.signal_shutdown("Ctrl+C detected")
    # except rospy.ROSInterruptException:
        # pass
