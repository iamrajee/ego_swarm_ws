#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from sensor_msgs.msg import Joy
from nav_msgs.msg import Odometry
from traj_utils.msg import MINCOTraj #?
from quadrotor_msgs.msg import GoalSet
import socket
import struct
import threading
from rospy_message_converter import message_converter
import threading

UDP_PORT = 8081
BUF_LEN = 1048576  # 1MB
BUF_LEN_SHORT = 1024  # 1KB

udp_server_fd = None
udp_send_fd = None
other_odoms_sub = None
one_traj_sub = None
mandatory_stop_sub = None
goal_sub = None
joy_sub = None
other_odoms_pub = None
one_traj_pub = None
mandatory_stop_pub = None
goal_pub = None
joy_pub = None
udp_ip = None
drone_id = None
odom_broadcast_freq = None
udp_recv_buf = bytearray(BUF_LEN)
udp_send_buf = bytearray(BUF_LEN)
addr_udp_send = None
odom_msg = None
MINCOTraj_msg = None
stop_msg = None
goal_msg = None
joy_msg = None

MESSAGE_TYPE = {'ODOM': 100, 'ONE_TRAJ': 101, 'STOP': 102, 'GOAL': 103, 'JOY': 104}


def init_broadcast(ip, port):
    global addr_udp_send
    addr_udp_send = (ip, port)
    udp_send_fd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_send_fd.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    return udp_send_fd


def udp_bind_to_port(port):
    server_fd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    server_fd.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR | socket.SO_REUSEPORT, 1)
    server_fd.bind(('0.0.0.0', port))
    return server_fd


def serialize_topic(msg_type, msg):
    msg_type_bytes = struct.pack('I', msg_type)
    msg_size_bytes = struct.pack('I', len(msg.serialize()))
    msg_bytes = message_converter.convert_ros_message_to_dictionary(msg)
    return msg_type_bytes + msg_size_bytes + msg_bytes


def deserialize_topic(data, msg_type):
    msg_type_bytes = struct.unpack('I', data[:4])[0]
    if msg_type_bytes != msg_type:
        return None
    msg_size = struct.unpack('I', data[4:8])[0]
    msg_data = data[8:8 + msg_size]
    return message_converter.convert_dictionary_to_ros_message(msg_data, type(msg))


def odom_sub_udp_cb(msg):
    global udp_send_fd, addr_udp_send
    t_now = rospy.Time.now()
    if (t_now - odom_sub_udp_cb.t_last).to_sec() * odom_broadcast_freq < 1.0:
        return
    odom_sub_udp_cb.t_last = t_now

    msg.child_frame_id = "drone_" + str(drone_id)
    serialized_msg = serialize_topic(MESSAGE_TYPE['ODOM'], msg)

    udp_send_fd.sendto(serialized_msg, addr_udp_send)



def one_traj_sub_udp_cb(msg):
    serialized_msg = serialize_topic(MESSAGE_TYPE['ONE_TRAJ'], msg)
    udp_send_fd.sendto(serialized_msg, addr_udp_send)


def mandatory_stop_sub_udp_cb(msg):
    serialized_msg = serialize_topic(MESSAGE_TYPE['STOP'], msg)
    udp_send_fd.sendto(serialized_msg, addr_udp_send)


def goal_sub_udp_cb(msg):
    serialized_msg = serialize_topic(MESSAGE_TYPE['GOAL'], msg)
    udp_send_fd.sendto(serialized_msg, addr_udp_send)


def joy_sub_udp_cb(msg):
    serialized_msg = serialize_topic(MESSAGE_TYPE['JOY'], msg)
    udp_send_fd.sendto(serialized_msg, addr_udp_send)


def udp_recv_fun():
    global udp_server_fd, udp_recv_buf, other_odoms_pub, one_traj_pub, mandatory_stop_pub, goal_pub, joy_pub

    udp_server_fd = udp_bind_to_port(UDP_PORT, udp_server_fd)
    
    # while True:
    #     try:
    #         data, addr_client = udp_server_fd.recvfrom(BUF_LEN)
    #     except Exception as e:
    #         rospy.logerr(f"Error receiving UDP data: {e}")
    #         continue

    while not rospy.is_shutdown():
        data, addr_client = udp_server_fd.recvfrom(BUF_LEN)

        msg_type = struct.unpack('I', data[:4])[0]
        msg_data = data[8:]
        msg = None

        if msg_type == MESSAGE_TYPE['ODOM']:
            msg = deserialize_topic(msg_data, MESSAGE_TYPE['ODOM'])
            if msg is not None:
                other_odoms_pub.publish(msg)
            else:
                rospy.logerr("Received message length does not match the sent one (2)!!!")

        elif msg_type == MESSAGE_TYPE['ONE_TRAJ']:
            msg = deserialize_topic(msg_data, MESSAGE_TYPE['ONE_TRAJ'])
            if msg is not None:
                one_traj_pub.publish(msg)
            else:
                rospy.logerr("Received message length does not match the sent one (2)!!!")

        elif msg_type == MESSAGE_TYPE['STOP']:
            msg = deserialize_topic(msg_data, MESSAGE_TYPE['STOP'])
            if msg is not None:
                mandatory_stop_pub.publish(msg)
            else:
                rospy.logerr("Received message length does not match the sent one (3)!!!")

        elif msg_type == MESSAGE_TYPE['GOAL']:
            msg = deserialize_topic(msg_data, MESSAGE_TYPE['GOAL'])
            if msg is not None:
                goal_pub.publish(msg)
            else:
                rospy.logerr("Received message length does not match the sent one (4)!!!")

        elif msg_type == MESSAGE_TYPE['JOY']:
            msg = deserialize_topic(msg_data, MESSAGE_TYPE['JOY'])
            if msg is not None:
                joy_pub.publish(msg)
            else:
                rospy.logerr("Received message length does not match the sent one (5)!!!")

        else:
            rospy.logerr("Unknown received message type???")

    udp_server_fd.close()

def main():
    global udp_server_fd, udp_send_fd, other_odoms_sub, one_traj_sub, mandatory_stop_sub, goal_sub, joy_sub
    global other_odoms_pub, one_traj_pub, mandatory_stop_pub, goal_pub, joy_pub
    global udp_ip, drone_id, odom_broadcast_freq

    rospy.init_node("swarm_bridge")  # Add this line to initialize the ROS node
    # nh = rospy.get_param("~")

    udp_ip = rospy.get_param("broadcast_ip", "127.0.0.255")
    drone_id = rospy.get_param("drone_id", 0)#-1)
    odom_broadcast_freq = rospy.get_param("odom_max_freq", 1000.0)
    
    odom_sub_udp_cb.t_last = rospy.Time.now()
    

    if drone_id == -1:
        rospy.logwarn("[swarm bridge] Wrong drone_id!")
        rospy.signal_shutdown("Wrong drone_id!")
        return

    other_odoms_sub = rospy.Subscriber("my_odom", Odometry, odom_sub_udp_cb, tcp_nodelay=True)
    other_odoms_pub = rospy.Publisher("/others_odom", Odometry, queue_size=10)

    one_traj_sub = rospy.Subscriber("/broadcast_traj_from_planner", MINCOTraj, one_traj_sub_udp_cb, tcp_nodelay=True)
    one_traj_pub = rospy.Publisher("/broadcast_traj_to_planner", MINCOTraj, queue_size=100)

    mandatory_stop_sub = rospy.Subscriber("/mandatory_stop_from_users", Empty, mandatory_stop_sub_udp_cb, tcp_nodelay=True)
    mandatory_stop_pub = rospy.Publisher("/mandatory_stop_to_planner", Empty, queue_size=100)

    goal_sub = rospy.Subscriber("/goal_user2brig", GoalSet, goal_sub_udp_cb, tcp_nodelay=True)
    goal_pub = rospy.Publisher("/goal_brig2plner", GoalSet, queue_size=100)

    joy_sub = rospy.Subscriber("/joystick_from_users", Joy, joy_sub_udp_cb, tcp_nodelay=True)
    joy_pub = rospy.Publisher("/joystick_from_bridge", Joy, queue_size=100)

    # boost.thread(udp_recv_fun)
    threading.Thread(target=udp_recv_fun).start()
    # ros.Duration(0.1).sleep()
    rospy.sleep(0.1)

    # UDP connect
    udp_send_fd = init_broadcast(udp_ip, UDP_PORT)

    rospy.spin()

    udp_server_fd.close()
    udp_send_fd.close()

if __name__ == "__main__":
    main()