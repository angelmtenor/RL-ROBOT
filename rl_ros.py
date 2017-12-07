# -*- coding: utf-8 -*-
#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" Link with ROS                                        v 0.7  Giraff robot """

import copy
import sys
import time
from threading import Thread, RLock

import numpy as np

try:
    import rospy
    # from std_msgs.msg import String
    # from my_python.srv import *
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist
    from std_srvs.srv import Empty
except ImportError:
    rospy = None
    Odometry = None
    LaserScan = None
    Twist = None
    Empty = None
    sys.exit("\n Please configure your ROS Environment "
             "e.g: $ . ~/catkin_ws/devel/setup.bash")

NODE_NAME = "rl_ros"

HAS_KINECT = False
HAS_ARM = False

N_DISTANCES = 8  # do not modify
N_LASERS = N_DISTANCES  # rl_vrep compatible
# distances measured by lasers:
distance_obstacle = np.full(N_DISTANCES, -1, dtype=np.float64)
# base pose 2D:
mobilebase_pose2d = np.full(3, -1, dtype=np.float64)  # x(m), y(m), theta(rad)

twist = Twist()
twist.linear.x = 0
twist.linear.y = 0
twist.linear.z = 0
twist.angular.x = 0
twist.angular.y = 0
twist.angular.z = 0

pub_cmd_vel = None
srv_reset_positions = None

mutex_odom = RLock()
mutex_base_scan = RLock()
mutex_twist = RLock()


def setup():
    """ Setup ROS node. Create listener and talker threads """
    global pub_cmd_vel
    global srv_reset_positions

    print("\n Initiating ROS Node")

    rospy.init_node(NODE_NAME, anonymous=True)
    # ToDo: Change to 'odom' in ROS launch for Giraff:
    rospy.Subscriber('odom_giraff', Odometry, callback_odom)
    rospy.Subscriber('laser_scan', LaserScan, callback_base_scan)
    pub_cmd_vel = rospy.Publisher(
        'cmd_vel', Twist, queue_size=1, tcp_nodelay=True)

    srv_reset_positions = rospy.ServiceProxy('reset_positions', Empty)

    thread_listener = Thread(target=listener, args=[])
    thread_talker = Thread(target=talker, args=[])
    thread_listener.start()
    thread_talker.start()

    rospy.loginfo("Node Initiated: %s", NODE_NAME)
    return


def callback_odom(data):
    """ callback from odometry subscribed topic. Update mobilebase_pose2d """
    global mobilebase_pose2d
    mutex_odom.acquire()
    mobilebase_pose2d[0] = data.pose.pose.position.x
    mobilebase_pose2d[1] = data.pose.pose.position.y
    mobilebase_pose2d[2] = data.pose.pose.orientation.z
    mutex_odom.release()
    return


def callback_base_scan(data):
    """ callback from odometry subscribed topic. Update distance_obstacle """
    global distance_obstacle
    laser_data = data.ranges
    n_data = len(laser_data)
    mutex_base_scan.acquire()
    # Get specific points from Hokuyo laser (240º)
    distance_obstacle[0] = laser_data[int(n_data / 2)]
    distance_obstacle[1] = laser_data[int(n_data / 2 + 45)]
    distance_obstacle[2] = laser_data[int(n_data / 2 + 90)]
    distance_obstacle[3] = laser_data[n_data - 1]
    distance_obstacle[4] = 50  # no rear laser
    distance_obstacle[5] = laser_data[0]
    distance_obstacle[6] = laser_data[int(n_data / 2 - 90)]
    distance_obstacle[7] = laser_data[int(n_data / 2 - 45)]
    distance_obstacle[distance_obstacle > 50] = 50
    distance_obstacle[np.isnan(distance_obstacle)] = 50
    mutex_base_scan.release()
    len(laser_data)
    return


def listener():
    """ Call rospy spin for subscribed topics"""
    rospy.spin()
    return


def talker():
    """ Publish ROS data """
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        mutex_twist.acquire()
        pub_cmd_vel.publish(twist)
        mutex_twist.release()
        rate.sleep()
    return


def finish():
    """ Close ROS node """
    print("\nFinishing ROS Node \n")
    rospy.signal_shutdown("Finished from RL-ROBOT")
    return


def move_wheels(left_speed, right_speed):
    """ Convert wheel speeds into (v,w) and update published speed values """
    # rospy.init_node('listener', anonymous=True)
    v_speed, w_speed = wheel_to_vw_speed(left_speed, right_speed)

    if any(distance_obstacle < 0.18):
        if v_speed > 0:
            v_speed = 0
    w_speed *= 5  # hand tuned for giraff robot

    mutex_twist.acquire()
    global twist
    twist.linear.x = v_speed
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = w_speed
    mutex_twist.release()
    return


def wheel_to_vw_speed(left_speed, right_speed):
    """  Return v,w speeds of the robot from
    left, right wheel speeds in rad/s """
    d = 0.15  # wheels radius of Giraff
    s = 0.46  # separation between wheels of Giraff
    v_left = left_speed * d / 2
    v_right = right_speed * d / 2
    v = (v_left + v_right) / 2
    w = (v_right - v_left) / s
    # print("v:",v," w:",w")
    return v, w


def get_mobilebase_pose2d():
    """ Returns the pose of the robot:  [ x(m), y(m), Theta(rad)  ] """
    mutex_odom.acquire()
    pos = mobilebase_pose2d
    mutex_odom.release()
    copy_pos = copy.copy(pos)  # fix shared memory issues
    return copy_pos


def get_distance_obstacle():
    """ Returns an array of distances measured by lasers (m) """
    mutex_base_scan.acquire()
    dist = distance_obstacle
    mutex_base_scan.release()
    copy_dist = copy.copy(dist)  # fix shared memory issues
    return copy_dist


def stop_motion():
    """ Stops robot motion """
    move_wheels(0, 0)


def setup_devices():
    """ just for V-Rep Compatibility"""
    pass


def connect():
    """ just for V-Rep Compatibility"""
    return


def disconnect():
    """ just for V-Rep Compatibility"""
    return


def start():
    """ just for V-Rep Compatibility"""
    reset_robot()
    time.sleep(0.5)
    return


def stop():
    """ just for V-Rep Compatibility"""
    reset_robot()
    time.sleep(0.5)
    return


def show_msg(message):
    """ Print a message (ROS log) """
    rl_str = "rl_mapir: " + message
    print(rl_str)
    rospy.loginfo(rl_str)
    return


def reset_robot():
    """ Call to a Reset ROS service if available """
    rospy.wait_for_service('reset_positions')
    return


# ToDo: Implement the following functions to use an arm in ROS

# def moveArm(v):
#    return
#
# def moveBiceps(v):
#    return
#
# def moveForearm(v):
#    return
#
# def stopArmAll(v):
#    return
#
# def getGripperPose3d():
#    return np.array(pos)
#
# def getGoalPose3d():
#    return np.array(pos)
