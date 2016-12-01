# -*- coding: utf-8 -*-
#   +-----------------------------------------------+
#   | RL-ROBOT. Reinforcement Learning for Robotics |
#   | Angel Martinez-Tenor                          |
#   | MAPIR. University of Malaga. 2016             |
#   +-----------------------------------------------+
""" Link with V-REP simulator """
import time

import numpy as np

import vrep  # Vrep API library

WAIT_RESPONSE = False  # True: Synchronous response (too much delay)

LASER_DISTRIBUTION = ('sensor_front', 'sensor_front_left', 'sensor_left',
                      'sensor_rear_left', 'sensor_rear', 'sensor_rear_right',
                      'sensor_right', 'sensor_front_right')
HAS_KINECT = False
HAS_ARM = True
HAS_GOAL_OBJECT = True

# V-REP data transmission modes:
WAIT = vrep.simx_opmode_oneshot_wait
ONESHOT = vrep.simx_opmode_oneshot
STREAMING = vrep.simx_opmode_streaming
BUFFER = vrep.simx_opmode_buffer

if WAIT_RESPONSE:
    MODE_INI = WAIT
    MODE = WAIT
else:
    MODE_INI = STREAMING
    MODE = BUFFER

N_LASERS = 8  # 1 point laser each

robotID = -1
laserID = [-1] * N_LASERS
left_motorID = -1
right_motorID = -1
clientID = -1

armID = -1
bicepsID = -1
forearmID = -1
gripperID = -1

ballID = -1  # Goal object to reach for 2d navigation and 3d arm motion tasks

kinect_rgb_ID = -1  # not used so far
kinect_depth_ID = -1  # not used so far

distance = np.full(N_LASERS, -1, dtype=np.float64)  # distances from lasers (m)
pose = np.full(3, -1, dtype=np.float64)  # Pose 2d base: x(m), y(m), theta(rad)


def show_msg(message):
    """ send a message for printing in V-REP """
    vrep.simxAddStatusbarMessage(clientID, message, WAIT)
    return


def connect():
    """ Connect to the simulator"""
    ip = '127.0.0.1'
    port = 19997
    vrep.simxFinish(-1)  # just in case, close all opened connections
    global clientID
    clientID = vrep.simxStart(ip, port, True, True, 3000, 5)
    # Connect to V-REP
    if clientID == -1:
        import sys
        sys.exit('\nV-REP remote API server connection failed (' + ip +
                 ':' + str(port) + '). Is V-REP running?')
    print('Connected to Robot')
    show_msg('Python: Hello')
    time.sleep(0.5)
    return


def disconnect():
    """ Disconnect from the simulator"""
    # Make sure that the last command sent has arrived
    vrep.simxGetPingTime(clientID)
    show_msg('RL-ROBOT: Bye')
    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
    time.sleep(0.5)
    return


def start():
    """ Start the simulation (force stop and setup)"""
    stop()
    setup_devices()
    vrep.simxStartSimulation(clientID, ONESHOT)
    time.sleep(0.5)
    # Solve a rare bug in the simulator by repeating:
    setup_devices()
    vrep.simxStartSimulation(clientID, ONESHOT)
    time.sleep(0.5)
    return


def stop():
    """ Stop the simulation """
    vrep.simxStopSimulation(clientID, ONESHOT)
    time.sleep(0.5)


def setup_devices():
    """ Assign the devices from the simulator to specific IDs """
    global robotID, left_motorID, right_motorID, laserID
    global armID, bicepsID, forearmID, gripperID
    global kinect_rgb_ID, kinect_depth_ID
    global ballID
    # rc: return_code (not used)
    # robot
    rc, robotID = vrep.simxGetObjectHandle(clientID, 'robot', WAIT)
    # motors
    rc, left_motorID = vrep.simxGetObjectHandle(clientID, 'leftMotor', WAIT)
    rc, right_motorID = vrep.simxGetObjectHandle(clientID, 'rightMotor', WAIT)
    # lasers
    for idx, item in enumerate(LASER_DISTRIBUTION):
        ec, laserID[idx] = vrep.simxGetObjectHandle(clientID, item, WAIT)
    # arm
    if HAS_ARM:
        rc, armID = vrep.simxGetObjectHandle(
            clientID, 'arm_joint', WAIT)
        rc, bicepsID = vrep.simxGetObjectHandle(
            clientID, 'biceps_joint', WAIT)
        rc, forearmID = vrep.simxGetObjectHandle(
            clientID, 'forearm_joint', WAIT)
        rc, gripperID = vrep.simxGetObjectHandle(
            clientID, 'gripper_1_visual', WAIT)
    # Kinect
    if HAS_KINECT:
        rc, kinect_rgb_ID = vrep.simxGetObjectHandle(
            clientID, 'kinect_rgb', WAIT)
        rc, kinect_depth_ID = vrep.simxGetObjectHandle(
            clientID, 'kinect_depth', WAIT)
    # ball
    if HAS_GOAL_OBJECT:
        rc, ballID = vrep.simxGetObjectHandle(clientID, 'Ball', WAIT)

    # start up devices

    # wheels
    vrep.simxSetJointTargetVelocity(clientID, left_motorID, 0, STREAMING)
    vrep.simxSetJointTargetVelocity(clientID, right_motorID, 0, STREAMING)
    # pose
    vrep.simxGetObjectPosition(clientID, robotID, -1, MODE_INI)
    vrep.simxGetObjectOrientation(clientID, robotID, -1, MODE_INI)
    # distances from lasers
    for i in laserID:
        vrep.simxReadProximitySensor(clientID, i, MODE_INI)

    if HAS_ARM:
        vrep.simxSetJointTargetVelocity(clientID, armID, 0, STREAMING)
        vrep.simxSetJointTargetVelocity(clientID, bicepsID, 0, STREAMING)
        vrep.simxSetJointTargetVelocity(clientID, forearmID, 0, STREAMING)
        vrep.simxGetObjectPosition(clientID, gripperID, -1, MODE_INI)

    if HAS_GOAL_OBJECT:
        vrep.simxGetObjectPosition(clientID, ballID, -1, MODE_INI)

    if HAS_KINECT:
        rc, resolution, image = vrep.simxGetVisionSensorImage(
            clientID, kinect_rgb_ID, 0, MODE_INI)
        im = np.array(image, dtype=np.uint8)
        im.resize([resolution[1], resolution[0], 3])
        # plt.imshow(im, origin='lower')
        # return_code, resolution, depth = vrep.simxGetVisionSensorImage(
        #     clientID, kinect_depth_ID, 0, MODE_INI)
        # de = np.array(depth)
        time.sleep(0.5)
    return


def get_image_rgb():
    """ Get RGB image from a Kinect """
    rc, resolution, image = vrep.simxGetVisionSensorImage(
        clientID, kinect_rgb_ID, 0, MODE)

    im = np.array(image, dtype=np.uint8)
    im.resize([resolution[1], resolution[0], 3])
    # im.shape
    # plt.imshow(im,origin='lower')
    return im


def get_image_depth():
    """ get image Depth from a Kinect """
    rc, resolution, depth = vrep.simxGetVisionSensorImage(
        clientID, kinect_depth_ID, 0, MODE)
    de = np.array(depth)
    return de


def get_mobilebase_pose2d():
    """ return the pose of the robot:  [ x(m), y(m), Theta(rad) ] """
    rc, pos = vrep.simxGetObjectPosition(clientID, robotID, -1, MODE)
    rc, ori = vrep.simxGetObjectOrientation(clientID, robotID, -1, MODE)
    pos = np.array([pos[0], pos[1], ori[2]])
    return pos


def get_distance_obstacle():
    """ return an array of distances measured by lasers (m) """
    for i in range(0, N_LASERS):
        rc, ds, detected_point, doh, dsn = vrep.simxReadProximitySensor(
            clientID, laserID[i], MODE)
        distance[i] = detected_point[2]
    return distance


def move_wheels(v_left, v_right):
    """ move the wheels. Input: Angular velocities in rad/s """
    vrep.simxSetJointTargetVelocity(clientID, left_motorID, v_left, STREAMING)
    vrep.simxSetJointTargetVelocity(clientID, right_motorID, v_right, STREAMING)
    return


def stop_motion():
    """ stop the base wheels """
    vrep.simxSetJointTargetVelocity(clientID, left_motorID, 0, STREAMING)
    vrep.simxSetJointTargetVelocity(clientID, right_motorID, 0, STREAMING)
    return


def move_arm(w):
    """ move arm joint. Angular velocities in rad/s (+anticlockwise) """
    vrep.simxSetJointTargetVelocity(clientID, armID, w, STREAMING)


def move_biceps(w):
    """ move biceps joint. Angular velocities in rad/s(+anticlockwise) """
    vrep.simxSetJointTargetVelocity(clientID, bicepsID, w, STREAMING)


def move_forearm(w):
    """ move forearm joint. Angular velocities in rad/s (+anticlockwise) """
    vrep.simxSetJointTargetVelocity(clientID, forearmID, w, STREAMING)


def stop_arm_all():
    """ stop arm joints """
    move_arm(0)
    move_biceps(0)
    move_forearm(0)


def get_gripper_pose3d():
    """ Returns the position of the gripper:  [ x(m), y(m), z(m) ] """
    rc, pos = vrep.simxGetObjectPosition(clientID, gripperID, -1, MODE)
    return np.array(pos)


def get_goal_pose_3d():
    """ Returns the position of the goal object:  [ x(m), y(m), z(m) ] """
    rc, pos = vrep.simxGetObjectPosition(clientID, ballID, -1, MODE)
    return np.array(pos)
