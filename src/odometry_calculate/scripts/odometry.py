#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
from std_msgs.msg import Int64, Int16, Int8, String, Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Quaternion, Twist, Pose
from agv_msgs.msg import *
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, sin, cos, atan
import copy

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

from common_function import *

"""


    #####  ######  ####  #        ##   #####    ##   ##### #  ####  #    #
    #    # #      #    # #       #  #  #    #  #  #    #   # #    # ##   #
    #    # #####  #      #      #    # #    # #    #   #   # #    # # #  #
    #    # #      #      #      ###### #####  ######   #   # #    # #  # #
    #    # #      #    # #      #    # #   #  #    #   #   # #    # #   ##
    #####  ######  ####  ###### #    # #    # #    #   #   #  ####  #    #


"""
# referen MIR robot
# fmt: off
ODOM_POSE_COVARIANCE = [0.25, 0, 0, 0, 0, 0,
                        0, 0.25, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 0.1]

ODOM_TWIST_COVARIANCE = [1e-2, 0, 0, 0, 0, 0,
                        0, 1e-2, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 0.1]
# fmt: on

PULLING_FREQ = 10.0

LEFT = 0
RIGHT = 1
LINEAR = 0
ANGULAR = 1

WHEEL_RADIUS = 0.0625
WHEEL_SEPARATION = 0.427
WHEEL_NUM = 2

ENCODER_RESOLUTION = 16384
TICK_PER_ROUND = float(ENCODER_RESOLUTION)
RAD_PER_TICK = (2 * pi) / TICK_PER_ROUND
TICK_PER_RAD = TICK_PER_ROUND / (2 * pi)

LASER_BASE_X = 0.312
LASER_BASE_Y = 0.0
LASER_HEAD_Z = 0.061
LASER_BASE_W = 0.0

odom_pose = [0.0, 0.0, 0.0]
odom_vel = [0.0, 0.0, 0.0]
last_diff_tick = [0, 0]
last_veltocity = [0.0, 0.0]
last_rad = [0.0, 0.0]
last_velocity = [0.0, 0.0]
joint_states = JointState()
motor_encoder_msg = EncoderDifferential()
odom_msg = Odometry()
laser_pose = Pose()
wheel_l = 0.0
wheel_r = 0.0

use_absolute_odom = False
fake_map_odom_tf = False
use_encoder_odom = True
abs_odom_received = False
vel_filter_threshold = 1
arduino_last_time = 0
br = tf.TransformBroadcaster()

"""


  ###### #    # #    #  ####  ##### #  ####  #    #
  #      #    # ##   # #    #   #   # #    # ##   #
  #####  #    # # #  # #        #   # #    # # #  #
  #      #    # #  # # #        #   # #    # #  # #
  #      #    # #   ## #    #   #   # #    # #   ##
  #       ####  #    #  ####    #   #  ####  #    #


"""
# def static_vars(**kwargs):
#     def decorate(func):
#         for k in kwargs:
#             setattr(func, k, kwargs[k])
#         return func
#     return decorate


def load_param():
    global WHEEL_RADIUS, WHEEL_SEPARATION, ENCODER_RESOLUTION, TICK_PER_RAD, TICK_PER_ROUND, RAD_PER_TICK

    WHEEL_RADIUS = rospy.get_param("~wheel_radius", 0.075)
    WHEEL_SEPARATION = rospy.get_param("~wheel_separation", 0.523)

    ENCODER_RESOLUTION = rospy.get_param("~encoder_resolution", 1024)
    TICK_PER_ROUND = float(ENCODER_RESOLUTION)
    RAD_PER_TICK = (2 * pi) / TICK_PER_ROUND
    TICK_PER_RAD = TICK_PER_ROUND / (2 * pi)
    rospy.loginfo(
        "Odom param: {}, {}, {}".format(
            WHEEL_RADIUS, WHEEL_SEPARATION, ENCODER_RESOLUTION
        )
    )


def deg_to_rad(deg):
    return deg * pi / 180.0


def rad_to_deg(rad):
    return rad * 180.0 / pi


def tick_to_rad(tick):
    return tick * RAD_PER_TICK


def initJointStates():
    global joint_states

    joint_states_name = ["wheel_left_joint", "wheel_right_joint"]

    # joint_states.header.frame_id = joint_state_header_frame_id
    # joint_states.name            = joint_states_name

    # joint_states.name_length     = WHEEL_NUM
    # joint_states.position_length = WHEEL_NUM
    # joint_states.velocity_length = WHEEL_NUM
    # joint_states.effort_length   = WHEEL_NUM


def updateTF():
    pass


def update_tf(br):
    if fake_map_odom_tf:
        map_quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
        br.sendTransform(
            (0.0, 0.0, 0.0),
            (
                map_quaternion[0],
                map_quaternion[1],
                map_quaternion[2],
                map_quaternion[3],
            ),
            rospy.Time.now(),
            "odom",
            "map",
        )

    br.sendTransform(
        (
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z,
        ),
        (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w,
        ),
        rospy.Time.now(),
        "base_footprint",
        "odom",
    )


@static_vars(joint_states_pos=[0.0, 0.0], joint_states_vel=[0.0, 0.0])
def updateJointStates():
    updateJointStates.joint_states_pos[LEFT] = last_rad[LEFT]
    updateJointStates.joint_states_pos[RIGHT] = last_rad[RIGHT]
    updateJointStates.joint_states_vel[LEFT] = last_velocity[LEFT]
    updateJointStates.joint_states_vel[RIGHT] = last_velocity[RIGHT]

    joint_states.position = updateJointStates.joint_states_pos
    joint_states.velocity = updateJointStates.joint_states_vel


@static_vars(last_tick=[0, 0])
def updateMotorInfo(left_tick, right_tick):
    global last_diff_tick

    last_diff_tick[LEFT] = left_tick - updateMotorInfo.last_tick[LEFT]
    updateMotorInfo.last_tick[LEFT] = left_tick
    last_rad[LEFT] += tick_to_rad(last_diff_tick[LEFT])

    last_diff_tick[RIGHT] = right_tick - updateMotorInfo.last_tick[RIGHT]
    updateMotorInfo.last_tick[RIGHT] = right_tick
    last_rad[RIGHT] += tick_to_rad(last_diff_tick[RIGHT])


@static_vars(
    last_theta=0.0, rad=0.0, theta=0.0, vel_x_zero_cnt=0, vel_w_zero_cnt=0
)
def calcOdometry(diff_time, update_pose=True):
    global last_diff_tick, last_veltocity, odom_msg, odom_pose, wheel_l, wheel_r
    step_time = diff_time

    if step_time < 0.01:
        return False

    updateMotorInfo(motor_encoder_msg.left, motor_encoder_msg.right)

    # wheel_l = tick_to_rad(last_diff_tick[LEFT])
    # wheel_r = tick_to_rad(last_diff_tick[RIGHT])
    wheel_l_filter = tick_to_rad(last_diff_tick[LEFT])
    wheel_r_filter = tick_to_rad(last_diff_tick[RIGHT])
    if abs(wheel_l_filter) < 2:
        wheel_l = wheel_l_filter
    else:
        rospy.logerr("Odometry left out of range")
        return

    if abs(wheel_r_filter) < 2:
        wheel_r = wheel_r_filter
    else:
        rospy.logerr("Odometry right out of range")
        return

    delta_s = WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0
    theta_s = WHEEL_RADIUS * (wheel_r - wheel_l) / WHEEL_SEPARATION
    calcOdometry.theta = calcOdometry.theta + theta_s

    delta_theta = theta_s  # calcOdometry.theta - calcOdometry.last_theta
    calcOdometry.last_theta = calcOdometry.theta

    if update_pose:
        odom_msg.pose.pose.position.x += delta_s * cos(
            calcOdometry.rad + (delta_theta / 2.0)
        )
        odom_msg.pose.pose.position.y += delta_s * sin(
            calcOdometry.rad + (delta_theta / 2.0)
        )
    calcOdometry.rad += delta_theta

    if delta_s != 0.0:
        odom_msg.twist.twist.linear.x = delta_s / float(step_time)
        calcOdometry.vel_x_zero_cnt = 0
    if delta_theta != 0.0:
        odom_msg.twist.twist.angular.z = delta_theta / float(step_time)
        calcOdometry.vel_w_zero_cnt = 0

    if delta_s == 0.0:
        calcOdometry.vel_x_zero_cnt += 1
        # if calcOdometry.vel_x_zero_cnt <= vel_filter_threshold:
        #     print(motor_encoder_msg.left, motor_encoder_msg.right)
        #     rospy.logerr('delta_s = 0')
        if calcOdometry.vel_x_zero_cnt > vel_filter_threshold:
            odom_msg.twist.twist.linear.x = 0.0

    if delta_theta == 0.0:
        calcOdometry.vel_w_zero_cnt += 1
        # if calcOdometry.vel_w_zero_cnt <= vel_filter_threshold:
        #     print(motor_encoder_msg.left, motor_encoder_msg.right)
        #     rospy.logwarn('delta_theta = 0')
        if calcOdometry.vel_w_zero_cnt > vel_filter_threshold:
            odom_msg.twist.twist.angular.z = 0.0

    last_veltocity[LEFT] = wheel_l / float(step_time)  # rad/s
    last_veltocity[RIGHT] = wheel_r / float(step_time)  # rad/s
    # motor_vel_msg = ('rad/s: ' + str(round(last_veltocity[LEFT], 2)) + ', ' + str(round(last_veltocity[RIGHT], 2)))
    # print(motor_vel_msg)
    # motor_vel_pub.publish(String(motor_vel_msg))

    if update_pose:
        odom_msg.pose.pose.position.z = 0.0
        abc = quaternion_from_euler(0.0, 0.0, calcOdometry.rad)
        odom_msg.pose.pose.orientation.x = abc[0]
        odom_msg.pose.pose.orientation.y = abc[1]
        odom_msg.pose.pose.orientation.z = abc[2]
        odom_msg.pose.pose.orientation.w = abc[3]
    # print("orientation : ")
    # print(abc)

    # Covariance
    odom_msg.pose.covariance = ODOM_POSE_COVARIANCE
    odom_msg.twist.covariance = ODOM_TWIST_COVARIANCE


def motor_encoder_cb(msg):
    global motor_encoder_msg, odom_msg, arduino_last_time
    motor_encoder_msg = msg

    if use_absolute_odom and abs_odom_received:
        odom_pose = offset_pose_xyz(
            laser_pose, -LASER_BASE_X, -LASER_BASE_Y, -LASER_BASE_W
        )
        odom_msg.pose.pose.position.x = odom_pose.position.x
        odom_msg.pose.pose.position.y = odom_pose.position.y
        odom_msg.pose.pose.orientation = odom_pose.orientation

    if (use_absolute_odom and abs_odom_received) or not use_absolute_odom:
        if use_encoder_odom:
            arduino_current_time = motor_encoder_msg.header.stamp.to_sec()
            arduino_diff_time = arduino_current_time - arduino_last_time
            calcOdometry(arduino_diff_time, not use_absolute_odom)
            arduino_last_time = arduino_current_time
        update_tf(br)
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_footprint"
        odom_pub.publish(odom_msg)


def reset_odom_cb(msg):
    # global odom_msg
    odom_msg.pose.pose.position.x = 0.0
    odom_msg.pose.pose.position.y = 0.0
    odom_msg.pose.pose.orientation.x = 0.0
    odom_msg.pose.pose.orientation.y = 0.0
    odom_msg.pose.pose.orientation.z = 0.0
    odom_msg.pose.pose.orientation.w = 1.0
    calcOdometry.last_theta = 0.0
    calcOdometry.rad = 0.0
    calcOdometry.theta = 0.0
    calcOdometry.vel_x_zero_cnt = 0.0
    calcOdometry.vel_w_zero_cnt = 0.0


def absolute_odom_cb(msg):
    global laser_pose, abs_odom_received
    if not abs_odom_received:
        rospy.loginfo("slamware/odom receive")
    abs_odom_received = True

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    ox = msg.pose.pose.orientation.x
    oy = msg.pose.pose.orientation.y
    oz = msg.pose.pose.orientation.z
    ow = msg.pose.pose.orientation.w
    z = euler_from_quaternion([ox, oy, oz, ow])

    laser_pose = copy.deepcopy(msg.pose.pose)


"""


    #    #   ##   # #    #
    ##  ##  #  #  # ##   #
    # ## # #    # # # #  #
    #    # ###### # #  # #
    #    # #    # # #   ##
    #    # #    # # #    #


"""

odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
reset_encoder_pub = rospy.Publisher("/reset_encoder", Empty, queue_size=10)
motor_vel_pub = rospy.Publisher("/motor_vel", String, queue_size=10)


def main():
    global odom_msg, use_absolute_odom, use_encoder_odom, fake_map_odom_tf, vel_filter_threshold

    rospy.init_node("odometry")
    rospy.loginfo("Init node odometry")

    rospy.Subscriber("/motor_encoder", EncoderDifferential, motor_encoder_cb)
    rospy.Subscriber("/reset_odom", Empty, reset_odom_cb)
    load_param()
    use_absolute_odom = rospy.get_param("~use_absolute_odom", False)
    use_encoder_odom = rospy.get_param("~use_encoder_odom", True)
    fake_map_odom_tf = rospy.get_param("~fake_map_odom_tf", False)
    vel_filter_threshold = rospy.get_param("~vel_filter_threshold", 1)

    rospy.loginfo("use_absolute_odom: {}".format(use_absolute_odom))
    rospy.loginfo("use_encoder_odom: {}".format(use_encoder_odom))
    rospy.loginfo("fake_map_odom_tf: {}".format(fake_map_odom_tf))

    if use_absolute_odom:
        rospy.Subscriber("/absolute_odom", Odometry, absolute_odom_cb)

    reset_encoder_pub.publish(Empty())
    listener = tf.TransformListener()

    arduino_last_time = rospy.get_time()

    initJointStates()
    rate = rospy.Rate(PULLING_FREQ)
    rospy.spin()


if __name__ == "__main__":
    main()
