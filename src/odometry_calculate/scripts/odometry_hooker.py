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


class Odometry_hooker(object):
    def __init__(self, name, *args, **kwargs) :
        self.init_variable()
        self.load_param()
        self.init_node()
        self.init_node()
        self.initJointStates()
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

    def init_variable(self):
        self.ODOM_POSE_COVARIANCE = [0.25, 0, 0, 0, 0, 0,
                                0, 0.25, 0, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 0.1]

        self.ODOM_TWIST_COVARIANCE = [1e-2, 0, 0, 0, 0, 0,
                                0, 1e-2, 0, 0, 0, 0,
                                0, 0, 1e6, 0, 0, 0,
                                0, 0, 0, 1e6, 0, 0,
                                0, 0, 0, 0, 1e6, 0,
                                0, 0, 0, 0, 0, 0.1]
        # fmt: on

        self.PULLING_FREQ = 10.0

        self.LEFT = 0
        self.RIGHT = 1
        self.LINEAR = 0
        self.ANGULAR = 1

        self.WHEEL_RADIUS = 0.0625
        self.WHEEL_SEPARATION = 0.427
        self.WHEEL_NUM = 2

        self.ENCODER_RESOLUTION = 16384
        self.TICK_PER_ROUND = float(self.ENCODER_RESOLUTION)
        self.RAD_PER_TICK = (2 * pi) / self.TICK_PER_ROUND
        self.TICK_PER_RAD = self.TICK_PER_ROUND / (2 * pi)

        self.LASER_BASE_X = 0.312
        self.LASER_BASE_Y = 0.0
        self.LASER_HEAD_Z = 0.061
        self.LASER_BASE_W = 0.0
        self.joint_states_pos=[0.0, 0.0]
        self.joint_states_vel=[0.0, 0.0]
        self.odom_pose = [0.0, 0.0, 0.0]
        self.odom_vel = [0.0, 0.0, 0.0]
        self.last_diff_tick = [0, 0]
        self.last_velocity = [0.0, 0.0]
        self.last_rad = [0.0, 0.0]
        self.last_velocity = [0.0, 0.0]
        self.joint_states = JointState()
        self.motor_encoder_msg = EncoderDifferential()
        self.odom_msg = Odometry()
        self.laser_pose = Pose()
        self.wheel_l = 0.0
        self.wheel_r = 0.0
        self.last_tick=[0, 0]
        # self.use_absolute_odom = False
        # self.fake_map_odom_tf = False
        # self.use_encoder_odom = True
        self.abs_odom_received = False
        # self.vel_filter_threshold = 1
        self.arduino_last_time = 0
        self.br = tf.TransformBroadcaster()
        self.last_theta = 0.0 
        self.rad = 0.0 
        self.theta = 0.0 
        self.vel_x_zero_cnt = 0.0 
        self.vel_w_zero_cnt = 0.0 
        self.load_param()
        self.use_absolute_odom = rospy.get_param("~use_absolute_odom", False)
        self.use_encoder_odom = rospy.get_param("~use_encoder_odom", True)
        self.fake_map_odom_tf = rospy.get_param("~fake_map_odom_tf", False)
        self.vel_filter_threshold = rospy.get_param("~vel_filter_threshold", 1)

        rospy.loginfo("use_absolute_odom: {}".format(self.use_absolute_odom))
        rospy.loginfo("use_encoder_odom: {}".format(self.use_encoder_odom))
        rospy.loginfo("fake_map_odom_tf: {}".format(self.fake_map_odom_tf))
        

    def load_param(self):
        

        self.WHEEL_RADIUS = rospy.get_param("~wheel_radius", 0.075)
        self.WHEEL_SEPARATION = rospy.get_param("~wheel_separation", 0.523)

        self.ENCODER_RESOLUTION = rospy.get_param("~encoder_resolution", 1024)
        self.TICK_PER_ROUND = float(self.ENCODER_RESOLUTION)
        self.RAD_PER_TICK = (2 * pi) / self.TICK_PER_ROUND
        self.TICK_PER_RAD = self.TICK_PER_ROUND / (2 * pi)
        rospy.loginfo(
            "Odom param: {}, {}, {}".format(
                self.WHEEL_RADIUS, self.WHEEL_SEPARATION, self.ENCODER_RESOLUTION
            )
        )
    def init_node(self):
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.reset_encoder_pub = rospy.Publisher("/reset_encoder", Empty, queue_size=10)
        self.motor_vel_pub = rospy.Publisher("/motor_vel", String, queue_size=10)

        # rospy.Subscriber("/motor_encoder", EncoderDifferential, self.motor_encoder_cb)
        # rospy.Subscriber("/reset_odom", Empty, self.reset_odom_cb)

        if self.use_absolute_odom:
            rospy.Subscriber("/absolute_odom", Odometry, self.absolute_odom_cb)

            self.reset_encoder_pub.publish(Empty())
    def deg_to_rad(self,deg):
        return deg * pi / 180.0


    def rad_to_deg(self,rad):
        return rad * 180.0 / pi


    def tick_to_rad(self,tick):
        return tick * self.RAD_PER_TICK


    def initJointStates(self):
        global joint_states

        self.joint_states_name = ["wheel_left_joint", "wheel_right_joint"]

        # joint_states.header.frame_id = joint_state_header_frame_id
        # joint_states.name            = joint_states_name

        # joint_states.name_length     = WHEEL_NUM
        # joint_states.position_length = WHEEL_NUM
        # joint_states.velocity_length = WHEEL_NUM
        # joint_states.effort_length   = WHEEL_NUM


    def updateTF():
        pass


    def update_tf(self,br):
        if self.fake_map_odom_tf:
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

    def updateJointStates(self):
        self.joint_states.position = self.last_rad
        self.joint_states.velocity = self.last_velocity


    def updateMotorInfo(self,left_tick, right_tick):

        self.last_diff_tick = [left_tick - self.last_tick[self.LEFT],right_tick - self.last_tick[self.RIGHT]]
        self.last_tick = [left_tick,right_tick]
        self.last_rad[self.LEFT] += self.tick_to_rad(last_diff_tick[self.LEFT])
        self.last_rad[self.RIGHT] += self.tick_to_rad(last_diff_tick[self.RIGHT])

    def calcOdometry(self,diff_time, update_pose=True):
        global last_diff_tick, last_velocity, odom_msg, odom_pose, wheel_l, wheel_r
        step_time = diff_time

        if step_time < 0.01:
            return False

        self.updateMotorInfo(self.motor_encoder_msg.left, self.motor_encoder_msg.right)

        # wheel_l = tick_to_rad(last_diff_tick[LEFT])
        # wheel_r = tick_to_rad(last_diff_tick[RIGHT])
        wheel_l_filter = self.tick_to_rad(last_diff_tick[self.LEFT])
        wheel_r_filter = self.tick_to_rad(last_diff_tick[self.RIGHT])
        if abs(wheel_l_filter) < 2 or abs(wheel_r_filter) < 2 :
            self.wheel_l = wheel_l_filter
            self.wheel_r = wheel_r_filter
        else:
            rospy.logerr("Odometry left out of range")
            return

        

        delta_s = self.WHEEL_RADIUS * (wheel_r + wheel_l) / 2.0
        theta_s = self.WHEEL_RADIUS * (wheel_r - wheel_l) / self.WHEEL_SEPARATION
        self.theta = self.theta + theta_s

        delta_theta = theta_s  # calcOdometry.theta - calcOdometry.last_theta
        self.last_theta = self.theta

        if update_pose:
            self.odom_msg.pose.pose.position.x += delta_s * cos(
                self.rad + (delta_theta / 2.0)
            )
            self.odom_msg.pose.pose.position.y += delta_s * sin(
                self.rad + (delta_theta / 2.0)
            )
        self.rad += delta_theta

        if delta_s != 0.0:
            self.odom_msg.twist.twist.linear.x = delta_s / float(step_time)
            self.vel_x_zero_cnt = 0
        else:
            self.vel_x_zero_cnt += 1
            if self.vel_x_zero_cnt > self.vel_filter_threshold:
                self.odom_msg.twist.twist.linear.x = 0.0
        if delta_theta != 0.0:
            self.odom_msg.twist.twist.angular.z = delta_theta / float(step_time)
            self.vel_w_zero_cnt = 0
        else:
            self.vel_w_zero_cnt += 1
            if self.vel_w_zero_cnt > self.vel_filter_threshold:
                self.odom_msg.twist.twist.angular.z = 0.0

        

        

        last_velocity = [wheel_l / float(step_time),wheel_r / float(step_time)]  # rad/s
        
        # motor_vel_msg = ('rad/s: ' + str(round(last_velocity[LEFT], 2)) + ', ' + str(round(last_velocity[RIGHT], 2)))
        # print(motor_vel_msg)
        # motor_vel_pub.publish(String(motor_vel_msg))

        if update_pose:
            self.odom_msg.pose.pose.position.z = 0.0
            abc = quaternion_from_euler(0.0, 0.0, self.rad)
            self.odom_msg.pose.pose.orientation.x = abc[0]
            self.odom_msg.pose.pose.orientation.y = abc[1]
            self.odom_msg.pose.pose.orientation.z = abc[2]
            self.odom_msg.pose.pose.orientation.w = abc[3]
        # print("orientation : ")
        # print(abc)

        # Covariance
        self.odom_msg.pose.covariance = self.ODOM_POSE_COVARIANCE
        self.odom_msg.twist.covariance = self.ODOM_TWIST_COVARIANCE


    def motor_encoder_cb(self,msg):
        # motor_encoder_msg = msg

        if self.use_absolute_odom and self.abs_odom_received:
            self.odom_pose = offset_pose_xyz(
                self.laser_pose, -self.LASER_BASE_X, -self.LASER_BASE_Y, -self.LASER_BASE_W
            )
            self.odom_msg.pose.pose.position.x = self.odom_pose.position.x
            self.odom_msg.pose.pose.position.y = self.odom_pose.position.y
            self.odom_msg.pose.pose.orientation = self.odom_pose.orientation

        if (self.use_absolute_odom and self.abs_odom_received) or not self.use_absolute_odom:
            if self.use_encoder_odom:
                arduino_current_time = self.motor_encoder_msg.header.stamp.to_sec()
                arduino_diff_time = arduino_current_time - self.arduino_last_time
                self.calcOdometry(arduino_diff_time, not self.use_absolute_odom)
                arduino_last_time = arduino_current_time
            self.update_tf(self.br)
            self.odom_msg.header.stamp = rospy.Time.now()
            self.odom_msg.header.frame_id = "odom"
            self.odom_msg.child_frame_id = "base_footprint"
            


    def reset_odom_cb(self,msg):
        # global odom_msg
        self.odom_msg.pose.pose.position.x = 0.0
        self.odom_msg.pose.pose.position.y = 0.0
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0
        self.last_theta = 0.0
        self.rad = 0.0
        self.theta = 0.0
        self.vel_x_zero_cnt = 0.0
        self.vel_w_zero_cnt = 0.0


    def absolute_odom_cb(self,msg):
        if not self.abs_odom_received:
            rospy.loginfo("slamware/odom receive")
        self.abs_odom_received = True

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        ox = msg.pose.pose.orientation.x
        oy = msg.pose.pose.orientation.y
        oz = msg.pose.pose.orientation.z
        ow = msg.pose.pose.orientation.w
        z = euler_from_quaternion([ox, oy, oz, ow])

        self.laser_pose = copy.deepcopy(msg.pose.pose)


    def loop(self):
        while not rospy.is_shutdown():
            rospy.Subscriber("/motor_encoder", EncoderDifferential, self.motor_encoder_cb)
            self.odom_pub.publish(self.odom_msg)
            if self.use_absolute_odom:
                self.reset_encoder_pub.publish(Empty())
    
            listener = tf.TransformListener()
            arduino_last_time = rospy.get_time()
            rate = rospy.Rate(self.PULLING_FREQ)
            self.rate.sleep()


"""


    #    #   ##   # #    #
    ##  ##  #  #  # ##   #
    # ## # #    # # # #  #
    #    # ###### # #  # #
    #    # #    # # #   ##
    #    # #    # # #    #


"""


def parse_opts():
    from optparse import OptionParser

    parser = OptionParser()
    parser.add_option(
        "-d",
        "--ros_debug",
        action="store_true",
        dest="log_debug",
        default=False,
        help="log_level=rospy.DEBUG",
    )
    parser.add_option(
        "-c",
        "--config_file",
        dest="config_file",
        default=os.path.join(
            rospkg.RosPack().get_path("odometry_calculate"),
            "cfg",
            "config.yaml",
        ),
    )

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)

def main(self):
    (options, args) = parse_opts()
    rospy.init_node("odometry")
    rospy.loginfo("Init node odometry")

    Odometry_hooker(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
