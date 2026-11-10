#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import os
import sys
import rospy
import rospkg
import copy
import json
import yaml
from math import sin, cos, pi
from geometry_msgs.msg import Twist, Quaternion, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf
from std_stamped_msgs.msg import (
    StringStamped,
    StringAction,
    StringFeedback,
    StringResult,
    StringGoal,
    EmptyStamped,
    Int16Stamped,
)

from tf.transformations import euler_from_quaternion, quaternion_from_euler


def yaw_to_quaternion(rad):
    quan = quaternion_from_euler(0, 0, rad)
    ret = Quaternion()
    ret.x = quan[0]
    ret.y = quan[1]
    ret.z = quan[2]
    ret.w = quan[3]
    return ret


def get_yaw(pose):
    rad = euler_from_quaternion(
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )[2]
    return rad


def offset_pose_y(pose, y):
    rad = get_yaw(pose)
    ret = Pose()
    ret.position.x = pose.position.x - y * sin(rad)
    ret.position.y = pose.position.y + y * cos(rad)
    ret.position.z = pose.position.z
    ret.orientation = pose.orientation
    return ret


def normalize_angle(angle):
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle


class OdomCalc(object):
    def __init__(self, name, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        # Action
        # Param
        # 1.42 -0.248
        self.steer_wheel_x = rospy.get_param("~steer_wheel_x", 1.42)
        self.steer_wheel_y = rospy.get_param("~steer_wheel_y", -0.248)
        self.calc_fr_encoder = rospy.get_param("~calc_fr_encoder", True)
        self.calc_fr_speed = rospy.get_param("~calc_fr_speed", False)
        self.pulse_per_m = rospy.get_param("~pulse_per_m", 1305)
        self.pulse_0_deg = rospy.get_param("~pulse_0_deg", 32)
        self.pulse_neg90_deg = rospy.get_param("~pulse_neg90_deg", 0)
        self.pulse_pos90_deg = rospy.get_param("~pulse_pos90_deg", -3676)
        self.use_odom_fr_origin = rospy.get_param("~use_odom_fr_origin", True)
        # TODO: Config direction
        self.pulse_per_deg = (self.pulse_0_deg - self.pulse_pos90_deg) / 90
        self.pulse_per_rad = (self.pulse_0_deg - self.pulse_pos90_deg) / (
            pi / 2
        )
        # Publisher
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.odom_virtual_pub = rospy.Publisher(
            "/odom_virtual", Odometry, queue_size=10
        )
        self.steering_angle_pub = rospy.Publisher(
            "/steering_angle", Float64, queue_size=10
        )
        # Subscriber
        if self.calc_fr_speed:
            rospy.Subscriber("/current_vel", Twist, self.current_vel_cb)
        if self.calc_fr_encoder:
            rospy.Subscriber(
                "/encoder_traction", Int16Stamped, self.en_traction_cb
            )
            rospy.Subscriber(
                "/encoder_steering", Int16Stamped, self.en_steering_cb
            )
        rospy.Subscriber("/odom_fr_nav", Odometry, self.odom_nav_cb)
        if self.use_odom_fr_origin:
            self.setup_odom_fr_org()
        # Loop
        self.loop()

    def init_variable(self, *args, **kwargs):
        # self.config_file = kwargs["config_file"]
        # rospy.loginfo("config_file: %s" % self.config_file)
        # Read YAML file
        self.br = tf.TransformBroadcaster()
        self.x_odom = 2  # ??? Có phải cho stage simulation không
        self.y_odom = 2
        self.yaw_odom = 1.57
        self.last_vel_time = rospy.get_time()
        self.first_init = True
        self.steering_angle = None
        self.last_steering_time = rospy.get_time()
        self.last_traction_time = rospy.get_time()
        self.last_traction_en = None

    def shutdown(self):
        rospy.loginfo("Shuting down")

    def setup_odom_fr_org(self):
        self.first_odom_x = None
        self.first_odom_y = None
        self.first_odom_yaw = None
        self.first_odo_af_rot_x = None
        self.first_odo_af_rot_y = None
        self.internal_odom = Odometry()
        self.odom_fr_org_pub = rospy.Publisher(
            "/odom_from_origin", Odometry, queue_size=1
        )
        self.odom_sub = rospy.Subscriber(
            "/control_odom_fr_org",
            StringStamped,
            self.ctrl_odom_fr_org_cb,
            queue_size=1,
        )
        # Nếu subscribe odom do chính node này publish ra để tính toán thì quỹ
        # đạo hơi khác so với dùng biến nội
        # self.odom_sub = rospy.Subscriber(
        #     "/odom", Odometry, self.odom_callback, queue_size=1
        # )

        # For calc from raw encoder
        self.last_traction_time_ = rospy.get_time()
        self.last_traction_en_ = None
        self.x_odom_ = 0
        # TODO: Check again
        self.y_odom_ = self.steer_wheel_y
        self.yaw_odom_ = 0

    def calc_odom_from_org(self, msg):
        if self.steering_angle == None:
            return
        encoder_traction = msg.data
        now = rospy.get_time()
        dt = now - self.last_traction_time_
        self.last_traction_time_ = now
        traction_vel = 0

        if self.last_traction_en_ == None:
            self.last_traction_en_ = encoder_traction
            return
        diff_encoder_raw = encoder_traction - self.last_traction_en_
        if abs(diff_encoder_raw) > 100:  # 2^16/2 = 32768
            # Ignore 1 cycle
            # FIXME:
            self.last_traction_en_ = encoder_traction
            rospy.logwarn("Encoder over: {}".format(diff_encoder_raw))
            return
        diff_encoder = diff_encoder_raw
        self.last_traction_en_ = encoder_traction

        if dt > 0:
            traction_vel = (diff_encoder / self.pulse_per_m) / dt  # m/s
        else:
            return

        d_yaw_odom = (
            traction_vel * sin(self.steering_angle) / self.steer_wheel_x
        )
        delta_linear = traction_vel * cos(self.steering_angle)
        d_x_odom = delta_linear * cos(self.yaw_odom_)
        d_y_odom = delta_linear * sin(self.yaw_odom_)

        self.x_odom_ += d_x_odom * dt
        self.y_odom_ += d_y_odom * dt
        self.yaw_odom_ += d_yaw_odom * dt

        if self.yaw_odom_ > pi:
            self.yaw_odom_ -= 2 * pi
        elif self.yaw_odom_ < -pi:
            self.yaw_odom_ += 2 * pi

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = self.x_odom_
        odom_msg.pose.pose.position.y = self.y_odom_
        odom_msg.pose.pose.orientation = yaw_to_quaternion(self.yaw_odom_)
        odom_msg.twist.twist.linear.x = d_x_odom
        odom_msg.twist.twist.angular.z = d_yaw_odom

        # Convert từ vị trí bánh lái ở cạnh ra bánh lái ở giữa
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose = offset_pose_y(
            odom_msg.pose.pose, -self.steer_wheel_y
        )
        self.odom_fr_org_pub.publish(odom_msg)

    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def ctrl_odom_fr_org_cb(self, msg):
        rospy.logwarn("Reset odom from origin")
        self.first_odo_af_rot_x = None  # For reset
        self.first_odom_x = None  # For reset first odom
        # For calc from raw encoder
        self.last_traction_en_ = None
        self.x_odom_ = 0
        # TODO: Check again
        self.y_odom_ = self.steer_wheel_y
        self.yaw_odom_ = 0
        if msg.data == "START":
            pass
        elif msg.data == "STOP":
            pass

    def odom_callback(self, msg):
        """
        Function for rotate and offset odom
        """
        # FIXME: 09Sep thử lại start ở góc bất kỳ vẫn sai
        odom_x = msg.pose.pose.position.x
        odom_y = msg.pose.pose.position.y
        odom_yaw = get_yaw(msg.pose.pose)
        if self.first_odom_x == None:
            self.first_odom_x = odom_x
            self.first_odom_y = odom_y
            self.first_odom_yaw = odom_yaw

        odom_rotated_yaw = odom_yaw - self.first_odom_yaw
        odom_rotated_yaw = normalize_angle(odom_rotated_yaw)
        # Công thức của Mr.Định
        # odom_rotated_x = sqrt(
        #     (odom_x - self.first_odom_x) ** 2
        #     + (odom_y - self.first_odom_y) ** 2
        # ) * cos(odom_rotated_yaw)
        # odom_rotated_y = sqrt(
        #     (odom_x - self.first_odom_x) ** 2
        #     + (odom_y - self.first_odom_y) ** 2
        # ) * sin(odom_rotated_yaw)

        # Xoay trục tọa độ (đã test OK)
        odom_rotated_x = odom_x * cos(self.first_odom_yaw) - odom_y * sin(
            self.first_odom_yaw
        )
        odom_rotated_y = odom_x * sin(self.first_odom_yaw) + odom_y * cos(
            self.first_odom_yaw
        )

        # Tịnh tiến về (0, 0)
        if self.first_odo_af_rot_x == None:
            self.first_odo_af_rot_x = odom_rotated_x
            self.first_odo_af_rot_y = odom_rotated_y
        odom_rotated_x -= self.first_odo_af_rot_x
        odom_rotated_y -= self.first_odo_af_rot_y

        self.internal_odom.pose.pose.position.x = odom_rotated_x
        self.internal_odom.pose.pose.position.y = odom_rotated_y
        self.internal_odom.pose.pose.orientation = yaw_to_quaternion(
            odom_rotated_yaw
        )
        self.internal_odom.header.stamp = rospy.Time.now()
        self.internal_odom.header.frame_id = "odom"
        self.odom_fr_org_pub.publish(self.internal_odom)

        # rospy.loginfo(
        #     "x_odom : {}, y_odom: {}, theta: {}".format(
        #         round(odom_rotated_x, 2),
        #         round(odom_rotated_y, 2),
        #         round(odom_rotated_yaw, 2),
        #     )
        # )

    def en_traction_cb(self, msg):
        self.calc_odom_from_org(msg)
        if self.steering_angle == None or self.first_init:
            return
        encoder_traction = msg.data
        now = rospy.get_time()
        dt = now - self.last_traction_time
        self.last_traction_time = now
        traction_vel = 0

        if self.last_traction_en == None:
            self.last_traction_en = encoder_traction
            return
        diff_encoder_raw = encoder_traction - self.last_traction_en
        if abs(diff_encoder_raw) > 100:  # 2^16/2 = 32768
            # Ignore 1 cycle
            # FIXME:
            self.last_traction_en = encoder_traction
            rospy.logwarn("Encoder over: {}".format(diff_encoder_raw))
            return
        diff_encoder = diff_encoder_raw
        self.last_traction_en = encoder_traction

        if dt > 0:
            traction_vel = (diff_encoder / self.pulse_per_m) / dt  # m/s
        else:
            return

        d_yaw_odom = (
            traction_vel * sin(self.steering_angle) / self.steer_wheel_x
        )
        delta_linear = traction_vel * cos(self.steering_angle)
        d_x_odom = delta_linear * cos(self.yaw_odom)
        d_y_odom = delta_linear * sin(self.yaw_odom)

        self.x_odom += d_x_odom * dt
        self.y_odom += d_y_odom * dt
        self.yaw_odom += d_yaw_odom * dt

        if self.yaw_odom > pi:
            self.yaw_odom -= 2 * pi
        elif self.yaw_odom < -pi:
            self.yaw_odom += 2 * pi

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = self.x_odom
        odom_msg.pose.pose.position.y = self.y_odom
        odom_msg.pose.pose.orientation = yaw_to_quaternion(self.yaw_odom)
        odom_msg.twist.twist.linear.x = d_x_odom
        odom_msg.twist.twist.angular.z = d_yaw_odom
        self.odom_virtual_pub.publish(odom_msg)

        # Convert từ vị trí bánh lái ở cạnh ra bánh lái ở giữa
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose = offset_pose_y(
            odom_msg.pose.pose, -self.steer_wheel_y
        )
        self.odom_pub.publish(odom_msg)
        # Test hệ tọa độ cho pallet
        # if self.use_odom_fr_origin:
        #     self.odom_callback(odom_msg)

    def en_steering_cb(self, msg):
        self.last_steering_time = rospy.get_time()
        # TODO: Config direction
        self.steering_angle = -msg.data / self.pulse_per_rad
        self.steering_angle_pub.publish(Float64(data=self.steering_angle))
        # quat = yaw_to_quaternion(self.steering_angle)
        # self.br.sendTransform(
        #     (
        #         self.steer_wheel_x,
        #         self.steer_wheel_y,
        #         0.0,
        #     ),
        #     (
        #         quat.x,
        #         quat.y,
        #         quat.z,
        #         quat.w,
        #     ),
        #     rospy.Time.now(),
        #     "steering_wheel",
        #     "base_link",
        # )

    def odom_nav_cb(self, msg):
        if self.first_init:
            self.first_init = False
            rospy.loginfo("Received NAV odom")
        else:
            return
        msg.pose.pose = offset_pose_y(msg.pose.pose, self.steer_wheel_y)
        self.x_odom = msg.pose.pose.position.x
        self.y_odom = msg.pose.pose.position.y
        self.yaw_odom = get_yaw(msg.pose.pose)

    def current_vel_cb(self, msg):
        traction_vel = msg.linear.x
        steering_angle = msg.angular.z
        now = rospy.get_time()
        dt = now - self.last_vel_time
        self.last_vel_time = now

        d_yaw_odom = traction_vel * sin(steering_angle) / self.steer_wheel_x
        delta_linear = traction_vel * cos(steering_angle)
        d_x_odom = delta_linear * cos(self.yaw_odom)
        d_y_odom = delta_linear * sin(self.yaw_odom)

        self.x_odom += d_x_odom * dt
        self.y_odom += d_y_odom * dt
        self.yaw_odom += d_yaw_odom * dt

        if self.yaw_odom > pi:
            self.yaw_odom -= 2 * pi
        elif self.yaw_odom < -pi:
            self.yaw_odom += 2 * pi

        odom_msg = Odometry()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = self.x_odom
        odom_msg.pose.pose.position.y = self.y_odom
        odom_msg.pose.pose.orientation = yaw_to_quaternion(self.yaw_odom)
        self.odom_virtual_pub.publish(odom_msg)

        # Convert từ vị trí bánh lái ở cạnh ra bánh lái ở giữa
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose = offset_pose_y(
            odom_msg.pose.pose, -self.steer_wheel_y
        )
        self.odom_pub.publish(odom_msg)

    """
    ##        #######   #######  ########
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ########
    ##       ##     ## ##     ## ##
    ##       ##     ## ##     ## ##
    ########  #######   #######  ##
    """

    def loop(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            rate.sleep()


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
    # parser.add_option(
    #     "-c",
    #     "--config_file",
    #     dest="config_file",
    #     default=os.path.join(
    #         rospkg.RosPack().get_path("odometry_calculate"),
    #         "cfg",
    #         "config.yaml",
    #     ),
    # )

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)


def main():
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    rospy.init_node("odom_steering_wheel", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    OdomCalc(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
