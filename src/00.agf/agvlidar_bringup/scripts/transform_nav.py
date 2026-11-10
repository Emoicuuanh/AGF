#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
from std_msgs.msg import Int64, Int16, Int8, String, Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, sin, cos, atan
from std_stamped_msgs.msg import StringStamped

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

from common_function import (
    print_debug,
    print_warn,
    print_error,
    print_info,
    offset_pose_to_pose,
    yaw_to_quaternion,
    EnumString,
)


class NAVStatus(EnumString):
    ALL_OK = 0
    LOCALIZATION_ERROR = 1
    SENSOR_DISCONNECTED = 2
    OTHER_ERROR = 3


class TransformNAV:
    def __init__(self):
        self.init_ros()
        self.init_varialble()
        self.poll()

    def init_ros(self):
        rospy.init_node("transform_nav")
        rospy.loginfo("Init node: " + rospy.get_name())
        # Param
        self.odom_link = rospy.get_param("~odom_link", "odom")
        self.nav_x = rospy.get_param("~nav_x", -1.14)
        self.nav_y = rospy.get_param("~nav_y", 0.0)
        self.nav_z = rospy.get_param("~nav_z", -2.2795)
        self.nav_yaw = rospy.get_param("~nav_yaw", pi)
        # Subscriber
        rospy.Subscriber("/nav350laser/odom", Odometry, self.odom_cb)
        # Publisher
        self.odom_pub = rospy.Publisher("/odom_fr_nav", Odometry, queue_size=10)
        self.module_stt_pub = rospy.Publisher(
            "~module_status", StringStamped, queue_size=10
        )

    def init_varialble(self):
        self.br = tf.TransformBroadcaster()
        self.odom_msg = Odometry()
        self.stt_msg = StringStamped()
        self.base_link_to_nav = Pose()
        self.base_link_to_nav.position.x = self.nav_x
        self.base_link_to_nav.position.y = self.nav_y
        self.base_link_to_nav.position.z = self.nav_z
        self.base_link_to_nav.orientation = yaw_to_quaternion(self.nav_yaw)
        self.last_odom_received = rospy.get_time()

    def odom_cb(self, msg):
        self.last_odom_received = rospy.get_time()
        # 1.14 0 2.2795
        # TODO: Check if pose.pose.position.x = y = -26628.797
        if (
            abs(msg.pose.pose.position.x) > 20000
            or abs(msg.pose.pose.position.y) > 20000
        ):
            self.stt_msg.data = NAVStatus.LOCALIZATION_ERROR.toString()
            rospy.logerr_throttle(30, "NAV350 cannot locate position")
            return
        base_link_pose = offset_pose_to_pose(
            msg.pose.pose, self.base_link_to_nav, type=3
        )
        self.odom_msg.header.stamp = rospy.Time.now()
        self.odom_msg.header.frame_id = self.odom_link
        self.odom_msg.child_frame_id = ""
        self.odom_msg.pose.pose = base_link_pose
        self.odom_pub.publish(self.odom_msg)
        # base_link_quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
        self.br.sendTransform(
            (
                base_link_pose.position.x,
                base_link_pose.position.y,
                base_link_pose.position.z,
            ),
            (
                base_link_pose.orientation.x,
                base_link_pose.orientation.y,
                base_link_pose.orientation.z,
                base_link_pose.orientation.w,
            ),
            rospy.Time.now(),
            "base_footprint",
            "odom",
        )
        self.stt_msg.data = NAVStatus.ALL_OK.toString()

    def update_tf(self):
        pass

    def poll(self):
        rate = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            if rospy.get_time() - self.last_odom_received > 1:
                self.stt_msg.data = NAVStatus.SENSOR_DISCONNECTED.toString()
            self.stt_msg.stamp = rospy.Time.now()
            self.module_stt_pub.publish(self.stt_msg)
            rate.sleep()


def main():
    transform_nav = TransformNAV()


if __name__ == "__main__":
    main()
