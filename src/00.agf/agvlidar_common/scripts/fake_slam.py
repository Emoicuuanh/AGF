#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
from std_stamped_msgs.msg import Int8Stamped, StringStamped
import json
from sensor_msgs.msg import Joy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, sin, cos, atan


class FakeSlam:
    def __init__(self):
        self.init_ros()
        self.poll()

    def init_ros(self):
        rospy.init_node("fake_slam")
        rospy.loginfo("Init node: " + rospy.get_name())
        self.slam_status_pub = rospy.Publisher(
            "/slam_manager/module_status", StringStamped, queue_size=10
        )
        self.current_map_pub = rospy.Publisher(
            "/current_map", StringStamped, queue_size=5
        )
        # Params
        self.map_file = rospy.get_param("~map_file", "")

    def poll(self):
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            self.current_map_pub.publish(
                StringStamped(stamp=rospy.Time.now(), data=self.map_file)
            )
            stt_dict = {
                "status": "",
                "state": "LOCALIZING",
                "current_map": "...{}".format(self.map_file),
                "error_code": "",
            }
            self.slam_status_pub.publish(
                StringStamped(stamp=rospy.Time.now(), data=json.dumps(stt_dict))
            )

            rate.sleep()


def main():
    FakeSlam()


if __name__ == "__main__":
    main()
