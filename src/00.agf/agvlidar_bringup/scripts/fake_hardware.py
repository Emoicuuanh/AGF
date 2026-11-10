#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
import json
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
)


class FakeHardware:
    def __init__(self):
        self.init_ros()
        self.init_varialble()
        self.poll()

    def init_ros(self):
        rospy.init_node("fake_hardware")
        rospy.loginfo("Init node: " + rospy.get_name())
        self.std_io_pub = rospy.Publisher(
            "/standard_io", StringStamped, queue_size=10
        )

    def init_varialble(self):
        pass

    def poll(self):
        rate = rospy.Rate(10.0)
        std_io = {
            "emg_button": 1,
            "auto_manual_sw": 1,
            "bumper": 1,
            "motor_enable_sw": 1,
        }
        std_io_std = json.dumps(std_io)
        std_io_msg = StringStamped(data=std_io_std)
        while not rospy.is_shutdown():
            std_io_msg.stamp = rospy.Time.now()
            self.std_io_pub.publish(std_io_msg)
            rate.sleep()


def main():
    fake_hardware = FakeHardware()


if __name__ == "__main__":
    main()
