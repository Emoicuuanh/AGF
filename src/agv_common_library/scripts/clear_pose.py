#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
from std_msgs.msg import Int64, Int16, Int8, String, Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, sin, cos, atan
import yaml

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

from common_function import *

last_pose_path = ""
input_argv = rospy.myargv(argv=sys.argv)
print("clear_pose param: {}".format(input_argv))
if len(input_argv) >= 2:
    if input_argv[1] != "0":
        last_pose_path = input_argv[1]


class ClearPose:
    def __init__(self):
        self.init_varialble()
        self.clear_pose()

    def init_varialble(self):
        self.file_path = last_pose_path

    def clear_pose(self):
        with open(self.file_path, "w") as file:
            pose = PoseWithCovarianceStamped()
            pose.header.frame_id = "map"
            pose.pose.pose.orientation.w = 1.0
            documents = yaml.dump(pose, file)
            print("Last pose cleaned")


def main():
    save_pose = ClearPose()


if __name__ == "__main__":
    main()
