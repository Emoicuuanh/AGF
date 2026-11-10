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


class SavePose:
    def __init__(self):
        self.init_varialble()
        self.init_ros()
        self.poll()

    def init_varialble(self):
        self.file_path = last_pose_path
        self.first_init = True
        self.first_msg = False
        self.last_pose = None

    def init_ros(self):
        rospy.init_node("save_pose")
        rospy.loginfo("Init node save_pose")

        rospy.Subscriber(
            "/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_cb
        )
        self.initialpose_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=10
        )

    def amcl_pose_cb(self, msg):
        if not self.first_init:
            if (
                msg.pose.pose.position.x != None
                and msg.pose.pose.position.y != None
                and msg.pose.pose.position.z != None
                and msg.pose.pose.orientation.x != None
                and msg.pose.pose.orientation.y != None
                and msg.pose.pose.orientation.z != None
                and msg.pose.pose.orientation.w != None
            ):
                print(msg)
                with open(self.file_path, "w") as file:
                    documents = yaml.dump(msg, file)
        else:
            try:
                with open(self.file_path) as file:
                    self.last_pose = yaml.load(file, Loader=yaml.FullLoader)
                    self.first_msg = True
                    print(self.last_pose)
            except Exception as e:
                rospy.logwarn(e)
        self.first_init = False

    def poll(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if self.first_msg:
                self.first_msg = False
                if True:
                    self.initialpose_pub.publish(self.last_pose)
                    rospy.loginfo("Recover last pose")
                else:
                    rospy.loginfo("Last pose error")
            rate.sleep()
        # rospy.spin()


def main():
    save_pose = SavePose()


if __name__ == "__main__":
    main()
