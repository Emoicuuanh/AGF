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

class FakeStatus():
    def __init__(self):
        self.init_ros()
        self.poll()

    def init_ros(self):
        rospy.init_node('fake_status')
        rospy.loginfo('Init node: ' + rospy.get_name())
        self.robot_status_pub = rospy.Publisher('/robot_status', StringStamped, queue_size=10)
        self.robot_mode_pub = rospy.Publisher('/robot_mode', StringStamped, queue_size=10)
        self.battery_pub = rospy.Publisher("/battery", Int8Stamped, queue_size=5)
        self.current_map_pub = rospy.Publisher("/current_map", StringStamped, queue_size=5)
        #
        self.map_file = rospy.get_param("~map_file", "")

    def poll(self):
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            self.current_map_pub.publish(StringStamped(stamp=rospy.Time.now(), data=self.map_file))
            self.battery_pub.publish(Int8Stamped(stamp=rospy.Time.now(), data=90))
            self.robot_mode_pub.publish(StringStamped(stamp=rospy.Time.now(), data="MANUAL"))
            stt_dict = {
                        "status": "WAITING",
                        "sound": "OFF",
                        "detail": "ALL_OK",
                        "state": "MANUAL",
                        "mode": "MANUAL",
                        "error_code": ""
                        }
            self.robot_status_pub.publish(StringStamped(stamp=rospy.Time.now(), data=json.dumps(stt_dict)))

            rate.sleep()

def main():
    FakeStatus()

if __name__ == '__main__':
    main()