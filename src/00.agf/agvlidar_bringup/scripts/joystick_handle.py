#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
import rospkg
from std_stamped_msgs.msg import Int8Stamped, StringStamped, EmptyStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi, sin, cos, atan

class JoystickHandle():
    def __init__(self):
        self.init_ros()
        self.init_varialble()
        self.poll()

    def init_ros(self):
        rospy.init_node('joystick_handle')
        rospy.loginfo('Init node: ' + rospy.get_name())
        self.liftup_pub = rospy.Publisher('/lift_cart', Int8Stamped, queue_size=10)
        self.run_pause_pub = rospy.Publisher("/request_run_stop", StringStamped, queue_size=5)
        self.misstion_control_pub = rospy.Publisher("/request_start_mission", StringStamped, queue_size=5)
        self.reset_error_pub = rospy.Publisher("/reset_error", EmptyStamped, queue_size=5)
        rospy.Subscriber('/joy', Joy, self.joy_cb)

    def init_varialble(self):
        self.button_lift_up = 3     # Y
        self.button_lift_down = 1   # A
        self.button_lift_mid = 2    # B
        self.button_reset = 0       # X
        self.button_run = 9         # START
        self.button_pause = 8       # BACK

        self.liftup_interval = 0.2  # Second
        self.last_liftup = rospy.get_time()
        self.begin_push_pause_button = rospy.get_time()
        self.begin_push_run_button = rospy.get_time()
        self.already_start = False
        self.already_stop = False
        self.last_pause_button = 0
        self.last_run_button = 0
        self.last_reset_button = 0

    def joy_cb(self, joy):
        now = rospy.get_time()
        msg = Int8Stamped()
        str_msg = StringStamped()
        if now - self.last_liftup >= self.liftup_interval:
            msg.stamp = rospy.Time.now()
            str_msg.stamp = rospy.Time.now()
            # Liftup
            if joy.buttons[self.button_lift_up] == 1:
                msg.data = 1
                self.liftup_pub.publish(msg)
            elif joy.buttons[self.button_lift_down] == 1:
                msg.data = 2
                self.liftup_pub.publish(msg)
            elif joy.buttons[self.button_lift_mid] == 1:
                msg.data = 3
                self.liftup_pub.publish(msg)
            # Run, pause, stop
            if joy.buttons[self.button_pause] == 1 and self.last_pause_button == 0:
                self.begin_push_pause_button = rospy.get_time()
            if joy.buttons[self.button_run] == 1 and self.last_run_button == 0:
                self.begin_push_run_button = rospy.get_time()
            if joy.buttons[self.button_run] == 1:
                str_msg.data = "RUN"
                if self.last_run_button == 0:
                    self.run_pause_pub.publish(str_msg)
                # Start mission queue
                if rospy.get_time() - self.begin_push_run_button > 2.0 and not self.already_start:
                    self.already_start = True
                    str_msg.data = "START"
                    self.misstion_control_pub.publish(str_msg)
            elif joy.buttons[self.button_pause] == 1:
                str_msg.data = "STOP"
                if self.last_pause_button == 0:
                    self.run_pause_pub.publish(str_msg)
                # Stop mission
                if rospy.get_time() - self.begin_push_pause_button > 2.0 and not self.already_stop:
                    self.already_stop = True
                    str_msg.data = "STOP"
                    self.misstion_control_pub.publish(str_msg)
            # Reset
            if joy.buttons[self.button_reset] == 1 and self.last_reset_button == 0:
                self.reset_error_pub.publish(EmptyStamped(stamp=rospy.Time.now()))

            self.last_liftup = now
            self.last_pause_button = joy.buttons[self.button_pause]
            self.last_run_button = joy.buttons[self.button_run]
            self.last_reset_button = joy.buttons[self.button_reset]
            if self.last_run_button == 0:
                self.already_start = False
            if self.last_pause_button == 0:
                self.already_stop = False

    def poll(self):
        rospy.spin()
        # rate = rospy.Rate(10.0)
        # while not rospy.is_shutdown():
        #     rate.sleep()

def main():
    JoystickHandle()

if __name__ == '__main__':
    main()