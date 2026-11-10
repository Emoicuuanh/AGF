#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import rospy
from std_msgs.msg import Int64, Int16, Int8, String, Empty
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalID
from std_stamped_msgs.msg import StringStamped


class RvizButtonConverter:
    def __init__(self):
        self.init_ros()
        self.init_varialble()
        self.pool()

    def init_ros(self):
        rospy.init_node("rviz_button_converter")
        rospy.loginfo("init node rviz_button_converter")
        rospy.Subscriber(
            "/rviz_visual_tools_gui", Joy, self.rviz_visual_tools_gui_cb
        )

        self.path_reset_pub = rospy.Publisher(
            "/path_reset", Empty, queue_size=10
        )
        self.move_base_cancel_pub = rospy.Publisher(
            "/move_base/cancel", GoalID, queue_size=10
        )
        self.move_to_point_cancel_pub = rospy.Publisher(
            "/move_to_point/cancel", GoalID, queue_size=10
        )
        self.request_start_mission_pub = rospy.Publisher(
            "/request_start_mission", StringStamped, queue_size=5
        )
        self.run_stop_control_pub = rospy.Publisher(
            "/mission_manager/run_pause_req", StringStamped, queue_size=5
        )

    def init_varialble(self):
        pass

    def rviz_visual_tools_gui_cb(self, msg):
        if msg.buttons[4] == 1:  # Origin: Stop button
            self.path_reset_pub.publish(Empty())
            self.move_base_cancel_pub.publish(GoalID())
            self.move_to_point_cancel_pub.publish(GoalID())
            stop_msg = StringStamped()
            stop_msg.stamp = rospy.Time.now()
            stop_msg.data = "STOP"
            self.request_start_mission_pub.publish(stop_msg)
        if msg.buttons[3] == 1:  # Origin: Break button
            run_stop_msg = StringStamped()
            run_stop_msg.stamp = rospy.Time.now()
            run_stop_msg.data = "PAUSE"
            self.run_stop_control_pub.publish(run_stop_msg)
        if msg.buttons[2] == 1:  # Origin: Continue button
            run_stop_msg = StringStamped()
            run_stop_msg.stamp = rospy.Time.now()
            run_stop_msg.data = "RUN"
            self.run_stop_control_pub.publish(run_stop_msg)
        if msg.buttons[1] == 1:  # Origin: Next button
            stop_msg = StringStamped()
            stop_msg.stamp = rospy.Time.now()
            stop_msg.data = "START"
            self.request_start_mission_pub.publish(stop_msg)

    def pool(self):
        rospy.spin()


def main():
    rviz_button_converter = RvizButtonConverter()


if __name__ == "__main__":
    main()
