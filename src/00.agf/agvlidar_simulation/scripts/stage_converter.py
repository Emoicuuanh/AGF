#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage

class StageConverter():
    def __init__(self):
        self.init_ros()
        self.init_varialble()
        self.poll()

    def init_ros(self):
        rospy.init_node('stage_converter')
        rospy.loginfo('Init node: ' + rospy.get_name())
        rospy.Subscriber('/scan_in', LaserScan, self.scan_in_cb)
        rospy.Subscriber('/odom_in', Odometry, self.odom_in_cb)
        rospy.Subscriber('/tf_in', TFMessage, self.tf_in_cb)

        self.scan_out_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
        self.odom_out_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.tf_out_pub = rospy.Publisher('/tf', TFMessage, queue_size=10)

    def init_varialble(self):
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.0)

    def scan_in_cb(self, msg):
        msg.header.frame_id = 'laser'
        self.scan_out_pub.publish(msg)

    def odom_in_cb(self, msg):
        msg.child_frame_id = 'base_footprint'
        self.odom_out_pub.publish(msg)

    def tf_in_cb(self, msg):
        for i in msg.transforms:
            if i.header.frame_id == 'base_link' and i.child_frame_id == 'base_laser_link':
                i.child_frame_id = 'laser'
            if i.header.frame_id == 'base_footprint' and i.child_frame_id == 'base_link':
                i.transform.translation.z += self.wheel_radius
        self.tf_out_pub.publish(msg)

    def poll(self):
        rospy.spin()

def main():
    stage_converter = StageConverter()

if __name__ == '__main__':
    main()