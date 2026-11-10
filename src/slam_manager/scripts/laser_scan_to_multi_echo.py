#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import MultiEchoLaserScan, LaserScan, LaserEcho

def scan_cb(msg):
    muls = MultiEchoLaserScan()
    echo_ranges = LaserEcho()
    echo_intensities = LaserEcho()
    echo_ranges.echoes = msg.ranges
    echo_intensities.echoes = msg.intensities
    muls.ranges.append(echo_ranges)
    muls.intensities.append(echo_intensities)
    echoes_pub.publish(muls)

echoes_pub = rospy.Publisher("/echoes", MultiEchoLaserScan, queue_size=5)

def main():
    rospy.init_node('laser_scan_to_multi_echo', disable_signals=True)
    rospy.loginfo('Init node ' + rospy.get_name())

    rospy.Subscriber("/scan", LaserScan, scan_cb)
    rospy.spin()

if __name__ == '__main__':
    main()