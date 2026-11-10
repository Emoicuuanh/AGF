#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Compare /odom_imu with /nav350laser/odom (both nav_footprint in odom).

Both tracks are taken relative to their pose at the start (or at ~reset), so
an offset in the starting pose does not count. Errors are given in the NAV350
pose's own axes:
  along  error along the NAV350 heading   -> speed scale (vel_scale)
  side   error across the NAV350 heading  -> heading error x distance
  yaw    heading difference                -> gyro bias / scale
plus the yaw rate ratio while turning (gyro scale).
"""

import threading
from math import sin, cos, atan2, hypot, degrees

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty


def pose2d(msg):
    p, q = msg.pose.pose.position, msg.pose.pose.orientation
    return p.x, p.y, atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return atan2(sin(a), cos(a))


def relative(origin, pose):
    """pose expressed in origin's axes"""
    ox, oy, oyaw = origin
    dx, dy = pose[0] - ox, pose[1] - oy
    return (dx * cos(oyaw) + dy * sin(oyaw), -dx * sin(oyaw) + dy * cos(oyaw), wrap(pose[2] - oyaw))


class CompareOdom:
    def __init__(self):
        rospy.init_node("compare_odom")
        self.ref_topic = rospy.get_param("~ref_topic", "/nav350laser/odom")
        self.test_topic = rospy.get_param("~test_topic", "/odom_imu")
        self.rate = rospy.get_param("~print_rate", 1.0)
        self.lock = threading.Lock()
        self.ref = self.test = None
        self.ref0 = self.test0 = None
        self.dist = 0.0
        self.last_ref = None
        self.turn_ref = self.turn_test = 0.0
        self.last_test = None
        rospy.Subscriber(self.ref_topic, Odometry, self.ref_cb, queue_size=10)
        rospy.Subscriber(self.test_topic, Odometry, self.test_cb, queue_size=10)
        rospy.Subscriber("~reset", Empty, self.reset_cb, queue_size=1)
        rospy.Subscriber("/odom_imu/reset", Empty, self.reset_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.rate), self.print_cb)
        rospy.loginfo("compare_odom: %s vs %s", self.test_topic, self.ref_topic)

    def reset_cb(self, _msg):
        with self.lock:
            self.ref0 = self.test0 = None

    def ref_cb(self, msg):
        p = pose2d(msg)
        with self.lock:
            if self.last_ref is not None and self.ref0 is not None:
                self.dist += hypot(p[0] - self.last_ref[0], p[1] - self.last_ref[1])
                self.turn_ref += abs(wrap(p[2] - self.last_ref[2]))
            self.last_ref = self.ref = p

    def test_cb(self, msg):
        p = pose2d(msg)
        with self.lock:
            if self.last_test is not None and self.test0 is not None:
                self.turn_test += abs(wrap(p[2] - self.last_test[2]))
            self.last_test = self.test = p

    def print_cb(self, _event):
        with self.lock:
            if self.ref is None or self.test is None:
                rospy.logwarn_throttle(5.0, "compare_odom: waiting for %s and %s", self.ref_topic, self.test_topic)
                return
            if self.ref0 is None or self.test0 is None:
                self.ref0, self.test0 = self.ref, self.test
                self.dist = self.turn_ref = self.turn_test = 0.0
                rospy.loginfo("compare_odom: start")
                return
            r = relative(self.ref0, self.ref)
            t = relative(self.test0, self.test)
            dist, turn_ref, turn_test = self.dist, self.turn_ref, self.turn_test

        # test position error, in the current reference heading
        ex, ey = t[0] - r[0], t[1] - r[1]
        along = ex * cos(r[2]) + ey * sin(r[2])
        side = -ex * sin(r[2]) + ey * cos(r[2])
        eyaw = wrap(t[2] - r[2])
        travelled_ref = hypot(r[0], r[1])
        travelled_test = hypot(t[0], t[1])
        scale = travelled_ref / travelled_test if travelled_test > 0.2 else float("nan")
        yaw_scale = turn_ref / turn_test if turn_test > 0.2 else float("nan")
        rospy.loginfo(
            "compare_odom: dist %.2fm | along %+.3fm side %+.3fm yaw %+.2fdeg | "
            "vel_scale x%.3f (straight-line) yaw rate x%.3f (turned %.0fdeg)",
            dist, along, side, degrees(eyaw), scale, yaw_scale, degrees(turn_ref))


def main():
    CompareOdom()
    rospy.spin()


if __name__ == "__main__":
    main()
