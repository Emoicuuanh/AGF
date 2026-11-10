#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Check how far the robot stopped from the target pose when moving_control
reports SUCCEEDED.

Passive: only listens to the moving_control action topics and TF.
  <action>/goal   (std_stamped_msgs/StringActionGoal)    json, target = waypoints[-1]
  <action>/result (std_stamped_msgs/StringActionResult)  SUCCEEDED -> measure

After settle_time the robot pose (global_frame -> robot_frame) is compared to
the target. Errors are given in the map frame and in the target frame:
  along   : + robot stopped past the target, - stopped before it
  lateral : + robot is left of the target heading, - right of it

Publishes ~offset (geometry_msgs/Pose2D: x=along, y=lateral, theta=yaw error)
and appends a row to csv_file if set.
"""

import os
import json
import threading
from math import atan2, cos, sin, hypot, degrees, pi

import rospy
import tf
from tf.transformations import euler_from_quaternion
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose2D
from std_stamped_msgs.msg import StringActionGoal, StringActionResult


def norm_angle(a):
    return atan2(sin(a), cos(a))


def yaw_from_quat(q):
    return euler_from_quaternion([q["x"], q["y"], q["z"], q["w"]])[2]


class GoalOffsetChecker(object):
    def __init__(self):
        self.action_name = rospy.get_param("~action_name", "/moving_control").rstrip("/")
        # fallback when the goal json has no params.frame_id (moving_control default is map)
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.robot_frame = rospy.get_param("~robot_frame", "base_footprint")
        # wait for the robot to fully stop before reading the pose
        self.settle_time = rospy.get_param("~settle_time", 1.0)
        # used when the goal json has no reconfigure xy_tolerance / yaw_tolerance
        self.xy_tol = rospy.get_param("~xy_tolerance", 0.05)
        self.yaw_tol = rospy.get_param("~yaw_tolerance", 0.05)
        self.csv_file = os.path.expanduser(rospy.get_param("~csv_file", ""))

        self.goals = {}  # goal_id -> target dict
        self.lock = threading.Lock()
        self.tf_listener = tf.TransformListener()
        self.offset_pub = rospy.Publisher("~offset", Pose2D, queue_size=5)

        rospy.Subscriber(self.action_name + "/goal", StringActionGoal, self.goal_cb)
        rospy.Subscriber(self.action_name + "/result", StringActionResult, self.result_cb)
        rospy.loginfo("check_goal_offset: watching %s, robot frame %s",
                      self.action_name, self.robot_frame)

    def goal_cb(self, msg):
        try:
            data = json.loads(msg.goal.data)
            wp = data["waypoints"][-1]
            pose = wp["position"]
            params = data.get("params", {})
            wp_params = wp.get("params", {})
            reconf = dict(params.get("reconfigure", {}))
            reconf.update(wp_params.get("reconfigure", {}))
            target = {
                "name": wp.get("name", ""),
                "frame": params.get("frame_id", self.global_frame) or self.global_frame,
                "x": pose["position"]["x"],
                "y": pose["position"]["y"],
                "yaw": yaw_from_quat(pose["orientation"]),
                "xy_tol": reconf.get("xy_tolerance", self.xy_tol),
                "yaw_tol": reconf.get("yaw_tolerance", self.yaw_tol),
                "stamp": rospy.get_time(),
            }
        except Exception as e:
            rospy.logwarn("check_goal_offset: cannot parse goal: %s", e)
            return
        with self.lock:
            self.goals[msg.goal_id.id] = target
            # keep only recent goals
            if len(self.goals) > 20:
                oldest = min(self.goals, key=lambda k: self.goals[k]["stamp"])
                self.goals.pop(oldest)
        rospy.loginfo("check_goal_offset: new target %s (%.3f, %.3f, %.1f deg) in %s",
                      target["name"], target["x"], target["y"],
                      degrees(target["yaw"]), target["frame"])

    def result_cb(self, msg):
        with self.lock:
            target = self.goals.pop(msg.status.goal_id.id, None)
        if msg.status.status != GoalStatus.SUCCEEDED:
            return
        if target is None:
            rospy.logwarn("check_goal_offset: SUCCEEDED for an unknown goal (started before this node?)")
            return
        rospy.Timer(rospy.Duration(self.settle_time),
                    lambda _: self.measure(target), oneshot=True)

    def measure(self, target):
        try:
            self.tf_listener.waitForTransform(target["frame"], self.robot_frame,
                                              rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.tf_listener.lookupTransform(target["frame"], self.robot_frame,
                                                            rospy.Time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logerr("check_goal_offset: TF %s -> %s failed: %s",
                         target["frame"], self.robot_frame, e)
            return
        rx, ry = trans[0], trans[1]
        ryaw = euler_from_quaternion(rot)[2]

        dx = rx - target["x"]
        dy = ry - target["y"]
        dist = hypot(dx, dy)
        c, s = cos(target["yaw"]), sin(target["yaw"])
        along = c * dx + s * dy
        lateral = -s * dx + c * dy
        dyaw = norm_angle(ryaw - target["yaw"])
        # robot may stop facing the opposite way when the pose is reached backward
        dyaw_flip = norm_angle(dyaw + pi)
        if abs(dyaw_flip) < abs(dyaw):
            rospy.logwarn("check_goal_offset: robot heading is ~180 deg from target")

        ok = dist <= target["xy_tol"] and abs(dyaw) <= target["yaw_tol"]
        log = rospy.loginfo if ok else rospy.logwarn
        log("check_goal_offset: [%s] %s target (%.3f, %.3f, %.2f deg) robot (%.3f, %.3f, %.2f deg)\n"
            "  dx %+.3f  dy %+.3f  dist %.3f m (tol %.3f)\n"
            "  along %+.3f  lateral %+.3f m  yaw %+.2f deg (tol %.2f)",
            "OK" if ok else "OUT OF TOLERANCE", target["name"],
            target["x"], target["y"], degrees(target["yaw"]),
            rx, ry, degrees(ryaw),
            dx, dy, dist, target["xy_tol"],
            along, lateral, degrees(dyaw), degrees(target["yaw_tol"]))

        self.offset_pub.publish(Pose2D(x=along, y=lateral, theta=dyaw))
        if self.csv_file:
            self.write_csv(target, rx, ry, ryaw, dx, dy, dist, along, lateral, dyaw, ok)

    def write_csv(self, target, rx, ry, ryaw, dx, dy, dist, along, lateral, dyaw, ok):
        new_file = not os.path.isfile(self.csv_file)
        try:
            with open(self.csv_file, "a") as f:
                if new_file:
                    f.write("time,name,frame,target_x,target_y,target_yaw,robot_x,robot_y,robot_yaw,"
                            "dx,dy,dist,along,lateral,dyaw_deg,ok\n")
                f.write("%.3f,%s,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%d\n" % (
                    rospy.get_time(), target["name"], target["frame"],
                    target["x"], target["y"], target["yaw"], rx, ry, ryaw,
                    dx, dy, dist, along, lateral, degrees(dyaw), int(ok)))
        except IOError as e:
            rospy.logerr("check_goal_offset: cannot write %s: %s", self.csv_file, e)


if __name__ == "__main__":
    rospy.init_node("check_goal_offset")
    GoalOffsetChecker()
    rospy.spin()
