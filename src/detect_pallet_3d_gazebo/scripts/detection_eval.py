#!/usr/bin/env python3
"""Scores the Livox pallet detections against the pallet's true pose in Gazebo.

Everything is compared in base_link, as detect_object consumes the detections (x, y, yaw;
z is ignored there). The pallet model origin is the front-face center on the floor with x
out of the face, the same convention as the detector frames.

  front  livox_center_detector -> pallet_front_frame. The center must lie ON the front face
         (perp = distance from the face) with the face's yaw. Along the face it is the middle
         of the visible / y-cropped part, so that offset ("along") is only reported.
  edge   livox_edge_detector -> pallet_edge_frame. The origin must be one of the two vertical
         front corners (y = +-pallet_width / 2) with the face's yaw.

Prints mean / std / max of each error, PASS or FAIL, and (result_file) appends one JSON line
per run for run_tilt_matrix.sh. Exit code 0 = PASS.
"""
import json
import math
import sys
import threading

import numpy as np
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import SetBool
from tf.transformations import quaternion_matrix

from detect_pallet_3d.srv import EnableCenterDetector

CHILD_FRAMES = {"front": "pallet_front_frame", "edge": "pallet_edge_frame"}


def pose_matrix(p):
    T = quaternion_matrix([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


def tf_matrix(t):
    r, tr = t.transform.rotation, t.transform.translation
    T = quaternion_matrix([r.x, r.y, r.z, r.w])
    T[:3, 3] = [tr.x, tr.y, tr.z]
    return T


def yaw_of(T):
    return math.atan2(T[1, 0], T[0, 0])  # = tf2::getYaw, as detect_object uses


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class DetectionEval:
    def __init__(self):
        self.detectors = [d.strip() for d in rospy.get_param("~detectors", "front").split(",") if d.strip()]
        for d in self.detectors:
            if d not in CHILD_FRAMES:
                raise ValueError(f"unknown detector '{d}', use front and / or edge")
        self.base_frame = rospy.get_param("~base_frame", "base_link")
        self.rig_model = rospy.get_param("~rig_model", "livox_rig")
        self.pallet_model = rospy.get_param("~pallet_model", "pallet")
        self.pallet_width = rospy.get_param("~pallet_width", 1.0)
        self.warmup = rospy.get_param("~warmup", 3.0)          # s after enable, not scored
        self.duration = rospy.get_param("~duration", 20.0)     # s scored
        self.max_age = rospy.get_param("~max_age", 0.5)        # s, older detection = lost
        self.min_samples = rospy.get_param("~min_samples", 20)
        self.tol_pos = rospy.get_param("~tol_pos", 0.02)       # m
        self.tol_yaw = math.radians(rospy.get_param("~tol_yaw_deg", 1.0))
        self.use_y_filter = rospy.get_param("~use_y_filter", True)
        self.pallet_size = rospy.get_param("~pallet_size", 1.0)
        self.result_file = rospy.get_param("~result_file", "")
        self.tag = {"livox_roll": rospy.get_param("~livox_roll", 0.0),
                    "livox_pitch": rospy.get_param("~livox_pitch", 0.0),
                    "detector_roll": rospy.get_param("~detector_roll", 0.0),
                    "detector_pitch": rospy.get_param("~detector_pitch", 0.0)}

        self.lock = threading.Lock()
        self.T_base_pallet = None
        self.samples = {d: [] for d in self.detectors}
        self.last_stamp = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.states_cb, queue_size=1)

    def states_cb(self, msg):
        try:
            i, j = msg.name.index(self.rig_model), msg.name.index(self.pallet_model)
        except ValueError:
            return
        T = np.linalg.inv(pose_matrix(msg.pose[i])) @ pose_matrix(msg.pose[j])
        with self.lock:
            self.T_base_pallet = T

    def enable(self, on):
        if "front" in self.detectors:
            rospy.wait_for_service("/livox_center_detector/enable")
            rospy.ServiceProxy("/livox_center_detector/enable", EnableCenterDetector)(
                on, self.use_y_filter, self.pallet_size)
        if "edge" in self.detectors:
            rospy.wait_for_service("/livox_edge_detector/enable")
            rospy.ServiceProxy("/livox_edge_detector/enable", SetBool)(on)

    def sample(self, det):
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, CHILD_FRAMES[det], rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        if (rospy.Time.now() - t.header.stamp).to_sec() > self.max_age or \
                t.header.stamp == self.last_stamp.get(det):
            return  # lost / same detection as last sample
        self.last_stamp[det] = t.header.stamp
        with self.lock:
            T_true = self.T_base_pallet
        if T_true is None:
            return

        T_det = tf_matrix(t)
        p = (np.linalg.inv(T_true) @ T_det)[:3, 3]  # detection in the true pallet frame
        d_yaw = wrap(yaw_of(T_det) - yaw_of(T_true))
        if det == "front":
            self.samples[det].append({"perp": p[0], "along": p[1], "yaw": d_yaw})
        else:
            corner_y = math.copysign(self.pallet_width / 2.0, p[1])
            self.samples[det].append({"perp": p[0], "along": p[1] - corner_y, "yaw": d_yaw})

    def summarize(self, det):
        s = self.samples[det]
        res = {"n": len(s)}
        for k in ("perp", "along", "yaw"):
            v = np.array([x[k] for x in s]) if s else np.zeros(0)
            if k == "yaw":
                v = np.degrees(v)
            res[k] = {"mean": float(v.mean()) if len(v) else None,
                      "std": float(v.std()) if len(v) else None,
                      "max": float(np.abs(v).max()) if len(v) else None}
        ok = res["n"] >= self.min_samples
        if ok:
            # front: "along" = middle of the visible part, not a pallet feature -> not scored
            scored = ["perp", "along"] if det == "edge" else ["perp"]
            for k in scored:
                ok &= abs(res[k]["mean"]) <= self.tol_pos and res[k]["std"] <= self.tol_pos
            ok &= abs(res["yaw"]["mean"]) <= math.degrees(self.tol_yaw) and \
                res["yaw"]["std"] <= math.degrees(self.tol_yaw)
        res["pass"] = bool(ok)
        return res

    def run(self):
        rospy.loginfo("[eval] waiting for /gazebo/model_states with %s and %s", self.rig_model, self.pallet_model)
        while not rospy.is_shutdown():
            with self.lock:
                if self.T_base_pallet is not None:
                    break
            rospy.sleep(0.2)
        self.enable(True)
        rospy.loginfo("[eval] %s enabled, scoring for %.0f s after %.0f s warm-up",
                      ",".join(self.detectors), self.duration, self.warmup)

        t0 = rospy.Time.now()
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - t0).to_sec()
            if elapsed >= self.warmup + self.duration:
                break
            if elapsed >= self.warmup:
                for d in self.detectors:
                    self.sample(d)
            rate.sleep()
        if rospy.is_shutdown():
            return False

        result = dict(self.tag)
        all_ok = True
        for d in self.detectors:
            r = self.summarize(d)
            result[d] = r
            all_ok &= r["pass"]
            if r["n"] == 0:
                rospy.logerr("[eval] %-5s no detection", d)
                continue
            log = rospy.loginfo if r["pass"] else rospy.logerr
            log("[eval] %-5s n=%d  perp=%+.4f+-%.4f m  along=%+.4f+-%.4f m  yaw=%+.2f+-%.2f deg  -> %s",
                d, r["n"], r["perp"]["mean"], r["perp"]["std"], r["along"]["mean"], r["along"]["std"],
                r["yaw"]["mean"], r["yaw"]["std"], "PASS" if r["pass"] else "FAIL")
        if self.result_file:
            with open(self.result_file, "a") as f:
                f.write(json.dumps(result) + "\n")
        return all_ok


if __name__ == "__main__":
    rospy.init_node("detection_eval")
    try:
        ok = DetectionEval().run()
    except rospy.ROSInterruptException:  # Ctrl+C / launch shut down before the end
        sys.exit(1)
    rospy.signal_shutdown("evaluation done")
    sys.exit(0 if ok else 1)
