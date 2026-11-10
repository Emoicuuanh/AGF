#!/usr/bin/env python3
"""ROS1 node: clearance field between the ground line and the estimated lower-stack line.

Subscribes
  scan                 sensor_msgs/LaserScan   vertical-plane LiDAR
  lower_stack_height   std_msgs/Float32        estimated top of lower stack above floor [m]
  fork_height          std_msgs/Float32        (optional) fork-top height above floor [m]
  enable               std_msgs/Bool           (optional) arm/disarm when ~require_enable

Publishes
  ~field_clear         std_msgs/Bool           debounced result; False on any fault (fail-safe)
  ~intrusion_count     std_msgs/Int32          returns inside the field (last scan)
  ~status              std_msgs/String         JSON with bounds, hits, reason, required fork height
  ~markers             visualization_msgs/MarkerArray

Services
  ~check               std_srvs/Trigger        evaluate the latest scan now (no debounce)
"""

import json
import math
import threading

import numpy as np
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, ColorRGBA, Float32, Int32, String
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray

from agf_clearance_field import (Debouncer, FieldParams, evaluate, plane_to_scan_manual,
                                 quat_to_matrix, required_fork_height, scan_to_plane_manual,
                                 scan_to_plane_tf)


def _rgba(r, g, b, a=1.0):
    return ColorRGBA(r=r, g=g, b=b, a=a)


class ClearanceFieldNode(object):
    def __init__(self):
        gp = rospy.get_param
        self.mode = gp('~mode', 'manual')                         # 'manual' | 'tf'
        # manual mounting model
        self.lidar_height = float(gp('~lidar_height', 1.05))
        self.angle_forward = math.radians(float(gp('~angle_forward_deg', 0.0)))
        self.direction = 1 if int(gp('~angle_direction', 1)) >= 0 else -1
        # tf mounting model
        self.reference_frame = gp('~reference_frame', 'base_footprint')
        self.floor_z = float(gp('~floor_z', 0.0))
        self.tf_timeout = float(gp('~tf_timeout', 0.05))
        # field
        f = gp('~field', {})
        self.p = FieldParams(x_min=float(f.get('x_min', 0.0)), x_max=float(f.get('x_max', 1.4)),
                             floor_margin=float(f.get('floor_margin', 0.02)),
                             tolerance=float(f.get('tolerance', 0.05)),
                             min_points=int(f.get('min_points', 3)))
        # timing / safety
        self.scan_timeout = float(gp('~scan_timeout', 0.3))
        self.height_timeout = float(gp('~height_timeout', 0.0))   # 0 = estimate never goes stale
        self.require_enable = bool(gp('~require_enable', False))
        self.deck_offset = float(gp('~deck_offset', 0.122))
        self.publish_markers = bool(gp('~publish_markers', True))
        self.deb = Debouncer(gp('~trigger_scans', 2), gp('~clear_scans', 3))

        d = float(gp('~default_stack_height', -1.0))
        self.stack_height = d if d > 0 else None
        self.stack_stamp = rospy.Time.now() if d > 0 else None
        self.fork_height = None
        self.enabled = not self.require_enable
        self.last_scan = None
        self.last_scan_rx = None
        self.lock = threading.Lock()

        self.tf_buf = None
        if self.mode == 'tf':
            import tf2_ros
            self.tf_buf = tf2_ros.Buffer(rospy.Duration(5.0))
            self._tf_listener = tf2_ros.TransformListener(self.tf_buf)
        elif self.mode != 'manual':
            rospy.logfatal('~mode must be "manual" or "tf", got %s', self.mode)
            raise SystemExit(1)

        self.pub_clear = rospy.Publisher('~field_clear', Bool, queue_size=1)
        self.pub_count = rospy.Publisher('~intrusion_count', Int32, queue_size=1)
        self.pub_status = rospy.Publisher('~status', String, queue_size=1)
        self.pub_markers = rospy.Publisher('~markers', MarkerArray, queue_size=1)

        rospy.Subscriber('scan', LaserScan, self.on_scan, queue_size=1)
        rospy.Subscriber('lower_stack_height', Float32, self.on_height, queue_size=1)
        rospy.Subscriber('fork_height', Float32, self.on_fork, queue_size=1)
        rospy.Subscriber('enable', Bool, self.on_enable, queue_size=1)
        rospy.Service('~check', Trigger, self.on_check)
        rospy.Timer(rospy.Duration(0.1), self.on_watchdog)

        rospy.loginfo('clearance_field: mode=%s field x=[%.2f, %.2f] tol=%.3f margin=%.3f min_pts=%d',
                      self.mode, self.p.x_min, self.p.x_max, self.p.tolerance,
                      self.p.floor_margin, self.p.min_points)

    # ------------------------------------------------------------ inputs
    def on_height(self, msg):
        with self.lock:
            self.stack_height = float(msg.data) if msg.data > 0 else None
            self.stack_stamp = rospy.Time.now()
            self.deb.reset()                       # new job -> start blocked again

    def on_fork(self, msg):
        with self.lock:
            self.fork_height = float(msg.data)

    def on_enable(self, msg):
        with self.lock:
            if msg.data and not self.enabled:
                self.deb.reset()
            self.enabled = bool(msg.data)

    # ------------------------------------------------------------ core
    def _plane_points(self, scan):
        if self.mode == 'manual':
            x, z = scan_to_plane_manual(scan.ranges, scan.angle_min, scan.angle_increment,
                                        scan.range_min, scan.range_max,
                                        self.angle_forward, self.direction, self.lidar_height)
            return x, z, None
        tfm = self.tf_buf.lookup_transform(self.reference_frame, scan.header.frame_id,
                                           scan.header.stamp, rospy.Duration(self.tf_timeout))
        q, t = tfm.transform.rotation, tfm.transform.translation
        rot = quat_to_matrix(q.x, q.y, q.z, q.w)
        trans = (t.x, t.y, t.z)
        x, z = scan_to_plane_tf(scan.ranges, scan.angle_min, scan.angle_increment,
                                scan.range_min, scan.range_max, rot, trans, self.floor_z)
        return x, z, trans

    def _height_stale(self):
        if self.stack_height is None:
            return True
        if self.height_timeout <= 0:
            return False
        return (rospy.Time.now() - self.stack_stamp).to_sec() > self.height_timeout

    def _evaluate(self, scan):
        try:
            x, z, trans = self._plane_points(scan)
        except Exception as e:  # tf lookup failures etc.
            return None, None, None, None, 'transform_error: %s' % e
        h = None if self._height_stale() else self.stack_height
        res = evaluate(x, z, h, self.p)
        return res, x, z, trans, None

    def on_scan(self, scan):
        with self.lock:
            self.last_scan, self.last_scan_rx = scan, rospy.Time.now()
            if not self.enabled:
                self._publish(False, 0, {'reason': 'disabled'}, scan, None, None, None, None)
                return
            res, x, z, trans, err = self._evaluate(scan)
            if err:
                self.deb.reset()
                self._publish(False, 0, {'reason': err}, scan, None, None, None, None)
                return
            if not res.valid:
                self.deb.reset()
                self._publish(False, 0, {'reason': res.reason}, scan, res, x, z, trans)
                return
            state = self.deb.update(res.clear)
            self._publish(state, res.count, self._status(res, state), scan, res, x, z, trans)

    def _status(self, res, state):
        s = {
            'reason': res.reason if state == res.clear else ('debouncing_' + res.reason),
            'clear': state, 'clear_raw': res.clear, 'count': res.count,
            'field_x': [self.p.x_min, self.p.x_max],
            'field_z': [round(res.z_bottom, 4), round(res.z_top, 4)],
            'stack_height_est': self.stack_height,
            'lowest_hit_z': res.lowest_hit_z, 'nearest_hit_x': res.nearest_hit_x,
            'required_fork_height': round(required_fork_height(res.z_top, self.deck_offset), 4),
        }
        if self.fork_height is not None:
            s['fork_height'] = self.fork_height
            s['carried_bottom'] = round(self.fork_height - self.deck_offset, 4)
            s['margin_to_field_top'] = round(self.fork_height - self.deck_offset - res.z_top, 4)
        return s

    def _publish(self, clear, count, status, scan, res, x, z, trans):
        self.pub_clear.publish(Bool(data=bool(clear)))
        self.pub_count.publish(Int32(data=int(count)))
        status.setdefault('clear', bool(clear))
        self.pub_status.publish(String(data=json.dumps(status)))
        if self.publish_markers and res is not None and res.valid and scan is not None:
            self.pub_markers.publish(self._markers(scan, res, x, z, trans, clear))

    # ------------------------------------------------------------ watchdog / service
    def on_watchdog(self, _evt):
        with self.lock:
            stale = (self.last_scan_rx is None or
                     (rospy.Time.now() - self.last_scan_rx).to_sec() > self.scan_timeout)
            if stale:
                self.deb.reset()
                self.pub_clear.publish(Bool(data=False))
                self.pub_status.publish(String(data=json.dumps({'clear': False, 'reason': 'scan_timeout'})))

    def on_check(self, _req):
        with self.lock:
            if self.last_scan is None:
                return TriggerResponse(success=False, message='no scan received')
            res, _, _, _, err = self._evaluate(self.last_scan)
            if err or not res.valid:
                return TriggerResponse(success=False, message=err or res.reason)
            msg = json.dumps({'clear': res.clear, 'count': res.count,
                              'field_z': [res.z_bottom, res.z_top],
                              'lowest_hit_z': res.lowest_hit_z, 'nearest_hit_x': res.nearest_hit_x})
            return TriggerResponse(success=bool(res.clear), message=msg)

    # ------------------------------------------------------------ RViz
    def _to_frame(self, xs, zs, trans):
        """Plane coords -> (frame_id, list of Point)."""
        xs, zs = np.asarray(xs, float), np.asarray(zs, float)
        if self.mode == 'manual':
            px, py = plane_to_scan_manual(xs, zs, self.angle_forward, self.direction, self.lidar_height)
            return None, [Point(x=a, y=b, z=0.0) for a, b in zip(px, py)]
        return self.reference_frame, [Point(x=trans[0] + a, y=trans[1], z=self.floor_z + b)
                                      for a, b in zip(xs, zs)]

    def _markers(self, scan, res, x, z, trans, clear):
        arr = MarkerArray()
        frame, rect = self._to_frame([self.p.x_min, self.p.x_max, self.p.x_max, self.p.x_min, self.p.x_min],
                                     [res.z_bottom, res.z_bottom, res.z_top, res.z_top, res.z_bottom], trans)
        frame = frame or scan.header.frame_id
        col = _rgba(0.2, 0.75, 0.3) if clear else _rgba(0.9, 0.2, 0.2)

        m = Marker(type=Marker.LINE_STRIP, action=Marker.ADD, ns='field', id=0)
        m.header.frame_id, m.header.stamp = frame, scan.header.stamp
        m.pose.orientation.w = 1.0
        m.scale.x = 0.015
        m.color, m.points = col, rect
        arr.markers.append(m)

        hs = res.z_top - self.p.tolerance
        _, est = self._to_frame([self.p.x_min, self.p.x_max], [hs, hs], trans)
        e = Marker(type=Marker.LINE_LIST, action=Marker.ADD, ns='stack_estimate', id=1)
        e.header = m.header
        e.pose.orientation.w = 1.0
        e.scale.x = 0.008
        e.color, e.points = _rgba(0.6, 0.6, 0.6, 0.8), est
        arr.markers.append(e)

        inside = (x >= self.p.x_min) & (x <= self.p.x_max) & (z > res.z_bottom) & (z <= res.z_top)
        _, hits = self._to_frame(x[inside], z[inside], trans)
        h = Marker(type=Marker.POINTS, action=Marker.ADD, ns='intrusions', id=2)
        h.header = m.header
        h.pose.orientation.w = 1.0
        h.scale.x = h.scale.y = 0.03
        h.color, h.points = _rgba(1.0, 0.55, 0.0), hits
        arr.markers.append(h)

        t = Marker(type=Marker.TEXT_VIEW_FACING, action=Marker.ADD, ns='label', id=3)
        t.header = m.header
        t.pose.orientation.w = 1.0
        t.pose.position = rect[2]
        t.scale.z = 0.08
        t.color = col
        t.text = '%s  %d pts  top %.3f m' % ('CLEAR' if clear else 'BLOCKED', res.count, res.z_top)
        arr.markers.append(t)
        return arr


def main():
    rospy.init_node('clearance_field')
    ClearanceFieldNode()
    rospy.spin()


if __name__ == '__main__':
    main()
