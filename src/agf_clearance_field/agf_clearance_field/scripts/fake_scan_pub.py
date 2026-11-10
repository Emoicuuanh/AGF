#!/usr/bin/env python3
"""Bench-test publisher: synthetic vertical-plane LaserScan of the AGF stacking scene.

Ray-casts, in the scan plane, the floor, the lower stack (pallet + load), the carried
pallet + load, the fork and the carriage plate -- the same 2D slice as the web simulator.
All scene parameters are read every cycle, so they can be changed live with rosparam:

  rosparam set /fake_scan/fork_height 1.30
  rosparam set /fake_scan/stack_height_actual 1.26
  rosparam set /fake_scan/distance 1.6

It also publishes the *estimated* stack height on lower_stack_height and the fork height
on fork_height so the clearance node can run stand-alone.
"""

import math

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

PAL_H, PAL_L, FORK_L, FORK_T, DECK = 0.144, 1.2, 1.15, 0.045, 0.122


def boxes(p):
    h, d = p['fork_height'], p['distance']
    zb = h - DECK
    return [
        (d, d + PAL_L, 0.0, p['stack_height_actual']),             # lower stack
        (0.03, 0.03 + PAL_L, zb, zb + PAL_H + p['carried_load']),  # carried pallet + load
        (-0.01, FORK_L, h - FORK_T, h),                            # fork
        (-0.07, -0.01, h - 0.2, h + 0.9),                          # carriage plate
    ]


def ray_box(ox, oz, vx, vz, b):
    t0, t1 = -math.inf, math.inf
    for o, v, lo, hi in ((ox, vx, b[0], b[1]), (oz, vz, b[2], b[3])):
        if abs(v) < 1e-12:
            if o < lo or o > hi:
                return None
            continue
        a, c = (lo - o) / v, (hi - o) / v
        if a > c:
            a, c = c, a
        t0, t1 = max(t0, a), min(t1, c)
        if t0 > t1:
            return None
    if t1 < 0:
        return None
    return max(t0, 0.0)


def main():
    rospy.init_node('fake_scan')
    gp = rospy.get_param
    pub = rospy.Publisher('scan', LaserScan, queue_size=1)
    pub_h = rospy.Publisher('lower_stack_height', Float32, queue_size=1)
    pub_f = rospy.Publisher('fork_height', Float32, queue_size=1)
    frame = gp('~frame_id', 'lidar_vertical')
    amin, amax = math.radians(-135.0), math.radians(135.0)
    inc = math.radians(float(gp('~resolution_deg', 0.5)))
    n = int(round((amax - amin) / inc)) + 1
    noise = float(gp('~noise_std', 0.005))
    rate = rospy.Rate(float(gp('~rate', 15.0)))
    rng = np.random.default_rng()

    while not rospy.is_shutdown():
        p = {k: float(gp('~' + k, v)) for k, v in (
            ('lidar_height', 1.05), ('fork_height', 1.35), ('distance', 1.6),
            ('stack_height_actual', 1.144), ('stack_height_estimate', 1.144),
            ('carried_load', 0.9))}
        oz = p['lidar_height']
        bx = boxes(p)
        ranges = []
        for i in range(n):
            a = amin + i * inc                     # angle_forward = 0, +angle = up
            vx, vz = math.cos(a), math.sin(a)
            best = math.inf
            for b in bx:
                t = ray_box(0.0, oz, vx, vz, b)
                if t is not None and t < best:
                    best = t
            if vz < -1e-9:
                best = min(best, -oz / vz)
            ranges.append(best + rng.normal(0.0, noise) if best <= 10.0 else math.inf)
        s = LaserScan()
        s.header.stamp, s.header.frame_id = rospy.Time.now(), frame
        s.angle_min, s.angle_max, s.angle_increment = amin, amin + (n - 1) * inc, inc
        s.range_min, s.range_max = 0.05, 10.0
        s.scan_time = 1.0 / 15.0
        s.ranges = ranges
        pub.publish(s)
        pub_h.publish(Float32(data=p['stack_height_estimate']))
        pub_f.publish(Float32(data=p['fork_height']))
        rate.sleep()


if __name__ == '__main__':
    main()
