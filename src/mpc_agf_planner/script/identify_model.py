#!/usr/bin/env python3
"""Fit the MPC robot model (wheelbase, steering lag, steering rate, delay) from a rosbag.

Record the three tests from the MPC setup guide into one bag:
  1. steering steps at standstill (+-0.3, +-0.8, +-1.5 rad)
  2. constant-steering circles at 0.3 m/s, forward and reverse
  3. speed and steering steps for the command delay
then run:
  rosrun mpc_agf_planner identify_model.py model_id.bag --track-width 0.9 --out mpc_model.yaml

/steering_angle and the steering command are the LEFT wheel's angle (Ackermann).
The track width W (distance between the steering pivots) is needed to fit the
wheelbase from it.
"""
import argparse
import math
import sys

import numpy as np


def load(bag_paths, cmd_topic, steer_topic, odom_topic):
    import rosbag  # imported here so --help works without ROS

    cmd, steer, odom = [], [], []
    for path in bag_paths:
        with rosbag.Bag(path) as bag:
            for topic, msg, t in bag.read_messages(topics=[cmd_topic, steer_topic, odom_topic]):
                ts = t.to_sec()
                if topic == cmd_topic:
                    tw = msg.twist if hasattr(msg, "twist") else msg
                    cmd.append((ts, tw.linear.x, tw.angular.z))
                elif topic == steer_topic:
                    steer.append((ts, msg.data))
                else:
                    tw = msg.twist.twist
                    odom.append((ts, tw.linear.x, tw.angular.z))
    for name, series in (("command", cmd), ("steering", steer), ("odom", odom)):
        if not series:
            sys.exit("No %s messages found; check the topic names." % name)
    return np.array(cmd), np.array(steer), np.array(odom)


def smooth(x, n=3):
    if len(x) < n:
        return x
    return np.convolve(x, np.ones(n) / n, mode="same")


def steering_steps(cmd, steer):
    """Steering steps at standstill: delay, time constant and rate."""
    delays, taus, rates = [], [], []
    for i in range(1, len(cmd)):
        t0, v, c1 = cmd[i]
        c0 = cmd[i - 1][2]
        if abs(c1 - c0) < 0.1 or abs(v) > 0.01:
            continue
        nxt = [j for j in range(i + 1, len(cmd)) if abs(cmd[j][2] - c1) > 0.05]
        t_end = min(t0 + 3.0, cmd[nxt[0]][0] if nxt else t0 + 3.0)
        w = (steer[:, 0] >= t0 - 0.05) & (steer[:, 0] <= t_end)
        ts, ds = steer[w, 0], steer[w, 1]
        if len(ts) < 10:
            continue
        d0 = ds[0]
        moved = np.where(np.abs(ds - d0) > 0.02)[0]
        if len(moved) == 0:
            continue
        t_start = ts[moved[0]]
        delays.append(t_start - t0)
        step = c1 - d0
        if abs(step) <= 0.5:
            reached = np.where((ds - d0) / step >= 0.632)[0]
            if len(reached):
                taus.append(ts[reached[0]] - t_start)
        if abs(step) >= 0.7:
            rate = np.abs(np.gradient(smooth(ds), ts))
            rates.append(float(np.percentile(rate, 95)))
    return delays, taus, rates


def circles(cmd, steer, odom, settle=1.5, min_len=3.0):
    """Constant-steering segments while driving: wheelbase, sign and zero offset."""
    out = []
    i = 0
    while i < len(cmd):
        t0, v0, c0 = cmd[i]
        j = i
        while j + 1 < len(cmd) and abs(cmd[j + 1][2] - c0) < 0.01 and abs(cmd[j + 1][1] - v0) < 0.02:
            j += 1
        t1 = cmd[j][0]
        if abs(v0) > 0.1 and abs(c0) > 0.1 and t1 - t0 > min_len:
            ws = (steer[:, 0] > t0 + settle) & (steer[:, 0] < t1)
            wo = (odom[:, 0] > t0 + settle) & (odom[:, 0] < t1)
            if ws.sum() > 5 and wo.sum() > 5:
                delta = float(np.mean(steer[ws, 1]))
                v = float(np.mean(odom[wo, 1]))
                w = float(np.mean(odom[wo, 2]))
                if abs(w) > 1e-3 and abs(math.tan(delta)) > 1e-3:
                    out.append({"delta": delta, "v": v, "w": w, "cmd": c0})
        i = j + 1
    return out


def speed_delays(cmd, odom):
    delays = []
    for i in range(1, len(cmd)):
        t0, v1, _ = cmd[i]
        if abs(cmd[i - 1][1]) < 0.01 and abs(v1) >= 0.1:
            w = (odom[:, 0] >= t0) & (odom[:, 0] <= t0 + 2.0)
            moved = np.where(np.abs(odom[w, 1]) > 0.02)[0]
            if len(moved):
                delays.append(odom[w, 0][moved[0]] - t0)
    return delays


def centre_to_left(centre, wheelbase, track_width, sign):
    """Centre angle -> left-wheel angle; must match steering_geometry.h."""
    k = track_width / (2.0 * wheelbase)
    phi = sign * centre
    return sign * math.atan2(math.sin(phi), math.cos(phi) - k * math.sin(phi))


def check(name, value, lo, hi, hint):
    if not (lo <= value <= hi):
        print("  WARNING: %s = %.3f outside the typical range %.2f-%.2f. %s" % (name, value, lo, hi, hint))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bags", nargs="+")
    ap.add_argument("--cmd-topic", default="/cmd_vel")
    ap.add_argument("--steer-topic", default="/steering_angle")
    ap.add_argument("--odom-topic", default="/odom")
    ap.add_argument("--track-width", type=float, default=0.0,
                    help="distance between the two steering pivots (m); 0 = single centre wheel")
    ap.add_argument("--out", default="mpc_model.yaml")
    args = ap.parse_args()

    cmd, steer, odom = load(args.bags, args.cmd_topic, args.steer_topic, args.odom_topic)

    d_steer, taus, rates = steering_steps(cmd, steer)
    d_speed = speed_delays(cmd, odom)
    segs = circles(cmd, steer, odom)

    print("Steering steps: %d (tau from %d, rate from %d)" % (len(d_steer), len(taus), len(rates)))
    print("Speed steps: %d" % len(d_speed))
    print("Circle segments: %d" % len(segs))
    if not segs:
        sys.exit("No constant-steering circles found; run test 2 from the setup guide.")

    # Sign: the turn direction (curvature omega / v) follows the wheel angle
    W = args.track_width
    signs = [1.0 if (s["w"] / s["v"]) * s["delta"] > 0 else -1.0 for s in segs]
    steer_sign = 1.0 if np.median(signs) > 0 else -1.0
    # Wheelbase from the left wheel: tan(phi_left) = L / (R - W/2) with R
    # signed by the turn, so L = tan(phi_left) * (1/kappa - W/2)
    ls = []
    for s in segs:
        kappa = s["w"] / s["v"]
        phi = steer_sign * s["delta"]
        ls.append(abs(math.tan(phi) * (1.0 / kappa - W / 2.0)))
    wheelbase = float(np.median(ls))
    # Zero offset: measured angle minus the left-wheel angle that explains the turn rate
    offsets = [s["delta"] - centre_to_left(steer_sign * math.atan(wheelbase * s["w"] / s["v"]),
                                           wheelbase, W, steer_sign) for s in segs]
    fwd = [l for l, s in zip(ls, segs) if s["v"] > 0]
    rev = [l for l, s in zip(ls, segs) if s["v"] < 0]
    left_turns = [l for l, s in zip(ls, segs) if s["w"] / s["v"] > 0]
    right_turns = [l for l, s in zip(ls, segs) if s["w"] / s["v"] < 0]

    tau = float(np.median(taus)) if taus else 0.4
    rate = float(np.median(rates)) if rates else 1.0
    delays = d_steer + d_speed
    delay = float(np.median(delays)) if delays else 0.1

    print("\nResults")
    print("  wheelbase       %.3f m  (forward %s, reverse %s)" % (
        wheelbase,
        "%.3f" % np.median(fwd) if fwd else "-",
        "%.3f" % np.median(rev) if rev else "-"))
    print("  steer_sign      %+.0f  (%d of %d segments agree)" % (
        steer_sign, sum(1 for s in signs if s == steer_sign), len(signs)))
    print("  steering offset %.4f rad (fix in the drive if above 0.01)" % float(np.median(offsets)))
    print("  tau_steer       %.3f s%s" % (tau, "" if taus else "  (no small steps, default kept)"))
    print("  max_steer_rate  %.3f rad/s%s" % (rate, "" if rates else "  (no large steps, default kept)"))
    print("  delay           %.3f s%s" % (delay, "" if delays else "  (no steps, default kept)"))

    check("tau_steer", tau, 0.1, 1.0, "Check /steering_angle is the measured angle, not the command.")
    check("max_steer_rate", rate, 0.5, 3.0, "Use the 1.5 rad steps.")
    check("delay", delay, 0.02, 0.3, "Check clock sync between drive and planner PCs.")
    if fwd and rev and abs(np.median(fwd) - np.median(rev)) > 0.05 * wheelbase:
        print("  WARNING: forward and reverse wheelbase differ by more than 5%; recheck the reverse circles.")
    if left_turns and right_turns and abs(np.median(left_turns) - np.median(right_turns)) > 0.05 * wheelbase:
        print("  WARNING: left and right turns give different wheelbases (%.3f vs %.3f m); check --track-width."
              % (np.median(left_turns), np.median(right_turns)))
    if len(set(signs)) > 1:
        print("  WARNING: steer_sign differs between segments; recheck the circle test.")

    with open(args.out, "w") as f:
        f.write("# Identified by identify_model.py from: %s\n" % ", ".join(args.bags))
        f.write("wheelbase: %.4f\n" % wheelbase)
        f.write("steer_sign: %.1f\n" % steer_sign)
        f.write("tau_steer: %.4f\n" % tau)
        f.write("max_steer_rate: %.4f\n" % rate)
        f.write("delay: %.4f\n" % delay)
        f.write("steer_track_width: %.4f  # given with --track-width\n" % W)
        f.write("ref_point_offset: 0.0  # measure: base_link to fixed-axle midpoint (m)\n")
    print("\nWrote %s" % args.out)


if __name__ == "__main__":
    main()
