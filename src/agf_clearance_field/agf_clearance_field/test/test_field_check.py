"""Unit tests for the field logic (no ROS needed): python3 -m pytest test/"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from agf_clearance_field import (Debouncer, FieldParams, evaluate, plane_to_scan_manual,  # noqa: E402
                                 quat_to_matrix, required_fork_height, scan_to_plane_manual,
                                 scan_to_plane_tf)

PAL_H, PAL_L, FORK_L, FORK_T, DECK = 0.144, 1.2, 1.15, 0.045, 0.122
AMIN, INC = math.radians(-135), math.radians(0.5)
N = 541


def scene_scan(fork_h, stack_h=1.144, d=1.6, lidar_h=1.05, carried=0.9, direction=1):
    """2D ray-cast of the stacking scene in the scan plane (same model as the simulator)."""
    zb = fork_h - DECK
    bx = [(d, d + PAL_L, 0.0, stack_h), (0.03, 0.03 + PAL_L, zb, zb + PAL_H + carried),
          (-0.01, FORK_L, fork_h - FORK_T, fork_h), (-0.07, -0.01, fork_h - 0.2, fork_h + 0.9)]
    out = []
    for i in range(N):
        a = direction * (AMIN + i * INC)
        vx, vz = math.cos(a), math.sin(a)
        best = math.inf
        for b in bx:
            t0, t1, ok = -math.inf, math.inf, True
            for o, v, lo, hi in ((0.0, vx, b[0], b[1]), (lidar_h, vz, b[2], b[3])):
                if abs(v) < 1e-12:
                    ok = ok and lo <= o <= hi
                    continue
                p, q = sorted(((lo - o) / v, (hi - o) / v))
                t0, t1 = max(t0, p), min(t1, q)
            if ok and t0 <= t1 and t1 >= 0:
                best = min(best, max(t0, 0.0))
        if vz < -1e-9:
            best = min(best, -lidar_h / vz)
        out.append(best if best <= 10 else math.inf)
    return out


def plane(ranges, direction=1, lidar_h=1.05):
    return scan_to_plane_manual(ranges, AMIN, INC, 0.05, 10.0, 0.0, direction, lidar_h)


P = FieldParams(x_min=0.0, x_max=1.4, floor_margin=0.02, tolerance=0.05, min_points=3)


def test_clear_when_carried_pallet_above_field():
    x, z = plane(scene_scan(fork_h=1.35))          # underside 1.228 > 1.194
    r = evaluate(x, z, 1.144, P)
    assert r.valid and r.clear and r.count == 0
    assert abs(r.z_top - 1.194) < 1e-9


def test_blocked_when_carried_pallet_too_low():
    x, z = plane(scene_scan(fork_h=1.30))          # underside 1.178 < 1.194
    r = evaluate(x, z, 1.144, P)
    assert r.valid and not r.clear and r.count >= 3
    assert 1.17 < r.lowest_hit_z < 1.195


def test_floor_is_ignored():
    x, z = plane(scene_scan(fork_h=2.4, d=5.0))   # only floor + far stack in view
    assert evaluate(x, z, 1.144, P).clear


def test_lower_stack_inside_depth_blocks():
    x, z = plane(scene_scan(fork_h=1.6, d=1.0))
    r = evaluate(x, z, 1.144, P)
    assert not r.clear and r.nearest_hit_x >= 0.99


def test_mirrored_mounting():
    x, z = plane(scene_scan(fork_h=1.30, direction=-1), direction=-1)
    assert not evaluate(x, z, 1.144, P).clear


def test_invalid_without_height():
    x, z = plane(scene_scan(fork_h=1.35))
    r = evaluate(x, z, None, P)
    assert not r.valid and not r.clear


def test_tf_matches_manual():
    # scan frame rolled +90 deg about x: scan-frame y -> reference z
    c = math.sqrt(0.5)
    rot = quat_to_matrix(c, 0.0, 0.0, c)
    rng = scene_scan(fork_h=1.30)
    xt, zt = scan_to_plane_tf(rng, AMIN, INC, 0.05, 10.0, rot, (0.4, 0.15, 1.05), 0.0)
    xm, zm = plane(rng)
    assert np.allclose(xt, xm, atol=1e-9) and np.allclose(zt, zm, atol=1e-9)


def test_plane_to_scan_roundtrip():
    xs, zs = np.array([0.0, 1.4, 1.4, 0.0]), np.array([0.02, 0.02, 1.194, 1.194])
    sx, sy = plane_to_scan_manual(xs, zs, 0.3, -1, 1.05)
    r, a = np.hypot(sx, sy), np.arctan2(sy, sx)
    x2, z2 = r * np.cos(-(a - 0.3)), 1.05 + r * np.sin(-(a - 0.3))
    assert np.allclose(x2, xs) and np.allclose(z2, zs)


def test_debouncer_starts_blocked_and_has_hysteresis():
    d = Debouncer(trigger_scans=2, clear_scans=3)
    assert [d.update(True) for _ in range(3)] == [False, False, True]
    assert d.update(False) is True and d.update(False) is False


def test_required_fork_height():
    assert abs(required_fork_height(1.194, 0.122) - 1.316) < 1e-9
