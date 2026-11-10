"""Pure-Python core of the clearance-field check (no ROS imports, unit-testable).

Geometry
--------
The LiDAR is mounted with its scan plane vertical, pointing toward the load.
Every valid beam is converted into plane coordinates:

    x  : horizontal distance in front of the LiDAR origin [m]
    z  : height above the floor [m]

The field is the rectangle bounded by the ground line and the estimated top
line of the lower stack:

    x_min <= x <= x_max
    floor_margin < z <= stack_height + tolerance

The field is CLEAR when fewer than ``min_points`` returns fall inside it.
If anything lower than the estimated stack top (+ tolerance) is in front of
the LiDAR within [x_min, x_max] -- typically the carried pallet or the fork
tips hanging too low -- the field is BLOCKED and the AGF must not drive in.
"""

import math
from typing import NamedTuple, Optional, Sequence, Tuple

import numpy as np


class FieldParams(NamedTuple):
    x_min: float = 0.0          # field start in front of the LiDAR [m]
    x_max: float = 1.4          # field end [m]
    floor_margin: float = 0.02  # ignore returns up to this height (floor noise) [m]
    tolerance: float = 0.05     # added on top of the estimated stack height [m]
    min_points: int = 3         # returns needed inside the field to call it blocked


class FieldResult(NamedTuple):
    valid: bool                 # False -> inputs missing / inconsistent (treat as blocked)
    clear: bool                 # instantaneous result for this scan
    count: int                  # returns inside the field
    z_bottom: float             # field bottom (floor line + margin) [m]
    z_top: float                # field top (stack line + tolerance) [m]
    lowest_hit_z: Optional[float]
    nearest_hit_x: Optional[float]
    reason: str


# ---------------------------------------------------------------- conversions
def beam_angles(n: int, angle_min: float, angle_increment: float) -> np.ndarray:
    return angle_min + np.arange(n, dtype=float) * angle_increment


def valid_mask(r: np.ndarray, range_min: float, range_max: float) -> np.ndarray:
    return np.isfinite(r) & (r >= range_min) & (r <= range_max)


def scan_to_plane_manual(ranges: Sequence[float], angle_min: float, angle_increment: float,
                         range_min: float, range_max: float,
                         angle_forward: float, direction: int,
                         lidar_height: float) -> Tuple[np.ndarray, np.ndarray]:
    """Manual mounting model.

    angle_forward : scan angle [rad] of the beam pointing horizontally toward the load
    direction     : +1 if increasing scan angle tilts the beam UP, -1 if DOWN
    lidar_height  : height of the scan centre above the floor [m]
    """
    r = np.asarray(ranges, dtype=float)
    a = beam_angles(r.size, angle_min, angle_increment)
    m = valid_mask(r, range_min, range_max)
    th = direction * (a[m] - angle_forward)
    return r[m] * np.cos(th), lidar_height + r[m] * np.sin(th)


def quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw) or 1.0
    x, y, z, w = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def scan_to_plane_tf(ranges: Sequence[float], angle_min: float, angle_increment: float,
                     range_min: float, range_max: float,
                     rot: np.ndarray, trans: Sequence[float],
                     floor_z: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
    """TF mounting model.

    rot, trans : transform reference_frame <- scan_frame (reference frame: x forward, z up)
    floor_z    : z of the floor in the reference frame (0 for base_footprint)
    x is measured from the LiDAR origin along the reference x axis.
    """
    r = np.asarray(ranges, dtype=float)
    a = beam_angles(r.size, angle_min, angle_increment)
    m = valid_mask(r, range_min, range_max)
    pts = np.vstack([r[m] * np.cos(a[m]), r[m] * np.sin(a[m]), np.zeros(int(m.sum()))])
    p = rot @ pts + np.asarray(trans, dtype=float).reshape(3, 1)
    return p[0] - float(trans[0]), p[2] - floor_z


def plane_to_scan_manual(x: np.ndarray, z: np.ndarray, angle_forward: float, direction: int,
                         lidar_height: float) -> Tuple[np.ndarray, np.ndarray]:
    """Inverse of scan_to_plane_manual -> (x, y) in the scan frame (for RViz markers)."""
    x = np.asarray(x, dtype=float)
    zr = np.asarray(z, dtype=float) - lidar_height
    r = np.hypot(x, zr)
    a = angle_forward + np.arctan2(zr, x) / direction
    return r * np.cos(a), r * np.sin(a)


# ---------------------------------------------------------------- evaluation
def field_bounds(stack_height: float, p: FieldParams) -> Tuple[float, float]:
    return p.floor_margin, stack_height + p.tolerance


def evaluate(x: np.ndarray, z: np.ndarray, stack_height: Optional[float],
             p: FieldParams) -> FieldResult:
    if stack_height is None or not math.isfinite(stack_height) or stack_height <= 0.0:
        return FieldResult(False, False, 0, p.floor_margin, float('nan'), None, None,
                           'no_stack_height')
    if p.x_max <= p.x_min:
        return FieldResult(False, False, 0, p.floor_margin, float('nan'), None, None,
                           'bad_x_range')
    zb, zt = field_bounds(stack_height, p)
    if zt <= zb:
        return FieldResult(False, False, 0, zb, zt, None, None, 'field_height_not_positive')
    x = np.asarray(x, dtype=float)
    z = np.asarray(z, dtype=float)
    inside = (x >= p.x_min) & (x <= p.x_max) & (z > zb) & (z <= zt)
    n = int(inside.sum())
    low = float(z[inside].min()) if n else None
    near = float(x[inside].min()) if n else None
    clear = n < p.min_points
    return FieldResult(True, clear, n, zb, zt, low, near, 'clear' if clear else 'intrusion')


class Debouncer:
    """Hysteresis on the per-scan result. Starts BLOCKED (fail-safe)."""

    def __init__(self, trigger_scans: int = 2, clear_scans: int = 3):
        self.trigger_scans = max(1, int(trigger_scans))
        self.clear_scans = max(1, int(clear_scans))
        self.state_clear = False
        self._run = 0

    def reset(self) -> None:
        self.state_clear = False
        self._run = 0

    def update(self, clear_now: bool) -> bool:
        if clear_now == self.state_clear:
            self._run = 0
            return self.state_clear
        self._run += 1
        need = self.clear_scans if clear_now else self.trigger_scans
        if self._run >= need:
            self.state_clear = clear_now
            self._run = 0
        return self.state_clear


def required_fork_height(field_top: float, deck_offset: float) -> float:
    """Fork-top height at which the carried pallet's underside sits on the field top.

    deck_offset: distance from fork top down to the carried pallet's underside
    (pallet height minus top-deck thickness, 0.122 m for a 144 mm pallet).
    """
    return field_top + deck_offset
