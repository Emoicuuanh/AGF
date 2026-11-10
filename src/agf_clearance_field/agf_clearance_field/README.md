# agf_clearance_field

ROS1 (Noetic, rospy) node for a vertical-plane 2D LiDAR on the AGF. It checks a
rectangular **clearance field** bounded by:

- **bottom:** the ground line (plus `floor_margin`, so floor returns never trigger it)
- **top:** the estimated top of the lower stack plus `tolerance`
- **depth:** `x_min … x_max` in front of the LiDAR origin

When the field is clear, the carried pallet underside and the fork tips are above the
estimated stack top (+ tolerance), so the AGF may drive over the lower stack. Any returns
inside the field mean something is too low (the carried pallet or forks, or an obstacle),
and the AGF must not move in.

```
      carried pallet  ▓▓▓▓▓▓▓▓
   z ┌──────────── field ────────┐  ← stack_height_est + tolerance
     │                           │
     │                           │        lower stack ███
  ───┴───────────────────────────┴──────────────────────── floor (+ floor_margin)
     x_min                     x_max     (do the check while the stack is beyond x_max)
```

## Build

```bash
cd ~/catkin_ws/src && cp -r /path/to/agf_clearance_field .
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

## Run

```bash
# On the robot
roslaunch agf_clearance_field clearance_field.launch \
    scan_topic:=/lidar_vertical/scan height_topic:=/agf/lower_stack_height

# Bench test with a synthetic scan + RViz
roslaunch agf_clearance_field clearance_field.launch sim:=true rviz:=true
rosparam set /fake_scan/fork_height 1.30          # carried pallet drops into the field -> BLOCKED
rosparam set /fake_scan/fork_height 1.35          # -> CLEAR
rosservice call /clearance_field/check            # one-shot evaluation of the latest scan
```

## Interface

| Topic / service | Type | Notes |
|---|---|---|
| `scan` (sub) | `sensor_msgs/LaserScan` | vertical scan plane |
| `lower_stack_height` (sub) | `std_msgs/Float32` | estimated lower-stack top **above the floor**, pallet included [m]. A new value resets the debouncer (starts BLOCKED). |
| `fork_height` (sub, optional) | `std_msgs/Float32` | fork-top height [m]. Only used for reporting `margin_to_field_top`. |
| `enable` (sub, optional) | `std_msgs/Bool` | used when `require_enable: true` |
| `~field_clear` (pub) | `std_msgs/Bool` | debounced. **False on any fault**: no scan, no height, TF error, disabled. |
| `~intrusion_count` (pub) | `std_msgs/Int32` | returns inside the field in the last scan |
| `~status` (pub) | `std_msgs/String` (JSON) | reason, field bounds, lowest/nearest hit, `required_fork_height` |
| `~markers` (pub) | `visualization_msgs/MarkerArray` | field outline (green/red), estimate line, intruding points |
| `~check` (srv) | `std_srvs/Trigger` | evaluates the latest scan without debounce; `success` = clear |

## Mounting

You can describe the mounting in one of two ways (`config/clearance_field.yaml`):

- **`mode: manual`**
  - `lidar_height`: height of the scan centre above the floor.
  - `angle_forward_deg`: the scan angle of the horizontal beam that points at the load.
  - `angle_direction`: `+1` if increasing angle tilts the beam up, `-1` if it tilts it down.

  To calibrate: place a box at a known distance and height and check `lowest_hit_z` / `nearest_hit_x` in `~status`.
- **`mode: tf`**: scan points are transformed into `reference_frame`, which must have x forward, z up and the floor at `floor_z`. Use this when the LiDAR has a URDF/static TF.

Lateral offset of the LiDAR (Y) does not enter the field maths. It only decides which part of the pallet the scan plane cuts. Keep the plane clear of the forks (±300 mm in the simulator).

## Limits

- The field top comes from the **estimate**. A lower stack that is taller than estimated by more than `tolerance` is **not** detected by this field. Verify the estimate separately, for example with a second field just above the estimated top, or by measuring the stack top.
- Run the check while the lower stack is farther than `x_max`. Once the stack is inside the field depth, it triggers the field itself.
- This is not a safety function. Use a certified safety device or a safety PLC for personnel protection and hard interlocks.

## Tests

```bash
python3 -m pytest test/            # no ROS needed
catkin_make run_tests_agf_clearance_field
```
