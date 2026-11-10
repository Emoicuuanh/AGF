# MPC Path Tracking: Setup and Tuning Guide

This guide takes you from a fresh checkout to a tuned MPC tracker on a real
robot. It covers how to:

1. Install the solver dependencies and build the package
2. Identify the robot model from a rosbag
3. Configure the planner
4. Bring it up and check it in RViz
5. Tune it live with dynamic reconfigure
6. Troubleshoot common problems

The MPC replaces the old pure pursuit tracker (`control()` / `findPointInPath()`).
If you are upgrading a robot, read [Migrating from pure pursuit](#migrating-from-pure-pursuit) first.

---

## How it works

Each control cycle, `trackPath()` in [src/mpc_agf_planner.cpp](../src/mpc_agf_planner.cpp) does the following:

```
global plan ──setPlan()──► PathReference (resampled every path_ds, with s, yaw, kappa)
                             │
                             └─► computeSpeedProfile(): curve limit + braking to goal
every cycle:
  robot pose ─► reference point (base_link + ref_point_offset)
             ─► nearest sample (windowed search) ─► errors e_y, e_psi, s
  speed      ─► profile at s, capped by max_vel_x / speed-limit request / low-speed zone,
                ramped from the measured speed with a_acc / a_brake
  steering   ─► /steering_angle (left wheel) → centre angle
             ─► delay compensation ─► MPC (OSQP) ─► centre command → left wheel → angular.z
```

**Model.** The MPC uses a kinematic tricycle (bicycle) model in path-error
coordinates, with a first-order lag on the steering motor:

```
e_y'   = v · sin(e_psi)
e_psi' = v · tan(delta) · steer_sign / L  −  |v| · kappa
delta' = (delta_cmd − delta) / tau_steer
```

The model is linearized each step around the feedforward angle
`delta_ff = atan(steer_sign · sign(v) · L · kappa)`, which is the steering the
path curvature needs. The resulting QP is solved with OSQP and warm-started
from the previous cycle.

**Cost per step** (the last step is multiplied by `qf_scale`):

```
q_ey · e_y²  +  q_epsi · e_psi²  +  r_steer · (delta_cmd − delta_ff)²  +  r_dsteer · (Δdelta_cmd)²
```

**Constraints.**

- Steering angle: `|delta_cmd| ≤ max_steer`.
- Steering rate: `|Δdelta_cmd| ≤ max_steer_rate · mpc_dt`.

**Safety behaviour.**

- **Low-speed hold:** below `v_min_mpc`, the previous steering command is held.
  At near-zero speed, steering cannot reduce the error and only swerves the
  robot, which was the old near-goal swerve.
- **Fallback:** if OSQP fails, a simple stabilizing law steers the robot and
  the solver is rebuilt on the next cycle. The planner logs
  `MPC solve failed, using fallback steering`.

**Ackermann steering.** `/steering_angle` and the command (`angular.z`) are the
**left wheel's** angle. The MPC works with the virtual centre-wheel angle. The
planner converts between the two using `steer_track_width`; see
[include/mpc_agf_planner/steering_geometry.h](../include/mpc_agf_planner/steering_geometry.h).

> `cmd_vel.angular.z` is a **steering angle in radians**, not a yaw rate.
> `max_angular_z` limits the wheel angle.

The ROS-free core lives in the `agf_mpc_core` static library and is unit-tested on its own:

| File                                                                                                | Role                                                              |
| --------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------- |
| [path_reference.h](../include/mpc_agf_planner/path_reference.h) / [.cpp](../src/path_reference.cpp) | Resampling, heading, curvature, nearest-point search, path errors |
| [speed_profile.h](../include/mpc_agf_planner/speed_profile.h) / [.cpp](../src/speed_profile.cpp)    | Speed along the path                                              |
| [mpc_controller.h](../include/mpc_agf_planner/mpc_controller.h) / [.cpp](../src/mpc_controller.cpp) | QP setup, solve, delay compensation, fallback                     |
| [steering_geometry.h](../include/mpc_agf_planner/steering_geometry.h)                              | Left-wheel ↔ centre-angle conversion                             |

---

## 1. Install dependencies and build

OSQP and osqp-eigen have no rosdep keys on Noetic, so install them
system-wide from source. The code targets **OSQP 0.6.x** and
**osqp-eigen 0.8.x**. The 1.x APIs differ, so don't use them.

```bash
# OSQP 0.6.3
git clone --recursive -b v0.6.3 https://github.com/osqp/osqp.git
cd osqp && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc) && sudo make install
cd ../..

# osqp-eigen 0.8.1
git clone -b v0.8.1 https://github.com/robotology/osqp-eigen.git
cd osqp-eigen && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc) && sudo make install
sudo ldconfig
```

Install the remaining dependencies (Eigen, and numpy/rosbag for the
identification script) with rosdep:

```bash
rosdep install --from-paths src --ignore-src -y
```

Build the package and run the unit tests:

```bash
catkin_make --only-pkg-with-deps mpc_agf_planner
catkin_make run_tests_mpc_agf_planner
catkin_test_results build/test_results/mpc_agf_planner
```

The tests in [test/test_mpc.cpp](../test/test_mpc.cpp) cover:

- Path resampling and curvature
- Reverse paths and the Ackermann conversion
- The speed profile
- Convergence from a lateral offset, steady-state error on a circle
- Reverse docking, negative `steer_sign`, and both turn directions with Ackermann
- No swerve near the goal
- Solve time and the fallback path

All tests should pass before you put the planner on a robot.

> If CMake reports `Could not find a package configuration file provided by "OsqpEigen"`,
> osqp-eigen isn't installed, or isn't under a prefix CMake searches. Add
> `-DCMAKE_PREFIX_PATH=/usr/local` to the build.

---

## 2. Identify the robot model

The MPC is only as good as its model. The values in
[config/mpc_model.yaml](../config/mpc_model.yaml) are **placeholders**, except
`steer_track_width = 1.2 m`. Replace them with values measured on your robot.

| Parameter             | Meaning                                                    | How you get it                        |
| --------------------- | ---------------------------------------------------------- | ------------------------------------- |
| `wheelbase`         | Fixed axle to steered wheel (m)                            | Fitted from the circle test           |
| `steer_sign`        | `+1` if a positive angle turns left when driving forward | Fitted from the circle test           |
| `tau_steer`         | Steering motor time constant (s)                           | Small steering steps                  |
| `max_steer_rate`    | Steering motor rate limit (rad/s)                          | Large steering steps                  |
| `delay`             | Command-to-motion latency (s)                              | Steering and speed steps              |
| `steer_track_width` | Distance between the two steering pivots (m)               | **Measure with a tape**         |
| `ref_point_offset`  | `base_link` to fixed-axle midpoint, forward positive (m) | **Measure from the URDF/robot** |

### 2.1 Record the tests

For all three tests:

- Keep the planner idle (no active goal).
- Drive the robot by publishing `/cmd_vel` directly. `linear.x` is the speed
  and `angular.z` is the left-wheel steering angle.
- Use a clear, flat area.
- Record all three tests in one bag:

```bash
rosbag record -O model_id.bag /cmd_vel /steering_angle /odom
```

Before each test, set `/steering_angle` to the **measured** wheel angle, not
the commanded one. Otherwise the fitted lag and rate are meaningless.

**Test 1: steering steps at standstill.** With `linear.x = 0`, step the
steering between angles and hold each one for about 3 s. Do both directions,
using these amplitudes:

- `±0.3` rad: steps of 0.5 rad or less give `tau_steer`.
- `±0.8` rad and `±1.5` rad: steps of 0.7 rad or more give `max_steer_rate`.

Stay within `max_angular_z`.

```bash
rostopic pub -r 20 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.0}, angular: {z: 0.3}}'
# Ctrl-C, then repeat with 0.0, -0.3, 0.0, 0.8, -0.8, 1.5, -1.5, 0.0 ...
```

**Test 2: constant-steering circles.** Drive at `0.3` m/s with a fixed
steering angle (|angle| > 0.1 rad). Hold each angle for **more than 3 s**,
because the first 1.5 s is discarded while the robot settles. Do left and
right turns, **forward and reverse** (`linear.x = -0.3`). Use several angles,
for example ±0.3 and ±0.6. This test gives `wheelbase`, `steer_sign` and the
steering zero offset.

**Test 3: speed steps.** With the steering straight, step `linear.x` from `0`
to `0.3` and back a few times. The script measures how long `/odom` takes to
respond, which gives `delay`.

### 2.2 Fit the model

```bash
rosrun mpc_agf_planner identify_model.py model_id.bag \
    --track-width 1.2 --out mpc_model.yaml
```

| Option            | Default             | Use it when                                                                     |
| ----------------- | ------------------- | ------------------------------------------------------------------------------- |
| `--cmd-topic`   | `/cmd_vel`        | Commands were recorded on another topic                                         |
| `--steer-topic` | `/steering_angle` |                                                                                 |
| `--odom-topic`  | `/odom`           |                                                                                 |
| `--track-width` | `0`               | **Always set it** for Ackermann robots; `0` means a single centre wheel |

You can pass several bags. Read the output before you use it. The script
warns when:

| Warning                                          | Likely cause                                                        |
| ------------------------------------------------ | ------------------------------------------------------------------- |
| `tau_steer` outside 0.1–1.0 s                 | `/steering_angle` is echoing the command, not the measurement     |
| `max_steer_rate` outside 0.5–3.0 rad/s        | No ±1.5 rad steps were recorded                                    |
| `delay` outside 0.02–0.3 s                    | Clocks differ between the drive PC and the planner PC               |
| Forward/reverse wheelbase differ by more than 5% | Bad reverse circles; redo them                                      |
| Left/right wheelbase differ by more than 5%      | `--track-width` is wrong                                          |
| `steer_sign differs between segments`          | Inconsistent circle data                                            |
| `steering offset` above 0.01 rad               | Wheel zero is off;**fix it in the drive**, not in the planner |

If a test is missing, the script keeps the default for that value and says so
(for example, `no small steps, default kept`).

### 2.3 Finish the model file

The script writes `ref_point_offset: 0.0`. Replace it with the measured
distance from `base_link` to the midpoint of the fixed (non-steered) axle,
with forward positive. The kinematic model is only valid at that point, so a
wrong value shows up as a steady offset in curves.

Copy the result over [config/mpc_model.yaml](../config/mpc_model.yaml) or keep
it with the robot's config, and load it into the planner's namespace:

```bash
rosparam load mpc_model.yaml /move_base/MpcAgfPlannerROS
```

Or, in a launch file:

```xml
<node pkg="move_base" type="move_base" name="move_base">
  <rosparam file="$(find mpc_agf_planner)/config/mpc_model.yaml"
            command="load" ns="MpcAgfPlannerROS"/>
  ...
</node>
```

---

## 3. Configure the planner

All parameters are read in `initialize()` from the planner's private
namespace. The model parameters come from section 2.

### Model and steering

| Parameter                                                                   | Default               | Notes                                                                                                            |
| --------------------------------------------------------------------------- | --------------------- | ---------------------------------------------------------------------------------------------------------------- |
| `wheelbase`, `steer_sign`, `tau_steer`, `max_steer_rate`, `delay` | 1.2, 1, 0.4, 1.0, 0.1 | From`identify_model.py`                                                                                        |
| `steer_track_width`                                                       | 0.0                   | **Set it for Ackermann robots.** At 0 you get the warning `steer_track_width not set`                    |
| `ref_point_offset`                                                        | 0.0                   | Measured                                                                                                         |
| `max_angular_z`                                                           | 1.2                   | Wheel angle limit (rad). The MPC's centre-angle limit is derived from it so that the inner wheel stays within it |

### MPC

| Parameter       | Default | Live? | Notes                                                       |
| --------------- | ------- | ----- | ----------------------------------------------------------- |
| `mpc_horizon` | 25      | no    | Prediction steps                                            |
| `mpc_dt`      | 0.1     | no    | Step length (s). The look-ahead is`horizon × dt` = 2.5 s |
| `q_ey`        | 40.0    | yes   | Lateral error weight                                        |
| `q_epsi`      | 6.0     | yes   | Heading error weight                                        |
| `r_steer`     | 0.3     | yes   | Penalty on steering away from the curvature feedforward     |
| `r_dsteer`    | 1.0     | yes   | Penalty on steering change per step                         |
| `qf_scale`    | 10.0    | yes   | Weight multiplier on the last step                          |
| `v_min_mpc`   | 0.05    | no    | Below this speed, steering is held instead of solved        |

### Speed

| Parameter       | Default | Live?     | Notes                                                                               |
| --------------- | ------- | --------- | ----------------------------------------------------------------------------------- |
| `max_vel_x`   | 0.7     | no        | Top speed. The speed-limit topic and low-speed zones can lower it further           |
| `a_lat_max`   | 0.25    | next plan | Curve speed limit: `v ≤ sqrt(a_lat_max /                                           |
| `a_dec`       | 0.3     | next plan | Planned braking toward the goal                                                     |
| `v_creep_min` | 0.1     | next plan | The profile never goes below this, so the robot always reaches the goal             |
| `creep_dist`  | 0.2     | next plan | Distance before the goal tolerance zone where the robot is already at `v_creep_min`, so the stop is short |
| `stop_lead_time` | 0.5   | yes       | The stop is sent this long before the goal (distance = speed × time) to cover the coast-down. Raise it if the robot stops past the goal, lower it if it stops short |
| `a_acc`       | 0.4     | no        | Acceleration ramp from the measured speed                                           |
| `a_brake`     | 1.2     | no        | Deceleration ramp when the target drops, for example when entering a low-speed zone |
| `path_ds`     | 0.05    | no        | Path resampling step (m)                                                            |

"Live" means you can change it with dynamic reconfigure (section 5). "Next
plan" means the change takes effect when the next path is received, because
that's when the speed profile is built.

### Goal behaviour (unchanged from before)

`xy_goal_tolerance`, `yaw_goal_tolerance`, `ignore_last_rotate`,
`max_correction_attempts` and `car_like/*`.

When tracking reaches the goal, the final rotation behaves as follows:

- It is skipped if the heading is already within `yaw_goal_tolerance`.
- If the position drifts out of tolerance during the rotation, the MPC drives
  back to the goal, up to `max_correction_attempts` times.

### Topics the planner needs

| Topic                              | Type                    | Content                                               |
| ---------------------------------- | ----------------------- | ----------------------------------------------------- |
| `/steering_angle`                | `std_msgs/Float64`    | **Measured** left-wheel angle (rad)             |
| `/cmd_vel_feedback`              | `geometry_msgs/Twist` | Measured speed. The speed ramp starts from this value |
| `odom_topic` (default `/odom`) | `nav_msgs/Odometry`   |                                                       |

---

## 4. Bring-up checklist

Work through these in order. Each step catches a different class of mistake
before it causes trouble at speed.

1. **Check the startup log.** It should print the model you loaded:
   ```
   MPC model: L 1.200 m, steer_sign 1, tau 0.400 s, rate 1.00 rad/s, delay 0.100 s
   Steering: track width 1.200 m, centre angle limit ... rad for wheel limit 1.200 rad
   ```

   If you still see the placeholder values, the yaml went into the wrong
   namespace.
2. **Add the debug topics in RViz:**
   - `/mpc_agf_planner/local_path` (Path): the remaining reference, up to 10 m ahead
   - `/mpc_agf_planner/mpc_predicted` (Path): where the MPC expects the robot to go
   - `/mpc_agf_planner/reference_point` (Marker): blue sphere at the robot's projection on the path
3. **Run a straight line, forward, at low `max_vel_x` (0.3).** Start 0.2 m
   off the path. The predicted path should curve smoothly back onto the
   reference, and the robot should follow it.
4. **Run a straight line in reverse.** This checks `steer_sign` and the
   reverse heading handling.
5. **Run left and right curves.** The two directions should look symmetric. If
   only one direction tracks badly, `steer_track_width` is wrong.
6. **Run a short goal (under 1 m).** The robot should creep in without
   swerving at the end.
7. **Raise `max_vel_x`** to the operating speed and repeat steps 3–5.

Plot the tracking error while you test:

```bash
rqt_plot '/mpc_agf_planner/tracking_error/data[0]' '/mpc_agf_planner/tracking_error/data[1]'  # quotes needed in zsh
```

`/mpc_agf_planner/tracking_error` is a `Float64MultiArray` with these fields:

| Index | Value                                                        |
| ----- | ------------------------------------------------------------ |
| 0     | `e_y`: lateral error (m), positive = left of the path      |
| 1     | `e_psi`: heading error (rad)                               |
| 2     | Measured steering,**centre** angle (rad)               |
| 3     | Commanded steering,**centre** angle (rad)              |
| 4     | Commanded speed (m/s, signed)                                |
| 5     | Solve time (ms); 0 while steering is held below`v_min_mpc` |

Indices 2 and 3 are centre angles (model space), so on an Ackermann robot they
won't match `/steering_angle` exactly.

---

## 5. Tuning

Get the model right before you touch the weights. A wrong `tau_steer` or
`delay` causes weaving that no weight setting will remove.

Open the live tuner:

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

When you change a weight, the solver resets and the new weights apply on the
next cycle. The speed parameters (`a_lat_max`, `a_dec`, `v_creep_min`) apply
from the next goal. Once you're happy with the values, copy them into the
robot's params file. Dynamic reconfigure values are lost on restart.

Use the defaults as a starting point and change one value at a time:

| Symptom                                                     | Try                                                                                                           |
| ----------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| Slow to return to the path, or a steady offset on straights | Raise`q_ey` (×1.5 at a time)                                                                               |
| Weaves or oscillates around the path                        | Check`tau_steer`/`delay` first, then raise `r_dsteer` or lower `q_ey`                                 |
| Steering is jerky or chatters                               | Raise`r_dsteer`                                                                                             |
| Heading settles slowly after converging laterally           | Raise`q_epsi`                                                                                               |
| Cuts or overshoots curves                                   | Check`wheelbase` and `ref_point_offset`, then raise `r_steer` to lean more on the curvature feedforward |
| Overshoots when the path ends or changes sharply            | Raise`qf_scale`                                                                                             |
| Too fast in curves                                          | Lower`a_lat_max`                                                                                            |
| Brakes too late before the goal                             | Lower`a_dec`                                                                                                |
| Takes too long to creep into the goal                       | Lower `creep_dist` or raise `v_creep_min` (and check `xy_goal_tolerance`)                                                        |

Watch `e_y` in `rqt_plot` while you tune. A good result converges without
overshoot and keeps the steering command (index 3) smooth.

---

## 6. Troubleshooting

| What you see                                           | Cause and fix                                                                                                                                                                                |
| ------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Robot steers the wrong way and diverges at once        | `steer_sign` is wrong. Rerun test 2, or flip it                                                                                                                                            |
| Good in one turn direction, poor in the other          | `steer_track_width` is wrong or 0                                                                                                                                                          |
| Steady offset in every curve                           | `wheelbase` or `ref_point_offset` is wrong                                                                                                                                               |
| Cuts or overshoots tight 90° turns; log shows `Path curve radius ... is tighter than the robot's minimum turning radius` | The path is tighter than the robot can steer. Make the corner wider in the global planner (a Bézier corner's radius is about 0.71 × its leg length), or raise `max_angular_z` if the wheel can turn further |
| Weaving that gets worse with speed                     | `delay` or `tau_steer` is underestimated; re-identify                                                                                                                                    |
| Robot never goes faster than about 0.15–0.2 m/s           | `/cmd_vel_feedback` isn't published. Speeding up is capped at the measured speed + 0.15 m/s, and the measured speed reads 0 |
| `MPC solve failed, using fallback steering`          | Look for odd model values (`tau_steer` near 0, or a huge `max_steer_rate`). If it happens only occasionally, check the solve time in `tracking_error[5]` against the controller period |
| Solve time close to the controller period              | Lower`mpc_horizon` (and raise `mpc_dt` to keep the look-ahead)                                                                                                                           |
| `Local planner: received an empty plan`              | The global planner sent an empty path.`setPlan` now rejects it                                                                                                                             |
| Swerves sideways at the goal during the final rotation | Shouldn't happen any more: the rotation is skipped within`yaw_goal_tolerance`, and the rolling stops when the error changes sign. If it does, check `car_like/*`                         |

---

## Migrating from pure pursuit

These parameters were **removed**. You can delete them from the robot config,
because they no longer have any effect:

```
distance_to_control_point   lookahead_dist    look_forward_gain
weight_acc                  weight_theta_e    weight_theta_d
k_p                         priority_angle_adjustment  (dynamic reconfigure)
```

The speed "modes A/B" have been replaced by the speed profile (`a_lat_max`,
`a_dec`, `v_creep_min`).

Steps for an existing robot:

1. Install OSQP and osqp-eigen, then rebuild (section 1).
2. Identify the model and load it (section 2). **Don't** run on the
   placeholders.
3. Set `steer_track_width` and `ref_point_offset`.
4. Run the bring-up checklist at low speed (section 4).
