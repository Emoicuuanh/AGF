# agf_clearance_field_gazebo

This package is a Gazebo 11 / ROS Noetic test bench for `agf_clearance_field`. It runs the real
`clearance_field_node.py` against a simulated vertical LiDAR, a lifting fork carrying a pallet
and load, and a lower stack. A scenario runner checks each `field_clear` decision against the
ground truth and against what physically happens when the AGF drives over the stack.

```
 side view (Gazebo camera)          lift_joint (fork_height)
      mast ║ ▓▓▓▓▓▓▓▓ carried load       │
           ║ ▓▓▓▓▓▓▓▓                    ▼
           ║ ════════ carried pallet / forks
   LiDAR ◉ ║                               ███ lower stack (respawned per case,
  ███████║                                 ███  contact sensor -> /lower_stack/contacts)
  ═══════╩════════ rail_joint (x) ════════════════════════════
       x=0 (LiDAR origin)      check at gap 1.6 m     stack face at x = 3.0
```

## Setup in WSL (Ubuntu-20.04, Noetic)

Requirements: `ros-noetic-gazebo-ros-pkgs`, `gazebo-ros-control`, `position-controllers`,
`effort-controllers`, `joint-state-controller`, `robot-state-publisher` and `xacro`. All of
these are already installed in the `Ubuntu-20.04` distro.

```bash
mkdir -p ~/clearance_ws/src && cd ~/clearance_ws/src
ln -s /mnt/e/backup-agf-02/src/agf_clearance_field/agf_clearance_field .
ln -s /mnt/e/backup-agf-02/src/agf_clearance_field/agf_clearance_field_gazebo .
cd ~/clearance_ws && source /opt/ros/noetic/setup.bash && catkin_make
source devel/setup.bash
```

The packages are symlinked, so edits made on Windows take effect without copying. You only need
to re-run `catkin_make` after adding new scripts.

## Run

```bash
# Automated test cases (config/scenarios.yaml), with the Gazebo GUI + RViz
roslaunch agf_clearance_field_gazebo clearance_sim.launch scenario:=true

# Headless / CI: exits when finished; the summary table is printed at the end
roslaunch agf_clearance_field_gazebo clearance_sim.launch gui:=false rviz:=false \
    scenario:=true exit_when_done:=true

# Test the TF mounting model instead of the manual one
roslaunch agf_clearance_field_gazebo clearance_sim.launch scenario:=true mode:=tf
```

Interactive use: start without `scenario:=true`, then change the scene live.

```bash
rosparam set /scene_manager/fork_height 1.30            # ramps the lift (0.25 m/s)
rosparam set /scene_manager/stack_height_actual 1.26    # respawns the lower stack
rosparam set /scene_manager/stack_height_estimate 1.20  # new estimate -> debouncer reset
rostopic pub -1 /agf/drive_controller/command std_msgs/Float64 0.2   # drive (m/s), 0 = stop
rostopic echo /clearance_field/status
```

If the Gazebo GUI crashes or stays black under WSLg, run `export LIBGL_ALWAYS_SOFTWARE=1`
first, or use `gui:=false` and rely on RViz.

## Results (2026-11-02, manual mode)

```
case                       fork    act    est    top bottom  field   truth     pts outcome   result
nominal_clear             1.400  1.144  1.144  1.194  1.278  CLEAR   CLEAR       0 placed    PASS
fork_too_low              1.250  1.144  1.144  1.194  1.128  BLOCKED BLOCKED   149 held      PASS
within_tolerance          1.300  1.144  1.144  1.194  1.178  BLOCKED CLEAR     145 held      PASS
stack_underestimated      1.350  1.260  1.144  1.194  1.228  CLEAR   BLOCKED     0 COLLIDED  PASS UNSAFE
stack_overestimated       1.300  1.044  1.144  1.194  1.178  BLOCKED CLEAR     145 held      PASS
tall_stack_clear          1.600  1.400  1.400  1.450  1.478  CLEAR   CLEAR       0 placed    PASS
tall_stack_too_low        1.500  1.400  1.400  1.450  1.378  BLOCKED BLOCKED   142 held      PASS
```

- `top` is the estimate plus the tolerance. `bottom` is the underside of the carried pallet,
  which is the fork height minus 0.122.
- `truth` compares `bottom` with the actual stack height.
- `stack_underestimated` is the known limit described in the main README. The field is correct
  for its input, but the input is 116 mm too low, so the AGF hits the stack.

## Model notes

- The AGF runs on a prismatic `rail_joint` along x. A velocity controller drives it, with force
  saturated at 20 kN. With a free-floating base, contacts pushed the truck up and over the stack.
- `lift_joint` is position-set every step, and `fork_height = lift_zero (1.40) + q`.
  `scene_manager` does this conversion.
- Collisions are detected by the contact sensor on the stack, not by the truck stopping. A
  position-set lift can be pushed upward by the contact instead of stalling the truck.
- The geometry matches `fake_scan_pub.py`. The LiDAR is at z = 1.05 with its scan plane at
  y = 0, between the forks (y = ±0.3). Pallets and loads are solid boxes. The LiDAR range noise
  has σ = 5 mm.
