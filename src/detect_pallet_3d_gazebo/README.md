# detect_pallet_3d_gazebo

Gazebo (classic 11, ROS Noetic) bench for the Livox pallet detectors in `detect_pallet_3d`:

| scenario | detector | output frame | scored in base_link |
|---|---|---|---|
| `front` | `livox_center_detector` | `pallet_front_frame` | distance of the center from the pallet front face, yaw |
| `edge`  | `livox_edge_detector`   | `pallet_edge_frame`  | distance to the nearest front corner (x and y), yaw |

The main purpose is the **Livox mount orientation** (`livox_roll` / `livox_pitch`): a tilted Livox
must give the same detection in `base_link` as a flat one.

## Bench

- `livox_rig`: `base_link` on the floor plus a Livox MID-360-like ray sensor (360° x -7..52°,
  720 x 60 rays, 10 Hz, 2 mm range noise). Mount args `livox_x/y/z/roll/pitch/yaw` default to
  `pallet_tf_publisher.yaml`. They set both the Gazebo sensor pose and the URDF TF
  `base_link -> livox_frame`. `planar_move` publishes `odom -> base_link`; drive the rig with `/cmd_vel`.
- `block_laser_to_cloud2.py`: block laser `PointCloud` -> `/livox/lidar` `PointCloud2`
  (x, y, z, intensity float32, `livox_frame`), as `livox_ros_driver2` publishes it. Misses are dropped.
- Pallet `pallet_loaded` (default) or `pallet_empty`: 1.2 x 1.0 x 0.15 m, three stringers with
  fork pockets, optional 0.6 m load flush with the front. The model origin is the front-face
  center on the floor, with x pointing out of the face (the detector frame convention).
- Both detectors run with the `amr_config` yaml. Their `livox_roll` / `livox_pitch` come from
  `detector_roll` / `detector_pitch`, which default to the real mount.
- `detection_eval.py` enables the detector, skips a 3 s warm-up, and scores `duration` s.
  It prints mean / std per error and PASS / FAIL (default tolerance 2 cm and 1°).

## Run

```bash
catkin build detect_pallet_3d_gazebo        # or catkin_make; needs gazebo_ros, gazebo_plugins

# interactive (Gazebo + RViz), front face 1.2 m ahead
roslaunch detect_pallet_3d_gazebo pallet_detection_sim.launch
# pallet edge, Livox pitched 0.15 rad (positive = x axis tilted down)
roslaunch detect_pallet_3d_gazebo pallet_detection_sim.launch scenario:=edge livox_pitch:=0.15
# misconfiguration: Livox tilted, detectors told it is flat -> mount warning in the log; the
# edge result shifts (~4 cm at 0.15 rad). The front detector barely moves for a small pitch
# error (its center is a measured point), only its crop band tilts.
roslaunch detect_pallet_3d_gazebo pallet_detection_sim.launch scenario:=edge livox_pitch:=0.15 detector_pitch:=0.0
# edge detector with odom as fixed frame while driving
roslaunch detect_pallet_3d_gazebo pallet_detection_sim.launch scenario:=edge edge_fixed_frame:=odom
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: -0.05}}'

# headless: flat + 4 tilts, each must PASS and match flat (1 cm / 1.5 deg)
rosrun detect_pallet_3d_gazebo run_tilt_matrix.sh front
rosrun detect_pallet_3d_gazebo run_tilt_matrix.sh edge
TILTS="0 0;0 0.25;3.14159 0" rosrun detect_pallet_3d_gazebo run_tilt_matrix.sh front
```

Other args: `pallet_x/y/yaw`, `pallet_model`, `use_y_filter`, `duration`, `h_samples`,
`v_samples`, `range_noise`, `center_config`, `edge_config`.

## Known detector issue found with this bench

`scenario:=edge` with the default pallet fails even with a flat Livox. Through the fork pockets
the Livox sees the horizontal deck surface (2-7 cm above the sensor, inside the edge crop band).
`find_side_plane` accepts it as the side face, because it only checks that the plane is
perpendicular to the front. The "edge" is then horizontal, and extending it to `edge_ref_z`
throws the origin metres away. Requiring a vertical side plane (|normal z| small, and trying
the next plane otherwise) fixes it: the corner is then found within about 2 cm, and tilted
mounts match flat. The fix is not applied in `detect_pallet_3d` yet.

## Limits

- The ray pattern is a regular grid, not the Livox non-repetitive pattern, so there are fewer
  points per frame at 0.1 s `buffer_time`. Raise `h_samples` / `v_samples` if a detector starves.
  CPU ray sensors get slow above roughly 100k rays.
- A tilted mount hits the pallet with different rays than a flat one, so flat and tilted results
  agree within noise, not bit for bit.
- A large tilt moves the field of view. For example, with roll = π (upside down) the MID-360
  only sees up to 7° above the horizon, which can cut the top of the crop band.
- The rig places the pallet on the floor. In placing, the edge detector looks at the pallet on
  the forks; only the geometry relative to the Livox matters to it.
