# Mock Robot Pose Publisher

A simple ROS node that publishes robot pose to replace MRPT for testing AGF without running the full SLAM system.

## Features

- Publishes `geometry_msgs/Pose` on `/robot_pose` topic
- Publishes TF transform from `map` to `base_link`
- Three operating modes:
  - **static**: Publish a fixed pose
  - **moving**: Simulate robot movement
  - **odom**: Forward pose from odometry topic

## Installation

```bash
cd ~/all_prj/AGF/agf_app_ws
catkin_make --pkg mock_robot_pose
source devel/setup.bash
```

## Usage

### Static Pose (Default)
Publish a fixed pose at origin:
```bash
roslaunch mock_robot_pose mock_pose.launch
```

Publish a fixed pose at specific location:
```bash
roslaunch mock_robot_pose mock_pose.launch x:=2.0 y:=1.5 yaw:=1.57
```

### Moving Pose
Simulate robot moving forward at 0.2 m/s:
```bash
roslaunch mock_robot_pose mock_pose.launch mode:=moving speed:=0.2
```

Simulate robot moving in a circle:
```bash
roslaunch mock_robot_pose mock_pose.launch mode:=moving circle_mode:=true circle_radius:=2.0 speed:=0.3
```

Simulate robot rotating in place:
```bash
roslaunch mock_robot_pose mock_pose.launch mode:=moving speed:=0.0 angular_speed:=0.5
```

### From Odometry
Forward pose from `/odom` topic:
```bash
roslaunch mock_robot_pose mock_pose.launch mode:=odom
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| mode | string | "static" | Operating mode: static, moving, odom |
| x | double | 0.0 | Initial X position (meters) |
| y | double | 0.0 | Initial Y position (meters) |
| yaw | double | 0.0 | Initial yaw angle (radians) |
| rate | double | 10.0 | Publishing rate (Hz) |
| global_frame_id | string | "map" | Global frame ID |
| base_frame_id | string | "base_link" | Base frame ID |
| speed | double | 0.1 | Linear speed (m/s) for moving mode |
| angular_speed | double | 0.0 | Angular speed (rad/s) for moving mode |
| circle_mode | bool | false | Enable circular motion |
| circle_radius | double | 2.0 | Circle radius (meters) |

## Topics

### Published
- `/robot_pose` (geometry_msgs/Pose): Robot pose in global frame
- TF: `map` → `base_link` transform

### Subscribed (odom mode only)
- `/odom` (nav_msgs/Odometry): Odometry data

## Examples

Test AGF with robot at parking position:
```bash
roslaunch mock_robot_pose mock_pose.launch x:=0.0 y:=0.0 yaw:=0.0
```

Test AGF with robot moving along warehouse:
```bash
roslaunch mock_robot_pose mock_pose.launch mode:=moving speed:=0.3 angular_speed:=0.1
```
