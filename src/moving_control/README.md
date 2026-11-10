# How to use

### 1. Run any ROS navigation program with `move_base/teb_local_planner`, `move_base/mpc_local_planner`... `move_to_point`

### 2. Run moving_control_server

```bash
rosrun moving_control moving_control_server.py
```

Or run with specific `planner_map.yaml` config file to reduce define un-use local_planner type

```bash
rosrun moving_control moving_control_server.py --planner_map path_to_planner_map.yaml
```

- Template of config file (see comment in file): [planner_map.yaml](cfg/planner_map.yaml)

### 3. Run moving_control_client to test

```bash
rosrun moving_control moving_control_client.py -j test_waypoints.json
```

- User can define custom waypoints similar [test_waypoints.json](json_template/test_waypoints.json)

- `params` in `waypoints` list will merge and overwrite `param` in root of json tree. If `params` in specific waypoint has not been defined, this waypoint will use default `params` in root of json file tree.

- Default config in [default_params.json](cfg/default_params.json). You can send a goal json without any params.

### 4. Parameter

`overwrite_planner_setting`:

- Decs: cho phép update reconfigure parameter
- Default: False
- Use: Thường dùng khi muốn thay đổi tham số của các waypoint trong 1 path (VD: docking cần thay đổi vận tốc, gia tốc từng waypoint)
