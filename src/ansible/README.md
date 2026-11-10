# AGF robot setup with Ansible

This playbook installs everything the AGF workspace needs on a fresh **Ubuntu 20.04** PC (ROS Noetic), then builds the workspace. It is safe to run again: finished steps are skipped.

## What it installs

| Role | Installs | Needed by |
|---|---|---|
| `base` | build-essential, cmake, git, rsync, ssh | everything |
| `ros_noetic` | ROS Noetic desktop, rosdep, catkin tools, ROS packages from launch files (usb_cam, rosbridge, joy, aruco_detect, ...) | all ROS packages |
| `system_libs` | Boost, Eigen, PCL, OpenCV, yaml-cpp, nlohmann-json, curl, apr, bullet, SDL, Qt5, VLC | agf_system, map_server, tf2_bullet, rviz_navigation_plugin, sound_control |
| `python_deps` | pymodbus, pymongo, pyserial, flask, psycopg2, shapely, peewee, websockets, python-vlc, open3d, scikit-learn | agf_plc_communication, agv_mongodb, sensorAPI, map_zone, path_creator, detect_pallet_3d |
| `mongodb` | MongoDB server + user `coffee` | agv_mongodb, control_system, agf_system |
| `mongo_cxx` | mongo-c-driver and mongo-cxx-driver in `/usr/local` (source build) | agf_system |
| `mrpt` | MRPT with ros1bridge in `/opt/mrpt` (source build) | mrpt_icp_slam_2d |
| `livox_sdk2` | Livox-SDK2 in `/usr/local` (source build) | livox_ros_driver2 |
| `hardware` | serial/camera/audio permissions, udev rules, UDP buffers, optional static IP for the lidar NIC | lidars, PLC, RS232, camera |
| `workspace` | `rosdep install` for anything left, then `catkin_make` | — |

All settings are in [group_vars/all.yml](group_vars/all.yml).

## 1. Prepare the robot

1. Install **Ubuntu 20.04** (desktop) and create user `mkac`. If you use a different user, set `robot_user`.
2. Connect it to the internet. The playbook downloads from apt, pip and GitHub.
3. Put the source in `/home/mkac/agf_app_ws/src`. You can use git clone or a USB copy, or let Ansible copy it (see step 3).

## 2. Install Ansible

Ansible does not run on Windows. Run it **on the robot itself** (simplest), or from a Linux or WSL machine.

```bash
sudo apt update && sudo apt install -y python3-pip
pip3 install --user "ansible-core>=2.12,<2.14"
export PATH=$HOME/.local/bin:$PATH
cd /home/mkac/agf_app_ws/src/ansible
ansible-galaxy collection install -r requirements.yml
```

## 3. Configure

Edit [group_vars/all.yml](group_vars/all.yml). The settings to check first:

| Variable | Default | Check |
|---|---|---|
| `robot_user`, `ws_root` | `mkac`, `~/agf_app_ws` | Match your robot |
| `pymodbus_version` | `3.6.9` | On a working robot: `pip3 show pymodbus` |
| `mrpt_version` | `2.10.0` | On a working robot: `grep MRPT_VERSION /opt/mrpt/include/mrpt/version.h` |
| `mongodb_version` | `auto` | 7.0, or 4.4 on CPUs without AVX |
| `udev_serial_rules` | empty | Add fixed names for USB-serial devices |
| `lidar_nic` | empty | NIC facing the lidars/PLC, to set a static IP |
| `install_perception_python` | `true` | Set `false` if detect_pallet_3d is not used (open3d is large) |

To copy the source from a laptop instead of putting it on the robot yourself, set `ws_sync_src: true` and `ws_sync_src_from: /path/to/src/`.

## 4. Run

**On the robot:**

```bash
cd /home/mkac/agf_app_ws/src/ansible
ansible-playbook -i "localhost," -c local site.yml -K
```

**From a laptop over SSH (one or many robots):** add the robots to [inventory.ini](inventory.ini), then:

```bash
ssh-copy-id mkac@192.168.1.101
ansible-playbook site.yml -K                  # all robots
ansible-playbook site.yml -K --limit agf02    # one robot
```

`-K` asks for the sudo password. The first run takes **1–2 hours**, mostly building MRPT.

### Run only some parts

Each role has a tag:

```bash
ansible-playbook -i "localhost," -c local site.yml -K --tags python        # pip packages only
ansible-playbook -i "localhost," -c local site.yml -K --tags workspace     # rosdep + catkin_make
ansible-playbook -i "localhost," -c local site.yml -K --skip-tags mrpt     # everything except MRPT
```

Tags: `base ros libs python mongodb mongocxx mrpt livox hardware workspace`

## 5. After the first run

1. **Log out and log back in**, or reboot. This applies the new group membership needed for serial and camera access.
2. Check each part:

```bash
source ~/.bashrc
rospack find mrpt_icp_slam_2d agf_system livox_ros_driver2   # packages built
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/video*                  # devices visible
groups                                                        # includes dialout video
mongosh -u coffee -p coffee --eval 'db.adminCommand("ping")'  # 'mongo' on MongoDB 4.4
python3 -c "import pymodbus, pymongo, serial, vlc; print(pymodbus.__version__)"
ping 192.168.1.1xx                                            # each lidar / PLC
```

3. Set up the robot config and autostart with `ros_entry` as before (`ros_entry/scripts/create_entry_point.sh`).

## Troubleshooting

| Problem | Fix |
|---|---|
| `packages.ros.org` 404 / missing Noetic | Noetic is end-of-life. Set `ros_apt_url: http://snapshots.ros.org/noetic/final/ubuntu` |
| `mongod` exits with `Illegal instruction` | The CPU has no AVX. Set `mongodb_version: "4.4"`, then `sudo apt purge mongodb-org*` and run again with `--tags mongodb` |
| MRPT build killed (out of memory) | Lower `heavy_build_jobs` to `1`, or add swap |
| `mrpt-ros1bridge-config.cmake` not found | The MRPT version is too new (ros1bridge moved out in 2.11). Use 2.10.x or the version from a working robot. Delete `/opt/agf_build_src/mrpt` before rebuilding |
| PLC reads wrong or zero values | `plc_pymodbus_client.py` passes `unit=1`, but pymodbus 3.x expects `slave=1` and ignores `unit`. Pin the pymodbus version from a working robot, or change the code to `slave=` |
| `catkin_make` fails on a missing package | The launch files reference packages that are not in this workspace: `agf_action_common`, `agvlidar_fiducial`, `auto_docking`, `keya_servo`, `move_base_manager`, `picking_pallet`. Add their source if you need them |
| Serial device "Permission denied" | Log out and log back in (groups), and check `ls -l /dev/ttyUSB0` |

## Folder layout

```
ansible/
├── README.md            # this guide
├── ansible.cfg
├── inventory.ini        # robot list (SSH mode)
├── requirements.yml     # ansible.posix collection
├── site.yml             # main playbook
├── group_vars/all.yml   # all settings
├── CATKIN_IGNORE        # stops catkin from scanning this folder
└── roles/
    ├── base/  ros_noetic/  system_libs/  python_deps/
    ├── mongodb/  mongo_cxx/  mrpt/  livox_sdk2/
    └── hardware/  workspace/
```
