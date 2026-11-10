#!/usr/bin/env python
import os
from os.path import expanduser
import sys
import rospkg
import shutil

HOME = expanduser("~")


def copy_folder_exclude(src_folder, des_folder, *exclude_files):
    cmd = "rsync -av --progress {} {}".format(src_folder, des_folder)
    if len(exclude_files):
        for i in exclude_files:
            cmd += " --exclude {}".format(i)
    print(cmd)
    os.system(cmd)


def copy_file(src, des):
    cmd = "rsync -av --progress {} {}".format(src, des)
    print(cmd)
    os.system(cmd)


def copy_folder_include(src_folder, des_folder, *include_files):
    include_str = " ".join(include_files)
    print(f"cp -r {src_folder}/{{{include_str}}} {des_folder}")


def copy_ros_pkg(
    des_dir,
    pkg_name,
    inside_dir="",
    include_files=tuple(),
    specific_files=tuple(),
):
    print("------------------Copy pkg: ", pkg_name, "------------------")
    if not isinstance(include_files, tuple):
        raise TypeError("include_files must be tuple")

    if inside_dir != "":
        des_dir = des_dir + "/" + inside_dir
        # os.system("mkdir -p {}".format(des_dir))
    cmd = "mkdir -p {}".format(des_dir + "/" + pkg_name)
    print(cmd)
    os.system(cmd)

    # List all files and directories in the source folder
    src_folder = os.path.join(rospkg.RosPack().get_path(pkg_name))
    src_items = os.listdir(src_folder)

    # Filter the files and directories to copy
    items_to_copy = [item for item in src_items if item in include_files]

    # Don't use it anymore because it uses the install folder
    # for roscpp_item in include_files:
    #     if "devel" in roscpp_item:
    #         items_to_copy.append(roscpp_item)

    # for s_file in specific_files:
    #     items_to_copy.append(s_file)

    items_to_copy.append("CMakeLists.txt")
    items_to_copy.append("package.xml")

    print("Items to copy", items_to_copy)

    # Copy the files and directories to the destination folder
    for item in items_to_copy:
        src_path = ""
        des_path = ""
        if "devel" not in item:
            src_path = os.path.join(src_folder, item)
            des_path = os.path.join(des_dir, pkg_name, item)
        else:
            src_path = item
            des_path = os.path.join(des_dir, pkg_name)
        print("Coping", src_path, " --> ", des_path)
        if os.path.isdir(src_path):  # Direction
            shutil.copytree(
                src_path,
                des_path,
                dirs_exist_ok=True,
                ignore=shutil.ignore_patterns("*/*.build", "*.build", "*.pyi"),
            )
        else:  # File, not a direction
            shutil.copy(src_path, des_path)


def get_parent_dir(dir, level=1):
    if level == 1:
        return os.path.abspath(os.path.join(dir, os.pardir))
    elif level > 1:
        for i in range(level):
            dir = os.path.abspath(os.path.join(dir, os.pardir))
        return dir
    else:
        raise ValueError(
            "Invalid level value. Level must be greater than or equal to 1."
        )


def get_ws_devel_dir(pkg_name):
    current_dir = os.path.dirname(os.path.abspath(__file__))
    ws_dir = get_parent_dir(current_dir, 3)
    lib_devel_dir = os.path.join(ws_dir, "devel", "lib", pkg_name)
    return lib_devel_dir


def main():
    des_dir = ""
    if len(sys.argv) <= 1 or sys.argv[1] == "":
        print("There are no destination folder")
        return
    des_dir = sys.argv[1]
    print(des_dir)

    # # agv_lidar
    # copy_ros_pkg(des_dir, "agvlidar_common", "agv_lidar")
    # copy_ros_pkg(des_dir, "agvlidar_bringup", "agv_lidar")
    # copy_ros_pkg(des_dir, "agvlidar_navigation", "agv_lidar")
    # copy_ros_pkg(des_dir, "agvlidar_simulation", "agv_lidar")
    # copy_ros_pkg(des_dir, "agvlidar_description", "agv_lidar")
    # # rospy node
    # copy_ros_pkg(des_dir, "amr_config", "")
    # copy_ros_pkg(des_dir, "control_system", "")
    # copy_ros_pkg(des_dir, "mission_manager", "")
    # copy_ros_pkg(des_dir, "slam_manager", "")
    # copy_ros_pkg(des_dir, "moving_control", "")
    # copy_ros_pkg(des_dir, "agv_mongodb", "")
    # copy_ros_pkg(des_dir, "agv_common_library", "")
    # copy_ros_pkg(des_dir, "sound_control", "")
    # copy_ros_pkg(des_dir, "led_control", "")
    # copy_ros_pkg(des_dir, "system_setting", "")
    # copy_ros_pkg(des_dir, "clear_log", "")
    # copy_ros_pkg(des_dir, "ultra_sonic", "")
    # copy_ros_pkg(des_dir, "keya_servo", "")
    # copy_ros_pkg(des_dir, "arduino_ros", "")
    # copy_ros_pkg(des_dir, "fiducial_msgs", "fiducials", "msg")
    copy_ros_pkg(
        des_dir=des_dir,
        pkg_name="aruco_detect",
        inside_dir="fiducials",
        include_files=(
            "launch",
            get_ws_devel_dir("aruco_detect"),
            get_ws_devel_dir("fiducial_slam"),
        ),
    )
    # roscpp node


if __name__ == "__main__":
    main()
