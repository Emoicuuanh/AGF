#!/usr/bin/env python
import os, sys
import rospkg

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

from make_release_file import copy_ros_pkg, get_ws_devel_dir

current_dir = os.path.dirname(os.path.abspath(__file__))


def main():
    des_dir = ""
    if len(sys.argv) <= 1 or sys.argv[1] == "":
        print("There are no destination folder")
        return
    des_dir = sys.argv[1]
    print("Destination dir:", des_dir)

    copy_ros_pkg(
        des_dir=des_dir,
        pkg_name="control_system",
        include_files=("release",),
    )


if __name__ == "__main__":
    main()
