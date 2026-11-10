#!/usr/bin/env python
import os
import sys
import rospy
import threading
from subprocess import call
from subprocess import Popen
import rospkg
import subprocess
from serial_port import get_serial_ports

common_folder = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_folder):
    common_folder = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_folder)
from common_function import EnumString, static_vars
from agv_msgs.msg import ArduinoIO

ARDUINO_TIMEOUT = 2
arduino_io_timeout = None
rosserial_process = None


def f1():
    global rosserial_process
    rosserial_process = subprocess.Popen(
        ["rosrun", "rosserial_python", "serial_node.py"]
    )  #  _port:=/dev/ttyACM0 _baud:=115200


def f2():
    rosserial_process.kill()


def rosserial_run(name):
    os.system(
        """ lxterminal -e " source /opt/ros/melodic/setup.bash; source ~/catkin_ws/devel/setup.bash; rosrun rosserial_python serial_node.py _port:={} _baud:=115200; read -n 1 -s " """.format(
            name
        )
    )


@static_vars(
    step=-1,
    prev_step=-1,
    time_check=None,
    doing=False,
    try_port=0,
    list_port=[],
)
def check_arduino_connection():
    if check_arduino_connection.step != check_arduino_connection.prev_step:
        print(
            "check arduino connection: {} -> {}".format(
                check_arduino_connection.prev_step,
                check_arduino_connection.step,
            )
        )
    check_arduino_connection.prev_step = check_arduino_connection.step

    if (
        rospy.get_time() - arduino_io_timeout > ARDUINO_TIMEOUT
        and check_arduino_connection.doing == False
    ):
        print("Arduino timeout")
        check_arduino_connection.step = 0
        check_arduino_connection.doing = True

    # Get port
    if check_arduino_connection.step == 0:
        check_arduino_connection.list_port = get_serial_ports("AU")
        raspberry_port = "/dev/ttyAMA0"
        if raspberry_port in check_arduino_connection.list_port:
            check_arduino_connection.list_port.remove(raspberry_port)
        print(check_arduino_connection.list_port)
        if len(check_arduino_connection.list_port) > 0:
            check_arduino_connection.try_port = 0
            check_arduino_connection.step = 1
        else:
            print("There is no any port")
    # Kill node if alive
    elif check_arduino_connection.step == 1:
        os.system("rosnode kill /rosserial")
        check_arduino_connection.step = 2
    # Run new node
    elif check_arduino_connection.step == 2:
        port = check_arduino_connection.list_port[
            check_arduino_connection.try_port
        ]
        cmd = "rosrun rosserial_python serial_node.py _port:={} _baud:=115200".format(
            port
        )
        os.system(cmd)
        check_arduino_connection.time_check = rospy.get_time()
        check_arduino_connection.step = 3
    # Wait
    elif check_arduino_connection.step == 3:
        if rospy.get_time() - check_arduino_connection.time_check > 5:
            check_arduino_connection.doing = False


def arduino_io_cb(msg):
    global arduino_io_timeout

    arduino_io_timeout = rospy.get_time()


def main():
    global arduino_io_timeout

    # CheckDbFile('node_manager')

    rospy.init_node("agv_node_manager")
    rospy.Subscriber("/arduino_io", ArduinoIO, arduino_io_cb)
    arduino_io_timeout = rospy.get_time()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        check_arduino_connection()
        rate.sleep()


if __name__ == "__main__":
    main()
