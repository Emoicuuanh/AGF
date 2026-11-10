#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Example: rosrun agv_common_library find_port_and_run.py roslaunch
# receptionist_bringup S1.launch port_laser_1 1-1 port_laser_2 3-4

import serial
import serial.tools.list_ports
import os
import sys
import rospy

input_argv = rospy.myargv(argv=sys.argv)
print("find_port_and_run: {}".format(input_argv))

cmd = ""
pkg = ""
file = ""
port_pair = []
if len(input_argv) >= 6 and len(input_argv) % 2 == 0:
    cmd = input_argv[1]
    pkg = input_argv[2]
    file = input_argv[3]
    for i in range(4, len(input_argv), 2):
        port_pair.append([input_argv[i], input_argv[i + 1]])
    final_cmd = cmd + " " + pkg + " " + file

    for i in range(len(port_pair)):
        for port in serial.tools.list_ports.comports():
            if port_pair[i][1] == port.location:
                port_pair[i][1] = port.device
                print("Set slot {} to {}".format(port.location, port.device))
                break
            print("There is no port for slot: {}".format(port_pair[i][1]))

    for i in range(len(port_pair)):
        final_cmd += " " + port_pair[i][0] + ":=" + port_pair[i][1]
    print("final command: {}".format(final_cmd))
    os.system(final_cmd)
else:
    print("args missing")
