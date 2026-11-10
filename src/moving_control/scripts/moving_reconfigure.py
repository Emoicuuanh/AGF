#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import json
import threading
import yaml
import argparse
from rosgraph.names import is_private
import rospy
import rospkg
import tf
import copy
import actionlib
from math import pi
import time
from nav_msgs.msg import Odometry
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import  Pose, PoseStamped, PoseArray, Twist
from std_msgs.msg import Int8, Header
from std_stamped_msgs.msg import StringStamped, StringFeedback, StringResult, StringAction, EmptyStamped
from actionlib_msgs.msg import GoalStatus, GoalID, GoalStatusArray
from safety_msgs.msg import SafetyStatus

common_func_dir = os.path.join(rospkg.RosPack().get_path('agv_common_library'), 'scripts')
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(rospkg.RosPack().get_path('agv_common_library'), 'release')
sys.path.insert(0, common_func_dir)

from module_manager import ModuleServer, ModuleClient, ModuleStatus
from common_function import (
    EnumString,
    lockup_pose,
    obj_to_dict,
    offset_pose_x,
    pose_stamped_array_to_pose_array,
    distance_two_pose,
    get_yaw,
    delta_angle,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_warn,
    print_error,
    print_info,
    MIN_FLOAT,
    distance_to_line_perpendicular_vs_goal,
    angle_robot_vs_robot_to_goal,
    yaw_to_quaternion,
    pose_dict_template
)

class MovingReconfigure(object):
    def __init__(self, name, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        # Publisher
        self.module_status_pub = rospy.Publisher('/moving_reconfigure_status', StringStamped, queue_size=5)
        # Subscriber
        rospy.Subscriber('/set_reconfigure_params', StringStamped, self.set_reconfigure_cb)
        # Loop
        self.loop()

    """
    #### ##    ## #### ######## ####    ###    ##
     ##  ###   ##  ##     ##     ##    ## ##   ##
     ##  ####  ##  ##     ##     ##   ##   ##  ##
     ##  ## ## ##  ##     ##     ##  ##     ## ##
     ##  ##  ####  ##     ##     ##  ######### ##
     ##  ##   ###  ##     ##     ##  ##     ## ##
    #### ##    ## ####    ##    #### ##     ## ########
    """

    def init_variable(self, *args, **kwargs):
        self.simulation = (kwargs["simulation"])
        print_debug("simulation: {}".format(self.simulation))

    def shutdown(self):
        self.cancel_all_action()
        print("Shutdown: {}".format(rospy.get_name()))

    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def set_reconfigure_cb(self, msg):
        start_time = rospy.get_time()
        try:
            msg_dict = json.loads(msg.data)
            client = dynamic_reconfigure.client.Client(msg_dict["server"], timeout=1)
            config = client.update_configuration(msg_dict["param_dict"])
        except Exception as e:
            rospy.logerr("Set reconfigure: {}".format(e))
        print_debug("Set reconfigure: \n{}".format(json.dumps(msg_dict, indent=2)))
        rospy.loginfo("Set reconfigure time: {}".format(round((rospy.get_time() - start_time), 3)))

    """
    ##        #######   #######  ########
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ########
    ##       ##     ## ##     ## ##
    ##       ##     ## ##     ## ##
    ########  #######   #######  ##
    """

    def loop(self):
        r = rospy.Rate(2.0)
        status_msg = StringStamped()
        while not rospy.is_shutdown():
            status_msg.stamp = rospy.Time.now()
            self.module_status_pub.publish(status_msg)
            r.sleep()

def parse_opts():
    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("-s", "--simulation",
                    action="store_true", dest="simulation", default=False, help="type \"-s\" if simulation")
    parser.add_option("-d", "--ros_debug",
                    action="store_true", dest="log_debug", default=False, help="log_level=rospy.DEBUG")

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)

def main():
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    rospy.init_node('moving_reconfigure', log_level=log_level, disable_signals=True)
    rospy.loginfo('Init node ' + rospy.get_name())
    MovingReconfigure(rospy.get_name(), **vars(options))

if __name__ == '__main__':
    main()