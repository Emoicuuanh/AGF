#!/usr/bin/env python
# -*- coding: utf-8 -*-


import os
import sys
import json
import rospy
import rospkg
import argparse
import actionlib
import time
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Int16, Int8
from math import pi
from actionlib_msgs.msg import GoalStatus
from std_stamped_msgs.msg import StringAction, StringGoal

goal_result = GoalStatus()

class PlannerSettingClient(object):
    def __init__(self, json_file):
        file_path = os.path.join(rospkg.RosPack().get_path('moving_control'), 'json_template', json_file)
        print(file_path)
        # Action client
        self.planner_setting_client = actionlib.SimpleActionClient('planner_setting', StringAction)
        self.planner_setting_client.wait_for_server(timeout=rospy.Duration(2))

        goal = StringGoal()

        with open(file_path) as j:
            data = json.load(j)

        rospy.loginfo("Send action goal:\n%s"%(json.dumps(data, indent=2)))

        goal.data = json.dumps(data, sort_keys=True)

        # Variables
        self.duplicate_count = 1
        self.last_fb = ''

        # Action
        self.planner_setting_client.send_goal(goal, feedback_cb=self.feedback)
        result = self.planner_setting_client.wait_for_result()
        rospy.loginfo("Action result: {}".format(result))

    def feedback(self, fb):
        if fb == self.last_fb:
            self.duplicate_count += 1
            sys.stdout.write("\033[F")
            rospy.loginfo("Action feedback: {} ({})".format(fb, self.duplicate_count))
        else:
            self.duplicate_count = 1
            rospy.loginfo("Action feedback: %s"%fb)
        self.last_fb = fb

def parse_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument("-j", '--json_file', default='planner_setting_0.2.json', type=str, help='json file path')
    opt = parser.parse_args()
    return opt

def main():
    rospy.init_node('planner_setting_client')
    rospy.loginfo('Init node ' + rospy.get_name())
    opt = parse_opt()
    print(opt)
    PlannerSettingClient(**vars(opt))

if __name__ == '__main__':
    main()