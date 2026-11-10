#! /usr/bin/env python

from mission_manager.msg import MissionFeedback, MissionGoal, MissionResult, MissionAction
import json
import sys, os
import rospy
import rospkg
import actionlib

path = os.path.join(rospkg.RosPack().get_path("mission_manager"), "mission_list")

try:
    mission_file = os.path.join(path, sys.argv[1])
except:
    rospy.logwarn("Did not input mission file. Using default")
    mission_file = os.path.join(path, "mission.json")

rospy.loginfo("Mission file: %s"%mission_file)

def feedback(fb):
    rospy.loginfo("Current Action: %s"%fb)

rospy.init_node("mission_client", log_level=rospy.DEBUG)

mission_client = actionlib.SimpleActionClient("mission_server", MissionAction)

mission_client.wait_for_server()

mgoal = MissionGoal()

with open(mission_file) as j:
    data = json.load(j)

rospy.loginfo("Send mission: %s"%(json.dumps(data, indent=2)))

mgoal.mission = json.dumps(data, sort_keys=True)

mission_client.send_goal(mgoal, feedback_cb=feedback)
mission_client.wait_for_result()