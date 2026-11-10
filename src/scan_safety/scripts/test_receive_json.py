#!/usr/bin/env python

from std_msgs import msg
import std_stamped_msgs
from std_stamped_msgs.msg import StringStamped
import json
import rospy
import os 

def settingCb(msgs):
    print(msgs)
    safetyMsg = json.loads(msgs.data)
    print(safetyMsg)

if __name__ == "__main__":
    # rospy.init_node("test_receive_json")
    # rospy.Subscriber("safety_job", StringStamped, settingCb)
    # rospy.spin()
    dir_path = os.path.dirname(os.path.realpath(__file__))
    print(dir_path)
    f = open(dir_path + "/default.json")
    data = json.load(f)
    print(data)
