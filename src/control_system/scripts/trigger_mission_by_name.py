#!/usr/bin/env python

import os
from pydoc_data import topics
import sys
import rospkg
import rospy
import datetime
from std_stamped_msgs.msg import StringStamped

agv_mongodb_dir = os.path.join(
    rospkg.RosPack().get_path("agv_mongodb"), "scripts"
)
if not os.path.isdir(agv_mongodb_dir):
    agv_mongodb_dir = os.path.join(
        rospkg.RosPack().get_path("agv_mongodb"), "release"
    )
sys.path.insert(0, agv_mongodb_dir)

from mongodb import mongodb


class TriggerManager(object):
    def __init__(self, *args, **kwargs):
        rospy.Subscriber(
            "trigger_mission_by_name", StringStamped, self.triggerCallback
        )
        self.message_pub = rospy.Publisher(
            "request_start_mission", StringStamped, queue_size=10
        )
        self.db = mongodb("mongodb://coffee:coffee@localhost:27017")
        rospy.loginfo("Start trigger_manager node")
        rospy.spin()

    def triggerCallback(self, msg):
        rospy.sleep(3)
        topic = msg.data
        self.db.emptyQueueMission()
        missionQueue = {}
        missionQueue["name"] = topic
        missionQueue["time"] = datetime.datetime.now()
        msgPub = StringStamped()
        msgPub.stamp = rospy.Time.now()
        msgPub.data = "START"
        self.db.newMissionQueue(missionQueue)
        self.message_pub.publish(msgPub)
        rospy.loginfo("Received trigger and request start mission")
        pass


if __name__ == "__main__":
    rospy.init_node("trigger_mission_by_name")
    TriggerManager()
