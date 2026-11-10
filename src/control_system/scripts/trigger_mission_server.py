#!/usr/bin/env python

#########################
# rostopic pub -1 trigger std_stamped_msgs/StringStamped '{stamp: {secs: 0, nsecs: 0}, data: 'test_put_object_on_tray'}'
#########################
import os
import sys
import rospkg
import rospy
import datetime
import actionlib

agv_mongodb_dir = os.path.join(
    rospkg.RosPack().get_path("agv_mongodb"), "scripts"
)
if not os.path.isdir(agv_mongodb_dir):
    agv_mongodb_dir = os.path.join(
        rospkg.RosPack().get_path("agv_mongodb"), "release"
    )
sys.path.insert(0, agv_mongodb_dir)
import json
from std_stamped_msgs.msg import (
    StringStamped,
    StringFeedback,
    StringResult,
    StringAction,
    EmptyStamped,
)
from mongodb import mongodb


class TriggerMissionServer(object):
    _feedback = StringFeedback()
    _result = StringResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            StringAction,
            execute_cb=self.execute_cb,
            auto_start=False,
        )
        self._as.start()
        self.trigger_mission = rospy.Publisher(
            "trigger_mission_by_name", StringStamped, queue_size=1
        )
        # Subscriber
        self.message_pub = rospy.Publisher(
            "request_start_mission", StringStamped, queue_size=10
        )
        self.db = mongodb("mongodb://coffee:coffee@localhost:27017")

    def execute_cb(self, goal):
        r = rospy.Rate(5)
        success = True
        rospy.loginfo("%s action started" % (self._action_name))
        try:
            self.listQueue = list(self.db.queueCollection.find())
            if len(self.listQueue) == 0:
                return
            else:
                missionName = self.listQueue[0]["name"]
            ########################################################################
        except:
            self._result.status = 2
            rospy.loginfo("%s: Preempted" % self._action_name)
            self._as.set_preempted(self._result)

        while not rospy.is_shutdown():
            self._feedback.data = "Action running"
            self._as.publish_feedback(self._feedback)
            if self._as.is_preempt_requested():
                self._result.status = 2
                rospy.loginfo("%s: Preempted" % self._action_name)
                self._as.set_preempted(self._result)
                break
            else:
                msgPub = StringStamped()
                msgPub.stamp = rospy.Time.now()
                msgPub.data = missionName
                self.trigger_mission.publish(msgPub)
                rospy.sleep(0.1)
                self._result.status = 3
                rospy.loginfo("%s: Succeeded" % self._action_name)
                self._as.set_succeeded(self._result)
                break


if __name__ == "__main__":
    rospy.init_node("trigger_mission_server")
    rospy.loginfo("Init node: %s" % rospy.get_name())
    server = TriggerMissionServer(rospy.get_name())
    rospy.spin()
