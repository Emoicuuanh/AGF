#!/usr/bin/env python

import os
import sys
import json
import rospkg
import rospy
import datetime
from std_stamped_msgs.msg import StringStamped
from geometry_msgs.msg import Pose

agv_mongodb_dir = os.path.join(
    rospkg.RosPack().get_path("agv_mongodb"), "scripts"
)
if not os.path.isdir(agv_mongodb_dir):
    agv_mongodb_dir = os.path.join(
        rospkg.RosPack().get_path("agv_mongodb"), "release"
    )
sys.path.insert(0, agv_mongodb_dir)

from mongodb import mongodb
from common_function import dict_to_obj, print_info, print_debug


class TriggerManager(object):
    def __init__(self, *args, **kwargs):
        rospy.Subscriber("trigger", StringStamped, self.triggerCallback)
        rospy.Subscriber(
            "trigger_complex", StringStamped, self.triggerComplexCallback
        )
        rospy.Subscriber("current_map", StringStamped, self.currentMapCallback)
        self.message_pub = rospy.Publisher(
            "request_start_mission", StringStamped, queue_size=10
        )
        db_address = rospy.get_param("/mongodb_address")
        self.db = mongodb(db_address)
        self.currentMap = ""
        rospy.loginfo("Start trigger_manager node")
        self.last_run_cb_create_mission = rospy.get_time()
        rospy.spin()

    def triggerCallback(self, msg):
        topic = msg.data
        trigger = self.db.dataListCollection.find_one(
            {"data.topic": topic, "map": self.currentMap}
        )
        if trigger == None:
            return
        self.db.emptyQueueMission()
        missionQueue = {}
        missionQueue["name"] = trigger["data"]["mission"]
        missionQueue["time"] = datetime.datetime.now()
        msgPub = StringStamped()
        msgPub.stamp = rospy.Time.now()
        msgPub.data = "START"
        self.db.newMissionQueue(missionQueue)
        self.message_pub.publish(msgPub)
        rospy.loginfo("Received trigger and request start mission")
        pass

    def currentMapCallback(self, msg):
        self.currentMap = msg.data
        pass

    def triggerComplexCallback(self, msg):
        if rospy.get_time() - self.last_run_cb_create_mission  < 2.0:
            return
        self.last_run_cb_create_mission = rospy.get_time()

        data_dict = json.loads(msg.data)
        topic = data_dict["topic"]
        triggerQuery = list(
            self.db.dataListCollection.find(
                {"data.topic": topic, "map": self.currentMap}
            )
        )
        # print_info(
        #     "Trigger query: \n{}".format(self.db.bson2Json(triggerQuery))
        # )
        if triggerQuery == None:
            return

        runImmediately = data_dict["run_immediately"]
        if runImmediately:
            self.db.emptyQueueMission()

        currentPoseDict = data_dict["pose"] if "pose" in data_dict else None
        currentPose = None
        if currentPoseDict != None:
            currentPose = dict_to_obj(currentPoseDict, Pose())

        poseAllows = []
        poseAllowTrigger = False
        missionName = ""
        for trigger in triggerQuery:
            print_info(
                "Event: {}, pose: {}".format(
                    trigger["data"]["topic"], trigger["data"]["pose"]
                )
            )
            missionName = trigger["data"]["mission"]
            print_info("Mission name: {}".format(missionName))

        #     poseAllows = (
        #         trigger["data"]["pose"] if "pose" in trigger["data"] else []
        #     )
        #     if poseAllows != []:
        #         for poseCheck in poseAllows:
        #             poseQuery = self.db.dataListCollection.find_one(
        #                 {
        #                     "type": "position",
        #                     "map": self.currentMap,
        #                     "name": poseCheck,
        #                 }
        #             )
        #             # print_info(
        #             #     "Pose in DB: \n{}".format(self.db.bson2Json(poseQuery))
        #             # )
        #             if poseQuery != None:
        #                 poseCheckData = poseQuery["data"]
        #                 poseCheckObj = dict_to_obj(poseCheckData, Pose())
        #                 # if self.checkNearPoint(poseCheckObj, currentPose):
        #                 checkResult = self.checkNearPoint(
        #                     poseCheckObj, currentPose, 0.3
        #                 )
        #                 print_info("Check result: {}".format(checkResult))
        #                 if checkResult:
        #                     poseAllowTrigger = True
        #                     rospy.loginfo(
        #                         'Trigger allow in "{}"'.format(poseCheck)
        #                     )
        #                     break
        #                 else:
        #                     rospy.logwarn(
        #                         'Trigger not allow in "{}"'.format(
        #                             "current position"
        #                         )
        #                     )
        #         # Only have 1 couple of event and position
        #         if poseAllowTrigger:
        #             break

        # if poseAllows != [] and poseAllowTrigger == False:
        #     return

        # (poseAllows != [] and poseAllowTrigger == True) or poseAllows == []
        if missionName == "":
            rospy.loginfo(
                "There is no mission assign with topic: {}".format(topic)
            )
            return
        self.db.makeMissionWithVar(missionName, self.currentMap, data_dict)
        rospy.loginfo('Added mission "{}" to queue'.format(missionName))

        if runImmediately:
            msgPub = StringStamped()
            msgPub.stamp = rospy.Time.now()
            msgPub.data = "START"
            self.message_pub.publish(msgPub)
            rospy.loginfo("Received trigger and request start mission")

    def checkNearPoint(self, pose_1, pose_2, xy_tol):
        if (
            abs(pose_1.position.x - pose_2.position.x) < xy_tol
            and abs(pose_1.position.y - pose_2.position.y) < xy_tol
        ):
            return True
        return False


if __name__ == "__main__":
    rospy.init_node("trigger_manager")
    TriggerManager()
