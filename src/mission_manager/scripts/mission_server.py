#!/usr/bin/env python
import rospy
import sys, os
import rospkg
from std_msgs.msg import String
import json
import inspect
from log_ultils import *
from queue import Queue
import threading

import actionlib
from mission_manager.msg import CheckPositionStatusAction, CheckPositionStatusFeedback, CheckPositionStatusGoal, CheckPositionStatusResult, CheckPositionStatusActionResult
from auto_docking.msg import ChargingActionResult, CartActionResult
from actionlib_msgs.msg import GoalStatus
from std_msgs.msg import String
from follow_waypoints.msg import FollowWaypointsActionResult
from mission_manager.msg import MissionAction, MissionFeedback, MissionActionGoal, MissionResult
import dynamic_reconfigure.client as recfg_client
class MissionStep:
    WAITING     = 0
    GET_ACTION  = 1
    EXCUTIVE    = 5
    WAIT_RESULT = 10
    PAUSE       = 15
    RESUME      = 20
    CANCEL      = 25

    show = {
        0:  "WAITING",       # Waiting for new action
        1:  "GET_ACTION",
        5:  "EXCUTIVE",      # send goals to action
        10: "WAIT_RESULT",   # Waiting for result
        15: "PAUSE",         # Pause mission
        20: "RESUME",        # Resume mission
        25: "CANCEL"           # Cancel mission
    }

class ActionResult:
    PENDING     = 0
    SUCCEEDED   = 1
    PREEMPTED   = 5
    RETRY       = 10
    FAIL        = 10
    ERROR       = 99

class MissionManager(object):
    _feedback = MissionFeedback()
    _result = MissionResult()

    def __init__(self, name):
        self._action_name = name


        self.test_docking_pub = rospy.Publisher("/test_docking", String, queue_size=10)
        self.single_path_pub = rospy.Publisher("/run_single_path", String, queue_size=10)

        # --- Subscribers to Actions result
        rospy.Subscriber("/marker_docking/result", ChargingActionResult, self.chargingResultCB)
        rospy.Subscriber("/check_position_status/result", CheckPositionStatusActionResult, self.checkPosResultCB)
        rospy.Subscriber("follow_waypoints/result", FollowWaypointsActionResult, self.followWaypointsResultCB )
        rospy.Subscriber("/auto_cart/result", CartActionResult, self.autoCartCB)

        # Setup check_position_status client
        self.checkPositionClient = actionlib.SimpleActionClient('check_position_status', CheckPositionStatusAction)
        if not self.checkPositionClient.wait_for_server():
            mylogerr("Cannot connect to check_position_status server")
            return False

        # Load mission database json
        db_path = os.path.join(rospkg.RosPack().get_path("mission_manager"), "db/mission_db.json")
        with open(db_path) as j:
            self.missiondb = json.load(j)

        # Dynamic reconfigure client
        self.safety_recfg_enable = False
        try:
            self.safety_recfg_client = recfg_client.Client("scan_safety", timeout=5)
            self.safety_recfg_enable = True
        except rospy.ROSException as e:
            rospy.logerr("Safety reconfigure client fail: %s"%e)
            self.safety_recfg_enable = False

        self._actionResult = GoalStatus().PENDING
        self._missionStep = MissionStep.WAITING
        self._preStep = 99

        self.action_queue = Queue()
        self.current_mission = ''
        self.current_action = ''
        self.current_action_type = ''
        self.mutex = threading._allocate_lock()

        self._as = actionlib.SimpleActionServer(self._action_name, MissionAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        mission_dict = json.loads(goal.mission)
        self.current_mission = mission_dict["name"]
        rospy.logdebug(self.current_mission)
        actions = mission_dict["actions"]
        total_action = len(actions)
        myloginfo("Received new mission: %s, have %i action(s)"%(self.current_mission, total_action))
        self.mutex.acquire()
        if self.action_queue.empty():
            for action in actions:
                myloginfo("Add action: %s"%action["name"])
                self.action_queue.put(action)
            self._missionStep = MissionStep.GET_ACTION
        self.mutex.release()
        success = False

        # empty = False
        cnt = 0
        while not success:
            # Khi chuyen buoc, hien thong bao
            if self._missionStep != self._preStep:
                mylogdebug("Mission Step: %s"%MissionStep.show.get(self._missionStep))
                self._preStep = self._missionStep

            # Check neu co yeu cau huy, huy action va tra ve False
            if self._as.is_preempt_requested():
                myloginfo("%s: Preempted during action %s"%(self.current_mission, self.current_action))
                self.mutex.acquire()
                self.action_queue.queue.clear()
                self.mutex.release()
                self._as.set_preempted()
                success = False
                break

            # Buoc 1: Get thong tin action. Publish ten action toi feedback
            if self._missionStep == MissionStep.GET_ACTION:
                self.mutex.acquire()
                if not self.action_queue.empty():
                    action = self.action_queue.get()
                    cnt += 1
                    self.current_action = action["name"]
                    self.current_action_type = action["type"]
                    myloginfo("Processing: %s/%s"%(self.current_mission, self.current_action))
                    self._missionStep = MissionStep.EXCUTIVE
                    self._actionResult = GoalStatus.PENDING
                    self._feedback.action = self.current_action
                    self._as.publish_feedback(self._feedback)
                    # empty = True
                else:
                    # if empty:
                    #     myloginfo("action empty")
                    #     empty = False
                    # self._missionStep = MissionStep.WAITING
                    myloginfo("Action queue empty")
                    if cnt == total_action:
                        success = True
                    else:
                        success = False
                        break
                self.mutex.release()

            # Buoc 2: Thuc hien action
            elif self._missionStep == MissionStep.EXCUTIVE:
                if action["type"] == "check_position_status":
                    goal = CheckPositionStatusGoal()
                    goal.x = action["params"]["position"]["x"]
                    goal.y = action["params"]["position"]["y"]
                    goal.theta = action["params"]["position"]["theta"]
                    goal.desired = action["params"]["desired"]
                    goal.timeout = action["params"]["timeout"]
                    # result = check_position_status_client(goal)
                    self.checkPositionClient.send_goal(goal)
                    self._missionStep = MissionStep.WAIT_RESULT
                elif action["type"] == "charging":
                    msg = String()
                    msg.data = "CHARGE,Charge,%s"%self.current_action
                    self.test_docking_pub.publish(msg)
                    mylogdebug("Publish: %s"%msg.data)
                    self._missionStep = MissionStep.WAIT_RESULT
                elif action["type"] == "move":
                    msg = String()
                    msg.data = json.dumps(action["params"])
                    # print(ype(msg.data))
                    mylogdebug("Publish to single_path")
                    self.single_path_pub.publish(msg)
                    self._missionStep = MissionStep.WAIT_RESULT
                    #TODO: coding here
                elif action["type"] == "pickup":
                    msg = String()
                    pos = action["params"]["marker_pos"]
                    msg.data = "PICKUP,Pickup,%s"%pos
                    # msg.data = "PICKUP,Pickup,%s"%self.current_action
                    self.test_docking_pub.publish(msg)
                    self._missionStep = MissionStep.WAIT_RESULT
                elif action["type"] == "droping":
                    msg = String()
                    pos = action["params"]["marker_pos"]
                    msg.data = "DROPING,Droping,%s"%pos
                    # msg.data = "DROPING,Droping,%s"%self.current_action
                    self.test_docking_pub.publish(msg)
                    self._missionStep = MissionStep.WAIT_RESULT
                elif action["type"] == "safety_setting":
                    print("safety setting")
                    try:
                        self.safety_setting(**action["params"])
                    except Exception as e:
                        mylogerr("Safety setting fail: %s"%e)
                        self.action_queue.queue.clear()
                        success = False
                        break
                    else:
                        myloginfo("Safety setting done!")
                        self._missionStep = MissionStep.GET_ACTION

                else:
                    mylogerr("Action type: %s not true value"%self.current_action)
                    # self._missionStep = MissionStep.WAITING
                    break

            # Buoc 3: Cho ket qua cua tung action
            elif self._missionStep == MissionStep.WAIT_RESULT:

                _actionResult = self._actionResult
                if _actionResult == ActionResult.SUCCEEDED:
                    self._missionStep = MissionStep.GET_ACTION
                    myloginfo("Action %s done!"%self.current_action)
                elif _actionResult == ActionResult.ERROR:
                    mylogerr("Action: %s Error. Canceling mission: %s!"%(self.current_action, self.current_mission))
                    self.mutex.acquire()
                    self.action_queue.queue.clear()
                    self.mutex.release()
                    self._missionStep = MissionStep.GET_ACTION
                elif _actionResult == ActionResult.RETRY:
                    myloginfo("Retry action: %s"%self.current_action)
                    self._missionStep = MissionStep.EXCUTIVE
                elif _actionResult == ActionResult.FAIL:
                    mylogerr("Action %s FAIL"%self.current_action)

                self._actionResult = ActionResult.PENDING

            rospy.Rate(50).sleep()

        if success:
            myloginfo("Mission: %s succeeded!"%self.current_mission)
            self._as.set_succeeded()

    def checkPosResultCB(self, msg):
        _actionResult = msg.result.result
        if _actionResult == True:
            self._actionResult = ActionResult.SUCCEEDED
        else:
            self._actionResult = ActionResult.ERROR

    def chargingResultCB(self, msg):
        _chargingResult = msg.status.status
        mylogdebug("ChargingResult: status: %i"%_chargingResult)
        if (self.current_action_type == "charging") and (_chargingResult == GoalStatus.SUCCEEDED):
            self._actionResult = ActionResult.SUCCEEDED
        # elif _chargingResult == GoalStatus.PREEMPTED:
        #     self._actionResult = ActionResult.ERROR
        # TODO: Bo sung phan ung cua action sac

    def followWaypointsResultCB(self, msg):
        _followWaypointsResult = msg.status.status
        mylogdebug("FollowWaypointsResult: status: %i"%_followWaypointsResult)
        if (self.current_action_type == "move") & (_followWaypointsResult == GoalStatus.SUCCEEDED):
            self._actionResult = ActionResult.SUCCEEDED
        elif _followWaypointsResult == GoalStatus.RECALLED:
            self._actionResult = ActionResult.RETRY

    def autoCartCB(self, msg):
        _autoCart = msg.status.status
        mylogdebug("CartActionResult: status: %i"%_autoCart)
        if (self.current_action_type == "pickup") or (self.current_action_type == "droping"):
            if _autoCart == GoalStatus.SUCCEEDED:
                self._actionResult = ActionResult.SUCCEEDED
            elif _autoCart == GoalStatus.PREEMPTED:
                self._actionResult = ActionResult.PREEMPTED

    def missionCallback(self, msg):
        # For debug:
        if msg.data == "":
            mylogwarn("Clear action_queue")
            self.action_queue.queue.clear()
            return True

        mission_dict = json.loads(msg.data)
        # myloginfo("Mission group: %s"%mission_dict["group"])
        # myloginfo("Number of action: %i"%len(mission_dict["actions"]))
        # mylogdebug("Mission dict: %s"%json.dumps(mission_dict, indent=2))
        self.current_mission = mission_dict["name"]
        myloginfo("Received Mission name: %s"%self.current_mission)
        actions = mission_dict["actions"]
        self.mutex.acquire()
        if self.action_queue.empty():
            for action in actions:
                myloginfo("Add action: %s"%(action["name"]))
                # do_action = check_position_status_client()
                # myloginfo(do_action)
                self.action_queue.put(action)
            self._missionStep = MissionStep.WAITING
        else:
            rospy.logwarn("Mission is proccessing, cannot input another")
        self.mutex.release()

    def safety_setting(self, **kwargs):
        """safety_setting: get footprint by name, safety_job to change in scan_safety config

        Raises:
            Exception: "Could not set footprint" when footprint==None
            Exception: "Update footprint fail..." when update_configuration fail
        """
        footprint = None

        if not self.safety_recfg_enable:
            raise Exception("Update footprint fail: could not init recfg client")
        new_cfg = dict()
        for key, value in kwargs.items():
            if (value != None and value != ''):
                if key == "footprint":
                    try:
                        # footprint_ = self.missiondb["footprint"][value]
                        footprint_ = value
                        new_cfg.update(footprint=footprint_)
                    except:
                        rospy.logerr("Footprint name: %s could not found in mission db")
                if key == "safety_job":
                    new_job = value
                    new_cfg.update(current_job=value)
        # if footprint == None:
            # raise Exception("Could not set footprint")
        rospy.loginfo("Scan safety new config: %s"%json.dumps(new_cfg, indent=2))
        try:
            self.safety_recfg_client.update_configuration(new_cfg)
        except rospy.ROSException as e:
            raise Exception("Update footprint fail: %s"%e)


if __name__ == "__main__":
    rospy.init_node("mission_server", log_level=rospy.DEBUG)
    missionManager = MissionManager(rospy.get_name())
    rospy.spin()

