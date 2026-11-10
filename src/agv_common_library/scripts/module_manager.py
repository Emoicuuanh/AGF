#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys

python3 = True if sys.hexversion > 0x03000000 else False
import rospy
import json
import threading
from std_stamped_msgs.msg import StringStamped, EmptyStamped
from common_function import EnumString


class ModuleStatus(EnumString):
    WAITING = 0
    RUNNING = 1
    PAUSED = 2
    ERROR = 3


class ModuleServer:
    def __init__(self, name):
        # Ros
        self.module_status_pub = rospy.Publisher(
            name + "/module_status", StringStamped, queue_size=5
        )
        rospy.Subscriber(
            name + "/run_pause_req", StringStamped, self.run_pause_req_cb
        )
        rospy.Subscriber(
            name + "/reset_error", EmptyStamped, self.reset_error_cb
        )
        rospy.Subscriber(
            "/request_start_mission",
            StringStamped,
            self.request_start_mission_cb,
        )
        # Variable
        self.name = name
        self.error_code = ""
        self.reset_action_req = False
        self.reset_error_request = False
        self.pause_req_by_server = False
        self.resume_req = False
        self.pause_req = False
        self.action_running = False
        self.module_state = ""
        self.module_status = ModuleStatus.WAITING

    def request_start_mission_cb(self, msg):
        if msg.data == "STOP":
            self.reset_action_req = True
            rospy.loginfo("STOP MISSION")

    def run_pause_req_cb(self, msg):
        if msg.data == "RUN":
            rospy.loginfo("Resume request")
            self.resume_req = True
            self.pause_req = False
        elif msg.data == "PAUSE":
            rospy.loginfo("Pause request")
            self.resume_req = False
            self.pause_req = True

    def reset_error_cb(self, msg):
        self.reset_error_request = True

    def reset_flag(self):
        self.pause_req = False
        self.resume_req = False
        self.reset_action_req = False
        self.reset_error_request = False
        self.error_code = ""

    def send_feedback(self, action, msg):
        self._feedback.data = msg
        action.publish_feedback(self._feedback)


class ModuleClient:
    def __init__(self, name, param_dict={}):
        rospy.loginfo("Init ModuleClient for: {}".format(name))
        # Ros
        if not "status_topic" in param_dict:
            rospy.Subscriber(
                name + "/module_status", StringStamped, self.module_status_cb
            )
        else:
            [msg_ns, msg_type] = param_dict["status_msg"].split("/", 1)
            cmd = "from {}.msg import {}".format(msg_ns, msg_type)
            exec(cmd)
            cmd = 'rospy.Subscriber("{}", {}, self.module_status_cb)'.format(
                param_dict["status_topic"], msg_type
            )
            exec(cmd)

        # Variable
        self.name = name
        self.display_name = param_dict["display_name"]
        self.parse_json = (
            param_dict["parse_json"] if "parse_json" in param_dict else True
        )
        self.handle_when_sim = (
            param_dict["handle_when_sim"]
            if "handle_when_sim" in param_dict
            else True
        )
        self.timeout = param_dict["timeout"] if "timeout" in param_dict else 2.0
        self.pause_if_error = (
            param_dict["pause_if_error"]
            if "pause_if_error" in param_dict
            else True
        )
        self.error_list = (
            param_dict["error_list"] if "error_list" in param_dict else []
        )
        self.state_list = (
            param_dict["state_list"] if "state_list" in param_dict else []
        )
        self.check_script = (
            param_dict["check_script"] if "check_script" in param_dict else ""
        )

        self.last_module_status = rospy.get_time()
        self.module_alive = False
        self.module_status_msg = None
        self.map_save_thread = threading.Thread(target=self.loop, args=())
        self.map_save_thread.start()
        self.module_status_dict = {}
        self.special_matching_error = ""
        self.special_matching_state = ""
        self.special_error_led = ""
        self.special_state_led = ""
        self.special_sound = ""
        self.error_code = ""

    def module_status_cb(self, msg):
        self.module_status_msg = msg
        self.last_module_status = rospy.get_time()
        if not self.parse_json:
            return
        self.special_matching_error = ""
        self.special_matching_state = ""
        self.special_error_led = ""
        self.special_state_led = ""
        self.special_sound = ""
        try:
            self.module_status_dict = json.loads(msg.data)
            # Normal error
            if "error_code" in self.module_status_dict:
                self.error_code = self.module_status_dict["error_code"]
            # Special error
            if python3:
                if "error_code" in self.module_status_dict and len(
                    self.error_list
                ):
                    for i in self.error_list:
                        if (
                            list(i.keys())[0]
                            in self.module_status_dict["error_code"]
                        ):
                            self.special_matching_error = list(i.values())[0][
                                "detail_status"
                            ]
                            if "led_status" in list(i.values())[0]:
                                self.special_error_led = list(i.values())[0][
                                    "led_status"
                                ]
                            return
                if "state" in self.module_status_dict and len(self.state_list):
                    for i in self.state_list:
                        if (
                            list(i.keys())[0]
                            in self.module_status_dict["state"]
                        ):
                            self.special_matching_state = list(i.values())[0][
                                "detail_status"
                            ]
                            if "led_status" in list(i.values())[0]:
                                self.special_state_led = list(i.values())[0][
                                    "led_status"
                                ]
                            if "sound_status" in list(i.values())[0]:
                                self.special_sound = list(i.values())[0][
                                    "sound_status"
                                ]
                            return
            else:
                if "error_code" in self.module_status_dict and len(
                    self.error_list
                ):
                    for i in self.error_list:
                        if i.keys()[0] in self.module_status_dict["error_code"]:
                            self.special_matching_error = i.values()[0][
                                "detail_status"
                            ]
                            if "led_status" in i.values()[0]:
                                self.special_error_led = i.values()[0][
                                    "led_status"
                                ]
                            return
                if "state" in self.module_status_dict and len(self.state_list):
                    for i in self.state_list:
                        if i.keys()[0] in self.module_status_dict["state"]:
                            self.special_matching_state = i.values()[0][
                                "detail_status"
                            ]
                            if "led_status" in i.values()[0]:
                                self.special_state_led = i.values()[0][
                                    "led_status"
                                ]
                            if "sound_status" in i.values()[0]:
                                self.special_sound = i.values()[0][
                                    "sound_status"
                                ]
                            return
        except Exception as e:
            rospy.logerr("module_status_cb: {}".format(e))

    def loop(self):
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.check_script == "":
                if rospy.get_time() - self.last_module_status > self.timeout:
                    self.module_alive = False
                else:
                    self.module_alive = True
            else:
                # Ex: msg.ranges[0] == -1
                if self.module_status_msg != None:
                    cmd = "self.module_alive = {}".format(self.check_script)
                    exec(cmd)
                if rospy.get_time() - self.last_module_status > self.timeout:
                    self.module_alive = False
            r.sleep()
