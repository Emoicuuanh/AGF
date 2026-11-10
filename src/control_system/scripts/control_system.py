#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys

python3 = True if sys.hexversion > 0x03000000 else False
import json

if python3:
    import urllib.parse
else:
    from urlparse import urlparse
import yaml
import threading
import requests
import rospy
import rospkg
import tf
import copy
from pathlib import Path
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    PoseArray,
    PoseWithCovarianceStamped,
)
from std_msgs.msg import Int8, Int16, String
from std_stamped_msgs.msg import (
    StringStamped,
    Int8Stamped,
    StringFeedback,
    StringResult,
    StringAction,
    StringActionGoal,
    EmptyStamped,
    Float32Stamped,
)
from std_stamped_msgs.srv import StringService, StringServiceResponse
from os.path import expanduser
from safety_msgs.msg import SafetyStatus

HOME = expanduser("~")

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

slam_manager_dir = os.path.join(
    rospkg.RosPack().get_path("slam_manager"), "scripts"
)
if not os.path.isdir(slam_manager_dir):
    slam_manager_dir = os.path.join(
        rospkg.RosPack().get_path("slam_manager"), "release"
    )
sys.path.insert(0, slam_manager_dir)
from slam_manager import MainState as SlamState

from mongodb import mongodb, LogLevel, MissionStatus
from module_manager import ModuleServer, ModuleClient, ModuleStatus
from common_function import (
    EnumString,
    lockup_pose,
    offset_pose_x,
    distance_two_pose,
    delta_angle,
    dict_to_obj,
    merge_two_dicts,
    print_debug,
    print_warn,
    print_error,
    print_info,
    get_line_info,
    YamlDumper,
    obj_to_dict,
    pose_dict_template,
    get_ubuntu_serial_hash,
)


class RobotStatus(EnumString):
    NONE = -1
    PAUSED = 0
    WAITING = 1
    RUNNING = 2
    EMG = 4
    ERROR = 5
    WAITING_INIT_POSE = 6


class DetailStatus(EnumString):
    NONE = -1
    ALL_OK = 0
    AUTO_CHARGING = 1
    MANUAL_CHARGING = 2
    IO_BOARD_ERROR = 5
    LOAD_MAP_ERROR = 6
    MAPPING_ERROR = 7
    FATAL_ERROR = 8
    GENERAL_ERROR = 9
    SAFETY_STOP = 10
    ODOMETRY_ERROR = 11
    IO_BOARD_DISCONNECT = 12
    BATTERY_LOW = 13
    BATTERY_EMPTY = 14
    MOTOR_OFF = 15
    SAFETY_DISABLED = 38
    MOTOR_OVER_CURRENT = 23
    MOTOR_CONTROL_ERROR = 24


class LedStatus(EnumString):
    STARTING = 0
    SHUTTING_DOWN = 1
    MANUAL = 2
    MAPPING = 3
    WAIT_RESPOND = 4
    PAUSED = 5
    EMG = 6
    SAFETY_STOP = 7
    GENERAL_ERROR = 10
    WAITING = 14
    FATAL_ERROR = 15
    BATTERY_LOW = 16
    BATTERY_EMPTY = 17
    MOTOR_OFF = 18
    RUNNING_NORMAL = 19
    SAFETY_DISABLED = 20
    WAITING_INIT_POSE = 21
    CHARGING_READY = 22
    MOTOR_OVER_CURRENT = 23
    MOTOR_CONTROL_ERROR = 24


class RobotMode(EnumString):
    NONE = -1
    MANUAL = 0
    AUTO = 1
    MAPPING = 2
    WAIT_RESPOND = 3


class MainState(EnumString):
    NONE = -1
    INIT = 0
    WAIT_MANUAL = 1
    MANUAL = 2
    WAIT_AUTO = 3
    AUTO = 4
    WAIT_MAPPING = 5
    MAPPING = 6
    LOADING_MAP = 7
    INIT_LOAD_MAP = 8
    LOAD_MAP_ERROR = 9
    MAPPING_ERROR = 10
    INIT_MAPPING = 11


class CurrentMap:
    map_dir = ""
    map_file = ""


SENSOR_DEACTIVATE = 0
SENSOR_ACTIVATE = 1
SWITCH_MANUAL_MODE = 0
SWITCH_AUTO_MODE = 1


class ControlSystem(object):
    def __init__(self, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        # Initial
        if not self.load_config():
            return
        rospy.on_shutdown(self.shutdown)
        # Publisher
        self.pub_init_server = rospy.Publisher(
            "/init_server", EmptyStamped, queue_size=1
        )
        self.pub_battery = rospy.Publisher(
            "/battery", Int8Stamped, queue_size=10
        )
        self.pub_robot_mode = rospy.Publisher(
            "/robot_mode", StringStamped, queue_size=10
        )
        self.pub_current_map = rospy.Publisher(
            "/current_map", StringStamped, queue_size=10
        )
        self.pub_robot_status = rospy.Publisher(
            "/robot_status", StringStamped, queue_size=10
        )
        self.pub_mission_control = rospy.Publisher(
            "/mission_control", StringStamped, queue_size=10
        )
        self.pub_mission_run_pause = rospy.Publisher(
            "/mission_manager/run_pause_req", StringStamped, queue_size=10
        )
        self.pub_mapping_req = rospy.Publisher(
            "/mapping_request", StringStamped, queue_size=5
        )
        self.pub_map_load_req = rospy.Publisher(
            "/map_load_request", StringStamped, queue_size=5, latch=True
        )
        self.pub_map_save_req = rospy.Publisher(
            "/map_save_request", StringStamped, queue_size=5
        )
        self.pub_temp_map_save_req = rospy.Publisher(
            "/temp_savemap_request", StringStamped, queue_size=5
        )
        self.pub_led_status = rospy.Publisher(
            "/led_status", StringStamped, queue_size=5, latch=True
        )
        self.pub_trigger = rospy.Publisher(
            "/trigger_complex", StringStamped, queue_size=5
        )
        self.pub_mission_reset_error = rospy.Publisher(
            "/mission_manager/reset_error", EmptyStamped, queue_size=5
        )
        self.pub_stop_moving = rospy.Publisher(
            "/stop_moving", EmptyStamped, queue_size=5
        )
        self.pub_request_start_mission = rospy.Publisher(
            "/request_start_mission", StringStamped, queue_size=5
        )
        self.pub_mission_fr_server = rospy.Publisher(
            "/mission_from_server", StringStamped, queue_size=5
        )
        self.pub_mission_fr_server_latch = rospy.Publisher(
            "/mission_from_server_latch",
            StringStamped,
            queue_size=5,
            latch=True,
        )
        self.pub_request_sound = rospy.Publisher(
            "/request_sound", StringStamped, queue_size=5
        )
        # Common module

        # Optional module
        for i in self.module_config:
            client_module = None
            if python3:
                client_module = ModuleClient(
                    list(i.keys())[0], list(i.values())[0]
                )
                if list(i.keys())[0] == "mission_manager":
                    self.mission_manager_module = client_module
                elif list(i.keys())[0] == "slam_manager":
                    self.slam_manager_module = client_module
            else:
                client_module = ModuleClient(i.keys()[0], i.values()[0])
                if i.keys()[0] == "mission_manager":
                    self.mission_manager_module = client_module
                elif i.keys()[0] == "slam_manager":
                    self.slam_manager_module = client_module
            self.module_list.append(client_module)

        # Subscriber
        rospy.Subscriber(
            "/init_server",
            EmptyStamped,
            self.init_server_cb,
        )
        rospy.Subscriber(
            "/mission_manager/module_status",
            StringStamped,
            self.mission_status_cb,
        )
        # rospy.Subscriber(
        #     "/moving_control/goal",
        #     StringActionGoal,
        #     self.moving_control_action_cb,
        # )
        rospy.Subscriber(
            "/current_goal_monving_control",
            PoseStamped,
            self.current_moving_control_goal_cb,
        )
        if self.option_config["use_followline"] and not self.simulation:
            rospy.Subscriber(
                "/followline_action/goal",
                FollowLineActionGoal,
                self.followline_action_cb,
            )
        rospy.Subscriber(
            "/error_control_motor_right", Int16, self.right_vel_error_cb
        )
        rospy.Subscriber(
            "/error_control_motor_left", Int16, self.left_vel_error_cb
        )
        rospy.Subscriber("/request_mode", StringStamped, self.request_mode_cb)
        rospy.Subscriber(
            "/request_savemap", StringStamped, self.request_savemap_cb
        )
        rospy.Subscriber(
            "/request_temp_savemap", StringStamped, self.request_temp_savemap_cb
        )
        rospy.Subscriber(
            "/request_change_map", StringStamped, self.request_change_map_cb
        )
        rospy.Subscriber(
            "/request_reload_map", StringStamped, self.request_reload_map_cb
        )
        rospy.Subscriber(
            "/request_run_stop", StringStamped, self.request_run_stop_cb
        )
        rospy.Subscriber(
            "/request_start_mission",
            StringStamped,
            self.request_start_mission_cb,
        )
        rospy.Subscriber("/standard_io", StringStamped, self.standard_io_cb)
        rospy.Subscriber("/arduino_error", StringStamped, self.arduino_error_cb)
        rospy.Subscriber("/reset_error", EmptyStamped, self.reset_error_cb)
        rospy.Subscriber("/fake_battery", Int8, self.fake_battery_cb)
        rospy.Subscriber(
            "/arduino_driver/float_param/battery_percent",
            Float32Stamped,
            self.real_battery_cb,
        )
        rospy.Subscriber(
            "/trigger_mission_from_server",
            StringStamped,
            self.trigger_mission_from_server_cb,
        )
        rospy.Subscriber(
            "/initialpose", PoseWithCovarianceStamped, self.initialpose_cb
        )
        rospy.Subscriber("/safety_status", SafetyStatus, self.safety_status_cb)
        rospy.Subscriber(
            "/moving_control/module_status",
            StringStamped,
            self.moving_control_module_status_cb,
        )
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        # Service
        # rospy.Service("run_pause_request", StringService, self.handle_run_pause)
        # Server
        self.init_server()
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
        self.simulation = kwargs["simulation"]
        print_debug("simulation: {}".format(self.simulation))
        self.simulation_str = "true" if self.simulation else "false"
        # TF
        self.tf_listener = tf.TransformListener()
        self.odom_frame = "odom"
        self.robot_frame = "base_footprint"
        self.map_frame = "map"
        self.current_pose = None
        # Config
        self.map_requesting = ""
        self.current_map_file = kwargs["current_map_file"]
        self.robot_config_file = kwargs["robot_config_file"]
        self.server_config_file = kwargs["robot_define"]
        self.current_map_cfg = CurrentMap()
        self.current_map_cfg.map_file = "mkac"
        self.current_map_cfg.map_dir = HOME + "/tmp/ros/maps/"
        # State
        self.slam_state = SlamState.NONE
        self.mission_status = ModuleStatus.WAITING
        self.running_mission_id = ""
        self.last_done_mission_id = ""
        self.led_status = LedStatus.STARTING.toString()
        self.main_state = MainState.INIT
        self.prev_state = MainState.NONE
        self.robot_mode = RobotMode.AUTO
        self.robot_mode_req = RobotMode.NONE
        self.robot_status = RobotStatus.NONE
        self.detail_status = DetailStatus.ALL_OK.toString()
        self.sound_status = None
        self.std_io_status = {"emg_button": False}
        self.status_dict = {
            "status": RobotStatus.NONE.toString(),
            "detail": DetailStatus.NONE.toString(),
        }
        self.module_list = []
        self.error_code = ""
        self.arduino_error_msg = StringStamped()
        self.arduino_error_cnt = 0
        self.load_map_cnt = 0
        self.mapping_cnt = 0
        # Flag
        self.save_map_request = False
        self.reset_error_request = False
        self.mission_from_server_req = False
        self.init_pose_received = False
        self.battery_percent = 90
        self.last_battery_fake = rospy.get_time()
        self.last_std_io_msg = rospy.get_time()
        self.last_odom_msg = rospy.get_time()
        # Button
        self.emg_button = SENSOR_DEACTIVATE
        self.start_1_button = SENSOR_DEACTIVATE
        self.start_2_button = SENSOR_DEACTIVATE
        self.stop_1_button = SENSOR_DEACTIVATE
        self.stop_2_button = SENSOR_DEACTIVATE
        self.auto_manual_sw = SENSOR_DEACTIVATE
        self.motor_enable_sw = SENSOR_DEACTIVATE
        # Safety
        self.last_safety_time = rospy.get_time()
        self.is_safety = False
        # Ros params
        self.pose_initiated = rospy.get_param("/pose_initiated", False)
        # Other
        self.moving_control_status_dict = {"status": "NONE"}
        # Server
        self.agv_db_object_id = None
        # Bumper
        self.bumper = SENSOR_DEACTIVATE
        # Motor
        self.control_motor_left_error = False
        self.control_motor_right_error = False
        self.motor_left_over_ampe = False
        self.motor_right_over_ampe = False
        # Pose
        self.tf_listener = tf.TransformListener()
        self.followline_goal = ""
        self.moving_control_goal = ""
        self.pause_by_cross_function = False
        self.current_mission_name = ""
        # Database
        db_address = rospy.get_param("/mongodb_address")
        # print_debug(db_address)
        self.db = mongodb(db_address)

    def init_server(self):
        if self.server_config == None:
            rospy.loginfo_throttle(30, "Server was not configured!")
            return
        self.api_url = self.server_config["server_address"]
        self.header = {
            "X-Parse-Application-Id": "APPLICATION_ID",
            "X-Parse-Master-Key": "YOUR_MASTER_KEY",
            "Content-Type": "application/json",
        }

        # Get AGV object ID in Database
        param = urllib.parse.urlencode(
            {"where": json.dumps({"name": self.server_config["agv_name"]})}
        )
        print_debug(
            "AGV name for system: {}".format(self.server_config["agv_name"])
        )
        try:
            agv_db_obj = requests.get(
                self.api_url + "classes/agv", params=param, headers=self.header
            )
            print_debug("Request server result: n{}".format(agv_db_obj))
            # print_debug(json.dumps(json.loads(agv_db_obj.text), indent=2))
            self.agv_db_object_id = json.loads(agv_db_obj.text)["results"][0][
                "objectId"
            ]
            print_debug("AGV obj db id: {}".format(self.agv_db_object_id))
        except Exception as e:
            self.agv_db_object_id = None
            rospy.logerr(e)

    def shutdown(self):
        # Kill all node
        print("Shutdown: {}".format(rospy.get_name()))

    """
    ######## ##     ## ##    ##  ######  ######## ####  #######  ##    ##
    ##       ##     ## ###   ## ##    ##    ##     ##  ##     ## ###   ##
    ##       ##     ## ####  ## ##          ##     ##  ##     ## ####  ##
    ######   ##     ## ## ## ## ##          ##     ##  ##     ## ## ## ##
    ##       ##     ## ##  #### ##          ##     ##  ##     ## ##  ####
    ##       ##     ## ##   ### ##    ##    ##     ##  ##     ## ##   ###
    ##        #######  ##    ##  ######     ##    ####  #######  ##    ##
    """

    def is_near_intersect_area(self, dist_check=2):
        try:
            if self.current_mission_name == "move":
                if self.moving_control_goal != "":
                    # for i in range(len(self.moving_control_goal)):
                    #     _pose = self.moving_control_goal[i]["position"]
                    #     goal_check = dict_to_obj(_pose, Pose())
                    #     distance = distance_two_pose(
                    #         self.current_pose, goal_check
                    #     )
                    #     if distance < dist_check:
                    #         return True
                    # return False
                    distance = distance_two_pose(
                        self.current_pose, self.moving_control_goal
                    )
                    if distance < dist_check:
                        return True
                    return False

            elif (
                self.current_mission_name == "hub"
                or self.current_mission_name == "matehand"
            ):
                if self.followline_goal != "":
                    distance = distance_two_pose(
                        self.current_pose, self.followline_goal
                    )
                    if distance < dist_check:
                        return True
                    else:
                        return False
            return False
        except Exception as e:
            rospy.logerr("error check near intersection: {}".format(e))
            return False

    def dump_config(self):
        # TODO: Only dump when load map successful
        with open(self.current_map_file, "w") as file:
            documents = yaml.dump(self.current_map_cfg, file)

    def load_config(self):
        try:
            # Current map
            print_debug(
                "Check current_map_file: {}".format(self.current_map_file)
            )
            if os.path.exists(self.current_map_file):
                with open(self.current_map_file) as file:
                    config_dict = yaml.load(file, Loader=yaml.Loader)
                    if config_dict == None:
                        rospy.logwarn("Current map file empty")
                        self.dump_config()
                    else:
                        config_dict.map_dir = (
                            self.current_map_cfg.map_dir
                        )  # Overwrite map_dir for different user name
                        self.current_map_cfg = config_dict
                        self.dump_config()
                        print_info("Current map file:")
                        print(
                            yaml.dump(
                                self.current_map_cfg,
                                Dumper=YamlDumper,
                                default_flow_style=False,
                            )
                        )
            else:
                print_warn(
                    "Check current_map_file not exist. Write default value"
                )
                self.dump_config()
            # Robot config
            if os.path.exists(self.robot_config_file):
                with open(self.robot_config_file) as file:
                    config_dict = yaml.load(file, Loader=yaml.Loader)
                    if config_dict == None:
                        rospy.logerr("Robot config file empty")
                    else:
                        self.robot_config = config_dict
                        self.battery_config = self.robot_config["battery"]
                        self.module_config = self.robot_config["module_list"]
                        self.option_config = self.robot_config["option"]
                        if "motor" in self.robot_config:
                            self.motor_config = self.robot_config["motor"]
                            rospy.Subscriber(
                                "/ampe_left",
                                Float32Stamped,
                                self.ampe_motor_left_cb,
                            )
                            rospy.Subscriber(
                                "/ampe_right",
                                Float32Stamped,
                                self.ampe_motor_right_cb,
                            )
                        if "use_followline" not in self.option_config:
                            self.option_config["use_followline"] = True
                            # TODO:
                            from follow_line.msg import FollowLineActionGoal
                        if "check_init_pose" not in self.option_config:
                            self.option_config["check_init_pose"] = True
                        if "use_slam" not in self.option_config:
                            self.option_config["use_slam"] = True
                        if "use_bumper" not in self.option_config:
                            self.option_config["use_bumper"] = False

                        # self.server_config = (
                        #     self.robot_config["server"]
                        #     if "server" in self.robot_config
                        #     else None
                        # )
                        print_info("Robot config file:")
                        print(
                            yaml.dump(
                                self.robot_config,
                                Dumper=YamlDumper,
                                default_flow_style=False,
                            )
                        )
            # TODO: add file robot config in folder robot config to define agv name, server
            # Server config
            if os.path.exists(self.server_config_file):
                with open(self.server_config_file) as file:
                    self.server_config = yaml.load(file, Loader=yaml.Loader)
                    print_info("Server config file:")
                    print(
                        yaml.dump(
                            self.server_config,
                            Dumper=YamlDumper,
                            default_flow_style=False,
                        )
                    )
                    if "server_address" not in self.server_config:
                        self.server_config = None
        except Exception as e:
            rospy.logerr("Load_config: {}".format(e))
            return False
        return True

    def set_pose_init(self, value):
        self.pose_initiated = value
        rospy.set_param("/pose_initiated", value)

    def pause_robot(self):
        self.pause_by_cross_function = False
        self.pub_mission_run_pause.publish(
            StringStamped(stamp=rospy.Time.now(), data="PAUSE")
        )

    def pause_robot_by_server(self):
        rospy.logwarn("Pause by cross function")
        self.pause_by_cross_function = True
        self.pub_mission_run_pause.publish(
            StringStamped(stamp=rospy.Time.now(), data="PAUSE_BY_SERVER")
        )

    def stop_moving(self):
        self.pub_stop_moving.publish(EmptyStamped(stamp=rospy.Time.now()))

    def resume_running(self):
        self.pub_mission_run_pause.publish(
            StringStamped(stamp=rospy.Time.now(), data="RUN")
        )

    def resume_running_by_server(self):
        rospy.logwarn("Resume running by cross function")
        self.pub_mission_run_pause.publish(
            StringStamped(stamp=rospy.Time.now(), data="RUN")
        )

    def load_map(self, map_file, abs_path=True, reload_map=False):
        msg = StringStamped()
        msg.stamp = rospy.Time.now()
        if abs_path:
            msg.data = self.current_map_cfg.map_dir + map_file
        else:
            msg.data = map_file
        rospy.logwarn(msg.data)
        if not reload_map:
            self.pub_map_load_req.publish(msg)
            rospy.loginfo("load_map: {}".format(msg.data))
        else:
            self.pub_map_load_req.publish(msg)
            rospy.loginfo("reload_map: {}".format(msg.data))

    def check_map(self, map_file):
        # Service
        rospy.loginfo("Check map file: {}".format(map_file))
        check_map_srv = rospy.ServiceProxy("check_map", StringService)
        try:
            result = check_map_srv(map_file)
            if result.respond == "OK":
                return True
            else:
                return False
        except Exception as e:
            rospy.logerr("Check map error: {}".format(e))
            return False

    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def init_server_cb(self, msg):
        if self.server_config != None:  # Retry init server
            self.init_server()
            if self.agv_db_object_id != None:
                rospy.logwarn("Re-init server")

    def mission_status_cb(self, msg):
        data_dict = json.loads(msg.data)
        self.current_mission_name = data_dict["current_mission_name"]

    # def moving_control_action_cb(self, msg):
    #     data_dict = json.loads(msg.goal.data)
    #     self.moving_control_goal = data_dict["waypoints"]

    def current_moving_control_goal_cb(self, msg):
        self.moving_control_goal = msg.pose

    def followline_action_cb(self, msg):
        self.followline_goal = msg.goal.target_pose.pose

    def ampe_motor_left_cb(self, msg):
        if msg.data > self.motor_config["max_ampe"]:
            self.motor_left_over_ampe = True
        else:
            self.motor_left_over_ampe = False

    def ampe_motor_right_cb(self, msg):
        if msg.data > self.motor_config["max_ampe"]:
            self.motor_right_over_ampe = True
        else:
            self.motor_right_over_ampe = False

    def left_vel_error_cb(self, msg):
        if msg.data == 1:
            self.control_motor_left_error = True
        else:
            self.control_motor_left_error = False

    def right_vel_error_cb(self, msg):
        if msg.data == 1:
            self.control_motor_right_error = True
        else:
            self.control_motor_right_error = False

    def request_mode_cb(self, msg):
        self.robot_mode_req = RobotMode[msg.data]
        rospy.loginfo("Request mode: {}".format(self.robot_mode_req.toString()))
        if self.robot_mode_req == RobotMode.MAPPING:
            self.mapping_cnt += 1
            print_debug("Mapping request time: {}".format(self.mapping_cnt))

    def request_savemap_cb(self, msg):
        if not self.option_config["use_slam"]:
            return
        rospy.loginfo("Request save map: {}".format(msg.data))
        try:
            self.map_requesting = json.loads(msg.data)["name"]
            map_name = self.current_map_cfg.map_dir + self.map_requesting
            msg.data = map_name
            self.pub_map_save_req.publish(msg)
            self.save_map_request = True
        except Exception as e:
            rospy.logerr("request_savemap_cb: {}".format(e))

    def request_temp_savemap_cb(self, msg):
        if not self.option_config["use_slam"]:
            return
        rospy.loginfo("Temp save map request: {}".format(msg.data))
        try:
            self.map_requesting = json.loads(msg.data)["name"]
            map_name = self.current_map_cfg.map_dir + self.map_requesting
            msg.data = map_name
            self.pub_temp_map_save_req.publish(msg)
        except Exception as e:
            rospy.logerr("request_temp_savemap_cb: {}".format(e))

    def request_change_map_cb(self, msg):
        if not self.option_config["use_slam"]:
            return
        rospy.loginfo("Request change map: {}".format(msg.data))
        file_check = self.current_map_cfg.map_dir + msg.data
        if self.check_map(file_check):
            self.main_state = MainState.INIT_LOAD_MAP
            self.current_map_cfg.map_file = msg.data
            self.dump_config()
        else:
            rospy.logerr(
                "The requesting map is not exist: {}".format(file_check)
            )

    def request_reload_map_cb(self, msg):
        if not self.option_config["use_slam"]:
            return
        rospy.loginfo("Request reload map: {}".format(msg.data))
        file_check = self.current_map_cfg.map_dir + msg.data
        if self.check_map(file_check):
            self.main_state = MainState.INIT_LOAD_MAP
        else:
            rospy.logerr(
                "The requesting map is not exist: {}".format(file_check)
            )

    def request_run_stop_cb(self, msg):
        # TODO: Change in app
        if msg.data == "STOP":
            msg.data = "PAUSE"
        self.pub_mission_run_pause.publish(msg)

    def handle_run_pause(self, req):
        self.pub_mission_run_pause.publish(
            StringStamped(stamp=rospy.Time.now(), data=req.request)
        )
        return StringServiceResponse("OK")

    def trigger_mission_from_server_cb(self, msg):
        self.mission_from_server_req = True

    def initialpose_cb(self, msg):
        self.init_pose_received = True

    def request_start_mission_cb(self, msg):
        if msg.data == "START":
            if self.robot_mode == RobotMode.AUTO:
                self.pub_mission_control.publish(msg)
        else:
            self.pub_mission_control.publish(msg)

    def standard_io_cb(self, msg):
        try:
            self.std_io_status = json.loads(msg.data)
            if "bumper" in self.std_io_status:
                self.bumper = self.std_io_status["bumper"]
            if "emg_button" in self.std_io_status:
                self.emg_button = self.std_io_status["emg_button"]
            if "start_1_button" in self.std_io_status:
                self.start_1_button = self.std_io_status["start_1_button"]
            if "start_2_button" in self.std_io_status:
                self.start_2_button = self.std_io_status["start_2_button"]
            if "stop_1_button" in self.std_io_status:
                self.stop_1_button = self.std_io_status["stop_1_button"]
            if "stop_2_button" in self.std_io_status:
                self.stop_2_button = self.std_io_status["stop_2_button"]
            if "auto_manual_sw" in self.std_io_status:
                self.auto_manual_sw = self.std_io_status["auto_manual_sw"]
            if "motor_enable_sw" in self.std_io_status:
                self.motor_enable_sw = self.std_io_status["motor_enable_sw"]
        except:
            rospy.logerr("standard_io error")
        self.last_std_io_msg = rospy.get_time()

    def odom_cb(self, msg):
        self.last_odom_msg = rospy.get_time()

    def arduino_error_cb(self, msg):
        self.arduino_error_msg = msg

    def reset_error_cb(self, msg):
        self.reset_error_request = True
        self.pub_mission_reset_error.publish(
            EmptyStamped(stamp=rospy.Time.now())
        )

    def fake_battery_cb(self, msg):
        self.battery_percent = msg.data
        self.last_battery_fake = rospy.get_time()

    def real_battery_cb(self, msg):
        # TODO: Get lowest value ultil reset
        if rospy.get_time() - self.last_battery_fake > 1.0:
            self.battery_percent = int(msg.data)

    def led_control_alive_cb(self, msg):
        self.last_led_alive = rospy.get_time()

    def safety_status_cb(self, msg):
        self.last_safety_time = rospy.get_time()
        safety_fields = list(msg.fields)  # [1, 2, 3, ...]
        if len(safety_fields) > 0:
            if safety_fields[0] == 1:
                self.is_safety = True
            else:
                self.is_safety = False
        else:
            self.is_safety = False

    def update_feedback(self):
        msg = StringStamped()
        msg.stamp = rospy.Time.now()
        # Publish robot_mode
        msg.data = self.robot_mode.toString()
        self.pub_robot_mode.publish(msg)
        # Publish robot_status
        self.status_dict["status"] = self.robot_status.toString()
        self.status_dict["state"] = self.main_state.toString()
        self.status_dict["mode"] = self.robot_mode.toString()
        self.status_dict["sound"] = (
            "OFF" if self.sound_status == "" else self.sound_status
        )
        self.status_dict["detail"] = self.detail_status
        self.status_dict["error_code"] = self.error_code  # TODO: Display in app
        msg.data = json.dumps(self.status_dict, indent=2)
        self.pub_robot_status.publish(msg)
        # Current map
        msg.data = self.current_map_cfg.map_file
        if self.option_config["use_slam"]:
            self.pub_current_map.publish(msg)
        msg = Int8Stamped()
        msg.data = self.battery_percent
        if msg.data > 100:
            msg.data = 100
        if msg.data < 0:
            msg.data = 0
        msg.stamp = rospy.Time.now()
        self.pub_battery.publish(msg)

    def update_error_code(self, module_disconnected="", msg=""):
        if module_disconnected != "":
            self.error_code = '"{}" module disconnected'.format(
                module_disconnected
            )
        elif msg != "":
            self.error_code = msg

    def clear_error_code(self):
        self.error_code = ""

    def moving_control_module_status_cb(self, msg):
        try:
            self.moving_control_status_dict = json.loads(msg.data)
        except Exception as e:
            rospy.logerr("moving_control_module_status_cb: {}".format(e))

    """
    ##        #######   #######  ########
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ##     ##
    ##       ##     ## ##     ## ########
    ##       ##     ## ##     ## ##
    ##       ##     ## ##     ## ##
    ########  #######   #######  ##
    """

    def check_condition_reset_err(self):
        if self.robot_mode_req == RobotMode.MAPPING:
            self.robot_mode_req = RobotMode.NONE
            self.main_state = MainState.INIT_MAPPING
        if self.slam_state == SlamState.LOCALIZING:
            self.main_state = MainState.MANUAL
            self.robot_mode_req = RobotMode.NONE
        if self.slam_state == SlamState.SWITCH_LOCALIZING:
            self.main_state = MainState.LOADING_MAP
        if self.slam_state == SlamState.SWITCH_MAPPING:
            self.main_state = MainState.WAIT_MAPPING
        if self.slam_state == SlamState.MAPPING:
            self.main_state = MainState.MAPPING
            self.robot_mode_req = RobotMode.NONE
        if self.robot_mode_req == RobotMode.MANUAL:
            self.robot_mode_req = RobotMode.NONE
            self.main_state = MainState.INIT_LOAD_MAP

    def loop(self):
        rospy.sleep(2.0)
        begin_load_map = rospy.get_time()
        last_pub_time = rospy.get_time()
        last_update_pose = rospy.get_time()
        begin_hold_stop_button = rospy.get_time()
        begin_load_map_error = rospy.get_time()
        begin_check_mode_change = rospy.get_time()
        begin_check_status_change = rospy.get_time()
        begin_request_mission = rospy.get_time()
        last_sound_time = rospy.get_time()
        begin_loop = rospy.get_time()
        trigger_button_time = rospy.get_time()
        trigger_button = ""
        first_check_mode = True
        pushed_stop_mission = False
        odom_ready = True
        prev_led_stt = None
        prev_sound_stt = None
        prev_auto_man_switch = None
        current_safety_job = ""
        pause_by_error = False
        last_robot_status = RobotStatus.NONE
        last_robot_mode = RobotMode.NONE
        last_start_1_button = SENSOR_DEACTIVATE
        last_start_2_button = SENSOR_DEACTIVATE
        last_stop_1_button = SENSOR_DEACTIVATE
        last_stop_2_button = SENSOR_DEACTIVATE
        last_module_disconnected = ""
        mission_queue_id = ""

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            if self.main_state != self.prev_state:
                rospy.loginfo(
                    "Main state: {} -> {}".format(
                        self.prev_state.toString(), self.main_state.toString()
                    )
                )
                self.prev_state = self.main_state

            self.detail_status = DetailStatus.ALL_OK.toString()

            # State
            if "state" in self.slam_manager_module.module_status_dict:
                self.slam_state = SlamState[
                    self.slam_manager_module.module_status_dict["state"]
                ]
            # Status
            if "status" in self.mission_manager_module.module_status_dict:
                self.mission_status = ModuleStatus[
                    self.mission_manager_module.module_status_dict["status"]
                ]
            # Running mission id
            if (
                "running_mission_id"
                in self.mission_manager_module.module_status_dict
            ):
                self.running_mission_id = (
                    self.mission_manager_module.module_status_dict[
                        "running_mission_id"
                    ]
                )
            if (
                "last_done_mission_id"
                in self.mission_manager_module.module_status_dict
            ):
                self.last_done_mission_id = (
                    self.mission_manager_module.module_status_dict[
                        "last_done_mission_id"
                    ]
                )
            # Safety job
            try:
                safety_disabled = False
                if (
                    "use_safety" in self.option_config
                    and self.option_config["use_safety"]
                ):
                    current_safety_job = json.loads(
                        rospy.get_param("/scan_safety_node_ng/job")
                    )
                    if len(current_safety_job["safety"]) == 0:
                        safety_disabled = True
            except:
                pass

            # Update current position
            temp_pose = None
            temp_pose = lockup_pose(
                self.tf_listener, self.map_frame, self.robot_frame
            )
            # print_info(temp_pose)
            if temp_pose != None:
                self.current_pose = temp_pose

            """
             ######  ########    ###    ######## ########
            ##    ##    ##      ## ##      ##    ##
            ##          ##     ##   ##     ##    ##
             ######     ##    ##     ##    ##    ######
                  ##    ##    #########    ##    ##
            ##    ##    ##    ##     ##    ##    ##
             ######     ##    ##     ##    ##    ########
            """

            # Switch mode
            # State: INIT
            if self.main_state == MainState.INIT:
                # slam_manager is ready
                if self.slam_state == SlamState.MAPPING:
                    self.main_state = MainState.MAPPING
                    self.robot_mode_req = (
                        RobotMode.NONE
                    )  # Clear initial value if mode is set
                    continue
                elif self.slam_state == SlamState.LOCALIZING:
                    if (
                        self.auto_manual_sw == SENSOR_DEACTIVATE
                        and "use_auto_man_sw" in self.option_config
                        and self.option_config["use_auto_man_sw"]
                    ):
                        self.main_state = MainState.MANUAL
                    else:
                        self.main_state = MainState.AUTO
                    continue
                elif self.slam_state == SlamState.SWITCH_LOCALIZING:
                    if (
                        self.auto_manual_sw == SENSOR_DEACTIVATE
                        and "use_auto_man_sw" in self.option_config
                        and self.option_config["use_auto_man_sw"]
                    ):
                        self.main_state = MainState.WAIT_MANUAL
                    else:
                        self.main_state = MainState.WAIT_AUTO
                    continue
                elif self.slam_state == SlamState.SWITCH_MAPPING:
                    self.main_state = MainState.WAIT_MAPPING
                    continue
                # slam_manager is not ready
                elif self.robot_mode_req == RobotMode.MANUAL:
                    self.robot_mode_req = RobotMode.NONE
                    self.pause_robot()
                    self.main_state = MainState.WAIT_MANUAL
                    if self.slam_state != SlamState.LOCALIZING:
                        self.main_state = MainState.INIT_LOAD_MAP
                elif rospy.get_time() - begin_loop > 2.0:
                    if self.slam_state != SlamState.LOCALIZING:
                        self.main_state = MainState.INIT_LOAD_MAP
            # State: WAIT_MAPPING
            elif self.main_state == MainState.WAIT_MAPPING:
                self.robot_mode = RobotMode.WAIT_RESPOND
                if self.slam_state == SlamState.MAPPING:
                    self.main_state = MainState.MAPPING
                elif self.slam_state == SlamState.ERROR_MAPPING:
                    self.main_state = MainState.MAPPING_ERROR
                # TODO: Handle when fake_slam but push "Create map"
                # elif self.slam_state == SlamState.LOCALIZING:
                #     self.main_state = MainState.MAPPING_ERROR
                # Timeout when slam_manager not run
                if (
                    rospy.get_time()
                    - self.slam_manager_module.last_module_status
                    > 2.0
                ):  # Sometime > 1.0 because get_node_list() take 1 sec
                    self.main_state = MainState.LOAD_MAP_ERROR
            # State: WAIT_MANUAL
            elif self.main_state == MainState.WAIT_MANUAL:
                # TODO: Handle /mission_status timeout
                self.robot_mode = RobotMode.WAIT_RESPOND
                self.main_state = MainState.MANUAL
            # State: WAIT_AUTO
            elif self.main_state == MainState.WAIT_AUTO:
                self.robot_mode = RobotMode.WAIT_RESPOND
                if self.slam_state == SlamState.LOCALIZING:
                    self.main_state = MainState.AUTO
                else:
                    print_error("Not yet switch to AUTO")
            # State: MANUAL
            elif self.main_state == MainState.MANUAL:
                self.robot_mode = RobotMode.MANUAL
                if self.init_pose_received:
                    self.init_pose_received = False
                    self.set_pose_init(True)
                if self.robot_mode_req == RobotMode.AUTO or (
                    self.auto_manual_sw == SWITCH_AUTO_MODE
                    and prev_auto_man_switch == SWITCH_MANUAL_MODE
                    and not self.simulation
                ):
                    self.robot_mode_req = RobotMode.NONE
                    self.main_state = MainState.WAIT_AUTO
                    continue
                elif self.robot_mode_req == RobotMode.MAPPING:
                    self.robot_mode_req = RobotMode.NONE
                    self.main_state = MainState.INIT_MAPPING
                    continue
                if self.slam_state == SlamState.ERROR_LOCALIZING and odom_ready:
                    self.main_state = MainState.LOAD_MAP_ERROR
                    continue
                self.check_condition_reset_err()
            # State: AUTO
            elif self.main_state == MainState.AUTO:
                self.robot_mode = RobotMode.AUTO
                if self.init_pose_received:
                    self.init_pose_received = False
                    self.set_pose_init(True)
                if self.robot_mode_req == RobotMode.MANUAL or (
                    self.auto_manual_sw == SENSOR_DEACTIVATE
                    and not self.simulation
                ):
                    self.robot_mode_req = RobotMode.NONE
                    # Stop robot
                    self.pause_robot()
                    self.main_state = MainState.WAIT_MANUAL
                elif self.robot_mode_req == RobotMode.MAPPING:
                    self.robot_mode_req = RobotMode.NONE
                    self.main_state = MainState.INIT_MAPPING
                if self.slam_state == SlamState.ERROR_LOCALIZING and odom_ready:
                    self.main_state = MainState.LOAD_MAP_ERROR
            # State: MAPPING
            elif self.main_state == MainState.MAPPING:
                self.robot_mode = RobotMode.MAPPING
                if self.robot_mode_req == RobotMode.MANUAL:
                    print_debug("MAPPING and request MANUAL")
                    self.robot_mode_req = RobotMode.NONE
                    self.main_state = MainState.INIT_LOAD_MAP
                    continue
                if (
                    self.save_map_request
                ):  # When stop MAPPING and force switch to MANUAL
                    if (
                        self.current_map_cfg.map_dir + self.map_requesting
                        == self.slam_manager_module.module_status_dict[
                            "current_map"
                        ]
                    ):
                        rospy.loginfo("MAPPING stop -> load new map")
                        self.save_map_request = False
                        self.current_map_cfg.map_file = self.map_requesting
                        self.dump_config()
                        self.main_state = MainState.INIT_LOAD_MAP
                        continue
                # Khi đang MAPPING mà /slam_manager disconnected, lúc kết nối lại cần reset
                self.check_condition_reset_err()
            # State: INIT_LOAD_MAP
            elif self.main_state == MainState.INIT_LOAD_MAP:
                map_file_check = (
                    self.current_map_cfg.map_dir + self.current_map_cfg.map_file
                )
                print_debug("Check map: {}".format(map_file_check))
                if self.check_map(map_file_check):
                    self.load_map_cnt += 1
                    print_debug(
                        "Load map request time: {}".format(self.load_map_cnt)
                    )
                    self.load_map(self.current_map_cfg.map_file)
                    begin_load_map = rospy.get_time()
                    self.main_state = MainState.LOADING_MAP
                else:
                    self.main_state = MainState.LOAD_MAP_ERROR
            # State: INIT_MAPPING
            elif self.main_state == MainState.INIT_MAPPING:
                self.pub_mapping_req.publish(StringStamped())
                self.main_state = MainState.WAIT_MAPPING
            # State: LOADING_MAP
            elif self.main_state == MainState.LOADING_MAP:
                self.robot_mode = RobotMode.WAIT_RESPOND
                if self.slam_state == SlamState.LOCALIZING:
                    if rospy.get_time() - begin_load_map > 2.0:
                        if (
                            first_check_mode
                            and self.auto_manual_sw == SWITCH_AUTO_MODE
                        ):
                            self.main_state = MainState.AUTO
                        else:
                            self.main_state = MainState.MANUAL
                        self.robot_mode_req = RobotMode.NONE
                        first_check_mode = False
                if self.slam_state == SlamState.ERROR_LOCALIZING:
                    self.main_state = MainState.LOAD_MAP_ERROR
                # Timeout when slam_manager not run
                if (
                    rospy.get_time()
                    - self.slam_manager_module.last_module_status
                    > 2.0
                ):  # Sometime > 1.0 because get_node_list() take 1 sec
                    self.main_state = MainState.LOAD_MAP_ERROR
            # State: LOAD_MAP_ERROR
            elif self.main_state == MainState.LOAD_MAP_ERROR:
                # TODO: Load prev map, load other map
                # self.robot_mode = RobotMode.MANUAL
                self.detail_status = DetailStatus.LOAD_MAP_ERROR.toString()
                self.check_condition_reset_err()
                # Auto reset
                auto_reset_load_map = False
                if rospy.get_time() - begin_load_map_error > 30.0:
                    auto_reset_load_map = True
                    rospy.logwarn("Auto reset LOAD_MAP_ERROR")
                if self.reset_error_request or auto_reset_load_map:
                    self.reset_error_request = False
                    self.main_state = MainState.INIT_LOAD_MAP
            # State: MAPPING_ERROR
            elif self.main_state == MainState.MAPPING_ERROR:
                self.robot_mode = RobotMode.MAPPING
                self.detail_status = DetailStatus.MAPPING_ERROR.toString()
                self.check_condition_reset_err()
                if self.reset_error_request:
                    self.reset_error_request = False
                    self.main_state = MainState.INIT_MAPPING

            # Set timeout
            if (
                self.main_state == MainState.LOAD_MAP_ERROR
                and self.prev_state != MainState.LOAD_MAP_ERROR
            ):
                begin_load_map_error = rospy.get_time()

            """
             ######  ########    ###    ######## ##     ##  ######
            ##    ##    ##      ## ##      ##    ##     ## ##    ##
            ##          ##     ##   ##     ##    ##     ## ##
             ######     ##    ##     ##    ##    ##     ##  ######
                  ##    ##    #########    ##    ##     ##       ##
            ##    ##    ##    ##     ##    ##    ##     ## ##    ##
             ######     ##    ##     ##    ##     #######   ######
            """
            # Check module check disconnect
            module_disconnected = ""
            special_matching_error = ""
            special_matching_state = ""
            special_error_led = ""
            special_state_led = ""
            special_sound = ""
            module_error_code = ""
            pause_if_error = True
            for m in self.module_list:
                if not m.module_alive and (
                    m.handle_when_sim or not self.simulation
                ):
                    self.update_error_code(module_disconnected=m.display_name)
                    module_disconnected = m.display_name
                    break
            for (
                m
            ) in (
                self.module_list
            ):  # Không để trong cùng vòng lặp đc vì nếu module trước lỗi mà module sau disconnect
                if m.special_matching_error != "":
                    special_matching_error = m.special_matching_error
                    special_error_led = m.special_error_led
                    self.update_error_code(msg=special_matching_error)
                    break
            for m in self.module_list:
                if m.error_code != "":
                    module_error_code = m.error_code
                    pause_if_error = m.pause_if_error
                    break
            for m in self.module_list:
                if m.special_matching_state != "":
                    special_matching_state = m.special_matching_state
                    special_state_led = m.special_state_led
                    special_sound = m.special_sound
                    break

            if module_disconnected == "" and special_matching_error == "":
                self.clear_error_code()

            # Update robot_status and detail_status
            self.robot_status = RobotStatus.NONE
            if (
                rospy.get_time() - self.last_std_io_msg > 1.0
                and not self.simulation
            ):
                self.robot_status = RobotStatus.ERROR
                self.detail_status = DetailStatus.FATAL_ERROR.toString()
                self.update_error_code(
                    msg=DetailStatus.IO_BOARD_DISCONNECT.toString()
                )
                # rospy.logwarn("IO_BOARD_DISCONNECT")
                odom_ready = False
            elif (
                rospy.Time.now().to_sec()
                - self.arduino_error_msg.stamp.to_sec()
                < 0.5
                and not self.simulation
            ):
                self.arduino_error_cnt += 1
                if self.arduino_error_cnt > 30:
                    self.robot_status = RobotStatus.ERROR
                    self.detail_status = DetailStatus.FATAL_ERROR.toString()
                    self.update_error_code(
                        msg=DetailStatus.IO_BOARD_ERROR.toString()
                    )
                    # rospy.logwarn("IO_BOARD_ERROR")
                    odom_ready = False
            elif (
                self.emg_button == SENSOR_DEACTIVATE
                or (self.bumper == SENSOR_DEACTIVATE and self.option_config["use_bumper"])
            ) and not self.simulation:
                self.robot_status = RobotStatus.EMG
                if last_robot_status != self.robot_status:
                    self.pause_robot()
            elif (
                self.motor_enable_sw == SENSOR_DEACTIVATE
                and not self.simulation
                and "use_motor_release" in self.option_config
                and self.option_config["use_motor_release"]
            ):
                self.robot_status = RobotStatus.ERROR
                self.detail_status = DetailStatus.MOTOR_OFF.toString()
            elif module_disconnected != "":
                # TOCHECK: module_disconnected but special_matching_error display
                self.robot_status = RobotStatus.ERROR
                self.detail_status = (
                    DetailStatus.FATAL_ERROR.toString()
                )  # Will be overwritten by update_feedback()
            # elif rospy.get_time() - self.last_odom_msg > 1.0:
            #     # If motor disconnect, lỗi FATAL_ERROR sẽ được ưu tiên hơn nên
            #     # không chạy đến đoạn này
            #     self.robot_status = RobotStatus.ERROR
            #     self.detail_status = DetailStatus.FATAL_ERROR.toString()
            #     self.update_error_code(
            #         msg=DetailStatus.ODOMETRY_ERROR.toString()
            #     )
            #     self.set_pose_init(False)
            elif (
                self.main_state == MainState.LOAD_MAP_ERROR
                or self.main_state == MainState.MAPPING_ERROR
            ):
                self.robot_status = RobotStatus.ERROR
                # self.detail_status = DetailStatus.LOAD_MAP_ERROR.toString()
                self.update_error_code(msg=self.detail_status)
            elif special_matching_error != "":
                self.robot_status = RobotStatus.ERROR
                self.detail_status = special_matching_error
            elif module_error_code != "":
                self.robot_status = RobotStatus.ERROR
                self.detail_status = DetailStatus.GENERAL_ERROR.toString()
                self.update_error_code(msg=module_error_code)
            elif (
                self.battery_percent < self.battery_config["empty_threshold"]
                and special_matching_state
                != DetailStatus.AUTO_CHARGING.toString()
                and special_matching_state
                != DetailStatus.MANUAL_CHARGING.toString()
            ):
                self.robot_status = RobotStatus.ERROR
                self.detail_status = DetailStatus.BATTERY_EMPTY.toString()
            elif self.motor_left_over_ampe or self.motor_right_over_ampe:
                self.robot_status = RobotStatus.ERROR
                self.detail_status = DetailStatus.MOTOR_OVER_CURRENT.toString()
                self.update_error_code(
                    msg=DetailStatus.MOTOR_OVER_CURRENT.toString()
                )
            elif (
                self.control_motor_left_error or self.control_motor_right_error
            ):
                self.robot_status = RobotStatus.ERROR
                self.detail_status = DetailStatus.MOTOR_CONTROL_ERROR.toString()
                self.update_error_code(
                    msg=DetailStatus.MOTOR_CONTROL_ERROR.toString()
                )
            elif (
                not self.pose_initiated
                and self.option_config["check_init_pose"]
            ):
                self.robot_status = RobotStatus.WAITING_INIT_POSE
            elif self.mission_status == ModuleStatus.WAITING:
                self.robot_status = RobotStatus.WAITING
            elif self.mission_status == ModuleStatus.PAUSED:
                self.robot_status = RobotStatus.PAUSED
            elif self.mission_status == ModuleStatus.RUNNING:
                self.robot_status = RobotStatus.RUNNING

            # Xử lý tình huống FATAL_ERROR có mứu ưu tiên cao hơn ODOMETRY_ERROR
            if rospy.get_time() - self.last_odom_msg > 1.0:
                self.set_pose_init(False)

            if module_disconnected != "" and last_module_disconnected == "":
                rospy.logerr(
                    "Module disconnected: {}".format(module_disconnected)
                )

            if not (
                self.detail_status
                == DetailStatus.IO_BOARD_DISCONNECT.toString()
                or self.detail_status == DetailStatus.IO_BOARD_ERROR.toString()
            ):
                odom_ready = True

            # Normal status, no need update robot_status
            if self.robot_status != RobotStatus.ERROR:
                if (
                    self.battery_percent < self.battery_config["low_threshold"]
                    and special_matching_state
                    != DetailStatus.AUTO_CHARGING.toString()
                    and special_matching_state
                    != DetailStatus.MANUAL_CHARGING.toString()
                ):
                    self.detail_status = DetailStatus.BATTERY_LOW.toString()
                elif (
                    safety_disabled
                    and self.robot_status == RobotStatus.RUNNING
                    and special_matching_state
                    != DetailStatus.AUTO_CHARGING.toString()
                    and special_matching_state
                    != DetailStatus.MANUAL_CHARGING.toString()
                ):
                    self.detail_status = DetailStatus.SAFETY_DISABLED.toString()
                elif (
                    self.is_safety
                    or rospy.get_time() - self.last_safety_time > 0.5
                ) and special_matching_state == "GOING_TO_POS":  # Hot fix SAFETY_STOP slow
                    self.detail_status = DetailStatus.SAFETY_STOP.toString()
                elif special_matching_state != "":
                    self.detail_status = special_matching_state

            # Clear Arduino Error
            if (
                rospy.Time.now().to_sec()
                - self.arduino_error_msg.stamp.to_sec()
                > 5.0
            ):
                self.arduino_error_cnt = 0

            """
            ##       ######## ########
            ##       ##       ##     ##
            ##       ##       ##     ##
            ##       ######   ##     ##
            ##       ##       ##     ##
            ##       ##       ##     ##
            ######## ######## ########
            """
            # Error
            if (
                self.detail_status == DetailStatus.FATAL_ERROR.toString()
            ):  # Lỗi kết nối với IO, cần phân biệt với các lỗi khác và có ưu tiên cao nhất
                self.led_status = LedStatus.FATAL_ERROR.toString()
            elif self.robot_status == RobotStatus.EMG:
                self.led_status = LedStatus.EMG.toString()
            elif (
                special_error_led != ""
            ):  # LIFT_STUCK, NO_CART, CHARGING_ERROR, MOVING_ERROR...
                self.led_status = special_error_led
            elif (
                self.detail_status == DetailStatus.MOTOR_OFF.toString()
            ):  # Lỗi có thể xử lý bằng tay
                self.led_status = LedStatus.MOTOR_OFF.toString()
            elif (
                self.robot_status == RobotStatus.ERROR
            ):  # Những lỗi cần tới tablet để xử lý
                self.led_status = LedStatus.GENERAL_ERROR.toString()
            # Other status
            elif (
                self.main_state == MainState.WAIT_MAPPING
                or self.main_state == MainState.WAIT_MANUAL
                or self.main_state == MainState.WAIT_AUTO
                or self.main_state == MainState.LOADING_MAP
            ):
                self.led_status = LedStatus.WAIT_RESPOND.toString()
            elif self.detail_status == DetailStatus.SAFETY_DISABLED.toString():
                self.led_status = LedStatus.SAFETY_DISABLED.toString()
            elif (
                self.detail_status == DetailStatus.AUTO_CHARGING.toString()
                and self.mission_status == ModuleStatus.WAITING
                and self.robot_mode == RobotMode.AUTO
            ):
                self.led_status = LedStatus.CHARGING_READY.toString()
            elif self.detail_status == DetailStatus.SAFETY_STOP.toString():
                self.led_status = LedStatus.SAFETY_STOP.toString()
            elif (
                special_state_led != ""
            ):  # AUTO_CHARGING, MANUAL_CHARGING, SAFETY_STOP, GOING_TO_POS, CHARGING_FULL...
                self.led_status = special_state_led
            elif self.detail_status == DetailStatus.BATTERY_EMPTY.toString():
                self.led_status = LedStatus.BATTERY_EMPTY.toString()
            elif self.robot_status == RobotStatus.WAITING_INIT_POSE:
                self.led_status = LedStatus.WAITING_INIT_POSE.toString()
            elif self.detail_status == DetailStatus.BATTERY_LOW.toString():
                self.led_status = LedStatus.BATTERY_LOW.toString()
            # Mode
            elif self.main_state == MainState.MANUAL:
                self.led_status = LedStatus.MANUAL.toString()
            elif self.main_state == MainState.MAPPING:
                self.led_status = LedStatus.MAPPING.toString()
            elif self.robot_status == RobotStatus.PAUSED:
                self.led_status = LedStatus.PAUSED.toString()
            elif self.robot_status == RobotStatus.WAITING:
                self.led_status = LedStatus.WAITING.toString()
            elif self.mission_status == ModuleStatus.RUNNING:
                self.led_status = LedStatus.RUNNING_NORMAL.toString()
            else:
                rospy.logwarn("Led status was not set")

            # Record log
            if (
                self.robot_status == RobotStatus.ERROR
                and last_robot_status != RobotStatus.ERROR
            ):
                self.db.recordLog(
                    "ERROR: {}".format(self.detail_status),
                    rospy.get_name(),
                    LogLevel.ERROR.toString(),
                )

            # Pause if error
            if (
                self.robot_status == RobotStatus.ERROR
                and last_robot_status != RobotStatus.ERROR
                and last_robot_status != RobotStatus.PAUSED
                and pause_if_error
                and self.robot_mode == RobotMode.AUTO
            ):
                rospy.logwarn("Pause by error")
                self.pause_robot()
                pause_by_error = True

            # Check clear pause_by_error
            if (
                self.robot_status == RobotStatus.WAITING
                and pause_by_error
                and self.robot_mode == RobotMode.AUTO
            ):
                pause_by_error = False
                rospy.logwarn("Clear pause by error")

            # Resume running if ERROR was cleared
            # if (
            #     self.robot_status == RobotStatus.PAUSED
            #     and pause_by_error
            #     and self.robot_mode == RobotMode.AUTO
            # ):
            #     rospy.logwarn("Resume after error")
            #     self.resume_running()
            #     pause_by_error = False

            # FIXME: Sometime MANUAL or ERROR but Robot still run
            # Eg: When /un_docking: MOVING_ERROR, mission_manager is PAUSED so do not forward reset cmd
            if (
                self.robot_mode != RobotMode.AUTO
                and last_robot_mode == RobotMode.AUTO
            ):
                begin_check_mode_change = rospy.get_time()
            if (
                self.robot_status != RobotStatus.RUNNING
                and self.robot_status == RobotStatus.RUNNING
            ):
                begin_check_status_change = rospy.get_time()
            if (
                self.robot_mode != RobotMode.AUTO
                and rospy.get_time() - begin_check_mode_change > 1.0
                and rospy.get_time() - begin_check_mode_change < 3.0
            ):
                if (
                    self.moving_control_status_dict["status"]
                    == ModuleStatus.RUNNING.toString()
                ):
                    self.pause_robot()
                    self.stop_moving()
                    rospy.logerr("Not yet PAUSED after switch mode to MANUAL")
            if (
                self.robot_status != RobotStatus.RUNNING
                and rospy.get_time() - begin_check_status_change > 1.0
                and rospy.get_time() - begin_check_status_change < 3.0
            ):
                if (
                    self.moving_control_status_dict["status"]
                    == ModuleStatus.RUNNING.toString()
                ):
                    self.pause_robot()
                    self.stop_moving()
                    rospy.logerr("Not yet PAUSED after status != RUNNING")

            """
            ########  ##     ## ######## ########  #######  ##    ##
            ##     ## ##     ##    ##       ##    ##     ## ###   ##
            ##     ## ##     ##    ##       ##    ##     ## ####  ##
            ########  ##     ##    ##       ##    ##     ## ## ## ##
            ##     ## ##     ##    ##       ##    ##     ## ##  ####
            ##     ## ##     ##    ##       ##    ##     ## ##   ###
            ########   #######     ##       ##     #######  ##    ##
            """

            # Button handle
            # Hold PAUSE button to stop mission
            # Không check Mode khi gửi stop mission vì không chắc chắn lúc đó
            # mission_manager có đang hoạt động không
            if (
                self.stop_1_button == SENSOR_ACTIVATE
                or self.stop_2_button == SENSOR_ACTIVATE
            ):
                if (
                    last_stop_1_button == SENSOR_DEACTIVATE
                    and last_stop_2_button == SENSOR_DEACTIVATE
                ):
                    pushed_stop_mission = False
                    begin_hold_stop_button = rospy.get_time()
                elif rospy.get_time() - begin_hold_stop_button >= 3.0:
                    if pushed_stop_mission == False:
                        pushed_stop_mission = True
                        rospy.loginfo("Stop all mission when hold PAUSE button")
                        self.pub_request_start_mission.publish(
                            StringStamped(stamp=rospy.Time.now(), data="STOP")
                        )
            # Reset error
            if (
                self.robot_status == RobotStatus.ERROR
            ):  # and self.robot_mode == RobotMode.AUTO:
                if (
                    self.start_1_button == SENSOR_ACTIVATE
                    and last_start_1_button == SENSOR_DEACTIVATE
                    or self.start_2_button == SENSOR_ACTIVATE
                    and last_start_2_button == SENSOR_DEACTIVATE
                ):
                    self.pub_mission_reset_error.publish(
                        EmptyStamped(stamp=rospy.Time.now())
                    )
                    rospy.loginfo("Reset error when push RUN button")
            # Trigger
            elif (
                self.robot_status == RobotStatus.WAITING
                and self.robot_mode == RobotMode.AUTO
            ):
                # Press and hold (not yet stable)
                # # Front RUN
                # if self.start_1_button == SENSOR_ACTIVATE and last_start_1_button == SENSOR_DEACTIVATE:
                #     trigger_button = "FRONT_RUN_BUTTON"
                # # Rear RUN
                # elif self.start_2_button == SENSOR_ACTIVATE and last_start_2_button == SENSOR_DEACTIVATE:
                #     trigger_button = "REAR_RUN_BUTTON"
                # # Front PAUSE
                # elif self.stop_1_button == SENSOR_ACTIVATE and last_stop_1_button == SENSOR_DEACTIVATE:
                #     trigger_button = "FRONT_PAUSE_BUTTON"
                # # Rear PAUSE
                # elif self.stop_2_button == SENSOR_ACTIVATE and last_stop_2_button == SENSOR_DEACTIVATE:
                #     trigger_button = "REAR_PAUSE_BUTTON"
                # # Hold front RUN
                # elif self.start_1_button == SENSOR_ACTIVATE and trigger_button == "FRONT_RUN_BUTTON":
                #     if rospy.get_time() - trigger_button_time > 1.0:
                #         self.pub_trigger.publish(StringStamped(stamp=rospy.Time.now(), data="FRONT_RUN_BUTTON"))
                #         rospy.loginfo("Send trigger when push front RUN button")
                #         # trigger_button = ""
                # # Hold rear RUN
                # elif self.start_2_button == SENSOR_ACTIVATE and trigger_button == "REAR_RUN_BUTTON":
                #     if rospy.get_time() - trigger_button_time > 1.0:
                #         self.pub_trigger.publish(StringStamped(stamp=rospy.Time.now(), data="REAR_RUN_BUTTON"))
                #         rospy.loginfo("Send trigger when push rear RUN button")
                #         # trigger_button = ""
                # # Hold front PAUSE
                # elif self.stop_1_button == SENSOR_ACTIVATE and trigger_button == "FRONT_PAUSE_BUTTON":
                #     if rospy.get_time() - trigger_button_time > 1.0:
                #         self.pub_trigger.publish(StringStamped(stamp=rospy.Time.now(), data="FRONT_PAUSE_BUTTON"))
                #         rospy.loginfo("Send trigger when push front PAUSE button")
                #         # trigger_button = ""
                # # Hold rear PAUSE
                # elif self.stop_2_button == SENSOR_ACTIVATE and trigger_button == "REAR_PAUSE_BUTTON":
                #     if rospy.get_time() - trigger_button_time > 1.0:
                #         self.pub_trigger.publish(StringStamped(stamp=rospy.Time.now(), data="REAR_PAUSE_BUTTON"))
                #         rospy.loginfo("Send trigger when push rear PAUSE button")
                #         # trigger_button = ""
                # # Clear flag
                # else:
                #     trigger_button_time = rospy.get_time()
                #     trigger_button = ""

                trigger_dict = {}
                trigger_dict["topic"] = ""
                trigger_dict["run_immediately"] = True
                # print_info(self.current_pose)
                if self.current_pose != None:
                    trigger_dict["pose"] = obj_to_dict(
                        self.current_pose, copy.deepcopy(pose_dict_template)
                    )
                    # Only press
                    if (
                        self.start_1_button == SENSOR_ACTIVATE
                        and last_start_1_button == SENSOR_DEACTIVATE
                    ):
                        trigger_dict["topic"] = "FRONT_RUN_BUTTON"
                        trigger_msg = json.dumps(trigger_dict)
                        self.pub_trigger.publish(
                            StringStamped(
                                stamp=rospy.Time.now(), data=trigger_msg
                            )
                        )
                        rospy.loginfo("Send trigger when push front RUN button")
                    elif (
                        self.start_2_button == SENSOR_ACTIVATE
                        and last_start_2_button == SENSOR_DEACTIVATE
                    ):
                        trigger_dict["topic"] = "REAR_RUN_BUTTON"
                        trigger_msg = json.dumps(trigger_dict)
                        self.pub_trigger.publish(
                            StringStamped(
                                stamp=rospy.Time.now(), data=trigger_msg
                            )
                        )
                        rospy.loginfo("Send trigger when push rear RUN button")
                    elif (
                        self.stop_1_button == SENSOR_ACTIVATE
                        and last_stop_1_button == SENSOR_DEACTIVATE
                    ):
                        trigger_dict["topic"] = "FRONT_PAUSE_BUTTON"
                        trigger_msg = json.dumps(trigger_dict)
                        self.pub_trigger.publish(
                            StringStamped(
                                stamp=rospy.Time.now(), data=trigger_msg
                            )
                        )
                        rospy.loginfo(
                            "Send trigger when push front PAUSE button"
                        )
                    elif (
                        self.stop_2_button == SENSOR_ACTIVATE
                        and last_stop_2_button == SENSOR_DEACTIVATE
                    ):
                        trigger_dict["topic"] = "REAR_PAUSE_BUTTON"
                        trigger_msg = json.dumps(trigger_dict)
                        self.pub_trigger.publish(
                            StringStamped(
                                stamp=rospy.Time.now(), data=trigger_msg
                            )
                        )
                        rospy.loginfo(
                            "Send trigger when push rear PAUSE button"
                        )
            # Resume
            elif (
                self.robot_status == RobotStatus.PAUSED
                and self.robot_mode == RobotMode.AUTO
            ):
                if (
                    self.start_1_button == SENSOR_ACTIVATE
                    and last_start_1_button == SENSOR_DEACTIVATE
                    or self.start_2_button == SENSOR_ACTIVATE
                    and last_start_2_button == SENSOR_DEACTIVATE
                ):
                    rospy.loginfo("Resume robot when push RUN button")
                    self.resume_running()
            # Pause
            elif self.robot_status == RobotStatus.RUNNING:
                if (
                    self.stop_1_button == SENSOR_ACTIVATE
                    and last_stop_1_button == SENSOR_DEACTIVATE
                    or self.stop_2_button == SENSOR_ACTIVATE
                    and last_stop_2_button == SENSOR_DEACTIVATE
                ):
                    rospy.loginfo("Pause robot when push PAUSE button")
                    self.pause_robot()

            """
             ######  ######## ########  ##     ## ######## ########
            ##    ## ##       ##     ## ##     ## ##       ##     ##
            ##       ##       ##     ## ##     ## ##       ##     ##
             ######  ######   ########  ##     ## ######   ########
                  ## ##       ##   ##    ##   ##  ##       ##   ##
            ##    ## ##       ##    ##    ## ##   ##       ##    ##
             ######  ######## ##     ##    ###    ######## ##     ##
            """
            # Get mission from server
            if (
                self.robot_status == RobotStatus.WAITING
                and self.robot_mode == RobotMode.AUTO
            ):
                if rospy.get_time() - begin_request_mission > 2.0:
                    if self.agv_db_object_id != None:
                        try:
                            data = {
                                "agv": self.server_config["agv_name"],
                            }
                            response = requests.post(
                                self.api_url + "functions/get_mission",
                                data=json.dumps(data),
                                headers=self.header,
                                timeout=1,
                            )  # TOCHECK
                            _temp = json.loads(response.text)
                            mission_json = json.dumps(_temp, indent=2)
                            print_debug(mission_json)
                            # print_debug(_temp["result"])
                            if (
                                _temp["result"] != None
                                and "actions" in _temp["result"]
                                and len(_temp["result"]["actions"]) > 0
                            ):
                                self.pub_mission_fr_server.publish(
                                    StringStamped(
                                        stamp=rospy.Time.now(),
                                        data=mission_json,
                                    )
                                )
                                self.pub_mission_fr_server_latch.publish(
                                    StringStamped(
                                        stamp=rospy.Time.now(),
                                        data=mission_json,
                                    )
                                )
                                # TODO: get from ros param
                                mission_queue_id = _temp["result"][
                                    "mission_queue_id"
                                ]
                                rospy.set_param(
                                    "/current_mission_queue_id",
                                    mission_queue_id,
                                )
                        except requests.exceptions.RequestException as e:
                            rospy.logerr_throttle(10.0, e)
                            self.agv_db_object_id = None

                        # Update mission done
                        if self.last_done_mission_id == mission_queue_id:
                            rospy.logwarn(
                                "Mission done id: {}".format(
                                    self.last_done_mission_id
                                )
                            )
                            data = {
                                "function": "UPDATE_MISSION",
                                "missionId": self.last_done_mission_id,
                                "progress": "done",
                            }
                            try:
                                while True:
                                    response_update_mission = requests.post(
                                        self.api_url + "functions/agvapi",
                                        data=json.dumps(data),
                                        headers=self.header,
                                        timeout=1,
                                    )
                                    result_update_mission = json.loads(
                                        response_update_mission.text
                                    )
                                    # if result_update_mission["result"] == "OK":
                                    #     break
                                    # else:
                                    #     rospy.logerr("UPDAVE MISSION FAIL")
                                    break
                            except requests.exceptions.RequestException as e:
                                rospy.logerr_throttle(10.0, e)
                                pass
                    elif self.server_config != None:  # Retry init server
                        self.pub_init_server.publish(
                            EmptyStamped(stamp=rospy.Time.now())
                        )
                    begin_request_mission = rospy.get_time()

            # Update position to server
            if self.agv_db_object_id != None:
                now = rospy.get_time()
                if now - last_update_pose >= 0.5:
                    last_update_pose = now
                    if self.current_pose != None:
                        pose = {
                            "x": temp_pose.position.x,
                            "y": temp_pose.position.y,
                            "z": 0.0,
                        }
                        data = {
                            "function": "UPDATE_AGV",
                            "agv": self.server_config["agv_name"],
                            "status": self.robot_status.toString(),
                            "position": pose,
                        }
                        try:
                            while True:
                                response_update_pose = requests.post(
                                    self.api_url + "functions/agvapi",
                                    data=json.dumps(data),
                                    headers=self.header,
                                    timeout=1,
                                )
                                result_update_pose = json.loads(
                                    response_update_pose.text
                                )
                                if result_update_pose["result"] == "OK":
                                    break
                                else:
                                    rospy.logerr("UPDAVE AGV FAIL")
                        except requests.exceptions.RequestException as e:
                            # rospy.logerr_throttle(10.0, e)
                            pass
            # Traffic control
            if self.server_config != None and (
                (
                    self.robot_status == RobotStatus.RUNNING
                    and self.robot_mode == RobotMode.AUTO
                )
                or (
                    self.robot_status == RobotStatus.PAUSED
                    and self.robot_mode == RobotMode.AUTO
                )
            ):
                if self.is_near_intersect_area(dist_check=2.5):
                    if self.current_pose != None:
                        pose = {
                            "x": temp_pose.position.x,
                            "y": temp_pose.position.y,
                            "z": 0.0,
                        }
                        data = {
                            "function": "TRAFFIC_CONTROL",
                            "agv": self.server_config["agv_name"],
                            "position": pose,
                        }
                        try:
                            response_traffic_control = requests.post(
                                self.api_url + "functions/agvapi",
                                data=json.dumps(data),
                                headers=self.header,
                                timeout=1,
                            )
                            response_traffic_control = json.loads(
                                response_traffic_control.text
                            )
                            if response_traffic_control["result"] == "STOP":
                                if self.robot_status == RobotStatus.RUNNING:
                                    rospy.logwarn("Stop by traffic control")
                                    self.pause_robot_by_server()
                            else:
                                if self.robot_status == RobotStatus.PAUSED:
                                    if self.pause_by_cross_function:
                                        rospy.logwarn(
                                            "Resume by traffic control"
                                        )
                                        self.resume_running_by_server()
                        except requests.exceptions.RequestException as e:
                            rospy.logerr_throttle(10.0, e)
                            pass
            # Publish status
            now = rospy.get_time()
            if now - last_pub_time >= 0.2:
                last_pub_time = now
                self.update_feedback()

            # Publish led status
            if self.led_status != prev_led_stt:
                msg = StringStamped()
                msg.stamp = rospy.Time.now()
                msg.data = self.led_status
                self.pub_led_status.publish(msg)
                prev_led_stt = self.led_status

            """
             ######   #######  ##     ## ##    ## ########
            ##    ## ##     ## ##     ## ###   ## ##     ##
            ##       ##     ## ##     ## ####  ## ##     ##
             ######  ##     ## ##     ## ## ## ## ##     ##
                  ## ##     ## ##     ## ##  #### ##     ##
            ##    ## ##     ## ##     ## ##   ### ##     ##
             ######   #######   #######  ##    ## ########
            """

            # Special sound
            if self.detail_status == DetailStatus.SAFETY_DISABLED.toString():
                self.sound_status = self.detail_status
            else:
                self.sound_status = special_sound

            if (
                self.sound_status != prev_sound_stt
            ):  # or rospy.get_time() - last_sound_time > 2.0: # Do not send request continuity because it will effect to request from app
                last_sound_time = rospy.get_time()
                self.pub_request_sound.publish(
                    StringStamped(
                        stamp=rospy.Time.now(), data=self.sound_status
                    )
                )
            prev_sound_stt = self.sound_status

            # Test server by topic
            if self.mission_from_server_req:
                self.mission_from_server_req = False
                if (
                    self.robot_mode == RobotMode.AUTO
                    and self.robot_status == RobotStatus.WAITING
                ):
                    # try:
                    if True:
                        mission_fr_server_file = os.path.join(
                            rospkg.RosPack().get_path("control_system"),
                            "mission",
                            "mission.json",
                        )
                        with open(mission_fr_server_file) as j:
                            print_warn(mission_fr_server_file)
                            mission_dict = json.load(j)
                            mission_json = json.dumps(mission_dict, indent=2)
                            # print_debug(mission_json)
                            self.pub_mission_fr_server.publish(
                                StringStamped(
                                    stamp=rospy.Time.now(), data=mission_json
                                )
                            )
                            self.pub_mission_fr_server_latch.publish(
                                StringStamped(
                                    stamp=rospy.Time.now(), data=mission_json
                                )
                            )
                    # except Exception as e:
                    #     rospy.logerr("Test mission from server: {}".format(e))

            # Update prev variable
            prev_auto_man_switch = self.auto_manual_sw
            last_start_1_button = self.start_1_button
            last_start_2_button = self.start_2_button
            last_stop_1_button = self.stop_1_button
            last_stop_2_button = self.stop_2_button
            last_robot_status = self.robot_status
            last_robot_mode = self.robot_mode
            last_module_disconnected = module_disconnected
            r.sleep()


def parse_opts():
    from optparse import OptionParser

    parser = OptionParser()
    parser.add_option(
        "-s",
        "--simulation",
        action="store_true",
        dest="simulation",
        default=False,
        help='type "-s" if simulation',
    )
    parser.add_option(
        "-d",
        "--ros_debug",
        action="store_true",
        dest="log_debug",
        default=False,
        help="log_level=rospy.DEBUG",
    )
    parser.add_option(
        "-c",
        "--current_map_file",
        dest="current_map_file",
        default=os.path.join(HOME + "/tmp/ros/maps/current_map.yaml"),
    )
    parser.add_option(
        "-r",
        "--robot_config_file",
        dest="robot_config_file",
        default=os.path.join(
            rospkg.RosPack().get_path("control_system"),
            "cfg",
            "robot_config.yaml",
        ),
    )
    parser.add_option(
        "--robot_define",
        dest="robot_define",
        default=os.path.join(
            HOME,
            "robot_config",
            "robot_define.yaml",
        ),
    )

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)


def main():
    # if not get_ubuntu_serial_hash():
    #     print_error("Control system: License verification failed.")
    #     return
    os.system("mkdir -p {}/tmp/ros/maps/".format(HOME))
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG

    rospy.init_node("control_system", log_level=log_level)
    rospy.loginfo("Init node " + rospy.get_name())
    ControlSystem(**vars(options))


if __name__ == "__main__":
    main()
