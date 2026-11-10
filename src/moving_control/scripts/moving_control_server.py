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
from math import pi, degrees, atan2
import numpy as np
import time
from agv_msgs.msg import (
    FollowLineSensor,
    DiffDriverMotorSpeed,
    FollowLineControl,
    WaypointGlobal,
    WaypointSafety,
)
from agv_msgs.srv import (
    WaypointsPath,
    WaypointsPathResponse,
    ArrayWaypoints,
    ArrayWaypointsResponse,
    ArraySafetyWaypoints,
    ArraySafetyWaypointsResponse,
)
from nav_msgs.msg import Odometry
import dynamic_reconfigure.client
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Twist
from std_msgs.msg import Int8, Header, Float32
from std_stamped_msgs.msg import (
    StringStamped,
    StringFeedback,
    StringResult,
    StringAction,
    EmptyStamped,
)
from actionlib_msgs.msg import GoalStatus, GoalID, GoalStatusArray
from safety_msgs.msg import SafetyStatus

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
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
    angle_robot_vs_goal_to_robot,
    yaw_to_quaternion,
    pose_dict_template,
    angle_vector_two_point,
    get_ubuntu_serial_hash,
)


class MainState(EnumString):
    NONE = -1
    SEND_GOAL = 0
    MOVING = 1
    RE_SEND_GOAL = 2
    RETURN_TO_RETRY = 3
    DONE = 6
    ERROR = 7
    STOP_BY_SAFETY = 8
    PAUSED = 9
    WAITING = 10
    WAIT_CLEAR_SAFETY = 11  # TODO
    FIND_LINE = 12
    FIND_LINE_ERROR = 13


class ToleranceType(EnumString):
    BOTH = 1
    XY = 2
    YAW = 3
    X_ONLY = 4
    NOT_REACH = 5


class MovingDirection(EnumString):
    INTERPOLATE = 0
    FORWARD = 1
    BACKWARD = 2


FORWARD = True
BACKWARD = False

goal_result = GoalStatus()
VIA_POINT_TOL_XY_BREAK = 0.2
VIA_POINT_TOL_YAW_BREAK = 0.1
NO_NEED_BREAK = Int8(0)
NEED_BREAK = Int8(1)


class MovingControl(object):
    _feedback = StringFeedback()
    _result = StringResult()

    def __init__(self, name, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        # Action server
        self._action_name = name
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            StringAction,
            execute_cb=self.moving_control_exec_cb,
            auto_start=False,
        )
        self._as.start()
        self._ac_planner_setting = actionlib.SimpleActionServer(
            "/planner_setting",
            StringAction,
            execute_cb=self.planner_setting_exec_cb,
            auto_start=False,
        )
        self._ac_planner_setting.start()
        # self.planner_setting_client = actionlib.SimpleActionClient('/planner_setting', StringAction)
        rospy.on_shutdown(self.shutdown)
        # Publisher
        self.break_control_pub = rospy.Publisher(
            "/break_control", Int8, queue_size=5
        )
        self.waypoints_following_pub = rospy.Publisher(
            "/waypoints_following", PoseArray, queue_size=5
        )
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)
        self.income_pose_pub = rospy.Publisher(
            "/income_pose", PoseStamped, queue_size=5
        )
        self.tolerance_pub = rospy.Publisher(
            "/moving_control/tolerance", StringStamped, queue_size=5
        )
        self.update_desired_params_pub = rospy.Publisher(
            "/update_desired_params", StringStamped, queue_size=5
        )
        self.update_reconfigure_params_pub = rospy.Publisher(
            "/update_reconfigure_params", StringStamped, queue_size=5
        )
        self.set_reconfigure_pub = rospy.Publisher(
            "/set_reconfigure_params", StringStamped, queue_size=5
        )
        self.stop_by_cross_pub = rospy.Publisher(
            "/stop_by_cross_line", Int8, queue_size=5
        )
        self.turn_radius_pub = rospy.Publisher(
            "/turn_radius", Float32, queue_size=5
        )
        self.followline_moving_control_pub = rospy.Publisher(
            "/followline_moving_control", Int8, queue_size=5
        )
        self.followline_control_publisher = rospy.Publisher(
            "/followline_control", FollowLineControl, queue_size=1
        )
        # Subscriber
        self.sub_line_sensor = rospy.Subscriber(
            "/followline_sensor", FollowLineSensor, self.line_sensor_cb
        )
        rospy.Subscriber("/safety_status", SafetyStatus, self.safety_status_cb)
        rospy.Subscriber("/robot_pose", Pose, self.robot_pose_cb)
        rospy.Subscriber("/stop_moving", EmptyStamped, self.stop_moving_cb)
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        rospy.Subscriber("~fake_error", EmptyStamped, self.fake_error_cb)
        rospy.Subscriber(
            "/update_desired_params",
            StringStamped,
            self.update_desired_params_cb,
            queue_size=1,
        )
        rospy.Subscriber(
            "/update_reconfigure_params",
            StringStamped,
            self.update_reconfigure_params_cb,
            queue_size=1,
        )
        # Service Server
        self.server_get_wps = rospy.Service(
            "/waypoints_to_global", WaypointsPath, self.waypoints_path_cb
        )
        self.server_get_wp_low_speed = rospy.Service(
            '/waypoints_low_speed', ArrayWaypoints, self.waypoints_low_speed_cb
        )
        self.server_get_wp_special_safety = rospy.Service(
            '/waypoints_special_safety', ArraySafetyWaypoints, self.waypoints_special_safety_cb
        )

        # ModuleServer
        self._asm = ModuleServer(name)
        self.last_module_status = self._asm.module_status
        # Initial
        self.planner_init(path=kwargs["planner_map"])
        self.load_default_params(path=kwargs["default_params"])
        self.load_planner_setting(path=kwargs["planner_setting"])
        self.loop()

    """

    # #    # # ##### #   ##   #
    # ##   # #   #   #  #  #  #
    # # #  # #   #   # #    # #
    # #  # # #   #   # ###### #
    # #   ## #   #   # #    # #
    # #    # #   #   # #    # ######

    """

    def init_variable(self, *args, **kwargs):
        self.simulation = kwargs["simulation"]
        print_debug("simulation: {}".format(self.simulation))
        # TF
        self.tf_listener = tf.TransformListener()
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.robot_base = "base_footprint"
        self.robot_pose = Pose()
        #
        self.planner_map = None
        self.planner_setting_dict = None
        self.current_local_planner = ""
        self.moving_direction = FORWARD
        self.dynamic_global_var = {}
        self.default_params_dict = {}
        self.safety_fields = []
        self.last_safety_fields = []
        self.delay_clear_safety = 5.0  # Second
        # Planner goal watchdog
        self.planner_pending_timeout = 2.0  # Second, re-send goal if planner does not accept it
        self.planner_feedback_timeout = 0.0  # Second, re-send goal if planner feedback stops (0: disable)
        self.planner_max_resend = 3  # Max re-send per waypoint before ERROR
        self.odom_msg = Odometry()
        self.find_line_vel = Twist()
        # Flag vars
        self.is_safety_stop = False
        self.fake_error = False
        self.move_action_status = -1
        self.move_action_result = -1
        self.prev_move_action_status = -1
        self.last_action_fb = rospy.get_time()
        self.last_scan_safety = rospy.get_time()
        self.last_robot_pose = rospy.get_time()
        self.array_wp_slow_speed = ArrayWaypointsResponse()
        self.array_wp_special_safety = ArraySafetyWaypointsResponse()
        # TODO: Get from config
        self.current_vel_param = "/move_to_point/max_vel_x"
        self.planner_type_rotate_and_straight = "move_to_point"
        self.planner_follow_line = "move_by_follow_line"
        self.last_state_use_follow_line = False
        self.mid_line_sensor = 0
        self.path_follow_line = []
        self.count_rotate_find_line = 0
        self.sign_rotate_find_line = 1
        self.array_wp_mb = WaypointsPathResponse()
        self.start_pose_plan = WaypointGlobal()
        self.bool_ignore_wp = False

    def shutdown(self):
        self.cancel_all_action()
        print("Shutdown: {}".format(rospy.get_name()))

    def cancel_all_action(self):
        for key, value in self.dynamic_global_var.items():
            self.dynamic_global_var[key]["action_cancel_pub"].publish(
                GoalID(stamp=rospy.Time.now())
            )

    """

    ###### #    # #    #  ####  ##### #  ####  #    #
    #      #    # ##   # #    #   #   # #    # ##   #
    #####  #    # # #  # #        #   # #    # # #  #
    #      #    # #  # # #        #   # #    # #  # #
    #      #    # #   ## #    #   #   # #    # #   ##
    #       ####  #    #  ####    #   #  ####  #    #

    """

    def load_planner_setting(self, path):
        # try:
        if True:
            with open(path) as file:
                planner_setting = yaml.load(file, Loader=yaml.Loader)
                self.planner_setting_dict = planner_setting[
                    "planner_setting_action"
                ]
                self.safety_vel_set = planner_setting["safety_vel"]["velocity"]
                self.safety_deceleration = planner_setting["safety_vel"][
                    "deceleration"
                ]
                self.obstacle_config = (
                    planner_setting["obstacle_config"]
                    if "obstacle_config" in planner_setting
                    else None
                )
                # List of params which will be set to by planner_setting action
                self.planner_setting_params = []
                for key, value in self.planner_setting_dict.items():
                    for i in value:
                        self.planner_setting_params.append(i)
                print_info(
                    "planner_setting params: {}".format(
                        self.planner_setting_params
                    )
                )
                print_debug("Safety vel set:\n{}".format(self.safety_vel_set))
        # except Exception as e:
        #     rospy.logerr('Error loading: {}'.format(e))

    def planner_init(self, path):
        try:
            with open(path) as file:
                self.planner_map = yaml.load(file, Loader=yaml.Loader)
        except Exception as e:
            rospy.logerr("Error loading: {}".format(e))
        # Loop all moving type and create Publisher, Subscriber
        for key, value in self.planner_map.items():
            action_server_name = value["action_server_name"]
            action_server_msg = value["action_server_msg"]
            action_server_prefix = value["action_server_prefix"]
            if key not in self.dynamic_global_var:
                # Import action msgs
                _cmd = "from {} import {}, {}, {}".format(
                    action_server_msg,
                    action_server_prefix + "Action",
                    action_server_prefix + "ActionResult",
                    action_server_prefix + "Goal",
                )
                print("cmd -->: " + _cmd)
                exec(_cmd)
                # Create action_client install
                # Similar: self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

                global temp  # Python3 exec() only can change global var
                action_client = None
                _cmd = "global temp; temp = actionlib.SimpleActionClient('{}', {})".format(
                    action_server_name, action_server_prefix + "Action"
                )
                print("cmd -->: " + _cmd)
                exec(_cmd)
                action_client = temp

                # Create action goal
                # move_base_goal = MoveBaseGoal(target_pose = goal)
                action_goal = None
                _cmd = "global temp; temp = {}Goal()".format(
                    action_server_prefix
                )
                print("cmd -->: " + _cmd)
                exec(_cmd)
                action_goal = temp

                # Create subscriber
                # rospy.Subscriber('/move_base/result', MoveToPointActionResult, move_to_point_result_cb)
                _cmd = """rospy.Subscriber('/{}/result', {}ActionResult, self.action_result_cb)""".format(
                    action_server_name, action_server_prefix
                )
                print("cmd -->: " + _cmd)
                exec(_cmd)

                _cmd = """rospy.Subscriber('/{}/status', GoalStatusArray, self.action_status_cb)""".format(
                    action_server_name
                )
                print("cmd -->: " + _cmd)
                exec(_cmd)

                # Create Publisher
                # move_base_cancel_publisher = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
                action_cancel_pub = None
                _cmd = """global temp; temp = rospy.Publisher('/{}/cancel', GoalID, queue_size=5)""".format(
                    action_server_name
                )
                print("cmd -->: " + _cmd)
                exec(_cmd)
                action_cancel_pub = temp

                # Assign object with a key
                planner_dict = {}
                planner_dict["action_client"] = action_client
                planner_dict["action_goal"] = action_goal
                planner_dict["action_cancel_pub"] = action_cancel_pub
                self.dynamic_global_var[key] = planner_dict
                print("---")
        print("dynamic_global_var:")
        for key, value in self.dynamic_global_var.items():
            print(key)
            for k, v in value.items():
                print("  {}: {}".format(k, type(v)))
        print("---")

    def load_default_params(self, path):
        default_params_path = path
        try:
            with open(default_params_path) as file:
                self.default_params_dict = json.load(file)
                print_debug(
                    default_params_path
                    + "\n"
                    + json.dumps(self.default_params_dict, indent=2)
                )
                self.default_lin_vel = self.default_params_dict["params"][
                    "reconfigure"
                ]["forward_vel"]
                self.default_back_vel = self.default_params_dict["params"][
                    "reconfigure"
                ]["backward_vel"]
                self.default_ang_vel = self.default_params_dict["params"][
                    "reconfigure"
                ]["angular_vel"]
                self.default_rot_vel = self.default_params_dict["params"][
                    "reconfigure"
                ]["rotate_vel"]
                self.last_lin_vel_bf_safety = (
                    self.default_lin_vel
                )  # Velocity regardless direction
                self.last_back_vel_bf_safety = (
                    self.default_back_vel
                )  # Velocity regardless direction
                self.last_ang_vel_bf_safety = self.default_ang_vel
                self.last_rot_vel_bf_safety = self.default_rot_vel
                self.set_lin_vel = self.default_lin_vel
                self.set_back_vel = self.default_back_vel
                self.set_ang_vel = self.default_ang_vel
                self.set_rot_vel = self.default_rot_vel
        except Exception as e:
            rospy.logerr("Error load default params: {}".format(e))

    def check_via_point_tol(
        self,
        frame_id,
        x_only_tolerance,
        target_pose,
        goal_xy_tol,
        goal_yaw_tol,
        via_point_xy_tol,
        via_point_yaw_tol,
    ):
        if goal_xy_tol > 0.06:
            target_xy_tol = goal_xy_tol + via_point_xy_tol
            target_yaw_tol = goal_yaw_tol + via_point_yaw_tol
        else:
            target_xy_tol = VIA_POINT_TOL_XY_BREAK
            target_yaw_tol = VIA_POINT_TOL_YAW_BREAK

        current_pose = lockup_pose(self.tf_listener, frame_id, self.robot_base)
        if current_pose == None:
            return ToleranceType.NOT_REACH
        # current_pose = self.robot_pose

        robot_yaw = get_yaw(current_pose)
        target_yaw = get_yaw(target_pose)
        delta_x = abs(current_pose.position.x - target_pose.position.x)
        delta_y = abs(current_pose.position.y - target_pose.position.y)
        delta_yaw = abs(delta_angle(robot_yaw, target_yaw))

        tolerance = str(delta_x) + "," + str(delta_y) + "," + str(delta_yaw)
        # tolerance_publisher.publish(tolerance)
        if (
            not x_only_tolerance
            and delta_x < target_xy_tol
            and delta_y < target_xy_tol
            and delta_yaw < target_yaw_tol
        ):
            return ToleranceType.BOTH
        if (
            x_only_tolerance
            and distance_to_line_perpendicular_vs_goal(
                current_pose, target_pose
            )
            < target_xy_tol
        ):
            return ToleranceType.X_ONLY
        if delta_x < target_xy_tol and delta_y < target_xy_tol:
            return ToleranceType.XY
        if delta_yaw < target_yaw_tol:
            return ToleranceType.YAW

        return ToleranceType.NOT_REACH

    def get_wp_display(self, wp_dict):
        ret = []
        for wp in wp_dict:
            pose_stamped = PoseStamped()
            pose_stamped.pose = self.dict_to_pose(wp["position"])
            ret.append(pose_stamped)
        return ret

    def send_feedback(self, action, msg):
        self._feedback.data = msg
        action.publish_feedback(self._feedback)

    def dict_to_pose(self, input_dict):
        w = {}
        w["position"] = input_dict["position"]
        w["orientation"] = input_dict["orientation"]
        return dict_to_obj(w, Pose())

    def create_wp_for_mb(self, array_wp):
        self.array_wp_mb.Waypoints = []
        self.array_wp_slow_speed.Waypoints = []
        self.array_wp_special_safety.Waypoints = []

        wp_mb = WaypointGlobal()
        wp_mb.Pose.pose = self.dict_to_pose(array_wp[self.current_point]["position"])
        wp_mb.radius = 0.0
        self.array_wp_mb.Waypoints.append(wp_mb)
        if "position" in array_wp[self.current_point]:
            if "properties" in array_wp[self.current_point]["position"]:
                if "Slow_speed" in array_wp[self.current_point]["position"]["properties"]:
                    if array_wp[self.current_point]["position"]["properties"]["Slow_speed"] == "True":
                        wp = PoseStamped()
                        wp.pose = self.dict_to_pose(array_wp[self.current_point]["position"])
                        self.array_wp_slow_speed.Waypoints.append(wp)

                if "Safety_job" in array_wp[self.current_point]["position"]["properties"]:
                    wp_safety = WaypointSafety()
                    wp_safety.Pose.pose = self.dict_to_pose(array_wp[self.current_point]["position"])
                    wp_safety.safety_job = int(array_wp[self.current_point]["position"]["properties"]["Safety_job"])
                    self.array_wp_special_safety.Waypoints.append(wp_safety)

        number_mb_pose = 0
        for i in range(len(array_wp)):
            print(array_wp[i]["position"]["position"])

        for i in range(len(array_wp)):
            if i < self.current_point:
                continue

            _pose = array_wp[i]["position"]
            if i < len(array_wp) - 2:
                _goal = array_wp[i + 1]["position"]
                _goal_next = array_wp[i + 2]["position"]
            else:
                _goal = None
                _goal_next = None

            if _goal_next is not None and _goal is not None:

                angle_curPose_to_Goal = angle_vector_two_point(
                    self.dict_to_pose(_goal),
                    self.dict_to_pose(_pose),
                )

                angle_Goal_to_nextGoal = angle_vector_two_point(
                    self.dict_to_pose(_goal_next),
                    self.dict_to_pose(_goal),
                )
                diff_angle = delta_angle(angle_curPose_to_Goal, angle_Goal_to_nextGoal)
                print(abs(degrees(diff_angle)))
                switch_plan = False
                if "position" in array_wp[i+1]:
                        if "properties" in array_wp[i+1]["position"]:
                            if "Switch_plan" in array_wp[i+1]["position"]["properties"]:
                                if array_wp[i+1]["position"]["properties"]["Switch_plan"] == "True":
                                    switch_plan = True

                if abs(degrees(diff_angle)) < 120 and not switch_plan:
                    wp_mb = WaypointGlobal()
                    wp_mb.Pose.pose = self.dict_to_pose(array_wp[i+1]["position"])
                    if "data" in array_wp[i+1]:
                        wp_mb.radius = array_wp[i+1]["data"]["radius"]
                    elif "position" in array_wp[i+1]:
                        wp_mb.radius = array_wp[i+1]["position"]["radius"]
                    else:
                        wp_mb.radius = 0.0
                    self.array_wp_mb.Waypoints.append(wp_mb)
                    if "position" in array_wp[i+1]:
                        if "properties" in array_wp[i+1]["position"]:
                            if "Slow_speed" in array_wp[i+1]["position"]["properties"]:
                                if array_wp[i+1]["position"]["properties"]["Slow_speed"] == "True":
                                    wp = PoseStamped()
                                    wp.pose = self.dict_to_pose(array_wp[i+1]["position"])
                                    self.array_wp_slow_speed.Waypoints.append(wp)

                            if "Safety_job" in array_wp[i+1]["position"]["properties"]:
                                wp_safety = WaypointSafety()
                                wp_safety.Pose.pose = self.dict_to_pose(array_wp[i+1]["position"])
                                wp_safety.safety_job = int(array_wp[i+1]["position"]["properties"]["Safety_job"])
                                self.array_wp_special_safety.Waypoints.append(wp_safety)
                    print("--------------0-----------------")
                    print(array_wp[i+1]["position"])
                    number_mb_pose = i
                else:
                    wp_mb = WaypointGlobal()
                    wp_mb.Pose.pose = self.dict_to_pose(array_wp[i+1]["position"])
                    if "data" in array_wp[i+1]:
                        wp_mb.radius = array_wp[i+1]["data"]["radius"]
                    elif "position" in array_wp[i+1]:
                        wp_mb.radius = array_wp[i+1]["position"]["radius"]
                    else:
                        wp_mb.radius = 0.0
                    self.array_wp_mb.Waypoints.append(wp_mb)
                    if "position" in array_wp[i+1]:
                        if "properties" in array_wp[i+1]["position"]:
                            if "Slow_speed" in array_wp[i+1]["position"]["properties"]:
                                if array_wp[i+1]["position"]["properties"]["Slow_speed"] == "True":
                                    wp = PoseStamped()
                                    wp.pose = self.dict_to_pose(array_wp[i+1]["position"])
                                    self.array_wp_slow_speed.Waypoints.append(wp)

                            if "Safety_job" in array_wp[i+1]["position"]["properties"]:
                                wp_safety = WaypointSafety()
                                wp_safety.Pose.pose = self.dict_to_pose(array_wp[i+1]["position"])
                                wp_safety.safety_job = int(array_wp[i+1]["position"]["properties"]["Safety_job"])
                                self.array_wp_special_safety.Waypoints.append(wp_safety)
                    number_mb_pose = i
                    print("--------------1-----------------")
                    print(self.array_wp_mb.Waypoints)
                    return number_mb_pose

        print("--------------2-----------------")
        wp_mb = WaypointGlobal()
        wp_mb.Pose.pose = self.dict_to_pose(array_wp[i]["position"])
        if "data" in array_wp[i]:
            wp_mb.radius = array_wp[i]["data"]["radius"]
        elif "position" in array_wp[i]:
            wp_mb.radius = array_wp[i]["position"]["radius"]
        else:
            wp_mb.radius = 0.0
        self.array_wp_mb.Waypoints.append(wp_mb)
        if "position" in array_wp[i]:
            if "properties" in array_wp[i]["position"]:
                if "Slow_speed" in array_wp[i]["position"]["properties"]:
                    if array_wp[i]["position"]["properties"]["Slow_speed"] == "True":
                        wp = PoseStamped()
                        wp.pose = self.dict_to_pose(array_wp[i]["position"])
                        self.array_wp_slow_speed.Waypoints.append(wp)

                if "Safety_job" in array_wp[i]["position"]["properties"]:
                    wp_safety = WaypointSafety()
                    wp_safety.Pose.pose = self.dict_to_pose(array_wp[i]["position"])
                    wp_safety.safety_job = int(array_wp[i]["position"]["properties"]["Safety_job"])
                    self.array_wp_special_safety.Waypoints.append(wp_safety)

        number_mb_pose = len(array_wp) - 2
        print(self.array_wp_mb.Waypoints)
        print("--------------2-----------------")
        return number_mb_pose

    """

     ####    ##   #      #      #####    ##    ####  #    #
    #    #  #  #  #      #      #    #  #  #  #    # #   #
    #      #    # #      #      #####  #    # #      ####
    #      ###### #      #      #    # ###### #      #  #
    #    # #    # #      #      #    # #    # #    # #   #
     ####  #    # ###### ###### #####  #    #  ####  #    #

    """

    def waypoints_low_speed_cb(self, request):
        return self.array_wp_slow_speed

    def waypoints_special_safety_cb(self, request):
        return self.array_wp_special_safety

    def waypoints_path_cb(self, request):
        return self.array_wp_mb

    def stop_moving_cb(self, msg):
        rospy.logwarn("Stop moving request")
        self.cancel_all_action()
        self._asm.pause_req = True

    def robot_pose_cb(self, msg):
        self.robot_pose = msg
        self.last_robot_pose = rospy.get_time()

    def odom_cb(self, msg):
        self.odom_msg = msg

    def action_result_cb(self, msg):
        self.move_action_result = msg.status.status

    def fake_error_cb(self, msg):
        self.fake_error = True

    def update_desired_params_cb(self, msg):
        # Update reconfigure params via ros callback instead of thread to easy handle
        # print_debug("Update desired params cb:\n {}".format(msg))
        self.update_desired_params(
            json.loads(msg.data),
            local_planner=self.current_local_planner,
            comment="update_desired_params_cb",
        )

    def update_reconfigure_params_cb(self, msg):
        # print_debug("Update reconfigure params cb")
        self.update_local_planner_params(
            json.loads(msg.data), self.planner_map, self.current_point
        )

    def action_status_cb(self, msg):
        # Tránh sử dụng action status vì sau khi SUCCEEDED vẫn tiếp tục bắn về SUCCEEDED vài giây nữa
        # if not 'self.prev_move_action_status' in vars(__builtins__):
        #     self.prev_move_action_status = -1
        if len(msg.status_list) > 0:
            self.move_action_status = msg.status_list[
                len(msg.status_list) - 1
            ].status
            if self.prev_move_action_status != self.move_action_status:
                # rospy.loginfo('move_base_status: %s' % self.move_action_status)
                self.prev_move_action_status = self.move_action_status

    def safety_status_cb(self, msg):
        try:
            self.safety_vel_set
        except:
            return
        self.last_scan_safety = rospy.get_time()
        # if self._asm.action_running:
        #     print_warn("Safety received: {}".format(self.last_scan_safety))
        self.safety_fields = list(msg.fields)  # [1, 2, 3, ...]
        safety_vel = 0.0
        is_safety = False
        goal_dict = {}
        goal_dict["params"] = {}
        self.is_safety_stop = False
        if (
            len(self.safety_fields)
            and self.safety_fields[0] == 1
            and not self.simulation
        ):
            self.is_safety_stop = True

        # TODO: Force update
        force_update = False
        # Safety when PAUSED then change to RUNNING, force update
        if (
            self._asm.module_status == ModuleStatus.RUNNING
            and self._asm.module_status != self.last_module_status
        ):
            force_update = True
            print_warn("Force update")
        if (
            self._asm.module_status == ModuleStatus.RUNNING
            and self.safety_fields != self.last_safety_fields
        ) or force_update:
            print_warn("Update safety fields: {}".format(self.safety_fields))
            if len(self.safety_fields):
                for field_idx in range(len(self.safety_fields)):
                    # print(field_idx, self.safety_fields[field_idx])
                    if self.safety_fields[field_idx] == 1:
                        is_safety = True
                        safety_vel = self.safety_vel_set[field_idx]
                        # Do not set desired_backward_sp when safety
                        if self.moving_direction == FORWARD:
                            goal_dict["params"]["desired_speed"] = safety_vel
                            goal_dict["params"][
                                "desired_backward_sp"
                            ] = self.set_back_vel
                        else:
                            goal_dict["params"][
                                "desired_backward_sp"
                            ] = safety_vel
                            goal_dict["params"][
                                "desired_speed"
                            ] = self.set_back_vel
                        goal_dict["params"]["angular_speed"] = (
                            safety_vel
                            * self.last_ang_vel_bf_safety
                            / self.last_lin_vel_bf_safety
                        )
                        goal_dict["params"]["rotate_speed"] = (
                            safety_vel
                            * self.last_rot_vel_bf_safety
                            / self.last_lin_vel_bf_safety
                        )
                        # print_debug(json.dumps(goal_dict, indent=2))
                        self.update_desired_params(
                            goal_dict,
                            local_planner=self.current_local_planner,
                            comment="set safety",
                        )
                        break
            if not is_safety:  # Return last vel before safety
                if self.moving_direction == FORWARD:
                    goal_dict["params"][
                        "desired_speed"
                    ] = self.last_lin_vel_bf_safety
                    goal_dict["params"][
                        "desired_backward_sp"
                    ] = self.last_back_vel_bf_safety
                else:
                    goal_dict["params"][
                        "desired_speed"
                    ] = self.last_back_vel_bf_safety
                    goal_dict["params"][
                        "desired_backward_sp"
                    ] = self.last_lin_vel_bf_safety
                goal_dict["params"][
                    "angular_speed"
                ] = self.last_ang_vel_bf_safety
                goal_dict["params"][
                    "rotate_speed"
                ] = self.last_rot_vel_bf_safety
                self.update_desired_params(
                    goal_dict,
                    local_planner=self.current_local_planner,
                    comment="restore after safety",
                )
        self.last_safety_fields = self.safety_fields
        self.last_module_status = self._asm.module_status

    def action_fb(self, data):
        self.last_action_fb = rospy.get_time()

    """

    #####    ##   #####    ##   #    #    #    # #####  #####    ##   ##### ######
    #    #  #  #  #    #  #  #  ##  ##    #    # #    # #    #  #  #    #   #
    #    # #    # #    # #    # # ## #    #    # #    # #    # #    #   #   #####
    #####  ###### #####  ###### #    #    #    # #####  #    # ######   #   #
    #      #    # #   #  #    # #    #    #    # #      #    # #    #   #   #
    #      #    # #    # #    # #    #     ####  #      #####  #    #   #   ######

    """

    def update_desired_params(
        self, param_input_dict, local_planner="all", comment=""
    ):
        param_dict = {}
        if local_planner == "":
            return

        for planner_type, value in self.planner_map.items():
            param_dict["reconfigure"] = {}
            if local_planner != "all":
                param_dict[
                    "local_planner"
                ] = local_planner  # Only update specific planner
            else:
                param_dict["local_planner"] = planner_type
            for desired_param, desired_value in param_input_dict[
                "params"
            ].items():
                for i in self.planner_setting_dict[desired_param]:
                    param_dict["reconfigure"][i] = desired_value
            print_warn(
                'Update local params from "{}", direction: {}, data: {}'.format(
                    comment,
                    self.moving_direction,
                    json.dumps(param_dict, indent=2),
                )
            )
            self.update_local_planner_params(
                param_dict,
                self.planner_map,
                "update from outside",
                comment=comment,
            )
            if local_planner != "all":
                break

    def update_params(self, data_dict, current_point, planner_map):
        # Use params list in global params
        # If waypoint has same param, use this param of waypoint
        # Merge dict
        rospy.loginfo(
            "Update params for wp {}/{}".format(
                current_point + 1, len(data_dict["waypoints"])
            )
        )
        global_params_dict = data_dict["params"]
        local_params_dict = data_dict["waypoints"][current_point]["params"]
        print("---")
        print("Global params dict")
        print(json.dumps(global_params_dict, indent=2))
        print("Local params dict")
        print(json.dumps(local_params_dict, indent=2))
        print("Merge params dict")
        final_params_dict = merge_two_dicts(
            global_params_dict, local_params_dict
        )
        print(json.dumps(final_params_dict, indent=2))
        # Update reconfigure params
        self.update_local_planner_params(final_params_dict, planner_map)
        return final_params_dict

    def update_local_planner_params(
        self, data_dict, planner_map, current_point, comment=""
    ):
        moving_type = data_dict["local_planner"]
        # print_warn("Update local planner params: {}".format(moving_type))
        defined_tag = []
        for s in planner_map[moving_type]["reconfigure_params"]:
            server = s["reconfigure_server"]
            # rospy.loginfo('Update param for {}, server: {}'.format(moving_type, server))
            param_dict = {}
            for i in s["params"]:
                # Value can None but param and tag can not
                tag = i["tag"]  # Not None
                param = i["param"]  # Not None
                defined_tag.append(tag)
                if tag != None or True:
                    # print('tag: {}, param: {}'.format(tag, param))
                    if tag in data_dict["reconfigure"]:
                        # print('Have tag: {}, param: {}'.format(tag, param))
                        value = data_dict["reconfigure"][tag]  # Can be None
                        # print_debug('Update param "{}" vs tag "{}, value = {}"'.format(param, tag, value))
                        if param != None and value != None:
                            param_dict[param] = value
                    # else:
                    #     # tag không có trong param['reconfigure'] nhưng trong planner_map có khai báo
                    #     if type(current_point) == int:
                    #         rospy.logwarn('Missing param for tag: {} of waypoint no {}'.format(tag, current_point+1))
                else:
                    print("None param: {}".format(param))

            # print('---\nParams will be updated:')
            # print(json.dumps(param_dict, indent=2))
            try:
                if param_dict != {}:
                    client = dynamic_reconfigure.client.Client(
                        server, timeout=1
                    )
                    config = client.update_configuration(param_dict)
                    # print_debug("Update \"{}\":\n{}".format(server, json.dumps(param_dict, indent=2)))
                # if param_dict != {}:
                #     msg_dict = {'server': server, 'param_dict': param_dict, "comment": comment}
                #     self.set_reconfigure_pub.publish(StringStamped(stamp=rospy.Time.now(), data=json.dumps(msg_dict)))
            except Exception as e:
                rospy.logwarn("Update moving reconfigure error: {}".format(e))

        # TODO: Check có param trong param['reconfigure'] nhưng trong planner_map chưa khai báo
        # for key, value in data_dict['reconfigure'].items():
        #     if key not in defined_tag and key != 'global_planner' and isinstance(current_point, int):
        #         rospy.logwarn('Param "{}" of waypoint no {} has not yet defined in planner_map'.format(key, current_point+1))

    def update_obstacle_state(self, avoid_obstacle):
        param_dict = {}
        param_dict["enabled"] = avoid_obstacle

        if self.obstacle_config == None:
            return
        for layer in self.obstacle_config:
            server = layer["reconfigure_server"]
            try:
                client = dynamic_reconfigure.client.Client(server, timeout=1)
                config = client.update_configuration(param_dict)
                print_debug("Update obstacle_layer: \n{}".format(param_dict))
            except Exception as e:
                rospy.logwarn(
                    "Update obstacle setting reconfigure error: {}".format(e)
                )

    """
    ######## ##     ## ########  ######  ##     ## ######## ########
    ##        ##   ##  ##       ##    ## ##     ##    ##    ##
    ##         ## ##   ##       ##       ##     ##    ##    ##
    ######      ###    ######   ##       ##     ##    ##    ######
    ##         ## ##   ##       ##       ##     ##    ##    ##
    ##        ##   ##  ##       ##    ## ##     ##    ##    ##
    ######## ##     ## ########  ######   #######     ##    ########
    """

    def planner_setting_exec_cb(self, goal):
        try:
            request_params_dict = json.loads(goal.data)
            print_warn(
                "Params update request:\n{}".format(
                    json.dumps(request_params_dict, indent=2)
                )
            )
            self.send_feedback(self._ac_planner_setting, "ACTIVE")
        except Exception as e:
            self.send_feedback(self._ac_planner_setting, "REJECTED")
            self._ac_planner_setting.set_preempted(text="Goal parse error")

        try:
            # Save last velocity
            if "desired_speed" in request_params_dict["params"]:
                self.set_lin_vel = request_params_dict["params"][
                    "desired_speed"
                ]
                # self.set_back_vel has been set only once time when load default params
                self.set_ang_vel = (
                    self.set_lin_vel
                    * self.default_ang_vel
                    / self.default_lin_vel
                )
                self.set_rot_vel = (
                    self.set_lin_vel
                    * self.default_rot_vel
                    / self.default_lin_vel
                )
                self.last_lin_vel_bf_safety = self.set_lin_vel
                self.last_ang_vel_bf_safety = self.set_ang_vel
                self.last_rot_vel_bf_safety = self.set_rot_vel
                print_debug("Set desired speed: {}".format(self.set_lin_vel))
                # TODO: Save desired_speed to db
            # Do not update reconfigure imeadiatly because when run this action, the robot has been stopped
            # self.update_desired_params(request_params_dict, local_planner=self.current_local_planner, comment="planner_setting_exec_cb")
            self._ac_planner_setting.set_succeeded(self._result)
        except Exception as e:
            rospy.logerr("Planner setting exec: {}".format(e))
            self._ac_planner_setting.set_aborted(text=e)

    def set_rule_of_path(self, path):
        for i in range(len(path)):
            path[i]["rule_path"] = ""

        for i in range(len(path) - 1):
            for j in range(len(self.path_follow_line)):
                for k in range(len(self.path_follow_line[j]) - 1):
                    if (
                        path[i]["name"] == self.path_follow_line[j][k]
                        and path[i + 1]["name"]
                        == self.path_follow_line[j][k + 1]
                    ) or (
                        path[i]["name"] == self.path_follow_line[j][k + 1]
                        and path[i + 1]["name"] == self.path_follow_line[j][k]
                    ):
                        path[i + 1]["rule_path"] = "Follow_line"
        return path

    def rotate_to_find_line(self, goal):
        # return 0 la da tim thay line chuyen qua state khac
        # return 1 la dang trong qua trinh tim line
        # return 2 la khong the tim thay line bao loi
        yaw_robot = get_yaw(self.robot_pose)
        # Đi lùi, sử dụng hàm angle_robot_vs_goal_to_robot
        # Đi tiến: sử dụng hàm angle_robot_vs_robot_to_goal
        # TODO: Thêm check hướng của Robot. Hiện đang mặc định đi lùi vào tape
        angle_robot_to_goal = (
            angle_robot_vs_robot_to_goal(goal, self.robot_pose)
            + self.sign_rotate_find_line * 0.5
        )

        if self.mid_line_sensor == 1 and np.abs(angle_robot_to_goal) < 0.7:
            self.count_rotate_find_line = 0
            return 0

        if self.count_rotate_find_line == 2:
            self.count_rotate_find_line = 0
            return 2

        if np.abs(angle_robot_to_goal) > 0.015:
            self.find_line_vel.angular.z = np.clip(
                angle_robot_to_goal, -0.2, 0.2
            )
            self.cmd_vel_pub.publish(self.find_line_vel)
            return 1
        else:
            self.sign_rotate_find_line = -self.sign_rotate_find_line
            self.count_rotate_find_line += 1
            return 1

    def line_sensor_cb(self, msg):
        if msg.data[7] == 1 or msg.data[8] == 1:
            self.mid_line_sensor = 1
        else:
            self.mid_line_sensor = 0

    def moving_control_exec_cb(self, goal):
        try:
            data_dict = json.loads(goal.data)
            rospy.logerr(f"data_dict: {data_dict}")
            if "path_follow_line" in data_dict:
                self.path_follow_line = data_dict["path_follow_line"]
            else:
                self.path_follow_line = []
            goal_accept = True
            print_warn("------------------Received goal:------------------")
            # print(json.dumps(data_dict, indent=2))
            if "params" not in data_dict:
                data_dict["params"] = {}
            if "reconfigure" not in data_dict["params"]:
                data_dict["params"]["reconfigure"] = {}
            # Set params in reconfigure from database params
            # TODO: check overwrite_planner_setting
            if "distance_threshold" in data_dict["params"]:
                data_dict["params"]["reconfigure"]["xy_tolerance"] = data_dict[
                    "params"
                ]["distance_threshold"]
                rospy.logwarn('"distance_threshold" not in data_dict params')
            # Get param from defaut param, prioritize for global param in root json tree
            data_dict["params"] = merge_two_dicts(
                self.default_params_dict["params"], data_dict["params"]
            )
            data_dict["params"]["reconfigure"] = merge_two_dicts(
                self.default_params_dict["params"]["reconfigure"],
                data_dict["params"]["reconfigure"],
            )
            # print_warn('Merge default params:')
            # print(json.dumps(data_dict, indent=2))
            # Get param from global param, prioritize for local param in each waypoint
            for i in range(len(data_dict["waypoints"])):
                if "params" not in data_dict["waypoints"][i]:
                    data_dict["waypoints"][i]["params"] = {}
                # TOCHECK: why not mege not empty ['reconfigure'] dict? Only copy this dict if not exist
                if "reconfigure" not in data_dict["waypoints"][i]["params"]:
                    data_dict["waypoints"][i]["params"]["reconfigure"] = {}
                # print_debug('''waypoint_{}'s params before merge:\n{}'''.format(i+1, json.dumps(data_dict['waypoints'][i]['params'], indent=2)))
                data_dict["waypoints"][i]["params"] = merge_two_dicts(
                    data_dict["params"], data_dict["waypoints"][i]["params"]
                )
                data_dict["waypoints"][i]["params"][
                    "reconfigure"
                ] = merge_two_dicts(
                    data_dict["params"]["reconfigure"],
                    data_dict["waypoints"][i]["params"]["reconfigure"],
                )
                # print_debug('''waypoint_{}'s params after merge:\n{}'''.format(i+1, json.dumps(data_dict['waypoints'][i]['params'], indent=2)))
            # print_warn('Merge local params:')
            # print(json.dumps(data_dict, indent=2))
            frame_id = data_dict["params"][
                "frame_id"
            ]  # Only use global frame_id, not yet use waypoint frame_id
            wp_display = self.get_wp_display(data_dict["waypoints"])
            self.waypoints_following_pub.publish(
                pose_stamped_array_to_pose_array(wp_display, frame_id)
            )
            # Avoid obstacle
            avoid_obstacle = data_dict["params"]["avoid_obstacle"]
            print_debug("Avoid obstacle: {}".format(avoid_obstacle))
            rospy.logdebug("Avoid obstacle: {}".format(avoid_obstacle))
            update_obstacle_thread = threading.Thread(
                target=self.update_obstacle_state, args=(avoid_obstacle,)
            )
            update_obstacle_thread.start()
            # self.update_obstacle_state(avoid_obstacle)
            # Go straight and rotate
            straight_and_rotate = False  # TODO: what happen with path guides
            straight_and_rotate = data_dict["params"]["straight_and_rotate"]
            # straight_and_rotate = True
            self.moving_direction = (
                data_dict["params"]["moving_direction"]
                if "moving_direction" in data_dict["params"]
                else FORWARD
            )
        except Exception as e:
            rospy.logerr("Action goal.data systax error: {}".format(e))
            self.send_feedback(self._as, "ABORTED")
            self._as.set_aborted(text="Action goal.data systax error")
            return

        # set rules for path
        data_dict["waypoints"] = self.set_rule_of_path(data_dict["waypoints"])
        # ignore first waypoint
        # self.bool_ignore_wp = False
        # if len(data_dict["waypoints"]) > 1:
        #     self.bool_ignore_wp = True
        #     self.start_pose_plan.Pose.pose = self.dict_to_pose(
        #         data_dict["waypoints"][0]["position"]
        #     )
        #     # self.start_pose_plan.radius = data_dict["waypoints"][0]["position"]["radius"]
        #     self.start_pose_plan.radius = 0.0
        #     data_dict["waypoints"].pop(0)

        # check start is near goal and set success
        start_pose =  self.dict_to_pose(data_dict["waypoints"][0]["position"])
        goal_pose =  self.dict_to_pose(data_dict["waypoints"][-1]["position"])
        dis_start_to_goal = distance_two_pose(start_pose, goal_pose)
        if abs(dis_start_to_goal) < 0.15:
            self._as.set_succeeded(self._result)
            return

        success = False
        _state = MainState.SEND_GOAL
        _state_when_safety = None
        _prev_state = MainState.NONE
        self.current_point = 0
        action_goal = None
        action_client = None
        local_planner = ""
        last_action_client = None
        last_local_planner = ""
        current_target_pose = PoseStamped()
        current_params_dict = {}
        is_retry = False
        check_action_status = False
        max_retry_time = 0
        move_base_retry_cnt = 0
        planner_resend_cnt = 0
        goal_sent_time = rospy.get_time()
        begin_check_timeout = rospy.get_time()
        begin_clear_safety = rospy.get_time()
        begin_delay_safety_sp = rospy.get_time()
        last_loop_time = rospy.get_time()
        first_print_timeout = False
        total_point = 0
        start_time = rospy.get_time()
        feedback_msg = ""
        self._asm.reset_flag()
        is_print_error = False
        safety_timeout = False
        robot_pose = None
        begin_error = False
        safety_speed = Twist()
        goal_dict = {}
        goal_dict["params"] = {}
        is_rotate_only = False
        first_changed_state = False
        self.is_safety_stop = False

        # Loop
        polling_rate = 20.0
        NOP_TIME = 0.0001
        rate = rospy.Rate(polling_rate)
        t = rospy.get_time()
        diff_time = 1.0 / polling_rate

        while goal_accept and not rospy.is_shutdown():
            self._asm.action_running = True

            if rospy.get_time() - t < diff_time:
                # Must be sleep to prevent high CPU load
                rospy.sleep(NOP_TIME)
                continue
            else:
                # rospy.loginfo("{}".format(round(1.0/(rospy.get_time() - t - NOP_TIME), 2)))
                t = rospy.get_time()

            # Scan safety timeout
            # TOCHECK: vòng lặp bị trễ khi update reconfigure => MOVING_DISCONNECTED
            now = rospy.get_time()
            duration = now - last_loop_time
            if duration > 0.25:
                print_error("Loop time: {}".format(round(duration, 2)))
            last_loop_time = rospy.get_time()

            if (
                rospy.get_time() - self.last_scan_safety > 5.0
                and not self.simulation
                and _state == MainState.MOVING
            ):
                safety_timeout = True
                if not is_print_error:
                    rospy.logerr("Safety module disconnected")
                    is_print_error = True
            else:
                is_print_error = False
                safety_timeout = False

            # TOCHECK: Don't know why action was preempt when UNDOCKING
            if self._as.is_preempt_requested() or self._asm.reset_action_req:
                # if self._asm.reset_action_req:
                rospy.logwarn(
                    "Is preempt: {}".format(self._as.is_preempt_requested())
                )
                rospy.logwarn(
                    "Is reset_action_req: {}".format(self._asm.reset_action_req)
                )
                rospy.loginfo("%s: Preempted" % self._action_name)
                self.cancel_all_action()
                self._as.set_preempted()
                success = False
                self.send_feedback(self._as, "PREEMPTED")
                break

            if self._asm.module_status != ModuleStatus.ERROR:
                self._asm.error_code = ""
            if _state != MainState.PAUSED and _state != MainState.ERROR:
                self._asm.module_status = ModuleStatus.RUNNING

            if _state != _prev_state:
                rospy.loginfo(
                    "Main state: {} -> {}".format(
                        _prev_state.toString(), _state.toString()
                    )
                )
                _prev_state = _state
                feedback_msg = _state.toString()
                self._asm.module_state = _state.toString()
                first_changed_state = True
            self.send_feedback(self._as, feedback_msg)

            if self.fake_error and _state != MainState.SEND_GOAL:
                print_debug("Fake error")
                self.fake_error = False
                _state = MainState.ERROR
                self.cancel_all_action()

            t_robot_pose = rospy.get_time() - self.last_robot_pose
            if t_robot_pose > 0.25:
                rospy.logerr(
                    "Missing robot_pose: {}".format(round(t_robot_pose, 2))
                )

            # State: SEND_GOAL
            if _state == MainState.SEND_GOAL:
                wp = data_dict["waypoints"][self.current_point]
                _pose = wp["position"]
                total_point = len(data_dict["waypoints"])
                if wp["rule_path"] == "Follow_line":
                    use_follow_line = True
                else:
                    use_follow_line = False
                # print(json.dumps(_pose, indent=4))
                # Check yaw of next point for go straight and rotate
                # print_debug(json.dumps(wp['params'], indent=2))

                """
                ########  #######  ##       ##        #######  ##      ## ##       #### ##    ## ########
                ##       ##     ## ##       ##       ##     ## ##  ##  ## ##        ##  ###   ## ##
                ##       ##     ## ##       ##       ##     ## ##  ##  ## ##        ##  ####  ## ##
                ######   ##     ## ##       ##       ##     ## ##  ##  ## ##        ##  ## ## ## ######
                ##       ##     ## ##       ##       ##     ## ##  ##  ## ##        ##  ##  #### ##
                ##       ##     ## ##       ##       ##     ## ##  ##  ## ##        ##  ##   ### ##
                ##        #######  ######## ########  #######   ###  ###  ######## #### ##    ## ########
                """
                if use_follow_line and "modify_status" not in wp["params"]:
                    if self.last_state_use_follow_line == False:
                        followline_ctrl_msg = FollowLineControl()
                        followline_ctrl_msg.Direction = 0
                        self.followline_control_publisher.publish(
                            followline_ctrl_msg
                        )
                        self.mid_line_sensor = 0
                        # robot_pose = lockup_pose(
                        #     self.tf_listener, self.map_frame, self.robot_base
                        # )
                        robot_pose = self.robot_pose
                        angle_robot_to_goal = angle_robot_vs_goal_to_robot(
                            dict_to_obj(_pose, Pose()), robot_pose
                        )
                        if angle_robot_to_goal > 0:
                            self.sign_rotate_find_line = -1
                        else:
                            self.sign_rotate_find_line = 1
                        _state = MainState.FIND_LINE
                        continue
                    try:
                        stop_by_cross_line = data_dict["params"][
                            "stop_by_cross_line"
                        ]
                    except:
                        stop_by_cross_line = False

                    if (
                        stop_by_cross_line
                        and self.current_point == total_point - 1
                    ):
                        self.stop_by_cross_pub.publish(1)
                    else:
                        self.stop_by_cross_pub.publish(0)

                    if self.current_point > 0:
                        wp_bf = data_dict["waypoints"][self.current_point - 1]
                        try:
                            turn_radius_bf = wp_bf["position"]["radius"]
                        except:
                            turn_radius_bf = 0.0
                        _pose_bf = wp_bf["position"]
                    else:
                        turn_radius_bf = 0.0
                        wp_bf = None
                        _pose_bf = None
                    if self.current_point > 1:
                        wp_bf_2 = data_dict["waypoints"][self.current_point - 2]
                        try:
                            turn_radius_bf_2 = wp_bf_2["position"]["radius"]
                        except:
                            turn_radius_bf_2 = 0.0
                        _pose_bf_2 = wp_bf_2["position"]
                    else:
                        turn_radius_bf_2 = 0.0
                        wp_bf_2 = None
                        _pose_bf_2 = None
                    if self.current_point < total_point - 1:
                        wp_next = data_dict["waypoints"][self.current_point + 1]
                        try:
                            turn_radius_next = wp_next["position"]["radius"]
                        except:
                            turn_radius_next = 0.0
                        _pose_next = wp_next["position"]
                    else:
                        turn_radius_next = 0.0
                        wp_next = None
                        _pose_next = None
                    try:
                        turn_radius = wp["position"]["radius"]
                    except:
                        turn_radius = 0.0
                    self.turn_radius_pub.publish(turn_radius)
                    # if (
                    #     _pose_bf is not None
                    #     and _pose_next is not None
                    #     and "modify_status" not in wp["params"]
                    #     and turn_radius <= 0.2
                    #     and turn_radius_next <= 0.2
                    # ):
                    #     angle_preGoal_to_curGoal = angle_vector_two_point(
                    #         dict_to_obj(_pose, Pose()),
                    #         dict_to_obj(_pose_bf, Pose()),
                    #     )
                    #     angle_curGoal_to_nextGoal = angle_vector_two_point(
                    #         dict_to_obj(_pose_next, Pose()),
                    #         dict_to_obj(_pose, Pose()),
                    #     )
                    #     diff_angle = delta_angle(
                    #         angle_preGoal_to_curGoal, angle_curGoal_to_nextGoal
                    #     )
                    #     if abs(degrees(diff_angle)) < 45:
                    #         ignore_goal = True
                    #     else:
                    #         ignore_goal = False
                    # else:
                    #     ignore_goal = False
                    # if ignore_goal:
                    #     data_dict["waypoints"].pop(self.current_point)
                    #     continue
                    if (
                        turn_radius_bf <= 0.2
                        and turn_radius_bf_2 <= 0.2
                        and "modify_status" not in wp["params"]
                    ):
                        org_pose = dict_to_obj(_pose, Pose())
                        rotate_income = False
                        # Get current robot position
                        # robot_pose = lockup_pose(
                        #     self.tf_listener, self.map_frame, self.robot_base
                        # )
                        # while robot_pose == None:
                        #     robot_pose = lockup_pose(
                        #         self.tf_listener,
                        #         self.map_frame,
                        #         self.robot_base,
                        #     )
                        robot_pose = self.robot_pose
                        # Calc income yaw
                        # TODO: Only check if distance is large than threshold
                        distance = distance_two_pose(org_pose, robot_pose)
                        rospy.loginfo("distance to pose: {}".format(distance))
                        diff_income_yaw = angle_robot_vs_robot_to_goal(
                            org_pose, robot_pose
                        )
                        # print_warn("diff_income_yaw: {}".format(diff_income_yaw))
                        income_yaw = (
                            get_yaw(robot_pose) + diff_income_yaw
                        )  # Absolute yaw
                        if (
                            abs(distance) < 0.2
                            and self.current_point < total_point - 1
                        ):
                            rospy.logwarn(
                                "IGNORE CURRENT POINT NUMBER: {} BECAUSE NEAR POSE ROBOT".format(
                                    self.current_point
                                )
                            )
                            if total_point == 1:
                                _state = MainState.DONE
                                continue
                            else:
                                self.current_point += 1
                                continue
                        if abs(degrees(diff_income_yaw)) > 135:
                            self.moving_direction = BACKWARD
                        else:
                            self.moving_direction = FORWARD
                        if self.moving_direction == BACKWARD:
                            income_yaw = income_yaw + pi
                        # print_warn("income_yaw: {}".format(income_yaw))
                        yaw_change_point_idx = self.current_point
                        # Check and make income rotate point
                        if (
                            self.moving_direction == FORWARD
                            and abs(diff_income_yaw)
                            > wp["params"]["reconfigure"]["yaw_tolerance"]
                        ) or (
                            self.moving_direction == BACKWARD
                            and abs(abs(diff_income_yaw) - pi)
                            > wp["params"]["reconfigure"]["yaw_tolerance"]
                        ):
                            rotate_income = True
                            # Make rotate income waypoint
                            sub_income_pose = copy.deepcopy(robot_pose)
                            # print_info("robot_pose: \n{}".format(sub_income_pose))
                            sub_income_pose.orientation = yaw_to_quaternion(
                                income_yaw
                            )
                            # print_info("income_rotate_pose: \n{}".format(sub_income_pose))
                            self.income_pose_pub.publish(
                                PoseStamped(
                                    header=Header(
                                        stamp=rospy.Time.now(),
                                        frame_id=self.map_frame,
                                    ),
                                    pose=sub_income_pose,
                                )
                            )
                            rotate_income_wp = copy.deepcopy(wp)
                            rotate_income_wp["params"][
                                "modify_status"
                            ] = "rotate_point"
                            rotate_income_wp["name"] = (
                                rotate_income_wp["name"] + "_rotate_income"
                            )
                            rotate_income_wp["params"][
                                "local_planner"
                            ] = (
                                self.planner_follow_line
                            )  # Only for send goal to planner, do not use for update reconfigure
                            rotate_income_wp["params"]["reconfigure"][
                                "rotate_only"
                            ] = True
                            rotate_income_wp["params"]["reconfigure"][
                                "yaw_tolerance"
                            ] = data_dict["params"]["reconfigure"][
                                "income_yaw_tolerance"
                            ]
                            rotate_income_wp["position"] = obj_to_dict(
                                sub_income_pose,
                                copy.deepcopy(pose_dict_template),
                            )
                            try:
                                rotate_income_wp["data"]["radius"] = 0.06
                            except:
                                rotate_income_wp["params"]["turn_radius"] = 0.06
                            # print_debug("rotate pose dict: \n{}".format(json.dumps(rotate_income_wp, indent=2)))
                            data_dict["waypoints"].insert(
                                self.current_point, rotate_income_wp
                            )  # Replace current point, current point is moved to the next point
                            # print_warn("Income pose dict in list: \n{}".format(json.dumps(data_dict['waypoints'][self.current_point], indent=2)))
                            yaw_change_point_idx = self.current_point + 1
                        # Change curent point yaw
                        new_main_yaw = income_yaw
                        temp_org_pose = copy.deepcopy(org_pose)
                        temp_org_pose.orientation = yaw_to_quaternion(
                            new_main_yaw
                        )
                        data_dict["waypoints"][yaw_change_point_idx][
                            "position"
                        ] = obj_to_dict(
                            temp_org_pose, copy.deepcopy(pose_dict_template)
                        )
                        data_dict["waypoints"][yaw_change_point_idx]["params"][
                            "modify_status"
                        ] = "yaw_changed"
                        # Set "global_planner" is yet use because "reconfigure" = false
                        data_dict["waypoints"][yaw_change_point_idx]["params"][
                            "reconfigure"
                        ]["global_planner"] = "carrot_planner/CarrotPlanner"
                        data_dict["waypoints"][yaw_change_point_idx]["params"][
                            "reconfigure"
                        ]["distance_ignore_angular"] = 0.4
                        data_dict["waypoints"][yaw_change_point_idx]["params"][
                            "local_planner"
                        ] = self.planner_follow_line
                        continue
                    if (turn_radius > 0.2 and turn_radius_bf > 0.2) or (
                        turn_radius_bf > 0.2
                        and turn_radius < 0.2
                        and turn_radius_next > 0.2
                    ):
                        rospy.logwarn(
                            "IGNORE GOAL: {}".format(
                                data_dict["waypoints"][self.current_point]
                            )
                        )
                        self.current_point += 1
                        self.followline_moving_control_pub.publish(0)
                        continue
                    elif turn_radius_bf > 0.2 and turn_radius_bf_2 > 0.2:
                        self.followline_moving_control_pub.publish(0)
                        rospy.logwarn("GO STRAIGHT AFTER TURN ANGLE > 135")
                    elif (turn_radius_bf > 0.2 and turn_radius_next > 0.2) or (
                        turn_radius > 0.2 and turn_radius_bf < 0.2
                    ):
                        self.followline_moving_control_pub.publish(4)
                        rospy.logwarn(
                            "GO STRAIGHT AFTER TURN ANGLE > 30 AND FORCE GO STRAIGH IF FOLLOWLINE NOT IN CENTER TAPE"
                        )
                    else:
                        # rospy.logerr(turn_radius)
                        # rospy.logerr(turn_radius_bf)
                        # rospy.logwarn("GO STRAIGHT AFTER TURN ANGLE > 30")
                        self.followline_moving_control_pub.publish(1)
                    if self.current_point == total_point - 1:
                        self.followline_moving_control_pub.publish(3)
                        rospy.logwarn("IGNORE ROTATION")
                    # elif self.current_point == 0:
                    #     rospy.logwarn("NOT IGNORE ROTATION")
                    #     self.followline_moving_control_pub.publish(2)
                    else:
                        # rospy.logwarn("NOT IGNORE ROTATION")
                        self.followline_moving_control_pub.publish(2)
                    continue

                """
                 ######  ######## ########     ###    ####  ######   ##     ## ########       ###    ##    ## ########     ########   #######  ########    ###    ######## ########
                ##    ##    ##    ##     ##   ## ##    ##  ##    ##  ##     ##    ##         ## ##   ###   ## ##     ##    ##     ## ##     ##    ##      ## ##      ##    ##
                ##          ##    ##     ##  ##   ##   ##  ##        ##     ##    ##        ##   ##  ####  ## ##     ##    ##     ## ##     ##    ##     ##   ##     ##    ##
                 ######     ##    ########  ##     ##  ##  ##   #### #########    ##       ##     ## ## ## ## ##     ##    ########  ##     ##    ##    ##     ##    ##    ######
                      ##    ##    ##   ##   #########  ##  ##    ##  ##     ##    ##       ######### ##  #### ##     ##    ##   ##   ##     ##    ##    #########    ##    ##
                ##    ##    ##    ##    ##  ##     ##  ##  ##    ##  ##     ##    ##       ##     ## ##   ### ##     ##    ##    ##  ##     ##    ##    ##     ##    ##    ##
                 ######     ##    ##     ## ##     ## ####  ######   ##     ##    ##       ##     ## ##    ## ########     ##     ##  #######     ##    ##     ##    ##    ########
                """
                if (
                    not use_follow_line
                    and straight_and_rotate
                    and "modify_status" not in wp["params"]
                ):
                    self.last_state_use_follow_line = False
                    # Is origin point
                    rospy.logerr(f"_pose: {json.dumps(_pose, indent=2)}")
                    #del _pose["display"]
                    #del _pose["is_localization"]
                    pose_edited = {}
                    pose_edited["position"] = _pose["position"]
                    pose_edited["orientation"] = _pose["orientation"]
                    org_pose = dict_to_obj(pose_edited, Pose())
                    rospy.logerr(f"_pose(edited): {json.dumps(pose_edited, indent=2)}")
                    rotate_income = False
                    # Get current robot position
                    robot_pose = lockup_pose(
                        self.tf_listener, self.map_frame, self.robot_base
                    )
                    # while robot_pose == None:
                    #     robot_pose = lockup_pose(
                    #         self.tf_listener, self.map_frame, self.robot_base
                    #     )
                    robot_pose = self.robot_pose
                    # Calc income yaw
                    # TODO: Only check if distance is large than threshold
                    diff_income_yaw = angle_robot_vs_robot_to_goal(
                        org_pose, robot_pose
                    )
                    # print_warn("diff_income_yaw: {}".format(diff_income_yaw))
                    income_yaw = (
                        get_yaw(robot_pose) + diff_income_yaw
                    )  # Absolute yaw
                    if self.moving_direction == BACKWARD:
                        income_yaw = income_yaw + pi
                    # print_warn("income_yaw: {}".format(income_yaw))
                    yaw_change_point_idx = self.current_point
                    # Check and make income rotate point
                    if (
                        self.moving_direction == FORWARD
                        and abs(diff_income_yaw)
                        > wp["params"]["reconfigure"]["yaw_tolerance"]
                        or self.moving_direction == BACKWARD
                        and abs(abs(diff_income_yaw) - pi)
                        > wp["params"]["reconfigure"]["yaw_tolerance"]
                    ):
                        rotate_income = True
                        # Make rotate income waypoint
                        sub_income_pose = copy.deepcopy(robot_pose)
                        # print_info("robot_pose: \n{}".format(sub_income_pose))
                        sub_income_pose.orientation = yaw_to_quaternion(
                            income_yaw
                        )
                        # print_info("income_rotate_pose: \n{}".format(sub_income_pose))
                        self.income_pose_pub.publish(
                            PoseStamped(
                                header=Header(
                                    stamp=rospy.Time.now(),
                                    frame_id=self.map_frame,
                                ),
                                pose=sub_income_pose,
                            )
                        )
                        rotate_income_wp = copy.deepcopy(wp)
                        rotate_income_wp["params"][
                            "modify_status"
                        ] = "rotate_point"
                        rotate_income_wp["name"] = (
                            rotate_income_wp["name"] + "_rotate_income"
                        )
                        rotate_income_wp["params"][
                            "local_planner"
                        ] = (
                            self.planner_type_rotate_and_straight
                        )  # Only for send goal to planner, do not use for update reconfigure
                        rospy.loginfo('Overwrite local_planner by "move_to_point"')
                        rotate_income_wp["params"]["reconfigure"][
                            "rotate_only"
                        ] = True
                        rotate_income_wp["params"]["reconfigure"][
                            "yaw_tolerance"
                        ] = 0.02
                        rotate_income_wp["position"] = obj_to_dict(
                            sub_income_pose, copy.deepcopy(pose_dict_template)
                        )
                        # print_debug("rotate pose dict: \n{}".format(json.dumps(rotate_income_wp, indent=2)))
                        data_dict["waypoints"].insert(
                            self.current_point, rotate_income_wp
                        )  # Replace current point, current point is moved to the next point
                        # print_warn("Income pose dict in list: \n{}".format(json.dumps(data_dict['waypoints'][self.current_point], indent=2)))
                        yaw_change_point_idx = self.current_point + 1
                    # Change curent point yaw
                    new_main_yaw = income_yaw
                    temp_org_pose = copy.deepcopy(org_pose)
                    temp_org_pose.orientation = yaw_to_quaternion(new_main_yaw)
                    data_dict["waypoints"][yaw_change_point_idx][
                        "position"
                    ] = obj_to_dict(
                        temp_org_pose, copy.deepcopy(pose_dict_template)
                    )
                    data_dict["waypoints"][yaw_change_point_idx]["params"][
                        "modify_status"
                    ] = "yaw_changed"
                    # Set "global_planner" is yet use because "reconfigure" = false
                    data_dict["waypoints"][yaw_change_point_idx]["params"][
                        "reconfigure"
                    ]["global_planner"] = "carrot_planner/CarrotPlanner"
                    data_dict["waypoints"][yaw_change_point_idx]["params"][
                        "reconfigure"
                    ]["distance_ignore_angular"] = 0.5
                    data_dict["waypoints"][yaw_change_point_idx]["params"][
                        "local_planner"
                    ] = self.planner_type_rotate_and_straight
                    rospy.loginfo('Overwrite local_planner by "move_to_point"')
                    # print_warn("Updated dict: \n{}".format(json.dumps(data_dict['waypoints'], indent=2)))
                    # Check final point
                    total_point = len(data_dict["waypoints"])
                    if (
                        self.current_point + 1 == total_point - 1
                        and rotate_income
                    ) or (
                        self.current_point + 1 == total_point
                        and not rotate_income
                    ):  # Last point trong trường hợp đã thêm _rotate_income khác với chưa thêm
                        # Hiện tại chỉ xử lý được next_action_type trong mission hiện tại, chưa xử lý được trong mission kế tiếp
                        if (
                            data_dict["next_action_type"] != "move"
                        ):  # This is last move action
                            # Create final rotate point
                            rotate_outcome_wp = copy.deepcopy(wp)
                            rotate_outcome_wp["params"][
                                "modify_status"
                            ] = "rotate_point"
                            rotate_outcome_wp["name"] = (
                                wp["name"] + "_rotate_outcome"
                            )
                            rotate_outcome_wp["params"][
                                "local_planner"
                            ] = (
                                self.planner_type_rotate_and_straight
                            )  # Only for send goal to planner, do not use for update reconfigure
                            rospy.loginfo('Overwrite local_planner by "move_to_point"')
                            rotate_outcome_wp["params"]["reconfigure"][
                                "rotate_only"
                            ] = True
                            rotate_outcome_wp["params"]["reconfigure"][
                                "yaw_tolerance"
                            ] = 0.02
                            rotate_outcome_wp["position"] = obj_to_dict(
                                org_pose, copy.deepcopy(pose_dict_template)
                            )
                            # rotate_outcome_wp['params']['modify_status'] = "rotate_point"
                            data_dict["waypoints"].append(rotate_outcome_wp)
                    # Execute in next cycle, this cycle only for make rotate income waypoint and update yaw for origin point
                    continue

                """
                ##      ##    ###    ##    ## ########   #######  #### ##    ## ########  ######     ########  ##          ###    ##    ## ##    ## ######## ########
                ##  ##  ##   ## ##    ##  ##  ##     ## ##     ##  ##  ###   ##    ##    ##    ##    ##     ## ##         ## ##   ###   ## ###   ## ##       ##     ##
                ##  ##  ##  ##   ##    ####   ##     ## ##     ##  ##  ####  ##    ##    ##          ##     ## ##        ##   ##  ####  ## ####  ## ##       ##     ##
                ##  ##  ## ##     ##    ##    ########  ##     ##  ##  ## ## ##    ##     ######     ########  ##       ##     ## ## ## ## ## ## ## ######   ########
                ##  ##  ## #########    ##    ##        ##     ##  ##  ##  ####    ##          ##    ##        ##       ######### ##  #### ##  #### ##       ##   ##
                ##  ##  ## ##     ##    ##    ##        ##     ##  ##  ##   ###    ##    ##    ##    ##        ##       ##     ## ##   ### ##   ### ##       ##    ##
                 ###  ###  ##     ##    ##    ##         #######  #### ##    ##    ##     ######     ##        ######## ##     ## ##    ## ##    ## ######## ##     ##
                """
                if not use_follow_line and not straight_and_rotate:
                    self.current_point = self.create_wp_for_mb(
                        data_dict["waypoints"]
                    )
                    _pose = data_dict["waypoints"][self.current_point + 1][
                        "position"
                    ]

                # print_info("Next cycle dict: \n{}".format(json.dumps(data_dict['waypoints'], indent=2)))
                wp_display = self.get_wp_display(data_dict["waypoints"])
                del wp_display[: self.current_point - 1]
                self.waypoints_following_pub.publish(
                    pose_stamped_array_to_pose_array(wp_display, frame_id)
                )

                current_target_pose.header.stamp = rospy.Time.now()
                current_target_pose.header.frame_id = frame_id
                current_target_pose.pose = self.dict_to_pose(_pose)
                print(
                    "---------------------------wp: {}/{}---------------------------".format(
                        self.current_point + 2, total_point
                    )
                )
                print(current_target_pose.pose)
                print('---')
                if "retries" in wp["params"]:
                    max_retry_time = wp["params"]["retries"]
                local_planner = wp["params"]["local_planner"]
                self.current_local_planner = local_planner
                rospy.loginfo(f'Local planner of wp {self.current_point + 1}: {local_planner}')
                # rospy.loginfo(f'self.dynamic_global_var: {self.dynamic_global_var}')
                # rospy.loginfo(f'self.dynamic_global_var[local_planner]: {self.dynamic_global_var[local_planner]}')
                action_goal = self.dynamic_global_var[local_planner][
                    "action_goal"
                ]
                action_goal.target_pose = current_target_pose
                action_client = self.dynamic_global_var[local_planner][
                    "action_client"
                ]
                # Update moving params
                # current_params_dict = self.update_params(data_dict, self.current_point, self.planner_map)
                current_params_dict = wp["params"]
                # Check if reconfigure params overwrite velocity set by planner setting action
                overwrite_planner_setting = wp["params"][
                    "overwrite_planner_setting"
                ]
                # print_warn("overwrite_planner_setting: {}".format(overwrite_planner_setting))
                # print_info("waypoint params: \n{}".format(json.dumps(current_params_dict, indent=2)))
                if (
                    not overwrite_planner_setting
                ):  # Get speed from default or planner_setting
                    for i in self.planner_setting_params:
                        # Delete params that define in planner_setting.yaml
                        del current_params_dict["reconfigure"][i]
                    self.last_lin_vel_bf_safety = self.set_lin_vel
                    self.last_back_vel_bf_safety = self.set_back_vel
                    self.last_ang_vel_bf_safety = self.set_ang_vel
                    self.last_rot_vel_bf_safety = self.set_rot_vel
                else:
                    # Overwrite last vel before safety. Ex: safety when docking
                    # Ignore moving_direction, all params was set in action goal
                    self.last_lin_vel_bf_safety = current_params_dict[
                        "reconfigure"
                    ]["forward_vel"]
                    self.last_back_vel_bf_safety = current_params_dict[
                        "reconfigure"
                    ]["backward_vel"]
                    self.last_ang_vel_bf_safety = current_params_dict[
                        "reconfigure"
                    ]["angular_vel"]
                    self.last_rot_vel_bf_safety = current_params_dict[
                        "reconfigure"
                    ]["rotate_vel"]

                if self.moving_direction == FORWARD:
                    goal_dict["params"][
                        "desired_speed"
                    ] = self.last_lin_vel_bf_safety
                    goal_dict["params"][
                        "desired_backward_sp"
                    ] = self.last_back_vel_bf_safety
                else:
                    goal_dict["params"][
                        "desired_speed"
                    ] = self.last_back_vel_bf_safety
                    goal_dict["params"][
                        "desired_backward_sp"
                    ] = self.last_lin_vel_bf_safety
                goal_dict["params"][
                    "angular_speed"
                ] = self.last_ang_vel_bf_safety
                goal_dict["params"][
                    "rotate_speed"
                ] = self.last_rot_vel_bf_safety
                # Update velocity
                # self.update_desired_params(goal_dict, local_planner=self.current_local_planner, comment="SEND_GOAL")
                # print_debug("Update speed for direction \"{}\": {}, {}".format(self.moving_direction, goal_dict["params"]["desired_speed"], goal_dict["params"]["desired_backward_sp"]))
                if straight_and_rotate or use_follow_line:
                    self.update_desired_params_pub.publish(
                        StringStamped(
                            stamp=rospy.Time.now(), data=json.dumps(goal_dict)
                        )
                    )
                    # print_info("params will be update: \n{}".format(json.dumps(current_params_dict, indent=2)))
                    # self.update_local_planner_params(current_params_dict, self.planner_map, self.current_point)
                    # Update another params, desired params already deleted
                    self.update_reconfigure_params_pub.publish(
                        StringStamped(
                            stamp=rospy.Time.now(),
                            data=json.dumps(current_params_dict),
                        )
                    )
                # TOCHECK: Cancel prev goal
                self.move_action_result = -1
                check_action_status = True
                # Cancel last action
                is_rotate_only = current_params_dict["reconfigure"][
                    "rotate_only"
                ]
                action_client.send_goal(action_goal, feedback_cb=self.action_fb)
                last_action_client = action_client
                last_local_planner = local_planner
                self.last_action_fb = rospy.get_time()
                goal_sent_time = rospy.get_time()
                start_time = rospy.get_time()
                _state = MainState.MOVING
                begin_check_timeout = rospy.get_time()
                first_print_timeout = False
            # State: FIND_LINE
            elif _state == MainState.FIND_LINE:
                # Check safety
                if self.is_safety_stop or safety_timeout:
                    _state_when_safety = _state
                    _state = MainState.STOP_BY_SAFETY
                    continue
                _pose = data_dict["waypoints"][self.current_point]["position"]
                find_line_result = self.rotate_to_find_line(
                    dict_to_obj(_pose, Pose())
                )
                # find_line_result = 0 la da tim thay line chuyen qua state khac
                # find_line_result = 1 la dang trong qua trinh tim line
                # find_line_result = 2 la khong the tim thay line bao loi
                self.last_state_use_follow_line = True
                if find_line_result == 0:
                    _state = MainState.SEND_GOAL
                    continue
                if find_line_result == 2:
                    _state = MainState.FIND_LINE_ERROR
                    continue

            # State: RE_SEND_GOAL
            elif _state == MainState.RE_SEND_GOAL:
                self.move_action_result = -1
                check_action_status = True
                if (
                    last_action_client != None
                    and last_local_planner != local_planner
                ):
                    print_debug("Cancel all goal of: {}".format(action_client))
                    last_action_client.cancel_all_goals()
                action_client.send_goal(action_goal, feedback_cb=self.action_fb)
                last_action_client = action_client
                last_local_planner = local_planner
                self.last_action_fb = rospy.get_time()
                goal_sent_time = rospy.get_time()
                _state = MainState.MOVING
            # State: STOP_BY_SAFETY
            elif _state == MainState.STOP_BY_SAFETY:
                # Hot fix STOP_BY_SAFETY but still moving
                # Check current params:
                current_vel_when_safety = rospy.get_param(
                    self.current_vel_param
                )
                # print_info("Current max_vel_x: {}".format(current_vel_param))
                if current_vel_when_safety > MIN_FLOAT:
                    # first_changed_state = False
                    goal_dict["params"]["desired_speed"] = 0.0
                    goal_dict["params"]["desired_backward_sp"] = 0.0
                    goal_dict["params"]["angular_speed"] = 0.0
                    goal_dict["params"]["rotate_speed"] = 0.0
                    # self.update_desired_params(goal_dict, local_planner=self.current_local_planner, comment="STOP_BY_SAFETY: force stop")
                    print_warn("Re-write desired params")
                    self.update_desired_params_pub.publish(
                        StringStamped(
                            stamp=rospy.Time.now(), data=json.dumps(goal_dict)
                        )
                    )
                # Check not safety
                if not self.is_safety_stop and not safety_timeout:
                    _state = _state_when_safety
                    if self.moving_direction == FORWARD:
                        goal_dict["params"][
                            "desired_speed"
                        ] = self.last_lin_vel_bf_safety
                        goal_dict["params"][
                            "desired_backward_sp"
                        ] = self.last_back_vel_bf_safety
                    else:
                        goal_dict["params"][
                            "desired_speed"
                        ] = self.last_back_vel_bf_safety
                        goal_dict["params"][
                            "desired_backward_sp"
                        ] = self.last_lin_vel_bf_safety
                    goal_dict["params"][
                        "angular_speed"
                    ] = self.last_ang_vel_bf_safety
                    goal_dict["params"][
                        "rotate_speed"
                    ] = self.last_rot_vel_bf_safety
                    # self.update_desired_params(goal_dict, local_planner=self.current_local_planner, comment="STOP_BY_SAFETY: restore")
                    print_warn("Restore desired params")
                    self.update_desired_params_pub.publish(
                        StringStamped(
                            stamp=rospy.Time.now(), data=json.dumps(goal_dict)
                        )
                    )
                    begin_clear_safety = rospy.get_time()
                # Pause handle
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    _state = MainState.PAUSED
                    self.cancel_all_action()
            # State: WAIT_CLEAR_SAFETY
            elif _state == MainState.WAIT_CLEAR_SAFETY:
                # TODO: Not yet use, nếu restore vận tốc tại đây thì vận tốc sẽ từ đang dừng chuyển sang vận tốc trước khi vào các vùng giảm tốc
                # mà bỏ qua các vùng giảm tốc dẫn đến giật.
                if (
                    rospy.get_time() - begin_clear_safety
                    > self.delay_clear_safety
                ):
                    if self.moving_direction == FORWARD:
                        goal_dict["params"][
                            "desired_speed"
                        ] = self.last_lin_vel_bf_safety
                        goal_dict["params"][
                            "desired_backward_sp"
                        ] = self.last_back_vel_bf_safety
                    else:
                        goal_dict["params"][
                            "desired_speed"
                        ] = self.last_back_vel_bf_safety
                        goal_dict["params"][
                            "desired_backward_sp"
                        ] = self.last_lin_vel_bf_safety
                    goal_dict["params"][
                        "angular_speed"
                    ] = self.last_ang_vel_bf_safety
                    goal_dict["params"][
                        "rotate_speed"
                    ] = self.last_rot_vel_bf_safety
                    # self.update_desired_params(goal_dict, local_planner=self.current_local_planner, comment="WAIT_CLEAR_SAFETY")
                    print_warn("Update desired params")
                    self.update_desired_params_pub.publish(
                        StringStamped(
                            stamp=rospy.Time.now(), data=json.dumps(goal_dict)
                        )
                    )
                    _state = MainState.MOVING
            # State: MOVING
            elif _state == MainState.MOVING:
                # # Check safety
                # is_safety = False
                # if self.safety_fields != self.last_safety_fields:
                #     print_warn("Update safety fields: {}".format(self.safety_fields))
                #     if len(self.safety_fields):
                #         for field_idx in range(len(self.safety_fields)):
                #             # print(field_idx, self.safety_fields[field_idx])
                #             if self.safety_fields[field_idx] == 1:
                #                 is_safety = True
                #                 safety_vel = self.safety_vel_set[field_idx]
                #                 if self.moving_direction == FORWARD:
                #                     goal_dict["params"]["desired_speed"] = safety_vel
                #                     goal_dict["params"]["desired_backward_sp"] = self.set_back_vel
                #                 else:
                #                     goal_dict["params"]["desired_backward_sp"] = safety_vel
                #                     goal_dict["params"]["desired_speed"] = self.set_back_vel
                #                 goal_dict["params"]["angular_speed"] = safety_vel * self.last_ang_vel_bf_safety / self.last_lin_vel_bf_safety
                #                 goal_dict["params"]["rotate_speed"] = safety_vel * self.last_rot_vel_bf_safety / self.last_lin_vel_bf_safety
                #                 # print_debug(json.dumps(goal_dict, indent=2))
                #                 # self.update_desired_params(goal_dict, local_planner=self.current_local_planner, comment="set safety")
                #                 self.update_desired_params_pub.publish(StringStamped(stamp=rospy.Time.now(), data=json.dumps(goal_dict)))
                #                 break
                #     if not is_safety: # Return last vel before safety
                #         if self.moving_direction == FORWARD:
                #             goal_dict["params"]["desired_speed"] = self.last_lin_vel_bf_safety
                #             goal_dict["params"]["desired_backward_sp"] = self.last_back_vel_bf_safety
                #         else:
                #             goal_dict["params"]["desired_speed"] = self.last_back_vel_bf_safety
                #             goal_dict["params"]["desired_backward_sp"] = self.last_lin_vel_bf_safety
                #         goal_dict["params"]["angular_speed"] = self.last_ang_vel_bf_safety
                #         goal_dict["params"]["rotate_speed"] = self.last_rot_vel_bf_safety
                #         # self.update_desired_params(goal_dict, local_planner=self.current_local_planner, comment="restore after safety")
                #         self.update_desired_params_pub.publish(StringStamped(stamp=rospy.Time.now(), data=json.dumps(goal_dict)))
                # Pause handle
                if self._asm.pause_req:
                    self._asm.reset_flag()
                    _state = MainState.PAUSED
                    self.cancel_all_action()
                    continue
                if self.is_safety_stop or safety_timeout:
                    if use_follow_line or straight_and_rotate:
                        _state_when_safety = _state
                        _state = MainState.STOP_BY_SAFETY
                        continue
                # Planner goal watchdog
                # Use status of the goal sent by this loop, /<planner>/result topic can carry result of an older goal
                planner_state = action_client.get_state()
                if planner_state in (
                    GoalStatus.PREEMPTED,
                    GoalStatus.SUCCEEDED,
                    GoalStatus.ABORTED,
                    GoalStatus.REJECTED,
                    GoalStatus.RECALLED,
                    GoalStatus.LOST,
                ):
                    self.move_action_result = planner_state
                else:
                    self.move_action_result = -1
                if planner_state in (GoalStatus.PREEMPTED, GoalStatus.RECALLED):
                    # Cancelled outside moving_control (Ex: rviz cancel button), do not auto restart
                    rospy.logerr(
                        "Planner goal was cancelled outside moving_control: {}".format(
                            GoalStatus.to_string(planner_state)
                        )
                    )
                    _state = MainState.ERROR
                    self.cancel_all_action()
                    continue
                planner_stuck = ""
                if (
                    planner_state == GoalStatus.PENDING
                    and rospy.get_time() - goal_sent_time
                    > self.planner_pending_timeout
                ):
                    planner_stuck = "goal still PENDING after {}s".format(
                        self.planner_pending_timeout
                    )
                elif (
                    planner_state == GoalStatus.ACTIVE
                    and self.planner_feedback_timeout > 0.0
                    and rospy.get_time() - self.last_action_fb
                    > self.planner_feedback_timeout
                ):
                    planner_stuck = "no feedback for {}s".format(
                        self.planner_feedback_timeout
                    )
                elif planner_state in (GoalStatus.REJECTED, GoalStatus.LOST):
                    planner_stuck = "goal {}".format(
                        GoalStatus.to_string(planner_state)
                    )
                if planner_stuck:
                    planner_resend_cnt += 1
                    if planner_resend_cnt > self.planner_max_resend:
                        rospy.logerr(
                            "Planner {}: {}, error after re-send {} times".format(
                                local_planner, planner_stuck, self.planner_max_resend
                            )
                        )
                        _state = MainState.ERROR
                        self.cancel_all_action()
                    else:
                        rospy.logwarn(
                            "Planner {}: {}, re-send goal ({}/{})".format(
                                local_planner,
                                planner_stuck,
                                planner_resend_cnt,
                                self.planner_max_resend,
                            )
                        )
                        _state = MainState.RE_SEND_GOAL
                    continue
                # if rospy.get_time() - self.last_action_fb >= 1.0:
                #     rospy.logerr("{} disconnected!".format(current_params_dict["local_planner"]))
                # Check via point tolerance
                xy_tol = current_params_dict["reconfigure"]["xy_tolerance"]
                yaw_tol = current_params_dict["reconfigure"]["yaw_tolerance"]
                x_only_tolerance = current_params_dict["reconfigure"][
                    "x_only_tolerance"
                ]
                via_xy_tol = current_params_dict["via_point_xy_tol"]
                via_yaw_tol = current_params_dict["via_point_yaw_tol"]

                # Check robot position nearly target goal (Use for via point, pass point)
                tol = self.check_via_point_tol(
                    frame_id,
                    x_only_tolerance,
                    current_target_pose.pose,
                    xy_tol,
                    yaw_tol,
                    via_xy_tol,
                    via_yaw_tol,
                )
                self.tolerance_pub.publish(
                    StringStamped(stamp=rospy.Time.now(), data=tol.toString())
                )
                if (
                    tol == ToleranceType.BOTH
                    or tol == ToleranceType.X_ONLY
                    or is_rotate_only
                    and tol == ToleranceType.YAW
                    or (check_action_status == True and use_follow_line)
                ):
                    # first_print_timeout use to ignore clear first line (only clear timeout has just printed)
                    # if not is_retry and first_print_timeout:
                    #     # Cần đảm bảo trong toàn bộ quá trình in xóa dòng, không có thông tin nào in xen vào giữa
                    # TODO: In xóa dòng khi re-size terminal window
                    #     sys.stdout.write("\033[F")
                    if not is_retry:
                        first_print_timeout = True
                        _timeout = round(
                            rospy.get_time() - begin_check_timeout, 2
                        )
                        # print('Timeout: {} sec, self.current_point: {}/{}, check_action_status: {}, move_action_result: {}'\
                        #     .format(_timeout, self.current_point+1, total_point, check_action_status, self.move_action_result))

                    # Break control (only active break when nearly goal position to avoid unnecessary break)
                    # Now active break when tolerance <= 0.05 # TODO: control break from waypoint param
                    if xy_tol <= 0.05:
                        self.break_control_pub.publish(NEED_BREAK)
                    else:
                        self.break_control_pub.publish(NO_NEED_BREAK)

                    # Final point or tolerance < 0.06 or yaw_changed or rotate_point
                    if (
                        self.current_point == total_point - 1
                        or xy_tol < 0.06
                        or "modify_status"
                        in data_dict["waypoints"][self.current_point]["params"]
                    ):  # TODO: or next point is rotate
                        if check_action_status == True:
                            if self.move_action_result == goal_result.SUCCEEDED:
                                # check_action_status = False # TESTING: disable check this flag to test
                                rospy.loginfo(
                                    "ACCURACY - move_base_status: %s"
                                    % self.move_action_result
                                )
                                _state = MainState.DONE
                                if is_retry:
                                    _state = MainState.RE_SEND_GOAL
                                    is_retry = False
                    elif (
                        check_action_status == True
                        and self.move_action_result == goal_result.SUCCEEDED
                    ):
                        # check_action_status = False
                        _state = MainState.DONE
                        rospy.loginfo(
                            "TOLERANCE OK - move_action_result: %s"
                            % self.move_action_result
                        )
                        if is_retry:
                            _state = MainState.RE_SEND_GOAL
                            is_retry = False

                    # Check timeout ['timeout'] second when nearly target but not yet reach
                    if (
                        rospy.get_time() - begin_check_timeout
                        > current_params_dict["timeout"]
                        and current_params_dict["timeout"] > 0.0
                        and not is_retry
                    ):
                        begin_check_timeout = rospy.get_time()
                        first_print_timeout = False
                        _state = MainState.RETURN_TO_RETRY
                else:  # self.move_action_result = goal_result.SUCCEEDED when goal not reached
                    if check_action_status == True and not use_follow_line:
                        if self.move_action_result == goal_result.SUCCEEDED:
                            if not is_retry:
                                # Cannot set Moving_Param.*_Goal_Tolerance > GOAL_TOLERANCE_*
                                rospy.loginfo(
                                    "OUT_OF_TOLERANCE - move_base_status: %s"
                                    % self.move_action_result
                                )
                                # check_action_status = False
                                _state = MainState.DONE
                                rospy.loginfo(
                                    "Retry point because OUT_OF_TOLERANCE: %s"
                                    % str(self.current_point + 1)
                                )
                            else:
                                _state = MainState.RETURN_TO_RETRY

                # Action ABORTED check
                if (
                    check_action_status == True
                    and self.move_action_result == goal_result.ABORTED
                ):
                    move_base_retry_cnt += 1
                    rospy.logwarn(
                        "Retry move_base when ABORTED: {} times".format(
                            move_base_retry_cnt
                        )
                    )
                    if move_base_retry_cnt >= max_retry_time:
                        rospy.logerr(
                            "Error after try {} times".format(max_retry_time)
                        )
                        _state = MainState.ERROR
                        self.cancel_all_action()
                    else:
                        _state = MainState.RE_SEND_GOAL
                    # check_action_status = False

            # State: RETURN_TO_RETRY # TODO: not yet use
            elif _state == MainState.RETURN_TO_RETRY:
                is_retry = True
                retry_pose = copy.deepcopy(current_target_pose)
                retry_pose.pose = offset_pose_x(
                    current_target_pose, current_params_dict["retry_distance"]
                )
                # TODO: Update tolerance params here
                # ['xy_tol'] = 0.1
                # ['yaw_tol'] = 0.1

                action_goal.target_pose = current_target_pose
                if (
                    last_action_client != None
                    and last_local_planner != local_planner
                ):
                    print_debug("Cancel all goal of: {}".format(action_client))
                    last_action_client.cancel_all_goals()
                action_client.send_goal(action_goal, feedback_cb=self.action_fb)
                last_action_client = action_client
                last_local_planner = local_planner
                self.last_action_fb = rospy.get_time()
                goal_sent_time = rospy.get_time()
                _state = MainState.MOVING  # TOCHECK: do not jump to MOVING
            # State: DONE
            elif _state == MainState.DONE:
                move_base_retry_cnt = 0
                planner_resend_cnt = 0
                # Measure running time
                duration = rospy.get_time() - start_time
                rospy.loginfo("Done with %i(s)", duration)
                # Next waypoint
                self.current_point += 1
                if self.current_point == len(data_dict["waypoints"]) - 1:
                    self.last_state_use_follow_line = False
                    success = True
                    break
                else:
                    # Remove finished waypoint
                    wp_display.pop(0)
                    self.waypoints_following_pub.publish(
                        pose_stamped_array_to_pose_array(wp_display, frame_id)
                    )
                    _state = MainState.SEND_GOAL
            # State: PAUSED
            elif _state == MainState.PAUSED:
                self._asm.module_status = ModuleStatus.PAUSED
                if self._asm.resume_req:
                    self._asm.reset_flag()
                    _state = MainState.RE_SEND_GOAL
                    # print_debug("Resume waypoint number: {}".format(self.current_point+1))
            # State: ERROR
            elif _state == MainState.ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = (
                    self._asm.error_code
                ) = "/moving_control: {}".format(_state.toString())
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    planner_resend_cnt = 0
                    _state = MainState.RE_SEND_GOAL
            # State: FIND_LINE_ERROR
            elif _state == MainState.FIND_LINE_ERROR:
                self._asm.module_status = ModuleStatus.ERROR
                self._asm.error_code = (
                    self._asm.error_code
                ) = "/moving_control: {}".format(_state.toString())
                if self._asm.reset_error_request:
                    self._asm.reset_flag()
                    _state = MainState.FIND_LINE

            # Update prev state
            # self.last_safety_fields = self.safety_fields

        self._asm.action_running = False

        if success:
            rospy.loginfo("%s: Succeeded" % self._action_name)
            self._as.set_succeeded(self._result)

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
        moving_status_time = rospy.get_time()
        status_msg = StringStamped()
        while not rospy.is_shutdown():
            r.sleep()
            if not self._asm.action_running:
                self._asm.module_status = ModuleStatus.WAITING
                self._asm.module_state = MainState.WAITING.toString()
                self._asm.error_code = ""
            now = rospy.get_time()
            if now - moving_status_time >= 0.5:
                moving_status_time = now
                status_msg.stamp = rospy.Time.now()
                status_msg.data = json.dumps(
                    {
                        "status": self._asm.module_status.toString(),
                        "state": self._asm.module_state,
                        "error_code": self._asm.error_code,
                    }
                )
                self._asm.module_status_pub.publish(status_msg)


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
        "--planner_map",
        dest="planner_map",
        default=os.path.join(
            rospkg.RosPack().get_path("moving_control"),
            "cfg",
            "planner_map.yaml",
        ),
        type=str,
        help="planner_map config file path",
    )
    parser.add_option(
        "--planner_setting",
        dest="planner_setting",
        default=os.path.join(
            rospkg.RosPack().get_path("moving_control"),
            "cfg",
            "planner_setting.yaml",
        ),
        type=str,
        help="planner_setting config file path",
    )
    parser.add_option(
        "--default_params",
        dest="default_params",
        default=os.path.join(
            rospkg.RosPack().get_path("moving_control"),
            "cfg",
            "default_params.json",
        ),
        type=str,
        help="default_param config file path",
    )

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)


def main():
    # if not get_ubuntu_serial_hash():
    #     print_error("Moving control: License verification failed.")
    #     return
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG
    rospy.init_node("moving_control", log_level=log_level, disable_signals=True)
    rospy.loginfo("Init node " + rospy.get_name())
    MovingControl(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
