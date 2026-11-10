#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import json
import yaml
import threading
import argparse
import rospy
import rospkg
import tf
import copy
from pathlib import Path
import time
from nav_msgs.msg import OccupancyGrid
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    PoseArray,
    PoseWithCovarianceStamped,
)
from std_msgs.msg import Int8, String, Empty
from std_stamped_msgs.msg import (
    StringStamped,
    StringFeedback,
    StringResult,
    StringAction,
)
from std_stamped_msgs.srv import StringService, StringServiceResponse

common_func_dir = os.path.join(
    rospkg.RosPack().get_path("agv_common_library"), "scripts"
)
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(
        rospkg.RosPack().get_path("agv_common_library"), "release"
    )
sys.path.insert(0, common_func_dir)

agv_mongodb_dir = os.path.join(
    rospkg.RosPack().get_path("agv_mongodb"), "scripts"
)
if not os.path.isdir(agv_mongodb_dir):
    agv_mongodb_dir = os.path.join(
        rospkg.RosPack().get_path("agv_mongodb"), "release"
    )
sys.path.insert(0, agv_mongodb_dir)

from mongodb import mongodb
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
)


def get_node_list():
    """This function take 1 sec"""
    nodes = os.popen("rosnode list").readlines()
    for i in range(len(nodes)):
        nodes[i] = nodes[i].replace("\n", "")
    return nodes


def kill_rosnode(node_name):
    cmd = "rosnode kill {}".format(node_name)
    kill_node = os.popen(cmd).readlines()
    if len(kill_node) == 2:
        if "killed" in kill_node[1]:
            print_debug('Killed node "{}"'.format(node_name))


class MainState(EnumString):
    NONE = -1
    INIT = 0
    SWITCH_MAPPING = 1
    MAPPING = 2
    SWITCH_LOCALIZING = 3
    LOCALIZING = 4
    ERROR = 5
    INIT_LOCALIZING = 6
    INIT_MAPPING = 7
    ERROR_MAPPING = 8
    ERROR_LOCALIZING = 9


class SlamManager(object):
    def __init__(self, name, *args, **kwargs):
        self.init_variable(*args, **kwargs)
        rospy.on_shutdown(self.shutdown)
        # Initial
        self.slam_init(kwargs["config_file"])
        # Publisher
        self.reset_odom_pub = rospy.Publisher(
            "/reset_odom", Empty, queue_size=5
        )
        self.initial_pose_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=5
        )
        # Subscriber
        rospy.Subscriber(
            "/mapping_request", StringStamped, self.mapping_request_cb
        )
        rospy.Subscriber(
            "/map_load_request", StringStamped, self.map_load_request_cb
        )
        rospy.Subscriber(
            "/map_save_request", StringStamped, self.map_save_request_cb
        )
        rospy.Subscriber(
            "/temp_savemap_request", StringStamped, self.map_temp_savemap_cb
        )
        rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        # ModuleServer
        self._asm = ModuleServer(name)
        # Service
        rospy.Service("check_map", StringService, self.handle_check_map)
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
        myargv = rospy.myargv(argv=sys.argv)
        print(myargv)
        self.subfix = ""
        if myargv[0][-3:] == ".py":
            self.subfix = "py"
        if myargv[0][-4:] == ".bin":
            self.subfix = "bin"
        print_debug("Subfix: {}".format(self.subfix))
        self.simulation = kwargs["simulation"]
        print_debug("simulation: {}".format(self.simulation))
        self.simulation_str = "true" if self.simulation else "false"
        # TF
        self.tf_listener = tf.TransformListener()
        self.map_frame = "map"
        self.odom_frame = "odom"
        self.robot_base = "base_footprint"
        # Flag and state
        self.main_state = MainState.INIT
        self.prev_state = MainState.NONE
        self.mapping_request = False
        self.load_map_request = False
        self.current_map = ""
        self.map_received = False
        self.map_publisher_id = ""
        self.prev_map_publisher_id = ""
        self.time_from_last_tf_normal = 0.0
        self.map_load_requesting = ""
        self.first_localization = False
        self.transfrom_error_cnt = 0
        self.load_map_cnt = 0
        self.mapping_cnt = 0
        self.mapping_req_cnt = 0
        self.load_map_req_cnt = 0
        self.cal_localization_time = False
        self.cal_load_mapping_time = False
        self.transform_error = False
        self.last_transform_normal = rospy.get_time()
        # DB config
        mongodb_address = rospy.get_param("/mongodb_address")
        rospy.loginfo(mongodb_address)
        self.db = mongodb(mongodb_address)
        #
        self.initial_pose = PoseWithCovarianceStamped()
        # fmt: off
        self.initial_pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.06853892326654787,
        ]
        # fmt: on

    def slam_init(self, cfg_file):
        try:
            with open(cfg_file) as file:
                self.slam_config_dict = yaml.load(file, Loader=yaml.Loader)
        except Exception as e:
            rospy.logerr("slam_init: {}".format(e))

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

    def load_tf(self):
        if "transform" in self.slam_config_dict:
            rospy.loginfo("Load TF")
            nodes = get_node_list()
            for n in self.slam_config_dict["transform"]["ros_node"]:
                if n in nodes:
                    kill_rosnode(n)
            tf_cmd = "{} simulation:={} subfix:={}".format(
                self.slam_config_dict["transform"]["launch_file"],
                self.simulation_str,
                self.subfix,
            )
            print_debug(tf_cmd)
            os.system(tf_cmd)
        else:
            rospy.logwarn("transform is not config")

    def do_mapping(self):
        rospy.loginfo("Begin mapping")
        nodes = get_node_list()
        for n in (
            self.slam_config_dict["mapping"]["ros_node"]
            + self.slam_config_dict["localization"]["ros_node"]
        ):
            if n in nodes:
                kill_rosnode(n)
        self.map_publisher_id = ""
        mapping_cmd = "{} simulation:={} subfix:={}".format(
            self.slam_config_dict["mapping"]["launch_file"],
            self.simulation_str,
            self.subfix,
        )
        print_debug(mapping_cmd)
        os.system(mapping_cmd)

    def do_map_save(self, map_file):
        rospy.loginfo("Save map file: {}".format(map_file))
        cmd = "{} {}".format(
            self.slam_config_dict["mapping"]["map_save_cmd"], map_file
        )
        print_debug(cmd)
        os.system(cmd)
        self.current_map = map_file
        rospy.loginfo("Map save image successful")
        # Download from db
        # self.download_map(map_file)
        # Save gridmap
        # self.check_and_gen_map(map_file, force_gen_map=True)

    def do_temp_map_save(self, map_file):
        rospy.loginfo("Save map file: {}".format(map_file))
        cmd = "{} {}".format(
            self.slam_config_dict["mapping"]["map_save_cmd"], map_file
        )
        print_debug(cmd)
        os.system(cmd)

    def do_map_load(self, map_file):
        rospy.loginfo("Load map file: {}".format(map_file))
        nodes = get_node_list()
        for n in (
            self.slam_config_dict["mapping"]["ros_node"]
            + self.slam_config_dict["localization"]["ros_node"]
        ):
            if n in nodes:
                kill_rosnode(n)
            else:
                print_debug('Node "{}" is not running'.format(n))
        self.map_publisher_id = ""
        self.current_map = map_file
        aruco_file_path = map_file + ".json"
        initial_pose_x, initial_pose_y, initial_pose_a = self.get_initial_pose()
        launch_localization_cmd = "{} simulation:={} map_file:={} initial_pose_x:={} initial_pose_y:={} initial_pose_a:={} aruco_file_path:={} subfix:={}".format(
            self.slam_config_dict["localization"]["launch_file"],
            self.simulation_str,
            map_file,
            initial_pose_x,
            initial_pose_y,
            initial_pose_a,
            aruco_file_path,
            self.subfix,
        )
        print_debug(launch_localization_cmd)
        os.system(launch_localization_cmd)

    def begin_load_map(self, map_file):
        # self.load_map_cnt += 1
        # print_debug("Load map time: {}".format(self.load_map_cnt))
        self.map_load_req = True
        self.first_localization = True
        self.map_load_thread = threading.Thread(
            target=self.do_map_load, args=(map_file,)
        )
        self.map_load_thread.start()
        self.tf_thread = threading.Thread(target=self.load_tf, args=())
        self.tf_thread.start()

    def begin_mapping(self):
        # self.mapping_cnt += 1
        # print_debug("Switch mapping time: {}".format(self.mapping_cnt))
        self.reset_odom_pub.publish(Empty())
        self.first_localization = True
        # self.initial_pose.header.stamp = rospy.Time.now()
        # self.initial_pose_pub.publish(self.initial_pose)
        self.tf_thread = threading.Thread(target=self.load_tf, args=())
        self.tf_thread.start()
        self.mapping_thread = threading.Thread(target=self.do_mapping)
        self.mapping_thread.start()

    def get_initial_pose(self):
        return 0.0, 0.0, 0.0

    def check_and_gen_map(self, map_file, force_gen_map=False):
        # TODO: When need download map?
        file_check = (
            map_file
            + "."
            + self.slam_config_dict["localization"]["map_file_ext"]
        )
        print_debug("Check map: {}".format(file_check))
        if Path(file_check).exists() and not force_gen_map:
            return True
        else:
            if not force_gen_map:
                rospy.logwarn(
                    "The requesting map is not exist. Try to create file: {}".format(
                        file_check
                    )
                )
            else:
                print_debug("Force gen map: {}".format(file_check))

            if "map_file_ext" in self.slam_config_dict["localization"]:
                map_file_ext = "map_file_ext: {}".format(
                    self.slam_config_dict["localization"]["map_file_ext"]
                )
                # TODO: Check other file exist?
                if "image_file_input" in self.slam_config_dict["localization"]:
                    image_file_input = self.slam_config_dict["localization"][
                        "image_file_input"
                    ]
                if (
                    "download_image_map"
                    in self.slam_config_dict["localization"]
                ):
                    download_image_map = self.slam_config_dict["localization"][
                        "download_image_map"
                    ]
                else:
                    download_image_map = True
                if download_image_map:
                    rospy.loginfo(
                        'Need download "{}" map'.format(image_file_input)
                    )
                else:
                    rospy.logwarn("Don't need download image map")
                if not Path(file_check).exists() or force_gen_map:
                    map_info = {}
                    # Download from db
                    if download_image_map:
                        map_info = self.download_map(map_file, image_file_input)
                    # Check input image file
                    downloaded_file = map_file + "." + image_file_input
                    begin_download = rospy.get_time()
                    while (
                        download_image_map
                        and not Path(downloaded_file).exists()
                        and rospy.get_time() - begin_download < 1.0
                    ):
                        # Download from db
                        map_info = self.download_map(map_file, image_file_input)
                        rospy.Rate(5.0).sleep()
                    if (
                        not Path(downloaded_file).exists()
                        and download_image_map
                    ):
                        rospy.logerr(
                            "Downloaded image file not exist: {}".format(
                                downloaded_file
                            )
                        )
                        return False
                    # Create map
                    if (
                        "convert_map_cmd"
                        in self.slam_config_dict["localization"]
                    ):
                        convert_map_cmd = self.slam_config_dict["localization"][
                            "convert_map_cmd"
                        ]
                        print_debug("Creating file: {}".format(file_check))
                        origin_x = map_info["info"]["origin"]["position"]["x"]
                        origin_y = map_info["info"]["origin"]["position"]["y"]
                        resolution = map_info["info"]["resolution"]
                        map_width = map_info["info"]["width"] * resolution
                        map_height = map_info["info"]["height"] * resolution
                        cx = origin_x + map_width / 2.0
                        cy = origin_y + map_height / 2.0
                        # image2gridmap -w -i $1.$2 --res $3 -o $4.gridmap --cx $5 --cy $6
                        # image2gridmap [-w] [–py <0.0>] [–px <0.0>] [–cy <0.0>] [–cx <0.0>] -r <0.1> [-o <map.gridmap.gz>] -i <map_image.png> [–] [–version] [-h]
                        cmd = "{} {} {} {} {} {} {}".format(
                            convert_map_cmd,
                            map_file,
                            image_file_input,
                            resolution,
                            map_file,
                            cx,
                            cy,
                        )
                        print_warn("Convert map cmd: {}".format(cmd))
                        os.system(cmd)

            if Path(file_check).exists():
                return True
            else:
                rospy.logerr("Create file error: {}".format(file_check))
                return False

    def download_map(self, map_file, image_file_type):
        [map_dir, map_file] = map_file.rsplit("/", 1)
        print_debug(
            'Downloading map: "{}.{}" to "{}"'.format(
                map_file, image_file_type, map_dir
            )
        )
        map_info = self.db.downloadMap(map_file, map_dir, image_file_type)
        return map_info

    def check_position(self):
        current_pose = lockup_pose(
            self.tf_listener, self.map_frame, self.robot_base
        )
        if current_pose == None:
            self.transfrom_error_cnt += 1
            if (
                self.transfrom_error_cnt >= 5
                and rospy.get_time() - self.last_transform_normal >= 1.0
            ):
                self.transform_error = True
            return
        self.first_localization = False
        self.transfrom_error_cnt = 0
        self.last_transform_normal = rospy.get_time()
        self.transform_error = False

    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def mapping_request_cb(self, msg):
        # self.mapping_req_cnt += 1
        # print_debug("Mapping request time: {}".format(self.mapping_req_cnt))
        self.mapping_request = True

    def map_load_request_cb(self, msg):
        # self.load_map_req_cnt += 1
        # print_debug("Load map request time: {}".format(self.load_map_req_cnt))
        if self.check_and_gen_map(msg.data, force_gen_map=True):
            self.load_map_request = True
            self.map_load_requesting = msg.data
        else:
            rospy.logerr("Map file not exist: {}".format(msg.data))

    def map_save_request_cb(self, msg):
        self.map_save_thread = threading.Thread(
            target=self.do_map_save, args=(msg.data,)
        )
        self.map_save_thread.start()

    def map_temp_savemap_cb(self, msg):
        temp_map_save_thread = threading.Thread(
            target=self.do_temp_map_save, args=(msg.data,)
        )
        temp_map_save_thread.start()

    def map_cb(self, msg):
        self.map_received = True
        self.map_publisher_id = msg._connection_header["callerid"]
        if self.prev_map_publisher_id != self.map_publisher_id:
            print_debug("/map callerid: {}".format(self.map_publisher_id))
        self.prev_map_publisher_id = self.map_publisher_id

    def handle_check_map(self, req):
        print_info(req.request)
        if self.check_and_gen_map(req.request):
            return StringServiceResponse("OK")
        else:
            return StringServiceResponse("NG")

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
        begin_mapping_time = rospy.get_time()
        begin_load_map_time = rospy.get_time()
        last_localization_err = False
        # Loop
        r = rospy.Rate(5.0)
        while not rospy.is_shutdown():
            localization_error = False
            self.check_position()
            self.time_from_last_tf_normal = (
                rospy.get_time() - self.last_transform_normal
            )
            if (self.transform_error and not self.first_localization) or (
                self.transform_error
                and self.first_localization
                and self.time_from_last_tf_normal > 30.0
            ):
                localization_error = True
                if last_localization_err != localization_error:
                    rospy.logerr(
                        "TF timeout is first: {}".format(
                            self.first_localization
                        )
                    )
                last_localization_err = localization_error

            if self.main_state != self.prev_state:
                print_debug(
                    "Main state: {} -> {}".format(
                        self.prev_state.toString(), self.main_state.toString()
                    )
                )
                self.prev_state = self.main_state

            # State: INIT
            if self.main_state == MainState.INIT:
                if self.load_map_request:
                    self.load_map_request = False
                    self.main_state = MainState.INIT_LOCALIZING
                if self.mapping_request:
                    self.mapping_request = False
                    self.main_state = MainState.INIT_MAPPING
                # If mrpt alive
                nodes = get_node_list()
                for n in self.slam_config_dict["localization"]["ros_node"]:
                    if n in nodes:
                        if not localization_error:
                            # Set current_map
                            if (
                                "map_file_param"
                                in self.slam_config_dict["localization"]
                            ):
                                self.current_map = rospy.get_param(
                                    self.slam_config_dict["localization"][
                                        "map_file_param"
                                    ],
                                    "",
                                )
                                if self.current_map != "":
                                    self.main_state = MainState.LOCALIZING
                                else:
                                    rospy.logerr(
                                        "ERROR_LOCALIZING: Cannot get current map"
                                    )
                                self.main_state = MainState.ERROR_LOCALIZING
                            else:
                                rospy.logerr(
                                    "ERROR_LOCALIZING: INIT WHEN LOCALIZING"
                                )
                                self.main_state = MainState.ERROR_LOCALIZING
                # If gmapping alive
                nodes = get_node_list()
                for n in self.slam_config_dict["mapping"]["ros_node"]:
                    if n in nodes:
                        if not localization_error:
                            self.main_state = MainState.MAPPING
                        else:
                            rospy.logerr("ERROR_LOCALIZING: INIT WHEN MAPPING")
                            self.main_state = MainState.ERROR_MAPPING
                # TF Error
                if localization_error:
                    rospy.logerr("ERROR_LOCALIZING: INIT")
                    self.main_state = MainState.ERROR_LOCALIZING
            # State: INIT_MAPPING
            elif self.main_state == MainState.INIT_MAPPING:
                self.cal_load_mapping_time = True
                self.transform_error = True
                begin_mapping_time = rospy.get_time()
                self.begin_mapping()
                self.main_state = MainState.SWITCH_MAPPING
            # State: SWITCH_MAPPING
            elif self.main_state == MainState.SWITCH_MAPPING:
                if (
                    rospy.get_time() - begin_mapping_time > 5.0
                    and not self.transform_error
                    and self.map_publisher_id
                    == self.slam_config_dict["mapping"]["ros_node"][0]
                ):
                    self.main_state = MainState.MAPPING
                if rospy.get_time() - begin_mapping_time > 30.0:
                    rospy.logerr("ERROR_MAPPING: wait map timeout")
                    self.main_state = MainState.ERROR_MAPPING
                # if localization_error:
                #     rospy.logerr("ERROR_MAPPING: TF")
                #     self.main_state = MainState.ERROR_MAPPING
            # State: INIT_LOCALIZING
            elif self.main_state == MainState.INIT_LOCALIZING:
                self.cal_localization_time = True
                self.transform_error = True
                begin_load_map_time = rospy.get_time()
                self.last_transform_normal = rospy.get_time()
                self.begin_load_map(self.map_load_requesting)
                self.main_state = MainState.SWITCH_LOCALIZING
            # State: SWITCH_LOCALIZING
            elif self.main_state == MainState.SWITCH_LOCALIZING:
                if (
                    rospy.get_time() - begin_load_map_time > 5.0
                    and not self.transform_error
                    and self.map_publisher_id
                    == self.slam_config_dict["localization"]["ros_node"][0]
                ):
                    self.main_state = MainState.LOCALIZING
                if rospy.get_time() - begin_load_map_time > 40.0:
                    rospy.logerr("ERROR_LOCALIZING: wait map timeout")
                    self.main_state = MainState.ERROR_LOCALIZING
                # if localization_error:
                #     rospy.logerr("ERROR_LOCALIZING: TF")
                #     self.main_state = MainState.ERROR_LOCALIZING
            # State: MAPPING
            elif self.main_state == MainState.MAPPING:
                if self.cal_load_mapping_time:
                    self.cal_load_mapping_time = False
                    rospy.loginfo(
                        "Switching to MAPPING take: {} seconds".format(
                            rospy.get_time() - begin_mapping_time
                        )
                    )
                if self.load_map_request:
                    self.load_map_request = False
                    self.main_state = MainState.INIT_LOCALIZING
                if self.mapping_request:
                    self.mapping_request = False
                    self.main_state = MainState.INIT_MAPPING
            # State: LOCALIZING
            elif self.main_state == MainState.LOCALIZING:
                if self.cal_localization_time:
                    self.cal_localization_time = False
                    rospy.loginfo(
                        "Switching to LOCALIZING take: {} seconds".format(
                            rospy.get_time() - begin_load_map_time
                        )
                    )
                if self.load_map_request:
                    self.load_map_request = False
                    self.main_state = MainState.INIT_LOCALIZING
                if self.mapping_request:
                    self.mapping_request = False
                    self.main_state = MainState.INIT_MAPPING
                if localization_error:
                    rospy.logerr("ERROR_LOCALIZING: TF")
                    self.main_state = MainState.ERROR_LOCALIZING
            # State: ERROR
            elif self.main_state == MainState.ERROR:
                pass
            # State: ERROR_MAPPING
            elif self.main_state == MainState.ERROR_MAPPING:
                if not self.transform_error:
                    self.main_state = MainState.MAPPING
                if self.load_map_request:
                    self.load_map_request = False
                    self.main_state = MainState.INIT_LOCALIZING
                if self.mapping_request:
                    self.mapping_request = False
                    self.main_state = MainState.INIT_MAPPING
                # If mrpt alive
                nodes = get_node_list()
                for n in self.slam_config_dict["localization"]["ros_node"]:
                    if n in nodes:
                        if not last_localization_err:
                            if (
                                "map_file_param"
                                in self.slam_config_dict["localization"]
                            ):
                                self.current_map = rospy.get_param(
                                    self.slam_config_dict["localization"][
                                        "map_file_param"
                                    ]
                                )
                                self.main_state = MainState.LOCALIZING
                # If gmapping alive
                nodes = get_node_list()
                for n in self.slam_config_dict["mapping"]["ros_node"]:
                    if n in nodes:
                        if not last_localization_err:
                            self.main_state = MainState.MAPPING
            # State: ERROR_LOCALIZING
            elif self.main_state == MainState.ERROR_LOCALIZING:
                if not localization_error:
                    self.main_state = MainState.LOCALIZING
                if self.load_map_request:
                    self.load_map_request = False
                    self.main_state = MainState.INIT_LOCALIZING
                if self.mapping_request:
                    self.mapping_request = False
                    self.main_state = MainState.INIT_MAPPING
                # If mrpt alive
                nodes = get_node_list()
                for n in self.slam_config_dict["localization"]["ros_node"]:
                    if n in nodes:
                        if not localization_error:
                            if (
                                "map_file_param"
                                in self.slam_config_dict["localization"]
                            ):
                                self.current_map = rospy.get_param(
                                    self.slam_config_dict["localization"][
                                        "map_file_param"
                                    ],
                                    "",
                                )
                                if self.current_map != "":
                                    self.main_state = MainState.LOCALIZING
                                else:
                                    self.main_state = MainState.ERROR_LOCALIZING
                # If gmapping alive
                nodes = get_node_list()
                for n in self.slam_config_dict["mapping"]["ros_node"]:
                    if n in nodes:
                        if not localization_error:
                            self.main_state = MainState.MAPPING

            msg = StringStamped()
            msg.data = {
                "status": "",
                "state": self.main_state.toString(),
                "current_map": self.current_map,
                "tf_time_normal": str(round(self.time_from_last_tf_normal)),
            }
            msg.data = json.dumps(msg.data)
            msg.stamp = rospy.Time.now()
            self._asm.module_status_pub.publish(msg)
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
    )
    parser.add_option(
        "-c",
        "--config_file",
        dest="config_file",
        default=os.path.join(
            rospkg.RosPack().get_path("slam_manager"), "cfg", "slam_config.yaml"
        ),
    )
    parser.add_option(
        "-d",
        "--ros_debug",
        action="store_true",
        dest="log_debug",
        default=False,
        help="log_level=rospy.DEBUG",
    )

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)


def main():
    (options, args) = parse_opts()
    log_level = None
    if options.log_debug:
        log_level = rospy.DEBUG

    rospy.init_node("slam_manager", log_level=log_level, disable_signals=True)
    rospy.loginfo("Init node " + rospy.get_name())

    # git un track db config file
    # [config_dir, db_config] = options.db_config.rsplit('/', 1)
    # print("Change dir to: \"{}\" and run \"git update-index --assume-unchanged {}\"".format(config_dir, db_config))
    # os.chdir(config_dir)
    # os.system("git update-index --assume-unchanged " + db_config)

    SlamManager(rospy.get_name(), **vars(options))


if __name__ == "__main__":
    main()
