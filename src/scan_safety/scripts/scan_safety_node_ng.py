#!/usr/bin/env python
#!
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import rospy
import rospkg
from sensor_msgs.msg import LaserScan
from math import cos, sin, sqrt
from math import pi as PI
from polar_utils import *
from safety_msgs.msg import SafetyStatus
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
# from scan_safety.cfg import SafetyScanConfig, Job0Config #, Job2Config, Job3Config, Job4Config
from scan_safety.cfg import SafetyScanNgConfig, SafetyScanConfig
from std_msgs.msg import Time, Int16
from geometry_msgs.msg import Twist
import threading
import copy
import os
import json
from std_stamped_msgs.msg  import StringStamped, StringFeedback, StringResult, StringAction
import actionlib

# TODO: Update footprint to cover leg of cart (ignore obstacles in footprint)
common_func_dir = os.path.join(rospkg.RosPack().get_path('agv_common_library'), 'scripts')
if not os.path.isdir(common_func_dir):
    common_func_dir = os.path.join(rospkg.RosPack().get_path('agv_common_library'), 'release')
sys.path.insert(0, common_func_dir)

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
    get_line_info
)

class ScanSafetyNode():
    _feedback = StringFeedback()
    _result = StringResult()

    def __init__(self):
        logLevel = 2
        print("Args: {}".format(sys.argv))
        if len(sys.argv)>3:
            if int(sys.argv[1])>0 and int(sys.argv[1]):
                logLevel = int(sys.argv[1])
        rospy.init_node("scan_safety_node_ng", log_level=logLevel)
        rospy.loginfo("Init node %s with log_level = %i"%(rospy.get_name(), logLevel))

        self.cfgChangedPub = rospy.Publisher("/scan_safety_cfg_changed_ng", Time, queue_size=10)
        # scan_topic = rospy.get_param("~scan_topic", '/scan')

        # Load default setting
        self.dir_path = os.path.dirname(os.path.realpath(__file__))
        # print_debug("current dir: " + self.dir_path)
        f = open(self.dir_path + "/default.json")
        self.defaultData = json.load(f)
        f.close()
        self.job = {} # self.job = {"safety":[{"index":0,"data":[[,],[]...]},{"index":1,"data":[[,],[]...]}....]
        self.job["safety"] = self.defaultData["safety"]
        self.footprint={} # self.footprint = {"footprint":[[,],[],...]}
        self.footprint["footprint"] = self.defaultData["footprint"]
        self.updateFields(self.job)
        self.updateFootprint(self.footprint)

        # print_debug("Default job: {}".format(json.dumps(self.job, indent=2)))

        self.obstacle = [0]*self.number_of_fields
        self.obstacle.append(111)
        self.slice_arr = [[]]*self.number_of_fields
        self.slice_size = rospy.get_param("~slice_size", 5)
        self.min_ray = rospy.get_param("~min_ray", 3)
        self.use_ultrasonic = rospy.get_param("~use_ultrasonic", False)
        self.use_move_base = rospy.get_param("~use_move_base", False)
        self.ultrasonic_min_range = rospy.get_param("~ultrasonic_min_range", 0.25)
        self.ultrasonic_offset = rospy.get_param("~ultrasonic_offset", 0.5)
        ultrasonic_topic = rospy.get_param("~ultrasonic_topic", '/ultrasonic/ultrasonic_ranges')
        self.ultrasonic_status = None
        self.checkObstacleSizeConfig()

        rospy.Subscriber("/scan", LaserScan, self.scanCallback)
        rospy.Subscriber("/safety_job", StringStamped, self.receiveJobCb)
        rospy.Subscriber("/safety_footprint", StringStamped, self.receiveFootprintCb)
        rospy.Subscriber("/safety_set_default", StringStamped, self.receivedSetDefault)
        if self.use_ultrasonic:
            from ultra_sonic.msg import UltrasonicRanges
            rospy.Subscriber(ultrasonic_topic, UltrasonicRanges, self.ultrasonicCallback)
            self.ultraSafetyStatusPub = rospy.Publisher("/ultrasonic_safety_status", SafetyStatus, queue_size=10)
            self.ultrasonic_config()
        self.scanSafetyStatusPub = rospy.Publisher("/scan_safety_status", SafetyStatus, queue_size=10)
        self.safetyStatusPub = rospy.Publisher("/safety_status", SafetyStatus, queue_size=10)
        srv = Server(SafetyScanNgConfig, self.reconfigureCB)

        self._ac_set_footprint = actionlib.SimpleActionServer('/set_footprint', StringAction, execute_cb=self.set_footprint_cb, auto_start=False)
        self._ac_set_footprint.start()
        self._ac_set_safety_job = actionlib.SimpleActionServer('/set_safety_job', StringAction, execute_cb=self.set_safety_job_cb, auto_start=False)
        self._ac_set_safety_job.start()

        rospy.logdebug("Main thread: %s"%threading.current_thread())
        while not rospy.is_shutdown():
            self.pubSafetyStatus(self.obstacle)
            rospy.sleep(0.1)

    def ultrasonic_config(self):
        rospy.loginfo("Set ultrasonic config")
        self.ultrasonic_sensors = rospy.get_param("~ultrasonic_sensors", dict({}))
        if python3:
            for sensor, params in self.ultrasonic_sensors.items():
                self.set_config(params)
        else:
            for sensor, params in self.ultrasonic_sensors.iteritems():
                self.set_config(params)
        print_debug("ultrasonic_sensors: \n{}".format(json.dumps(self.ultrasonic_sensors, indent=2)))

    def set_config(self, params):
        params["ranges"] = [0.0] * len(self.job["safety"])
        for i in range(len(self.job["safety"])):
            if params["x_from_base_link"] >= 0.0:
                params["ranges"][i] = self.get_max_range_x(self.job["safety"][i]["data"]) - params["x_from_base_link"]
            else:
                params["ranges"][i] = abs(self.get_min_range_x(self.job["safety"][i]["data"])) - abs(params["x_from_base_link"])
                #
            if params["ranges"][i] < 0.0: # Disable safety
                params["ranges"][i] = 0.0
            elif params["ranges"][i] < self.ultrasonic_min_range + self.ultrasonic_offset:
                params["ranges"][i] = self.ultrasonic_min_range + self.ultrasonic_offset

    def get_max_range_x(self, data_array):
        max = 0.0
        for i in data_array:
            if i[0] > max:
                max = i[0]
        print_info("max: {}".format(max))
        return max

    def get_min_range_x(self, data_array):
        min = 0.0
        for i in data_array:
            if i[0] < min:
                min = i[0]
        print_info("min: {}".format(min))
        return min

    """
     ######     ###    ##       ##       ########     ###     ######  ##    ##
    ##    ##   ## ##   ##       ##       ##     ##   ## ##   ##    ## ##   ##
    ##        ##   ##  ##       ##       ##     ##  ##   ##  ##       ##  ##
    ##       ##     ## ##       ##       ########  ##     ## ##       #####
    ##       ######### ##       ##       ##     ## ######### ##       ##  ##
    ##    ## ##     ## ##       ##       ##     ## ##     ## ##    ## ##   ##
     ######  ##     ## ######## ######## ########  ##     ##  ######  ##    ##
    """

    def receiveJobCb(self, msg):
        rospy.loginfo("Set safety job")
        # print(msg.data)
        self.job = json.loads(msg.data)
        # print(self.job)
        print(json.dumps(self.job))
        rospy.set_param("~job", json.dumps(self.job))

        # print(msgJson)
        self.updateFields(self.job)

    def receiveFootprintCb(self, msg):
        rospy.loginfo("Set footprint")
        # print(msg.data)
        self.footprint = json.loads(msg.data)
        print(json.dumps(self.footprint))
        rospy.set_param("~footprint", json.dumps(self.footprint))
        self.updateFootprint(self.footprint)

    def receivedSetDefault(self, msg):
        if msg.data == "save":
            self.defaultData.clear()
            self.defaultData.update(self.job)
            self.defaultData.update(self.footprint)
            f = open(self.dir_path + "/default.json", 'w')
            json.dump(self.defaultData, f, indent=4)
            f.close()
            rospy.loginfo("Safety data was set as default!")

    def ultrasonicCallback(self, msg):
        self.ultrasonic_status = SafetyStatus()
        if len(self.job["safety"]) != 0:
            self.ultrasonic_status.fields = [0] * len(self.job["safety"])
            if python3:
                for sensor, params in self.ultrasonic_sensors.items():      # Check each sensor
                    if "ranges" not in params: break
                    self.set_safety_value(msg, params)
            else:
                for sensor, params in self.ultrasonic_sensors.iteritems():  # Check each sensor
                    if "ranges" not in params: break
                    self.set_safety_value(msg, params)
        self.ultrasonic_status.header.stamp = rospy.Time.now()
        self.ultraSafetyStatusPub.publish(self.ultrasonic_status)

    def set_safety_value(self, msg, params):
        for i in range(len(params["ranges"])):                      # Check each range in the sensor
            distance = float(msg.ranges[params["index"]]/1000.0)    # mm to m
            if self.ultrasonic_min_range < distance and distance < params["ranges"][i]:
                self.ultrasonic_status.fields[i] = 1

    def reconfigureCB(self, config, level):
        rospy.loginfo("Update from scan reconfigure")
        rospy.logdebug("reconfig CB thread: %s"%threading.current_thread())

        jobStr = json.dumps(self.job)
        if config.job == "":
            config.job = jobStr
            job_dict = json.dumps(self.job, indent=2)
            if "params" in job_dict:
                print_warn("Bug - Reconfigure job = Empty:\n{}".format(json.dumps(self.job, indent=2)))
                self.updateFields(json.loads(job_dict)["params"])
            else:
                self.updateFields(self.job)
        if config.job != jobStr:
            self.job = json.loads(config.job)
            job_dict = json.dumps(self.job, indent=2)
            if "params" in job_dict:
                print_warn("Bug - Reconfigure job change:\n{}".format(json.dumps(self.job, indent=2)))
                self.updateFields(json.loads(job_dict)["params"])
            else:
                self.updateFields(self.job)
        # rospy.set_param("~")
        self.slice_size = config.slice_size
        self.min_ray= config.min_ray
        self.checkObstacleSizeConfig()

        if config.footprint == "":
            footprintStr =  json.dumps(self.footprint)
            config.footprint = footprintStr
            self.updateFootprint(self.footprint)
        else:
            footprintStr = '{"footprint": [[0.6, 0.5], [-0.6, 0.5], [-0.6, -0.5], [0.6, -0.5], [0.6, 0.5]]}'
            # footprintStr = config.footprint
            footprintJs = json.loads(footprintStr)
            self.updateFootprint(footprintJs)
        rospy.set_param("~footprint", config.footprint)

        return config

    def checkObstacleSizeConfig(self):
        if self.min_ray > self.slice_size:
            rospy.logerr("Condition fail. min_ray bigger than slice_size")

    def updateFields(self, jsonInput):
        """ Called when init node or change job, change field
            Calculate some factors and append into self.Fields
            self.Fields = [field1[zone1[p, phi, theta_P, theta_Q, k, i],...], field2[zone1..
        """
        # print_debug("Json input:\n{}".format(json.dumps(jsonInput, indent=2)))
        # self.number_of_fields = rospy.get_param("~job_%i/number_of_fields"%self.current_job, 3)

        self.number_of_fields = len(jsonInput["safety"])
        N_fields = self.number_of_fields
        rospy.loginfo("Number of fields: %i"%N_fields)
        # self.obstacle = [0]*N_fields

        # self.obstacle.append(-111)
        # print(self.obstacle)
        self.slice_arr = [[]]*N_fields

        self.Vertexes = []  # [vertexs1[point1[x,y], ...],[vertexs2[... ]
        self.Fields = []     # [field1[zone1[p, phi, theta_P, theta_Q, k, i],...], field2[zone1..]
        try:
            # Duyet qua cac vung an toan (vung 1, vung 2, vung 3)
            for i in range(N_fields):
                # param_str = rospy.get_param("~job_%i/field_%i"%(self.current_job, i+1), '')
                # vertexes = str2arr(param_str)
                vertexes = jsonInput["safety"][i]["data"]
                self.Vertexes.append(vertexes)
                zone = []
                """zone[[p, phi, theta_P, theta_Q, k, i]]
                Trong do:
                    p, phi: Hai he so trong PT duong thang qua 2 diem P, Q
                            trong he toa do Polar r = p/(cos(theta - phi))
                    theta_P, theta_Q: Goc cua diem P va diem Q
                    k: He so de xet goc tao boi OP va OQ la goc nhon hay goc tu
                    i: index cua canh duoc xet
                """
                for i in range(len(vertexes) - 1):
                    P = vertexes[i]
                    Q = vertexes[i+1]
                    p, phi = zone_from_two_points(P, Q)
                    theta_P = point_descart_to_polar(P)[1]
                    theta_Q = point_descart_to_polar(Q)[1]
                    k = 1 if (abs(theta_Q - theta_P) < PI) else -1

                    zone.append([p, phi, theta_P, theta_Q, k, i])
                self.Fields.append(zone)
            self.emitCfgChanged()
        except Exception as e:
            rospy.logerr(e)

    def updateFootprint(self, jsonInput):
        FootprintEdges = []
        Footprint = jsonInput["footprint"]
        for i in range(len(Footprint)-1):
            P = Footprint[i]
            Q = Footprint[i+1]
            p, phi = zone_from_two_points(P, Q)
            theta_P = point_descart_to_polar(P)[1]
            theta_Q = point_descart_to_polar(Q)[1]
            #TODO: add comment
            k = 1 if (abs(theta_Q - theta_P) < PI) else -1

            FootprintEdges.append([p, phi, theta_P, theta_Q, k, i])
        self.emitCfgChanged()
        self.FootprintEdges = copy.deepcopy(FootprintEdges)

    def emitCfgChanged(self):
        msg = Time()
        msg.data = rospy.Time.now()
        self.cfgChangedPub.publish(msg)

    def scanCallback(self, msg):
        N_fields = self.number_of_fields
        obs = [0]*N_fields
        Fields = copy.deepcopy(self.Fields)
        FootprintEdge = copy.deepcopy(self.FootprintEdges)
        ranges_raw = list(msg.ranges)
        N_ranges_raw = len(ranges_raw)
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min

        ranges = []
        angles = []
        # Duyet qua tat ca cac tia laser, neu diem phan hoi thuoc vung footprint thi loai bo khoi mang
        for i in range(N_ranges_raw):
            angle = standard_angle(angle_min + i*angle_increment)
            in_ft = False
            for z in FootprintEdge:
                r = z[0]/cos(angle - z[1])
                s_theta = (angle - z[2])*(angle - z[3])*z[4]
                if (s_theta <= 0) and ranges_raw[i] <= r:
                    in_ft = True
                    break
            if not in_ft:
                ranges.append(ranges_raw[i])
                angles.append(angle)
        N_ranges = len(ranges)
        # rospy.logdebug("raw: %i, N_ranges: %i"%(N_ranges_raw, N_ranges))

        try:
            # Loop 1: Duyet qua cac vung an toan (vung 1, 2, 3)
            for i in range(N_fields):
                # Loop 2: Duyet qua cac tia laser tu 0->360 do
                for laser_range, angle in zip(ranges, angles):
                    tmp_obs = 0
                    # Loop 3: Duyet qua cac canh cua vung cam bien
                    for z in Fields[i]:
                        R = z[0]/cos(angle-z[1])
                        s_theta = (angle - z[2])*(angle - z[3])*z[4]
                        if (s_theta <= 0) and laser_range <= R:
                            tmp_obs = 1
                            # print("laser_range: %.2f, R: %.2f, s_theta: %i"%(laser_range,R,s_theta))
                            # rospy.logdebug(tmp_obs)
                            break
                    self.slice_arr[i].append(tmp_obs)
                    # rospy.logdebug("slice_arr: {}".format(self.slice_arr))
                    # Fix lenght of slice_arr by cut items in front of array
                    if len(self.slice_arr[i]) > self.slice_size:
                        self.slice_arr[i] = self.slice_arr[i][-self.slice_size:]

                    if sum(self.slice_arr[i]) > self.min_ray:
                        for k in range(i, N_fields):
                            obs[k] = 1
                        # obs.append(rospy.Time.now().to_sec())
                        # rospy.logdebug("obs: {}".format(obs))
                        break
                if(sum(obs) > 0):
                    break
            self.obstacle = copy.deepcopy(obs)
            rospy.logdebug("obs->self.obs: {}".format(obs))
        except Exception as e:
            rospy.logwarn(e)
        # rospy.logdebug("Stop: %f"%rospy.Time.now().to_sec())
        # exit()

    def pubSafetyStatus(self, obstacle):
        msg = SafetyStatus()
        msg.job = 0
        msg.fields = copy.deepcopy(obstacle)
        msg.header.stamp = rospy.Time.now()
        self.scanSafetyStatusPub.publish(msg)
        if self.use_ultrasonic == False:
            self.safetyStatusPub.publish(msg)
        else:
            # Merge scan vs ultrasonic
            if self.ultrasonic_status == None: return
            if len(msg.fields) != len(self.ultrasonic_status.fields): return
            # Check ultrasonic disconnect
            if rospy.Time.now() - self.ultrasonic_status.header.stamp < rospy.Duration(0.5):
                for i in range(len(msg.fields)):
                    msg.fields[i] = 1 if self.ultrasonic_status.fields[i] == 1 or msg.fields[i] == 1 else 0
            self.safetyStatusPub.publish(msg)

    def send_feedback(self, action, msg):
        self._feedback.data = msg
        action.publish_feedback(self._feedback)

    def set_safety_job_cb(self, goal):
        rospy.loginfo("Set safety job received:\n{}".format(goal.data))

        try:
            goal_dict = json.loads(goal.data)
            msg = StringStamped()
            msg.data = json.dumps(goal_dict["params"], indent=2)
            self.send_feedback(self._ac_set_safety_job, "ACTIVE")
        except:
            self._ac_set_safety_job.set_aborted(text="Goal input error")
            return

        try:
            self.receiveJobCb(msg)
            self.ultrasonic_config()
            self._ac_set_safety_job.set_succeeded()
        except:
            self._ac_set_safety_job.set_aborted(text="Execute error")

    def set_footprint_cb(self, goal):
        rospy.loginfo("Set footprint received:\n{}".format(goal.data))

        try:
            goal_dict = json.loads(goal.data)
            msg = StringStamped()
            len_of_points = len(goal_dict["params"]["footprint"])
            if len_of_points >= 2:
                first_point = goal_dict["params"]["footprint"][0]
                last_point = goal_dict["params"]["footprint"][len_of_points-1]
                if first_point[0] != last_point[0] or first_point[1] != last_point[1]:
                    rospy.logwarn("Set footprint: auto add last point same first point")
                    goal_dict["params"]["footprint"].append(first_point)
                footprint_arr = str(goal_dict["params"]["footprint"])
                if self.use_move_base:
                    rospy.loginfo("move_base footprint: {}".format(footprint_arr))
                    param_dict = {"footprint": footprint_arr}
                    client = Client("/move_base/global_costmap/", timeout=1)
                    client.update_configuration(param_dict)
                    client = Client("/move_base/local_costmap/", timeout=1)
                    client.update_configuration(param_dict)
                rospy.set_param("/move_base/TebLocalPlannerROS/footprint_model/vertices", goal_dict["params"]["footprint"])

            msg.data = json.dumps(goal_dict["params"], indent=2)
            self.send_feedback(self._ac_set_footprint, "ACTIVE")
        except:
            self._ac_set_footprint.set_aborted(text="Goal input error")
            return

        try:
            self.receiveFootprintCb(msg)
            self._ac_set_footprint.set_succeeded()
        except:
            self._ac_set_footprint.set_aborted(text="Execute error")

if __name__ == "__main__":
    scan_safety = ScanSafetyNode()
