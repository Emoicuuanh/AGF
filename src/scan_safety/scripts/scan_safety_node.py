#!/usr/bin/env python
#!
import sys
import rospy
from sensor_msgs.msg import LaserScan
from math import cos, sin, sqrt
from math import pi as PI
from polar_utils import *
from safety_msgs.msg import SafetyStatus
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
from scan_safety.cfg import SafetyScanConfig, Job0Config #, Job2Config, Job3Config, Job4Config
from std_msgs.msg import Time, Int16
from geometry_msgs.msg import Twist
import threading
import copy

#TODO: Update footprint to cover leg of cart (ignore obstacles in footprint)

class ScanSafetyNode():
    def __init__(self):
        rospy.loginfo("init node scan_safety")
        logLevel = 2
        print((sys.argv) )
        if int(sys.argv[1])>0 and int(sys.argv[1]):
            logLevel = int(sys.argv[1])
        rospy.init_node("scan_safety_node", log_level=logLevel)
        rospy.loginfo("Init node %s with log_level = %i"%(rospy.get_name(), logLevel))

        self.cfgChangedPub = rospy.Publisher("/scan_safety_cfg_changed", Time, queue_size=10)
        self.current_job = rospy.get_param("~current_job", 0)
        self.number_of_jobs = rospy.get_param("~number_of_jobs", 4)
        self.number_of_fields = 0
        self.VMaxs = []
        self.Fields_str = []
        self.FootprintEdges = []
        # Dynamic reconfigure server
        srv = Server(SafetyScanConfig, self.reconfigureCB)
        srvJob0 = Server(Job0Config, self.jobCfgCB, "job_0")

        self.obstacle = [0]*self.number_of_fields
        self.obstacle.append(111)
        self.slice_arr = [[]]*self.number_of_fields
        self.SliceSize = rospy.get_param("~slice_size", 5)
        self.MinRay = rospy.get_param("~min_ray", 3)
        self.checkObstacleSizeConfig()

        scan_topic = rospy.get_param("~scan_topic", '/scan')
        rospy.Subscriber(scan_topic, LaserScan, self.scanCallback)
        self.safetyStatusPub = rospy.Publisher("/safety_status", SafetyStatus, queue_size=10)

        self.vx = 0.0
        self.omega = 0.0
        self.velCBLastest = rospy.Time.now()
        rospy.Subscriber(self.cmd_vel_input, Twist, self.cmdVelCB)
        self.cmdVelPub = rospy.Publisher(self.cmd_vel_output, Twist, queue_size=10)

        rospy.logdebug("Main thread: %s"%threading.current_thread())
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            if self.is_publish_vel and sum(self.obstacle):
                if now - self.velCBLastest < rospy.Duration(0.15):
                    self.pubVel(self.vx, self.omega)
            rospy.logdebug("publish obs: {}".format(self.obstacle))
            self.pubSafetyStatus(self.obstacle)
            rospy.sleep(0.1)

    def cmdVelCB(self, msg):
        """ velocity callback
            - is vel out of maximum velocity? -> Publish v_max
        """
        _vx = msg.linear.x
        _omega = msg.angular.z

        obstacles = copy.deepcopy(self.obstacle[:3])
        vmaxs = copy.deepcopy(self.VMaxs)
        self.velCBLastest = rospy.Time.now()

        s_obs = sum(obstacles)
        if s_obs > 0:
            vmax = vmaxs[-s_obs]
            if abs(_vx) > vmax:
                k = (vmax/abs(_vx))
                _vx = vmax if _vx > 0 else -vmax
                _omega = _omega*k
        self.vx = _vx
        self.omega = _omega

    def pubVel(self, vx, omega):
        vel_out = Twist()
        vel_out.linear.x = vx
        vel_out.angular.z = omega
        self.cmdVelPub.publish(vel_out)

    def reconfigureCB(self, config, level):
        # rospy.logdebug(config.current_job)
        rospy.loginfo("Update from scan reconfigure")
        rospy.logdebug("reconfig CB thread: %s"%threading.current_thread())
        if config.current_job != self.current_job:
            self.current_job = config.current_job
            rospy.set_param("~current_job", config.current_job)
            self.updateFields()
        self.SliceSize = config.slice_size
        self.MinRay= config.min_ray
        self.checkObstacleSizeConfig()
        self.is_publish_vel = config.publish_vel
        self.cmd_vel_input = config.cmd_vel_input
        self.cmd_vel_output = config.cmd_vel_output
        rospy.set_param("~footprint", config.footprint)
        self.updateFootprint()
        return config

    def jobCfgCB(self, config, level):
        rospy.loginfo("Update from job reconfigure")
        # rospy.logdebug("jobCfgCB thread: %s"%threading.current_thread())
        # self.number_of_fields = config.number_of_fields
        rospy.set_param("~job_0/number_of_fields", config.number_of_fields)
        rospy.set_param("~job_0/field_1", config.field_1)
        rospy.set_param("~job_0/vmax_1", config.vmax_1)
        rospy.set_param("~job_0/field_2", config.field_2)
        rospy.set_param("~job_0/vmax_2", config.vmax_2)
        rospy.set_param("~job_0/field_3", config.field_3)
        rospy.set_param("~job_0/vmax_3", config.vmax_3)
        self.updateFields()
        return config

    def checkObstacleSizeConfig(self):
        if self.MinRay > self.SliceSize:
            rospy.logerr("Condition fail. min_ray bigger than slice_size")

    def updateFields(self):
        """ Called when init node or change job, change field
            Calculate some factors and append into self.Fields
            self.Fields = [field1[zone1[p, phi, theta_P, theta_Q, k, i],...], field2[zone1..
        """
        rospy.loginfo("Current job: %i"%self.current_job)
        self.number_of_fields = rospy.get_param("~job_%i/number_of_fields"%self.current_job, 3)
        N_fields = self.number_of_fields
        rospy.loginfo("Number of fields: %i"%N_fields)
        self.obstacle = [0]*N_fields
        self.obstacle.append(-111)
        self.slice_arr = [[]]*N_fields

        self.Vertexes = []  # [vertexs1[point1[x,y], ...],[vertexs2[... ]
        self.Fields = []     # [field1[zone1[p, phi, theta_P, theta_Q, k, i],...], field2[zone1..]
        try:
            # Duyet qua cac vung an toan (vung 1, vung 2, vung 3)
            for i in range(N_fields):
                param_str = rospy.get_param("~job_%i/field_%i"%(self.current_job, i+1), '')
                vertexes = str2arr(param_str)
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

            #TODO: send notification to show node
            self.emitCfgChanged()
            self.updateVMaxs()
        except Exception as e:
            rospy.logerr(e)

    def updateFootprint(self):
        # self.Footprint = []
        FootprintEdges = []

        fp_str = rospy.get_param("~footprint", '[[0.6, 0.5], [-0.6, 0.5], [-0.6, -0.5], [0.6, -0.5], [0.6, 0.5]]')
        Footprint = str2arr(fp_str)
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
        # rospy.logdebug("Start: %f"%rospy.Time.now().to_sec())
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
                    if len(self.slice_arr[i]) > self.SliceSize:
                        self.slice_arr[i] = self.slice_arr[i][-self.SliceSize:]

                    if sum(self.slice_arr[i]) > self.MinRay:
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
        msg.job = self.current_job
        msg.fields = copy.deepcopy(obstacle)
        msg.header.stamp = rospy.Time.now()
        self.safetyStatusPub.publish(msg)

    def updateVMaxs(self):
        self.is_publish_vel = rospy.get_param("~publish_vel", False)
        self.cmd_vel_input = rospy.get_param("~cmd_vel_input", '') #/cmd_vel')
        self.cmd_vel_output = rospy.get_param("~cmd_vel_output", '') #/safety_cmd_vel')
        rospy.logdebug("[updateVMasx]: is_publish_vel: %i", int(self.is_publish_vel))
        rospy.logdebug("[updateVMasx]: cmd_vel_input: %s", self.cmd_vel_input)
        rospy.logdebug("[updateVMasx]: cmd_vel_output: %s", self.cmd_vel_output)
        VMaxs = []
        for i in range(self.number_of_fields):
            vmax = rospy.get_param("~job_%i/vmax_%i"%(self.current_job, i+1), 1)
            VMaxs.append(vmax)
        self.VMaxs = copy.deepcopy(VMaxs)
        rospy.logdebug("[updateVMasx]: VMaxs[]: {}".format(VMaxs))



if __name__ == "__main__":
    scan_safety = ScanSafetyNode()
