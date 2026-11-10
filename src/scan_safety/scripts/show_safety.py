#!/usr/bin/env python
import sys
import rospy
from sensor_msgs.msg import LaserScan
from math import cos, sin, sqrt
from math import pi as PI
from polar_utils import *
from safety_msgs.msg import SafetyStatus
# from scan_safety.srv import FieldsUpdated, FieldsUpdatedResponse
from std_msgs.msg import Header, Time
from matplotlib import pyplot as plt
from matplotlib import animation as animation
import copy

fig = plt.figure()
ax = fig.add_subplot(1,1,1)

C = ['red', 'orange', 'yellow']


class ShowSafety():
    def __init__(self):
        rospy.loginfo("Show safety scan")
        # Get log_level from args
        try:
            tmp = int(sys.argv[-1])
            if tmp > 0 and tmp < 6:
                _log_level = tmp
        except:
            _log_level = 2
        print("_log_level %i"%_log_level)

        rospy.init_node("show_safety", log_level=_log_level)
        self.lastestCfgUpdate = rospy.Time().now()
        self.updateFields()
        self.lastestSttUpdate = rospy.Time().now()
        self.number_of_ranges = rospy.get_param("~number_of_ranges", 720)
        self.Angles = []
        for i in range(self.number_of_ranges):
            self.Angles.append(float(i)*PI/360.0)

        self.X = [0]*self.number_of_ranges
        self.Y = [0]*self.number_of_ranges
        rospy.Subscriber("/scan", LaserScan, self.scanCallback)
        rospy.Subscriber('/scan_safety_cfg_changed', Time, self.cfgChangedCB)

        self.obstacle = [0]*self.number_of_fields
        rospy.Subscriber("safety_status", SafetyStatus, self.safetyCallback)

        ani = animation.FuncAnimation(fig, self.animate, fargs=(), interval=100)
        plt.show()

    def cfgChangedCB(self, msg):
        print(msg.data)
        if msg.data != self.lastestCfgUpdate:
            self.lastestCfgUpdate = msg.data
            self.updateFields()

    def animate(self, i):
        # self.updateFields()
        Fx = copy.deepcopy(self.Fx)
        Fy = copy.deepcopy(self.Fy)
        N_fields = self.number_of_fields
        X = copy.deepcopy(self.X)
        Y = copy.deepcopy(self.Y)
        is_running = False
        rospy.logdebug("ani-lastest: %f"%self.lastestSttUpdate.to_sec())
        rospy.logdebug("ani-now: %f"%rospy.Time.now().to_sec())
        if (rospy.Time.now() - self.lastestSttUpdate) > rospy.Duration(5):
            obs = [0]*N_fields
            is_running = False
        else:
            obs = copy.deepcopy(self.obstacle)
            is_running = True

        ax.clear()
        try:
            for i in range(N_fields-1,-1,-1):
                # color zone with detecting obstacles
                if obs[i]:
                    ax.fill_between(Fx[i], Fy[i], facecolor = C[i])
                else:
                    ax.fill_between(Fx[i], Fy[i], facecolor = 'springgreen')
                # Draw zone bounder
                ax.plot(Fx[i][1:-1], Fy[i][1:-1], c='r')
            # color robot footprint
            ax.fill_between(self.rbX, self.rbY, facecolor='gray')
            ax.text(0.5,0,'Head')
            # Draw scan points
            ax.scatter(X, Y, color='b', s=0.5)
        except Exception as e:
            rospy.logwarn(e)
        ax.grid(True)
        ax.scatter(0,0,c='black')
        ax.axis('equal')
        ax.set_xlim([-2,2])
        ax.set_ylim([-2,2])
        if is_running:
            title_obj = plt.title("current job: %i | %s"%(self.current_job, "Safety Node is running"))
            plt.setp(title_obj, color='b')
        else:
            title_obj = plt.title("current job: %i | %s"%(self.current_job, "Safety Node Stop"))
            plt.setp(title_obj, color='r')

    def updateFields(self):
        self.current_job = rospy.get_param("/scan_safety/current_job", 0)
        rospy.logdebug("Current job: %i"%self.current_job)
        current_job = self.current_job
        self.number_of_fields = rospy.get_param("/scan_safety/job_%i/number_of_fields"%current_job, 3)
        number_of_fields = self.number_of_fields
        rospy.logdebug("Number of fields: %i"%number_of_fields)

        Vertexes = []
        Fx = []
        Fy = []

        for i in range(number_of_fields):
            param_str = rospy.get_param("/scan_safety/job_%i/field_%i"%(current_job, i+1), '')
            # tmp = param_str[2:-2]
            # while ' ' in tmp:
            #     tmp = tmp.replace(' ', '')
            # tmp_arr = tmp.split('],[')
            # vertexes = []
            # for p in tmp_arr:
            #     [x,y] = p.split(',')
            #     vertexes.append([float(x), float(y)])
            vertexes = str2arr(param_str)
            Vertexes.append(vertexes)

        for i in range(number_of_fields):
            Fx.append([0])
            Fy.append([0])
            for vertex in Vertexes[i]:
                Fx[i].append(vertex[0])
                Fy[i].append(vertex[1])
            Fx[i].append(0)
            Fy[i].append(0)
        self.Vertexes = copy.deepcopy(Vertexes)
        self.Fx = copy.deepcopy(Fx)
        self.Fy = copy.deepcopy(Fy)

        fp_str = rospy.get_param("/scan_safety/footprint", '')
        rb_footprint = str2arr(fp_str)
        rbX = [0]
        rbY = [0]
        for p in rb_footprint:
            rbX.append(p[0])
            rbY.append(p[1])
        rbX.append(0)
        rbY.append(0)
        self.rbX = copy.deepcopy(rbX)
        self.rbY = copy.deepcopy(rbY)

    def scanCallback(self, msg):
        ranges = list(msg.ranges)
        N_ranges = len(ranges)
        angle_increment = msg.angle_increment
        angle_start = msg.angle_min
        X = [0.0]*N_ranges
        Y = [0.0]*N_ranges
        for i in range(N_ranges):
            angle = angle_start + float(i)*angle_increment
            X[i] = ranges[i]*cos(angle)
            Y[i] = ranges[i]*sin(angle)
        self.X = copy.deepcopy(X)
        self.Y = copy.deepcopy(Y)

    def safetyCallback(self, msg):
        self.obstacle = list(msg.fields)
        self.lastestSttUpdate = msg.header.stamp
        rospy.logdebug("obstacle: {}".format(self.obstacle))
        rospy.logdebug("stamp: {}".format(self.lastestSttUpdate.to_sec()))

if __name__ == "__main__":
    show = ShowSafety()