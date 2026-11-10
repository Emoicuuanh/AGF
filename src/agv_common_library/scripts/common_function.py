#!/usr/bin/env python
# coding=utf-8
import sys

python3 = True if sys.hexversion > 0x03000000 else False
from time import time

# pip install pyserial
import serial
import serial.tools.list_ports
import glob
from math import pi, sqrt, atan2
import json
import yaml
import copy
from collections import namedtuple
from math import pi, sin, cos, tan
from enum import Enum
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, PoseStamped
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    PoseArray,
    Quaternion,
    Pose2D,
)
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from PyQt5 import QtCore, QtGui
import os
import signal
from subprocess import check_output
import tf
import tf2_ros
import rospy
from inspect import getframeinfo, stack
import rospkg
from std_stamped_msgs.msg import StringStamped, EmptyStamped
from datetime import datetime, date
import hashlib
from os.path import expanduser

# Constant
HOME = expanduser("~")
SENSOR_DEACTIVATE = 0
SENSOR_ACTIVATE = 1
MIN_FLOAT = 0.0001

# Need copy.deepcopy each time use this variable
pose_dict_template = {
    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 0.0},
}


class PrintColor:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


class YamlDumper(yaml.Dumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(YamlDumper, self).increase_indent(flow, False)


class InlineClass(object):
    def __init__(self, dict):
        self.__dict__ = dict


class VelWithPid:
    def __init__(self):
        self.last_time = rospy.get_time()
        self.total_error = 0.0
        self.last_error = 0.0

    def cal_vel(
        self,
        current_vel=0.0,
        target_vel=0.0,
        acc=0.0,
        dec=0.0,
        error=0.0,
        kp=1.0,
        ki=0.0,
        kd=0.0,
        min_vel=0.0,
        clear_pid=False,
    ):
        """[summary]

        Args:
            current_vel ([float]): [description]
            target_vel ([float]): [description]
            acc ([float]): [description]
            dec (float, optional): [description]. Defaults to 0.0.
            error (float, optional): [description]. Defaults to 0.0.
            kp (float, optional): [description]. Defaults to 1.0.
            ki (float, optional): [description]. Defaults to 0.0.
            kd (float, optional): [description]. Defaults to 0.0.
            min_vel ([float]): [description]

        Returns:
            [type]: [description]
        """
        current_time = rospy.get_time()
        diff_time = current_time - self.last_time
        if diff_time > 0:
            diff_vel = abs(acc * diff_time)
            self.last_time = current_time
        else:
            return current_vel

        if error <= 0.0:
            target_vel = -target_vel
            min_vel = -min_vel

        # ACC
        vel_with_acc = current_vel
        if vel_with_acc <= target_vel - diff_vel:
            vel_with_acc += diff_vel
        elif vel_with_acc > target_vel + diff_vel:
            vel_with_acc -= diff_vel
        else:
            vel_with_acc = target_vel

        # PID
        if clear_pid:
            self.total_error = 0.0
            self.last_error = 0.0

        self.total_error = self.total_error + error

        vel_p = kp * error
        vel_i = ki * self.total_error
        vel_d = kd * (error - self.last_error)
        vel_pid = vel_p + vel_i * diff_time + vel_d / diff_time

        self.last_error = error

        if error >= 0.0:
            final_vel = max(min_vel, min(vel_pid, target_vel, vel_with_acc))
        else:
            final_vel = min(min_vel, max(vel_pid, target_vel, vel_with_acc))
        return final_vel


class EnumString(Enum):
    @classmethod
    def list(cls):
        return list(map(lambda c: c.toString(), cls))

    def toString(self):
        return str(self.name)


class ActionResult(EnumString):
    NONE = -1
    PENDING = 0
    ACTIVE = 1
    PREEMPTED = 2
    SUCCEEDED = 3
    ABORTED = 4
    REJECTED = 5
    PREEMPTING = 6
    RECALLING = 7
    RECALLED = 8
    LOST = 9


def static_vars(**kwargs):
    def decorate(func):
        for k in kwargs:
            setattr(func, k, kwargs[k])
        return func

    return decorate


def dict_to_obj(_dict, _obj, _str=""):
    """Convert a dictionary to non iterable object with a base dict (Take care copy.deepcopy)"""
    if type(_dict) is dict:
        if python3:
            for key, value in _dict.items():
                if type(value) is dict:
                    # Check _obj contain attribute
                    if hasattr(_obj, key):
                        dict_to_obj(value, _obj, _str=key + ".")
                    else:
                        print_debug(
                            'dict_to_obj: object has no attr "{}"'.format(key)
                        )
                else:
                    att = "_obj." + _str + key
                    cmd = att + "=" + str(value)
                    # print(cmd)
                    exec(cmd)
        else:
            for key, value in _dict.iteritems():
                if type(value) is dict:
                    # Check _obj contain attribute
                    if hasattr(_obj, key):
                        dict_to_obj(value, _obj, _str=key + ".")
                    else:
                        print_debug(
                            'dict_to_obj: object has no attr "{}"'.format(key)
                        )
                else:
                    att = "_obj." + _str + key
                    cmd = att + "=" + str(value)
                    # print(cmd)
                    exec(cmd)
    return _obj


def obj_to_dict_recursive(_obj, _base_dict, _str="", _cmd_list=[]):
    if type(_base_dict) is dict:
        if python3:
            for key, value in _base_dict.items():
                if type(value) is dict:
                    # Check _obj contain attribute
                    if hasattr(_obj, key):
                        obj_to_dict_recursive(_obj, value, _str=key + ".")
                    else:
                        print_debug(
                            'obj_to_dict_recursive: object has no attr "{}"'.format(
                                key
                            )
                        )
                else:
                    att = "_obj." + _str + key
                    att_arr = att.split(".")[1:]
                    att_str = "_base_dict"
                    for a in att_arr:
                        att_str += '["{}"]'.format(a)
                    cmd = att_str + "=" + att
                    _cmd_list.append(cmd)
        else:
            for key, value in _base_dict.iteritems():
                if type(value) is dict:
                    # Check _obj contain attribute
                    if hasattr(_obj, key):
                        obj_to_dict_recursive(_obj, value, _str=key + ".")
                    else:
                        print_debug(
                            'obj_to_dict_recursive: object has no attr "{}"'.format(
                                key
                            )
                        )
                else:
                    att = "_obj." + _str + key
                    att_arr = att.split(".")[1:]
                    att_str = "_base_dict"
                    for a in att_arr:
                        att_str += '["{}"]'.format(a)
                    cmd = att_str + "=" + att
                    _cmd_list.append(cmd)
    else:
        print("obj_to_dict_recursive: input is not a dict")
    return _cmd_list


def obj_to_dict(_obj, _base_dict):
    """Convert a non iterable object to dictionary with a base dict (Take care: _base_dict = copy.deepcopy(template_dict))"""
    _cmd_list = obj_to_dict_recursive(_obj, _base_dict)

    for cmd in _cmd_list:
        # print_warn(cmd)
        exec(cmd)
    return _base_dict


def dict_to_pose(input_dict):
    w = {}
    w["position"] = {}
    w["position"]["x"] = input_dict["position"]["x"]
    w["position"]["y"] = input_dict["position"]["y"]
    w["orientation"] = input_dict["orientation"]
    return dict_to_obj(w, Pose())


def merge_two_dicts(x, y):
    # FIXME: child dict not update
    z = x.copy()  # start with keys and values of x
    z.update(y)  # modifies z with keys and values of y
    return z


def get_line_info():
    obj = type(
        "obj",
        (object,),
        {
            "file_name": stack()[1][1],
            "line_no": getframeinfo(stack()[1][0]).lineno,
            "function_name": stack()[1][3],
        },
    )
    return obj


def print_info(txt, header="INFO_"):
    # Get line from called function
    caller = getframeinfo(stack()[1][0])
    try:
        time = "{}.{}".format(
            str(rospy.Time.now().secs)[0:10], str(rospy.Time.now().nsecs)[0:6]
        )
        node_name = rospy.get_name()
    except:
        time = ""
        node_name = ""
    print(
        "[{}] [{}] [{}] [{}]: {}".format(
            header, time, node_name, caller.lineno, txt
        )
    )


def print_debug(txt, title="", header="DEBUG_"):
    # Get line from called function
    caller = getframeinfo(stack()[1][0])
    try:
        time = "{}.{}".format(
            str(rospy.Time.now().secs)[0:10], str(rospy.Time.now().nsecs)[0:6]
        )
        node_name = rospy.get_name()
    except:
        time = ""
        node_name = ""
    if title == "":
        print(
            PrintColor.OKGREEN
            + "[{}] [{}] [{}] [{}]: {}".format(
                header, time, node_name, caller.lineno, txt
            )
            + PrintColor.ENDC
        )
    else:
        print(
            PrintColor.OKGREEN
            + "[{}] [{}] [{}] [{}]: {}\n{}".format(
                header, time, node_name, caller.lineno, title, txt
            )
            + PrintColor.ENDC
        )


def print_warn(txt, title="", header="WARN_"):
    caller = getframeinfo(stack()[1][0])
    try:
        time = "{}.{}".format(
            str(rospy.Time.now().secs)[0:10], str(rospy.Time.now().nsecs)[0:6]
        )
        node_name = rospy.get_name()
    except:
        time = ""
        node_name = ""
    if title == "":
        print(
            PrintColor.WARNING
            + "[{}] [{}] [{}] [{}]: {}".format(
                header, time, node_name, caller.lineno, txt
            )
            + PrintColor.ENDC
        )
    else:
        print(
            PrintColor.WARNING
            + "[{}] [{}] [{}] [{}]: {}\n{}".format(
                header, time, node_name, caller.lineno, title, txt
            )
            + PrintColor.ENDC
        )


def print_error(txt, title="", header="ERROR_"):
    caller = getframeinfo(stack()[1][0])
    try:
        time = "{}.{}".format(
            str(rospy.Time.now().secs)[0:10], str(rospy.Time.now().nsecs)[0:6]
        )
        node_name = rospy.get_name()
    except:
        time = ""
        node_name = ""
    if title == "":
        print(
            PrintColor.FAIL
            + "[{}] [{}] [{}] [{}]: {}".format(
                header, time, node_name, caller.lineno, txt
            )
            + PrintColor.ENDC
        )
    else:
        print(
            PrintColor.FAIL
            + "[{}] [{}] [{}] [{}]: {}\n{}".format(
                header, time, node_name, caller.lineno, title, txt
            )
            + PrintColor.ENDC
        )


def qt_get_color_rgba(r, g, b, a):
    color = QtGui.QColor(r, g, b)
    alpha = a
    values = "{r}, {g}, {b}, {a}".format(
        r=color.red(), g=color.green(), b=color.blue(), a=alpha
    )
    return values


def qt_change_background_color(widget, r, g, b):
    widget.setStyleSheet(
        "QLabel { background-color: rgba("
        + qt_get_color_rgba(r, g, b, 255)
        + "); }"
    )


def save_json(data, savePath):
    json_string = json.dumps([ob.__dict__ for ob in data], indent=2)
    print(json_string)
    datastore = json.loads(json_string)
    with open(savePath, "w") as f:
        json.dump(datastore, f, indent=2)


def load_json(loadPath):
    def _json_object_hook(d):
        return namedtuple("X", d.keys())(*d.values())

    def json2obj(data):
        return json.loads(data, object_hook=_json_object_hook)

    if loadPath:
        with open(loadPath, "r") as f:
            datastore = yaml.safe_load(f)
            data_dict = json.dumps(datastore)
            ret = json2obj(data_dict)
        return ret


def x_y_theta_to_pose_stamped(frame_id, x, y, rad):
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    quan = quaternion_from_euler(0, 0, rad)
    pose.pose.orientation.x = quan[0]
    pose.pose.orientation.y = quan[1]
    pose.pose.orientation.z = quan[2]
    pose.pose.orientation.w = quan[3]
    return pose


def make_transform_stamped(fram_id, child_fram_id, tx, ty, tz, rx, ry, rz):
    ret = tf2_ros.TransformStamped()
    ret.header.frame_id = fram_id
    ret.child_frame_id = child_fram_id
    ret.transform.translation.x = tx
    ret.transform.translation.y = ty
    ret.transform.translation.z = tz
    quat = quaternion_from_euler(rx, ry, rz)
    ret.transform.rotation.x = quat[0]
    ret.transform.rotation.y = quat[1]
    ret.transform.rotation.z = quat[2]
    ret.transform.rotation.w = quat[3]
    ret.header.stamp = rospy.Time.now()
    return ret


def pose_with_cov_to_vector_3(pose):
    ori = euler_from_quaternion(
        [
            pose.pose.pose.orientation.x,
            pose.pose.pose.orientation.y,
            pose.pose.pose.orientation.z,
            pose.pose.pose.orientation.w,
        ]
    )
    ret = Vector3()
    ret.x = pose.pose.pose.position.x
    ret.y = pose.pose.pose.position.y
    ret.z = ori[2]
    return ret


def yaw_to_quaternion(rad):
    quan = quaternion_from_euler(0, 0, rad)
    ret = Quaternion()
    ret.x = quan[0]
    ret.y = quan[1]
    ret.z = quan[2]
    ret.w = quan[3]
    return ret


def distance_two_points(xA, yA, xB, yB):
    distance = sqrt((xB - xA) ** 2 + (yB - yA) ** 2)
    return distance


def get_yaw(pose):
    rad = euler_from_quaternion(
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )[2]
    return rad


def rad_to_deg(rad):
    return rad * 180.0 / pi


def pose_stamped_to_pose_2d(pose):
    yaw = rad = euler_from_quaternion(
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    )[2]
    return Pose2D(pose.position.x, pose.position.y, yaw)


def pose_stamped_array_to_pose_array(_waypoints, _frame_id):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = _frame_id
    poses.poses = [pose.pose for pose in _waypoints]
    return poses


def waypoints_to_arr_of_pose_stamped(wp):
    poseStampedArray = []
    for p in wp:
        ps = PoseStamped()
        ps.header = p.Pose.header
        ps.pose = p.Pose.pose
        poseStampedArray.append(ps)
    return poseStampedArray


def delta_angle(cur, des):
    ret = des - cur
    ret = (ret + pi) % (2 * pi) - pi
    return ret


def normalize_angle(angle):
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle


def distance_two_pose(src, des):
    x = des.position.x - src.position.x
    y = des.position.y - src.position.y
    ret = sqrt(x**2 + y**2)
    return ret


def angle_two_pose(src, des):
    angle_src = get_yaw(src)
    angle_des = get_yaw(des)
    ret = delta_angle(angle_src, angle_des)
    return ret


def angle_robot_vs_target_angle(angle, robot_pose):
    """Tính góc tạo bởi Robot và góc "angle" """
    current_angle = get_yaw(robot_pose)
    target_angle = angle
    ret = delta_angle(current_angle, target_angle)
    return ret


def angle_robot_vs_robot_to_goal(target, robot_pose):
    """Tính góc tạo bởi hướng hiện tại của Robot và đường thẳng nối từ tâm Robot tới Goal"""
    robotYaw = get_yaw(robot_pose)
    x = target.position.x - robot_pose.position.x
    y = target.position.y - robot_pose.position.y
    robot_target_angle = atan2(y, x)
    ret = delta_angle(robotYaw, robot_target_angle)
    return ret


def angle_robot_vs_goal_to_robot(target, robot_pose):
    """Tính góc tạo bởi hướng hiện tại của Robot và đường thẳng nối từ tâm Goal tới Robot"""
    robotYaw = get_yaw(robot_pose)
    x = robot_pose.position.x - target.position.x
    y = robot_pose.position.y - target.position.y
    robot_target_angle = atan2(y, x)
    ret = delta_angle(robotYaw, robot_target_angle)
    return ret


def angle_goal_vs_robot_to_goal(target, robot_pose):
    """Tính góc tạo bởi hướng hiện tại của Goal và đường thẳng nối từ tâm Robot tới Goal"""
    goal_yaw = get_yaw(target)
    x = target.position.x - robot_pose.position.x
    y = target.position.y - robot_pose.position.y
    robot_target_angle = atan2(y, x)
    ret = delta_angle(robot_target_angle, goal_yaw)
    return ret


def angle_vector_two_point(pose_1, pose_2):
    """Tính góc tạo bởi đường thẳng nối giữa 2 điểm và Ox"""
    x = pose_1.position.x - pose_2.position.x
    y = pose_1.position.y - pose_2.position.y
    ret = atan2(y, x)
    return ret


def distance_to_line_perpendicular_vs_goal(robotPose, goal):
    alpha = get_yaw(goal)
    # tinh phuong trinh duong thang vuong goc voi (goal) va di qua (goal)
    # vecto phap tuyen la vector (goal) co goc alpha voi Ox: (cos(alpha), sin(alpha))
    line_a = cos(alpha)
    line_b = sin(alpha)
    # Phuong trinh duong thang di qua diem (x0, y0): a(x-x0)+b(y-y0)=0 <=> ax + by - ax0 - by0 = 0 <=> ax + by + c = 0
    line_c = -line_a * goal.position.x - line_b * goal.position.y

    if line_a == 0 and line_b == 0:
        return "ERROR"
    # Khoang cach voi duong thang: abs(ax0+by0+c)/sqrt(a^2+b^2)
    _result = abs(
        line_a * robotPose.position.x + line_b * robotPose.position.y + line_c
    ) / sqrt(line_a * line_a + line_b * line_b)
    return _result


def signed_distance_to_line_perpendicular_vs_goal(robotPose, goal):
    alpha = get_yaw(goal)
    # tinh phuong trinh duong thang vuong goc voi (goal) va di qua (goal)
    # vecto phap tuyen la vector (goal) co goc alpha voi Ox: (cos(alpha), sin(alpha))
    line_a = cos(alpha)
    line_b = sin(alpha)
    # Phuong trinh duong thang di qua diem (x0, y0): a(x-x0)+b(y-y0)=0 <=> ax + by - ax0 - by0 = 0 <=> ax + by + c = 0
    line_c = -line_a * goal.position.x - line_b * goal.position.y

    if line_a == 0 and line_b == 0:
        return False
    # Khoang cach voi duong thang: abs(ax0+by0+c)/sqrt(a^2+b^2)
    _result = (
        line_a * robotPose.position.x + line_b * robotPose.position.y + line_c
    ) / sqrt(line_a * line_a + line_b * line_b)
    return _result


def distance_to_vector_goal(robotPose, goal):
    alpha = get_yaw(goal)
    # tinh phuong trinh duong thang d song song voi (goal) va di qua (goal)
    # d co vecto chi phuong vector (goal): (cos(alpha), sin(alpha)) <=> (-b, a)
    a1 = sin(alpha)
    b1 = -cos(alpha)
    # Phuong trinh duong thang di qua diem (x0, y0): a(x-x0)+b(y-y0)=0 <=> ax + by - ax0 - by0 = 0 <=> ax + by + c = 0
    c1 = -a1 * goal.position.x - b1 * goal.position.y

    if a1 == 0 and b1 == 0:
        return "ERROR"
    # Khoang cach voi duong thang: abs(ax0+by0+c)/sqrt(a^2+b^2)
    _result = (
        a1 * robotPose.position.x + b1 * robotPose.position.y + c1
    ) / sqrt(a1 * a1 + b1 * b1)
    return _result


def get_perpendicular_line_robot_goal(robotPose, goal, frame_id="map"):
    """Tính khoảng cách từ Robot tới vector Goal và đường qua 3 điểm: Robot, hình chiếu của Robot trên vector Goal và Goal"""
    alpha = get_yaw(goal)
    # tinh phuong trinh duong thang d song song voi (goal) va di qua (goal)
    # d co vecto chi phuong vector (goal): (cos(alpha), sin(alpha)) <=> (-b, a)
    a1 = sin(alpha)
    b1 = -cos(alpha)
    # Phuong trinh duong thang di qua diem (x0, y0): a(x-x0)+b(y-y0)=0 <=> ax + by - ax0 - by0 = 0 <=> ax + by + c = 0
    c1 = -a1 * goal.position.x - b1 * goal.position.y

    if a1 == 0 and b1 == 0:
        return "ERROR"
    # Khoang cach voi duong thang: abs(ax0+by0+c)/sqrt(a^2+b^2)
    _result = (
        a1 * robotPose.position.x + b1 * robotPose.position.y + c1
    ) / sqrt(a1 * a1 + b1 * b1)

    # Phuong trinh duong thang d' di qua diem hien tai va vuong goc voi vector (goal)
    # d' co vector chinh tac la vector (goal): (cos(alpha), sin(alpha)) <=> (d, e)
    a2 = -b1  # cos
    b2 = a1  # sin
    c2 = -a2 * robotPose.position.x - b2 * robotPose.position.y

    # Toa do giao diem cua d va d'. Xet truong hop a1 != 0, va line a = 0 (a1 va b1 khong dong thoi = 0)
    x = 0
    y = 0
    if a1 != 0:
        y = (c2 - c1 * a2 / a1) / ((b1 * a2 / a1) - b2)
        x = (-c1 - b1 * y) / a1
    elif a1 == 0 and a2 != 0:
        y = -c1 / b1  # a1 va b1 khong dong thoi bang 0
        x = (
            (b2 * c1 / b1) - c2
        ) / a2  # neu d = 0, d va d' song song => khong co giao diem

    # pt1 * a2, pt2 * a1
    # if a2*b1 - a1*b2 !=0:
    #     y = (a1*c2 - a2*c1) / (a2*b1 - a1*b2)
    #     x = (-c1 - b1*y) / a1
    # else:
    #     return

    path = Path()
    path.header.frame_id = frame_id
    path.poses.append(
        xy_to_pose_stamped(frame_id, robotPose.position.x, robotPose.position.y)
    )
    path.poses.append(xy_to_pose_stamped(frame_id, x, y))
    path.poses.append(
        xy_to_pose_stamped(frame_id, goal.position.x, goal.position.y)
    )

    # straight_path_publisher.publish(path)
    return _result, path


def xy_to_pose_stamped(_frame_id, _x, _y):
    ret = PoseStamped()
    ret.header.frame_id = _frame_id
    ret.pose.position.x = _x
    ret.pose.position.y = _y
    return ret


def get_pose_from_frame(tf_listener, base_frame, goal_from_another_frame):
    """[summary]

    Args:
        tf_listener (tf.TransformListener): [description]
        base_frame (str): [description]
        goal_from_another_frame (PoseStamped): [description]

    Returns:
        ret (PoseStamped)
    """
    try:
        tf_listener.waitForTransform(
            base_frame,
            goal_from_another_frame.header.frame_id,
            rospy.Time.now(),
            rospy.Duration(1.0),
        )
        ret = tf_listener.transformPose(base_frame, goal_from_another_frame)
    except (
        Exception
    ) as e:  # (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.logwarn_throttle(5, "get_pose_from_frame: {}".format(e))
        return None
    return ret


def lockup_pose(tf_listener, base_frame, robot_frame):
    try:
        tf_listener.waitForTransform(
            base_frame, robot_frame, rospy.Time.now(), rospy.Duration(1.0)
        )
        (trans, rot) = tf_listener.lookupTransform(
            base_frame, robot_frame, rospy.Time(0)
        )
    except (
        Exception
    ) as e:  # (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.logwarn_throttle(10, "lockup_pose: {}".format(e))
        return None
    ret = Pose()
    ret.position.x = trans[0]
    ret.position.y = trans[1]
    ret.position.z = trans[2]
    ret.orientation.x = rot[0]
    ret.orientation.y = rot[1]
    ret.orientation.z = rot[2]
    ret.orientation.w = rot[3]
    return ret


def lockup_transform(tf_listener, target_frame, source_frame, duration=1.0):
    try:
        tf_listener.waitForTransform(
            target_frame,
            source_frame,
            rospy.Time.now(),
            rospy.Duration(duration),
        )
        (trans, rot) = tf_listener.lookupTransform(
            target_frame, source_frame, rospy.Time(0)
        )
    except (
        Exception
    ) as e:  # (tf.Exception, tf.ConnectivityException, tf.LookupException):
        rospy.logwarn_throttle(10, "lockup_transform: {}".format(e))
        return None
    ret = Pose()
    ret.position.x = trans[0]
    ret.position.y = trans[1]
    ret.position.z = trans[2]
    ret.orientation.x = rot[0]
    ret.orientation.y = rot[1]
    ret.orientation.z = rot[2]
    ret.orientation.w = rot[3]
    return ret


def offset_pose_x(pose, x):
    rad = get_yaw(pose)
    ret = Pose()
    ret.position.x = pose.position.x + x * cos(rad)
    ret.position.y = pose.position.y + x * sin(rad)
    ret.position.z = pose.position.z
    ret.orientation = pose.orientation
    return ret


def offset_pose_y(pose, y):
    rad = get_yaw(pose)
    ret = Pose()
    ret.position.x = pose.position.x - y * sin(rad)
    ret.position.y = pose.position.y + y * cos(rad)
    ret.position.z = pose.position.z
    ret.orientation = pose.orientation
    return ret


def offset_pose_yaw(org_pose, offset_yaw):
    ret = copy.deepcopy(org_pose)
    org_yaw = get_yaw(org_pose)
    ret.orientation = yaw_to_quaternion(org_yaw + offset_yaw)
    return ret


def offset_pose_xyz(pose, x, y, w):
    rad = get_yaw(pose)
    ret = Pose()
    ret = offset_pose_x(pose, x)
    ret = offset_pose_y(ret, y)

    new_rad = rad + w
    ret.orientation = yaw_to_quaternion(new_rad)
    return ret


def offset_pose_xy_theta(org_pose, x, y, theta):
    """Offset theta first"""
    ret = copy.deepcopy(org_pose)
    org_yaw = get_yaw(org_pose)
    ret.orientation = yaw_to_quaternion(org_yaw + theta)
    ret = offset_pose_x(ret, x)
    ret = offset_pose_y(ret, y)
    return ret


def offset_pose_to_pose(org_pose, offset_pose, type=1):
    """offset_pose_to_pose

    Args:
        org_pose (Pose()): origin pose that need to offset
        offset_pose (Pose()): Offset matrix as pose
        type (int, optional): Type of offset.
                            type = 1: offset follow x, y, yaw
                            type = 3: offset follow yaw, x, y
                            Defaults to 1.

    Returns:
        Pose(): Pose offsetted
    """
    ret = copy.deepcopy(org_pose)
    if type == 1:  # Transfer theo thu tu: x, y, yaw
        org_yaw = get_yaw(org_pose)
        offset_yaw = get_yaw(offset_pose)
        ret = offset_pose_x(org_pose, offset_pose.position.x)
        ret = offset_pose_y(ret, offset_pose.position.y)
        ret.orientation = yaw_to_quaternion(org_yaw + offset_yaw)
    if type == 3:  # Transfer theo thu tu: yaw, x, y
        org_yaw = get_yaw(org_pose)
        offset_yaw = get_yaw(offset_pose)
        ret.orientation = yaw_to_quaternion(org_yaw + offset_yaw)
        ret = offset_pose_x(ret, offset_pose.position.x)
        ret = offset_pose_y(ret, offset_pose.position.y)
    return ret


def double_waypoint(yaml_file, wp_des):
    ret = []
    wp_straight = copy.deepcopy(wp_des)
    wp_low_sp = copy.deepcopy(wp_des)

    wp_des.Pose.pose = offset_pose_x(wp_des.Pose.pose, 1.3)
    set_wp_param_as_tag(yaml_file, wp_des, "xy_tol", 0.2)
    set_wp_param_as_tag(yaml_file, wp_des, "yaw_tol", 0.1)

    wp_low_sp.Pose.pose = wp_des.Pose.pose
    set_wp_param_as_tag(yaml_file, wp_low_sp, "forward_vel", 0.1)
    set_wp_param_as_tag(yaml_file, wp_low_sp, "backward_vel", 0.1)

    wp_straight.Moving_Type = "STRAIGHT"
    set_wp_param(yaml_file, wp_straight, "go_straight_speed", 0.1)
    set_wp_param(yaml_file, wp_straight, "go_straight_backward_speed", 0.1)
    set_wp_param(yaml_file, wp_straight, "go_straight_speed_reduce", 0.03)
    set_wp_param(
        yaml_file, wp_straight, "go_straight_distance_reduce_speed", 0.2
    )
    set_wp_param(yaml_file, wp_straight, "go_straight_tol_x", 0.03)

    ret.append(wp_des)
    ret.append(wp_low_sp)
    ret.append(wp_straight)
    return ret


def set_wp_param(yaml_file, wp, param_name, value):
    moving_type = wp.Moving_Type
    param_list = []
    column = ""
    for p in yaml_file[moving_type]["reconfigure_params"]:
        param_list.extend(p["params"])
    for p in param_list:
        if p["param"] == param_name:
            column = p["column"]
            break
    # print_debug('Set param: {}, value: {}, column: {}, moving_type: {}'.format(param_name, value, column, moving_type))
    if column != "":
        param = setattr(wp.Moving_Param, column, value)
    else:
        print_error("Set waypoint param error")
    return wp


def set_wp_param_as_tag(yaml_file, wp, tag, value):
    moving_type = wp.Moving_Type
    param_list = []
    column = ""
    for p in yaml_file[moving_type]["reconfigure_params"]:
        param_list.extend(p["params"])
    for p in param_list:
        if p["tag"] == tag:
            column = p["column"]
            break
    # print_debug('Set param as tag: {}, value: {}, column: {}, moving_type: {}'.format(tag, value, column, moving_type))
    if column != "":
        param = setattr(wp.Moving_Param, column, value)
    else:
        print_error("Set waypoint param as tag error")
    return wp


def get_wp_param(yaml_file, wp, param_name):
    moving_type = wp.Moving_Type
    param_list = []
    column = ""
    param_value = None
    for p in yaml_file[moving_type]["reconfigure_params"]:
        param_list.extend(p["params"])
    for p in param_list:
        if p["param"] == param_name:
            column = p["column"]
            break
    # print_debug('Get param: {}, column: {}, moving_type: {}'.format(param_name, column, moving_type))
    if column != "":
        param_value = getattr(wp.Moving_Param, column)
    else:
        print_error("Get waypoint param error")
    return param_value


def get_wp_param_as_tag(yaml_file, wp, tag):
    moving_type = wp.Moving_Type
    param_list = []
    column = ""
    param_value = None
    for p in yaml_file[moving_type]["reconfigure_params"]:
        param_list.extend(p["params"])
    for p in param_list:
        if p["tag"] == tag:
            column = p["column"]
            break
    # print_debug('Get param as tag: {}, column: {}, moving_type: {}'.format(tag, column, moving_type))
    if column != "":
        param_value = getattr(wp.Moving_Param, column)
    else:
        print_error("Get waypoint param as tag error")
    return param_value


def get_pid(name):
    try:
        return map(int, check_output(["pidof", name]).split())
    except:
        return []


def get_proc_path(_type, _name=None):
    listProc = get_pid(_type)
    if listProc == []:
        print("There are no pid with type: " + _type)
        return []
    print(listProc)
    ret = []

    for pid in listProc:
        # try:
        with open("/proc/{}/cmdline".format(pid), mode="rb") as fd:
            content = fd.read().decode().split("\x00")
            if len(content) < 2:
                continue
            path = content[1]
            if _name == None:
                ret.append(InlineClass({"pid": pid, "path": path}))
                continue
            if _name in path:
                ret.append(InlineClass({"pid": pid, "path": path}))
        # except Exception:
        #     print('Exception')
    return ret


def kill_proc(_type, _name=None):
    procList = get_proc_path(_type, _name)
    if len(procList) > 0:
        for p in procList:
            print("Kill process " + p.path + " by id: " + str(p.pid))
            os.kill(p.pid, signal.SIGTERM)
    else:
        print(
            "There are no process with type = "
            + str(_type)
            + " & "
            + " path = "
            + str(_name)
        )


def get_serial_ports(mark="A-Za-z"):
    """Lists serial port names

    :raises EnvironmentError:
        On unsupported or unknown platforms
    :returns:
        A list of the serial ports available on the system
    """
    if sys.platform.startswith("win"):
        ports = ["COM%s" % (i + 1) for i in range(256)]
    elif sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
        # this excludes your current terminal "/dev/tty"
        # ports = glob.glob('/dev/tty[A-Za-z]*') # all port
        ports = glob.glob("/dev/tty[{}]*".format(mark))
    elif sys.platform.startswith("darwin"):
        ports = glob.glob("/dev/tty.*")
    else:
        raise EnvironmentError("Unsupported platform")

    result = []
    # if ports:
    #     for port in ports:
    #         print(port)
    return ports


def get_raspberry_serial():
    hardware_serial = "0000000000000000"
    try:
        f = open("/proc/cpuinfo", "r")
        for line in f:
            if line[0:6] == "Serial":
                hardware_serial = line[10:26]
        f.close()
    except:
        hardware_serial = "ERROR"
    return hardware_serial


def get_ubuntu_serial():
    hardware_serial = "0000000000000000"
    try:
        f = open("/etc/machine-id", "r")
        hardware_serial = f.readline().replace("\n", "")
        f.close()
    except:
        hardware_serial = "ERROR"
    return hardware_serial


def get_ubuntu_serial_hash():
    command = 'dmesg | grep UUID | grep "Kernel" | sed "s/.*UUID=//g" | sed "s/\ ro\ quiet.*//g"'
    output = os.popen(command).read()
    serial = output.strip()

    serial = "Mkac@" + serial + "2917"
    hash_fr_user = hashlib.md5(serial.encode("UTF-8")).hexdigest()
    # print("Hash fr user:", hash_fr_user)
    final_txt = "1007" + str(hash_fr_user) + "meik0!"
    final_hash = hashlib.md5(final_txt.encode("UTF-8")).hexdigest()
    # print("Final hash:", final_hash)

    f = open(HOME + "/tmp/ros/license", "r")
    hardware_serial = f.readline().replace("\n", "")
    # print(hardware_serial)
    f.close()
    # print_debug(hardware_serial)
    if final_hash == hardware_serial:
        return True
    else:
        return False


def find_soft_port(hardware_port):
    if "dev" in hardware_port:
        return hardware_port
    for port in serial.tools.list_ports.comports():
        if port.location == hardware_port:
            return port.device


def find_soft_port_from_pid(port_pid):
    for p in serial.tools.list_ports.comports():
        if p.pid == port_pid:
            return p.device
            # print('Port can tim : '+ str(p.location) + str(p.device))
