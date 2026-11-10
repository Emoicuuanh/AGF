#! /usr/bin/env python
# -*- coding: utf-8 -*-

"""
Dead-reckoning odometry from PLC wheel speed + Livox MID-360 gyro, for
monitoring in rviz. Does NOT publish TF, so it can't fight with
forklift_tf_publisher / MRPT.

  x   += v * cos(yaw) * dt
  y   += v * sin(yaw) * dt
  yaw += (gyro_z - bias) * dt     (or PLC twist.angular.z if use_imu_yaw=false)

PLC speed only has one sign, so the direction is latched on every start from
standstill and held until the robot stops again (a forklift can only reverse
by stopping):
  vel_sign_source=cmd  sign of the speed command plc_driver sends to the PLC
                       (/cmd_vel_send_plc), the PLC speed gives the magnitude
  vel_sign_source=imu  sign of the integrated Livox base-x acceleration
  vel_sign_source=plc  PLC speed sign as is

Subscribes:
  /modbus_wrapper/input (std_msgs/Int32MultiArray)  PLC registers, wheel_source=modbus
      (plc_driver has publish_odom() commented out, and move_base's local
       planner listens on /odom, so the registers are decoded here instead)
  /odom        (nav_msgs/Odometry)  wheel_source=odom, twist.linear.x / angular.z
  /livox/imu   (sensor_msgs/Imu)    MID-360, gyro in livox_frame
  /cmd_vel_send_plc (geometry_msgs/Twist)  vel_sign_source=cmd, commanded direction
  ~reset       (std_msgs/Empty)     re-sync start pose to current TF pose
      (also done when the PLC auto_manual_sw register changes, wheel_source=modbus)
  /mirror_detect (std_msgs/Bool)    True = NAV350 mirror lost, reset when it comes back
Publishes:
  /odom_imu       (nav_msgs/Odometry)
  /odom_imu/path  (nav_msgs/Path)
"""

import struct
import threading
from math import sin, cos, atan2

import rospy
import tf
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Empty
from std_msgs.msg import Int32MultiArray as HoldingRegister
from tf.transformations import quaternion_from_euler, quaternion_matrix


def yaw_to_quaternion(rad):
    quan = quaternion_from_euler(0, 0, rad)
    return Quaternion(quan[0], quan[1], quan[2], quan[3])


def word_to_float(w1, w2):
    # same decoding as plc_driver.word_to_float
    return struct.unpack('<f', struct.pack('<HH', w1 & 0xFFFF, w2 & 0xFFFF))[0]


def get_yaw(q):
    return atan2(2.0 * (q[3] * q[2] + q[0] * q[1]), 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))


class OdomImu:
    def __init__(self):
        rospy.init_node("odom_imu")
        self.global_frame = rospy.get_param("~global_frame", "map")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        # frame the pose is published for: integration is done for base_frame,
        # nav_footprint publishes the NAV350 pose like /nav350laser/odom
        self.child_frame = rospy.get_param("~child_frame", self.base_frame)
        self.imu_topic = rospy.get_param("~imu_topic", "/livox/imu")
        # modbus: decode PLC registers, odom: nav_msgs/Odometry topic
        self.wheel_source = rospy.get_param("~wheel_source", "modbus")
        self.wheel_odom_topic = rospy.get_param("~wheel_odom_topic", "/odom")
        self.modbus_topic = rospy.get_param("~modbus_topic", "/modbus_wrapper/input")
        # register offsets, same as plc_driver.yaml input_fields
        self.vel_x_index = rospy.get_param("~vel_x_index", 6)
        self.vel_z_index = rospy.get_param("~vel_z_index", 10)
        # auto_manual_sw register, re-sync the pose to TF when the mode changes
        self.mode_index = rospy.get_param("~mode_index", 14)
        self.reset_on_mode_change = rospy.get_param("~reset_on_mode_change", True)
        # re-sync to the TF pose when the NAV350 mirror comes back (MRPT only
        # uses /odom_imu while the mirror is lost, so this never moves MRPT).
        # Delayed so forklift_tf_publisher's NAV350 pose is in TF first.
        self.reset_on_nav_back = rospy.get_param("~reset_on_nav_back", True)
        self.mirror_topic = rospy.get_param("~mirror_topic", "/mirror_detect")
        self.nav_back_delay = rospy.get_param("~nav_back_delay", 0.5)
        # re-sync to the TF pose the moment the mirror is lost, so dead
        # reckoning starts from the last NAV350 pose (MRPT re-aligns on the jump)
        self.reset_on_nav_lost = rospy.get_param("~reset_on_nav_lost", True)
        self.use_imu_yaw = rospy.get_param("~use_imu_yaw", True)
        self.publish_rate = rospy.get_param("~publish_rate", 20.0)
        self.path_length = rospy.get_param("~path_length", 2000)
        # sign/scale of PLC speed, flip if rviz shows the robot going backwards
        self.vel_scale = rospy.get_param("~vel_scale", 1.0)
        # cmd: direction from the command sent to the PLC, imu: from IMU accel,
        # plc: use PLC sign (PLC speed is magnitude only for cmd/imu)
        self.vel_sign_source = rospy.get_param("~vel_sign_source", "cmd")
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel_send_plc")
        # a command older than this is not trusted to explain a start
        self.cmd_timeout = rospy.get_param("~cmd_timeout", 2.0)
        self.cmd_min_vel = rospy.get_param("~cmd_min_vel", 0.001)
        # PLC speed at which the IMU direction is latched after a start
        self.latch_vel = rospy.get_param("~latch_vel", 0.03)
        # MID-360 reports acceleration in g
        self.acc_scale = rospy.get_param("~acc_scale", 9.80665)
        # gyro bias is learnt only while the robot is standing still
        self.still_vel = rospy.get_param("~still_vel", 0.005)
        self.still_time = rospy.get_param("~still_time", 1.0)
        self.bias_alpha = rospy.get_param("~bias_alpha", 0.01)
        self.wheel_timeout = rospy.get_param("~wheel_timeout", 0.5)

        self.lock = threading.Lock()
        self.tf_listener = tf.TransformListener()
        self.R_imu = None  # 3x3 rotation livox_frame -> base_frame
        # child_frame pose in base_frame (x, y, yaw), None until looked up
        self.child_offset = (0.0, 0.0, 0.0) if self.child_frame == self.base_frame else None
        self.x = self.y = self.yaw = 0.0
        self.v = 0.0
        self.w_wheel = 0.0
        self.w_used = 0.0
        self.v_used = 0.0
        self.t_wheel = None
        self.t_last = None
        self.t_still = None
        self.gyro_bias = 0.0
        self.acc_bias = 0.0
        self.v_imu = 0.0  # integrated base-x accel since last stop
        self.direction = 0  # latched +1/-1, 0 = not decided yet
        self.last_direction = 1
        self.cmd_sign = 0  # sign of the last non-zero speed command
        self.t_cmd = None
        self.mode = None  # last auto_manual_sw value
        self.mirror_lost = None  # last /mirror_detect value (True = lost)
        self.need_reset = True

        self.odom_pub = rospy.Publisher("/odom_imu", Odometry, queue_size=10)
        self.path_pub = rospy.Publisher("/odom_imu/path", Path, queue_size=1)
        self.path = Path()
        self.path.header.frame_id = self.global_frame

        if self.wheel_source == "modbus":
            self.wheel_name = self.modbus_topic
            rospy.Subscriber(self.modbus_topic, HoldingRegister, self.modbus_cb, queue_size=10)
        else:
            self.wheel_name = self.wheel_odom_topic
            rospy.Subscriber(self.wheel_odom_topic, Odometry, self.wheel_cb, queue_size=10)
        rospy.Subscriber(self.imu_topic, Imu, self.imu_cb, queue_size=200)
        if self.vel_sign_source == "cmd":
            rospy.Subscriber(self.cmd_topic, Twist, self.cmd_cb, queue_size=10)
        if self.reset_on_nav_back or self.reset_on_nav_lost:
            rospy.Subscriber(self.mirror_topic, Bool, self.mirror_cb, queue_size=10)
        rospy.Subscriber("~reset", Empty, self.reset_cb, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_cb)
        rospy.loginfo(
            "odom_imu: %s + %s -> /odom_imu in %s, yaw from %s",
            self.wheel_name, self.imu_topic, self.global_frame,
            "IMU gyro" if self.use_imu_yaw else "PLC angular.z",
        )

    def reset_cb(self, _msg):
        with self.lock:
            self.need_reset = True

    def mirror_cb(self, msg):
        lost = bool(msg.data)
        with self.lock:
            back = self.mirror_lost is True and not lost
            gone = self.mirror_lost is not True and lost
            self.mirror_lost = lost
            if gone and self.reset_on_nav_lost:
                # TF odom -> base_footprint still holds the last NAV350 pose
                self.need_reset = True
        if gone and self.reset_on_nav_lost:
            rospy.loginfo("odom_imu: NAV350 mirror lost, reset to last NAV350 pose")
        if back and self.reset_on_nav_back:
            rospy.loginfo("odom_imu: NAV350 mirror back, reset to TF pose in %.1fs", self.nav_back_delay)
            rospy.Timer(rospy.Duration(self.nav_back_delay), self.nav_back_reset, oneshot=True)

    def nav_back_reset(self, _event):
        with self.lock:
            if self.mirror_lost:
                return  # lost again during the delay, keep integrating
            self.need_reset = True

    def cmd_cb(self, msg):
        if abs(msg.linear.x) < self.cmd_min_vel:
            return  # zero command keeps the last direction (robot still braking)
        with self.lock:
            self.cmd_sign = 1 if msg.linear.x > 0 else -1
            self.t_cmd = rospy.get_time()

    def try_reset(self):
        """Start (or re-sync) from the robot pose currently in TF so the
        odom_imu track overlays the real robot in rviz."""
        try:
            trans, rot = self.tf_listener.lookupTransform(self.global_frame, self.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, "odom_imu: waiting for TF %s -> %s: %s", self.global_frame, self.base_frame, e)
            return False
        self.x, self.y, self.yaw = trans[0], trans[1], get_yaw(rot)
        self.path.poses = []
        self.need_reset = False
        rospy.loginfo("odom_imu: start pose x=%.3f y=%.3f yaw=%.3f", self.x, self.y, self.yaw)
        return True

    def lookup_imu_rotation(self, imu_frame):
        try:
            _, rot = self.tf_listener.lookupTransform(self.base_frame, imu_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, "odom_imu: waiting for TF %s -> %s: %s", self.base_frame, imu_frame, e)
            return
        self.R_imu = quaternion_matrix(rot)[:3, :3]
        rospy.loginfo("odom_imu: %s -> %s rotation loaded", imu_frame, self.base_frame)

    def lookup_child_offset(self):
        try:
            trans, rot = self.tf_listener.lookupTransform(self.base_frame, self.child_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, "odom_imu: waiting for TF %s -> %s: %s", self.base_frame, self.child_frame, e)
            return
        self.child_offset = (trans[0], trans[1], get_yaw(rot))
        rospy.loginfo(
            "odom_imu: publishing %s pose, offset in %s x=%.3f y=%.3f yaw=%.3f",
            self.child_frame, self.base_frame, *self.child_offset)

    def set_wheel(self, v, w):
        with self.lock:
            self.v = v * self.vel_scale
            self.w_wheel = w
            self.t_wheel = rospy.get_time()

    def wheel_cb(self, msg):
        self.set_wheel(msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def modbus_cb(self, msg):
        d = msg.data
        if max(self.vel_x_index, self.vel_z_index) + 1 >= len(d):
            rospy.logwarn_throttle(5.0, "odom_imu: %s has only %d registers", self.modbus_topic, len(d))
            return
        v = word_to_float(d[self.vel_x_index], d[self.vel_x_index + 1])
        w = word_to_float(d[self.vel_z_index], d[self.vel_z_index + 1])
        rospy.loginfo_throttle(2.0, "odom_imu: PLC v=%.3f w=%.3f", v, w)
        self.set_wheel(v, w)
        if self.reset_on_mode_change and self.mode_index < len(d):
            self.check_mode(d[self.mode_index])

    def check_mode(self, mode):
        with self.lock:
            if self.mode is not None and mode != self.mode:
                rospy.loginfo("odom_imu: auto_manual_sw %d -> %d, reset to TF pose", self.mode, mode)
                self.need_reset = True
                self.direction = 0
            self.mode = mode

    def imu_cb(self, msg):
        if self.R_imu is None:
            self.lookup_imu_rotation(msg.header.frame_id)
            if self.R_imu is None:
                return
        g = msg.angular_velocity
        a = msg.linear_acceleration
        R = self.R_imu
        # yaw rate around base_frame z, works for a tilted Livox too (the side
        # offset doesn't matter for angular rate)
        gyro_z = R[2, 0] * g.x + R[2, 1] * g.y + R[2, 2] * g.z
        # acceleration along base_frame x, gravity leak from tilt is removed
        # by acc_bias learnt while still
        acc_x = self.acc_scale * (R[0, 0] * a.x + R[0, 1] * a.y + R[0, 2] * a.z)
        t = msg.header.stamp.to_sec()

        with self.lock:
            if self.need_reset and not self.try_reset():
                self.t_last = None
                return
            if self.t_last is None or t <= self.t_last or t - self.t_last > 0.5:
                self.t_last = t
                return
            dt = t - self.t_last
            self.t_last = t

            wheel_ok = self.t_wheel is not None and rospy.get_time() - self.t_wheel < self.wheel_timeout
            v = self.v if wheel_ok else 0.0
            if not wheel_ok:
                rospy.logwarn_throttle(5.0, "odom_imu: no %s, speed held at 0", self.wheel_name)

            still = wheel_ok and abs(v) < self.still_vel and abs(self.w_wheel) < self.still_vel
            if still:
                if self.t_still is None:
                    self.t_still = t
                elif t - self.t_still > self.still_time:
                    self.gyro_bias += self.bias_alpha * (gyro_z - self.gyro_bias)
                    self.acc_bias += self.bias_alpha * (acc_x - self.acc_bias)
                # stopped: velocity is known to be 0, direction is free to change
                self.v_imu = 0.0
                self.direction = 0
            else:
                self.t_still = None
                self.v_imu += (acc_x - self.acc_bias) * dt

            if self.vel_sign_source == "cmd":
                v = self.cmd_signed_speed(abs(v)) * (-1.0 if self.vel_scale < 0 else 1.0)
            elif self.vel_sign_source == "imu":
                v = self.signed_speed(abs(v)) * (-1.0 if self.vel_scale < 0 else 1.0)

            if self.use_imu_yaw:
                w = 0.0 if still else gyro_z - self.gyro_bias
            else:
                w = self.w_wheel if wheel_ok else 0.0
            self.w_used = w
            self.v_used = v

            # midpoint integration
            yaw_mid = self.yaw + 0.5 * w * dt
            self.x += v * cos(yaw_mid) * dt
            self.y += v * sin(yaw_mid) * dt
            self.yaw = atan2(sin(self.yaw + w * dt), cos(self.yaw + w * dt))

    def cmd_signed_speed(self, speed):
        """PLC speed magnitude -> signed speed, direction from the command sent
        to the PLC, latched at the start. Called with self.lock held."""
        if self.direction == 0 and speed >= self.still_vel:
            fresh = self.t_cmd is not None and rospy.get_time() - self.t_cmd < self.cmd_timeout
            if fresh:
                self.direction = self.cmd_sign
            else:
                # moving without a recent command (e.g. manual drive)
                self.direction = self.last_direction
                rospy.logwarn(
                    "odom_imu: moving without %s, keep %+d", self.cmd_topic, self.direction)
            self.last_direction = self.direction
            rospy.loginfo(
                "odom_imu: moving %s (cmd %+d, PLC %.3f)",
                "forward" if self.direction > 0 else "backward", self.cmd_sign, speed)
        return (self.direction or self.last_direction) * speed

    def signed_speed(self, speed):
        """PLC speed magnitude -> signed speed, direction from the IMU.
        Called with self.lock held."""
        if self.direction == 0:
            if speed < self.latch_vel:
                # still accelerating: provisional sign, the step is tiny anyway
                return speed if self.v_imu >= 0 else -speed
            if abs(self.v_imu) >= 0.3 * speed:
                self.direction = 1 if self.v_imu > 0 else -1
            else:
                # IMU too weak/noisy to tell, assume same way as last time
                self.direction = self.last_direction
                rospy.logwarn(
                    "odom_imu: IMU direction unclear (v_imu=%.3f, PLC %.3f), keep %+d",
                    self.v_imu, speed, self.direction)
            self.last_direction = self.direction
            rospy.loginfo(
                "odom_imu: moving %s (v_imu=%.3f, PLC %.3f)",
                "forward" if self.direction > 0 else "backward", self.v_imu, speed)
        return self.direction * speed

    def publish_cb(self, _event):
        with self.lock:
            if self.need_reset:
                if self.use_imu_yaw:
                    return  # imu_cb resets on the next sample
                self.try_reset()
                return
            x, y, yaw, v, w = self.x, self.y, self.yaw, self.v_used, self.w_used

        if self.child_offset is None:
            self.lookup_child_offset()
            if self.child_offset is None:
                return
        # base_frame pose -> child_frame pose (rigid offset on the robot)
        ox, oy, oyaw = self.child_offset
        cx = x + ox * cos(yaw) - oy * sin(yaw)
        cy = y + ox * sin(yaw) + oy * cos(yaw)
        cyaw = atan2(sin(yaw + oyaw), cos(yaw + oyaw))
        # velocity of that point, in child_frame axes
        vbx, vby = v - w * oy, w * ox
        cvx = vbx * cos(oyaw) + vby * sin(oyaw)
        cvy = -vbx * sin(oyaw) + vby * cos(oyaw)

        now = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.global_frame
        odom.child_frame_id = self.child_frame
        odom.pose.pose.position.x = cx
        odom.pose.pose.position.y = cy
        odom.pose.pose.orientation = yaw_to_quaternion(cyaw)
        odom.twist.twist.linear.x = cvx
        odom.twist.twist.linear.y = cvy
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        ps = PoseStamped()
        ps.header = odom.header
        ps.pose = odom.pose.pose
        self.path.header.stamp = now
        self.path.poses.append(ps)
        if len(self.path.poses) > self.path_length:
            del self.path.poses[: len(self.path.poses) - self.path_length]
        self.path_pub.publish(self.path)


def main():
    OdomImu()
    rospy.spin()


if __name__ == "__main__":
    main()
