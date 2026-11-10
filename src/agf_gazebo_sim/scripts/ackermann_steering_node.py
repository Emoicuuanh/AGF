#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class AckermannSteeringNode:
    def __init__(self):
        rospy.init_node('ackermann_steering_node')
        
        # Robot parameters
        self.wheelbase = 1.36  # distance between front and rear axles
        self.track_width = 0.7  # distance between left and right wheels
        
        # Publishers for steering joint commands
        self.left_steering_pub = rospy.Publisher('/agf_model/left_steering_position_controller/command', Float64, queue_size=1)
        self.right_steering_pub = rospy.Publisher('/agf_model/right_steering_position_controller/command', Float64, queue_size=1)
        
        # Publishers for wheel velocity commands
        self.front_left_wheel_pub = rospy.Publisher('/agf_model/front_left_wheel_velocity_controller/command', Float64, queue_size=1)
        self.front_right_wheel_pub = rospy.Publisher('/agf_model/front_right_wheel_velocity_controller/command', Float64, queue_size=1)
        self.rear_left_wheel_pub = rospy.Publisher('/agf_model/rear_left_wheel_velocity_controller/command', Float64, queue_size=1)
        self.rear_right_wheel_pub = rospy.Publisher('/agf_model/rear_right_wheel_velocity_controller/command', Float64, queue_size=1)
        
        # Subscriber for cmd_vel
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.left_angle = 0.0
        self.right_angle = 0.0
        self.front_left_vel = 0.0
        self.front_right_vel = 0.0
        self.rear_left_vel = 0.0
        self.rear_right_vel = 0.0
        self.turning_radius = float('inf')
        
        rospy.loginfo("Ackermann Steering Node started")
        
    def cmd_vel_callback(self, msg):
        print("Received cmd_vel:", msg)
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        wheel_radius = 0.5  # 0.1 m

        # ✅ Default values (avoid unbound vars)
        self.left_angle = self.right_angle = 0.0
        self.front_left_vel = self.front_right_vel = self.rear_left_vel = self.rear_right_vel = 0.0

        # ✅ Handle motion logic
        if abs(linear_vel) > 1e-6:  # Only compute steering when actually moving
            if abs(angular_vel) < 1e-6:
                # Straight movement
                self.turning_radius = float('inf')
                self.left_angle = self.right_angle = 0.0
                self.front_left_vel = self.front_right_vel = self.rear_left_vel = self.rear_right_vel = linear_vel / wheel_radius
            else:
                # Turning motion
                self.turning_radius = linear_vel / angular_vel

                if self.turning_radius > 0:  # Turning left
                    self.left_angle  = math.atan(self.wheelbase / (self.turning_radius - self.track_width/2))
                    self.right_angle = math.atan(self.wheelbase / (self.turning_radius + self.track_width/2))
                    self.front_left_vel  = linear_vel * (self.turning_radius - self.track_width/2) / (self.turning_radius * wheel_radius)
                    self.front_right_vel = linear_vel * (self.turning_radius + self.track_width/2) / (self.turning_radius * wheel_radius)
                else:  # Turning right
                    self.turning_radius = abs(self.turning_radius)
                    self.left_angle  = -math.atan(self.wheelbase / (self.turning_radius + self.track_width/2))
                    self.right_angle = -math.atan(self.wheelbase / (self.turning_radius - self.track_width/2))
                    self.front_left_vel  = linear_vel * (self.turning_radius + self.track_width/2) / (self.turning_radius * wheel_radius)
                    self.front_right_vel = linear_vel * (self.turning_radius - self.track_width/2) / (self.turning_radius * wheel_radius)

                self.rear_left_vel = self.front_left_vel
                self.rear_right_vel = self.front_right_vel

        # ✅ Log joint commands using ROS
        rospy.loginfo("Ackermann steering calculation:")
        rospy.loginfo("  Input: linear_vel=%.3f m/s, angular_vel=%.3f rad/s, turning_radius=%.3f m",
                    linear_vel, angular_vel, self.turning_radius)
        rospy.loginfo("  Steering angles (radians): left=%.3f, right=%.3f",
                    self.left_angle, self.right_angle)
        rospy.loginfo("  Wheel velocities (rad/s): FL=%.3f, FR=%.3f, RL=%.3f, RR=%.3f",
                    self.front_left_vel, self.front_right_vel, self.rear_left_vel, self.rear_right_vel)

        # ✅ Clamp steering to safe limit
        max_angle = 0.5236
        self.left_angle = max(-max_angle, min(max_angle, self.left_angle))
        self.right_angle = max(-max_angle, min(max_angle, self.right_angle))

        # ✅ Publish steering commands
        self.left_steering_pub.publish(Float64(self.left_angle))
        self.right_steering_pub.publish(Float64(self.right_angle))

        # ✅ Publish wheel speed
        self.front_left_wheel_pub.publish(Float64(self.front_left_vel))
        self.front_right_wheel_pub.publish(Float64(self.front_right_vel))
        self.rear_left_wheel_pub.publish(Float64(self.rear_left_vel))
        self.rear_right_wheel_pub.publish(Float64(self.rear_right_vel))

        rospy.logdebug(f"[Ackermann] L={self.left_angle:.3f}, R={self.right_angle:.3f} | FL={self.front_left_vel:.2f}, FR={self.front_right_vel:.2f}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = AckermannSteeringNode()
        node.run()
    except rospy.ROSInterruptException:
        pass