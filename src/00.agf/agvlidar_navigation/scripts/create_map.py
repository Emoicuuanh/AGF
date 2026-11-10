#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32MultiArray as HoldingRegister
from std_msgs.msg import Float64, String, UInt16
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
from geometry_msgs.msg import Twist
import struct
import threading
from agf_msgs.msg import Int8Array, SafetyStt

def laser_odom_cb(msg):
    # print msg.pose.pose
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    ox = msg.pose.pose.orientation.x
    oy = msg.pose.pose.orientation.y
    oz = msg.pose.pose.orientation.z
    ow = msg.pose.pose.orientation.w
    z =  euler_from_quaternion([ox,oy,oz,ow])

    br = tf.TransformBroadcaster()
    br.sendTransform((x, y, 0.0),
                    (ox, oy, oz, ow),
                    rospy.Time.now(),
                    "front_mount",
                    "odom")

    laser_quaternion = quaternion_from_euler(0,0,3.141592)
    br.sendTransform((1.06, 0, 0),
                    (laser_quaternion[0], laser_quaternion[1], laser_quaternion[2], laser_quaternion[3]),
                    rospy.Time.now(),
                    "base_link",
                    "front_mount")

if __name__ == "__main__":
    rospy.init_node('create_map', anonymous=True)
    rospy.Subscriber('/nav350laser/odom', Odometry, laser_odom_cb)
    rospy.loginfo('Init create map node')
    rate = rospy.Rate(10.0)
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():

        rate.sleep()
    rospy.spin()