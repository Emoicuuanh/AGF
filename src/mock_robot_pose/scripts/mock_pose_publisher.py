#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import Pose

class RobotPosePublisher:
    def __init__(self):
        rospy.init_node('robot_pose_publisher', anonymous=False)
        
        # Parameters
        self.rate_hz = rospy.get_param('~rate', 10.0)  # Default 10 Hz
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.base_link_frame = rospy.get_param('~base_link_frame', 'base_link')
        
        # TF Listener
        self.tf_listener = tf.TransformListener()
        
        # Publisher
        self.pose_pub = rospy.Publisher('/robot_pose', Pose, queue_size=10)
        
        rospy.loginfo("Robot Pose Publisher started")
        rospy.loginfo("Publishing /robot_pose at {} Hz".format(self.rate_hz))
        rospy.loginfo("Transform: {} -> {}".format(self.map_frame, self.base_link_frame))
        
    def run(self):
        rate = rospy.Rate(self.rate_hz)
        
        while not rospy.is_shutdown():
            try:
                # Lookup transform from map to base_link
                (trans, rot) = self.tf_listener.lookupTransform(
                    self.map_frame, 
                    self.base_link_frame, 
                    rospy.Time(0)
                )
                
                # Create Pose message
                pose_msg = Pose()
                pose_msg.position.x = trans[0]
                pose_msg.position.y = trans[1]
                pose_msg.position.z = trans[2]
                pose_msg.orientation.x = rot[0]
                pose_msg.orientation.y = rot[1]
                pose_msg.orientation.z = rot[2]
                pose_msg.orientation.w = rot[3]
                
                # Publish
                self.pose_pub.publish(pose_msg)
                
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn_throttle(5.0, "TF lookup failed: {}".format(e))
                
            rate.sleep()

if __name__ == '__main__':
    try:
        node = RobotPosePublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass
