#!/usr/bin/env python3
"""gazebo_ros_block_laser -> Livox-like PointCloud2.

The block laser publishes sensor_msgs/PointCloud (+ "intensity" channel) and reports a miss
as a point at max range. The detectors read what livox_ros_driver2 publishes: PointCloud2
x, y, z, intensity (float32) in livox_frame on /livox/lidar. Misses are dropped here.
"""
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud, PointCloud2, PointField

FIELDS = [PointField("x", 0, PointField.FLOAT32, 1),
          PointField("y", 4, PointField.FLOAT32, 1),
          PointField("z", 8, PointField.FLOAT32, 1),
          PointField("intensity", 12, PointField.FLOAT32, 1)]


class BlockLaserToCloud2:
    def __init__(self):
        self.min_range = rospy.get_param("~min_range", 0.1)
        self.max_range = rospy.get_param("~max_range", 20.0)
        self.frame_id = rospy.get_param("~frame_id", "livox_frame")
        self.pub = rospy.Publisher("points_out", PointCloud2, queue_size=1)
        rospy.Subscriber("points_in", PointCloud, self.cloud_cb, queue_size=1, buff_size=2**24)

    def cloud_cb(self, msg):
        n = len(msg.points)
        if n == 0:
            return
        xyz = np.array([(p.x, p.y, p.z) for p in msg.points], np.float32)
        inten = np.zeros(n, np.float32)
        if msg.channels and len(msg.channels[0].values) == n:
            inten = np.asarray(msg.channels[0].values, np.float32)
        r = np.linalg.norm(xyz, axis=1)
        keep = (r > self.min_range) & (r < self.max_range - 0.01)
        cloud = np.column_stack([xyz[keep], inten[keep]]).astype(np.float32)

        out = PointCloud2()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id
        out.height, out.width = 1, len(cloud)
        out.fields = FIELDS
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = 16 * len(cloud)
        out.is_dense = True
        out.data = cloud.tobytes()
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("block_laser_to_cloud2")
    BlockLaserToCloud2()
    rospy.spin()
