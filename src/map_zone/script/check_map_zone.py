#!/usr/bin/env python3

import json
import math
import rospy
# import matplotlib.pyplot as plt
from pymongo import MongoClient
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from std_msgs.msg import Header
from std_stamped_msgs.msg import StringStamped
import numpy as np
from shapely.geometry import Point, Polygon
import time
import random

class MapZoneReaderNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('map_zone_reader_node', anonymous=True)

        # Get parameters from the launch file or parameter server
        self.mongo_uri = rospy.get_param('~mongodb_address', "mongodb://localhost:27017/")
        # self.map_name = rospy.get_param('~map_name', "default_map")
        self.db_name = rospy.get_param('~db_name', "admin")
        self.collection_name = rospy.get_param('~collection_name', "datalist")
        self.heading_alignment_tolerance_deg = rospy.get_param('~heading_alignment_tolerance_deg', 30.0)

        # Initialize MongoDB connection
        self.client = None
        self.db = None
        self.collection = None
        self.zones = None
        self.map_name = None
        self.connect()

        # Subscribe to /tf topic
        self.base_link_position = None
        self.base_link_pose = None
        self.base_link_rpy = None
        rospy.Subscriber('/robot_pose', Pose, self.pose_callback)
        rospy.Subscriber('/current_map', StringStamped, self.map_callback)

        # Publisher for zone list
        self.zone_publisher = rospy.Publisher('/current_zones', StringStamped, queue_size=1)

        self.name_zone_publisher = rospy.Publisher('/zone_status', StringStamped, queue_size=1)

        self.heading_alignment_publisher = rospy.Publisher('/zone_heading_alignment', StringStamped, queue_size=1)

        # Initialize last published zone list as empty
        # self.last_zones = []
        self.last_zones = None
        self.last_map_name = None


    def connect(self):
        """
        Connect to MongoDB.
        """
        try:
            self.client = MongoClient(self.mongo_uri)
            self.db = self.client[self.db_name]
            self.collection = self.db[self.collection_name]
            rospy.loginfo(f"Successfully connected to MongoDB at {self.mongo_uri}")
        except Exception as e:
            rospy.logerr(f"Error connecting to MongoDB: {e}")

    # def get_zones_by_map_name(self, map_name):
    #     """
    #     Fetch zones based on map name from MongoDB.
    #     """
    #     try:
    #         # Query zones based on the map name
    #         documents = self.collection.find({"map": map_name, "type": "zone"})
    #         zones = {}

    #         for doc in documents:
    #             # Use the string representation of the _id as the key
    #             zone_id = str(doc["_id"])
    #             zones[zone_id] = {
    #                 "type_zone": doc.get("name", ""),
    #                 "polygon": doc.get("data", {}).get("polygon", []),
    #                 "properties": doc.get("data", {}).get("properties", {})
    #             }

    #         return zones

    #     except Exception as e:
    #         rospy.logerr(f"Error querying MongoDB: {e}")
    #         return {}

    def get_zones_by_map_name(self, map_name):
        """
        Fetch zones based on map name from MongoDB.
        """
        try:
            # Query zones based on the map name
            documents = self.collection.find({"map": map_name, "type": "zone"})
            zones = {}

            for doc in documents:
                zone_id = str(doc["_id"])
                data = doc.get("data", {})
                polygon = data.get("polygon", doc.get("polygon", []))

                properties = data.get("properties", doc.get("properties", {})).copy()
                if "max_velocity" in data:
                    properties["max_velocity"] = data["max_velocity"]
                if "safety_job" in data:
                    properties["safety_job"] = data["safety_job"]

                zones[zone_id] = {
                    "type_zone": doc.get("name", ""),
                    "polygon": polygon,
                    "properties": properties
                }

            return zones

        except Exception as e:
            rospy.logerr(f"Error querying MongoDB: {e}")
            return {}



    def pose_callback(self, msg):
        x = msg.position.x
        y = msg.position.y
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.base_link_position = (x, y)
        self.base_link_pose = (x, y, yaw)
        self.base_link_rpy = (roll, pitch, yaw)
        # rospy.loginfo(f"Received robot_pose: x={x}, y={y}, yaw={yaw}")

    def map_callback(self, msg):
        self.map_name = msg.data
        if self.map_name != self.last_map_name:
            rospy.loginfo(f"Received current_map: {self.map_name}")
            self.last_map_name = self.map_name

    def get_polygon_main_axis_angle(self, polygon_points):
        """
        Return the polygon main-axis angle in degrees using the longest edge of
        the minimum rotated rectangle. The result is normalized to [0, 180).
        """
        if len(polygon_points) < 3:
            return None

        polygon = Polygon([(point['x'], point['y']) for point in polygon_points])
        if polygon.is_empty or polygon.area == 0:
            return None

        rectangle = polygon.minimum_rotated_rectangle
        rectangle_points = list(rectangle.exterior.coords)[:4]

        edges = []
        for index in range(4):
            x1, y1 = rectangle_points[index]
            x2, y2 = rectangle_points[(index + 1) % 4]
            dx = x2 - x1
            dy = y2 - y1
            edge_length = math.hypot(dx, dy)
            edges.append((edge_length, dx, dy))

        _, dx, dy = max(edges, key=lambda edge: edge[0])
        return math.degrees(math.atan2(dy, dx)) % 180

    def get_heading_alignment(self, robot_yaw_rad, zone_angle_deg):
        """
        Compare robot heading with the zone main axis and return True if aligned.
        Angles are compared modulo 180 because an axis has no forward direction.
        Returns False if angle is unknown or robot yaw is unavailable.
        """
        if zone_angle_deg is None or robot_yaw_rad is None:
            return False

        robot_heading_deg = math.degrees(robot_yaw_rad) % 180
        heading_error_deg = abs(((robot_heading_deg - zone_angle_deg + 90.0) % 180.0) - 90.0)

        return heading_error_deg <= self.heading_alignment_tolerance_deg

    def check_zone(self, x, y):
        """
        Check which zones the (x, y) point is inside and return zone types with their properties.
        """
        point = Point(x, y)
        zones_in = []
        robot_yaw = self.base_link_pose[2] if self.base_link_pose is not None else None

        for zone_id, zone_data in self.zones.items():
            polygon_points = zone_data.get('polygon', [])
            polygon = Polygon([(p['x'], p['y']) for p in polygon_points])

            # Check if the point is inside the polygon
            if polygon.covers(point):
                type_zone = zone_data.get('type_zone', 'unknown')
                properties = zone_data.get('properties', {})  # Get properties of the zone
                zone_angle_deg = self.get_polygon_main_axis_angle(polygon_points)
                alignment = self.get_heading_alignment(robot_yaw, zone_angle_deg)
                # Add zone type and properties to the result
                zones_in.append({
                    "type_zone": type_zone,
                    "properties": properties,
                    "heading_alignment": alignment,
                })

        return zones_in

    def merge_zones(self, zones_in):
        merged_type_zone = []
        merged_properties = {}

        for zone in zones_in:
            if zone['type_zone'] not in merged_type_zone:
                merged_type_zone.append(zone['type_zone'])

        aligned_zones = [z for z in zones_in if z.get('heading_alignment')]
        non_aligned_zones = [z for z in zones_in if not z.get('heading_alignment')]

        # Non-aligned zones fill in properties first, aligned zones overwrite (aligned takes priority)
        for zone in non_aligned_zones:
            for key, value in zone['properties'].items():
                if key not in merged_properties:
                    merged_properties[key] = value

        for zone in aligned_zones:
            for key, value in zone['properties'].items():
                merged_properties[key] = value

        # True if at least one zone is aligned (its properties will be used)
        merged_heading_alignment = len(aligned_zones) > 0

        return [{'type_zone': merged_type_zone, 'properties': merged_properties, 'heading_alignment': merged_heading_alignment}]

    def run(self):
        """
        Main logic of the node.
        """
        rate = rospy.Rate(15)  # 15 Hz
        while not rospy.is_shutdown():
            t1 = time.time()
            # Only proceed if map_name and base_link_position are available
            if self.map_name is not None \
                and self.base_link_position is not None \
                and self.base_link_pose is not None:
                # Fetch zones for the given map
                self.zones = self.get_zones_by_map_name(self.map_name)
                if self.zones:
                    pass
                    # rospy.loginfo(f"Found {len(self.zones)} zones for map '{self.map_name}'.",
                    # )
                else:
                    rospy.logwarn(f"No zones found for map '{self.map_name}'.")
                    self.last_zones = []  # Reset last_zones to avoid stale data

                # Check zones only if zones were successfully fetched
                x, y = self.base_link_position
                zones_in = self.check_zone(x, y) if self.zones else []
                zones_merged = self.merge_zones(zones_in)
                self.last_zones = zones_in
                rospy.loginfo_throttle(
                    2.0,
                    f"Publishing current_zones for map='{self.map_name}', "
                    f"pose=({x}, {y}), zones_in={zones_merged}",
                )
                # self.zone_publisher.publish(str(zones_merged))
                # self.zone_publisher.publish(json.dumps(zones_merged))
                self.zone_publisher.publish(stamp=rospy.Time.now(), data=json.dumps(zones_merged[0]))

                heading_alignment_status = []
                for zone in zones_in:
                    alignment = zone.get('heading_alignment', {})
                    heading_alignment_status.append({
                        "zone_name": zone.get('type_zone', ''),
                        "aligned": alignment
                    })

                self.heading_alignment_publisher.publish(
                    stamp=rospy.Time.now(),
                    data=json.dumps(heading_alignment_status),
                )

                # Publish each type_zone string in zones_merged
                for zone in zones_merged:
                    type_zones = zone.get('type_zone', [])
                    if not type_zones:
                        stt_dict = {"zone_name": ""}
                        self.name_zone_publisher.publish(stamp=rospy.Time.now(), data=json.dumps(stt_dict))
                    else:
                        for type_zone in type_zones:
                            stt_dict = {"zone_name": f"{type_zone}"}
                            self.name_zone_publisher.publish(stamp=rospy.Time.now(), data=json.dumps(stt_dict))
            else:
                missing_inputs = []
                if self.map_name is None:
                    missing_inputs.append("current_map")
                if self.base_link_position is None or self.base_link_pose is None:
                    missing_inputs.append("robot_pose")
                rospy.logwarn_throttle(
                    2.0,
                    "Waiting for {} to be set.".format(", ".join(missing_inputs)),
                )
            rate.sleep()

        # Cleanup on shutdown
        self.shutdown()


    def shutdown(self):
        """
        Cleanup when shutting down the node.
        """
        if self.client:
            self.client.close()
            rospy.loginfo("MongoDB connection closed.")


if __name__ == '__main__':
    try:
        node = MapZoneReaderNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
