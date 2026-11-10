#!/usr/bin/env python3

import json
import rospy
# import matplotlib.pyplot as plt
from pymongo import MongoClient
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

        # Initialize MongoDB connection
        self.client = None
        self.db = None
        self.collection = None
        self.zones = None
        self.map_name = None
        self.connect()

        # Subscribe to /tf topic
        self.base_link_position = None
        rospy.Subscriber('/robot_pose', Pose, self.pose_callback)
        rospy.Subscriber('/current_map', StringStamped, self.map_callback)

        # Publisher for zone list
        self.zone_publisher = rospy.Publisher('/current_zones', StringStamped, queue_size=1)

        self.name_zone_publisher = rospy.Publisher('/zone_status', StringStamped, queue_size=1)

        # Initialize last published zone list as empty
        # self.last_zones = []
        self.last_zones = None
        

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
                polygon = data.get("polygon", [])

                # Tạo mới properties từ 2 trường
                properties = {}
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
        self.base_link_position = (x, y)
        # rospy.loginfo(f"Base link position relative to map: x={x}, y={y}")
       
    def map_callback(self, msg):
        self.map_name = msg.data

    def check_zone(self, x, y):
        """
        Check which zones the (x, y) point is inside and return zone types with their properties.
        """
        point = Point(x, y)
        zones_in = []

        for zone_id, zone_data in self.zones.items():
            polygon_points = zone_data.get('polygon', [])
            polygon = Polygon([(p['x'], p['y']) for p in polygon_points])
            
            # Check if the point is inside the polygon
            if polygon.contains(point):
                type_zone = zone_data.get('type_zone', 'unknown')
                properties = zone_data.get('properties', {})  # Get properties of the zone
                
                # Log the properties for debugging purposes
                # rospy.loginfo(f"Position ({x}, {y}) is inside zone {zone_id}. Zone type: {type_zone}. Properties: {properties}")
                
                # Add zone type and properties to the result
                zones_in.append({
                    "type_zone": type_zone,
                    "properties": properties
                })

        return zones_in

    def merge_zones(self, zones_in):
        merged_type_zone = []
        merged_properties = {}

        for zone in zones_in:
            if zone['type_zone'] not in merged_type_zone:
                merged_type_zone.append(zone['type_zone'])

            for key, value in zone['properties'].items():
                if key in merged_properties:
                    merged_properties[key] = random.choice([merged_properties[key], value])
                else:
                    merged_properties[key] = value

        return [{'type_zone': merged_type_zone, 'properties': merged_properties}]

    def run(self):
        """
        Main logic of the node.
        """
        rate = rospy.Rate(15)  # 15 Hz
        while not rospy.is_shutdown():
            t1 = time.time()
            # Only proceed if map_name and base_link_position are available
            if self.map_name is not None and self.base_link_position is not None:
                # Fetch zones for the given map
                self.zones = self.get_zones_by_map_name(self.map_name)
                if self.zones:
                    pass
                    # rospy.loginfo(f"Zones for map '{self.map_name}':")
                    # for zone_id, zone_data in self.zones.items():
                    #     rospy.loginfo(f"Zone ID: {zone_id}, Data: {zone_data}")
                else:
                    rospy.logwarn(f"No zones found for map '{self.map_name}'.")
                    self.last_zones = []  # Reset last_zones to avoid stale data

                # Check zones only if zones were successfully fetched
                x, y = self.base_link_position
                zones_in = self.check_zone(x, y) if self.zones else []
                # Only publish if the zone list has changed
                # if zones_in != self.last_zones:
                zones_merged = self.merge_zones(zones_in)
                self.last_zones = zones_in
                rospy.loginfo(f"Publishing zones: {zones_merged}")
                # self.zone_publisher.publish(str(zones_merged))
                # self.zone_publisher.publish(json.dumps(zones_merged))
                self.zone_publisher.publish(stamp=rospy.Time.now(), data=json.dumps(zones_merged[0]))



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
                rospy.logwarn_once("Waiting for map_name or base_link_position to be set.")
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



































# #!/usr/bin/env python3

# import rospy
# import matplotlib.pyplot as plt
# from pymongo import MongoClient
# # from tf2_msgs.msg import TFMessage
# from geometry_msgs.msg import Pose
# from std_msgs.msg import String
# import numpy as np
# from shapely.geometry import Point, Polygon
# import time
# import random

# class MapZoneReaderNode:
#     def __init__(self):
#         # Initialize ROS node
#         rospy.init_node('map_zone_reader_node', anonymous=True)

#         # Get parameters from the launch file or parameter server
#         self.mongo_uri = rospy.get_param('~mongodb_address', "mongodb://localhost:27017/")
#         self.map_name = rospy.get_param('~map_name', "default_map")
#         self.db_name = rospy.get_param('~db_name', "admin")
#         self.collection_name = rospy.get_param('~collection_name', "map_zone")

#         # Initialize MongoDB connection
#         self.client = None
#         self.db = None
#         self.collection = None
#         self.zones = None
#         self.connect()

#         # Subscribe to /tf topic
#         self.base_link_position = None
#         rospy.Subscriber('/robot_pose', Pose, self.pose_callback)

#         # Publisher for zone list
#         self.zone_publisher = rospy.Publisher('/zones_in', String, queue_size=0)

#         # Initialize last published zone list as empty
#         # self.last_zones = []
#         self.last_zones = None

#     def connect(self):
#         """
#         Connect to MongoDB.
#         """
#         try:
#             self.client = MongoClient(self.mongo_uri)
#             self.db = self.client[self.db_name]
#             self.collection = self.db[self.collection_name]
#             rospy.loginfo(f"Successfully connected to MongoDB at {self.mongo_uri}")
#         except Exception as e:
#             rospy.logerr(f"Error connecting to MongoDB: {e}")

#     def get_zones_by_map_name(self, map_name):
#         """
#         Fetch zones based on map name from MongoDB.
#         """
#         try:
#             document = self.collection.find_one({"name": map_name})
#             if document:
#                 return document.get("zones", {})
#             else:
#                 rospy.logwarn(f"No map found with name: {map_name}")
#                 return {}
#         except Exception as e:
#             rospy.logerr(f"Error querying MongoDB: {e}")
#             return {}

#     def pose_callback(self, msg):
#         x = msg.position.x
#         y = msg.position.y
#         self.base_link_position = (x, y)
#         # rospy.loginfo(f"Base link position relative to map: x={x}, y={y}")
#         """
#         Callback to process transformations from the /tf topic.
#         """
#         # for transform in msg.transforms:
#         #     if transform.header.frame_id == "map" and transform.child_frame_id == "base_link":
#         #         x = transform.transform.translation.x
#         #         y = transform.transform.translation.y
#         #         self.base_link_position = (x, y)
#         #         rospy.loginfo(f"Base link position relative to map: x={x}, y={y}")

#     # def check_zone(self, x, y):
#     #     """
#     #     Check which zones the (x, y) point is inside.
#     #     """
#     #     point = Point(x, y)
#     #     zones_in = []

#     #     for zone_id, zone_data in self.zones.items():
#     #         polygon_points = zone_data.get('polygon', [])
#     #         polygon = Polygon([(p['x'], p['y']) for p in polygon_points])
            
#     #         # Check if the point is inside the polygon
#     #         if polygon.contains(point):
#     #             type_zone = zone_data.get('type_zone', 'unknown')
#     #             if type_zone not in zones_in:  # Only add if not already in zones_in
#     #                 # rospy.loginfo(f"Position ({x}, {y}) is inside zone {zone_id}. Zone type: {type_zone}")
#     #                 zones_in.append(type_zone)

#     #     return zones_in

#     def check_zone(self, x, y):
#         """
#         Check which zones the (x, y) point is inside and return zone types with their properties.
#         """
#         point = Point(x, y)
#         zones_in = []

#         for zone_id, zone_data in self.zones.items():
#             polygon_points = zone_data.get('polygon', [])
#             polygon = Polygon([(p['x'], p['y']) for p in polygon_points])
            
#             # Check if the point is inside the polygon
#             if polygon.contains(point):
#                 type_zone = zone_data.get('type_zone', 'unknown')
#                 properties = zone_data.get('properties', {})  # Get properties of the zone
                
#                 # Log the properties for debugging purposes
#                 # rospy.loginfo(f"Position ({x}, {y}) is inside zone {zone_id}. Zone type: {type_zone}. Properties: {properties}")
                
#                 # Add zone type and properties to the result
#                 zones_in.append({
#                     "type_zone": type_zone,
#                     "properties": properties
#                 })

#         return zones_in

#     def merge_zones(self, zones_in):
#         """
#         Merge all zones into a single entry with:
#         - type_zone: a set of unique type_zone values.
#         - properties: merged properties where conflicting values are chosen randomly.
#         """
#         merged_type_zone = set()
#         merged_properties = {}

#         for zone in zones_in:
#             # Gộp type_zone
#             merged_type_zone.add(zone['type_zone'])

#             # Gộp properties
#             for key, value in zone['properties'].items():
#                 if key in merged_properties:
#                     # Nếu key đã tồn tại, chọn ngẫu nhiên giữa giá trị hiện tại và giá trị mới
#                     merged_properties[key] = random.choice([merged_properties[key], value])
#                 else:
#                     # Nếu key chưa tồn tại, thêm vào
#                     merged_properties[key] = value

#         # Kết quả cuối cùng
#         return [{'type_zone': merged_type_zone, 'properties': merged_properties}]

#     def plot_zones(self):
#         """
#         Plot zones on a 2D graph using matplotlib.
#         """
#         fig, ax = plt.subplots()

#         for zone_id, zone_data in self.zones.items():
#             polygon = zone_data.get('polygon', [])
#             if polygon:
#                 x_coords = [p['x'] for p in polygon]
#                 y_coords = [p['y'] for p in polygon]

#                 # Close the polygon
#                 x_coords.append(x_coords[0])
#                 y_coords.append(y_coords[0])

#                 ax.fill(x_coords, y_coords, alpha=0.5, label=f"Zone {zone_id} ({zone_data.get('type_zone', 'unknown')})")

#         ax.set_xlabel('X Coordinate')
#         ax.set_ylabel('Y Coordinate')
#         ax.set_title(f"Zones for {self.map_name}")
#         ax.legend(loc='best')
#         plt.grid(True)
#         plt.show()

#     def run(self):
#         """
#         Main logic of the node.
#         """
#         # Fetch zones for the given map
#         self.zones = self.get_zones_by_map_name(self.map_name)

#         if self.zones:
#             rospy.loginfo(f"Zones for map '{self.map_name}':")
#             for zone_id, zone_data in self.zones.items():
#                 rospy.loginfo(f"Zone ID: {zone_id}, Data: {zone_data}")
#             # Plot the zones
#             # self.plot_zones()
#         else:
#             rospy.logwarn(f"No zones found for map '{self.map_name}'.")

#         # Keep node alive and monitor base_link position
#         rate = rospy.Rate(15)  # 15 Hz
#         self.shutdown()
#         while not rospy.is_shutdown():
#             t1=time.time()
#             if self.base_link_position is not None:
#                 x, y = self.base_link_position

#                 # Check which zones the base_link position is in
#                 zones_in = self.check_zone(x, y)
#                 # Only publish if the zone list has changed
#                 if zones_in != self.last_zones:
#                     zones_merged = self.merge_zones(zones_in)
#                     # print(f'Zones in: {zones_in}')
#                     self.last_zones = zones_in
#                     rospy.loginfo(f"Publishing zones: {zones_merged}")
#                     self.zone_publisher.publish(str(zones_merged))  # Publish list as a comma-separated string
#             # print(time.time()-t1)
#             # rospy.loginfo(f"Time to check zones : {time.time()-t1} seconds")
#             rate.sleep()


#     def shutdown(self):
#         """
#         Cleanup when shutting down the node.
#         """
#         if self.client:
#             self.client.close()
#             rospy.loginfo("MongoDB connection closed.")


# if __name__ == '__main__':
#     try:
#         node = MapZoneReaderNode()
#         node.run()
#     except rospy.ROSInterruptException:
#         pass

