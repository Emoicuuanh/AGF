#!/usr/bin/env python
import rospy
import json
import os
import subprocess
import yaml
from std_msgs.msg import String
from pymongo import MongoClient
from bson.objectid import ObjectId

class ConfigIPSubscriber:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('config_ip_subscriber', anonymous=True)
        rospy.loginfo("Starting config_ip_subscriber node")

        # MongoDB configuration
        self.mongo_uri = "mongodb://localhost:27017"
        self.db_name = "admin"
        self.collection_name = "amrs"
        self.mongo_client = MongoClient(self.mongo_uri)
        self.db = self.mongo_client[self.db_name]
        self.collection = self.db[self.collection_name]

        # File paths (adjust as needed)
        self.dashboard_config_path = "/home/mkac/parse_server_ws/dashboard-config.json"  # Path to dashboard-config.json
        self.docker_compose_path = "/home/mkac/parse_server_ws/docker-compose.yml"       # Path to docker-compose.yml

        # Subscribe to /config_IP topic
        self.subscriber = rospy.Subscriber(
            '/config_IP', String, self.callback, queue_size=10
        )

    def callback(self, msg):
        """Callback for /config_IP topic messages."""
        try:
            new_ip = msg.data
            rospy.loginfo(f"Received IP: {new_ip}")

            # Update MongoDB
            self.update_mongodb(new_ip)
            # Update dashboard-config.json
            self.update_dashboard_config(new_ip)
            # Update docker-compose.yml and restart services
            self.update_docker_compose(new_ip)

        except Exception as e:
            rospy.logerr(f"Error processing message: {e}")

    def update_mongodb(self, ip):
        """Update the IP in the amrs collection."""
        try:
            # Update document where name_amr is 'AMR01'
            result = self.collection.update_one(
                {"name_amr": "AMR01"},
                {"$set": {"ip": ip}},
                upsert=False
            )
            if result.matched_count > 0:
                rospy.loginfo(f"Updated IP to {ip} for AMR01 in MongoDB")
            else:
                rospy.logwarn("No document found with name_amr: AMR01")
        except Exception as e:
            rospy.logerr(f"Error updating MongoDB: {e}")

    def update_dashboard_config(self, ip):
        """Update the serverURL in dashboard-config.json."""
        try:
            if not os.path.exists(self.dashboard_config_path):
                rospy.logerr(f"Dashboard config file not found: {self.dashboard_config_path}")
                return

            with open(self.dashboard_config_path, 'r') as file:
                config = json.load(file)

            # Update the serverURL in the apps array
            for app in config.get('apps', []):
                if app.get('appName') == 'MKEM_APP':
                    app['serverURL'] = f"http://{ip}:1337/parse"
                    rospy.loginfo(f"Updated serverURL to {app['serverURL']} in dashboard-config.json")

            # Write the updated config back to the file
            with open(self.dashboard_config_path, 'w') as file:
                json.dump(config, file, indent=4)

            rospy.loginfo("Successfully updated dashboard-config.json")
        except Exception as e:
            rospy.logerr(f"Error updating dashboard-config.json: {e}")

    def update_docker_compose(self, ip):
        """Update PARSE_SERVER_PUBLIC_SERVER_URL in docker-compose.yml and restart services."""
        try:
            if not os.path.exists(self.docker_compose_path):
                rospy.logerr(f"Docker Compose file not found: {self.docker_compose_path}")
                return

            # Read the docker-compose.yml file
            with open(self.docker_compose_path, 'r') as file:
                compose_config = yaml.safe_load(file)

            # Update PARSE_SERVER_PUBLIC_SERVER_URL in the parse service
            parse_service = compose_config.get('services', {}).get('parse', {})
            parse_env = parse_service.get('environment', [])
            new_url = f"http://{ip}:1337/parse"

            # Update the environment variable
            updated = False
            for i, env_var in enumerate(parse_env):
                if env_var.startswith('PARSE_SERVER_PUBLIC_SERVER_URL='):
                    parse_env[i] = f"PARSE_SERVER_PUBLIC_SERVER_URL={new_url}"
                    updated = True
                    break
            if not updated:
                parse_env.append(f"PARSE_SERVER_PUBLIC_SERVER_URL={new_url}")

            # Write the updated config back to the file
            with open(self.docker_compose_path, 'w') as file:
                yaml.safe_dump(compose_config, file, default_flow_style=False)

            rospy.loginfo(f"Updated PARSE_SERVER_PUBLIC_SERVER_URL to {new_url} in docker-compose.yml")

            # Restart Docker Compose services
            self.restart_docker_compose()

        except Exception as e:
            rospy.logerr(f"Error updating docker-compose.yml: {e}")

    def restart_docker_compose(self):
        """Restart Docker Compose services."""
        try:
            # Change working directory to /home/mkac/parse_server_ws
            subprocess.run(
                ["sudo", "docker", "compose", "-f", self.docker_compose_path, "up", "-d"],
                # cwd="/home/mkac/parse_server_ws",  # Specify working directory
                check=True,
                capture_output=True,
                text=True
            )
            rospy.loginfo("Successfully restarted Docker Compose services")
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Error restarting Docker Compose: {e.stderr}")
        except Exception as e:
            rospy.logerr(f"Error restarting Docker Compose: {e}")

    def shutdown(self):
        """Cleanup on node shutdown."""
        rospy.loginfo("Shutting down config_ip_subscriber node")
        self.mongo_client.close()

    def run(self):
        """Keep the node running."""
        rospy.on_shutdown(self.shutdown)
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ConfigIPSubscriber()
        node.run()
    except rospy.ROSInterruptException:
        pass







# #!/usr/bin/env python
# import rospy
# import json
# import os
# import subprocess
# import yaml
# from std_stamped_msgs.msg import StringStamped
# from pymongo import MongoClient
# from bson.objectid import ObjectId

# class ConfigIPSubscriber:
#     def __init__(self):
#         # Initialize ROS node
#         rospy.init_node('config_ip_subscriber', anonymous=True)
#         rospy.loginfo("Starting config_ip_subscriber node")

#         # MongoDB configuration
#         self.mongo_uri = "mongodb://localhost:27017"
#         self.db_name = "admin"
#         self.collection_name = "amrs"
#         self.mongo_client = MongoClient(self.mongo_uri)
#         self.db = self.mongo_client[self.db_name]
#         self.collection = self.db[self.collection_name]

#         # File paths (adjust as needed)
#         self.dashboard_config_path = "/home/mkac/parse_server_ws/dashboard-config.json"  # Path to dashboard-config.json
#         self.docker_compose_path = "/home/mkac/parse_server_ws/docker-compose.yml"       # Path to docker-compose.yml

#         # Subscribe to /config_IP topic
#         self.subscriber = rospy.Subscriber(
#             '/config_IP', StringStamped, self.callback, queue_size=10
#         )

#     def callback(self, msg):
#         """Callback for /config_IP topic messages."""
#         try:
#             new_ip = msg.data
#             rospy.loginfo(f"Received IP: {new_ip}")

#             # Update MongoDB
#             self.update_mongodb(new_ip)
#             # Update dashboard-config.json
#             self.update_dashboard_config(new_ip)
#             # Update docker-compose.yml and restart services
#             self.update_docker_compose(new_ip)

#         except Exception as e:
#             rospy.logerr(f"Error processing message: {e}")

#     def update_mongodb(self, ip):
#         """Update the IP in the amrs collection."""
#         try:
#             # Update document where name_amr is 'AMR01'
#             result = self.collection.update_one(
#                 {"name_amr": "AMR01"},
#                 {"$set": {"ip": ip}},
#                 upsert=False
#             )
#             if result.matched_count > 0:
#                 rospy.loginfo(f"Updated IP to {ip} for AMR01 in MongoDB")
#             else:
#                 rospy.logwarn("No document found with name_amr: AMR01")
#         except Exception as e:
#             rospy.logerr(f"Error updating MongoDB: {e}")

#     def update_dashboard_config(self, ip):
#         """Update the serverURL in dashboard-config.json."""
#         try:
#             if not os.path.exists(self.dashboard_config_path):
#                 rospy.logerr(f"Dashboard config file not found: {self.dashboard_config_path}")
#                 return

#             with open(self.dashboard_config_path, 'r') as file:
#                 config = json.load(file)

#             # Update the serverURL in the apps array
#             for app in config.get('apps', []):
#                 if app.get('appName') == 'MKEM_APP':
#                     app['serverURL'] = f"http://{ip}:1337/parse"
#                     rospy.loginfo(f"Updated serverURL to {app['serverURL']} in dashboard-config.json")

#             # Write the updated config back to the file
#             with open(self.dashboard_config_path, 'w') as file:
#                 json.dump(config, file, indent=4)

#             rospy.loginfo("Successfully updated dashboard-config.json")
#         except Exception as e:
#             rospy.logerr(f"Error updating dashboard-config.json: {e}")

#     def update_docker_compose(self, ip):
#         """Update PARSE_SERVER_PUBLIC_SERVER_URL in docker-compose.yml and restart services."""
#         try:
#             if not os.path.exists(self.docker_compose_path):
#                 rospy.logerr(f"Docker Compose file not found: {self.docker_compose_path}")
#                 return

#             # Read the docker-compose.yml file
#             with open(self.docker_compose_path, 'r') as file:
#                 compose_config = yaml.safe_load(file)

#             # Update PARSE_SERVER_PUBLIC_SERVER_URL in the parse service
#             parse_service = compose_config.get('services', {}).get('parse', {})
#             parse_env = parse_service.get('environment', [])
#             new_url = f"http://{ip}:1337/parse"

#             # Update the environment variable
#             updated = False
#             for i, env_var in enumerate(parse_env):
#                 if env_var.startswith('PARSE_SERVER_PUBLIC_SERVER_URL='):
#                     parse_env[i] = f"PARSE_SERVER_PUBLIC_SERVER_URL={new_url}"
#                     updated = True
#                     break
#             if not updated:
#                 parse_env.append(f"PARSE_SERVER_PUBLIC_SERVER_URL={new_url}")

#             # Write the updated config back to the file
#             with open(self.docker_compose_path, 'w') as file:
#                 yaml.safe_dump(compose_config, file, default_flow_style=False)

#             rospy.loginfo(f"Updated PARSE_SERVER_PUBLIC_SERVER_URL to {new_url} in docker-compose.yml")

#             # Restart Docker Compose services
#             self.restart_docker_compose()

#         except Exception as e:
#             rospy.logerr(f"Error updating docker-compose.yml: {e}")

#     def restart_docker_compose(self):
#         """Restart Docker Compose services."""
#         try:
#             # Run docker-compose up -d in the directory containing docker-compose.yml
#             subprocess.run(
#                 ["docker-compose", "-f", self.docker_compose_path, "up", "-d"],
#                 check=True,
#                 capture_output=True,
#                 text=True
#             )
#             rospy.loginfo("Successfully restarted Docker Compose services")
#         except subprocess.CalledProcessError as e:
#             rospy.logerr(f"Error restarting Docker Compose: {e.stderr}")
#         except Exception as e:
#             rospy.logerr(f"Error restarting Docker Compose: {e}")

#     def shutdown(self):
#         """Cleanup on node shutdown."""
#         rospy.loginfo("Shutting down config_ip_subscriber node")
#         self.mongo_client.close()

#     def run(self):
#         """Keep the node running."""
#         rospy.on_shutdown(self.shutdown)
#         rospy.spin()

# if __name__ == '__main__':
#     try:
#         node = ConfigIPSubscriber()
#         node.run()
#     except rospy.ROSInterruptException:
#         pass