#!/usr/bin/env python
#!
import rospy
import requests
import yaml

from datetime import datetime
from std_msgs.msg import Bool
from std_msgs.msg import Header, Int32MultiArray
from agf_msgs.msg import Int8Array, SafetyStt
import json
from std_stamped_msgs.msg import StringStamped
class SensorDataClient:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('sensor_data_client', anonymous=True)

        # Load YAML configuration
        yaml_file = rospy.get_param('~config_file', '/home/mkac/Desktop/config.yaml')
        try:
            with open(yaml_file, 'r') as file:
                self.config = yaml.safe_load(file)
        except Exception as e:
            rospy.logerr(f"ERROR READ YAML: {str(e)}")
            raise

        # Get server parameters
        try:
            self.ip = self.config['server']['server_ip']
            self.port = self.config['server']['server_port']
            self.server_url = f"http://{self.ip}:{self.port}/Safety"
            self.area=self.config['server']['area']
            self.agf_name=self.config['server']['name']
        except KeyError as e:
            rospy.logerr(f"Dont have {str(e)} in file YAML")
            raise

        rospy.loginfo(f"server: {self.server_url}")
        self.manual_sw =0
        self.door_1=1 # server 0 = mo
        self.door_2=1
        self.door_3=1
        self.door_4=1
        self.robot_status=0
        self.job_status=[]
        self.stop_api=0
        self.safety_status=0
        self.status_rb=None
        # Initialize sensor data and timestamps
        self.sensor_data = {sensor: '0' for sensor in self.config['sensors']}
        self.latest_timestamps = {sensor: None for sensor in self.config['sensors']}
        self.sensorPub = rospy.Publisher(
            "/safety_canon_door_sensors", Int32MultiArray, queue_size=1
        )
        rospy.Subscriber('/robot_status', StringStamped, self.status_robot_callback)
        # Initialize subscribers
        self.subscribers = {}
        for sensor, info in self.config['sensors'].items():
            topic = info['topic']
            self.subscribers[sensor] = rospy.Subscriber(
                topic, SafetyStt, self.sensor_callback, callback_args=sensor
            )

        # Initialize publisher for State topic
        self.state_pub = rospy.Publisher('safety_camera_canon_api', Bool, queue_size=1)

        # Frequency for sending data
        self.rate = rospy.Rate(2)  # 1 Hz

        # Server status
        self.server_alive = False
        # rospy.spin()
        self.check_server()
    def status_robot_callback(self,msg):
        self.status_rb = json.loads(msg.data)
    # def standard_io_callback(self, msg):
    #     try:
    #         # rospy.loginfo(f"Standard IO: {msg.data} (Type: {type(msg.data)})")

    #         data_dict = json.loads(msg.data)

    #         self.manual_sw = data_dict["emg_button"]
    #         # self.stop_button = data_dict["stop_1_button"]
    #         # self.handbrake_status = data_dict["parking_break_sensor"]
    #         # self.confirm_homing = data_dict.get("confirm_homing", None)

    #     except json.JSONDecodeError as e:
    #         rospy.logerr(f"JSON decode error: {e}")
    #     except KeyError as e:
    #         rospy.logerr(f"Missing key in data: {e}")
    def sensor_callback(self, msg, sensor):
        # print(msg.Range_1)

        if msg.Current_Job==99:
            self.sensor_data[sensor] = '1'
        elif msg.Range_1:
            self.sensor_data[sensor] = '1'
        else:
            self.sensor_data[sensor] = '0'
        # self.latest_timestamps[sensor] = msg.header.stamp

    def check_server(self):
        try:
            #http://10.0.112.18:5000/Safety?Sensor="AGF_Sensor1=0;AGF_Sensor2=0;AGF_Sensor3=0;AGF_Sensor4=0;Time='2025-07-02 14:24:55.255';Mode="Manual";State" // Auto
            test_data = {sensor: '0' for sensor in self.config['sensors']}
            test_data['Time'] = f"'{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}'"
            sensor_string = ';'.join([f"{key}={value}" for key, value in test_data.items()])
            sensor_json= json.dumps(test_data)
            mode_string="MANUAL"
            if self.status_rb is not None:
                mode_string=self.status_rb["mode"]
                if self.status_rb["status"] == "WAITING" or self.status_rb["status"]=="ERROR":
                    self.robot_status = 0
                else:
                    self.robot_status = 1

            url = f"{self.server_url}?Sensor=\"{sensor_string};Mode=\'{mode_string}\';StateAGF={self.robot_status};AGF=\"{self.agf_name}\""
            print(url)
            # Send GET request
            response = requests.get(url, timeout=3)
            try:
                response_data = response.json()
                # print(f"response:{response_data}")
                # Check response
                self.server_alive = response.status_code == 200 and response_data.get('Result') == 'OK'
                if self.server_alive:
                    rospy.loginfo("Server connect: %s", self.server_url)
                else:
                    rospy.logwarn("Server wrong data: %s response:%s", response_data.get('Result', 'No Result'),response_data)
            except:
                print("Error")

        except requests.RequestException as e:
            self.server_alive = False
            self.state_pub.publish(True)
            rospy.logerr("Error checking server: %s", str(e))

    def check_safety_state(self, area):
        """Send GET request for a specific area and publish State to topic."""
        try:
            url = f"{self.server_url}?Area=\'{area}\'"
            print(url)
            response = requests.get(url, timeout=3)
            response_data = response.json()

            if response.status_code == 200 and 'State' in response_data:
                state = int(response_data['State'])
                self.stop_api=1-state
                print(response_data)
                if 'Device' in response_data:
                    self.door_1=int(response_data['Device']['door_1'])
                    self.door_2=int(response_data['Device']['door_2'])
                    self.door_3=int(response_data['Device']['door_3'])
                    self.door_4=int(response_data['Device']['door_4'])
                    resultArray = Int32MultiArray()

                    resultArray.data = [self.door_1, self.door_2, self.door_3, self.door_4]
                    self.sensorPub.publish(resultArray)
                # print(state)
                if state in [0, 1]: # 1 la dung 0 la ko dung
                    state=1-state

                    # state=0
                    # state=0
                    # Publish State as Bool message (0 -> False, 1 -> True) #
                    print(state)
                    self.state_pub.publish(Bool(data=bool(state)))
                    rospy.loginfo(f"Published State={state} for Area={area}")
                else:

                    rospy.logwarn(f"Invalid State value: {state} for Area={area} response:{response_data}")
            else:
                rospy.logerr(f"Failed to get valid response for Area={area}: {response_data}")
        except requests.RequestException as e:
            rospy.logerr(f"Error sending GET request for Area={area}: {str(e)}")

    def run(self):
        while not rospy.is_shutdown():
            if not self.server_alive:
                rospy.logwarn("Server not responding...")
                self.check_server()
                self.rate.sleep()
                continue

            self.check_safety_state(self.area)
            # Send sensor data
            sensor_data = self.sensor_data.copy()
            sensor_data['Time'] = f"'{datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]}'"
            sensor_string = ';'.join([f"{key}={value}" for key, value in sensor_data.items()])
            self.safety_status=0
            for key,value in sensor_data.items():
                # print(type(value))
                if(value=='0'):
                    # print("SAFETy")
                    self.safety_status=1
            # print(self.safety_status)
            # sensor_json= json.dumps(sensor_string)

            mode_string="MANUAL"
            if self.status_rb is not None:
                mode_string=self.status_rb["mode"]
                if self.status_rb["status"] == "WAITING" or self.status_rb["status"]=="ERROR" or mode_string=="MANUAL" or self.safety_status or self.stop_api:
                    self.robot_status = 0
                else:
                    self.robot_status = 1

            # url = f"{self.server_url}?Sensor=\"{sensor_string};Mode=\'{mode_string}\';StateAGF={self.robot_status}\""
            url = f"{self.server_url}?Sensor=\"{sensor_string};Mode=\'{mode_string}\';StateAGF={self.robot_status};AGF=\"{self.agf_name}\""
            # print(url)
            try:
                response = requests.get(url, timeout=5)
                try:
                    response_data = response.json()
                    # print(response_data)
                    if response.status_code == 200 and response_data.get('Result') == 'OK':
                        # rospy.loginfo("Successfully sent: %s", sensor_string)
                        done=True
                    else:
                        rospy.logerr("Communication failed: %s", response_data.get('Result'))
                except:
                    print("Error")

            except requests.RequestException as e:
                rospy.logerr("Communication error: %s", str(e))
                self.server_alive = False

            # Check safety state for a specific area (e.g., 'Machine 1')


            self.rate.sleep()

if __name__ == '__main__':
    try:
        client = SensorDataClient()
        client.run()
    except rospy.ROSInterruptException:
        pass