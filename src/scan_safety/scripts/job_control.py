#!/usr/bin/env python
import sys, os
import rospy, rospkg
import dynamic_reconfigure.client
import json

def callback(config):
    print("callback")
    print(config)
    pass

# Load json from file
path = rospkg.RosPack().get_path("mission_manager")
file_db = os.path.join(path, "db/mission_db.json")
file_ms = os.path.join(path, "mission_list/footprint.json")
with open(file_db) as js:
    data = json.load(js)

with open(file_ms) as ms:
    mission = json.load(ms)
footprint = mission["actions"][0]
params = (footprint["params"])
print(params)
print(type(params))

def set_footprint(self, **kwargs):
    print(len(kwargs))
    for key, value in kwargs.items():
        if(key == "footprint_name"):
            print("%s: %s"%(key, value))


if __name__ == "__main__":
    rospy.init_node("scan_safety_dynamic_client", log_level=rospy.DEBUG)
    try:
        client = dynamic_reconfigure.client.Client("scan_safety", timeout=5, config_callback=None)
        rospy.loginfo("client connect successful!")
    except rospy.ROSException as e:
        rospy.logerr(e)
        rospy.logerr("Client cannot connect to server")

    r = rospy.Rate(0.1)
    print ("ad")
    # while not rospy.is_shutdown():
    # client.update_configuration({"current_job": 3})
    # client.update_configuration({"footprint":'[[0.6, 0.5], [-0.6, 0.5], [-0.6, -0.5], [0.6, -0.5], [0.6, 0.5]]'})
    # r.sleep()

    set_footprint("a", **params)