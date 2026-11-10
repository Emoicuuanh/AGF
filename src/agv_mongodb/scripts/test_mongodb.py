#!/usr/bin/env python

# python -m pip install pymongo
from mongodb import mongodb
import rospy
import actionlib
import os
import sys
from datetime import datetime
from std_stamped_msgs.msg import StringAction, StringGoal
from bson.json_util import dumps
import rospkg

def parse_opts():
    from optparse import OptionParser
    parser = OptionParser()
    parser.add_option("-f", "--file", dest="file_name", default="",
                    help="Save mission to json FILE", metavar="FILE")
    parser.add_option("-p", "--current_pose", dest="current_pose", default="",
                    help="Current position of Robot defined in beginning of Path guide")
    parser.add_option("-s", "--send_action",
                    action="store_true", dest="send_mission", default=False,
                    help="Send action goal to /mission_manager")

    (options, args) = parser.parse_args()
    print("Options:\n:{}".format(options))
    print("Args:\n:{}".format(args))
    return (options, args)

if __name__ == "__main__":
    (options, args) = parse_opts()

    db = mongodb("mongodb://localhost:27017/")
    mission_dict = db.printJson(db.getQueueMission(options.current_pose))
    db.updateQueueMission("2/7", "Doing")

    json_string = dumps(mission_dict, indent=2, sort_keys=True)
    # Save file option
    if options.file_name != "":
        file_to_save = os.path.join(rospkg.RosPack().get_path('mission_manager'), 'mission_list', options.file_name + '.json')
        print('File will be saved: {}'.format(file_to_save))

        with open(file_to_save, 'w') as file:
            file.write(json_string)

    # Send mission_manager action goal
    if options.send_mission:
        rospy.init_node("mission_manager_client")
        action_client = actionlib.SimpleActionClient('mission_manager_server', StringAction)
        action_client.wait_for_server(timeout=rospy.Duration(5))
        rospy.loginfo("Sending goal to /mission_manager")
        goal = StringGoal(data=json_string)
        action_client.send_goal(goal)
        rospy.sleep(3)
