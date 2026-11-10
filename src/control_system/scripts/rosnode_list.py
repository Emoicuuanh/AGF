#!/usr/bin/env python

import os
import sys
import json
import socket
import rospy
import datetime
from std_stamped_msgs.msg import StringStamped

from rosnode import (
    ROSNodeIOException,
    rosnode_ping,
    rosgraph,
    ID,
    cleanup_master_blacklist,
    get_node_names,
    kill_nodes,
)

check_node = "mav_software"


def rosnode_cleanup():
    """
    This function was copied from rosnode pkg
    This is a semi-hidden routine for cleaning up stale node
    registration information on the ROS Master. The intent is to
    remove this method once Master TTLs are properly implemented.
    """
    pinged, unpinged = rosnode_ping_all()
    if unpinged:
        master = rosgraph.Master(ID)
        print("Unable to contact the following nodes:")
        print("\n".join(" * %s" % n for n in unpinged))
        cleanup_master_blacklist(master, unpinged)


def rosnode_ping_all(verbose=False, skip_cache=False, grep=""):
    """
    Ping all running nodes
    @return [str], [str]: pinged nodes, un-pingable nodes
    @raise ROSNodeIOException: if unable to communicate with master
    """
    master = rosgraph.Master(ID)
    try:
        state = master.getSystemState()
    except socket.error:
        raise ROSNodeIOException("Unable to communicate with master!")

    nodes = []
    for s in state:
        for t, l in s:
            nodes.extend(l)
    nodes = list(set(nodes))  # uniq
    if verbose:
        print(
            "Will ping the following nodes: \n"
            + "".join([" * %s\n" % n for n in nodes])
        )
    pinged = []
    unpinged = []
    for node in nodes:
        if grep == "":
            if rosnode_ping(
                node, max_count=1, verbose=verbose, skip_cache=skip_cache
            ):
                pinged.append(node)
            else:
                unpinged.append(node)
        elif grep in node:
            if rosnode_ping(
                node, max_count=1, verbose=verbose, skip_cache=skip_cache
            ):
                pinged.append(node)
            else:
                unpinged.append(node)
    return pinged, unpinged


# TODO: use function in rosnode pkg
def get_node_list(grep=""):
    """This function take 1 sec"""
    cmd = "rosnode list"
    if grep != "":
        cmd += "| grep {}".format(grep)

    nodes = os.popen(cmd).readlines()
    for i in range(len(nodes)):
        nodes[i] = nodes[i].replace("\n", "")
    return nodes


class RosNodeList(object):
    def __init__(self, *args, **kwargs):
        self.rosnode_list_pub = rospy.Publisher(
            "/rosnode_list", StringStamped, queue_size=5
        )
        rospy.Subscriber(
            "/kill_others_node", StringStamped, self.kill_others_node_cb
        )
        self.loop()

    def kill_others_node_cb(self, msg):
        nodes = get_node_names()
        for n in nodes:
            if check_node in n:
                if n not in msg.data and msg.data not in n:
                    os.system("rosnode kill {}".format(n))
                    rospy.loginfo("Killed node: {}".format(n))

    def loop(self):
        r = rospy.Rate(1)
        last_node_list_len = 0
        while not rospy.is_shutdown():
            # Kill all node cannot be ping
            pinged, unpinged = rosnode_ping_all(False, False, check_node)
            if len(unpinged) > 0:
                for n in unpinged:
                    if check_node in n:
                        master = rosgraph.Master(ID)
                        print("Unable to contact the following nodes:")
                        print("\n".join(" * %s" % n for n in unpinged))
                        cleanup_master_blacklist(master, unpinged)

            self.rosnode_list_pub.publish(
                StringStamped(stamp=rospy.Time.now(), data=json.dumps(pinged))
            )
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("rosnode_list")
    rospy.loginfo("Init node: " + rospy.get_name())
    RosNodeList()
