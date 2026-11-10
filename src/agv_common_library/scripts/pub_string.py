#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import json
import rospy
from std_stamped_msgs.msg import StringStamped
from rosgraph_msgs.msg import Log


class PubString:
    def __init__(
        self,
        topic_name,
        string_value="",
        once=False,
        once_no_latch=False,
        rate=10.0,
    ):
        self.topic_name = topic_name
        if string_value != "":
            self.data_value = string_value
        try:
            self.data_value = json.dumps(json.loads(string_value), indent=2)
            print("Dict value:\n{}".format(self.data_value))
        except:
            pass
        self.pub_once = once
        self.pub_once_no_latch = once_no_latch
        self.pub_rate = rate
        self.init_varialble()
        self.init_ros()
        self.poll()

    def init_varialble(self):
        self.allow_publish = False
        self.msg_pub = StringStamped()

    def init_ros(self):
        rospy.init_node("pub_string_stamped_one")
        rospy.loginfo("Init node: " + rospy.get_name())

        self.pub = rospy.Publisher(
            self.topic_name, StringStamped, queue_size=10, latch=True
        )
        rospy.Subscriber("/rosout", Log, self.rosout_cb)

    def rosout_cb(self, msg):
        # rospy.loginfo("rosout received")
        self.allow_publish = True

    def poll(self):
        rate = rospy.Rate(self.pub_rate)
        while not rospy.is_shutdown():
            if self.allow_publish:
                self.msg_pub.stamp = rospy.Time.now()
                self.msg_pub.data = self.data_value
                self.pub.publish(self.msg_pub)
                if self.pub_once_no_latch:
                    rospy.loginfo("Publish successful!")
                    break
                if self.pub_once:
                    rospy.loginfo("Publish successful!")
                    rospy.sleep(3)
                    break
            rate.sleep()


def parse_opts():
    from optparse import OptionParser

    parser = OptionParser()
    parser.add_option(
        "-t",
        "--topic_name",
        dest="topic_name",
        default="",
        help="Input topic name",
    )
    parser.add_option(
        "-s",
        "--string_value",
        dest="string_value",
        default="",
        help='If data is dict, in put with format \'{"key1": "value1"}\'',
    )
    parser.add_option(
        "-r",
        "--rate",
        dest="rate",
        default=10,
        type=float,
        help="Publish rate (Float)",
    )
    parser.add_option(
        "-1",
        "--once",
        action="store_true",
        dest="once",
        default=False,
        help="Publish once time then latch 3 seconds",
    )
    parser.add_option(
        "-n",
        "--once_no_latch",
        action="store_true",
        dest="once_no_latch",
        default=False,
        help="Publish once time then quit",
    )

    (options, args) = parser.parse_args()
    print("Options:\n{}".format(options))
    print("Args:\n{}".format(args))
    return (options, args)


def main():
    (options, args) = parse_opts()
    PubString(**vars(options))


if __name__ == "__main__":
    main()
