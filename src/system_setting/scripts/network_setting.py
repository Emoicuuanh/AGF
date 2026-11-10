#!/usr/bin/python

import subprocess

from numpy import row_stack
import rospy
from std_msgs.msg import String
import json


class NetworkSettingNode():
    def __init__(self):
        rospy.init_node('network_setting')
        rospy.Subscriber("/network_setting/refresh",
                         String, self.refreshcallback)
        rospy.Subscriber("/network_setting/connect",
                         String, self.connectcallback)
        self.curWifiPub = rospy.Publisher(
            "/network_setting/current_wifi", String, queue_size=10)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
        pass

    def connectcallback(self, msg):
        print("receive connect msg")
        myjson = json.loads(msg.data)
        try:
            subprocess.check_output(["nmcli", "dev", "wifi", "rescan"])
        except Exception as e:
            print(e)
        # print("refreshed wifi")
        self.setWifi(myjson['ssid'], myjson['password'])
        pass

    def refreshcallback(self, msg):
        try:
            subprocess.check_output(["nmcli", "dev", "wifi", "rescan"])
        except Exception as e:
            print(e)
        _current_wifi = self.getCurrentWifi()
        pubMsg = String()
        pubMsg.data = json.dumps(_current_wifi)
        self.curWifiPub.publish(pubMsg)
        # print(_current_wifi)
        pass

    def getCurrentWifi(self):
        results = subprocess.check_output(
            ["nmcli", "dev", "wifi", "list"])

        ls = results.split("\n")
        ssid_pos = ls[0].find('SSID')
        mode_pos = ls[0].find('MODE')
        signal_pos = ls[0].find('SIGNAL')
        bar_pos = ls[0].find('BARS')
        # print(ls)
        ssid = []
        cur_wifi = {}
        cur_wifi['ssid'] = ''
        cur_wifi['signal'] = ''
        for i in range(1, len(ls)):
            if ls[i] != '':
                # str = ls[i][ssid_pos:mode_pos].rstrip().lstrip()
                str_ssid = ls[i][ssid_pos:mode_pos]
                str_signal = ls[i][signal_pos:bar_pos]
                str_ssid = str_ssid.rstrip().lstrip()
                str_signal = str_signal.rstrip().lstrip()
                # print(str)
                if str_ssid not in ssid and str_ssid != '--':
                    ssid.append(str_ssid)
                if ls[i][0] == '*':
                    cur_wifi['ssid'] = str_ssid
                    cur_wifi['signal'] = str_signal

        return cur_wifi
        pass

    def setWifi(self, ssid, password):
        try:
            results = subprocess.check_output(
                ["nmcli", "dev", "wifi", "connect", ssid, "password", password])
            print(results)
            if results.find('successfully') != -1:
                cur_wifi = self.getCurrentWifi()
                self.curWifiPub.publish(json.dumps(cur_wifi))
            else:
                print("connect wifi failed")
        except Exception as e:
            print(e)
            pass

        pass


if __name__ == '__main__':
    NetworkSettingNode()
