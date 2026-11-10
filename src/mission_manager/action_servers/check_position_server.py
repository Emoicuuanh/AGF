#! /usr/bin/env python

import rospy

import actionlib

from mission_manager.msg import CheckPositionStatusAction, CheckPositionStatusFeedback, CheckPositionStatusGoal, CheckPositionStatusResult

class CheckPositionStatus(object):
    # create messages that are used to publish feedback/result
    _feedback = CheckPositionStatusFeedback()
    _result = CheckPositionStatusResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, CheckPositionStatusAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        # helper variables
        r = rospy.Rate(1)
        success = True
        print(goal)
        self._feedback.rest_time = goal.timeout

        # publish info to the console for the user
        rospy.loginfo('%s: Executing' % (self._action_name))

        # start executing the action
        for i in range(10):
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break
            self._feedback.rest_time = goal.timeout*(10-i)/10
            rospy.logdebug("[%s] publish feedback: %0.2f"%(rospy.get_name(), self._feedback.rest_time))
            self._as.publish_feedback(self._feedback)
            r.sleep()

        if success:
            self._result.result = True
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('check_position_status', log_level=rospy.DEBUG)
    server = CheckPositionStatus(rospy.get_name())
    rospy.spin()
