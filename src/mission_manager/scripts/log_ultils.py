import rospy

def mylogdebug(msg):
    rospy.logdebug("[%s] %s"%(rospy.get_name(), msg))

def myloginfo(msg):
    rospy.loginfo("[%s] %s"%(rospy.get_name(), msg))

def mylogwarn(msg):
    rospy.logwarn("[%s] %s"%(rospy.get_name(), msg))

def mylogerr(msg):
    rospy.logerr("[%s] %s"%(rospy.get_name(), msg))