#!/usr/bin/env python

# ROS imports
import roslib
import rospy
import time
# Msgs imports
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import std_msgs

# Service imports
from std_srvs.srv import Empty, EmptyResponse

# Python imports
import numpy as np
from numpy.linalg import *
import math as math1

#Custom imports
#from eulerangles import *
import sys
from math import floor, exp, sqrt, atan2, sin, pi, asin, cos
from geometry_msgs.msg import Point, Vector3, Quaternion
from math_utils import *

from poseArmController import *


#planner messages
from ltl3_NTUA.msg import *



enable = False
header = -1

def enableSrv(req):
    global enable
    enable = True
    rospy.loginfo('%s Enabled', 'Gesture')
    return EmptyResponse()


def disableSrv(req):
    global enable
    enable = False
    rospy.loginfo('%s Disabled', 'Gesture')
    return EmptyResponse()

flag_conf = True

def updateNextMove(data):
    Action = data.type
    global header, flag_conf, enable

    if data.header != header:
       flag_conf = True
       header = data.header 
    
    if data.type == 'oyobsgrasp' and flag_conf == True:
       print "recognizing gesture"
       time.sleep(10)
       pub_observingdone.publish(1)
       print "I published the gesture was recognized"
       enable = True
       print "grasping object"
       time.sleep(5)
       enable = False
       flag_conf = False
       confMsg = confirmation()
       confMsg.name = 'oyobsgrasp'
       confMsg.done = 1
       confMsg.confheader = header
       print confMsg.done
       pub_confirm.publish(confMsg)
       print "I published the confirmation of oyobsgrasp"


rospy.Subscriber("/next_move_OY", activity, updateNextMove)
pub_confirm = rospy.Publisher('activity_done_OY', confirmation)

pub_observingdone = rospy.Publisher('/OY/observation_done', std_msgs.msg.Int8)


counter = 0;
if __name__ == '__main__':
    try:
        rospy.init_node('observe_node')
        #Arm Controller Object   
        rospy.sleep(1.0/300.0)
        rospy.spin()
    except rospy.ROSInterruptException: pass
