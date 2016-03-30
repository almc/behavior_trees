# -*- encoding: UTF-8 -*- 

'''Move To: Small example to make Nao Move To an Objective'''

import math
import sys
from naoqi import ALProxy

import roslib
import numpy
import Queue
roslib.load_manifest('ltl3_NTUA')
import rospy
from ltl3_NTUA.msg import pose, activity, confirmation, knowledge

goal_x = 99.0;
goal_y = 99.0;
goal_t = 3.0;

def StiffnessOn(proxy):
    pNames = "Body"
    pStiffnessLists = 1.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)


def StiffnessOff(proxy):
    pNames = "Body"
    pStiffnessLists = 0.0
    pTimeLists = 1.0
    proxy.stiffnessInterpolation(pNames, pStiffnessLists, pTimeLists)

def new_waypoint_callback(data):
    global now
    now = rospy.get_rostime()
    global goal_x;
    global goal_y;
    global goal_t;
    goal_x = 0.01*data.x;
    goal_y = 0.01*data.y;
    goal_t = -data.psi;
    print "goal", goal_x, goal_y, goal_t;


def main(robotIP):
    try:
        motionProxy = ALProxy("ALMotion", robotIP, 9559)
    except Exception, e:
        print "Could not create proxy to ALMotion"
        print "Error was: ", e
        print "Try running the script again"


    rospy.Subscriber('next_move_PY_relative', activity, new_waypoint_callback)
    
    StiffnessOn(motionProxy)
    motionProxy.setWalkArmsEnabled(True, True)
    motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION", True]])
    
    #~ motionProxy.setWalkArmsEnabled(False, False)
    #~ motionProxy.setMotionConfig([["ENABLE_FOOT_CONTACT_PROTECTION",False]])
    # motionProxy.waitUntilMoveIsFinished()
    global goal_x, goal_y, goal_t;

    while not rospy.is_shutdown():
        if (rospy.get_rostime() - now < rospy.Duration(0.2) ):
            Theta = numpy.arctan2(goal_y, goal_x)
            if ( abs(goal_x) < 0.20 and abs(goal_y) < 0.20  and abs(goal_t) < 0.4):
                print "************************************************************Location reached, waiting for the next waypoint"
                continue
            if ( abs(goal_x) < 0.20 and abs(goal_y) < 0.20 ):
                print '************************************************************case 1: turning towards final orientation'
                motionProxy.post.moveTo(0.0, 0.0, goal_t,
                    [ ["MaxStepX", 0.02],         # step of 2 cm in front
                      ["MaxStepY", 0.16],         # default value
                      ["MaxStepTheta", 0.2],      # default value
                      ["MaxStepFrequency", 0.0],  # low frequency
                      ["StepHeight", 0.01],       # step height of 1 cm
                      ["TorsoWx", 0.0],           # default value
                      ["TorsoWy", 0.1] ])         # torso bend 0.1 rad in front
            elif ( abs(goal_y) < 0.30 ):
                print '************************************************************case 2: moving forward'
                motionProxy.post.moveTo(goal_x, goal_y, Theta,
                    [ ["MaxStepX", 0.02],         # step of 2 cm in front
                      ["MaxStepY", 0.16],         # default value
                      ["MaxStepTheta", 0.2],      # default value
                      ["MaxStepFrequency", 0.0],  # low frequency
                      ["StepHeight", 0.01],       # step height of 1 cm
                      ["TorsoWx", 0.0],           # default value
                      ["TorsoWy", 0.1] ])         # torso bend 0.1 rad in front
            else:
                print '************************************************************case 3: rotating around'
                motionProxy.post.moveTo(0.0, 0.0, Theta,
                    [ ["MaxStepX", 0.02],         # step of 2 cm in front
                      ["MaxStepY", 0.16],         # default value
                      ["MaxStepTheta", 0.2],      # default value
                      ["MaxStepFrequency", 0.0],  # low frequency
                      ["StepHeight", 0.01],       # step height of 1 cm
                      ["TorsoWx", 0.0],           # default value
                      ["TorsoWy", 0.1] ])         # torso bend 0.1 rad in front
            rospy.sleep(0.5)
        else:
            print "waypoint too old"
    StiffnessOff(motionProxy)


if __name__ == "__main__":
    robotIp = "192.168.2.197"

    global now
    rospy.init_node('nao_wp_tracker')
    now = rospy.get_rostime()

    if len(sys.argv) <= 1:
        print "Usage python motion_moveTo.py robotIP (optional default: 127.0.0.1)"
    else:
        robotIp = sys.argv[1]

    main(robotIp)

