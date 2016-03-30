#!/usr/bin/env python

# ROS imports
import roslib

import rospy

# Msgs imports
from geometry_msgs.msg import Twist, Quaternion, Vector3

# Service imports
from std_srvs.srv import Empty, EmptyResponse
from ar_pose.msg import *
from ltl3_NTUA.msg import *
# Python imports
import numpy as np
from numpy.linalg import *
import math as math1

#Custom imports
import sys
from math import floor, exp, sqrt, atan2, sin, pi, asin, cos
from geometry_msgs.msg import Point, Vector3, Quaternion

header = -1

class poseBaseVSController:
    def __init__(self, name):
        """ Position Based Visual Servo Control for the Youbot Base """
        self.name = name
      
        # Load parameters
        self.enable = True #Service Flag for enable/disable the controller
        self.counter = 0
         
        #Initilize Time
        self.t0 = rospy.Time.now().to_sec()
        
        #Initialiaze platform States
        self.pose1 = np.zeros(3,dtype=float) #Youbot Base 1
        self.pose2 = np.zeros(3,dtype=float) #Youbot Base 2
        
       #Initialize desired pose states
        self.posed = np.zeros(3,dtype=float) #For the time Youbot 1 (IP:147.102.51.243)
        
        #Desired pose given in terminal
        self.posed[0] = 0.0 #(float (sys.argv[1]))
        self.posed[1] = 0.0 #(float (sys.argv[2]))
        self.posed[2] = 0.0 #(float (sys.argv[3]))
        
        # Create publishers
        self.pub_vel = rospy.Publisher('/py/cmd_vel', Twist)
        self.pub_confirm = rospy.Publisher('activity_done_PY', confirmation)
        
        # Create Subscriber
        #rosOY.Subscriber("/cur_pose_C", pose, self.updatePose)
        rospy.Subscriber("/ceiling/pose", ARMarkersExtended, self.updateRobotsPose)
        rospy.Subscriber("/next_move_PY", activity, self.updateNextMove)

        #Create services
        self.enable_srv = rospy.Service('/youbot_base_control/enable', Empty, self.enableSrv)
        self.disable_srv = rospy.Service('/youbot_base_control/disable', Empty, self.disableSrv)
     
     
        self.xd_old = 0.0
        self.yd_old = 0.0
        
        self.Action = ''
        self.flag_conf = False

    def updateNextMove(self, data):
        self.Action = data.type
        global header

        if data.header != header:
           self.flag_conf = True
           header = data.header 
        
        if data.type == 'goto' and self.flag_conf == True:
           self.posed[0] = data.x
           self.posed[1] = -data.y
           self.posed[2] = data.psi


    def updateRobotsPose(self, m):

        for poses in m.markers:
      
            if poses.id == 0:
               #print "updating pose 1"
               self.pose1[0] = poses.pose.pose.position.x
               self.pose1[1] = -poses.pose.pose.position.y
          
               quat1 = Quaternion(0,0,poses.pose.pose.orientation.y,poses.pose.pose.orientation.x)   
               euler1 = self.quat2eulerZYX (quat1)
               self.pose1[2] = -euler1.z
         

            if poses.id == 1:
               #print "updating pose 0"
               self.pose2[0]= poses.pose.pose.position.x
               self.pose2[1] = -poses.pose.pose.position.y
               
               quat2 = Quaternion(0,0,poses.pose.pose.orientation.y,poses.pose.pose.orientation.x)
               euler2 = self.quat2eulerZYX (quat2)
               self.pose2[2] = -euler2.z
               
               
    #Quaternion to Euler-ZYX
    def quat2eulerZYX (self,q):
        euler = Vector3()
        tol = self.quat2eulerZYX.tolerance

        qww, qxx, qyy, qzz = q.w*q.w, q.x*q.x, q.y*q.y, q.z*q.z
        qwx, qxy, qyz, qxz= q.w*q.x, q.x*q.y, q.y*q.z, q.x*q.z
        qwy, qwz = q.w*q.y, q.w*q.z

        test = -2.0 * (qxz - qwy)
        if test > +tol:
           euler.x = atan2 (-2.0*(qyz-qwx), qww-qxx+qyy-qzz)
           euler.y = +0.5 * pi
           euler.z = 0.0

           return euler

        elif test < -tol:
             euler.x = atan2 (-2.0*(qyz-qwx), qww-qxx+qyy-qzz)
             euler.y = -0.5 * pi
             euler.z = tol

             return euler

        else:
           euler.x = atan2 (2.0*(qyz+qwx), qww-qxx-qyy+qzz)
           euler.y = asin (test)
           euler.z = atan2 (2.0*(qxy+qwz), qww+qxx-qyy-qzz)

           return euler
    quat2eulerZYX.tolerance=0.99999
        

    def enableSrv(self, req):
        self.enable = Truef
        rospy.loginfo('%s Enabled', self.name)
        return EmptyResponse()


    def disableSrv(self, req):
        self.enable = False
        rospy.loginfo('%s Disabled', self.name)
        return EmptyResponse()
   
               
    def motionControl(self):

        x = self.pose2[0]
        y = self.pose2[1]
        psi = self.pose2[2]

        x_d = self.posed[0]
        y_d = self.posed[1]
        psi_d = self.posed[2]

        #Pose Errors
        e_x = x - x_d
        e_y = y - y_d
        e_psi = psi - psi_d

        #rospy.loginfo("ex: %s ey: %s epsi: %s",e_x, e_y, e_psi)

        #Controller Gains
        k_x = 0.15
        k_y = 0.15
        k_psi = 0.15

        #Motion Control Scheme
        u = -k_x*e_x*cos(psi) - k_y*e_y*sin(psi)
        v =  k_x*e_x*sin(psi) - k_y*e_y*cos(psi)
        r = -k_psi*e_psi 
        
        cmdMsg = Twist()

        #u = 0.0
        #v = 0.0
        #r = 0.0

        #print "x:", x, "y:", y, "psi:", psi

        criteria1 = 0.2
        criteria2 = 0.05
        
        norm1 = sqrt(e_x**2+e_y**2)
        norm2 = abs(e_psi)
        
        if norm1 < criteria1 and norm2 < criteria2 and self.flag_conf == True:
           self.flag_conf = False
           confMsg = confirmation()
           confMsg.name = 'goto'
           confMsg.done = 1
           print '=============================================='
           confMsg.confheader = header
           print header
           print confMsg.done
           self.pub_confirm.publish(confMsg)
           
        #Publish Position Msg
        if self.Action == 'goto':
           cmdMsg.linear.x = u
           cmdMsg.linear.y = v
           cmdMsg.angular.z = r
        else:
           cmdMsg.linear.x = 0.0
           cmdMsg.linear.y = 0.0
           cmdMsg.angular.z = 0.0
        #print "I am in publishing", u, v, r 

        self.pub_vel.publish(cmdMsg)
        
if __name__ == '__main__':
    try:
        rospy.init_node('poseBaseVSController_PY')
        
        poseBaseObj = poseBaseVSController(rospy.get_name())
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - poseBaseObj.t0
            poseBaseObj.motionControl()
            rospy.sleep(1.0/10.0)

    except rospy.ROSInterruptException: pass
pose
