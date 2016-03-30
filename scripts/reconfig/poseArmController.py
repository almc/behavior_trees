#!/usr/bin/env python

# ROS imports
import roslib
import rospy

# Msgs imports
from brics_actuator.msg import JointVelocities, JointPositions, JointValue
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Vector3, Quaternion

# Service imports
from std_srvs.srv import Empty, EmptyResponse


# Python imports
import numpy as np
from numpy.linalg import *
from math import *
from PyKDL import *



class poseArmController:
    def __init__(self, name):
        """ Control Framework for Youbot Arm """
        self.name = name
       
        #Minimum Joint Values
        self.qmin = np.array([-2.9496,-1.1344,-2.6354,-1.7889,-2.9234])
        #Maximum Joint Values
        self.qmax = np.array([2.9496,1.5707,2.5481,1.7889,2.9234])
        #Offset Values -> [j0, j1, j2, j3, j4] = zeros(5) ---> Arm Stands up
        self.q_offset = np.array([2.9496, 1.1344, -2.5481, 1.7889, 2.9234])
        #self.q_offset = np.array([2.9496, 1.1344, -2.5481, 1.7889, 0.0])
        
        # Load parameters
        self.enable = True #Service Flag for enable/disable the framework
         
        #Initilize Time
        self.t0 = rospy.Time.now().to_sec()
        
		  #Initialiaze Joint States
        self.q_pos = JntArray(5)     #Joint Positions
        self.q_vel = JntArray(5)      #Joint Velocities
        self.q_torques = JntArray(5)  #Joint Torques
        
        for i in range (0,5):
          self.q_pos[i] = 0.0     
          self.q_vel[i] = 0.0      
          self.q_torques[i] = 0.0  
	            
        # Create publishers
        #Joint Position Control
        self.pub_qpos = rospy.Publisher("PY/arm_1/arm_controller/position_command", JointPositions)
        #Joint Velocity Control
        self.pub_qvel = rospy.Publisher("PY/arm_1/arm_controller/velocity_command", JointVelocities)       

        # Create Subscriber
        #Get Joint States (position, velocity, torque)
        rospy.Subscriber("PY/joint_states", JointState, self.updateArmStates)
        

        #Create services
        self.enable_srv = rospy.Service('csc_youbot_arm/enable', Empty, self.enableSrv)
        self.disable_srv = rospy.Service('csc_youbot_arm/disable', Empty, self.disableSrv)
        
        
    def enableSrv(self, req):
        self.enable = True
        rospy.loginfo('%s Enabled', self.name)
        return EmptyResponse()


    def disableSrv(self, req):
        self.enable = False
        rospy.loginfo('%s Disabled', self.name)
        return EmptyResponse()
        
    def updateArmStates(self, data):
        if data.name[0] == 'arm_joint_1':
           for i in range (0, 5):
               self.q_pos[i] = data.position[i] - self.q_offset[i]
           for i in range (0, 5):
               self.q_vel[i] = data.velocity[i]  
           for i in range (0, 5):
               self.q_torques[i] = data.effort[i] 
               
    def jsPoscontrol(self,qd):
        
        #Initialize Position Command Msgs
        qposMsg = JointPositions()
        qposMsg.positions = [JointValue(), JointValue(), JointValue(), JointValue(), JointValue()]

              
        #Fill Position Msg
        qposMsg.positions[0].joint_uri = 'arm_joint_1'
        qposMsg.positions[0].unit = 'rad'
        qposMsg.positions[0].value = qd[0] + self.q_offset[0] 
        
        qposMsg.positions[1].joint_uri = 'arm_joint_2'
        qposMsg.positions[1].unit = 'rad'
        qposMsg.positions[1].value = qd[1] + self.q_offset[1] 
        qposMsg.positions[2].joint_uri = 'arm_joint_3'
        qposMsg.positions[2].unit = 'rad'
        qposMsg.positions[2].value = qd[2] + self.q_offset[2]
      
        qposMsg.positions[3].joint_uri = 'arm_joint_4'
        qposMsg.positions[3].unit = 'rad'
        qposMsg.positions[3].value = qd[3] + self.q_offset[3]
        
        qposMsg.positions[4].joint_uri = 'arm_joint_5'
        qposMsg.positions[4].unit = 'rad'
        qposMsg.positions[4].value = qd[4] + self.q_offset[4]
        
        #Publish Position Msg
        self.pub_qpos.publish(qposMsg)
        

  
        
if __name__ == '__main__':
    try:
        rospy.init_node('poseArmController')
        armCTRL = poseArmController(rospy.get_name())
       
        qd = JntArray(5)
        qd[0] = 0.0
        qd[1] = 0.0
        qd[2] = 0.0
        qd[3] = 0.0
        qd[4] = 0.0
          
        while not rospy.is_shutdown():
            t = rospy.Time.now().to_sec() - armCTRL.t0
            
            armCTRL.jsPoscontrol(qd)
            rospy.sleep(1.0/300.0)
        rospy.spin()

    except rospy.ROSInterruptException: pass
