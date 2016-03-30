#!/usr/bin/env python
import roslib
roslib.load_manifest('ltl3')
import rospy
from ltl3.msg import pose
import math
import tf
from tf.transformations import euler_from_quaternion

ROS_TO_MAZE = 166
RAD_TO_DEG = 57.3

if __name__ == '__main__':
    rospy.init_node('tf2pose')
    listener = tf.TransformListener()
    pose_pub_B = rospy.Publisher('cur_pose_B', pose)
    pose_pub_C = rospy.Publisher('cur_pose_C', pose)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            listener.waitForTransform('data/multi/patt.letter_a1', 'data/multi/patt.letter_b1', rospy.Time(0),rospy.Duration(0.2))
            (trans_B,rot_B) = listener.lookupTransform('data/multi/patt.letter_a1', 'data/multi/patt.letter_b1', rospy.Time(0))
            #(trans_C,rot_C) = listener.lookupTransform('data/multi/patt.letter_a2', 'data/multi/patt.letter_c2', rospy.Time(0))
            # trans, (dx, dy, dz)
            # rot, (x,y,z,w) should be [theta,0,0,1]
        except tf.Exception:
            try:
               listener.waitForTransform('data/multi/patt.letter_a2', 'data/multi/patt.letter_b2', rospy.Time(0),rospy.Duration(0.2))
               (trans_B,rot_B) = listener.lookupTransform('data/multi/patt.letter_a2', 'data/multi/patt.letter_b2', rospy.Time(0))
            except tf.Exception:
               info = 'No such letter found!'
               rospy.loginfo(info)
        try:
            listener.waitForTransform('data/multi/patt.letter_a1', 'data/multi/patt.letter_c1', rospy.Time(0),rospy.Duration(0.2))
            (trans_C,rot_C) = listener.lookupTransform('data/multi/patt.letter_a1', 'data/multi/patt.letter_c1', rospy.Time(0))
            #(trans_C,rot_C) = listener.lookupTransform('data/multi/patt.letter_a2', 'data/multi/patt.letter_c2', rospy.Time(0))
            # trans, (dx, dy, dz)
            # rot, (x,y,z,w) should be [theta,0,0,1]
        except tf.Exception:
            try:
               listener.waitForTransform('data/multi/patt.letter_a2', 'data/multi/patt.letter_c2', rospy.Time(0),rospy.Duration(0.2))
               (trans_C,rot_C) = listener.lookupTransform('data/multi/patt.letter_a2', 'data/multi/patt.letter_c2', rospy.Time(0))
            except tf.Exception:
               info = 'No such letter found!'
               rospy.loginfo(info)
        #############################################
        # roatation round x, y, z axies, for letter "B"
        angles_B= euler_from_quaternion(rot_B)
        cur_pose_B = pose()
        cur_pose_B.x = trans_B[0]*ROS_TO_MAZE # in cm
        cur_pose_B.y = trans_B[1]*ROS_TO_MAZE # in cm
        cur_pose_B.theta = angles_B[2] # in radian
        info = 'letter B position: X:%s, Y:%s, Theta:%s' %(cur_pose_B.x, cur_pose_B.y, cur_pose_B.theta*RAD_TO_DEG)
        rospy.loginfo(info)
        pose_pub_B.publish(cur_pose_B)
        #############################################
        # roatation round x, y, z axies, for letter "C"
        angles_C= euler_from_quaternion(rot_C)
        cur_pose_C = pose()
        cur_pose_C.x = trans_C[0]*ROS_TO_MAZE # in cm
        cur_pose_C.y = trans_C[1]*ROS_TO_MAZE # in cm
        cur_pose_C.theta = angles_C[2] # in radian
        info = 'letter C position: X:%s, Y:%s, Theta:%s' %(cur_pose_C.x, cur_pose_C.y, cur_pose_C.theta*RAD_TO_DEG)
        rospy.loginfo(info)
        pose_pub_C.publish(cur_pose_C)
	########
        rate.sleep()
