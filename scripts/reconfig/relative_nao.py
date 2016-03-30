#!/usr/bin/env python
import roslib
roslib.load_manifest('ltl3_NTUA')
import rospy
import Queue
import numpy

import math
import tf
from tf.transformations import euler_from_quaternion

from ar_pose.msg import ARMarkerExtended, ARMarkersExtended
from ltl3_NTUA.msg import pose, activity, confirmation, knowledge
from math import sqrt, cos, sin, radians
from numpy.linalg import inv

def pose_callback(data):
    global ac_POSE
    global POSE

    pose = [0, 0, 0] # x, y, theta
    for marker in data.markers:
        if marker.id == 2: # ID for NAO marker
            pose[0] = marker.pose.pose.position.x
            pose[1] = marker.pose.pose.position.y
            angle = euler_from_quaternion([marker.pose.pose.orientation.x, 
                marker.pose.pose.orientation.y, marker.pose.pose.orientation.z,
                marker.pose.pose.orientation.w])
            pose[2] = angle[2]

    while ac_POSE.qsize() < 5:
        # print "while loop"
        ac_POSE.put(pose)
        # print "inserting to FIFO queue", [data.x, data.y, data.theta]
    else:
        #print "else"
        ac_POSE.get()
        ac_POSE.put(pose)
        # print "inserting to FIFO queue", [data.x, data.y, data.theta]
        # print "qsize", ac_POSE.qsize()
    POSE   = median_filter(ac_POSE)


def median_filter(raw_pose):
    list_pose = list(raw_pose.queue)
    x_list = [pose[0] for pose in list_pose]
    median_x = numpy.median(x_list)
    y_list = [pose[1] for pose in list_pose]
    median_y = numpy.median(y_list)
    theta_list = [pose[2] for pose in list_pose]
    median_theta = numpy.median(theta_list)
    return [median_x, median_y, median_theta]

def activity_callback(data):
    global goal_loc
    goal_loc[0] = data.x
    goal_loc[1] = data.y
    goal_loc[2] = data.psi
    goal_loc[3] = data.header   
    return goal_loc


def rotate_3d_vector(theta, tr_x, tr_y, x_goal, y_goal):
    new_v = [0.0, 0.0, 0.0];
    T_G_new_N = numpy.array([[cos(theta), sin(theta), tr_x], [-sin(theta), cos(theta), tr_y], [0.0, 0.0, 1] ]);
    T_G_new_N_inv = inv(T_G_new_N);
    goal_pos_tmp = numpy.array([x_goal, y_goal, 1]);
    new_v = T_G_new_N_inv.dot(goal_pos_tmp);
    #new_v[0] =  cos(theta)*v[0] + sin(theta)*v[1] + tr_x*v[2]
    #new_v[1] = -sin(theta)*v[0] + cos(theta)*v[1] + tr_y*v[2]
    #new_v[2] =  0.0*v[0] + 0.0*v[1] + 1.0*v[2]
    return new_v

if __name__ == '__main__':
    global POSE
    global goal_loc
    global ac_POSE

    ac_POSE   = Queue.Queue(5)
    POSE = [0, 0, 0]
    goal_loc = [0, 0, 0, 0]
    relative_goal = [0, 0, 0]

    rospy.init_node('relative_nao')

    relative_nao_pub = rospy.Publisher('next_move_PY_relative',activity)
    nao_confirm_pub = rospy.Publisher('activity_done_PY',confirmation)

    rospy.Subscriber('ceiling/pose', ARMarkersExtended, pose_callback)
    rospy.Subscriber('next_move_PY',  activity, activity_callback)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
            #while (goal_loc[0] != 0) or (goal_loc[1] != 0):
            x_nao_G = POSE[0];
            y_nao_G = POSE[1];
            x_nao_G_new =  x_nao_G;
            y_nao_G_new = -y_nao_G;
            x_goal_G = goal_loc[0];
            y_goal_G = goal_loc[1];
            x_goal_G_new =  x_goal_G;
            y_goal_G_new = -y_goal_G;

            print "xy_nao_G", x_nao_G, y_nao_G;
            print "xy_nao_G_new", x_nao_G_new, y_nao_G_new;
            error_vector = rotate_3d_vector(POSE[2], x_nao_G_new, y_nao_G_new, x_goal_G_new, y_goal_G_new)
            #print error_vector
            print "xy_goal_N", error_vector;
            #relative_goal[0] = goal_loc[0] - POSE[0]
            #relative_goal[1] = goal_loc[1] - POSE[1]
            #relative_goal[2] = goal_loc[2] - POSE[2]
            #oriented_relative_pose = rotate_2d_vector(relative_goal, -POSE[2])
            relative_nao_msg = activity()
            relative_nao_msg.type = ''
            #relative_nao_msg.header = goal_loc[3]
            relative_nao_msg.x = 100*error_vector[0];#oriented_relative_pose[0]*100
            relative_nao_msg.y = 100*error_vector[1];#oriented_relative_pose[1]*100
            relative_psi = goal_loc[2] - POSE[2]
            relative_psi_to_pi = ( relative_psi + numpy.pi) % (2 * numpy.pi ) - numpy.pi 
            relative_nao_msg.psi = relative_psi_to_pi
            relative_nao_pub.publish(relative_nao_msg)
            print "errors (x,y,theta)", numpy.absolute(relative_nao_msg.x), numpy.absolute(relative_nao_msg.y), numpy.absolute(relative_nao_msg.psi);
            if (numpy.absolute(relative_nao_msg.x) <= 20.0 and numpy.absolute(relative_nao_msg.y) <= 20.0 and numpy.absolute(relative_nao_msg.psi) <= 0.4):
                print "sending confirmation now"
                #print error_vector
                nao_confirm_msg = confirmation()
                nao_confirm_msg.confheader = goal_loc[3]
                nao_confirm_msg.name = 'goto'
                nao_confirm_msg.done = 1
                nao_confirm_pub.publish(nao_confirm_msg)
            #print 'realtive_nao, (x,y)', (relative_nao_msg.x,relative_nao_msg.y)
            #rospy.sleep(0.5)
            rate.sleep()
