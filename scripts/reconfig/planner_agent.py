#!/usr/bin/env python
import roslib
import numpy
import Queue
roslib.load_manifest('ltl3_NTUA')
import rospy
from ltl3.msg import pose, activity, confirmation, knowledge
from math import sqrt, cos, sin, radians
import numpy
from init import *
import sys

from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner

ac_POSE   = Queue.Queue(5)
POSE = [0, 0, 0]

confirm   = ['none', 0]
object_name = None

def distance(pose1, pose2):
    return sqrt( (pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2 )

def pose_callback(data): # data = '(x, y, theta)' x, y in cm, theta in radians
    global ac_POSE
    global POSE
    while ac_POSE.qsize() < 5:
        # print "while loop"
        ac_POSE.put([data.x, data.y, data.theta])
        # print "inserting to FIFO queue", [data.x, data.y, data.theta]
    else:
        #print "else"
        ac_POSE.get()
        ac_POSE.put([data.x, data.y, data.theta])
        # print "inserting to FIFO queue", [data.x, data.y, data.theta]
        # print "qsize", ac_POSE.qsize()
    POSE   = median_filter(ac_POSE)

def confirm_callback(data):
    global confirm
    name = data.name
    done = data.done
    confirm = [name, done]
    print "confirm", confirm

def knowledge_callback(data):
    global object_name
    object_name = data.object
    print 'object [', object_name, '] detected!'

def median_filter(raw_pose):
    list_pose = list(raw_pose.queue)
    x_list = [pose[0] for pose in list_pose]
    median_x = numpy.median(x_list)
    y_list = [pose[1] for pose in list_pose]
    median_y = numpy.median(y_list)
    theta_list = [pose[2] for pose in list_pose]
    median_theta = numpy.median(theta_list)
    return [median_x, median_y, median_theta]

def check_dist(cur_pose, waypoint):
    dist = distance(cur_pose, waypoint)
    # print "dist", dist
    if (dist <= 30.0):
        # print "**********reached waypoint**********"
        return True
    return False

def rotate_2d_vector(v, theta):
    new_v = [0,0]
    new_v[0] = cos(theta)*v[0] - sin(theta)*v[1]
    new_v[1] = sin(theta)*v[0] + cos(theta)*v[1]
    return new_v

def planner(letter, ts, act, task):
    global POSE
    global c
    global confirm
    rospy.init_node('planner_%s' %letter)
    print 'Agent %s: planner started!' %(letter)
    ###### publish to
    activity_pub   = rospy.Publisher('next_move_%s' %letter, activity)
    ###### subscribe to
    rospy.Subscriber('cur_pose_%s' %letter, pose, pose_callback)
    rospy.Subscriber('activity_done_%s' %letter, confirmation, confirm_callback)
    rospy.Subscriber('knowledge_%s' %letter, knowledge, knowledge_callback)
    ####### agent information
    c = 0
    k = 0
    flag   = 0
    full_model = MotActModel(ts, act)
    planner = ltl_planner(full_model, task, None)
    ####### initial plan synthesis
    planner.optimal(10,'static')
    #######
    while not rospy.is_shutdown():
        while not POSE:
            rospy.sleep(0.1)
        planner.cur_pose = POSE[0:2]
        next_activity_bowser = activity()
        next_activity = activity()
        ###############  check for knowledge udpate
        if object_name:
            # konwledge detected
            planner.update(object_name)
            print 'Agent %s: object incorporated in map!' %(letter,)
            planner.replan()
        ############### send next move
        next_move = planner.next_move
        ############### implement next activity
        if isinstance(next_move, str):
            # next activity is action
            next_activity.type = next_move
            next_activity.x = 0
            next_activity.y = 0
            print 'Agent %s: next action %s!' %(letter, next_activity.type)
            while not ((confirm[0]==next_move) and (confirm[1]>0)):
                activity_pub.publish(next_activity)
                rospy.sleep(0.5)
            print 'Agent %s: action %s done!' %(letter, next_activity.type)
        else:
            print 'Agent %s: next waypoint (%d,%d)!' %(letter, next_move[0], next_move[1])
            while not (check_dist(POSE[0:2], next_move)):
                relative_x = next_move[0]-POSE[0]
                relative_y = next_move[1]-POSE[1]
                relative_pose = [relative_x, relative_y]
                oriented_relative_pose = rotate_2d_vector(relative_pose, -POSE[2])
                next_activity.type = 'goto'
                next_activity.x = oriented_relative_pose[0]
                next_activity.y = oriented_relative_pose[1]
                activity_pub.publish(next_activity)
                rospy.sleep(0.5)
            print 'Agent %s: waypoint (%d,%d) reached!' %(letter, next_move[0], next_move[1])
        planner.trace.append(planner.find_next_move())

def planner_agent(agent_letter):
    if agent_letter in init:
        agent_ts, agent_act, agent_task = init[agent_letter]
        planner(agent_letter, agent_ts, agent_act, agent_task)
    else:
        print('Agent not specified in init.py')


if __name__ == '__main__':
    ########
    if len(sys.argv) == 2:
        agent_letter = str(sys.argv[1])
        # to run: python planner_agent.py B
    ###############
    try:
        planner_agent(agent_letter)
    except rospy.ROSInterruptException:
        pass
