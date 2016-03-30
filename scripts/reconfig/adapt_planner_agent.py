#!/usr/bin/env python
import roslib
import numpy
import Queue
roslib.load_manifest('ltl3')
import rospy
from ltl3.msg import pose, activity, confirmation, knowledge
from math import sqrt, cos, sin, radians
import numpy
from NTUA_init import *
import sys

from ltl_tools.ts import MotionFts, ActionModel, MotActModel
from ltl_tools.planner import ltl_planner


confirm   = ['none', 0]
object_name = None
region = None

def distance(pose1, pose2):
    return sqrt( (pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2 )

def confirm_callback(data):
    global confirm
    name = data.name
    done = data.done
    confirm = [name, done]
    print "confirm", confirm

def knowledge_callback(data):
    global object_name
    global region
    object_name = data.object
    region = data.region
    print 'object [', object_name, '] detected at', region


def planner(letter, ts, act, task):
    global POSE
    global c
    global confirm
    global object_name
    global region
    rospy.init_node('planner_%s' %letter)
    print 'Agent %s: planner started!' %(letter)
    ###### publish to
    activity_pub   = rospy.Publisher('next_move_%s' %letter, activity, queue_size=10)
    ###### subscribe to
    rospy.Subscriber('activity_done_%s' %letter, confirmation, confirm_callback)
    rospy.Subscriber('knowledge_%s' %letter, knowledge, knowledge_callback)
    ####### agent information
    c = 0
    k = 0
    flag   = 0
    full_model = MotActModel(ts, act)
    #planner = ltl_planner(full_model, task, None)
    planner = ltl_planner(full_model, task, None)
    ####### initial plan synthesis
    planner.optimal(10)
    #######
    while not rospy.is_shutdown():
        next_activity = activity()
        ###############  check for knowledge udpate
        if object_name:
            # konwledge detected
            planner.update(object_name, region)
            print 'Agent %s: object incorporated in map!' %(letter,)
            planner.replan_simple()
            object_name = None
        ############### send next move
        next_move = planner.next_move
        next_state = planner.next_state
        ############### implement next activity
        if isinstance(next_move, str):
            # next activity is action
            next_activity.type = next_move
            next_activity.x = 0
            next_activity.y = 0
            print 'Agent %s: next action %s!' %(letter, next_activity.type)
            while not ((confirm[0]==next_move) and (confirm[1]>0)):
                activity_pub.publish(next_activity)
                rospy.sleep(0.06)
            rospy.sleep(1)
            confirm[1] = 0
            print 'Agent %s: action %s done!' %(letter, next_activity.type)
        else:
            print 'Agent %s: next waypoint (%.2f,%.2f)!' %(letter, next_move[0], next_move[1])
            while not ((confirm[0]=='goto') and (confirm[1]>0)):
                #relative_x = next_move[0]-POSE[0]
                #relative_y = next_move[1]-POSE[1]
                #relative_pose = [relative_x, relative_y]
                #oriented_relative_pose = rotate_2d_vector(relative_pose, -POSE[2])
                next_activity.type = 'goto'
                #next_activity.x = oriented_relative_pose[0]
                #next_activity.y = oriented_relative_pose[1]
                next_activity.x = next_move[0]
                next_activity.y = next_move[1]
                activity_pub.publish(next_activity)
                rospy.sleep(0.06)
            rospy.sleep(1)
            confirm[1] = 0
            print 'Agent %s: waypoint (%.2f,%.2f) reached!' %(letter, next_move[0], next_move[1])
            planner.pose = [next_move[0], next_move[1]]
        planner.find_next_move()    

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
