#!/usr/bin/env python
import roslib
import numpy
import Queue
#roslib.load_manifest('ltl3')
import rospy
from ltl3_NTUA.msg import pose, confirmation, knowledge
from math import sqrt, cos, sin, radians
import numpy
#from NTUA_init import *
import sys
from nao_basic.msg import *
from ar_pose.msg import *
#from ltl_tools.ts import MotionFts, ActionModel, MotActModel
#from ltl_tools.planner import ltl_planner
from geometry_msgs.msg import Quaternion, Vector3
from math import floor, exp, sqrt, atan2, sin, pi, asin, cos

ac_POSE   = Queue.Queue(5)
POSE = [0.0, 0.0, 0.0]

confirm   = ['none', 0]
object_name = None
region = None

def distance(pose1, pose2):
    return sqrt( (pose1[0]-pose2[0])**2+(pose1[1]-pose2[1])**2 )

def pose_callback(m): # data = '(x, y, theta)' x, y in cm, theta in radian

    global POSE
    for poses in m.markers:
        if poses.id == 2:
           #print "updating pose 2"
           POSE[0]= poses.pose.pose.position.x
           POSE[1] = -poses.pose.pose.position.y
           
           quat2 = Quaternion(0,0,poses.pose.pose.orientation.y,poses.pose.pose.orientation.x)
           euler2 = quat2eulerZYX (quat2)
           POSE[2] = -euler2.z


#Quaternion to Euler-ZYX
def quat2eulerZYX (q):
    euler = Vector3()
    tol = quat2eulerZYX.tolerance

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


def check_dist(cur_pose, waypoint):
    dist = distance(cur_pose, waypoint)
    # print "dist", dist
    if (dist <= 0.5):
        # print "**********reached waypoint**********"
        return True
    return False

def rotate_2d_vector(v, theta):
    new_v = [0,0]
    new_v[0] = cos(theta)*v[0] - sin(theta)*v[1]
    new_v[1] = sin(theta)*v[0] + cos(theta)*v[1]
    return new_v

def planner( ):
    global POSE
    letter = 'NAO'
    rospy.init_node('planner_%s' %letter)
    print 'Agent %s: planner started!' %(letter)
    ###### publish to
    activity_pub   = rospy.Publisher('next_move_mover', activity, queue_size=10)
    ###### subscribe to
    rospy.Subscriber("/ceiling/pose", ARMarkersExtended, pose_callback)
    ##############################
    #wps = ((0.08, y) for y in [1.97-0.5*k for k in xrange(0,7)])
    #Waypoints = [a for a in wps]
    Waypoints = [[0.64, -0.48],[0.41,0.7]]
    next_activity = activity()
    print 'NAO waypoints:'
    print Waypoints
    ##################
    ############### implement next activity
    direction = 1 #1 from door to pool, -1 from pool to door
    K = 0
    while True:
        while ((K < len(Waypoints) -1 and direction == 1) or (K > 0 and direction == -1)):
            wp = Waypoints[K]
            print 'Agent %s: next waypoint (%.2f,%.2f)!' %(letter, wp[0], wp[1])
            while not (check_dist(POSE[0:2], wp)):
                relative_x = wp[0]-POSE[0]
                relative_y = wp[1]-POSE[1]
                relative_pose = [relative_x, relative_y]
                oriented_relative_pose = rotate_2d_vector(relative_pose, -POSE[2])
                next_activity.type = 'goto'
                next_activity.x = oriented_relative_pose[0]*100
                next_activity.y = oriented_relative_pose[1]*100
                activity_pub.publish(next_activity)
                rospy.sleep(1)
            print 'Agent %s: waypoint (%.2f,%.2f) reached!' %(letter, wp[0], wp[1])
            K += 1*direction
        print 'ALL waypoints finished'
        direction*= -1


if __name__ == '__main__':
    ########
    if len(sys.argv) == 2:
        agent_letter = str(sys.argv[1])
        # to run: python planner_agent.py B
    ###############
    try:
        planner()
    except rospy.ROSInterruptException:
        pass
