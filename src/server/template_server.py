#! /usr/bin/env python

import roslib; roslib.load_manifest('behavior_trees')
import rospy
import actionlib
import behavior_trees.msg
# import actionlib_tutorials.msg

class ROSAction(object):
  # create messages that are used to publish feedback/result
  _feedback = behavior_trees.msg.ROSFeedback()
  _result   = behavior_trees.msg.ROSResult()

  def __init__(self, name):
    self._action_name = name
    self._as = actionlib.SimpleActionServer(self._action_name, behavior_trees.msg.ROSAction, execute_cb=self.goal_cb, auto_start = False)
    self._as.start()

  def goal_cb(self, goal):
    r = rospy.Rate(100)
    success = True
    self._feedback.FEEDBACK_ = 0
    # self._as.publish_feedback(self._feedback)

    rospy.loginfo('action_name: %s, goal: %i' % (self._action_name, goal.GOAL_))
    for i in xrange(1, 2000):
      rospy.loginfo('i: %i' % i)
      if rospy.is_shutdown():
        # rospy.loginfo('aborting through ros')
        break
      if self._as.is_preempt_requested():
        rospy.loginfo('%s: Preempted' % self._action_name)
        # self._as.set_preempted()
        # break
      else:
        # publish the feedback
        self._as.publish_feedback(self._feedback)
        r.sleep()

    rospy.loginfo('sending feedback')
    self._feedback.FEEDBACK_ = 2
    self._as.publish_feedback(self._feedback)


if __name__ == '__main__':
  rospy.init_node('action_name_default')
  ROSAction(rospy.get_name())
  rospy.loginfo('starting node %s' % rospy.get_name() )
  rospy.spin()
