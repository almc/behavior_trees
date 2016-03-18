"""Template server file.

Provides an example of a python behavior trees server
"""
# ! /usr/bin/env python

import roslib
import rospy
import actionlib
import behavior_trees.msg
# import actionlib_tutorials.msg

roslib.load_manifest('behavior_trees')


class ROSPYAction(object):
    """Create messages that are used to publish feedback/result."""

    def __init__(self, name):
        """ROSPYAction constructor.

        Starts the actionlib simple action server
        """
        # "protected" variables definition
        self._feedback = NODE_ERROR
        self._result = NODE_ERROR
        self._start_time = rospy.Time.now()
        self._elapsed_time = rospy.Duration(0)
        self._action_name = name
        self._goal = None
        self._started = False
        self._active = False
        self._as = actionlib.SimpleActionServer(self._action_name,
                                                behavior_trees.msg.ROSAction,
                                                execute_cb=self.goal_cb,
                                                auto_start=False)
        self._as.start()

    def goal_cb(self, goal):
        """Implement the callback logic.

        When the actionlib client sends a goal, this
        method will be called.
        """
        print "%%%%%%%%%% goalCB %%%%%%%%%%"
        self._goal = self._as.accept_new_goal()
        print "Received Goal: %d" % self._goal

        if self._feedback != SUCCESS and self._feedback != FAILURE:
            # future mutex
            started = self._started

            print "started: %d" % started

            # the python server will only assume positive ticks
            if started:
                self._reset_timeout()
            else:
                self._start()
        else:
            # mutexes?
            self._feedback.FEEDBACK_ = NODE_ERROR
            self._feedback.RESULT_ = NODE_ERROR

        print "%%%%%%%%%% goalCB Exit%%%%%%%%%%"

    def reset_timeout(self):
        """Call this method when a tick is received."""
        # start time mutex?
        self._start_time = rospy.Time.now()

    def start(self):
        """Start the execution thread."""
        print "Executing Reset Callback Now"
        self.reset_cb()
        # future mutex?
        self._started = True
        # future mutex?
        self._active = True

        print "Starting Thread Now"
        print "++++++++++++++++++++++++++++++++++++++++++++++"

    def stop(self):
        """Stop the execution thread."""
        print "Stopping Thread Now"
        # future mutex?
        self._started = False
        # future mutex?
        self._active = False
        print "Stopped successfully"

    def timeout_check(self):
        """Check if timeout was achieved.

        If no ticks are received in more than a timeout threshold,
        the execution thread will timeout and be killed
        """
        # future mutexes?
        self._elapsed_time = rospy.Time.now() - self._start_time
        dt = self._elapsed_time

        print "elapsed_time since tick: %f" % dt.to_sec()

        if dt.to_sec() > TIMEOUT_THRESHOLD:
            return False
        else:
            return True

    def send_feedback(self):
        """Publish the actionlib feedback."""
        print "Sending Feedback now"
        # mutex?
        print "Sending Feedback: %d" % self._feedback.FEEDBACK_
        self._as.publish_feedback(self._feedback)

    def set_feedback(self, state):
        """Set the action feedback."""
        # mutex?
        self._feedback.FEEDBACK_ = state
