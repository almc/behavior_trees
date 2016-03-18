"""Template server file.

Provides an example of a python behavior trees server
"""
# ! /usr/bin/env python

import roslib
import rospy
import actionlib
import behavior_trees.msg
import threading
from threading import Thread, Lock

roslib.load_manifest('behavior_trees')

FAILURE = 0
SUCCESS = 1
RUNNING = 2
NODE_ERROR = 3

TIMEOUT_THRESHOLD = 2.0
EXECUTION_FREQUENCY = 10.0


class ROSPYAction(object):
    """Create messages that are used to publish feedback/result."""

    def __init__(self, name):
        """ROSPYAction constructor.

        Starts the actionlib simple action server
        """
        # "protected" variables definition
        self._feedback = behavior_trees.msg.ROSFeedback()
        self._feedback.FEEDBACK_ = NODE_ERROR
        self._result = behavior_trees.msg.ROSResult()
        self._result.RESULT_ = NODE_ERROR
        self._start_time = rospy.Time.now()
        self._elapsed_time = rospy.Duration(0)
        self._action_name = name
        self._goal = None
        self._started = False
        self._active = False

        self._mutex_started = Lock()
        self._mutex_active = Lock()
        self._mutex_start_time = Lock()
        self._mutex_elapsed_time = Lock()
        self._mutex_feedback = Lock()
        self._mutex_result = Lock()
        self._execution_thread = None

        self._as = actionlib.SimpleActionServer(self._action_name,
                                                behavior_trees.msg.ROSAction,
                                                auto_start=False)
        self._as.register_goal_callback(self.goal_cb)
        self._as.register_preempt_callback(self.preempt_cb)
        self._as.start()

    def goal_cb(self):
        """Implement the callback logic.

        When the actionlib client sends a goal, this
        method will be called.
        """
        print "%%%%%%%%%% goalCB %%%%%%%%%%"
        self._goal = self._as.accept_new_goal()
        print "Received Goal"

        if self._feedback.FEEDBACK_ != SUCCESS and\
                self._feedback.FEEDBACK_ != FAILURE:
            self._mutex_started.acquire()
            started = self._started
            self._mutex_started.release()

            print "started: %d" % started

            # the python server will only assume positive ticks
            if started:
                self.reset_timeout()
            else:
                self.start()

        if self._feedback.FEEDBACK_ == SUCCESS or\
           self._feedback.FEEDBACK_ == FAILURE:

            print "returned %d, setting NODE_ERROR" % self._feedback.FEEDBACK_
            self._mutex_feedback.acquire()
            self._feedback.FEEDBACK_ = NODE_ERROR
            self._mutex_feedback.release()

            self._mutex_result.acquire()
            self._result.RESULT_ = NODE_ERROR
            self._mutex_result.release()

        print "%%%%%%%%%% goalCB Exit%%%%%%%%%%"

    def preempt_cb(self):
        """Preempt the action."""
        print "%%%%%%%%%% preemptCB %%%%%%%%%%"
        self.reset_timeout()

    def reset_timeout(self):
        """Call this method when a tick is received."""
        print "reseting timeout"
        self._mutex_start_time.acquire()
        self._start_time = rospy.Time.now()
        self._mutex_start_time.release()

    def start(self):
        """Start the execution thread."""
        print "Executing Reset Callback Now"
        self.reset_cb()
        self._mutex_started.acquire()
        self._started = True
        self._mutex_started.release()

        self._mutex_active.acquire()
        self._active = True
        self._mutex_active.release()

        print "Starting Thread Now"
        print "++++++++++++++++++++++++++++++++++++++++++++++"

        self._execution_thread = Thread(target=self.execution_thread)
        self._execution_thread.start()

    def stop(self):
        """Stop the execution thread."""
        print "Stopping Thread Now"
        self._mutex_started.acquire()
        self._started = False
        self._mutex_started.release()

        self._mutex_active.acquire()
        self._active = False
        self._mutex_active.release()
        print "Stopped successfully"

    def timeout_check(self):
        """Check if timeout was achieved.

        If no ticks are received in more than a timeout threshold,
        the execution thread will timeout and be killed
        """
        self._mutex_elapsed_time.acquire()
        self._mutex_start_time.acquire()
        self._elapsed_time = rospy.Time.now() - self._start_time
        dt = self._elapsed_time
        self._mutex_elapsed_time.release()
        self._mutex_start_time.release()

        print "elapsed_time since tick: %f" % dt.to_sec()

        if dt.to_sec() > TIMEOUT_THRESHOLD:
            return False
        else:
            return True

    def send_feedback(self):
        """Publish the actionlib feedback."""
        print "Sending Feedback now"
        self._mutex_feedback.acquire()
        print "Sending Feedback: %d" % self._feedback.FEEDBACK_
        self._as.publish_feedback(self._feedback)
        self._mutex_feedback.release()
        print "Sent feedback"

    def set_feedback(self, state):
        """Set the action feedback."""
        self._mutex_feedback.acquire()
        self._feedback.FEEDBACK_ = state
        self._mutex_feedback.release()

    def is_active(self):
        """Return the active flag value."""
        self._mutex_active.acquire()
        active = self._active
        self._mutex_active.release()
        return active

    def execution_thread(self):
        """Implement the logic for the execute callback.

        Thread that runs the execute callback for the server.
        Implements extra logic for timming out.
        """
        r = rospy.Rate(EXECUTION_FREQUENCY)
        self._mutex_start_time.acquire()
        self._start_time = rospy.Time.now()
        t0 = self._start_time
        self._mutex_start_time.release()

        while self.is_active() and not rospy.is_shutdown():
            print "current_thread(): %d"\
                   % threading.current_thread().ident

            active = self.timeout_check()
            print "im active: %d" % active
            self._mutex_active.acquire()
            if active:
                active = self._active

            self._active = active
            self._mutex_active.release()

            if active:
                dt = rospy.Time.now() - t0
                t0 = rospy.Time.now()
                print "executing cb %f" % dt.to_sec()

                if self.execute_cb(dt):
                    print "callback returned!"
                    break

                self.send_feedback()

            else:
                print "self._execution_thread is not active"

            r.sleep()

        print "About to Destroy Thread"
        print "-----------------------"
        self.send_feedback()
        self.stop()
