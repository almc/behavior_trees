"""Template server file.

Provides an example of a python behavior trees server
"""
# ! /usr/bin/env python

import rospy
import roslib
from rospyaction import ROSPYAction, RUNNING, FAILURE, SUCCESS
roslib.load_manifest('behavior_trees')


class ActionName(ROSPYAction):
    """Class definition implementing the functions execute_cb and reset_cb."""

    def __init__(self, name):
        """ActionName constructor."""
        ROSPYAction.__init__(self, name)
        self._time_to_complete = rospy.Duration(0)

    def execute_cb(self, dt):
        """Run the action logic."""
        print "Executing Main Task, elapsed_time: %f" % dt.to_sec()

        self._time_to_complete = self._time_to_complete + dt

        if self._time_to_complete.to_sec() < 1.5:
            self.set_feedback(RUNNING)
            return 0  # Returning 0 keeps the action alive

        else:
            self.set_feedback(SUCCESS)
            return 1

    def reset_cb(self):
        """Reset the action time."""
        print "Executing the reset callback"
        self._time_to_complete = rospy.Duration(0)

if __name__ == '__main__':
    rospy.init_node('action_name_default')
    ActionName(rospy.get_name())
    rospy.loginfo('starting node %s' % rospy.get_name())
    rospy.spin()
