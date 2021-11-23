 #! /usr/bin/env python

from __future__ import print_function

from threading import main_thread
from genpy import message

import rospy
import actionlib
import messages.msg 

class robotposition(object):
    _feedback = messages.msg.posActionFeedback
    _result = messages.msg.posResult

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, messages.msg.posAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        
    def execute_cb(self, goal):
        r = rospy.Rate(1)
        success = True

        self._feedback.sequence = []
        self._feedback.sequence.append(0)
        self._feedback.sequence.append(1)

        rospy.loginfo('%s: Executing %i, %i' % (self._action_name, goal.order, self._feedback.sequence[0], self._feedback.sequence[1]))

        for i in range(1, goal.order):

            rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted
            success = False
            break
        self._feedback.sequence.append(self._feedback.sequence[i] + self._feedback.sequence[i-1])

        self._as.publish_feedback(self._feedback)

        r.sleep

        if success:
            self._result.sequence = self._feedback.sequence
            rospy.loginfo('%s: Succeeded' % self._action_name)
            self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('Robot_position_msg')
    server = robotposition(rospy.get_name())
    rospy.spin

    








