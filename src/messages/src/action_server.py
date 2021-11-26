 #! /usr/bin/env python
from __future__ import print_function
from threading import main_thread
import rospy
import actionlib
import messages.msg

class robotposition(object):
    # Uesd for feedback of result and goal if needed
    _feedback = messages.msg.posFeedback()
    _result = messages.msg.posResult()
    _goal = messages.msg.posGoal()

    def __init__(self, name):
        self._action_name = name
        # Create and start the action server
        self._as = actionlib.SimpleActionServer(self._action_name, messages.msg.posAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        
    def execute_cb(self, goal):
        r = rospy.Rate(1)
        success = True

        # Print the position of recived goal
        print("Position: ")
        print("x: ", goal.x)
        print("y: ", goal.y)
        print("z: ", goal.z)
        print("rotx: ", goal.rotx)
        print("roty: ", goal.roty)
        print("rotz: ", goal.rotz)

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)

        return goal.x, goal.y, goal.z, goal.rotx, goal.roty, goal.rotz

if __name__ == '__main__':
    rospy.init_node('Robot_position_msg')
    server = robotposition(rospy.get_name())
    rospy.spin

    








