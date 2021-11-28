 #! /usr/bin/env python
from __future__ import print_function
from threading import main_thread
import rospy
import actionlib
import robot_handler.msg

class robotposition(object):
    # Uesd for feedback of result and goal if needed
    _feedback = robot_handler.msg.posFeedback()
    _result = robot_handler.msg.posResult()
    _goal = robot_handler.msg.posGoal()


    def __init__(self, name):
        self._action_name = name
        # Create and start the action server
        self._as = actionlib.SimpleActionServer(self._action_name, robot_handler.msg.posAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        
    def execute_cb(self, goal):
        r = rospy.Rate(1)
        self._as.set_succeeded()
        success = True

        self._goal_x = goal.x
        self._goal.y = goal.y
        self._goal.z = goal.z
        self._goal.rotx = goal.rotx
        self._goal.roty = goal.roty
        self._goal.rotz = goal.rotz

        # Print the position of recived goal
        print("Position: ")
        print("x: ", self._goal.x)
        print("y: ", self._goal.y)
        print("z: ", self._goal.z)
        print("rotx: ", self._goal.rotx)
        print("roty: ", self._goal.roty)
        print("rotz: ", self._goal.rotz)

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)

        return self._goal.x, self._goal.y, self._goal.z, self._goal.rotx, self._goal.roty, self._goal.rotz,

if __name__ == '__main__':
    rospy.init_node('Custom_Python_Script')
    server = robotposition(rospy.get_name())
    rospy.spin

    








