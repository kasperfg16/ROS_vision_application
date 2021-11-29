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
        #self._as.set_succeeded()
        success = True

        #Get the received goal values.
        self._goal.x = goal.x
        self._goal.y = goal.y
        self._goal.z = goal.z
        self._goal.rotx = goal.rotx
        self._goal.roty = goal.roty
        self._goal.rotz = goal.rotz

        #Set values for the result variables that we want to return:
        self._result.x = goal.x
        self._result.y = goal.y
        self._result.z = goal.z
        self._result.rotx = goal.rotx
        self._result.roty = goal.roty
        self._result.rotz = goal.rotz

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
            self._feedback.robot_moved_str = "process succeeded"
            #Feedback should only be used to give the user periodic feedback, not to inform if the process has ended.
            self._as.publish_feedback(self._feedback)
            #Result is the real indication, that the process has ended.
            self._as.set_succeeded(self._result)    
        else:
            #This should of couse be changed at some point, so that the client receives usefull information
            #Such as using set_aborted instead of set_succeeded.
            self._feedback.robot_moved_str = "process failed"
            self._as.publish_feedback(self._feedback)
            self._as.set_succeeded(self._result)
            

        return self._goal.x, self._goal.y, self._goal.z, self._goal.rotx, self._goal.roty, self._goal.rotz,

if __name__ == '__main__':
    rospy.init_node('Custom_Python_Script')
    server = robotposition(rospy.get_name())
    rospy.spin

    








