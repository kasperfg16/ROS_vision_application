 #! /usr/bin/env python
from __future__ import print_function
from threading import main_thread

import copy
import sys
from math import atan, atan2, pi, sqrt
import numpy as np

import geometry_msgs.msg
import robot_handler.msg
import moveit_msgs.msg
from std_msgs.msg import String

import rospy
import rospy.rostime
import actionlib
import moveit_commander
from moveit_commander.conversions import pose_to_list
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.planning_scene_interface import PlanningSceneInterface

from robodk.robodk import *
from tf.transformations import *
import csv
from scipy.spatial.transform import Rotation as R


def all_close(goal, actual, tolerance):
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True

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

        super(robotposition, self).__init__()

        # BEGIN_SUB_TUTORIAL setup
        ##
        # First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_group_python_interface',
        #                anonymous=True)

        # Instantiate a `RobotCommander`_ object. This object is the outer-level interface to
        # the robot:
        robot = moveit_commander.RobotCommander()

        # Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
        # to the world surrounding the robot:
        scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        # to one group of joints.  In this case the group is the joints in the Panda
        # arm so we set ``group_name = panda_arm``. If you are using a different robot,
        # you should change this value to the name of your robot arm planning group.
        # This interface can be used to plan and execute motions on the Panda:
        ur5_robot_name = " "

        if self._goal.robot_name == "camera_robot":
            ur5_robot_name = "ur5_cam"
        if self._goal.robot_name == "lightbar_robot":
            ur5_robot_name = "ur5_light_bar"


        group = moveit_commander.MoveGroupCommander(ur5_robot_name)

        group.set_num_planning_attempts(100)
        # group.set_planner_id("geometric::AnytimePathShortening")

        # We create a `DisplayTrajectory`_ publisher which is used later to publish
        # trajectories for RViz to visualize:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        # END_SUB_TUTORIAL

        # BEGIN_SUB_TUTORIAL basic_info
        ##
        # Getting Basic Information
        # ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = group.get_planning_frame()
        print("============ Reference frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = group.get_end_effector_link()
        print("============ End effector: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Robot Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        # END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names



        
    def execute_cb(self, goal):
        r = rospy.Rate(1)
        self._as.set_succeeded()
        success = True

        self._goal.x = goal.x
        self._goal.y = goal.y
        self._goal.z = goal.z
        self._goal.rotx = goal.rotx
        self._goal.roty = goal.roty
        self._goal.rotz = goal.rotz

        # Print the position of recived goal
        print("Position: \n Robot: %s \n x: %s \n x: %s \n y: %s \n z: %s \n rotx: %s \n roty: %s \n rotz: %s " % 
        (self._goal.robot_name,
        self._goal.x, 
        self._goal.y, 
        self._goal.z, 
        self._goal.rotx, 
        self._goal.roty, 
        self._goal.rotz))

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)

        robotposition.go_to_pose_goal(
        self._goal.robot_name, 
        self._goal.x, 
        self._goal.y, 
        self._goal.z, 
        self._goal.rotx, 
        self._goal.roty, 
        self._goal.rotz)


    def add_static_scene(self, posBacklight):
        '''
        Adds collision objects to the planning scene
        '''

        planning_frame = self.planning_frame
        scene = self.scene

        # Add tables as collision object
        pose_tables = geometry_msgs.msg.PoseStamped()
        table_id = 'table'
        pose_tables.header.frame_id = planning_frame
        pose_tables.pose.position.x = 0.0
        pose_tables.pose.position.y = 0.0
        pose_tables.pose.position.z = 0.0
        table_size = (0.001, 0.001, 0.001)
        table_path_mesh = "./src/scene_meshes/table.stl"

        scene.add_mesh(table_id, pose_tables, table_path_mesh, table_size)

        # Add backlight as collision object
        pose_backlight = geometry_msgs.msg.PoseStamped()
        backlight_id = 'backlight'
        pose_backlight.header.frame_id = planning_frame
        pose_backlight.pose.position.x = posBacklight[0]
        pose_backlight.pose.position.y = posBacklight[1]
        pose_backlight.pose.position.z = posBacklight[2]

        # Convert from euler angles to quaterion
        rx = pi/2
        ry = 0
        rz = 0
        q_orig = quaternion_from_euler(0, 0, 0)
        q_rot = quaternion_from_euler(rx, ry, rz)
        q_new = quaternion_multiply(q_rot, q_orig)
        pose_backlight.pose.orientation.x = q_new[0]
        pose_backlight.pose.orientation.y = q_new[1]
        pose_backlight.pose.orientation.z = q_new[2]
        pose_backlight.pose.orientation.w = q_new[3]
        backlight_size = (0.001, 0.001, 0.001)
        backlight_path_stl = "./src/scene_meshes/backlight.stl"

        scene.add_mesh(backlight_id, pose_backlight,
                       backlight_path_stl, backlight_size)

    def go_to_pose_goal(self, x, y, z, rx, ry, rz):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        # BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        # Planning to a Pose Goal
        # ^^^^^^^^^^^^^^^^^^^^^^^
        # We can plan a motion for this group to a desired pose for the
        # end-effector:

        # Calculate quaternions from euler angles

        pose_goal = geometry_msgs.msg.Pose()
        q_rot = quaternion_from_euler(rx, ry, rz)
        pose_goal.orientation.x = q_rot[0]
        pose_goal.orientation.y = q_rot[1]
        pose_goal.orientation.z = q_rot[2]
        pose_goal.orientation.w = q_rot[3]
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        group.set_pose_target(pose_goal)

        # Now, we call the planner to compute the plan and execute it.
        plan = group.go(wait=True)
        
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()

        # END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)


def main():
    try:
        posBacklight = [0.370, 0.160, 0]
        RobPos = robotposition()
        RobPos.add_static_scene(posBacklight)
        input()

        RobPos.go_to_pose_goal(RobPos._goal.x, RobPos._goal.y, RobPos._goal.z, RobPos._goal.rotx, RobPos._goal.roty, RobPos._goal.rotz)

    

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    rospy.init_node('Custom_Python_Script')
    server = robotposition(rospy.get_name())
    rospy.spin

    main()
    








