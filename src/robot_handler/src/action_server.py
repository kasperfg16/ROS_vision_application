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
        print("Position: \n x: %s \n x: %s \n y: %s \n z: %s \n rotx: %s \n roty: %s \n rotz: %s ", self._goal.x, self._goal.y, self._goal.z, self._goal.rotx, self._goal.roty, self._goal.rotz)
        print("x: ", self._goal.x)
        print("y: ", self._goal.y)
        print("z: ", self._goal.z)
        print("rotx: ", self._goal.rotx)
        print("roty: ", self._goal.roty)
        print("rotz: ", self._goal.rotz)

        if success:
            rospy.loginfo('%s: Succeeded' % self._action_name)

        return self._goal.x, self._goal.y, self._goal.z, self._goal.rotx, self._goal.roty, self._goal.rotz,

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


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

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
        ur5_cam_name = "ur5_light_bar"
        group = moveit_commander.MoveGroupCommander(ur5_cam_name)

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

    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        # BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        # Planning to a Joint Goal
        # ^^^^^^^^^^^^^^^^^^^^^^^^
        # The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ so the first
        # thing we want to do is move it to a slightly better configuration.
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -pi/4
        joint_goal[2] = 0
        joint_goal[3] = -pi/2
        joint_goal[4] = 0
        joint_goal[5] = pi/3
        joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        group.stop()

        # END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_joints = self.group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

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

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        # BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        # Cartesian Paths
        # ^^^^^^^^^^^^^^^
        # You can plan a Cartesian path directly by specifying a list of waypoints
        # for the end-effector to go through:
        ##
        waypoints = []

        wpose = group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # eef_step
            0.0)         # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        # END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        # BEGIN_SUB_TUTORIAL display_trajectory
        ##
        # Displaying a Trajectory
        # ^^^^^^^^^^^^^^^^^^^^^^^
        # You can ask RViz to visualize a plan (aka trajectory) for you. But the
        # group.plan() method does this automatically so this is not that useful
        # here (it just displays the same trajectory again):
        ##
        # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        # We populate the trajectory_start with our current robot state to copy over
        # any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        # END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        group = self.group

        # BEGIN_SUB_TUTORIAL execute_plan
        ##
        # Executing a Plan
        # ^^^^^^^^^^^^^^^^
        # Use execute if you would like the robot to follow
        # the plan that has already been computed:
        group.execute(plan, wait=True)

        # **Note:** The robot's current joint state must be within some tolerance of the
        # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        # END_SUB_TUTORIAL

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        # BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        # Ensuring Collision Updates Are Receieved
        # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        # If the Python node dies before publishing a collision object update message, the message
        # could get lost and the box will not appear. To ensure that the updates are
        # made, we wait until we see the changes reflected in the
        # ``get_known_object_names()`` and ``get_known_object_names()`` lists.
        # For the purpose of this tutorial, we call this function after adding,
        # removing, attaching or detaching an object in the planning scene. We then wait
        # until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        # END_SUB_TUTORIAL


def startHemisPath(tutorial, veiwPoint):
    # The goal here is to autogenerate a path in the shape of an hemisphere (with the object in the center),
    # that the robot can follow. We can then change the size of the hemisphere to
    # test from different distances. Furthermore, the lights should always point at the object,
    # So in some way we must change the Rotation matrix of the tcp, so that the z-axis always points to
    # the center of the hemisphere.

    a = 5  # Size of hemisphere (half sphere)
    # we find out what the max x-coordinate is (radius)
    max_x = sqrt(a-pow(0, 2)-pow(0, 2))
    max_y = max_x  # Since we are dealing with an hemisphere
    # Min_x must therefore be the oposite of max_x (asuming that the center of the hemisphere is y=0)
    min_y = -max_y
    min_x = -max_x
    xyz = []  # List for only XYZ coordinates.
    # List that contains the XYZ Coordinate and Roll Pitch Yaw Coordinates.
    i = 0  # Used for indexing xyzrpw list.

    # Reference frame (Center of Hemisphere)
    #[500, 160, 155]
    inspection_center_xyz = veiwPoint
    step = 0.5

    rpy = []

    # Generating the 3D points for an hemisphere
    # We iterate over all y-coordinate before we change to new x-coordinate, thereafter
    # we also change the polarity of the min_y, max_y and step variables.
    # This results in a back and forth movement
    # Example: "For the first x-coordinate, the y-coordinates start at negative and end at positive.
    # For the second x-coordinate, the y-coordinates start at positive and end at negative."
    #
    # TODO:
    #   - The Z axis of the tcp should always point at the center of the hemisphere.
    #       - I have tried to do this by finding the angle between the center of the hemisphere and
    #         the point on an arbitrary point on the hemisphere, but so far this can only give me
    #         the pitch and yaw angle and not roll. Furthermore, the found pitch and yaw seems to not
    #         match with what it is supposed to be.
    #
    for x_it in np.arange(max_x, min_x, -0.1):
        for y_it in np.arange(min_y, max_y, step):
            if sqrt(pow(x_it, 2) + pow(y_it, 2)) <= a:
                try:
                  # Computing the x, y and z coordinates of the points in the hemisphere wrt. to the robot frame
                  # (we multiply with 50 just to make the hemisphere a bit larger)
                  z_hemsphe = sqrt(a-pow(x_it,2)-pow(y_it,2))*50+inspection_center_xyz[2]
                  # We add 500 to move the hemisphere a bit away from origon.
                  x_hemsphe = x_it*50 + inspection_center_xyz[0]
                  y_hemsphe = y_it*50 + inspection_center_xyz[1]

                  # Compute roll, pitch and yaw of the camera with fixed angles wrt. the robot frame.
                  # vectors from camera frame to inspection frame
                  x_vect = inspection_center_xyz[0] - x_hemsphe
                  y_vect = inspection_center_xyz[1] - y_hemsphe
                  z_vect = inspection_center_xyz[2] - z_hemsphe

                  if x_vect < 0 and z_vect > 0:
                      rot_y = atan(z_vect/x_vect)/pi*180
                  elif x_vect > 0 and z_vect > 0:
                      rot_y = atan(x_vect/z_vect)/pi*180
                  elif x_vect > 0 and z_vect < 0:
                      rot_y = -atan(z_vect/x_vect)/pi*180+180
                  elif x_vect < 0 and z_vect < 0:
                      rot_y = atan(x_vect/z_vect)/pi*180+270
                  else:
                      print(
                          "computiation fail in y angle between camera and inspection object")

                  if y_vect > 0 and z_vect < 0:
                      rot_x = atan(y_vect/(sqrt(pow(z_vect, 2)+pow(x_vect, 2))))/pi*180-90
                  elif y_vect < 0 and z_vect < 0:
                      rot_x = - atan((sqrt(pow(z_vect, 2)+pow(x_vect, 2)))/y_vect)/pi*180-180
                  elif y_vect < 0 and z_vect > 0:
                      rot_x = atan(z_vect/y_vect)/pi*180
                  elif y_vect > 0 and z_vect > 0:
                      rot_x = atan(y_vect/z_vect)/pi*180
                  else:
                      print(
                          "computiation fail in x angle between camera and inspection object")

                  pathPoints = transl(
                      x_hemsphe, y_hemsphe, z_hemsphe)*rotz(0*pi/180)*roty(0*pi/180)*rotx(0*pi/180)

                  xyz.append(
                      [pathPoints[0, 3], pathPoints[1, 3], pathPoints[2, 3]])

                  # Fixed rotation
                  rot_y1 = rot_y - 90

                  # Euler rotation
                  rot_x2 = rot_x + 90
                  
                  rpy.append([rot_x2,rot_y1,0])

                  i += 1
                  # Using the double for loop actually results is us trying to find values that exceed
                  # the hemipshere, the try except func prevents the program from stopping, when this happens.
                except:
                  print("compute error")
        max_y = -max_y
        min_y = -min_y
        step = -step

    print("Number of points in hemossphere: ",len(xyz))
    with open('Hemisphere.csv', 'w', encoding='UTF8') as f:
        writer = csv.writer(f)
        for pos in xyz:
            writer.writerow(pos)

    print("path generated")

    for i in range(len(xyz)):
        x = xyz[i][0] * 0.001
        y = xyz[i][1] * 0.001
        z = xyz[i][2] * 0.001
        rx = -np.deg2rad(rpy[i][0])
        ry = np.deg2rad(rpy[i][1])
        rz = np.deg2rad(rpy[i][2])
        tutorial.go_to_pose_goal(x, y, z, rx, ry, rz)
def main():
    try:
        posBacklight = [0.370, 0.160, 0]
        veiwPoint = [370, 160, 155]
        # tutorial = MoveGroupPythonIntefaceTutorial()
        #tutorial.add_static_scene(posBacklight)
        input()
        print("Starting hemissphere path")
        #startHemisPath(tutorial, veiwPoint)

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == '__main__':
    rospy.init_node('Custom_Python_Script')
    server = robotposition(rospy.get_name())
    rospy.spin

    main()
    








