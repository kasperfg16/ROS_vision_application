# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/opt/ros/noetic/share/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/opt/ros/noetic/share/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/ubuntu/p5_project/devel_isolated/ur_calibration;/home/ubuntu/p5_project/devel_isolated/ur_robot_driver;/home/ubuntu/p5_project/devel_isolated/ur_kinematics;/home/ubuntu/p5_project/devel_isolated/ur_gazebo;/home/ubuntu/p5_project/devel_isolated/ur5_cam_moveit_config;/home/ubuntu/p5_project/devel_isolated/ur_description;/home/ubuntu/p5_project/devel_isolated/ur_dashboard_msgs;/home/ubuntu/p5_project/devel_isolated/universal_robots;/home/ubuntu/p5_project/devel_isolated/controller_stopper;/opt/ros/noetic'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/ubuntu/p5_project/devel_isolated/ur_description/env.sh')

output_filename = '/home/ubuntu/p5_project/build_isolated/ur_description/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
