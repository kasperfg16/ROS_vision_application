# CMake generated Testfile for 
# Source directory: /home/ubuntu/p5_project_group_364/src/Universal_Robots_ROS_Driver/ur_calibration
# Build directory: /home/ubuntu/p5_project_group_364/build_isolated/ur_calibration
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_ur_calibration_gtest_calibration_test "/home/ubuntu/p5_project_group_364/build_isolated/ur_calibration/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/ubuntu/p5_project_group_364/build_isolated/ur_calibration/test_results/ur_calibration/gtest-calibration_test.xml" "--return-code" "/home/ubuntu/p5_project_group_364/devel_isolated/ur_calibration/lib/ur_calibration/calibration_test --gtest_output=xml:/home/ubuntu/p5_project_group_364/build_isolated/ur_calibration/test_results/ur_calibration/gtest-calibration_test.xml")
set_tests_properties(_ctest_ur_calibration_gtest_calibration_test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;98;catkin_run_tests_target;/opt/ros/noetic/share/catkin/cmake/test/gtest.cmake;37;_catkin_add_google_test;/home/ubuntu/p5_project_group_364/src/Universal_Robots_ROS_Driver/ur_calibration/CMakeLists.txt;64;catkin_add_gtest;/home/ubuntu/p5_project_group_364/src/Universal_Robots_ROS_Driver/ur_calibration/CMakeLists.txt;0;")
subdirs("gtest")
