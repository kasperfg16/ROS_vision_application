# CMake generated Testfile for 
# Source directory: /home/ubuntu/p5_project/src/fmauch_universal_robot/ur_description
# Build directory: /home/ubuntu/p5_project/build_isolated/ur_description
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_ur_description_roslaunch-check_tests_roslaunch_test_ur5.xml "/home/ubuntu/p5_project/build_isolated/ur_description/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/ubuntu/p5_project/build_isolated/ur_description/test_results/ur_description/roslaunch-check_tests_roslaunch_test_ur5.xml.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/ubuntu/p5_project/build_isolated/ur_description/test_results/ur_description" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/ubuntu/p5_project/build_isolated/ur_description/test_results/ur_description/roslaunch-check_tests_roslaunch_test_ur5.xml.xml\" \"/home/ubuntu/p5_project/src/fmauch_universal_robot/ur_description/tests/roslaunch_test_ur5.xml\" ")
set_tests_properties(_ctest_ur_description_roslaunch-check_tests_roslaunch_test_ur5.xml PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/ubuntu/p5_project/src/fmauch_universal_robot/ur_description/CMakeLists.txt;10;roslaunch_add_file_check;/home/ubuntu/p5_project/src/fmauch_universal_robot/ur_description/CMakeLists.txt;0;")
subdirs("gtest")
