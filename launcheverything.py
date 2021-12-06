import os
import subprocess
import time

#path = " cd Dokumenter/ROB5_project/ROS_vision_application"

# os.path.join(os.getenv(''))
path = os.getcwd()
hh = os.system('cd ' + str(path))
print(str(path))

os.popen('roslaunch ur_description vision_application_bringup.launch')
time.sleep(7)
os.popen('roslaunch vision_application_movit_config vision_application_moveit_planning_execution.launch')
os.popen('roslaunch vision_application_movit_config moveit_rviz.launch rviz_config:=$(rospack find vision_application_movit_config)/launch/moveit.rviz')
os.popen('roslaunch rosbridge_server rosbridge_websocket.launch')
os.popen( 'Path to python file')