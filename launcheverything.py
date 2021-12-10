import os
import subprocess
import time

#path = " cd Dokumenter/ROB5_project/ROS_vision_application"

# os.path.join(os.getenv(''))
path = os.path.abspath("launcheverything.py")

for root, dirs, files in os.walk("/home"):
    for name in files:
        if name == "launcheverything.py":
            path = str(root)
            print(path)

for root, dirs, files in os.walk("/home"):
    for runserver in files:
        if runserver == "runserver.py":
            path2 = os.path.join(root,runserver)
            print(str(path2))

subprocess.Popen("source " + path + "/devel/setup.bash", shell=True, executable='/bin/bash')
os.system('cd ' + str(path))
subprocess.Popen("roslaunch ur_description vision_application_bringup.launch", shell=True)
time.sleep(7)
subprocess.Popen("roslaunch vision_application_movit_config vision_application_moveit_planning_execution.launch", shell=True)
subprocess.Popen("roslaunch vision_application_movit_config moveit_rviz.launch rviz_config:=$(rospack find vision_application_movit_config)/launch/moveit.rviz", shell=True)
subprocess.Popen("roslaunch rosbridge_server rosbridge_websocket.launch", shell=True)
subprocess.Popen("python3 " + path + "/run_robot_handler.py", shell=True)

subprocess.Popen("python3 " + str(path2), shell=True)

