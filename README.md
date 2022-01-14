# ROS_vision_application

Reposetory containing program that handles motion planning and motion execution of two ur5 manipulators and image capturing using a B&R Smart sensor, Light bar. The program is used in 5. semester project at AAU by group 564 - 2021

## Disclaimer

- This code is only tested and developed for Ubuntu 20.04 using ROS1 and MoveIt1 using two ur5 robots. Python3 was used. All other configurations may/will need a reconfiguration of the code and us done at own risk and suffering ;-)

## Dependencies

Dependency | Command to install
------- | -------
[NumPy](https://pypi.org/project/numpy/) | ```pip install numpy```
[python-Math](https://pypi.org/project/python-math/) | ```pip install python-math```
[MoveIt](https://moveit.ros.org/install/) | ```sudo apt install ros-noetic-moveit```
[net-tools](https://www.howtoinstall.me/ubuntu/18-04/net-tools/) | ```sudo apt install net-tools```
[rosbridge-suite](http://wiki.ros.org/rosbridge_suite) | ```sudo apt-get install ros-noetic-rosbridge-suite```

## How to setup

1. In a terminal install dependencies listed above

    ``` bash
    pip install numpy
    pip install python-math
    sudo apt install ros-noetic-moveit
    sudo apt install net-tools
    sudo apt-get install ros-noetic-rosbridge-suite
    ```

2. Install ROS1

    <http://wiki.ros.org/noetic/Installation/Ubuntu>

3. In a termial:

    a)

    Create a workspace folder with a PATH of your choise and go into the folder

    ``` bash
    mkdir <PATH>

    cd <PATH>
    ```

    b)

    Clone the Universal_Robots_ROS_Driver into the workspace folder

    Info about the driver is found here: <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver>

    ``` bash
    # Clone the driver
    git clone <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git> src/Universal_Robots_ROS_Driver

    # Clone fork of the description. This is currently necessary, until the changes are merged upstream.
    $ git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot

    # Install dependencies
    sudo apt update -qq
    rosdep update
    rosdep install --from-paths src --ignore-src -y
    ```

    Info about the driver is found here: <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver>

    c)

    Clone the reposetory into a folder

    ```bash
    cd <PATH>

    git clone <https://github.com/kasperfg16/ROS_vision_application.git>
    ```

    d)

    Build the catkin workspace

    ```bash
    cd <PATH>

    catkin_make
    ```

    e)

    Go through this tutorial with both UR5 robots NOTE:

    <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/install_urcap_cb3.md>

## How to run a demo in rviz (not on real robots)

1. In a terminal

    a)

    cd the workspace

    ``` bash
    cd <PATH>
    ```

    b)

    Source the workspace

    ```bash
    source devel/setup.bash
    ```

    c)

    Launch the demo workspace

    ```bash
    roslaunch vision_application_movit_config demo.launch
    ```

2. Follow the README in the reposetory:

    <https://github.com/BenMusak/564-Automated-Robotic-Vision-Light-Configuration>

## How to use with real UR robots

1. **For safety and for better performance** connect with wired connection to the same network as the UR5 robots and the B&R Smart Sensor as errors otherwise can occur connect

2. In a termial:

    a)

    Add route to the Smart Sensor webserver

    Make sure the B&R Smart Sensor have the ip adress: 192.168.87.210

    ```bash
    sudo route add -net 192.168.200.0 netmask 255.255.255.0 gw 192.168.87.210
    ```

    b)

    Go to the webadress:

    192.168.87.210:81/MappVision

    c)

    Go to gcam

    d)

    Close the webpage

    e)

    cd the workspace

    ```bash
    cd <PATH>/
    ```

    f)

    Source the workspace

    ```bash
    source devel/setup.bash
    ```

    g)

    Run the driver

    ```bash
    roslaunch ur_description vision_application_bringup.launch
    ```

3. In ANOTHER termial:

    a)

    cd the workspace

    ```bash
    cd <PATH>/
    ```

    b)

    Source the workspace

    ```bash
    source devel/setup.bash
    ```

    c)

    Run the moveit_planning_execution.launch file

    ``` bash
    roslaunch vision_application_movit_config vision_application_moveit_planning_execution.launch
    ```

4. In ANOTHER termial:

    a)

    cd the workspace

    ```bash
    cd <PATH>/
    ```

    b)

    Source the workspace

    ```bash
    source devel/setup.bash
    ```

    c) Run the moveit_planning_execution.launch file

    ```bash
    roslaunch vision_application_movit_config moveit_rviz.launch
    ```

5. In ANOTHER termial:

    a)

    cd the workspace

    ```bash
    cd <PATH>/
    ```

    b)

    Source the workspace

    ```bash
    source devel/setup.bash
    ```

    c)

    Run the rosbridge server.

    ```bash
    roslaunch rosbridge_server rosbridge_websocket.launch
    ```

6. On both the UR5s, run the **External Control** program that was created on both the UR5s by pressing the play button

    ![alt text](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/master/ur_robot_driver/doc/initial_setup_images/cb3_11_program_view_excontrol.png?raw=true)

7. In ANOTHER termial:

    a)

    cd the workspace

    ```bash
    cd <PATH>/
    ```

    b) now run the run_robot_handler.py Python script.

    ```bash
    source devel/setup.bash

    cd src/robot_handler/src/

    python3 run_robot_handler.py
    ```

8. Follow the README in the reposetory:

    <https://github.com/BenMusak/564-Automated-Robotic-Vision-Light-Configuration>

## How to control from local web-server

1.
    a)

    Start RVIZ up just like if you would run in on the robots or just simulation.
    But do not run the Move_Group_Python_Interface.py Python script!

    b)

    Install rosbridge suite.

    ```bash
    sudo apt-get install ros-noetic-rosbridge-suite
    ```

    c)

    Run the rosbridge server.

    ```bash
    roslaunch rosbridge_server rosbridge_websocket.launch
    ```

    a)

    cd the workspace in wich run_robot_handler.py is

    ```bash
    cd <PATH 2>/
    ```

    e)

    Now run the run_robot_handler.py Python script.

    ```bash
    source devel/setup.bash

    cd src/robot_handler/src/

    python3 run_robot_handler.py
    ```

## References

1. MoveIt

    <http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/index.html>

2. Universal_Robots_ROS_Driver

    <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver>
