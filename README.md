# ROS_vision_application

Repo containing program that handles motion planning and execution of ur5 manipulators. The program is used in 5. semester project at AAU by group 564 - 2021

## Disclaimer

- This code is only tested and developed for Ubuntu 20.04 using ROS1 and MoveIt1. All other configurations is used at own risk and suffering ;-)

## Dependencies

Dependency | Command to install
------- | -------
[NumPy](https://pypi.org/project/numpy/) | ```pip install numpy```
[python-Math](https://pypi.org/project/python-math/) | ```pip install python-math```
[MoveIt](https://moveit.ros.org/install/) | ```sudo apt install ros-noetic-moveit```

## How to setup and run a demo in rviz

1. Install ROS1

    <http://wiki.ros.org/noetic/Installation/Ubuntu>

2. Clone the Universal_Robots_ROS_Driver

    ``` bash
    git clone <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git> src/Universal_Robots_ROS_Driver
    ```

    Read info abaout the driver here: <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver>

3. In a termial:

    a) Clone the reposetory into a folder of your choise (<PATH>)

    ```bash
    cd <PATH>/

    git clone <https://github.com/kasperfg16/ROS_vision_application.git>
    ```

    a) Build the catkin workspace

    ```bash
    cd <PATH>

    catkin_make
    ```

## How to run a demo in rviz

1. In a terminal

    a) Source the workspace

    ```bash
    source devel/setup.bash
    ```

    c) Launch the demo workspace

    ```bash
    roslaunch vision_application_movit_config demo.launch
    ```

2. In  ANOTHER termial:

    a) cd the workspace

    ```bash
    cd <PATH>
    ```

    b) Run the pyhon script

    ```bash
    python3 src/Move_Group_Python_Interface.py
    ```

## How to use with real UR robots

1. Connect to the same network as the UR robots  - Preferably with wired connection as errors otherwise can occur

2. In a termial:

    a) cd the workspace

    ```bash
    cd p5_project_group_364/
    ```

    b) Source the workspace

    ```bash
    source devel/setup.bash
    ```

    ! Note:\
    **If `catkin_make_isolated` was used, use `source devel_isolated/setup.bash` instead.** \
    !

    c) Run the driver

    ```bash
    roslaunch ur_description vision_application_bringup.launch
    ```

3. In  ANOTHER termial:

    a) cd the workspace

    ```bash
    cd p5_project_group_364/
    ```

    b) Source the workspace

    ```bash
    source devel/setup.bash
    ```

    ! Note:\
    **If `catkin_make_isolated` was used, use `source devel_isolated/setup.bash` instead.** \
    !

    c) Run the moveit_planning_execution.launch file

    ```bash
    roslaunch vision_application_movit_config vision_application_moveit_planning_execution.launch
    ```

4. In  ANOTHER termial:

    a) cd the workspace

    ```bash
    cd p5_project_group_364/
    ```

    b) Source the workspace

    ```bash
    source devel/setup.bash
    ```

    ! Note:\
    **If `catkin_make_isolated` was used, use `source devel_isolated/setup.bash` instead.** \
    !

    c) Run the moveit_planning_execution.launch file

    ```bash
    roslaunch vision_application_movit_config moveit_rviz.launch rviz_config:=$(rospack find vision_application_movit_config)/launch/moveit.rviz
    ```

5. In  ANOTHER termial:

    a) cd the workspace

    ```bash
    cd p5_project_group_364/
    ```

    b) Source the workspace

    ```bash
    source devel/setup.bash
    ```

    c) Run the moveit_planning_execution.launch file

    ```bash
    python3 /home/ubuntu/p5_project_group_364/src/Move_Group_Python_Interface.py
    ```

## How to control from local web-server

1.
    a) Start RVIZ up just like if you would run in on the robots or just simulation.
    But do not run the Move_Group_Python_Interface.py Python script!

    b) Install rosbridge suite.

    ```bash
    sudo apt-get install ros-noetic-rosbridge-suite
    ```

    c) Run the rosbridge server.

    ```bash
    roslaunch rosbridge_server rosbridge_websocket.launch
    ```

    d) now run the run_robot_handler.py Python script.

    ```bash
    python run_robot_handler.py
    ```

## References

1. MoveIt

    <http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/index.html>

2. Universal_Robots_ROS_Driver

    <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver>
