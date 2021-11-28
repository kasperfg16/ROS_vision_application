# ROS_vision_application

Repo containing program that handles motion planning and execution of ur5 manipulators. The program is used in 5. semester project at AAU by group 364 - 2021

## Disclaimer

- This code is only tested and developed for Ubuntu 20.04 using ROS1 and MoveIt1. All other configurations is used at own risk and suffering ;-)

## How to setup and run a demo in rviz

1. Install ROS1

    <http://wiki.ros.org/noetic/Installation/Ubuntu>

2. Install Universal_Robots_ROS_Driver

    <https://github.com/UniversalRobots/Universal_Robots_ROS_Driver>

3. In a termial:

    a) Install MoveIt1

    ```bash
    sudo apt install ros-noetic-moveit
    ```

    b) Build the catkin workspace

    ```bash
    cd p5_project_group_364/

    catkin_make
    ```

    ! Note:\
    **There have been problems using `catkin_make` this problem can be solved with `catkin_make:isolated`.** \
    !

    c) Source the workspace

    ```bash
    source devel/setup.bash
    ```

    ! Note:\
    **If `catkin_make:isolated` was used use `source devel_isolated/setup.bash` instead.** \
    !

    d) Launch the demo workspace

    ```bash
    roslaunch vision_application_movit_config_2_0 demo.launch
    ```

4. In  ANOTHER termial:

    a) Run a path

    ```bash
    /bin/python3 /home/ubuntu/p5_project_group_364/src/Move_Group_Python_Interface.py
    ```

## How to use with real UR robots

1. In a termial:

    a) cd the workspace

    ```bash
    cd p5_project_group_364/
    ```

    b) Source the workspace

    ```bash
    source devel/setup.bash
    ```

    ! Note:\
    **If `catkin_make:isolated` was used use `source devel_isolated/setup.bash` instead.** \
    !

    c) Run the driver

    ```bash
    roslaunch ur_robot_driver vision_application_bringup.launch robot_ip:=INSERT_ROBOT_IP_HERE
    ```

2. In  ANOTHER termial:

    a) cd the workspace

    ```bash
    cd p5_project_group_364/
    ```

    b) Source the workspace

    ```bash
    source devel/setup.bash
    ```

    ! Note:\
    **If `catkin_make:isolated` was used use `source devel_isolated/setup.bash` instead.** \
    !

    c) Run the moveit_planning_execution.launch file

    ```bash
    roslaunch vision_application_movit_config_2_0 vision_application_moveit_planning_execution.launch
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

    c) Run the moveit_planning_execution.launch file

    ```bash
    roslaunch vision_application_movit_config_2_0 moveit_rviz.launch rviz_config:=$(rospack find vision_application_movit_config_2_0)/launch/moveit.rviz
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

    c) Run the moveit_planning_execution.launch file

    ```bash
    python3 /home/ubuntu/p5_project_group_364/src/Move_Group_Python_Interface.py
    ```

## References

1. MoveIt

    <http://docs.ros.org/en/kinetic/api/moveit_tutorials/html/index.html>
