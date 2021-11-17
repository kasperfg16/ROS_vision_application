# ROS_vision_application

Repo containing program that handles motion planning and execution of ur5 manipulators. The program is used in 5. semester project at AAU by group 364 - 2021

## Disclaimer

- This code is only tested and developed for Ubuntu 20.04 using ROS1 and MoveIt1. All other configurations is used at own risk ;-)

## How to setup

1. Install ROS1

    <http://wiki.ros.org/noetic/Installation/Ubuntu>

2. In a termial:

    a) Install MoveIt1

    ```bash
    sudo apt install ros-noetic-moveit
    ```

    b) Build the catkinworkspace

    ```bash
    cd p5_project_group_364/

    catkin_make
    ```

    c) Source the workspace

    ```bash
    source devel/setup.bash
    ```

    d) Launch the demo workspace

    ```bash
    roslaunch vision_application_movit_config demo.launch
    ```
