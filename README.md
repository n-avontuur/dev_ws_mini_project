# dev_ws_mini_project_src
This is a project for Avans hogeschool, mechatronica, minur robotics.
It's developt for ROS2 - distribution : Humble.

## Prep Project
For this project you should use Ubuntu 22.04 and ROS2 distribution Humble.\
To make this easy the only thing you need to do is Flash Ubuntu 22.04 on a sd and pc.
- sudo apt update & upgrade

Clone repository and start the dedicated install file, found in the folder Installation.\
RPI install : RPI-install.sh\
PC install : pc-install.sh


## Run Launch files

Run the following command
 - ~cd ros-ws (Our workspace)~
 - ~source install/setup.bash~
 - ~ros2 launch depthai_examples stereo_inertial_node.launch.py~


## Project folder structure

- ros-ws
-   ├── build
-   ├── install
-   └── src
    -  └── Cpp
        -   ├── CONTRIBUTING.md
        -   ├── LICENSE
        -   ├── rclcpp
        -   ├── rclpy
        -   └── README.md
    -  └── Py
        -   ├── mini_project\
        -   ├── resource\
        -   ├── test\
        -   ├── package.xm;
        -   ├── setup.cfg
        -   └── setup.py
