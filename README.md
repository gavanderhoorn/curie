Curie
=========

ROS-Industrial Special Project: High DOF Planning for Assembly Processes

About [Marie Skłodowska Curie](https://en.wikipedia.org/wiki/Marie_Curie)

Further information on the Decartes project can be found on the ROS [wiki](http://wiki.ros.org/curie).  Those who are interested in contributing should look at the open [issues](https://github.com/ros-industrial-consortium/curie/issues) and then email the [ROS-Industrial developer group](mailto:swri-ros-pkg-dev@googlegroups.com) before performing any work.

## Docker

An [automated docker image](https://hub.docker.com/r/davetcoleman/curie/builds/) is available for testing builds, see ``curie/docker``. To use this, setup Docker per their website then run:

    docker run -it davetcoleman/curie:latest

## ROS Jade Installation

Tested on Ubuntu 14.04. To build this package in a new workspace:

    sudo apt-get remove ompl
    mkdir -p ws_moveit/src
    cd ws_moveit/src
    wstool init .
    wstool merge https://raw.githubusercontent.com/davetcoleman/curie/kinetic-devel/curie.rosinstall
    wstool update
    touch robotiq/robotiq_action_server/CATKIN_IGNORE
    touch robotiq/robotiq_c_model_control/CATKIN_IGNORE
    touch robotiq/robotiq_ethercat/CATKIN_IGNORE
    touch robotiq/robotiq_s_model_control/CATKIN_IGNORE
    touch universal_robot/ur_gazebo/CATKIN_IGNORE
    touch roscon_2015/plan_and_run/CATKIN_IGNORE
    rosdep install --from-paths . --ignore-src --rosdistro jade
    cd ..
    catkin build
