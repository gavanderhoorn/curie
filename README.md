Curie
=========

ROS-Industrial Special Project: High DOF Planning for Assembly Processes

[Marie Skłodowska Curie](https://en.wikipedia.org/wiki/Marie_Curie) _Marie Skłodowska Curie (/ˈkjʊri, kjʊˈriː/;[2] French: [kyʁi]; Polish: [kʲiˈri]; 7 November 1867 – 4 July 1934), born Maria Salomea Skłodowska [ˈmarja salɔˈmɛa skwɔˈdɔfska], was a Polish and naturalized-French physicist and chemist who conducted pioneering research on radioactivity. She was the first woman to win a Nobel Prize, the first person and only woman to win twice, the only person to win twice in multiple sciences, and was part of the Curie family legacy of five Nobel Prizes. She was also the first woman to become a professor at the University of Paris, and in 1995 became the first woman to be entombed on her own merits in the Panthéon in Paris._

Further information on the Decartes project can be found on the ROS [wiki](http://wiki.ros.org/curie).  Those who are interested in contributing should look at the open [issues](https://github.com/ros-industrial-consortium/curie/issues) and then email the [ROS-Industrial developer group](mailto:swri-ros-pkg-dev@googlegroups.com) before performing any work.

## Docker

An [automated docker image](https://hub.docker.com/r/davetcoleman/curie/builds/) is available for testing builds, see ``curie/docker``. To use this, setup Docker per their website then run:

    docker run -it davetcoleman/curie:latest

## ROS Jade Installation

Note: for now we have to install MoveIt! from source because OMPL requires C++11 and Descartes requires the new IK funcitonality

This should also work on Indigo. Was tested on Ubuntu 14.04. To build this package in a new workspace:

    mkdir -p ws_moveit/src
    cd ws_moveit/src
    wstool init .
    wstool merge https://raw.githubusercontent.com/davetcoleman/moveit/kinetic-devel/moveit.rosinstall
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
