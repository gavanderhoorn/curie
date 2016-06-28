# davetcoleman/curie:curie-jade

FROM moveit/moveit_docker:moveit-jade-source
MAINTAINER Dave Coleman dave@dav.ee

ENV CATKIN_WS=/root/ws_moveit
WORKDIR $CATKIN_WS/src

# delete old source code, download curie source
RUN cd .. && rm -rf src/ && mkdir src && cd src && \
    wstool init && \
    wstool merge https://raw.githubusercontent.com/davetcoleman/curie/kinetic-devel/curie.rosinstall && \
    wstool update

# disable packages that do not have proper dependencies
RUN touch robotiq/robotiq_action_server/CATKIN_IGNORE && \
    touch robotiq/robotiq_c_model_control/CATKIN_IGNORE && \
    touch robotiq/robotiq_ethercat/CATKIN_IGNORE && \
    touch robotiq/robotiq_s_model_control/CATKIN_IGNORE && \
    touch universal_robot/ur_gazebo/CATKIN_IGNORE && \
    touch roscon_2015/plan_and_run/CATKIN_IGNORE

# update apt-get because osrf image clears this cache and download deps
RUN apt-get update && \
    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} && \
    apt-get remove -y ros-${ROS_DISTRO}-ompl && \
    rm -rf /var/lib/apt/lists/*

# build repo
WORKDIR $CATKIN_WS
RUN source /ros_entrypoint.sh && \
    catkin build --no-status
