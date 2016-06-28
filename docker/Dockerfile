# davetcoleman/curie:curie-jade

FROM moveit/moveit_docker:moveit-jade-source
MAINTAINER Dave Coleman dave@dav.ee

ENV CATKIN_WS=/root/ws_moveit
WORKDIR $CATKIN_WS/src

# download curie source
RUN wstool merge https://raw.githubusercontent.com/davetcoleman/curie/kinetic-devel/curie.rosinstall && \
    wstool update

# update apt-get because osrf image clears this cache and download deps
#RUN apt-get update && \
#    rosdep install -y --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} && \
#    rm -rf /var/lib/apt/lists/*

# build repo
#WORKDIR $CATKIN_WS
#RUN source /ros_entrypoint.sh && \
#    catkin build --no-status
