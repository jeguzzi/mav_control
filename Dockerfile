FROM ros
MAINTAINER Jerome Guzzi "jerome@idsia.ch"


RUN apt-get update && apt-get install -y \
   git build-essential python-rosdep python-catkin-tools \
   libgeos-c1v5 \
   ros-kinetic-dynamic-reconfigure \
   python-pip \
   ros-kinetic-mavros-msgs \
   ros-kinetic-tf \
   ros-kinetic-tf2-ros \
   ros-kinetic-tf2-geometry-msgs

RUN pip install numpy shapely utm



RUN mkdir -p /root/catkin_ws/src

WORKDIR /root/catkin_ws

COPY . src/mav_control

RUN /bin/bash -c 'source /opt/ros/kinetic/setup.bash; catkin init; catkin build'

RUN /bin/sed -i \
  '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/root\/catkin_ws\/devel\/setup.bash"' \
  /ros_entrypoint.sh
