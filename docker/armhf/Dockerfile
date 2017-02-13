FROM jeguzzi/ros:armhf-kinetic-ros-dev
MAINTAINER Jerome Guzzi "jerome@idsia.ch"


RUN apt-get update && apt-get install -y \
   libgeos-c1v5 \
   python-pip \
   ros-kinetic-dynamic-reconfigure \
   ros-kinetic-mavros \
   ros-kinetic-tf \
   ros-kinetic-tf2-ros \
   ros-kinetic-tf2-geometry-msgs

RUN pip install numpy shapely utm

COPY  . src/mav_control

RUN catkin build
