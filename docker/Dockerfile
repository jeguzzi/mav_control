FROM jeguzzi/ros:kinetic-ros-dev
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
   libgeos-c1v5 \
   python-pip \
   ros-kinetic-dynamic-reconfigure \
   ros-kinetic-mavros \
   ros-kinetic-tf \
   ros-kinetic-tf2-ros \
   ros-kinetic-tf2-geometry-msgs \
   ros-kinetic-robot-state-publisher \
   ros-kinetic-xacro \
   && rm -rf /var/lib/apt/lists/*

RUN pip install numpy shapely utm

COPY mav_control src/mav_control
COPY dfw-models/erle_models src/erle_models

RUN git clone https://github.com/lrse/ros-keyboard.git src/ros-keyboard

RUN apt-get update && apt-get install -y \
   libsdl1.2-dev \
   && rm -rf /var/lib/apt/lists/*

RUN catkin build
