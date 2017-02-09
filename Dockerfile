FROM mavros:kinetic
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
   libgeos-c1v5 \
   ros-kinetic-dynamic-reconfigure \
   python-pip \
   ros-kinetic-tf \
   ros-kinetic-tf2-ros \
   ros-kinetic-tf2-geometry-msgs

RUN pip install numpy shapely utm

COPY . src/mav_control

RUN catkin build
