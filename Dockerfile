FROM mavros_optitrack
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
   libgeos-c1v5 \
   ros-kinetic-dynamic-reconfigure

RUN pip install numpy shapely utm

COPY . /home/root/catkin_ws/src/mav_control
RUN /bin/bash -c '. /opt/ros/kinetic/setup.bash; catkin_make -C /home/root/catkin_ws;'
