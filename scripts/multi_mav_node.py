#!/usr/bin/env python
import rospy
from mav_control.mav_path_follower import MAVPathFollower
from mav_control.controller import MultiController


if __name__ == '__main__':
    rospy.init_node('multi_mav_controller', anonymous=True)
    MultiController(MAVPathFollower)
    rospy.spin()
