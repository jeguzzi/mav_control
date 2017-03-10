#!/usr/bin/env python
import rospy
from mav_control.mav_path_follower import MAVPathFollower
from mav_control.controller import Controller


if __name__ == '__main__':
    rospy.init_node('single_mav_controller', anonymous=True)
    Controller(MAVPathFollower)
    rospy.spin()
