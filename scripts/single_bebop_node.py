#!/usr/bin/env python
import rospy
from mav_control.bebop_path_follower import BebopPathFollower
from mav_control.controller import Controller


if __name__ == '__main__':
    rospy.init_node('single_controller', anonymous=True)
    Controller(BebopPathFollower)
    rospy.spin()
