#!/usr/bin/env python
import rospy
from mav_control.path_follower import PathFollower
from mav_control.controller import MultiController


if __name__ == '__main__':
    rospy.init_node('multi_controller', anonymous=True)
    MultiController(PathFollower)
    rospy.spin()
