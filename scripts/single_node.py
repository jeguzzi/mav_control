#!/usr/bin/env python
import rospy
from mav_control.path_follower import PathFollower
from mav_control.controller import Controller


if __name__ == '__main__':
    rospy.init_node('single_controller', anonymous=True)
    Controller(PathFollower)
    rospy.spin()
