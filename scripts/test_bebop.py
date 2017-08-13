#!/usr/bin/env python

import rospy

from mav_control.test_suite_bebop import BebopControlSuite


if __name__ == '__main__':
    BebopControlSuite()
    rospy.spin()
