#!/usr/bin/env python

import rospy

from mav_control.test_suite import ControlSuite


if __name__ == '__main__':
    ControlSuite()
    rospy.spin()
