#!/usr/bin/env python

import rospy

from mav_control.test_suite_mav import MAVControlSuite


if __name__ == '__main__':
    MAVControlSuite()
    rospy.spin()
