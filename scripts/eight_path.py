#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import tf


def eight_path(center, radius):

    msg = Path()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "World"
    a = np.sqrt(2)/2
    # define an 8 in the x-y plane

    #wps = [(np.array([np.cos(t), np.sin(t), 0]), t - np.pi*0.5) for t in np.linspace(np.pi,-np.pi,20)]
    #wps = wps + [(np.array([- 1 + np.cos(t), np.sin(t), 0]), t + np.pi*0.5) for t in np.linspace(0,2*np.pi,20)]


    wps = ([(np.array([0.666 + 0.333*np.cos(t), 0.666 + 0.333*np.sin(t), 0]), 0) for t in np.linspace(0,np.pi,20)] +
           [(np.array([0 + 0.333*np.cos(t), 0.0 + 0.333*np.sin(t), 0]), 0) for t in np.linspace(0,-np.pi,20)] +
           [(np.array([-0.666 + 0.333*np.cos(t), 0.666 + 0.333*np.sin(t), 0]), 0) for t in np.linspace(0,np.pi,20)] +
           [(np.array([-0.5 + 0.5*np.cos(t), -0.5 + 0.5*np.sin(t), 0]), 0) for t in np.linspace(np.pi,1.5*np.pi,20)] +
           [(np.array([0.5 + 0.5*np.cos(t), -0.5 + 0.5*np.sin(t), 0]), 0) for t in np.linspace(-0.5*np.pi,0,20)] +
           [(np.array([1,0.666,0]),0)])
    # wps = [
    #     (np.array([0, 0, 0]), np.pi / 2),
    #     (np.array([1 - a, a, 0]), np.pi / 4),
    #     (np.array([1, 1, 0]), 0),
    #     (np.array([1 + a, a, 0]), -np.pi / 4),
    #     (np.array([2, 0, 0]), -np.pi / 2),
    #     (np.array([1 + a, -a, 0]), -np.pi* 3.0 / 4),
    #     (np.array([1, -1, 0]), -np.pi),
    #     (np.array([1 - a, -a, 0]), np.pi * 3.0 / 4),
    #     (np.array([0, 0, 0]), np.pi / 2),
    #     (np.array([-1 + a, a, 0]), np.pi * 3.0 / 4),
    #     (np.array([-1, 1, 0]), np.pi),
    #     (np.array([-1 - a, a, 0]), - np.pi * 3 / 4),
    #     (np.array([-2, 0, 0]), -np.pi / 2),
    #     (np.array([-1 - a, -a, 0]), -np.pi / 4),
    #     (np.array([-1, -1, 0]), 0),
    #     (np.array([-1 + a, -a, 0]), np.pi / 4),
    #     (np.array([0, 0, 0]), np.pi / 2)
    # ]

    pose_header = Header(frame_id='World')

    msg.poses = [PoseStamped(
        header=pose_header,
        pose=Pose(
            position=Point(*(center + radius * point)),
            orientation=Quaternion(0, 0, np.sin(
                angle * 0.5), np.cos(angle * 0.5))
        )
    )
        for (point, angle) in wps]

    return msg

if __name__ == '__main__':

    rospy.init_node('eight_path', anonymous=True)
    pub = rospy.Publisher('path', Path, queue_size=1, latch=True)
    center = np.array(rospy.get_param('~center'))
    radius = rospy.get_param('~radius')
    msg = eight_path(center, radius)
    pub.publish(msg)
    rospy.spin()
