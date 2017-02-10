#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
# import tf


def arc((x, y), radius, a0, a1, subdivisions=20):
    if a0 > a1:
        s = -1.0
    else:
        s = 1.0
    return [(np.array([x + radius * np.cos(t), y + radius * np.sin(t), 0]),
             t + s * np.pi * 0.5)
            for t in np.linspace(a0, a1, subdivisions)]


def circle_path():
    return arc((0, 0), 1, 0, 2 * np.pi)


def f1_path():
    pi = np.pi
    wps = sum([arc((x, y), r, a0, a1) for (x, y, r, a0, a1) in [
        (2.0 / 3, 2.0 / 3, 1.0 / 3, 0, pi),
        (0, 0, 1.0 / 3, 0, -pi),
        (-2.0 / 3, 2.0 / 3, 1.0 / 3, 0, pi),
        (-0.5, -0.5, 0.5, pi, 1.5 * pi),
        (0.5, -0.5, 0.5, -0.5 * pi, 0)
    ]], [])

    return wps + [wps[0]]


def eight_path(alpha):
    pi = np.pi
    t = np.sin(alpha)
    r = t / (1 + t)
    l = 1.0 / (1 + t)
    wps = (arc((l, 0), r, -pi + alpha, pi - alpha) +
           arc((-l, 0), r, -alpha, -2 * pi + alpha))

    return wps + [wps[0]]


def path_msg(wps, center, radius, frame_id):

    msg = Path()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id

    pose_header = Header(frame_id=frame_id)

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

    rospy.init_node('publish_loop', anonymous=True)
    pub = rospy.Publisher('path', Path, queue_size=1, latch=True)
    center = np.array(rospy.get_param('~center'))
    radius = rospy.get_param('~radius')
    frame_id = rospy.get_param('~frame_id')
    t = rospy.get_param('~type')
    paths = {'f1': f1_path(),
             'eight': eight_path(np.pi * 0.25),
             'circle': circle_path()}
    msg = path_msg(paths.get(t, paths['f1']), center, radius, frame_id)
    pub.publish(msg)
    rospy.spin()
