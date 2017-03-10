from __future__ import print_function
import rospy
import numpy as np
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


def loop(_type, center, radius, frame_id, **kwargs):
    shape = {'f1': f1_path,
             'eight': eight_path,
             'circle': circle_path}
    if _type not in shape:
        return None
    try:
        return path_msg(shape[_type](**kwargs), center, radius, frame_id)
    except Exception as e:
        print('error creating loop: %s' % e)
        return None


def arc(center, radius, a0, a1, subdivisions=20):
    x, y = center
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


def eight_path(alpha=(np.pi * 0.5)):
    pi = np.pi
    t = np.sin(alpha)
    r = t / (1 + t)
    center = 1.0 / (1 + t)
    wps = (arc((center, 0), r, -pi + alpha, pi - alpha) +
           arc((-center, 0), r, -alpha, -2 * pi + alpha))

    return wps + [wps[0]]


def path_msg(wps, center, radius, frame_id):

    msg = Path()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame_id

    pose_header = Header(frame_id=frame_id)

    msg.poses = [
        PoseStamped(
            header=pose_header,
            pose=Pose(position=Point(*(center + radius * point)),
                      orientation=Quaternion(0, 0, np.sin(angle * 0.5), np.cos(angle * 0.5))))
        for point, angle in wps]

    return msg
