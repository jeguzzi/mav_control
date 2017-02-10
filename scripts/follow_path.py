#!/usr/bin/env python
import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Pose, Quaternion, Point
from nav_msgs.msg import Path
import numpy as np
#import quaternion
from shapely.geometry import LineString, Point, MultiPoint
from mavros_msgs.msg import GlobalPositionTarget
# TODO proof of concept for 2D paths
from dynamic_reconfigure.server import Server
from mav_control.cfg import PathFollowerConfig


def _a(point):
    return [point.x, point.y]


def _q(o):
    return [o.x, o.y, o.z, o.w]

wgs84_a = 6378137.0
wgs84_b = 6356752.3142


def piece(curve, s0, s1, loop, cs):
    print 'A', s0,s1
    if s0 < 0:
        if loop:
            s0 = s0 + curve.length
        else:
            s = 0
    if s1 > curve.length:
        if loop:
            s1 = s1 - curve.length

    p0 = curve.interpolate(s0)
    p1 = curve.interpolate(s1)
    # pi = curve.interpolate(0.5 * (s0 + s1))
    i0 = np.argmax(cs > s0)
    i1 = np.argmax(cs > s1)
    print 'B', s0, s1
    print i0, i1
    if i0 < i1:
        coords = list(curve.coords[(i0+1):i1])
    else:
        coords = list(curve.coords[(i0+1):]) + list(curve.coords[:i1])

    return LineString([p0] + coords + [p1])


    # print curve.difference(
    #     MultiPoint([p0, p1]))
    #
    # ci = [c for c in curve.difference(
    #     MultiPoint([p0, p1])) if c.contains(pi)]
    #
    # if ci:
    #     return ci[0]
    # else:
    #     raise NameError("Could not cut curve")


def project(point, curve, old_s, max_delta, loop, cs):
    if old_s is None:
        return curve.project(point)
    curve_piece = piece(curve, old_s - max_delta, old_s + max_delta, loop, cs)
    #print curve_piece.wkt
    print curve_piece.length
    s = curve_piece.project(point)
    print 'point', point,'s', s
    if s > 0 and s < curve_piece.length:
        s = s + old_s - max_delta
        if s > curve.length:
            s = s - curve.length
        return s
    else:
        return curve.project(point)


class PathFollower(object):
    """docstring for PathFollower"""

    def __init__(self):
        super(PathFollower, self).__init__()

        rospy.init_node('follow_path', anonymous=True)

        self.max_delta = 1
        self.s = None
        self.curve = None
        self.delta = rospy.get_param("~delta", 0.5)
        self.horizon = rospy.get_param("~distance", 1.5)
        self.loop = rospy.get_param("~loop", True)
        origin_ = rospy.get_param("origin", {})
        lat = origin_.get("latitude", 0)
        lon = origin_.get("longitude", 0)
        self.origin = np.array([lon, lat])
        self.alt = origin_.get("altitude", 0)
        self.angle = angle = origin_.get("heading", 0) * np.pi / 180.0

        # For now, let the earth be modelled as a circle with radius WGS84_a

        nx = 1.0 / (wgs84_a * np.cos(lat * np.pi / 180.0) * np.pi / 180.0)
        ny = 1.0 / (wgs84_b * np.pi / 180.0)

        self.local2latlon = np.matrix([[np.cos(angle) * nx, -np.sin(angle) * nx],
                                       [np.sin(angle) * ny, np.cos(angle) * ny]])

        rospy.Subscriber("pose", PoseStamped, self.has_updated_pose)
        rospy.Subscriber("path", Path, self.has_updated_path)
        self.pub = rospy.Publisher(
            "target", GlobalPositionTarget, queue_size=1)
        self.pub_pose = rospy.Publisher(
            "target_pose", PoseStamped, queue_size=1)

        srv = Server(PathFollowerConfig, self.reconfigure)

        rospy.spin()

    def reconfigure(self, config, level):
        self.delta = config.get('delta', self.delta)
        self.horizon = config.get('distance', self.horizon)
        return config

    def has_updated_path(self, msg):
        self.curve = LineString([_a(pose.pose.position) for pose in msg.poses])
        self.ps = np.array(self.curve)
        self.ls = np.linalg.norm(np.diff(self.ps, axis=0), axis=1)
        self.cs = np.cumsum(self.ls)
        self.yaws = [np.arctan2(
            pose.pose.orientation.z, pose.pose.orientation.w) * 2.0 for pose in msg.poses]
        # print self.curve
        # print self.ls
        # print 'YAWS', self.yaws
        # print 'QS', self.qs

    def target_pose(self, pose):

        current_point = np.array(_a(pose.position))

        #s = self.curve.project(Point(_a(pose.position)))

        print 'old s', self.s

        self.s = project(Point(current_point), self.curve,
                         self.s, self.max_delta, self.loop, self.cs)
        print 'new s', self.s

        s = self.s + self.delta

        # Make clear why I'm not using interpolate here!
        # [Why i'm also computing the angle]

        # print ':s', s
        # print self.cs[-1]

        if s > self.cs[-1]:
            if self.loop:
                s = s - self.cs[-1]
            else:
                s = self.cs[-1]

        # print 's', s

        i0 = np.argmax(self.cs > s)
        i1 = (i0 + 1) % len(self.ps)
        im1 = (i0 - 1 + len(self.ps)) % len(self.ps)

        # print i0, i1, im1

        if im1 == len(self.cs):
            ds = s
        else:
            ds = s - self.cs[im1]

        n = self.ls[i0]

        a = ds / n

        # print ds, n, a

        point = (1 - a) * self.ps[i0] + a * self.ps[i1]

        # quaternion = (self.qs[i0] *
        #               np.power(self.qs[i0].inverse() * self.qs[i1], a)
        #              )

        yaw0 = self.yaws[i0]
        yaw1 = self.yaws[i1]
        yaw = np.arctan2(
            np.sin(yaw0) * (1 - a) + np.sin(yaw1) * a,
            np.cos(yaw0) * (1 - a) + np.cos(yaw1) * a)

        dp = (point - current_point)
        dpn = np.linalg.norm(dp)
        dp = dp / dpn
        point = current_point + max(self.horizon, dpn) * dp

        # print yaw0, yaw1, yaw

        msg = PoseStamped(
            header=Header(frame_id='World'),
            pose=Pose(
                position=Point(point[0], point[1], 0),
                orientation=Quaternion(0, 0, np.sin(
                    yaw * 0.5), np.cos(yaw * 0.5))
            )
        )

        return msg

    def pose2wgs84(self, point, yaw):
        return (self.local2latlon.dot(point) + self.origin, yaw + self.angle - np.pi * 0.5)

    def publish_target_pose(self, pose):
        point = _a(pose.pose.position)
        # print 'point' , point
        yaw = np.arctan2(pose.pose.orientation.z,
                         pose.pose.orientation.w) * 2.0
        latlon, heading = self.pose2wgs84(point, yaw)
        lon, lat = latlon.tolist()[0]
        # print lon, lat
        msg = GlobalPositionTarget()
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
        msg.type_mask = (GlobalPositionTarget.IGNORE_VX +
                         GlobalPositionTarget.IGNORE_VY +
                         GlobalPositionTarget.IGNORE_VZ +
                         GlobalPositionTarget.IGNORE_AFX +
                         GlobalPositionTarget.IGNORE_AFY +
                         GlobalPositionTarget.IGNORE_AFZ +
                         GlobalPositionTarget.IGNORE_YAW +
                         GlobalPositionTarget.IGNORE_YAW_RATE)
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = 0.0
        msg.yaw = heading
        self.pub.publish(msg)
        self.pub_pose.publish(pose)

    def has_updated_pose(self, msg):
        if self.curve:
            pose = msg.pose
            t_pose = self.target_pose(pose)
            self.publish_target_pose(t_pose)


if __name__ == '__main__':
    PathFollower()
