#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from geometry_msgs.msg import Point as PointMsg
from nav_msgs.msg import Path, Odometry
import numpy as np
from shapely.geometry import LineString, Point, Polygon
from mavros_msgs.msg import GlobalPositionTarget
from mavros_msgs.srv import SetMode
from dynamic_reconfigure.server import Server
from mav_control.cfg import PathFollowerConfig
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import utm
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformListener


def _a(point):
    return [point.x, point.y]


def _q(o):
    return [o.x, o.y, o.z, o.w]


def _yaw(o):
    return euler_from_quaternion(_q(o))[2]


class PathFollower(object):
    """docstring for PathFollower"""

    def __init__(self):
        super(PathFollower, self).__init__()
        rospy.init_node('follow_path', anonymous=True)
        self.tfBuffer = tf2_ros.Buffer()
        self.tf = TransformListener(self.tfBuffer)
        self.curve = None
        self.following = False
        self.path = None
        self.delta = rospy.get_param("~delta", 0.5)
        self.horizon = rospy.get_param("~distance", 1.5)
        self.loop = rospy.get_param("~loop", True)
        self.min_distance = rospy.get_param("~min_distance", 0.5)
        # We assume that the range is given in map frame (as for now, 2D)
        _range = rospy.get_param("~range", None)
        if _range:
            self.range_shape = Polygon(_range)
            self.range_height = (0, 100)
            rospy.loginfo("With range %s", self.range_shape.wkt)
        else:
            self.range_shape = self.range_height = None
            rospy.loginfo('Without range')

        rospy.wait_for_service('mavros/set_mode')
        self.change_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)

        rospy.Subscriber("odom", Odometry, self.has_updated_odometry)
        rospy.Subscriber("pose", PoseStamped, self.has_updated_pose)
        rospy.Subscriber("path", Path, self.has_updated_path)
        self.pub = rospy.Publisher(
            "target", GlobalPositionTarget, queue_size=1)
        self.pub_pose = rospy.Publisher(
            "target_pose", PoseStamped, queue_size=1)

        Server(PathFollowerConfig, self.reconfigure)

        rospy.spin()

    def guide(self):
        # self.change_mode(SetModeRequest.MAV_MODE_GUIDED_ARMED, '')
        self.change_mode(0, 'GUIDED')

    def hover(self):
        # self.change_mode(SetModeRequest.MAV_MODE_MANUAL_ARMED, '')
        self.change_mode(0, 'MANUAL')

    def in_range(self, pose):
        if not self.range_shape:
            return True
        p = Point(_a(pose.pose.position))
        return p.within(self.range_shape)

    def has_arrived(self, pose):
        if self.loop:
            return False
        target = self.ps[-1]
        distance = np.linalg.norm(target - _a(pose.pose.position))
        return distance < self.min_distance

    def reconfigure(self, config, level):
        self.delta = config.get('delta', self.delta)
        self.horizon = config.get('distance', self.horizon)
        return config

    def has_updated_path(self, msg):
        self.path = msg
        if not msg.poses:
            self.path = None
            self.curve = None
            self.following = False
            self.hover()
            rospy.loginfo("Empty path, will stop drone")
            return

        self.curve = LineString(
            [_a(pose.pose.position) for pose in msg.poses])
        self.ps = np.array(self.curve)
        self.ls = np.linalg.norm(np.diff(self.ps, axis=0), axis=1)
        self.cs = np.cumsum(self.ls)
        self.yaws = [_yaw(pose.pose.orientation) for pose in msg.poses]
        self.following = True
        self.guide()
        rospy.loginfo("Got new path, will guide to drone")

    def target(self, pose):
        current_point = np.array(_a(pose.position))
        s = self.curve.project(Point(_a(pose.position)))
        s = s + self.delta
        if s > self.cs[-1]:
            if self.loop:
                s = s - self.cs[-1]
            else:
                s = self.cs[-1]
        i0 = np.argmax(self.cs > s)
        i1 = (i0 + 1) % len(self.ps)
        im1 = (i0 - 1 + len(self.ps)) % len(self.ps)
        if im1 == len(self.cs):
            ds = s
        else:
            ds = s - self.cs[im1]
        n = self.ls[i0]
        a = ds / n
        point = (1 - a) * self.ps[i0] + a * self.ps[i1]
        yaw0 = self.yaws[i0]
        yaw1 = self.yaws[i1]
        yaw = np.arctan2(
            np.sin(yaw0) * (1 - a) + np.sin(yaw1) * a,
            np.cos(yaw0) * (1 - a) + np.cos(yaw1) * a)
        dp = (point - current_point)
        dpn = np.linalg.norm(dp)
        dp = dp / dpn
        dist = max(self.horizon, dpn)
        if not self.loop:
            e = np.linalg.norm(current_point - self.ps[-1])
            dist = min(dist, e)
        point = current_point + dist * dp
        return point, yaw

    def target_pose_stamped(self, pose_stamped):
        try:
            transform = self.tfBuffer.lookup_transform(
                self.path.header.frame_id, pose_stamped.header.frame_id,
                rospy.Time(0),
                rospy.Duration(0.1)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
            return None
        pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        if not self.in_range(pose):
            return None
        point, yaw = self.target(pose.pose)
        q = quaternion_from_euler(0, 0, yaw)
        msg = PoseStamped(
            header=Header(frame_id=self.path.header.frame_id),
            pose=Pose(
                position=PointMsg(point[0], point[1], 0),
                orientation=Quaternion(*q)
            )
        )
        if self.has_arrived(pose):
            self.following = False
            self.hover()
        return msg

    def pose2wgs84(self, pose_stamped):
        try:
            transform = self.tfBuffer.lookup_transform(
                'utm', pose_stamped.header.frame_id, rospy.Time(0),
                rospy.Duration(0.1)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
            return
        utm_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
        p = utm_pose.pose.position
        q = _q(utm_pose.pose.orientation)
        _, _, yaw = euler_from_quaternion(q)
        lat, lon = utm.to_latlon(p.x, p.y, 32, 'T')
        # TODO check that yaw is defined in NED
        northing = yaw - np.pi * 0.5
        # Ardupilot corrently ignore yaw!
        return lat, lon, northing

    def publish_target_pose(self, pose):
        lat, lon, heading = self.pose2wgs84(pose)
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
        if self.following:
            t_pose_stamped = self.target_pose_stamped(msg)
            if t_pose_stamped:
                self.publish_target_pose(t_pose_stamped)

    def has_updated_odometry(self, msg):
        if self.following:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose
            self.has_updated_pose(pose_stamped)


if __name__ == '__main__':
    PathFollower()
