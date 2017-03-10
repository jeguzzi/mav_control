import numpy as np
from shapely.geometry import LineString, Point, Polygon
import rospy
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Header, Empty
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from geometry_msgs.msg import Point as PointMsg
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def array_from_msg(point_msg):
    return np.array(point_from_msg(point_msg))


def point_from_msg(point_msg):
    return [point_msg.x, point_msg.y]


def quaternion_from_msg(quaternion_msg):
    return [quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w]


def yaw_from_msg(quaternion_msg):
    return euler_from_quaternion(quaternion_from_msg(quaternion_msg))[2]


def get_transform(tf_buffer, from_frame, to_frame):
    try:
        return tf_buffer.lookup_transform(
            from_frame, to_frame, rospy.Time(0), rospy.Duration(0.1)
        )
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
        rospy.logerr(e)
        return None


def path_in_frame(tf_buffer, path, frame_id):
    t = get_transform(tf_buffer, frame_id, path.header.frame_id)
    if not t:
        return None
    msg = Path(header=path.header)
    msg.header.frame_id = frame_id
    msg.poses = [tf2_geometry_msgs.do_transform_pose(pose, t) for pose in path.poses]
    return msg


def pose_in_frame(tf_buffer, pose_s, frame_id):
    t = get_transform(tf_buffer, frame_id, pose_s.header.frame_id)
    if not t:
        return None
    return tf2_geometry_msgs.do_transform_pose(pose_s, t)


class PathFollower(object):
    """docstring for PathFollower"""
    # TODO: set speed

    def __init__(self, tf_buffer=None, ns=None, control_range=None):
        super(PathFollower, self).__init__()
        rospy.init_node('follow_path', anonymous=True)
        if not tf_buffer:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        else:
            self.tf_buffer = tf_buffer
        self.curve = None
        self.following = False
        self.path = None
        self.waypoint = None
        self.loop = False
        self.frame_id = rospy.get_param("~frame_id", "map")
        self.delta = rospy.get_param("~delta", 0.5)
        self.horizon = rospy.get_param("~distance", 1.5)
        self.min_distance = rospy.get_param("~min_distance", 0.5)
        self.target_height = rospy.get_param("~target_height", 0)
        self.height_range = rospy.get_param("~height_range", (0, 100))
        if not control_range:
            control_range = rospy.get_param("~range", None)
        if control_range:
            self.range_shape = Polygon(control_range)
            rospy.loginfo("With range %s", self.range_shape.wkt)
        else:
            self.range_shape = self.range_height = None
            rospy.loginfo('Without range')

        self.pub_pose = rospy.Publisher("{ns}target_pose".format(ns=ns), PoseStamped, queue_size=1)
        rospy.Subscriber("{ns}odom".format(ns=ns), Odometry, self.has_updated_odometry)
        rospy.Subscriber("{ns}stop".format(ns=ns), Empty, self.stop)
        rospy.Subscriber("{ns}pose".format(ns=ns), PoseStamped, self.has_updated_pose)
        rospy.Subscriber("{ns}path".format(ns=ns), Path, self.has_updated_path)
        rospy.Subscriber("{ns}waypoint".format(ns=ns), PoseStamped, self.has_updated_waypoint)

    def stop(self, msg=None):
        # Virtual
        self.path = None
        self.curve = None
        self.waypoint = None
        self.following = False

    def start(self):
        # Virtual
        self.following = True

    def publish_target_pose(self, pose_s):
        # Virtual
        self.pub_pose.publish(pose_s)

    def in_range(self, point):
        if not self.range_shape:
            return True
        p = Point(point_from_msg(point))
        return p.within(self.range_shape)

    @property
    def target_point(self):
        if self.waypoint:
            return self.waypoint.pose.position
        if self.path:
            return self.path.poses[-1].pose.position
        return None

    def has_arrived(self, point):
        if self.loop:
            return False
        distance = np.linalg.norm(array_from_msg(self.target_point) - array_from_msg(point))
        return distance < self.min_distance

    def reconfigure(self, config, level):
        self.delta = config.get('delta', self.delta)
        self.horizon = config.get('distance', self.horizon)
        self.min_distance = config.get('distance', self.min_distance)
        return config

    def has_updated_waypoint(self, msg):
        self.waypoint = pose_in_frame(self.tf_buffer, msg, self.frame_id)
        self.path = None
        self.loop = False
        following = self.waypoint is not None
        if following:
            rospy.loginfo("Got new wp, will guide the drone")
            self.start()
        else:
            rospy.loginfo("Got invalid wp, will stop the drone")
            self.stop()

    def has_updated_path(self, msg):
        self.path = path_in_frame(self.tf_buffer, msg, self.frame_id)
        self.waypoint = None
        if not msg.poses or not self.path:
            rospy.loginfo("Got invalid/empty path, will stop the drone")
            self.stop()
            return

        self.curve = LineString([point_from_msg(pose.pose.position) for pose in msg.poses])
        self.ps = np.array(self.curve)
        self.ls = np.linalg.norm(np.diff(self.ps, axis=0), axis=1)
        self.cs = np.cumsum(self.ls)
        self.loop = False
        if np.linalg.norm(self.ps[0] - self.ps[-1]) < 1e-3:
            if np.linalg.norm(self.ps[0] - self.ps[len(self.ps) / 2]) > 1:
                self.loop = True
        self.yaws = [yaw_from_msg(pose.pose.orientation) for pose in msg.poses]
        rospy.loginfo("Got new path (loop=%s), will guide the drone", self.loop)
        self.start()

    def target_along_path(self, current_point):
        s = self.curve.project(Point(current_point))
        s = s + self.delta
        if s > self.cs[-1]:
            if self.loop:
                s = s - self.cs[-1]
            else:
                return self.ps[-1], self.yaws[-1]
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

    def target_pose_along_path(self, pose_s):
        current_point = array_from_msg(pose_s.pose.position)
        point, yaw = self.target_along_path(current_point)
        q = quaternion_from_euler(0, 0, yaw)
        msg = PoseStamped(
            header=Header(frame_id=self.path.header.frame_id),
            pose=Pose(position=PointMsg(point[0], point[1], 0), orientation=Quaternion(*q)))
        return msg

    def has_updated_pose(self, msg):
        if self.following:
            pose_s = pose_in_frame(self.tf_buffer, msg, self.frame_id)
            if not pose_s:
                rospy.logerr('Could not transform pose %s to frame %s', msg, self.frame_id)
                return
            point = pose_s.pose.position
            if not self.in_range(point):
                rospy.loginfo('Current pose is outside of control range')
                return
            if self.has_arrived(point):
                rospy.loginfo('Has arrived, will stop')
                self.stop()
                return
            target_pose_s = None
            if self.waypoint:
                target_pose_s = self.waypoint
            elif self.path:
                target_pose_s = self.target_pose_along_path(pose_s)
            if not target_pose_s:
                rospy.logerr('No target pose')
                return
            self.publish_target_pose(target_pose_s)

    def has_updated_odometry(self, msg):
        if self.following:
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose
            self.has_updated_pose(pose_stamped)
