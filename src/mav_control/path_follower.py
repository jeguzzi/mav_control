import numpy as np
from shapely.geometry import LineString, Point, Polygon
import rospy
import tf2_ros
import tf2_geometry_msgs
from std_msgs.msg import Header, Empty
from geometry_msgs.msg import (PoseStamped, Pose, Quaternion, TwistStamped, Twist, Vector3,
                               Vector3Stamped)
from geometry_msgs.msg import Point as PointMsg
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def array_from_msg(point_msg):
    return np.array(point_from_msg(point_msg))


def array3_from_msg(point_msg):
    return np.array([point_msg.x, point_msg.y, point_msg.z])


def point_from_msg(point_msg):
    return [point_msg.x, point_msg.y]


def quaternion_from_msg(quaternion_msg):
    return [quaternion_msg.x, quaternion_msg.y, quaternion_msg.z, quaternion_msg.w]


def yaw_from_msg(quaternion_msg):
    return euler_from_quaternion(quaternion_from_msg(quaternion_msg))[2]


def angle_difference(angle_1, angle_2):
    a1, a2 = np.unwrap(np.array([angle_1, angle_2]))
    return a2 - a1


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
    # rospy.loginfo('pose_in_frame %s %s %s', tf_buffer, pose_s, frame_id)
    t = get_transform(tf_buffer, frame_id, pose_s.header.frame_id)
    if not t:
        return None
    return tf2_geometry_msgs.do_transform_pose(pose_s, t)


def twist_in_frame(tf_buffer, twist_s, frame_id):
    t = get_transform(tf_buffer, frame_id, twist_s.header.frame_id)
    if not t:
        return None
    h = Header(frame_id=frame_id, stamp=twist_s.header.stamp)
    vs_l = Vector3Stamped(header=h, vector=twist_s.twist.linear)
    vs_a = Vector3Stamped(header=h, vector=twist_s.twist.angular)
    msg = TwistStamped(header=h)
    msg.twist.linear = tf2_geometry_msgs.do_transform_vector3(vs_l, t).vector
    msg.twist.angular = tf2_geometry_msgs.do_transform_vector3(vs_a, t).vector
    return msg


def normalize_s(s, max_s, loop):
    if s < 0:
        if loop:
            s = s + max_s
        else:
            s = 0
    elif s > max_s:
        if loop:
            s = s - max_s
        else:
            s = max_s
    return s


def curve_segment(curve, s0, s1, loop, cs):
    p0 = curve.interpolate(s0)
    p1 = curve.interpolate(s1)
    i0 = np.argmax(cs > s0)
    i1 = np.argmax(cs > s1)
    if i0 < i1:
        coords = list(curve.coords[(i0 + 1):i1])
    else:
        coords = list(curve.coords[(i0 + 1):]) + list(curve.coords[:i1])

    return LineString([p0] + coords + [p1])


def project(point, curve, old_s, max_delta, loop, cs):
    if old_s is None:
        return curve.project(point)
    if max_delta > curve.length:
        return curve.project(point)
    min_s = normalize_s(old_s - max_delta, curve.length, loop)
    max_s = normalize_s(old_s + max_delta, curve.length, loop)
    segment = curve_segment(curve, min_s, max_s, loop, cs)
    s = segment.project(point)
    return normalize_s(s + old_s - max_delta, curve.length, loop)


def t_speed(x0, x1, tau, max_speed):
    v = (x1 - x0) / tau
    s = np.linalg.norm(v)
    if s > max_speed:
        v = v / s * max_speed
    return v


def t_angular_speed(x0, x1, tau, max_speed):
    v = angle_difference(x0, x1) / tau
    s = np.abs(v)
    if s > max_speed:
        v = v / s * max_speed
    return v


class PathFollower(object):
    """docstring for PathFollower"""
    # TODO: set speed

    def __init__(self, tf_buffer=None, ns='', control_range=None, kind=None):
        super(PathFollower, self).__init__()
        if not tf_buffer:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        else:
            self.tf_buffer = tf_buffer
        self.curve = None
        self.following = False
        self.path = None
        self.waypoint = None
        self.waypoint_velocity = None
        self.loop = False

        self.max_position_error = rospy.get_param("~max_position_error", 10.0)

        self.frame_id = rospy.get_param("~frame_id", "map")
        self.delta = rospy.get_param("~delta", 0.5)
        self.horizon = rospy.get_param("~distance", 1.5)
        self.min_distance = rospy.get_param("~min_distance", 0.5)
        # self.target_height = rospy.get_param("~target_height", 0)
        # self.height_range = rospy.get_param("~height_range", (0, 100))
        self.track_s = rospy.get_param("~track_s", True)
        self.flat = rospy.get_param("~flat", False)
        # 0th order: control by sending target points
        # 1th order: control by sending target velocities
        self.control_order = rospy.get_param("~control_order", 0)
        # relaxation time for 1th order control, i.e should reach path/point in \tau time
        self.tau = rospy.get_param("~tau", 0.5)
        self.eta = rospy.get_param("~eta", 1.0)
        # speed for 1th order control, i.e. should move at constant speed `speed`
        self.target_speed = rospy.get_param("~speed", 0.3)
        self.target_angular_speed = rospy.get_param("~angular_speed", 0.3)

        self.s = None
        self.max_speed = rospy.get_param("~max_speed", 2.0)
        odom = rospy.get_param("~odom", 'odom')
        pose = rospy.get_param("~pose", 'pose')
        rate = rospy.get_param('~max_rate', 5.0)
        self.min_dt = 1.0 / rate
        self.last_t = rospy.Time.now()
        if not control_range:
            control_range = rospy.get_param("~range", None)
        if control_range:
            self.range_shape = Polygon(control_range)
            rospy.loginfo("With range %s", self.range_shape.wkt)
        else:
            self.range_shape = self.range_height = None
            rospy.loginfo('Without range')
        if not kind:
            self.kind = rospy.get_param("~kind", None)

        self.pub_pose = rospy.Publisher("{ns}target_pose".format(ns=ns), PoseStamped, queue_size=1)
        self.pub_twist = rospy.Publisher("{ns}target_twist".format(ns=ns), TwistStamped,
                                         queue_size=1)
        rospy.Subscriber("{ns}{odom}".format(ns=ns, odom=odom), Odometry, self.has_updated_odometry)
        rospy.Subscriber("{ns}stop".format(ns=ns), Empty, self.stop)
        rospy.Subscriber("{ns}{pose}".format(ns=ns, pose=pose), PoseStamped, self.has_updated_pose)
        rospy.Subscriber("{ns}path".format(ns=ns), Path, self.has_updated_path)
        rospy.Subscriber("{ns}waypoint".format(ns=ns), PoseStamped, self.has_updated_waypoint)
        rospy.Subscriber("{ns}waypoint_velocity".format(ns=ns), TwistStamped,
                         self.has_updated_waypoint_velocity)

    def should_send(self):
        dt = rospy.Time.now() - self.last_t
        return dt.to_sec() > self.min_dt

    def stop(self, msg=None):
        # Virtual
        was_following = self.following
        rospy.loginfo('Stop %s', was_following)
        self.following = False
        # HACK: Always set self.following = None first
        # Else path may be unset but following may be true
        self.path = None
        self.curve = None
        self.waypoint = None
        self.waypoint_velocity = None
        if was_following and self.control_order == 1:
            msg = TwistStamped()
            msg.header.frame_id = self.frame_id
            msg.header.stamp = rospy.Time.now()
            self.publish_target_twist(msg)

    def start(self):
        # Virtual
        rospy.loginfo('Start')
        self.following = True

    def publish_target_pose(self, pose_s):
        # Virtual
        self.pub_pose.publish(pose_s)

    def publish_target_twist(self, twist_s):
        # Virtual
        self.pub_twist.publish(twist_s)

    def in_range(self, point):
        if not self.range_shape:
            return True
        p = Point(point_from_msg(point))
        return p.within(self.range_shape)

    @property
    def target_point(self):
        if self.waypoint:
            return self.waypoint.pose.position
        if self.path and self.path.poses:
            return self.path.poses[-1].pose.position
        return None

    def has_arrived(self, point):
        # rospy.loginfo('has_arrived %s', self.loop)
        if self.loop:
            return False
        distance = np.linalg.norm(array_from_msg(self.target_point) - array_from_msg(point))
        # rospy.loginfo('has_arrived %s %s %.1f -> %s', point, self.target_point, distance,
        #               distance < self.min_distance)
        return distance < self.min_distance

    def reconfigure(self, config, level):
        self.delta = config.get('delta', self.delta)
        self.horizon = config.get('distance', self.horizon)
        self.min_distance = config.get('min_distance', self.min_distance)
        self.tau = config.get('tau', self.tau)
        self.target_speed = config.get('speed', self.target_speed)
        self.target_angular_speed = config.get('angular_speed', self.target_angular_speed)
        self.control_order = config.get('control_order', self.control_order)
        self.track_s = config.get('track_s', self.track_s)
        self.flat = config.get('flat', self.flat)
        return config

    def has_updated_waypoint_velocity(self, msg):
        self.waypoint_velocity = twist_in_frame(self.tf_buffer, msg, self.frame_id)
        self.path = None
        self.loop = False
        self.waypoint = None
        following = self.waypoint_velocity is not None
        if following:
            rospy.loginfo("Got new wp, will guide the drone")
            self.start()
        else:
            rospy.loginfo("Got invalid wp, will stop the drone")
            self.stop()

    def has_updated_waypoint(self, msg):
        self.waypoint = pose_in_frame(self.tf_buffer, msg, self.frame_id)
        self.path = None
        self.loop = False
        self.waypoint_velocity = None
        following = self.waypoint is not None
        if following:
            rospy.loginfo("Got new wp, will guide the drone")
            self.start()
        else:
            rospy.loginfo("Got invalid wp, will stop the drone")
            self.stop()

    def has_updated_path(self, msg):
        # rospy.loginfo('has_updated_path %s', msg)
        self.path = path_in_frame(self.tf_buffer, msg, self.frame_id)
        # rospy.loginfo('trasformed to %s', self.path)
        self.waypoint = None
        self.waypoint_velocity = None
        if not msg.poses or not self.path:
            rospy.loginfo("Got invalid/empty path, will stop the drone")
            self.stop()
            return

        self.curve = LineString([point_from_msg(pose.pose.position) for pose in self.path.poses])
        self.ps = np.array(self.curve)
        self.ls = np.linalg.norm(np.diff(self.ps, axis=0), axis=1)
        self.cs = np.cumsum(self.ls)
        self.loop = False
        if np.linalg.norm(self.ps[0] - self.ps[-1]) < 1e-3:
            if np.linalg.norm(self.ps[0] - self.ps[len(self.ps) / 2]) > 1:
                self.loop = True
        self.yaws = [yaw_from_msg(pose.pose.orientation) for pose in self.path.poses]
        self.zs = [pose.pose.position.z for pose in self.path.poses]

        # velocity along the path (with norm self.target_speed)
        # self.vs[i] is the velocity to be hold between points i and i+1
        xs = np.array([array_from_msg(pose.pose.position) for pose in self.path.poses])
        vs = np.diff(xs, axis=0).T
        vs = (vs / np.linalg.norm(vs, axis=0)).T
        self.vs = np.append(vs, vs[:1], axis=0)
        # rospy.loginfo("Got new path (loop=%s), will guide the drone", self.loop)
        # rospy.loginfo('curve %s', self.curve.wkt)
        if self.track_s:
            self.s = None
        self.start()

# TODO: add a parameter for s velocity (vs s space).
# At low speed the dp/tau component is much more important
# e.g. v ~ 0.3, delta ~ 0.5 m, tau ~ 1 s -> v = 0.5/1.0 = 0.5 m/s > max_speed
# so it's basically the same controller as before. May use v at the projection point +s
# and p at the projection point

    def target_along_path(self, current_point, current_z):
        cp = Point(current_point)
        if self.track_s:
            dt = (rospy.Time.now() - self.last_t).to_sec()
            # print(self.s)
            s = self.s = project(cp, self.curve, self.s, self.max_speed * dt, self.loop, self.cs)
            # print(self.s, dt, self.max_speed * dt)
        else:
            s = self.curve.project(cp)
        # nearest_point = self.curve.interpolate(s).coords[0]
        s = s + self.delta
        if s > self.cs[-1]:
            if self.loop:
                s = s - self.cs[-1]
            else:
                v = [0, 0, t_speed(current_z, self.zs[-1], self.tau, self.target_speed)]
                return self.ps[-1], self.yaws[-1], self.zs[-1], v
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
        z0 = self.zs[i0]
        z1 = self.zs[i1]
        z = z0 * (1 - a) + z1 * a
        yaw0 = self.yaws[i0]
        yaw1 = self.yaws[i1]
        yaw = np.arctan2(
            np.sin(yaw0) * (1 - a) + np.sin(yaw1) * a,
            np.cos(yaw0) * (1 - a) + np.cos(yaw1) * a)
        dp = (point - current_point)
        target_v = self.vs[i0]
        dp_t = dp - np.dot(target_v, dp) * target_v
        v = self.target_speed * target_v + dp_t / self.tau

        dpn = np.linalg.norm(dp)
        dp = dp / dpn
        dist = max(self.horizon, dpn)
        if not self.loop:
            e = np.linalg.norm(current_point - self.ps[-1])
            dist = min(dist, e)
        point = current_point + dist * dp

        nv = np.linalg.norm(v)

        if self.loop:
            target_dist = np.inf
        else:
            target_dist = np.linalg.norm(self.ps[-1] - current_point)
        target_speed = np.min([nv, self.target_speed, target_dist / self.eta])
        v = v / nv * target_speed
        vz = t_speed(current_z, z, self.tau, self.target_speed)

        return point, yaw, z, [v[0], v[1], vz]

    def target_pose_along_path(self, pose_s):
        if not self.following:
            return None
        current_point = array_from_msg(pose_s.pose.position)
        # rospy.loginfo('current_point %s', current_point)
        current_z = pose_s.pose.position.z
        point, yaw, z, v = self.target_along_path(current_point, current_z)
        if self.flat:
            z = current_z
        # rospy.loginfo('target_point %s', point)
        q = quaternion_from_euler(0, 0, yaw)
        msg = PoseStamped(
            header=Header(frame_id=self.path.header.frame_id),
            pose=Pose(position=PointMsg(point[0], point[1], z), orientation=Quaternion(*q)))
        return msg

    def target_twist_along_path(self, pose_s):
        if not self.following:
            return None
        current_point = array_from_msg(pose_s.pose.position)
        current_z = pose_s.pose.position.z
        current_yaw = yaw_from_msg(pose_s.pose.orientation)
        point, yaw, z, v = self.target_along_path(current_point, current_z)
        target_angular_speed = t_angular_speed(current_yaw, yaw, self.tau,
                                               self.target_angular_speed)
        if self.flat:
            v[2] = 0
        msg = TwistStamped(
            header=Header(frame_id=self.path.header.frame_id, stamp=rospy.Time.now()),
            twist=Twist(linear=Vector3(*v), angular=Vector3(0, 0, target_angular_speed)))
        return msg

    def target_twist_to_point(self, pose_s, target_pose_s):
        if not self.following:
            return None
        # assume pose_s and target_pose_s have already been transformed to the same frame
        # (self.frame_id)
        assert pose_s.header.frame_id == target_pose_s.header.frame_id, \
            ('target_twist_to_point different frames %s %s' %
             (pose_s.header.frame_id, target_pose_s.header.frame_id))

        current_point = array_from_msg(pose_s.pose.position)
        target_point = array_from_msg(target_pose_s.pose.position)

        target_velocity = t_speed(current_point, target_point, self.tau, self.target_speed)
        if self.flat:
            target_velocity_z = 0
        else:
            target_velocity_z = t_speed(pose_s.pose.position.z, target_pose_s.pose.position.z,
                                        self.tau, self.target_speed)

        current_yaw = yaw_from_msg(pose_s.pose.orientation)
        target_yaw = yaw_from_msg(target_pose_s.pose.orientation)
        target_angular_speed = t_angular_speed(
            current_yaw, target_yaw, self.tau, self.target_angular_speed)

        msg = TwistStamped(
            header=Header(frame_id=pose_s.header.frame_id, stamp=rospy.Time.now()),
            twist=Twist(linear=Vector3(target_velocity[0], target_velocity[1], target_velocity_z),
                        angular=Vector3(0, 0, target_angular_speed)))
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
            target_twist_s = None
            if self.waypoint:
                if self.control_order == 0:
                    target_pose_s = self.waypoint
                    if self.flat:
                        target_pose_s.pose.position.z = pose_s.pose.position.z
                else:
                    target_twist_s = self.target_twist_to_point(pose_s, self.waypoint)
            elif self.waypoint_velocity:
                target_twist_s = self.waypoint_velocity
                if self.flat:
                    target_twist_s.twist.linear.z = 0
            elif self.path:
                if self.control_order == 0:
                    target_pose_s = self.target_pose_along_path(pose_s)
                else:
                    target_twist_s = self.target_twist_along_path(pose_s)
            if not target_pose_s and not target_twist_s:
                rospy.logerr('No target pose or twist')
                return
            if self.should_send():
                self.last_t = rospy.Time.now()
                if target_pose_s:
                    self.publish_target_pose(target_pose_s)
                elif target_twist_s:
                    self.publish_target_twist(target_twist_s)

    def has_updated_odometry(self, msg):
        if self.following:
            # CHANGED: test if localized
            # as of covariance is small enough
            if msg.pose.covariance[0] > self.max_position_error ** 2:
                return

            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = msg.pose.pose
            self.has_updated_pose(pose_stamped)
