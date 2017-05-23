import utm
import rospy
from mavros_msgs.msg import GlobalPositionTarget
from mavros_msgs.srv import SetMode
from tf.transformations import euler_from_quaternion
from .path_follower import PathFollower, quaternion_from_msg, np, pose_in_frame, twist_in_frame


# # Message for SET_POSITION_TARGET_GLOBAL_INT
# #
# # Some complex system requires all feautures that mavlink
# # message provide. See issue #402, #415.
#
# std_msgs/Header header
#
# uint8 coordinate_frame
# uint8 FRAME_GLOBAL_INT = 5
# uint8 FRAME_GLOBAL_REL_ALT = 6
# uint8 FRAME_GLOBAL_TERRAIN_ALT = 11
#
# uint16 type_mask
# uint16 IGNORE_LATITUDE = 1	# Position ignore flags
# uint16 IGNORE_LONGITUDE = 2
# uint16 IGNORE_ALTITUDE = 4
# uint16 IGNORE_VX = 8	# Velocity vector ignore flags
# uint16 IGNORE_VY = 16
# uint16 IGNORE_VZ = 32
# uint16 IGNORE_AFX = 64	# Acceleration/Force vector ignore flags
# uint16 IGNORE_AFY = 128
# uint16 IGNORE_AFZ = 256
# uint16 FORCE = 512	# Force in af vector flag
# uint16 IGNORE_YAW = 1024
# uint16 IGNORE_YAW_RATE = 2048
#
# float64 latitude
# float64 longitude
# float32 altitude	# in meters, AMSL or above terrain
# geometry_msgs/Vector3 velocity
# geometry_msgs/Vector3 acceleration_or_force
# float32 yaw
# float32 yaw_rate


def pose2wgs84(utm_pose_s, utm_zone_number=32, utm_zone_letter='T'):
    p = utm_pose_s.pose.position
    q = quaternion_from_msg(utm_pose_s.pose.orientation)
    _, _, yaw = euler_from_quaternion(q)
    lat, lon = utm.to_latlon(p.x, p.y, utm_zone_number, utm_zone_letter)
    # DONE: checked that yaw is defined in NED
    direction = yaw - np.pi * 0.5
    # Ardupilot currently ignore yaw!
    return lat, lon, direction


def vector2enu(vector_msg):
    # ROS convention is ENU
    # MAVLINK convention is NED
    # AND mavros excpect ENU
    return [vector_msg.x, vector_msg.y, vector_msg.z]


def vector2ned(vector_msg):
    # ROS convention is ENU
    # MAVLINK convention is NED

    return [vector_msg.y, vector_msg.x, -vector_msg.z]


def guide_mode_for(kind):
    return {'rover': 'GUIDED', 'copter': 'GUIDED'}.get(kind or 'rover')


def hover_mode_for(kind):
    return {'rover': 'MANUAL', 'copter': 'LOITER'}.get(kind or 'rover')


def enu2ned(velocity):
    return velocity[1], velocity[0], -velocity[2]


class MAVPathFollower(PathFollower):
    """docstring for PathFollower"""

    def __init__(self, tf_buffer=None, ns='', control_range=None, kind=None):
        rospy.loginfo('Will init MAVPathFollower for %s', ns)
        super(MAVPathFollower, self).__init__(tf_buffer=tf_buffer, ns=ns,
                                              control_range=control_range, kind=kind)
        target = rospy.get_param("~target", 'target')
        self.guide_mode = guide_mode_for(self.kind)
        self.hover_mode = hover_mode_for(self.kind)
        # Ardupilot ignore setpoints' yaws !!!!
        # self.ingnore_yaw = rospy.get_param("~ignore_yam", True)
        self.overwrite_altitude_target = rospy.get_param("~overwrite_altitude_target", False)
        self.altitude_target = rospy.get_param("~altitude_target", 0.0)
        alt_frame = rospy.get_param("~altitude_frame", 'relative')
        if alt_frame not in ['amsl', 'terrain', 'relative']:
            rospy.logerr('Wrong type of altitude frame. Should be one of amsl, terrain or relative')
            alt_frame = 'relative'
        self.altitude_frame = alt_frame
        self.global_pub = rospy.Publisher("{ns}{target}".format(ns=ns, target=target),
                                          GlobalPositionTarget, queue_size=1)
        self.change_mode = None
        rospy.loginfo('Will wait for service %smavros/set_mode', ns)
        try:
            rospy.wait_for_service('{ns}mavros/set_mode'.format(ns=ns), timeout=5)
            rospy.loginfo('Service %smavros/set_mode is available', ns)
            self.change_mode = rospy.ServiceProxy('{ns}mavros/set_mode'.format(ns=ns), SetMode)
        except rospy.ROSException:
            rospy.logwarn('Service %smavros/set_mode is not available', ns)
        rospy.loginfo('Initialized MAVPathFollower for %s', ns)
    # def reconfigure(self, config, level):
    #     config = super(MAVPathFollower, self).reconfigure(config, level)
    #     alt_frame = config.get('altitude_frame', self.altitude_frame)
    #     if alt_frame not in ['amsl', 'terrain', 'relative']:
    #         rospy.logerr('Wrong type of altitude frame. Should be one of amsl, terrain or relative')
    #         alt_frame = 'relative'
    #     self.altitude_frame = alt_frame
    #     return config

    def guide(self):
        # self.change_mode(SetModeRequest.MAV_MODE_GUIDED_ARMED, '')
        if self.change_mode:
            rospy.loginfo('Will switch to %s mode', self.guide_mode)
            rospy.sleep(0.1)
            result = self.change_mode(0, self.guide_mode)
            rospy.loginfo('Has switched to mode? %s', result)

    def hover(self):
        # self.change_mode(SetModeRequest.MAV_MODE_MANUAL_ARMED, '')
        if self.change_mode:
            rospy.loginfo('Will switch to %s mode', self.hover_mode)
            rospy.sleep(0.1)
            result = self.change_mode(0, self.hover_mode)
            rospy.loginfo('Has switched to mode? %s', result)

    def start(self):
        self.guide()
        super(MAVPathFollower, self).start()

    def stop(self, msg=None):
        self.hover()
        super(MAVPathFollower, self).stop(msg=msg)

    def coordinate_frame(self):
        if self.altitude_frame == 'amsl':
            return GlobalPositionTarget.FRAME_GLOBAL_INT
        elif self.altitude_frame == 'terrain':
            return GlobalPositionTarget.FRAME_GLOBAL_TERRAIN_ALT
        elif self.altitude_frame == 'relative':
            return GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
        else:
            rospy.logerr('Wrong type of altitude frame %s. '
                         'Should be one of amsl, relative or terrain',
                         self.altitude_frame)
            # Default to relative to terrain.
            return GlobalPositionTarget.FRAME_GLOBAL_REL_ALT

    def altitude(self, z):
        if self.altitude_target == 'relative':
            return 0
        if self.overwrite_altitude_target:
            return self.altitude_target
        else:
            return z

    def publish_target_pose(self, pose_s):
        # rospy.loginfo('publish_target_pose %s', pose_s)
        utm_pose_s = pose_in_frame(self.tf_buffer, pose_s, 'utm')
        if not utm_pose_s:
            rospy.logerr('Cannot compute pose in UTM frame')
            return
        lat, lon, heading = pose2wgs84(utm_pose_s)
        msg = GlobalPositionTarget()
        msg.coordinate_frame = self.coordinate_frame()
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
        msg.altitude = self.altitude(utm_pose_s.pose.position.z)
        # msg.yaw = heading
        self.global_pub.publish(msg)
        super(MAVPathFollower, self).publish_target_pose(pose_s)

    def publish_target_twist(self, twist_s):
        utm_twist_s = twist_in_frame(self.tf_buffer, twist_s, 'utm')
        if not utm_twist_s:
            rospy.logerr('Cannot compute twist in UTM frame')
            return
        # CHANGED: Mavros setpoint_raw plugin expect a velocity in ENU frame!!!
        v = vector2enu(utm_twist_s.twist.linear)
        msg = GlobalPositionTarget()
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT
        msg.type_mask = (GlobalPositionTarget.IGNORE_LATITUDE +
                         GlobalPositionTarget.IGNORE_LONGITUDE +
                         GlobalPositionTarget.IGNORE_ALTITUDE +
                         GlobalPositionTarget.IGNORE_AFX +
                         GlobalPositionTarget.IGNORE_AFY +
                         GlobalPositionTarget.IGNORE_AFZ +
                         GlobalPositionTarget.IGNORE_YAW +
                         GlobalPositionTarget.IGNORE_YAW_RATE)
        msg.velocity.x = v[0]
        msg.velocity.y = v[1]
        msg.velocity.z = v[2]
        self.global_pub.publish(msg)
        super(MAVPathFollower, self).publish_target_twist(twist_s)
