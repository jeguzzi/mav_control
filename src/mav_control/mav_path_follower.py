import utm
import rospy
from mavros_msgs.msg import GlobalPositionTarget
from mavros_msgs.srv import SetMode
from tf.transformations import euler_from_quaternion
from .path_follower import PathFollower, quaternion_from_msg, np, pose_in_frame


def pose2wgs84(utm_pose_s, utm_zone_number=32, utm_zone_letter='T'):
    p = utm_pose_s.pose.position
    q = quaternion_from_msg(utm_pose_s.pose.orientation)
    _, _, yaw = euler_from_quaternion(q)
    lat, lon = utm.to_latlon(p.x, p.y, utm_zone_number, utm_zone_letter)
    # DONE: checked that yaw is defined in NED
    direction = yaw - np.pi * 0.5
    # Ardupilot currently ignore yaw!
    return lat, lon, direction


class MAVPathFollower(PathFollower):
    """docstring for PathFollower"""

    def __init__(self, tf_buffer=None, ns='', control_range=None):
        super(MAVPathFollower, self).__init__(tf_buffer=tf_buffer, ns=ns,
                                              control_range=control_range)
        target = rospy.get_param("~target", 'target')
        self.ingnore_yaw = rospy.get_param("~ignore_yam", True)
        self.overwrite_altitude_target = rospy.get_param("~overwrite_altitude_target", False)
        self.altitude_target = rospy.get_param("~altitude_target", 0.0)
        alt_frame = rospy.get_param("~altitude_frame", 'terrain')
        if alt_frame not in ['amsl', 'terrain']:
            rospy.logerr('Wrong type of altitude frame. Should be one of amsl or terrain')
            alt_frame = 'terrain'
        self.altitude_frame = alt_frame
        self.global_pub = rospy.Publisher("{ns}{target}".format(ns=ns, target=target),
                                          GlobalPositionTarget, queue_size=1)
        self.change_mode = None
        rospy.wait_for_service('{ns}mavros/set_mode'.format(ns=ns))
        self.change_mode = rospy.ServiceProxy('{ns}mavros/set_mode'.format(ns=ns), SetMode)

    def guide(self):
        # self.change_mode(SetModeRequest.MAV_MODE_GUIDED_ARMED, '')
        if self.change_mode:
            self.change_mode(0, 'GUIDED')

    def hover(self):
        # self.change_mode(SetModeRequest.MAV_MODE_MANUAL_ARMED, '')
        if self.change_mode:
            self.change_mode(0, 'MANUAL')

    def start(self):
        self.guide()
        super(MAVPathFollower, self).start()

    def stop(self, msg=None):
        self.hover()
        super(MAVPathFollower, self).stop(msg=msg)

    def coordinate_frame(self):
        if self.altitude_target == 'amsl':
            return GlobalPositionTarget.FRAME_GLOBAL_INT
        elif self.altitude_target == 'terrain':
            return GlobalPositionTarget.FRAME_GLOBAL_TERRAIN_ALT
        else:
            rospy.logerr('Wrong type of altitude frame %s. Should be one of amsl or terrain',
                         self.altitude_frame)
            # Default to relative to terrain.
            return GlobalPositionTarget.FRAME_GLOBAL_TERRAIN_ALT

    def altitude(self, z):
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
                         GlobalPositionTarget.IGNORE_YAW_RATE)

        if self.ignore_yaw:
            msg.type_mask += GlobalPositionTarget.IGNORE_YAW

        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = self.altitude(utm_pose_s.position.z)
        msg.yaw = heading
        self.global_pub.publish(msg)
        super(MAVPathFollower, self).publish_target_pose(pose_s)
