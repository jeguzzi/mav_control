import utm
import rospy
from mavros_msgs.msg import GlobalPositionTarget
from mavros_msgs.srv import SetMode
from tf.transformations import euler_from_quaternion
from .path_follower import PathFollower, quaternion_from_msg, np, pose_in_frame


def pose2wgs84(utm_pose_s):
    p = utm_pose_s.pose.position
    q = quaternion_from_msg(utm_pose_s.pose.orientation)
    _, _, yaw = euler_from_quaternion(q)
    lat, lon = utm.to_latlon(p.x, p.y, 32, 'T')
    # DONE: checked that yaw is defined in NED
    direction = yaw - np.pi * 0.5
    # Ardupilot currently ignore yaw!
    return lat, lon, direction


class MAVPathFollower(PathFollower):
    """docstring for PathFollower"""

    def __init__(self, **kwrgs):
        super(MAVPathFollower, self).__init__(**kwrgs)
        self.change_mode = None
        rospy.wait_for_service('mavros/set_mode')
        self.change_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.global_pub = rospy.Publisher("target", GlobalPositionTarget, queue_size=1)

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

    def publish_target_pose(self, pose_s):
        utm_pose_s = pose_in_frame(self.tf_buffer, pose_s, 'utm')
        if not utm_pose_s:
            rospy.logerr('Cannot compute pose in UTM frame')
            return
        lat, lon, heading = pose2wgs84(utm_pose_s)
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
        self.global_pub.publish(msg)
        super(MAVPathFollower, self).publish_target_pose(pose_s)
