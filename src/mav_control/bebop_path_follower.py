import rospy
from geometry_msgs.msg import Twist
from .path_follower import PathFollower


class BebopPathFollower(PathFollower):
    """docstring for PathFollower"""

    def __init__(self, tf_buffer=None, ns='', control_range=None, kind=None):
        super(BebopPathFollower, self).__init__(tf_buffer=tf_buffer, ns=ns,
                                                control_range=control_range, kind=kind)

        rospy.Subscriber('cmd_vel_input', Twist, self.has_received_input)

    def has_received_input(self, msg):
        if self.following:
            rospy.loginfo('Will stop')
            self.stop()
