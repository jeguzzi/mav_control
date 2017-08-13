import numpy as np
import rospy
import tf2_ros
from keyboard.msg import Key
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Empty, String
from mav_control.test_commands import ControlTest
from mav_control.path_follower import array_from_msg, pose_in_frame


class ControlSuite(object):

    def stop(self):
        self.stop_pub.publish()

    def load_tests(self, config):
        return [ControlTest(self, **c) for c in config]

    def has_updated_pose(self, msg):
        self.pose = msg

    def has_updated_odometry(self, msg):
        p = PoseStamped()
        p.header = msg.header
        p.pose = msg.pose.pose
        self.pose = p

    def arrived_at(self, pose_s, tol):
        if not self.pose:
            rospy.logwarn('not localized')
            return False
        p = pose_in_frame(self.tf_buffer, self.pose, pose_s.header.frame_id)
        dp = array_from_msg(p.pose.position) - array_from_msg(pose_s.pose.position)
        dist = np.linalg.norm(dp)
        if dist < tol:
            rospy.loginfo('DONE')
            return True
        return False

    def travelled(self, pose_s, tol):
        # VIRTUAL
        def t():
            return self.arrived_at(pose_s, tol)
        return t

    def got_key(self, msg):
        if msg.code == Key.KEY_SPACE:
            if self.test and self.test.running:
                rospy.loginfo('Will stop test')
                self.test.stop()
            return
        if self.test and self.test.running:
            rospy.loginfo('Test already running')
            return
        for test in self.tests:
            if test.key == msg.code:
                self.test = test
                rospy.loginfo('Will start test')
                self.test.run()

    def __init__(self):
        rospy.init_node('control_test', anonymous=True)
        self.should_stop = False
        self.state = self.pose = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.stop_pub = rospy.Publisher('stop', Empty, queue_size=1)
        self.path_pub = rospy.Publisher('path', Path, queue_size=1, latch=True)
        self.targets_pub = rospy.Publisher('targets', String, queue_size=1, latch=True)
        self.waypoint_pub = rospy.Publisher('waypoint', PoseStamped, queue_size=1, latch=True)
        self.tests = self.load_tests(rospy.get_param('~tests'))
        # print(self.tests)
        self.cmd_done = False
        self.test = None
        rospy.Subscriber('pose', PoseStamped, self.has_updated_pose, queue_size=1)
        rospy.Subscriber('odom', Odometry, self.has_updated_odometry, queue_size=1)
        rospy.Subscriber('keyboard/keydown', Key, self.got_key, queue_size=1)
