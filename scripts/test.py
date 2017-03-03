#!/usr/bin/env python

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Odometry
from keyboard.msg import Key
from mavros_msgs.msg import State
import tf2_geometry_msgs
from tf2_ros import TransformListener
import numpy as np
import tf2_ros
import threading


def _a(point):
    return [point.x, point.y]


def key2code(key):
    return getattr(Key, key, None)


class ControlSuite(object):

    def load_tests(self, config):
        return [ControlTest(self, **c) for c in config]

    def get_transform(self, from_frame, to_frame):
        try:
            return self.tfBuffer.lookup_transform(
                from_frame, to_frame, rospy.Time(0), rospy.Duration(0.1)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
            return None

    def pose_in_frame(self, pose_s, frame_id):
        t = self.get_transform(pose_s.header.frame_id, frame_id)
        if not t:
            return None
        return tf2_geometry_msgs.do_transform_pose(pose_s, t)

    def has_updated_pose(self, msg):
        self.pose = msg

    def has_updated_odometry(self, msg):
        p = PoseStamped()
        p.header = msg.header
        p.pose = msg.pose.pose
        self.pose = msg.pose

    def has_updated_state(self, msg):
        self.state = msg

    def travelled(self, pose_s, tol):
        def t():
            if not self.state:
                return False
            if self.state.guided:
                return False
            if not self.pose:
                return False
            p = self.pose_in_frame(self.pose, pose_s.frame_id)
            if np.linalg.norm(_a(p.pose.position) - _a(pose_s.pose.position)) < tol:
                return True
            return False
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
        self.state = self.pose = None
        self.tfBuffer = tf2_ros.Buffer()
        self.tf = TransformListener(self.tfBuffer)
        self.path_pub = rospy.Publisher('path', Path, queue_size=1, latch=True)
        self.waypoint_pub = rospy.Publisher('waypoint', PoseStamped, queue_size=1, latch=True)
        self.tests = self.load_tests(rospy.get_param('~tests'))
        # print(self.tests)
        self.cmd_done = False
        self.test = None
        rospy.Subscriber('pose', PoseStamped, self.has_updated_pose, queue_size=1)
        rospy.Subscriber('odom', Odometry, self.has_updated_odometry, queue_size=1)
        rospy.Subscriber('keyboard/keydown', Key, self.got_key, queue_size=1)
        rospy.Subscriber('mavros/state', State, self.has_updated_state, queue_size=1)
        rospy.spin()


class ControlTest(object):
    @staticmethod
    def command(suite, _type, config):
        cmd = {'sleep': SleepCommand,
               'waypoint': WaypointCommand,
               'path': PathCommand}.get(_type, None)
        if cmd:
            return cmd(suite, **config)
        else:
            return None

    def __init__(self, suite, key, commands):
        self.key = key2code(key)
        self.config = commands
        self.commands = [self.command(suite, *(c.items()[0])) for c in commands]
        self.running_thead = None

    def __repr__(self):
        cs = '\n'.join([c.__repr__() for c in self.commands])
        return '\nKey:\n{0}\nCommands:\n{1}\n'.format(self.key, cs)

    @property
    def running(self):
        return self.running_thead and self.running_thead.is_alive()

    def run_all_commands(self):
        rospy.loginfo('--- Start test %s ---', self.config)
        for c, config in zip(self.commands, self.config):
            if self.should_stop:
                break
            if c:
                rospy.loginfo('- Start cmd %s -', config)
                c(self)
                rospy.loginfo('- Done -')
        rospy.loginfo('--- Test Done ---')

    def run(self):
        self.should_stop = False
        self.running_thead = threading.Thread(target=self.run_all_commands)
        self.running_thead.start()

    def stop(self):
        self.should_stop = True


class SleepCommand(object):
    def __init__(self, suite, duration):
        self.duration = duration

    def __call__(self, parent):
        rospy.sleep(rospy.Duration(self.duration))

    def __repr__(self):
        return '*** Sleep for {0}'.format(self.duration)


class WaypointCommand(object):
    def __init__(self, suite, frame_id, point, tol=0.5, speed=0.3):
        self.waypoint = PoseStamped()
        self.waypoint.header.frame_id = frame_id
        p = point + [0]
        self.waypoint.pose.position = Point(*p)
        self.test = suite.travelled(self.waypoint, tol)
        self.pub = suite.waypoint_pub
        self.tol = tol

    def __call__(self, parent):
        self.pub.publish(self.waypoint)
        while not self.test() and not parent.should_stop:
            print('.')
            rospy.sleep(1)

    def __repr__(self):
        return '*** Go to\n{0}\nwith tol {1}'.format(self.waypoint, self.tol)


class PathCommand(object):
    def __init__(self, suite, frame_id, curve, tol=0.5, speed=0.3):
        h = Header(frame_id=frame_id)
        self.path = Path()
        self.path.header = h
        self.path.poses = [PoseStamped(h, Pose(position=Point(*(p + [0])))) for p in curve]
        self.test = suite.travelled(self.path.poses[-1], tol)
        self.tol = tol
        self.pub = suite.path_pub

    def __call__(self, parent):
        self.pub.publish(self.path)
        while not self.test() and not parent.should_stop:
            print('.')
            rospy.sleep(1)

    def __repr__(self):
        return '*** Travel along\n{0}\nwith tol {1}'.format(self.path, self.tol)


if __name__ == '__main__':
    ControlSuite()
