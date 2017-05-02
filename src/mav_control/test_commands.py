import threading
import rospy
from mav_control.loops import loop
from geometry_msgs.msg import PoseStamped, Pose, Point
from std_msgs.msg import Header, String
from keyboard.msg import Key
from nav_msgs.msg import Path


def key2code(key):
    # type: (str) -> int
    return getattr(Key, key, None)


class ControlTest(object):
    @staticmethod
    def command(suite, _type, config):
        cmd = {'sleep': SleepCommand,
               'waypoint': WaypointCommand,
               'path': PathCommand,
               'sync': SyncCommand,
               'loop': LoopCommand}.get(_type, None)
        if cmd:
            return cmd(suite, **config)
        else:
            return None

    def __init__(self, suite, key, commands):
        self.key = key2code(key)
        self.config = commands
        self.commands = [self.command(suite, *(c.items()[0])) for c in commands]
        self.running_thread = None
        self.suite = suite
        self.should_stop = False

    def __repr__(self):
        cs = '\n'.join([c.__repr__() for c in self.commands])
        return '\nKey:\n{0}\nCommands:\n{1}\n'.format(self.key, cs)

    @property
    def running(self):
        return self.running_thread and self.running_thread.is_alive()

    def run_all_commands(self):
        rospy.loginfo('--- Start test %s ---', self.config)
        self.should_stop = False
        for c, config in zip(self.commands, self.config):
            if self.should_stop:
                break
            if c:
                rospy.loginfo('- Start cmd %s -', config)
                c(self)
                rospy.loginfo('- Done -')
        rospy.loginfo('--- Test Done ---')

    def run(self):
        self.running_thread = threading.Thread(target=self.run_all_commands)
        self.running_thread.start()

    def stop(self):
        self.should_stop = True


class SleepCommand(object):
    def __init__(self, suite, duration):
        self.duration = duration

    def __call__(self, parent):
        rospy.sleep(rospy.Duration(self.duration))
        return 0

    def __repr__(self):
        return '*** Sleep for {0}'.format(self.duration)


class WaypointCommand(object):
    def __init__(self, suite, frame_id, point, tol=0.5, speed=0.3):
        self.waypoint = PoseStamped()
        self.waypoint.header.frame_id = frame_id
        if len(point) == 2:
            p = point + [0]
        elif len(point) == 3:
            p = point
        self.waypoint.pose.position = Point(*p)
        self.test = suite.travelled(self.waypoint, tol)
        self.pub = suite.waypoint_pub
        self.tol = tol
        self.stop = suite.stop

    def __call__(self, parent):
        self.pub.publish(self.waypoint)
        while True:
            rospy.sleep(1)
            if self.test():
                return 0
            if parent.should_stop:
                self.stop()
                return 1

    def __repr__(self):
        return '*** Go to\n{0}\nwith tol {1}'.format(self.waypoint, self.tol)


class PathCommand(object):
    def __init__(self, suite, frame_id, curve, tol=0.5, speed=0.3, altitude=0):
        h = Header(frame_id=frame_id)
        self.path = Path()
        self.path.header = h
        self.path.poses = [PoseStamped(h, Pose(position=Point(*(p + [altitude])))) for p in curve]
        self.test = suite.travelled(self.path.poses[-1], tol)
        self.tol = tol
        self.pub = suite.path_pub
        self.stop = suite.stop

    def __call__(self, parent):
        self.pub.publish(self.path)
        while True:
            rospy.sleep(1)
            if self.test():
                return 0
            if parent.should_stop:
                self.stop()
                return 1

    def __repr__(self):
        return '*** Travel along\n{0}\nwith tol {1}'.format(self.path, self.tol)


class SyncCommand(object):
    def __init__(self, suite, targets, point=None, frame_id=None, tol=0.5):
        self.msg = String(','.join(targets))
        self.tol = tol
        # DONE: add test, maybe as a last pose to reach if target[-1] != ...
        if not point or not frame_id:
            self.test = lambda: False
        else:
            pose_s = PoseStamped(Header(frame_id=frame_id), Pose(position=Point(*(point + [0]))))
            self.test = suite.travelled(pose_s, tol)
        self.pub = suite.targets_pub
        self.stop = suite.stop

    def __call__(self, parent):
        self.pub.publish(self.msg)
        while True:
            rospy.sleep(1)
            if self.test():
                break
            if parent.should_stop:
                self.stop()
                break

    def __repr__(self):
        return '*** Travel to cells\n{0}\nwith tol {1}'.format(self.msg, self.tol)


class LoopCommand(PathCommand):
    def __init__(self, suite, name, center, radius, frame_id='map', **kwargs):
        self.path = loop(name, center, radius, frame_id, **kwargs)
        self.tol = 1
        self.test = suite.travelled(self.path.poses[-1], self.tol)
        self.pub = suite.path_pub
        self.stop = suite.stop
