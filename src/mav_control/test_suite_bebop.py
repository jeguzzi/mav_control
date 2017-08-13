import rospy
from mav_control.test_suite import ControlSuite
from std_msgs.msg import Empty


class BebopControlSuite(ControlSuite):

    def stop(self):
        pass

    def has_received_stop(self, msg):
        if self.test and self.test.running:
            self.test.stop()

    def __init__(self):
        rospy.init_node('control_test', anonymous=True)
        rospy.Subscriber('stop', Empty, self.has_received_stop, queue_size=1)
        super(BebopControlSuite, self).__init__()
