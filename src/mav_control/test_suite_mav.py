from __future__ import print_function
import rospy
from mavros_msgs.msg import State
from mav_control.test_suite import ControlSuite


class MAVControlSuite(ControlSuite):

    def has_updated_state(self, msg):
        self.state = msg

    def travelled(self, pose_s, tol):
        def t():
            if not self.state:
                print('not connected')
                return False
            if self.state.guided and self.state.mode not in ['MANUAL', 'LOITER']:
                print('guided')
                return False
            else:
                print('hovering')
            return self.arrived_at(pose_s, tol)
        return t

    def __init__(self):
        rospy.init_node('control_test', anonymous=True)
        super(MAVControlSuite, self).__init__()
        rospy.Subscriber('mavros/state', State, self.has_updated_state, queue_size=1)
