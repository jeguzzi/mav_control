#!/usr/bin/env python
import rospy
from mav_control.loops import path_msg, f1_path, eight_path, circle_path, np, Path

if __name__ == '__main__':

    rospy.init_node('publish_loop', anonymous=True)
    pub = rospy.Publisher('path', Path, queue_size=1, latch=True)
    center = np.array(rospy.get_param('~center'))
    radius = rospy.get_param('~radius')
    frame_id = rospy.get_param('~frame_id')
    t = rospy.get_param('~type')
    paths = {'f1': f1_path(),
             'eight': eight_path(np.pi * 0.25),
             'circle': circle_path()}
    msg = path_msg(paths.get(t, paths['f1']), center, radius, frame_id)
    pub.publish(msg)
    rospy.spin()
