#!/usr/bin/env python
import platform
import rospy
from dynamic_reconfigure.server import Server
from mav_control.cfg import PathFollowerConfig
import tf2_ros


class Controller(object):

    def __init__(self, cls):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.init_agent_controllers(cls)
        Server(PathFollowerConfig, self.reconfigure)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        for controller in self.controllers:
            controller.stop()
        rospy.sleep(2)

    def reconfigure(self, config, level):
        for controller in self.controllers:
            controller.reconfigure(config, level)
        return config

    def init_agent_controllers(self, cls):
        self.controllers = [cls(tf_buffer=self.tf_buffer)]


class MultiController(Controller):

    def init_agent_controllers(self, cls):
        drones = rospy.get_param("~drones", rospy.get_param('/drones', []))
        hostname = platform.uname()[1]
        control_range = rospy.get_param("/controllers/{0}/range".format(hostname), None)
        self.controllers = [cls(ns="{0}/".format(drone['ns']), control_range=control_range,
                                tf_buffer=self.tf_buffer, kind=drone.get('kind', None))
                            for drone in drones]
