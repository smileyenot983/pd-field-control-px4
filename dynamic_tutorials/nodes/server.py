#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from dynamic_tutorials.cfg import TutorialsConfig

from dynamic_reconfigure.msg import Config


def callback(config, level):

    rospy.loginfo("Reconfigure Request(x,y,z): {field_x}, {field_y}, {field_z}".format(**config))
    return config




if __name__ == "__main__":
    rospy.init_node("dynamic_tutorials", anonymous = False)

    srv = Server(TutorialsConfig, callback)

    rospy.spin()