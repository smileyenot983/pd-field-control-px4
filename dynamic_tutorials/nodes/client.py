#!/usr/bin/env python

import rospy

import dynamic_reconfigure.client

from std_msgs.msg import Float32

def callback(config):
    rospy.loginfo("Field(x,y,z) were set to {field_x},{field_y},{field_z}".format(**config))

    pub_x.publish(config.field_x)
    pub_y.publish(config.field_y)
    pub_z.publish(config.field_z)



if __name__ == "__main__":
    rospy.init_node("dynamic_client")

    client = dynamic_reconfigure.client.Client("dynamic_tutorials", timeout=30, config_callback=callback)

    pub_x = rospy.Publisher('/dynamic_reconfigure/parameters/field_x', Float32, queue_size=10)
    pub_y = rospy.Publisher('/dynamic_reconfigure/parameters/field_y', Float32, queue_size=10)
    pub_z = rospy.Publisher('/dynamic_reconfigure/parameters/field_z', Float32, queue_size=10)
    rospy.spin()
