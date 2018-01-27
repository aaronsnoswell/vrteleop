#!/usr/bin/env python

"""
A simple forwarder node
"""

import sys
import rospy
import argparse
from importlib import import_module


"""
Simple class that forwards messages
"""
class Forwarder(object):

    # Has the topic been advertised yet?
    advertised = False

    # The output topic
    pub = None

    """
    Constructor
    """
    def __init__(self, input_topic, output_topic=None):

        self.node_name = "forwarder"
        self.input_topic = input_topic

        self.output_topic = output_topic
        if self.output_topic is None:
            self.output_topic = self.input_topic + "_forwarded"

        rospy.init_node(self.node_name, anonymous=True)
        rospy.loginfo("Started forwarder node '{}'".format(rospy.get_name()))

        rospy.Subscriber(self.input_topic, rospy.AnyMsg, self.callback)
        rospy.loginfo("Subscribed to '{}'".format(self.input_topic))

        rospy.spin()

    """
    Message callback
    """
    def callback(self, data):

        if not self.advertised:
            # Detect message type
            connection_header =  data._connection_header['type'].split('/')
            ros_pkg = connection_header[0] + '.msg'
            msg_type = connection_header[1]
            msg_class = getattr(import_module(ros_pkg), msg_type)

            # Publish the message
            self.pub = rospy.Publisher(self.output_topic, msg_class, queue_size=10)
            rospy.loginfo("Advertised as '{}'".format(self.output_topic))
            self.advertised = True

        self.pub.publish(data)


"""
Main
"""
def main():
    
    # Set up arg parsing    
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_topic", type=str)
    parser.add_argument("output_topic", type=str, nargs="?", default=None)
    args = parser.parse_args(rospy.myargv(sys.argv[1:]))

    Forwarder(args.input_topic, args.output_topic)


if __name__ == '__main__':
    main()
