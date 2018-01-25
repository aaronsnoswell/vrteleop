#!/usr/bin/env python

"""
Drops X out of every Y messages
"""

import sys
import rospy
import argparse
from importlib import import_module


"""
Simple class that drops messages
"""
class Dropper(object):

    # Has the topic been advertised yet?
    advertised = False

    # The output topic
    pub = None

    # Count of messages we've seen since last drop
    count = 0

    """
    Constructor
    """
    def __init__(self, input_topic, x, y, output_topic=None):

        self.node_name = "dropper"
        self.input_topic = input_topic
        self.x = x
        self.y = y

        self.output_topic = output_topic
        if self.output_topic is None:
            self.output_topic = self.input_topic + "_dropped"

        rospy.init_node(self.node_name, anonymous=True)
        print("Started drop node '{}'".format(rospy.get_name()))

        rospy.Subscriber(self.input_topic, rospy.AnyMsg, self.callback)
        print("Subscribed to '{}'".format(self.input_topic))

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
            print("Advertised as '{}'".format(self.output_topic))
            self.advertised = True

        if self.count >= self.x:
            print("Publishing")
            self.pub.publish(data)

        self.count += 1

        if self.count >= self.y:
            self.count = 0


"""
Main
"""
def main():
    
    # Set up arg parsing    
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_topic", type=str)
    parser.add_argument("x", type=int)
    parser.add_argument("y", type=int)
    parser.add_argument("output_topic", type=str, nargs="?", default=None)
    args = parser.parse_args(rospy.myargv(sys.argv[1:]))

    Dropper(args.input_topic, args.x, args.y, args.output_topic)


if __name__ == '__main__':
    main()
