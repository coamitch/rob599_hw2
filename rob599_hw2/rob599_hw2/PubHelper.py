#!/usr/bin/env python3

# Node used to help generate example Twist messages to test the speed_limiter node. User
# should be able to type in components for the linear and angular velocities and publish.
#
# PubHelper.py
#
# Colin Mitchell

# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import random

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node


# creating a class to manage the node function
class PubHelperNode(Node):
    # constructor
    def __init__(self):
        # init parent (node)
        super().__init__("pub_helper")

        # creating a subscriber and publisher for speed in and out respectively
        self._pub = self.create_publisher(Twist, "speed_in", 10)

        # creating a timer to generate a publish a message every 2 seconds
        self._timer = self.create_timer(2, self._timerCallback)

    # member methods
    def packTwistMsg(self, vals: list) -> Twist:
        # takes input values list and packs them into a Twist message to publish
        linearVelos = Vector3()
        linearVelos.x = float(vals[0])
        linearVelos.y = float(vals[1])
        linearVelos.z = float(vals[2])

        angularVelos = Vector3()
        angularVelos.x = float(vals[3])
        angularVelos.y = float(vals[4])
        angularVelos.z = float(vals[5])

        msg = Twist()
        msg.linear = linearVelos
        msg.angular = angularVelos

        return msg

    def _timerCallback(self):
        # callback to generate a new message and publish it
        vals = [random.uniform(-12, 12) for x in range(6)]
        msg = self.packTwistMsg(vals)
        self._pub.publish(msg)


def main(args=None):
    # init rclpy
    rclpy.init(args=args)

    # creating our speed limiter node
    phNode = PubHelperNode()

    # spinning and making sure we are shutting down cleanly when we are done
    rclpy.spin(phNode)
    rclpy.shutdown()


if __name__ == "__main__":
    # just calling main
    main()
