#!/usr/bin/env python3

# Node used to limit the linear and rotational speed of a robot.
# This node subscribes to a topic (speed_in) and publishes a topic (speed_out). Node maintains
# two velocity constraints for the linear motion and the rotational motion.
#
# SpeedLimiter.py
#
# Colin Mitchell

# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import copy

import rclpy
from geometry_msgs.msg import Twist, Vector3
from rclpy.node import Node


# creating a class to manage the node function
class SpeedLimiterNode(Node):
    # constructor
    def __init__(self):
        # init parent (node)
        super().__init__("speed_limiter")

        # creating a subscriber and publisher for speed in and out respectively
        self._sub = self.create_subscription(Twist, "speed_in", self._subCallback, 10)
        self._pub = self.create_publisher(Twist, "speed_out", 10)

        # defining our linear and rotational velocity constaints.
        self._veloConst = 10.0

    # member methods
    def adjustVelos(self, velos: Vector3) -> Vector3:
        # checking each velocity component and adjusting if necessary

        adjVelos = copy.deepcopy(velos)

        # adjusting x
        if adjVelos.x > self._veloConst:
            adjVelos.x = self._veloConst

        # adjusting y
        if adjVelos.y > self._veloConst:
            adjVelos.y = self._veloConst

        # adjusting z
        if adjVelos.z > self._veloConst:
            adjVelos.z = self._veloConst

        return adjVelos

    def _subCallback(self, msg: Twist):
        # on each incoming message, we need to check each of the components of the linear and rotational
        # velocities and adjust them for the outgoing message.

        # building a new message with the adjusted values
        newMsg = Twist()
        newMsg.linear = self.adjustVelos(msg.linear)
        newMsg.angular = self.adjustVelos(msg.angular)

        # publishing the message
        self._pub.publish(newMsg)


def main(args=None):
    # init rclpy
    rclpy.init(args=args)

    # creating our speed limiter node
    slNode = SpeedLimiterNode()

    # spinning and making sure we are shutting down cleanly when we are done
    rclpy.spin(slNode)
    rclpy.shutdown()


if __name__ == "__main__":
    # just calling main
    main()
