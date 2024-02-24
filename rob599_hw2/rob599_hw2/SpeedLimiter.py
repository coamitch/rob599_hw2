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

        # defining our linear and rotational velocity constaints. (Part 3)
        # self._veloConst = 10.0

        # defining our linear and rotational velocity constraints using parameters and
        # setting a value for them (part 4)
        self.declare_parameters(
            namespace="", parameters=[("linear_max", 10.0), ("angular_max", 10.0)]
        )

    # getters/setters
    @property
    def linMaxParam(self):
        self._linMaxParam = self.get_parameter("linear_max")
        return self._linMaxParam.value

    @property
    def angMaxParam(self):
        self._angMaxParam = self.get_parameter("angular_max")
        return self._angMaxParam.value

    # member methods
    def adjustVelos(self, velos: Vector3, type) -> Vector3:
        # checking each velocity component and adjusting if necessary

        # checking which max we need to use
        if type == "lin":
            maxVelo = self.linMaxParam
        elif type == "ang":
            maxVelo = self.angMaxParam
        else:
            maxVelo = 0.0

        # copying the incoming velocities
        adjVelos = copy.deepcopy(velos)

        # adjusting each component based on the specified max
        # adjusting x
        if abs(adjVelos.x) > maxVelo:
            if adjVelos.x < 0:
                adjVelos.x = -1 * maxVelo
            else:
                adjVelos.x = maxVelo

        # adjusting y
        if abs(adjVelos.y) > maxVelo:
            if adjVelos.y < 0:
                adjVelos.y = -1 * maxVelo
            else:
                adjVelos.y = maxVelo

        # adjusting z
        if abs(adjVelos.z) > maxVelo:
            if adjVelos.z < 0:
                adjVelos.z = -1 * maxVelo
            else:
                adjVelos.z = maxVelo

        return adjVelos

    def _subCallback(self, msg: Twist):
        # on each incoming message, we need to check each of the components of the linear and rotational
        # velocities and adjust them for the outgoing message.

        # building a new message with the adjusted values
        newMsg = Twist()
        newMsg.linear = self.adjustVelos(msg.linear, "lin")
        newMsg.angular = self.adjustVelos(msg.angular, "ang")

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
