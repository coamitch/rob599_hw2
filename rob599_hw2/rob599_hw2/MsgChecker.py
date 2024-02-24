#!/usr/bin/env python3

# Node used to check each of the individual values and make sure they are within some
# limit. additionally logs how many messages its recieved and how many were "valid".
#
# MsgChecker.py
#
# Colin Mitchell

# Every Python node in ROS2 should include these lines.  rclpy is the basic Python
# ROS2 stuff, and Node is the class we're going to use to set up the node.
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


# creating a class to manage the node function
class MsgCheckerNode(Node):
    # constructor
    def __init__(self):
        # init parent (node)
        super().__init__("msg_checker")

        # creating a subscriber to read the speed out topic
        self._speedOutSub = self.create_subscription(
            Twist, "speed_out", self._outCallback, 10
        )
        self._speedInSub = self.create_subscription(
            Twist, "speed_in", self._inCallback, 10
        )

        # defining our linear and rotational velocity constraints using parameters and
        # setting a value for them
        self.declare_parameters(
            namespace="",
            parameters=[("linear_check_max", 10.0), ("angular_check_max", 10.0)],
        )

        self._timer = self.create_timer(30, self._timerCallback)

        self._resetCounts()

    # getters/setters
    @property
    def linCheckParam(self):
        self._linCheckParam = self.get_parameter("linear_check_max")
        return self._linCheckParam.value

    @property
    def angCheckParam(self):
        self._angCheckParam = self.get_parameter("angular_check_max")
        return self._angCheckParam.value

    # member methods
    def _resetCounts(self):
        self._inMsgCount = 0
        self._inValidCount = 0

        self._outMsgCount = 0
        self._outValidCount = 0

    def _checkMsg(self, msg: Twist) -> bool:
        # checks the message and determines if the values are within the set parameters
        if abs(msg.linear.x) > self.linCheckParam:
            return False
        elif abs(msg.linear.y) > self.linCheckParam:
            return False
        elif abs(msg.linear.z) > self.linCheckParam:
            return False

        if abs(msg.angular.x) > self.angCheckParam:
            return False
        elif abs(msg.angular.y) > self.angCheckParam:
            return False
        elif abs(msg.angular.z) > self.angCheckParam:
            return False

        return True

    def _outCallback(self, msg: Twist):
        # callback used to check if the message is "valid" or not and keep counts

        # message recieved, so adding to the tally
        self._outMsgCount += 1

        # checking if the message is valid
        if self._checkMsg(msg):
            self._outValidCount += 1

    def _inCallback(self, msg: Twist):
        # callback used to check if the message is "valid" or not and keep counts

        # message recieved, so adding to the tally
        self._inMsgCount += 1

        # checking if the message is valid
        if self._checkMsg(msg):
            self._inValidCount += 1

    def _timerCallback(self):
        # logging stats for each topic
        self.get_logger().info(
            "/speed_in stats: {0}/{1} valid messages recieved.".format(
                self._inValidCount, self._inMsgCount
            )
        )

        self.get_logger().info(
            "/speed_out stats: {0}/{1} valid messages recieved.".format(
                self._outValidCount, self._outMsgCount
            )
        )

        self._resetCounts()


def main(args=None):
    # init rclpy
    rclpy.init(args=args)

    # creating our speed limiter node
    mcNode = MsgCheckerNode()

    # spinning and making sure we are shutting down cleanly when we are done
    rclpy.spin(mcNode)
    rclpy.shutdown()


if __name__ == "__main__":
    # just calling main
    main()
