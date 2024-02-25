#!/usr/bin/env python3

# Server side for nasa (part 10)
#
# ActionServer.py
#
# Colin Mitchell

import time

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from rob599_hw2_msgs.action import Countdown


class NasaLaunchServerNode(Node):
    def __init__(self):
        # init super class
        super().__init__("nasa")

        # setting up the action server
        self._server = ActionServer(self, Countdown, "launch_rocket", self._asCallback)

    def _asCallback(self, goal):
        # callback for the action server

        # starting count
        startingCount = goal.request.starting_count

        # setting up the feedback msg and init
        feedbackMsg = Countdown.Feedback()

        # counting down (i.e. publishing our feedback)
        for i in range(startingCount, 0):
            feedbackMsg.time_to_launch = i
            goal.publish_feedback(feedbackMsg)
            time.sleep(1)

        # finished counting down, so we need to send a launched message
        result = Countdown.Result()
        result.launch_status = "..., 2, 1, 0, all engines running ... LIFT OFF! We have liftoff! Tower cleared."

        return result


def main(args=None):
    # init rclpy
    rclpy.init(args=args)

    # creating our speed limiter node
    nasaNode = NasaLaunchServerNode()

    # spinning and making sure we are shutting down cleanly when we are done
    rclpy.spin(nasaNode)
    rclpy.shutdown()


if __name__ == "__main__":
    # just calling main
    main()
