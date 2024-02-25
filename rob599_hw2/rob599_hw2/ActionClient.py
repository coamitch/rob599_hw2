#!/usr/bin/env python3

# Client side for nasa (part 10)
#
# ActionClient.py
#
# Colin Mitchell


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rob599_hw2_msgs.action import Countdown


class NasaLaunchClientNode(Node):
    def __init__(self):
        super().__init__("flight_control")

        self._client = ActionClient(self, Countdown, "launch_rocket")

    def sendGoal(self, startTimer: int):
        goal = Countdown.Goal()
        goal.starting_count = startTimer

        self._client.wait_for_server()

        self._result = self._client.send_goal_async(
            goal, feedback_callback=self._cdFeedback
        )

        self._result.add_done_callback(self._jaCallback)

    def _cdFeedback(self, feedback):
        # logging the feedback we get from the action server (this is just the countdown)
        self.get_logger().info(f"T-minus: {feedback.feedback.time_to_launch}")

    # fires when the job is accepted
    def _jaCallback(self, result):
        goal = result.result()

        if not goal.accepted:
            self.get_logger().info("Countdown hault...")
        else:
            self.get_logger().info("Countdown commencing...")

        self._resultHandle = goal.get_result_async()
        self._resultHandle.add_done_callback(self._dCallback)

    # fires when there is a result to process (job is done)
    def _dCallback(self, future):
        result = future.result().result

        self.get_logger().info(result.launched)


def main(args=None):
    rclpy.init(args=args)

    client = NasaLaunchClientNode()

    client.sendGoal(10)

    rclpy.spin(client)


if __name__ == "__main__":
    main()
