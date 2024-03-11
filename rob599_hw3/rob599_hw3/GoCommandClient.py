#!/usr/bin/env python3

# Client side for sending waypoints to the base node for the assignment
#
# GoCommandClient.py
#
# Colin Mitchell


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from rob599_hw3_msgs.action import GoCommand


class GoCommandClient(Node):
    def __init__(self):
        super().__init__("go_to_client")

        self._client = ActionClient(self, GoCommand, 'go_to')

    def sendGoal(self, mem_key: string):
        goal = GoCommand.Goal()
        goal.mem_key = mem_key

        self._client.wait_for_server()

        self._result = self._client.send_goal_async(
            goal, feedback_callback=self._cdFeedback
        )

        self._result.add_done_callback(self._jaCallback)

    def _cdFeedback(self, feedback):
        # logging the feedback we get from the action server
        self.get_logger().info(f"Eta: {feedback.feedback.time_remaining}")

    # fires when the job is accepted
    def _jaCallback(self, result):
        goal = result.result()

        if not goal.accepted:
            self.get_logger().info("Waypoint rejected...")
        else:
            self.get_logger().info("Waypoint recieved...")

        self._resultHandle = goal.get_result_async()
        self._resultHandle.add_done_callback(self._jdCallback)

    # fires when there is a result to process (job is done)
    def _jdCallback(self, future):
        result = future.result().result

        self.get_logger().info(result.obj_status)

    def requestMemKey(self):
        # function used to loop and get mem_keys from user input
        done = False

        while not done:
            rslt = input('Please provide a mem key: ')

            self.sendGoal(rslt)

def main(args=None):
    rclpy.init(args=args)

    client = GoCommandClient()

    client.requestMemKey()

    rclpy.spin(client)


if __name__ == "__main__":
    main()
