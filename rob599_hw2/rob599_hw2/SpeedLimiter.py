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
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
import copy

# creating a class to manage the node function
class SpeedLimiterNode(Node):
  # constructor
  def __init__(self):
    # init parent (node)
    super().__init__('speed_limiter')

    # creating a subscriber and publisher for speed in and out respectively
    self._sub = self.create_subscription(Twist, 'speed_in', self._subCallback, 10)
    self._pub = self.create_publisher(Twist, 'speed_out', 10)

    # defining our linear and rotational velocity constaints.
    self._linVeloConstX = 10.0
    self._linVeloConstY = 10.0
    self._linVeloConstZ = 10.0

    self._angVeloConstX = 10.0
    self._angVeloConstY = 10.0
    self._angVeloConstZ = 10.0

  # member methods
  def adjustLinVelos(self, linVelos: Vector3) -> Vector3:
    # checking each linear velocity component adjusting the velocities if necessary

    adjLinVelos = copy.deepcopy(linVelos)

    # adjusting x
    if adjLinVelos.x > self._linVeloConstX:
      adjLinVelos.x = self._linVeloConstX

    # adjusting y
    if adjLinVelos.y > self._linVeloConstY:
      adjLinVelos.y = self._linVeloConstY

    # adjusting z
    if adjLinVelos.z > self._linVeloConstZ:
      adjLinVelos.z = self._linVeloConstZ

    return adjLinVelos

  def adjustAngVelos(self, angVelos: Vector3) -> Vector3:
    # checking each linear velocity component adjusting the velocities if necessary
    adjAngVelos = copy.deepcopy(angVelos)

    # adjusting x
    if adjAngVelos.x > self._angVeloConstX:
      adjAngVelos.x = self._angVeloConstX

    # adjusting y
    if adjAngVelos.y > self._angVeloConstY:
      adjAngVelos.y = self._angVeloConstY

    # adjusting z
    if adjAngVelos.z > self._angVeloConstZ:
      adjAngVelos.z = self._angVeloConstZ

    return adjAngVelos

  def _subCallback(self, msg: Twist):
    # on each incoming message, we need to check each of the components of the linear and rotational
    # velocities and adjust them for the outgoing message.

    # building a new message with the adjusted values
    newMsg = Twist()
    newMsg.linear = self.adjustLinVelos(msg.linear)
    newMsg.angular = self.adjustAngVelos(msg.angular)

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

if __name__ == '__main__':
  # just calling main
  main()



