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
from rclpy.parameter import Parameter


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
        # setting a value for them (part 4) as well as the watch dog parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("linear_max", 10.0),
                ("angular_max", 10.0),
                ("with_watchdog", True),
                ("watchdog_period", 5),
            ],
        )

        # watch dog timer set to parameter time.
        self._wdTimer = self.create_timer(self.watchdogPeriodParam, self._wdCallback)

    # getters/setters
    @property
    def linMaxParam(self):
        self._linMaxParam = self.get_parameter("linear_max")
        return self._linMaxParam.value

    @property
    def angMaxParam(self):
        self._angMaxParam = self.get_parameter("angular_max")
        return self._angMaxParam.value

    @property
    def watchdogPeriodParam(self):
        # this is a little long for an attribute name, but its just for the timer
        self._watchdogPeriodParam = self.get_parameter("watchdog_period")
        return self._watchdogPeriodParam.value

    @property
    def withWatchdogParam(self):
        self._withWatchdogParam = self.get_parameter("with_watchdog")
        return self._withWatchdogParam.value

    @withWatchdogParam.setter
    def withWatchdogParam(self, flag: bool):
        param = Parameter("with_watchdog", rclpy.Parameter.Type.BOOL, flag)
        paramList = [param]
        self.set_parameters(paramList)

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

        # clearing our watchdog flag since we got a message
        self.withWatchdogParam = False

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

    def _wdCallback(self):
        # in this timer callback, we just need to check if the parameter bool is true or
        # not. this is based on how long its been since the last incoming message.

        if self.withWatchdogParam:
            # our bool is true so we send a zero velo Twist message and leave the flag as
            # is since its already true.
            msg = self.packTwistMsg([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self._pub.publish(msg)

        else:
            # otherwise, our message was false (meaning we saw a message in the period time)
            # and we reset our flag
            self.withWatchdogParam = True


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
