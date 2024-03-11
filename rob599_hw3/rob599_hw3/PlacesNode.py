#!/usr/bin/env python3

# Main node for this assignment
#
# PlacesNode.py
#
# Colin Mitchell

# Pull in the stuff we need from rclpy.
import os
import pickle as pkl

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.action import ActionServer

# importing a string command service type
from std_srvs.srv import Trigger
from rob599_hw3_msgs.srv import StringInput
from tf2_ros import TransformException
from visualization_msgs.msg import Marker

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rob599_hw3_msgs.action import GoCommand


class MemorizeLocsService(Node):
    def __init__(self):
        # Initialize the superclass.
        super().__init__("places")

        # Creating services for memorizing and forgetting positions
        self._memorizeSrv = self.create_service(
            StringInput, "memorize_position", self._memorizeCallback
        )
        self._clearMemSrv = self.create_service(
            Trigger, "clear_positions", self._clearCallback
        )

        # creating servics for saving and loading memorized positions
        self._saveSrv = self.create_service(StringInput, "save", self._saveCallback)
        self._loadSrv = self.create_service(StringInput, "load", self._loadCallback)

        # creating an action to allow use to specify a place to go to
        self._goToActSrv = ActionServer(self, GoCommand, 'go_to', self._goToCallback)

        # creating an action for patrol
        self._patrolActSrv = ActionServer(self, Patrol, 'patrol', self._patrolCallback)

        # creating a service to go to the front door when requested
        self._doorSrv = self.create_service(Trigger, 'knock_knock', self._knockCallback)

        # This gives us access to the navigation API.
        self._navigator = BasicNavigator()

        # Creating a publisher for markers and dictionary to store memorized positions
        self._markerPub = self.create_publisher(Marker, "mem_positions", 10)
        self._markerDict = dict() # this is a bad name, it holds the info needed to make a pose
        self._markerID = 0

        # getting this python file's directory
        self._cwd = os.path.dirname(__file__)
        self._filepath = self._cwd + f'/../resources'

    def _memorizeCallback(self, request, response):
        # making a new position marker and "memorizing it" (i.e., storing it in our dictionary)
        try:
            pose, poseDict = self.getCurrentPose()
            self._markerDict[request.data] = poseDict

            poseMarker = self.buildMarker(pose)
            textMarker = self.buildTextMarker(pose, request.data)

            self._markerPub.publish(poseMarker)
            self._markerPub.publish(textMarker)
            self.get_logger().info(
                f"memorized {request.data} with marker id {poseMarker.id}."
            )

        except TransformException as e:
            self.get_logger().info(f"Transform failed: {e}")
            return False

        return True

    def _clearCallback(self, request, response):
        self._reset()

        return True

    def _saveCallback(self, request, response):
        # reads in the request string as the file name WITHOUT the extension. assumed
        # that all files are kept in the "rob599_hw3/data/" directory.

        try:
            # putting the file path together with the file name
            filename = f'{request.data}.pkl'
            pklFile = open(f'{self._filepath}/{filename}', "wb")

            # dumping the dictionary into the file. its important to remember that we opened
            # this file for writing, so we will overwrite whatever is there.
            pkl.dump(self._markerDict, pklFile)

            # closing the file
            pklFile.close()

        except Exception as e:
            self.get_logger().info(f"Pose mem dump failed: {e}")
            return False

        return True

    def _loadCallback(self, request, response):
        # loads poses from the the dictionary and publishes them
        # clearing the current information prior to loading in the new poses
        self._reset()

        try:
            # putting the file path together with the file name
            filepath = f'{request.data}.pkl'
            pklFile = open(f'{self._filepath}/{filename}', "rb")

            # dumping the dictionary into the file. its important to remember that we opened
            # this file for writing, so we will overwrite whatever is there.
            self._markerDict = pkl.load(pklFile)

            # closing the file
            pklFile.close()

        except Exception as e:
            self.get_logger().info(f"Pose mem load failed: {e}")

            # if we failed to load anything, we make sure we have a dictionary to continue
            # using even if its empty.
            self._markerDict = dict()

            return False

        # iterating through all the entries and publishing the saved nodes. the main
        # assumption here is that the contents of the pickle file is whatever we wrote
        # out to begin with.
        for key, value in self._markerDict.items():
            pose = self.buildPoseFromDict(value)
            poseMarker = self.buildMarker(pose)
            textMarker = self.buildTextMarker(pose, key)

            self._markerPub.publish(poseMarker)
            self._markerPub.publish(textMarker)

            self.get_logger().info(
                f"Loaded {key} with marker id {textMarker.id}."
            )

        return True

    def _goToCallback(self, goal):
        # reads in the goal string and attempts to move the robot to the goal with the
        # associated key. if no key exists, then we do not move

        # getting the goal key
        goalKey = goal.request.mem_key

        # trying to find the key in our current dictionary
        if goalKey not in self._markerDict:
            result = GoCommand.Result()
            result.obj_status = "Command failed: location unknown."
            return result

        # otherwise we do have a key and we can pull the pose
        goalPose = self.buildPoseFromDict(self._markerDict[goalKey])
        self._navigator.goToPose(goalPose)

        # now we need to update our feedback with the estimated time to arrival. this
        # is gonna look really similar to the example code provided by Bill
        feedbackMsg = Countdown.Feedback()
        while not self._navigator.isTaskComplete():
            # Retrieve feedback on how we're doing.
            navFeedback = self.navigator.getFeedback()
            feedbackMsg.time_remaining = Duration.from_msg(navFeedback.estimated_time_remaining).seconds
            goal.publish_feedback(feedbackMsg)

            # We can cancel the command if it takes longer than 3 minutes to get to the goal
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                self.navigator.cancelTask()

        # checking our status for our final result
        result = GoCommand.Result()
        match self.navigator.getResult():
            case TaskResult.SUCCEEDED:
                result.obj_status = 'Command succeeded: made it to goal!'

            case TaskResult.CANCELED:
                result.obj_status = 'Command failed: mission canceled.'

            case TaskResult.FAILED:
                result.obj_status = 'Command failed: failed to reach our goal.'

            case _:
                result.obj_status = 'Unknown result: navigation feedback unknown.'

        return result

    def _patrolCallback(self, goal):
        # action server used to initiate a patrol of the waypoints we have memorized at
        # this point in time.

        # checking if we need to start or not
        if not goal.request.patrolFlag:
            result = Patrol.Result()
            result.obj_status = "Patrol status: not starting my patrol."
            return result

        # otherwise we recieved a True for the patrol flag and we need to start moving in
        # the house
        numWaypoints = len(self._markerDict.keys())
        waypointsVisited = 0

        feedbackMsg = Countdown.Feedback()
        feedbackMsg.patrol_status = 'Starting patrol.'
        goal.publish_feedback(feedbackMsg)

        for key in self._markerDict.keys():
            # getting the next pose to move to
            goalPose = self.buildPoseFromDict(self._markerDict[goalKey])

            # starting the move and keeping track of the movement progress
            self._navigator.goToPose(goalPose)

            while not self._navigator.isTaskComplete():
                # Retrieve feedback on how we're doing.
                navFeedback = self.navigator.getFeedback()

                # putting together a feedback message
                eta = Duration.from_msg(navFeedback.estimated_time_remaining).seconds
                feedbackMsg.patrol_status = f'Time to {key}: {feedbackMsg}s'
                goal.publish_feedback(feedbackMsg)

                # We can cancel the command if it takes longer than 3 minutes to get to the goal
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                    self.navigator.cancelTask()

            # checking if we made it to the waypoint or not
            match self.navigator.getResult():
                case TaskResult.SUCCEEDED:
                    feedbackMsg.patrol_status = ''

                case TaskResult.CANCELED:
                    feedbackMsg.patrol_status = 'Patrol segment cancelled, moving on with mission.'

                case TaskResult.FAILED:
                    feedbackMsg.patrol_status = 'Patrol segment failed, moving on with mission.'

                case _:
                    feedbackMsg.patrol_status = 'Unknown error occurred, moving on with mission.'

            # publishing our patrol progress after we visit each waypoint
            waypointsVisited += 1
            feedbackMsg.patrol_status += f'Patrol {(waypointsVisited / numWaypoints) * 100} complete.'
            goal.publish_feedback(feedbackMsg)

        # once we have gone to each saved waypoint, we relay that we are finished
        result = GoCommand.Result()
        result.obj_status = 'Patrol finished. All clear.'

        return result

    def _knockCallback(self, request, response):
        # trigger service, so we just try and move to the door if we have the waypoint
        if 'front_door' not in self._markerDict.keys():
            return "I don't know where that noise is coming from. Can you help me memorize 'front_door'?"

        # othewise we know where the door is and we just need to move there
        goalPose = self.buildPoseFromDict(self._markerDict['front_door'])
        self._navigator.goToPose(goalPose)

        # now we need to update our feedback with the estimated time to arrival. this
        # is gonna look really similar to the example code provided by Bill
        while not self._navigator.isTaskComplete():
            # We can cancel the command if it takes longer than 3 minutes to get to the goal
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
                self.navigator.cancelTask()

        match self.navigator.getResult():
            case TaskResult.SUCCEEDED:
                return True

            case TaskResult.CANCELED:
                return 'Finding the door took too long.'

            case TaskResult.FAILED:
                return 'I failed to find the door.'

            case _:
                return 'Something unexpected happened on my way to the door.'

    def getCurrentPose(self, frame_id):
        # Build a stamped pose for the base link origin.  If we don't set the time in the header,
        # then we get the latest TF update.
        origin = PoseStamped()
        origin.header.frame_id = "base_link"

        # Set the position.
        origin.pose.position.x = 0.0
        origin.pose.position.y = 0.0
        origin.pose.position.z = 0.0

        # Set an arbitrary orientation.
        origin.pose.orientation.x = 0.0
        origin.pose.orientation.y = 0.0
        origin.pose.orientation.z = 0.0
        origin.pose.orientation.w = 1.0

        # Get the transform to the map frame.  This will cause an exception if it fails, but we'll
        # deal with that in the calling function.
        newPose = self.tf_buffer.transform(
            origin, frame_id, rclpy.duration.Duration(seconds=1)
        )

        # packing a dictionary of values that we can serialize more easily
        poseDict = dict()
        poseDict["frame_id"] = frame_id

        poseDict["px"] = newPose.pose.position.x
        poseDict["py"] = newPose.pose.position.y
        poseDict["pz"] = newPose.pose.position.z

        poseDict["ox"] = newPose.pose.orientation.x
        poseDict["oy"] = newPose.pose.orientation.y
        poseDict["oz"] = newPose.pose.orientation.z
        poseDict["ow"] = newPose.pose.orientation.w

        return (newPose.pose, poseDict)

    def buildMarker(self, pose):
        # Make the marker.
        marker = Marker()

        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.id = self._markerID
        self._markerID += 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set the size of the sphere.  It can be oblate, so we set three scales.
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set the color.
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # Get the current pose of the robot, in the map frame.
        marker.pose = pose

        return marker

    def buildTextMarker(self, pose, text):
        # builds a line from the robot at (0,0) to the point given
        marker = Marker()
        marker.header.frame_id = "laser_link"
        marker.header.stamp = rclpy.Time.now()

        marker.type = Marker.TEXT_VIEW_FACING
        marker.id = self._markerID
        self._markerID += 1
        marker.ns = "est_text"

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = text.__str__()
        marker.pose = pose

        return marker

    def buildPoseFromDict(self, poseDict):
        # generates a new pose stamped from the contents in poseDict
        pose = PoseStamped()
        pose.header.frame_id = poseDict["frame_id"]

        pose.pose.position.x = poseDict["px"]
        pose.pose.position.y = poseDict["py"]
        pose.pose.position.z = poseDict["pz"]

        pose.pose.orientation.x = poseDict["ox"]
        pose.pose.orientation.y = poseDict["oy"]
        pose.pose.orientation.z = poseDict["oz"]
        pose.pose.orientation.w = poseDict["ow"]

        return pose

    def deleteMarkers(self):
        # deletes all markers currently active
        marker = Marker()
        marker.id = 0
        marker.action = Marker.DELETEALL

        self.pub.publish(marker)

    def _reset(self):
        # clears the dictionary of markers to print
        self._markerDict = dict()
        self._markerID = 0

        # deleting all makers
        self.deleteMarkers()


# This is the entry point for the node.
def main(args=None):
    # Initialize rclpy.
    rclpy.init(args=args)

    # The ROS2 idiom is to encapsulate everything in a class derived from Node.
    service = MemorizeLocsService()

    # Spin with the node, and explicily call shutdown() when we're done.
    rclpy.spin(service)
    rclpy.shutdown()


# This is the entry point when we call the node directly.
if __name__ == "__main__":
    main()
