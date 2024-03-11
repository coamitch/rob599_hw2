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
from visualization_msgs.msg import MarkerArray, Marker

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rob599_hw3_msgs.action import GoCommand, Patrol
from rclpy.duration import Duration

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf2_geometry_msgs
from ament_index_python.packages import get_package_prefix


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

        # Creating a publisher for markers and dictionary to store memorized positions
        self._markerPub = self.create_publisher(MarkerArray, "mem_positions", 10)
        self._markerDict = dict() # dictionary used to keep track of the markers we have memorized
        self._poseDict = dict() # keeps track of poses we have memorized to write out
        self._markerID = 0

        # creating a timer to publish the current memorized markers
        self._timer = self.create_timer(1, self._timerCallback)

        # creating servics for saving and loading memorized positions
        self._saveSrv = self.create_service(StringInput, "save", self._saveCallback)
        self._loadSrv = self.create_service(StringInput, "load", self._loadCallback)

        # creating an action to allow use to specify a place to go to
        self._goToActSrv = ActionServer(self, GoCommand, 'go_to', self._goToCallback)
        self._patrolActSrv = ActionServer(self, Patrol, 'patrol', self._patrolCallback)
        self._doorSrv = self.create_service(Trigger, 'knock_knock', self._knockCallback)

        # This gives us access to the navigation API and transforms
        self._navigator = BasicNavigator()
        self._tfBuffer = Buffer()
        self._tfListener = TransformListener(self._tfBuffer, self)

        # getting this python file's directory
        self._resourcePath = os.path.join(
            get_package_prefix("rob599_hw3"),
            "../../",
            "src/rob599_hw2/rob599_hw3/resource",
        )

    def _memorizeCallback(self, request, response):
        # making a new position marker and "memorizing it" (i.e., storing it in our dictionary)
        try:
            # getting the pose and poseDict to write out
            newPose, newPoseDict = self.getCurrentPose('map')
            self._poseDict[request.data] = newPoseDict

            # generating markers and storing in dictionary
            poseMarker, textMarker = self.buildMarkers(newPose, request.data)
            self._markerDict[request.data] = (poseMarker, textMarker)

            # no need to publish the marker since our timer callback takes care of it
            self.get_logger().info(
                f"memorized {request.data} with marker id {poseMarker.id}."
            )

            response.message = 'memorized position.'
            response.success = True

            return response

        except TransformException as e:
            self.get_logger().info(f"Transform failed: {e}")
            response.message = e.__str__()
            response.success = False

            return response

    def _clearCallback(self, request, response):
        # clearing the pose and marker dictionaries and resetting the marker ID counter
        self._poseDict = dict()
        self._markerDict = dict()
        self._markerID = 0

        response.message = 'Postisions forgotten.'
        response.success = True

        return response

    def _timerCallback(self):
        markerArray = MarkerArray()
        markerArray.markers = []

        for value in self._markerDict.values():
            markerArray.markers.append(value[0]) # adding the sphere marker
            markerArray.markers.append(value[1]) # adding the text marker

        # if the marker dictionary is empty, then we need to clear all the markers that
        # are currently being displace. this occurs when we have nothing memorized
        if len(markerArray.markers) == 0:
            deleteMarker = Marker()
            deleteMarker.action = Marker.DELETEALL
            markerArray.markers.append(deleteMarker)

        # publishing the array of markers we have
        self._markerPub.publish(markerArray)

    def _saveCallback(self, request, response):
        # reads in the request string as the file name WITHOUT the extension. assumed
        # that all files are kept in the "rob599_hw3/resouce/" directory.

        try:
            # putting the file path together with the file name
            filename = f'{request.data}.pkl'
            pklFile = open(f'{self._resourcePath}/{filename}', "wb")

            # dumping the pose dictionary into the file. its important to remember that we opened
            # this file for writing, so we will overwrite whatever is there. additionally, we
            # only dump the pose dict since its formatted for serialization, not the marker dict.
            pkl.dump(self._poseDict, pklFile)

            # closing the file
            pklFile.close()

            self.get_logger().info(f"Saved memorized positions to {pklFile}")

            response.message = f'Positions saved to {pklFile}.'
            response.success = True

            return response

        except Exception as e:
            self.get_logger().info(f"Pose mem dump failed: {e}")

            response.message = f'Failed to save memorized positions: {e}.'
            response.success = False

            return response

    def _loadCallback(self, request, response):
        # loads poses from the the dictionary and publishes them
        # clearing the current information prior to loading in the new poses
        self._poseDict = dict()
        self._markerDict = dict()
        self._markerID = 0

        try:
            # putting the file path together with the file name
            filename = f'{request.data}.pkl'
            pklFile = open(f'{self._resourcePath}/{filename}', "rb")

            # dumping the dictionary into the file. its important to remember that we opened
            # this file for writing, so we will overwrite whatever is there.
            self._poseDict = pkl.load(pklFile)

            # closing the file
            pklFile.close()

            # now we need to reconstruct the marker dict for publishing
            self.buildMarkerDict()

            self.get_logger().info(f"Loaded memorized positions from {pklFile}")

            response.message = f'Positions loaded from {pklFile}.'
            response.success = True

            return response

        except Exception as e:
            self.get_logger().info(f"Pose mem load failed: {e}")

            response.message = f'Failed to load saved poses: {e}.'
            response.success = False

            return response

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
        goalPose = self.buildPoseStampedFromDict(self._poseDict[goalKey])
        self._navigator.goToPose(goalPose)

        # now we need to update our feedback with the estimated time to arrival. this
        # is gonna look really similar to the example code provided by Bill
        feedbackMsg = GoCommand.Feedback()
        while not self._navigator.isTaskComplete():
            # Retrieve feedback on how we're doing.
            navFeedback = self._navigator.getFeedback()
            feedbackMsg.time_remaining = Duration.from_msg(navFeedback.estimated_time_remaining).nanoseconds / 1e9
            goal.publish_feedback(feedbackMsg)

            # We can cancel the command if it takes longer than 3 minutes to get to the goal
            if Duration.from_msg(navFeedback.navigation_time) > Duration(seconds=180.0):
                self._navigator.cancelTask()

        # checking our status for our final result
        goal.succeed()
        result = GoCommand.Result()
        match self._navigator.getResult():
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
        if not goal.request.patrol_flag:
            result = Patrol.Result()
            result.obj_status = "Patrol status: not starting my patrol."
            return result

        # otherwise we recieved a True for the patrol flag and we need to start moving in
        # the house
        numWaypoints = len(self._markerDict.keys())
        waypointsVisited = 0

        feedbackMsg = Patrol.Feedback()
        feedbackMsg.patrol_status = 'Starting patrol.'
        goal.publish_feedback(feedbackMsg)

        for key in self._markerDict.keys():
            # getting the next pose to move to
            goalPose = self.buildPoseStampedFromDict(self._poseDict[key])

            # starting the move and keeping track of the movement progress
            self._navigator.goToPose(goalPose)

            while not self._navigator.isTaskComplete():
                # Retrieve feedback on how we're doing.
                navFeedback = self._navigator.getFeedback()

                # putting together a feedback message
                eta = Duration.from_msg(navFeedback.estimated_time_remaining).nanoseconds / 1e9
                feedbackMsg.patrol_status = f'Time to {key}: {eta} s'
                goal.publish_feedback(feedbackMsg)

                # We can cancel the command if it takes longer than 3 minutes to get to the goal
                if Duration.from_msg(navFeedback.navigation_time) > Duration(seconds=180.0):
                    self._navigator.cancelTask()

            # checking if we made it to the waypoint or not
            match self._navigator.getResult():
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
            feedbackMsg.patrol_status += f'Patrol {(waypointsVisited / numWaypoints) * 100}% complete.'
            goal.publish_feedback(feedbackMsg)

            # telling a joke for extra credit
            feedbackMsg.patrol_status = 'What did the robot carry in its wallet? Cache.'
            goal.publish_feedback(feedbackMsg)

        # once we have gone to each saved waypoint, we relay that we are finished
        goal.succeed()
        result = Patrol.Result()
        result.obj_status = 'Patrol finished. All clear.'

        return result

    def _knockCallback(self, request, response):
        # trigger service, so we just try and move to the door if we have the waypoint
        if 'front_door' not in self._markerDict.keys():
            response.message = "I don't know where that noise is coming from. Can you help me memorize 'front_door'?"
            response.success = False

            return response

        # othewise we know where the door is and we just need to move there
        goalPose = self.buildPoseStampedFromDict(self._poseDict['front_door'])
        self._navigator.goToPose(goalPose)

        # now we need to update our feedback with the estimated time to arrival. this
        # is gonna look really similar to the example code provided by Bill
        while not self._navigator.isTaskComplete():
            # We can cancel the command if it takes longer than 3 minutes to get to the goal
            navFeedback = self._navigator.getFeedback()
            if Duration.from_msg(navFeedback.navigation_time) > Duration(seconds=180.0):
                self._navigator.cancelTask()

        match self._navigator.getResult():
            case TaskResult.SUCCEEDED:
                response.message = "Whose there?"
                response.success = True

            case TaskResult.CANCELED:
                response.message = 'Finding the door took too long.'
                response.success = False

            case TaskResult.FAILED:
                response.message = 'I failed to find the door.'
                response.success = False

            case _:
                response.message = 'Something unexpected happened on my way to the door.'
                response.success = False

        return response

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
        newPose = self._tfBuffer.transform(
            origin, frame_id, Duration(seconds=1)
        )

        # packing a dictionary of values that we can serialize for write out
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

    def buildMarkers(self, pose, text):
        # building the sphere marker
        sphereMarker = Marker()

        sphereMarker.header.frame_id = "map"
        sphereMarker.header.stamp = self.get_clock().now().to_msg()

        sphereMarker.id = self._markerID
        sphereMarker.type = Marker.SPHERE
        sphereMarker.action = Marker.ADD
        sphereMarker.ns = 'sphere_marker'

        # Set the size of the sphere.  It can be oblate, so we set three scales.
        sphereMarker.scale.x = 0.1
        sphereMarker.scale.y = 0.1
        sphereMarker.scale.z = 0.1

        # Set the color.
        sphereMarker.color.r = 0.0
        sphereMarker.color.g = 1.0
        sphereMarker.color.b = 1.0
        sphereMarker.color.a = 1.0

        # Get the current pose of the robot, in the map frame.
        sphereMarker.pose = pose

        # building the accompanying text marker
        textMarker = Marker()

        textMarker.header.frame_id = 'map'
        textMarker.header.stamp = self.get_clock().now().to_msg()

        textMarker.id = self._markerID
        textMarker.type = Marker.TEXT_VIEW_FACING
        textMarker.action = Marker.ADD
        textMarker.ns = 'text_marker'

        textMarker.scale.x = 0.1
        textMarker.scale.y = 0.1
        textMarker.scale.z = 0.1

        textMarker.color.r = 1.0
        textMarker.color.g = 0.0
        textMarker.color.b = 0.0
        textMarker.color.a = 1.0

        textMarker.text = text
        textMarker.pose = pose

        # chaning the z position so the text is above the sphere
        textMarker.pose.position.z = 0.2

        self._markerID += 1

        return (sphereMarker, textMarker)

    def buildPoseStampedFromDict(self, poseDict):
        # generates a new pose stamped from the contents in poseDict
        newPose = PoseStamped()
        newPose.header.frame_id = poseDict["frame_id"]

        newPose.pose.position.x = poseDict["px"]
        newPose.pose.position.y = poseDict["py"]
        newPose.pose.position.z = poseDict["pz"]

        newPose.pose.orientation.x = poseDict["ox"]
        newPose.pose.orientation.y = poseDict["oy"]
        newPose.pose.orientation.z = poseDict["oz"]
        newPose.pose.orientation.w = poseDict["ow"]

        return newPose

    def buildMarkerDict(self):
        # uses the current pose dictionary to reassemble the marker dictionary
        for key, value in self._poseDict.items():
            # building the current pose
            pose = self.buildPoseStampedFromDict(value).pose

            # generating markers for it and storing
            poseMarker, textMarker = self.buildMarkers(pose, key)
            self._markerDict[key] = (poseMarker, textMarker)

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
