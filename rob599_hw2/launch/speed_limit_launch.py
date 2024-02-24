import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription(
        [
            # launching the Twist message publisher
            launch_ros.actions.Node(package="rob599_hw2", executable="pub_helper"),
            # launching the speed limiter node
            launch_ros.actions.Node(package="rob599_hw2", executable="speed_limiter"),
            # launching the stat tracker
            launch_ros.actions.Node(package="rob599_hw2", executable="msg_checker"),
        ]
    )
