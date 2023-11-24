from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="controller_gui",
            executable="con_gui",
            output="screen",
            arguments=["--ros-args", "--disable-external-lib-logs"]
        ),
        Node(
            package="controller_publisher",
            executable="con_pub",
            arguments=["--ros-args", "--disable-external-lib-logs"]
        )
    ])
            
