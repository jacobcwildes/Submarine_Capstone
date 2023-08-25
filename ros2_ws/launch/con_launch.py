from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="controller_gui",
            executable="con_gui"
        ),
        Node(
            package="controller_publisher",
            executable="con_pub"
        )
    ])
            
