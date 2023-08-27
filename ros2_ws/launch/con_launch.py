from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="controller_gui",
            executable="con_gui",
            output="screen"
        ),
        Node(
            package="controller_publisher",
            executable="con_pub",
            output="own_log"
        )
    ])
            
