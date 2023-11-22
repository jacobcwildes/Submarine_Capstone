from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="submarine_coms",
            executable="sub_coms"
        ),
        Node(
            package="imagetools",
            executable="cam2image",
            remappings=[('/image', '/camera/image')]
            arguments=[( 'device_id', '0'),
                       ('width', '640'),
                       ('height', '480'),
                       ('freq', '30')]
    ])
