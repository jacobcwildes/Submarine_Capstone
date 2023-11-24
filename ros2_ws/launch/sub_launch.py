from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="submarine_coms",
            executable="sub_coms"
        ),
        Node(
            package="image_tools",
            executable="cam2image",
            remappings=[('/image', '/camera/image')],
            arguments=[( 'device_id', '0'),
                       ('width', '640'),
                       ('height', '480'),
                       ('freq', '30')]
        )
<<<<<<< HEAD
])
=======
    ])
>>>>>>> 851654c63e455defcc5edffaa6d815c6028ce472
