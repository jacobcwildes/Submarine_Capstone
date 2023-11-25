from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy import qos

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="submarine_coms",
            executable="sub_coms",
            arguments=["--ros-args", "--disable-external-lib-logs"]
        ),
        Node(
            package="image_tools",
            executable="cam2image",
            remappings=[('/image', '/camera/image')],
            arguments=[( 'device_id', '0'),
                       ('width', '640'),
                       ('height', '480'),
                       ('frequency', '15'),
                       ('reliability', 'qos.qos_profile_sensor_data'), 
                       ("--ros-args", "--disable-external-lib-logs")]#,
            #qos_profile=qos.qos_profile_sensor_data
        )
])

