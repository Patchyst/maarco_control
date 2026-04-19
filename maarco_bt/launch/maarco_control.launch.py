from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='heading_controller',
            executable='heading_controller',
            name='heading_controller',
            output='screen'
        ),
        Node(
            package='motor_driver',
            executable='motor_driver',
            name='motor_driver',
            output='screen'
        ),
        Node(
            package='serial_reader',
            executable='serial_node',
            name='serial_node',
            output='screen'
        ),
    ])