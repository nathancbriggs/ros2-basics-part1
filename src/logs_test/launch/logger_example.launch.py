from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='logs_test',
            executable='logger_example',
            name='logger_example',
            emulate_tty=True,
            output='screen',
        ),
    ])