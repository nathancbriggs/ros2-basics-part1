import launch
import launch_ros.actions


def generate_launch_description():
    # Nodes
    green_detector_node_node = launch_ros.actions.Node(
        package='basics_ros2_multithreading',
        executable='green_detector_node',
        arguments=[],
        output='screen',
    )

    return launch.LaunchDescription([
        green_detector_node_node
    ])