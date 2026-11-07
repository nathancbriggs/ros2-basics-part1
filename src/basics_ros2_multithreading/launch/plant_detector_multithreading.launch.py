import launch
import launch_ros.actions
from tracetools_launch.action import Trace


def generate_launch_description():
    # Nodes
    plant_detector_multithreading_node = launch_ros.actions.Node(
        package='basics_ros2_multithreading',
        executable='plant_detector_multithreading',
        arguments=[],
        output='screen',
    )

    return launch.LaunchDescription([
        plant_detector_multithreading_node
    ])