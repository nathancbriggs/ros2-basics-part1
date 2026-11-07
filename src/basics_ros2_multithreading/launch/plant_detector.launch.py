import launch
import launch_ros.actions

def generate_launch_description():
    
    # Nodes
    plant_detector_node = launch_ros.actions.Node(
        package='basics_ros2_multithreading',
        executable='plant_detector',
        arguments=[],
        output='screen',
    )

    return launch.LaunchDescription([
        plant_detector_node
    ])