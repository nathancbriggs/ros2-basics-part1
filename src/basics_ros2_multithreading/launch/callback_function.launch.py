import launch
import launch_ros.actions

def generate_launch_description():
    
    # Nodes
    callback_function_node = launch_ros.actions.Node(
        package='basics_ros2_multithreading',
        executable='callback_function',
        arguments=[],
        output='screen',
    )

    return launch.LaunchDescription([
        callback_function_node
    ])