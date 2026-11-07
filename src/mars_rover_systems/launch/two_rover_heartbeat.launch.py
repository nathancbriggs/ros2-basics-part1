from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
  return LaunchDescription([
    Node(
      package = 'mars_rover_systems',
      executable = 'heartbeat_executable',
      output = 'screen'),
    Node(
      package = 'mars_rover_systems',
      executable = 'heartbeat_executable2',
      output = 'screen')
  ])