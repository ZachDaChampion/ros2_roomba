import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

  create_2_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(get_package_share_directory('create_bringup'), 'launch', 'create_2.launch')
    )
  )

  simple_controller = Node(
    package='simple_controller',
    executable='simplecontroller',
    name='simplecontroller',
  )

  odom_printer = Node(
    package='simple_controller',
    executable='odom_printer',
    name='odom_printer',
  )

  return LaunchDescription([
    create_2_launch,
    simple_controller,
    odom_printer,
  ])