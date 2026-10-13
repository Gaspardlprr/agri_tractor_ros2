"""Lance le nœud Visual Teach & Repeat (la simulation doit déjà tourner).

  ros2 launch agri_tractor_ros2 vtr.launch.py mode:=view
  ros2 launch agri_tractor_ros2 vtr.launch.py mode:=teach  path:=~/vtr_paths/rang1
  ros2 launch agri_tractor_ros2 vtr.launch.py mode:=repeat path:=~/vtr_paths/rang1

Visualiser : ros2 run rqt_image_view rqt_image_view /vtr/debug_image
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('agri_tractor_ros2'), 'config', 'vtr.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='view',
                              description='view | teach | repeat'),
        DeclareLaunchArgument('path', default_value='~/vtr_paths/path1'),
        Node(package='agri_tractor_ros2', executable='vtr_node', output='screen',
             parameters=[cfg, {'use_sim_time': True,
                               'mode': LaunchConfiguration('mode'),
                               'path_dir': LaunchConfiguration('path')}]),
    ])
