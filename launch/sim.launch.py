"""Lance la simulation complète du motoculteur.

  ros2 launch agri_tractor_ros2 sim.launch.py            # avec interface Gazebo
  ros2 launch agri_tractor_ros2 sim.launch.py gui:=false # serveur seul (plus rapide)

Puis, dans un autre terminal, conduite au clavier :
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('agri_tractor_ros2')
    world = os.path.join(pkg, 'worlds', 'agri_world.sdf')
    xacro_file = os.path.join(pkg, 'urdf', 'tractor.urdf.xacro')
    bridge_cfg = os.path.join(pkg, 'config', 'bridge.yaml')
    ctrl_cfg = os.path.join(pkg, 'config', 'controller.yaml')

    gui = LaunchConfiguration('gui')
    use_ctrl = LaunchConfiguration('controller')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gz_args = PythonExpression(["'-r ", world, "' + ('' if '", gui, "' == 'true' else ' -s')"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items())

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'agri_world', '-topic', 'robot_description',
                   '-name', 'tractor', '-x', '0.0', '-y', '0.0', '-z', '0.27'])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}])

    image_bridge = Node(
        package='ros_gz_image', executable='image_bridge', output='screen',
        arguments=['/camera/image_raw'],
        parameters=[{'use_sim_time': True}])

    controller = Node(
        package='agri_tractor_ros2', executable='tractor_controller', output='screen',
        parameters=[ctrl_cfg, {'use_sim_time': True}],
        condition=IfCondition(use_ctrl))

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('controller', default_value='true'),
        gazebo, rsp, bridge, image_bridge,
        TimerAction(period=3.0, actions=[spawn]),       # laisse Gazebo démarrer
        TimerAction(period=5.0, actions=[controller]),  # après l'apparition du robot
    ])
