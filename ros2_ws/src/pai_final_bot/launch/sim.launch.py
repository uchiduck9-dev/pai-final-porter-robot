"""Ignition Gazebo のアリーナにロボットを出し、ROS 2 とブリッジする。

使い方:
  ros2 launch pai_final_bot sim.launch.py              # シミュレーションのみ
  ros2 launch pai_final_bot sim.launch.py teleop:=true # キーボード操作用の xterm も開く
  ros2 launch pai_final_bot sim.launch.py headless:=true # 画面を出さずに動かす（テスト用）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('pai_final_bot')
    world = os.path.join(share, 'worlds', 'arena.sdf')
    models = os.path.join(share, 'models')
    resource_path = models + os.pathsep + os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')

    teleop = LaunchConfiguration('teleop')
    headless = LaunchConfiguration('headless')

    return LaunchDescription([
        DeclareLaunchArgument('teleop', default_value='false',
                              description='キーボード操作ノードを xterm で起動する'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Gazebo の画面を出さずにシミュレーションだけ動かす'),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),

        ExecuteProcess(cmd=['ign', 'gazebo', '-r', world], output='screen',
                       condition=UnlessCondition(headless)),
        ExecuteProcess(cmd=['ign', 'gazebo', '-s', '-r', world], output='screen',
                       condition=IfCondition(headless)),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            parameters=[{
                'config_file': os.path.join(share, 'config', 'bridge.yaml'),
                'use_sim_time': True,
            }],
            output='screen',
        ),

        # ロボットの DetachableJoint は起動時にスーツケースを固定してしまうため、起動後に解除しておく
        TimerAction(period=3.0, actions=[
            ExecuteProcess(cmd=['ros2', 'topic', 'pub', '--times', '3', '/porter/detach',
                                'std_msgs/msg/Empty', '{}'], output='log'),
        ]),

        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            prefix='xterm -e',
            condition=IfCondition(teleop),
            output='screen',
        ),
    ])
