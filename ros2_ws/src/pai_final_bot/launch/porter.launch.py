"""ポーターロボットのノードを起動する（シミュレーションは sim.launch.py で別に起動しておく）。

使い方:
  ros2 launch pai_final_bot porter.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pai_final_bot',
            executable='porter_node',
            parameters=[{'use_sim_time': True}],
            output='screen',
            emulate_tty=True,
        ),
    ])
