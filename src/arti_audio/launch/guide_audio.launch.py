"""
guide_audio_node 실행.

예:
ros2 launch arti_audio guide_audio.launch.py                  # 네임스페이스 = $ARTI_ROBOT_NS
ros2 launch arti_audio guide_audio.launch.py namespace:=/robot1
ros2 launch arti_audio guide_audio.launch.py player_cmd:="pw-play --volume 0.6"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('arti_audio'), 'config', 'guide_audio.yaml')
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value=EnvironmentVariable('ARTI_ROBOT_NS', default_value=''),
            description='로봇 네임스페이스 (기본: $ARTI_ROBOT_NS, 비면 네임스페이스 없음)'),
        DeclareLaunchArgument(
            'player_cmd', default_value='pw-play',
            description='재생 명령 (예: "pw-play --volume 0.6", "pw-play --target <sink>")'),
        Node(
            package='arti_audio', executable='guide_audio_node',
            namespace=LaunchConfiguration('namespace'), output='screen',
            parameters=[params, {'player_cmd': LaunchConfiguration('player_cmd')}]),
    ])
