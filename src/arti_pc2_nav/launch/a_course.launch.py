"""ArtI PC2 코스 실행 launch: 코스 노드 + TTS 노드 + RViz 반원 마커 노드.

Gazebo/Nav2/localization 자체는 띄우지 않습니다 (따로 실행되어 있어야 함).
안전을 위해 enable_motion 기본값은 false(주행 잠금)입니다.

실물 로봇 실행 예:
  ros2 launch arti_pc2_nav a_course.launch.py robot_namespace:=robot4 \\
    enable_motion:=true simulation_mode:=false explanation_transport:=topic \\
    start_tts:=true tts_engine:=gtts use_sim_time:=false require_dock_status:=true
"""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = Path(get_package_share_directory('arti_pc2_nav'))
    return LaunchDescription([
        # ---------------- launch 인자 ----------------
        DeclareLaunchArgument('robot_namespace', default_value='robot4'),   # 토픽 namespace
        DeclareLaunchArgument('config_file', default_value=str(pkg / 'config/a_course.yaml')),  # 좌표 설정
        DeclareLaunchArgument('enable_motion', default_value='false'),      # true여야 실제 주행
        DeclareLaunchArgument('simulation_mode', default_value='false'),    # true면 해설 2초 모의
        DeclareLaunchArgument('use_sim_time', default_value='false'),       # Gazebo면 true
        DeclareLaunchArgument('skip_undock', default_value='false'),        # 시뮬레이션 전용
        DeclareLaunchArgument('simulate_docking', default_value='false'),   # 시뮬레이션 전용 모의 도킹
        DeclareLaunchArgument('require_dock_status', default_value='true'), # Dock 후 is_docked 확인
        DeclareLaunchArgument('simulation_battery_percent', default_value='-1.0'),
        # [실물 로봇 추가] TTS 관련
        DeclareLaunchArgument('explanation_transport', default_value='topic'),  # 'topic' = hardcoded_tts_node 사용
        DeclareLaunchArgument('start_tts', default_value='true'),           # TTS 노드 같이 실행
        DeclareLaunchArgument('tts_engine', default_value='gtts'),          # 'gtts' 또는 'espeak'

        # ---------------- 코스 관람 노드 ----------------
        Node(package='arti_pc2_nav', executable='a_course_node', name='a_course_node',
             namespace=LaunchConfiguration('robot_namespace'), output='screen',
             # TurtleBot4는 /robot4/tf 를 쓰므로 namespace 안의 tf로 연결
             remappings=[('/tf','tf'),('/tf_static','tf_static')],
             parameters=[{
                 'config_file': LaunchConfiguration('config_file'),
                 'enable_motion': ParameterValue(LaunchConfiguration('enable_motion'), value_type=bool),
                 'simulation_mode': ParameterValue(LaunchConfiguration('simulation_mode'), value_type=bool),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'skip_undock': ParameterValue(LaunchConfiguration('skip_undock'), value_type=bool),
                 'simulate_docking': ParameterValue(LaunchConfiguration('simulate_docking'), value_type=bool),
                 'require_dock_status': ParameterValue(LaunchConfiguration('require_dock_status'), value_type=bool),
                 'simulation_battery_percent': ParameterValue(LaunchConfiguration('simulation_battery_percent'), value_type=float),
                 'explanation_transport': LaunchConfiguration('explanation_transport'),
             }]),

        # ---------------- TTS 노드 (start_tts:=true 일 때만) ----------------
        Node(package='arti_pc2_nav', executable='hardcoded_tts_node', name='hardcoded_tts_node',
             namespace=LaunchConfiguration('robot_namespace'), output='screen',
             condition=IfCondition(LaunchConfiguration('start_tts')),
             parameters=[{'engine': LaunchConfiguration('tts_engine')}]),

        # ---------------- RViz 반원 마커 노드 ----------------
        Node(package='arti_pc2_nav', executable='zone_marker_node', name='arti_zone_markers',
             namespace=LaunchConfiguration('robot_namespace'), output='screen',
             parameters=[{
                 'config_file': LaunchConfiguration('config_file'),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'map_frame': 'map',
             }]),
    ])
