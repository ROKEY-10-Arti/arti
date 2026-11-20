"""ArtI PC2 코스 실행 launch: 코스 노드 + 음성(arti_audio) + 사람 인식(person_tracker) + RViz 반원 마커 노드.

Nav2/localization 자체는 띄우지 않습니다 (따로 실행되어 있어야 함).
안전을 위해 enable_motion 기본값은 false(주행 잠금)입니다.

음성 흐름 (모두 robot_namespace 안):
  a_course_node ──explain_artwork(action, 작품 번호)──▶ guide_audio_node ──/arti/description──▶ PC3
  a_course_node ──guide_message(topic, 안내 키)──────▶ guide_audio_node   (키 = arti_audio guide_lines.yaml)

사람 인식 흐름 (prepare_tour 등록 단계, tracker:=true):
  노트북 웹캠(video_source 0) ─camera_pub─▶ top_cam/image_raw/compressed ─gesture_test_node(주먹→손바닥으로 대상 지정, 클릭도 됨)
  ─▶ target_person/point_map ─▶ a_course_node (1초 끊김 없이 보이면 등록)

키오스크(PC3) 대신 직접 호출해 보기:
  ros2 action send_goal /robot4/arti/prepare_tour arti_interfaces/action/PrepareTour "{start: true}" --feedback

실물 로봇 실행 예:
  ros2 launch arti_pc2_nav a_course.launch.py robot_namespace:=robot4 \\
    enable_motion:=true simulation_mode:=false use_sim_time:=false require_dock_status:=true
  음성 없이:  audio:=false   /  재생 명령:  player_cmd:="pw-play --volume 0.6"
  인식 없이:  tracker:=false skip_register:=true watch_visitor:=false   (등록·거리 감시 생략)

카메라: robot4는 앞장서고 관람객이 뒤에서 따라오므로 노트북 웹캠을 '뒤쪽'을 보게 올린다 (follow팀과 반대).
  camera_yaw:=180 (기본) 이 카메라 TF(top_cam_tf, yaw)와 인식 노드(cam_yaw) 둘 다에 들어간다.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = Path(get_package_share_directory('arti_pc2_nav'))
    audio_share = get_package_share_directory('arti_audio')
    tracker_share = get_package_share_directory('person_tracker')
    tracker_on = IfCondition(LaunchConfiguration('tracker'))
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
        DeclareLaunchArgument('audio', default_value='true'),               # guide_audio_node 같이 실행
        DeclareLaunchArgument('player_cmd', default_value='pw-play'),       # 재생 명령 (음량·출력 장치)
        DeclareLaunchArgument('tracker', default_value='true'),             # person_tracker(웹캠+인식) 같이 실행
        DeclareLaunchArgument('skip_register', default_value='false'),      # true면 인식 없이 등록 (시험용)
        DeclareLaunchArgument('watch_visitor', default_value='true'),       # 관람 중 관람객 거리 감시 (멀면 정지·안내)
        DeclareLaunchArgument('camera_yaw', default_value='180.0'),         # 웹캠 방향 [deg] (robot4: 뒤쪽 = 180)
        DeclareLaunchArgument('tracker_node', default_value='gesture_test_node'),  # 인식 노드: gesture_test_node(제스처+클릭) | multi_tracker_node(클릭만)

        # ---------------- 코스 주행 노드 ----------------
        Node(package='arti_pc2_nav', executable='a_course_node', name='a_course_node',
             namespace=LaunchConfiguration('robot_namespace'), output='screen',
             # TurtleBot4는 /robot4/tf 를 쓰므로 namespace 안의 tf로 연결
             remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
             parameters=[{
                 'config_file': LaunchConfiguration('config_file'),
                 'enable_motion': ParameterValue(LaunchConfiguration('enable_motion'), value_type=bool),
                 'simulation_mode': ParameterValue(LaunchConfiguration('simulation_mode'), value_type=bool),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'skip_undock': ParameterValue(LaunchConfiguration('skip_undock'), value_type=bool),
                 'simulate_docking': ParameterValue(LaunchConfiguration('simulate_docking'), value_type=bool),
                 'require_dock_status': ParameterValue(LaunchConfiguration('require_dock_status'), value_type=bool),
                 'simulation_battery_percent': ParameterValue(LaunchConfiguration('simulation_battery_percent'), value_type=float),
                 'skip_register': ParameterValue(LaunchConfiguration('skip_register'), value_type=bool),
                 'watch_visitor': ParameterValue(LaunchConfiguration('watch_visitor'), value_type=bool),
             }]),

        # ---------------- 음성 노드 (audio:=true 일 때만) ----------------
        # arti_audio 패키지의 launch를 그대로 사용. namespace를 넘겨야 /robot4/explain_artwork 가 생김
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(audio_share, 'launch', 'guide_audio.launch.py')),
            launch_arguments={'namespace': LaunchConfiguration('robot_namespace'),
                              'player_cmd': LaunchConfiguration('player_cmd')}.items(),
            condition=IfCondition(LaunchConfiguration('audio'))),

        # ---------------- 사람 인식 (tracker:=true 일 때만) ----------------
        # arti_follow와 같은 구성: 카메라(웹캠, person_tracker/config/phone_camera.yaml) + 인식 노드.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(tracker_share, 'launch', 'phone_camera.launch.py')),
            launch_arguments={'namespace': LaunchConfiguration('robot_namespace'),
                              'yaw': LaunchConfiguration('camera_yaw')}.items(),   # 카메라 TF 방향
            condition=tracker_on),
        TimerAction(period=2.0, condition=tracker_on, actions=[           # 카메라가 먼저 켜지게
            Node(package='person_tracker', executable=LaunchConfiguration('tracker_node'),
                 name='multi_person_tracker_node', namespace=LaunchConfiguration('robot_namespace'),
                 output='screen',
                 parameters=[{# 인식 노드도 같은 카메라 방향으로 LiDAR와 맞춤 (안 넣으면 yaml의 0 = 앞쪽)
                              'cam_yaw': ParameterValue(LaunchConfiguration('camera_yaw'), value_type=float)}])]),

        # ---------------- RViz 반원 마커 노드 ----------------
        Node(package='arti_pc2_nav', executable='zone_marker_node', name='arti_zone_markers',
             namespace=LaunchConfiguration('robot_namespace'), output='screen',
             parameters=[{
                 'config_file': LaunchConfiguration('config_file'),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'map_frame': 'map',
             }]),
    ])
