"""[코스 관람] ArtI PC2 코스 실행 launch (터미널 하나, 변경 ①~⑩은 a_course_node.py 맨 위 목록): localization + Nav2 + 코스 노드 + 음성(arti_audio)
+ 사람 인식(person_tracker) + RViz 구역 마커. 로봇은 도크에 붙인 채로 실행한다.

  nav:=true (기본)  0 s localization — arti_bringup localization.launch.py (follow 팀과 같음),
                        AMCL 시작 위치 = arti_common init_pose.yaml /robot4.undock
                    5 s Nav2 — config/nav2_real.yaml, autostart=false (노드만 뜨고 활성화는 언독 후 코스 노드가 요청)
  언독이 끝나면 코스 노드가 init_pose.yaml undock을 initialpose로 보낸다 (= RViz 2D Pose Estimate 자동).
  nav:=false        localization·Nav2를 따로 띄웠을 때 (코스 노드·음성·인식만)
  도킹 중엔 절전으로 LiDAR가 꺼져 AMCL이 map→odom TF를 안 낸다 → Nav2 global_costmap이 활성화되지 못한다.
  그래서 호출 → 언독 → initialpose → scan·TF가 들어오면 코스 노드가 Nav2 STARTUP을 요청하고,
  scan·AMCL·TF·Nav2를 최대 60 s 기다렸다 출발한다. 안 되면 그 자리에서 다시 도킹한다.
안전을 위해 enable_motion 기본값은 false(주행 잠금)입니다.

음성 흐름 (모두 robot_namespace 안):
  a_course_node ──explain_artwork(action, 작품 번호)──▶ guide_audio_node ──/arti/description──▶ PC3
  a_course_node ──guide_message(topic, 안내 키)──────▶ guide_audio_node   (키 = arti_audio guide_lines.yaml)

사람 인식 흐름 (prepare_tour 등록 단계, tracker:=true):
  노트북 웹캠(video_source 0) ─(직접, direct_camera:=true 기본)─▶ gesture_test_node(주먹→손바닥으로 대상 지정, 클릭도 됨)
  (direct_camera:=false면 전처럼 camera_pub ─▶ top_cam/image_raw/compressed ─▶ 인식 노드)
  ─▶ target_person/point_map ─▶ a_course_node (1초 끊김 없이 보이면 등록)

키오스크(PC3) 대신 직접 호출해 보기:
  ros2 action send_goal /robot4/arti/prepare_tour arti_interfaces/action/PrepareTour "{start: true}" --feedback

실물 로봇 실행 예 (이것 하나로 전부):
  ros2 launch arti_pc2_nav a_course.launch.py enable_motion:=true
  지도:  map:=<yaml> (기본 워크스페이스 maps/gallery.yaml)   /   RViz는 필요할 때 따로
  음성 없이:  audio:=false   /  재생 명령:  player_cmd:="pw-play --volume 0.6"
  인식 없이:  tracker:=false skip_register:=true watch_visitor:=false   (등록·거리 감시 생략)

카메라: robot4는 앞장서고 관람객이 뒤에서 따라오므로 노트북 웹캠을 '뒤쪽'을 보게 올린다 (follow팀과 반대).
  camera_yaw:=180 (기본) 이 카메라 TF(top_cam_tf, yaw)와 인식 노드(cam_yaw) 둘 다에 들어간다.
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetParameter, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
import yaml


def _localization_and_nav2(context):
    """[코스 관람 변경 ①②] nav:=true: localization(arti_bringup, follow 팀과 같은 launch) → 5 s 뒤 Nav2(nav2_real.yaml).
    전: 터미널 3개(localization, RViz 2D Pose Estimate, Nav2)를 손으로 순서대로 띄움 → 지금: 이 launch 하나.
    AMCL 시작 위치 = arti_common init_pose.yaml robots./robot4.undock (arti_bringup이 넣음).
    언독이 끝나면 코스 노드가 같은 값을 initialpose로 한 번 더 보낸다 (도킹 중엔 LiDAR가 꺼져 있어서)."""
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    if arg('nav').lower() not in ('true', '1'):
        return []
    ns = '/' + arg('robot_namespace').strip('/')
    map_file = arg('map') or os.environ.get('ARTI_MAP', '') or os.path.join(
        os.path.dirname(os.path.dirname(get_package_prefix('arti_pc2_nav'))), 'maps', 'gallery.yaml')
    if not os.path.isfile(map_file):
        raise RuntimeError(f'지도 파일이 없음: {map_file} (map:=<yaml>로 지정)')
    bringup = get_package_share_directory('arti_bringup')
    tb4_nav = get_package_share_directory('turtlebot4_navigation')
    pkg = get_package_share_directory('arti_pc2_nav')
    return [
        # [코스 관람 변경 ⑥] localization lifecycle_manager의 bond_timeout은 nav2_bringup이 params 파일로 안 넘겨서 기본 4 s.
        # 서버(PC3) 경유 discovery가 느리면 "map_server was unable to be reached after 4.00s by bond"로 포기한다 (2026-11-20)
        # → 이 그룹 안 노드에만 bond_timeout 30 s (Nav2 nav2_real.yaml과 같음). arti_bringup은 그대로.
        GroupAction([
            SetParameter(name='bond_timeout', value=30.0),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup, 'launch', 'localization.launch.py')),
                launch_arguments={'namespace': ns, 'map': map_file, 'start_point': 'undock'}.items()),
        ]),
        # [코스 관람 변경 ⑤] Nav2: turtlebot4_navigation nav2.launch.py와 같은 구성(namespace, costmap scan remap)인데 autostart만 false.
        #   (nav2.launch.py에는 autostart 인자가 없어서 navigation_launch.py를 직접 부름)
        #   왜: 도킹 중엔 LiDAR가 꺼져 AMCL이 map→odom TF를 안 낸다. 그 상태로 켜면 global_costmap이 TF를 기다리다
        #   bond 시간 초과 → "Aborting bringup"으로 Nav2가 멈춘다 (2026-11-20).
        #   → 노드만 띄워 두고, 언독 후 scan·TF가 들어오면 코스 노드(a_course_node)가 STARTUP을 요청한다.
        TimerAction(period=5.0, actions=[GroupAction([                   # map_server가 먼저 지도를 내도록
            PushRosNamespace(ns),
            SetRemap(ns + '/global_costmap/scan', ns + '/scan'),
            SetRemap(ns + '/local_costmap/scan', ns + '/scan'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(tb4_nav, 'launch', 'navigation_launch.py')),
                launch_arguments=[('use_sim_time', 'false'),
                                  ('params_file', os.path.join(pkg, 'config', 'nav2_real.yaml')),
                                  ('use_composition', 'False'),
                                  ('namespace', ns),
                                  ('autostart', arg('nav2_autostart'))]),
        ])]),
    ]


def _tracker(context):
    """[코스 관람 변경 ⑩] tracker:=true: 카메라 TF(top_cam_tf) + 인식 노드. arti_follow와 같은 카메라 설정(person_tracker/config/phone_camera.yaml).
    direct_camera:=true(기본): 인식 노드가 웹캠(video_source, yaml 기본 0)을 직접 연다 → camera_pub 안 띄움
      (같은 카메라를 두 번 열면 충돌, 영상 토픽이 네트워크를 안 탐).
    direct_camera:=false: 전처럼 camera_pub → top_cam/image_raw/compressed → 인식 노드가 구독."""
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    if arg('tracker').lower() not in ('true', '1'):
        return []
    share = get_package_share_directory('person_tracker')
    direct = arg('direct_camera').lower() in ('true', '1')
    video = arg('video_source')
    if direct and not video:
        with open(os.path.join(share, 'config', 'phone_camera.yaml'), encoding='utf-8') as f:
            video = str((yaml.safe_load(f) or {}).get('video_source', '0')).strip()
    ns = arg('robot_namespace')
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'phone_camera.launch.py')),
        launch_arguments={'namespace': ns, 'yaw': arg('camera_yaw'),          # 카메라 TF 방향
                          'publish_camera': 'false' if direct else 'true'}.items())
    tracker = Node(package='person_tracker', executable=arg('tracker_node'),
                   name='multi_person_tracker_node', namespace=ns, output='screen',
                   parameters=[{
                       # 인식 노드도 같은 카메라 방향으로 LiDAR와 맞춤 (안 넣으면 yaml의 0 = 앞쪽)
                       'cam_yaw': float(arg('camera_yaw')),
                       'video_source': video if direct else ''}])   # '' = 토픽 구독
    return [LogInfo(msg=f'[인식] 카메라 {"직접 " + repr(video) if direct else "토픽 (camera_pub)"}'),
            camera, TimerAction(period=2.0, actions=[tracker])]     # TF가 먼저 뜨게


def generate_launch_description():
    pkg = Path(get_package_share_directory('arti_pc2_nav'))
    audio_share = get_package_share_directory('arti_audio')
    return LaunchDescription([
        # ---------------- [코스 관람 변경 ⑥] discovery: 이 launch가 띄우는 노드는 super client 아님 ----------------
        # .bashrc(/etc/turtlebot4_discovery/setup.bash)는 터미널이면 ROS_SUPER_CLIENT=True.
        # super client는 서버(PC3)에서 네트워크 전체(robot1·키오스크 포함) 주소록을 받으므로, 노드 20개가 다 그러면
        # 와이파이 discovery 트래픽이 커진다. 노드는 자기 토픽만 찾으면 되므로 False. (ros2 topic list 등 터미널 명령은 그대로 True)
        DeclareLaunchArgument('super_client', default_value='False'),
        SetEnvironmentVariable('ROS_SUPER_CLIENT', LaunchConfiguration('super_client')),

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
        DeclareLaunchArgument('nav', default_value='true'),                # localization + Nav2도 같이 띄움
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('nav2_autostart', default_value='false'),
        DeclareLaunchArgument('direct_camera', default_value='true'),     # true: 인식 노드가 웹캠 직접 / false: camera_pub 토픽
        DeclareLaunchArgument('video_source', default_value=''),          # 직접 받을 카메라 ('' = phone_camera.yaml, 0 = 웹캠, DroidCam URL)   # false: 코스 노드가 언독 후 Nav2를 켬                    # 지도 yaml (기본 워크스페이스 maps/gallery.yaml)

        # ---------------- localization + Nav2 (nav:=true) ----------------
        OpaqueFunction(function=_localization_and_nav2),

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
        OpaqueFunction(function=_tracker),

        # ---------------- RViz 반원 마커 노드 ----------------
        Node(package='arti_pc2_nav', executable='zone_marker_node', name='arti_zone_markers',
             namespace=LaunchConfiguration('robot_namespace'), output='screen',
             parameters=[{
                 'config_file': LaunchConfiguration('config_file'),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'map_frame': 'map',
             }]),
    ])
