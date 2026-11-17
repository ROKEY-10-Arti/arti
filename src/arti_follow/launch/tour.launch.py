"""관람 안내 한 번에: 추종(follow_test.launch.py) + 작품 구역 판정(artwork_zone) + 음성(arti_audio guide_audio_node).

    ros2 launch arti_follow tour.launch.py
    → 인식 창에서 따라갈 사람을 클릭하면 로봇이 따라가고,
      관람객이 작품 앞 반원 구역에서 5초 멈추면 설명, 출구·키오스크 구역에 들어가면 안내 음성

옵션 (follow_test.launch.py 옵션은 그대로 넘어감)
    drive:=false             인식·계산·로그만 하고 cmd_vel은 보내지 않음
    test:=false / bag:=false 진단 로그·bag 녹화 끄기
    tracker:=false           인식 모듈은 따로 띄울 때
    tracker_node:=false      카메라만 띄우고 인식 노드는 뺄 때 (tour_no_tracker.launch.py)
    video_source:=0          웹캠 번호 또는 DroidCam URL
    audio:=false             음성 노드는 따로 띄울 때 (구역 판정만)
    player_cmd:="pw-play --volume 0.6"   재생 명령 (출력 장치·음량)
    zones_file:=<경로>       구역 파일 (기본 arti_common/config/artworks.yaml)
    namespace:=/robot1       기본은 $ARTI_ROBOT_NS
전제: AMCL(localization)이 이미 이 네임스페이스로 떠 있다 (map→odom).

토픽 (<ns> 아래)
    target_person/point_map → artwork_zone → explain_artwork(action) → guide_audio_node → 스피커
                                           → guide_message(String)   ↗
    artwork_zone/state       지금 판단 (idle, moving, dwell ..., explaining art_01)
    artwork_zone/reset       설명한 작품 목록 비우기 (새 관람객)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

FOLLOW_ARGS = ['drive', 'test', 'bag', 'tracker', 'tracker_node', 'video_source', 'log_dir']


def generate_launch_description():
    follow_share = get_package_share_directory('arti_follow')
    audio_share = get_package_share_directory('arti_audio')
    ns = LaunchConfiguration('namespace')

    follow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(follow_share, 'launch', 'follow_test.launch.py')),
        launch_arguments=[('namespace', ns)] + [(a, LaunchConfiguration(a)) for a in FOLLOW_ARGS])
    audio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(audio_share, 'launch', 'guide_audio.launch.py')),
        launch_arguments=[('namespace', ns), ('player_cmd', LaunchConfiguration('player_cmd'))],
        condition=IfCondition(LaunchConfiguration('audio')))
    zone = Node(
        package='arti_follow', executable='artwork_zone', namespace=ns, output='screen',
        parameters=[{'zones_file': LaunchConfiguration('zones_file')}])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable('ARTI_ROBOT_NS', default_value=''),
                              description='로봇 네임스페이스 (기본: $ARTI_ROBOT_NS)'),
        DeclareLaunchArgument('drive', default_value='true', description='false면 cmd_vel을 보내지 않음'),
        DeclareLaunchArgument('test', default_value='true', description='진단 로그를 켜고 log_dir에 저장'),
        DeclareLaunchArgument('bag', default_value='true', description='test일 때 ros2 bag 녹화 (카메라 영상 제외)'),
        DeclareLaunchArgument('tracker', default_value='true', description='사람 인식 모듈도 같이 띄우기'),
        DeclareLaunchArgument('tracker_node', default_value='true',
                              description='false면 카메라만 띄우고 인식 노드(multi_tracker_node)는 뺌'),
        DeclareLaunchArgument('video_source', default_value='0', description='웹캠 번호 또는 DroidCam URL'),
        DeclareLaunchArgument('log_dir', default_value='~/arti_logs/follow', description='테스트 로그 폴더'),
        DeclareLaunchArgument('audio', default_value='true', description='음성 노드(guide_audio_node)도 같이 띄우기'),
        DeclareLaunchArgument('player_cmd', default_value='pw-play', description='재생 명령'),
        DeclareLaunchArgument('zones_file', default_value='',
                              description='구역 파일 (비우면 arti_common/config/artworks.yaml)'),
        follow, audio, zone,
    ])
