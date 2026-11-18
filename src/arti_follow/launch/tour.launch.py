"""관람 안내 한 번에: 추종(follow_test.launch.py) + 작품 구역 판정(artwork_zone) + 음성(arti_audio guide_audio_node).

    ros2 launch arti_follow tour.launch.py
    → <ns>/arti/start_tour 액션을 보내면 undock → 초기 위치 → 출발 방향 보기 → 인식 기다림,
      인식 창에서 따라갈 사람을 클릭하면 "인식되었습니다" 안내 후 로봇이 따라가고,
      관람객이 작품 앞 반원 구역에서 5초 멈추면 설명, 출구·키오스크 구역에 들어가면 안내 음성

옵션 — 이 파일은 audio·zones_file만 선언한다. 나머지는 include한 launch의 인자가 그대로 통한다
    (ros2 launch 명령줄 인자는 include된 launch까지 전달됨. 목록: ros2 launch arti_follow tour.launch.py --show-args)
    audio:=false             음성 노드는 따로 띄울 때 (구역 판정만)
    zones_file:=<경로>       구역 파일 (기본 arti_common/config/artworks.yaml)
    follow_test.launch.py    namespace, auto_start, drive, test(bag 녹화), bag_dir, tracker(all|camera|none)
    guide_audio.launch.py    player_cmd:="pw-play --volume 0.6"   재생 명령 (출력 장치·음량)
전제: AMCL(localization)이 이미 이 네임스페이스로 떠 있다 (map→odom).

토픽 (<ns> 아래)
    target_person/point_map → artwork_zone → explain_artwork(action) → guide_audio_node → 스피커
                                           → guide_message(String)   ↗
    artwork_zone/state       지금 판단 (idle, moving, dwell ..., explaining art_01)
    arti/tour_end            관람 끝(visitor_follower) → 인식 모듈·artwork_zone이 관람객 데이터 삭제
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    follow_share = get_package_share_directory('arti_follow')
    audio_share = get_package_share_directory('arti_audio')
    ns = LaunchConfiguration('namespace')

    follow = IncludeLaunchDescription(        # 인자는 명령줄 값이 그대로 전달됨
        PythonLaunchDescriptionSource(os.path.join(follow_share, 'launch', 'follow_test.launch.py')))
    audio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(audio_share, 'launch', 'guide_audio.launch.py')),
        condition=IfCondition(LaunchConfiguration('audio')))
    zone = Node(
        package='arti_follow', executable='artwork_zone', namespace=ns, output='screen',
        parameters=[{'zones_file': LaunchConfiguration('zones_file')}])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable('ARTI_ROBOT_NS', default_value=''),
                              description='로봇 네임스페이스 (기본: $ARTI_ROBOT_NS)'),   # artwork_zone도 씀
        DeclareLaunchArgument('audio', default_value='true', description='음성 노드(guide_audio_node)도 같이 띄우기'),
        DeclareLaunchArgument('zones_file', default_value='',
                              description='구역 파일 (비우면 arti_common/config/artworks.yaml)'),
        follow, audio, zone,
    ])
