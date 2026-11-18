"""관람 안내 한 번에 — Nav2 버전 (tour.launch.py와 같고, 추종 노드만 nav2_follow).

    ros2 launch arti_follow tour_nav2.launch.py              # Nav2를 nav2_follow.yaml로 이미 띄워 둔 경우
    ros2 launch arti_follow tour_nav2.launch.py nav2:=true   # Nav2도 여기서 nav2_follow.yaml로 같이 띄움
    → <ns>/arti/start_tour 액션을 보내면 undock → 초기 위치 → 키오스크 쪽 보기 → 인식 기다림 → 추종 → 출구 → 복귀·도킹
      ros2 action send_goal /robot1/arti/start_tour arti_interfaces/action/StartTour "{mode: FREE}" --feedback

옵션 — 이 파일은 nav2·audio·zones_file만 선언한다. 나머지는 include한 launch의 인자가 그대로 통한다
    nav2:=true               Nav2(turtlebot4_navigation nav2.launch.py)를 arti_bringup/config/nav2_follow.yaml로 같이 띄움.
                             기본 false: 이미 떠 있는 Nav2를 씀. 그 Nav2가 기본 설정이면 먼저 끄고 nav2:=true로 (같은 노드가 둘이 됨)
    audio:=false             음성 노드는 따로 띄울 때
    zones_file:=<경로>       구역 파일 (기본 arti_common/config/artworks.yaml)
    follow_test.launch.py    namespace, auto_start, drive, test(bag 녹화), bag_dir, tracker (follower는 여기서 nav2로 고정)
    guide_audio.launch.py    player_cmd
전제: localization(AMCL)이 이 네임스페이스로 떠 있다. nav2_follow의 설정 값은 arti_follow/config/nav2_follow.json

뜨는 것 (<ns> 아래)
    (nav2:=true) Nav2 일체          planner·controller·bt_navigator 등 → navigate_to_pose, goal_update
    follow_test.launch.py            카메라·인식 노드 + nav2_follow + bag 녹화
    artwork_zone                     작품 구역 판정 → explain_artwork, guide_message
    guide_audio_node (audio:=true)   음성
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    follow_share = get_package_share_directory('arti_follow')
    audio_share = get_package_share_directory('arti_audio')
    nav2_params = os.path.join(get_package_share_directory('arti_bringup'), 'config', 'nav2_follow.yaml')
    ns = LaunchConfiguration('namespace')

    # Nav2: nav2:=true일 때만, 우리 설정(nav2_follow.yaml)으로
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('turtlebot4_navigation'), 'launch', 'nav2.launch.py')),
        launch_arguments={'namespace': ns, 'params_file': nav2_params}.items(),
        condition=IfCondition(LaunchConfiguration('nav2')))
    nav2_note = LogInfo(
        msg=f'[tour_nav2] Nav2는 따로 떠 있어야 함 (nav2_follow.yaml): ros2 launch turtlebot4_navigation nav2.launch.py '
            f'namespace:=<ns> params_file:={nav2_params}',
        condition=UnlessCondition(LaunchConfiguration('nav2')))
    # 인식 + 추종: follow_test.launch.py를 follower:=nav2로 (나머지 인자는 명령줄 값이 그대로 전달됨)
    follow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(follow_share, 'launch', 'follow_test.launch.py')),
        launch_arguments={'follower': 'nav2'}.items())
    audio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(audio_share, 'launch', 'guide_audio.launch.py')),
        condition=IfCondition(LaunchConfiguration('audio')))
    zone = Node(
        package='arti_follow', executable='artwork_zone', namespace=ns, output='screen',
        parameters=[{'zones_file': LaunchConfiguration('zones_file')}])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable('ARTI_ROBOT_NS', default_value=''),
                              description='로봇 네임스페이스 (기본: $ARTI_ROBOT_NS)'),
        DeclareLaunchArgument('nav2', default_value='false',
                              description='true면 Nav2를 nav2_follow.yaml로 같이 띄움 (이미 떠 있는 Nav2가 있으면 먼저 끄기)'),
        DeclareLaunchArgument('audio', default_value='true', description='음성 노드(guide_audio_node)도 같이 띄우기'),
        DeclareLaunchArgument('zones_file', default_value='',
                              description='구역 파일 (비우면 arti_common/config/artworks.yaml)'),
        nav2, nav2_note, follow, audio, zone,
    ])
