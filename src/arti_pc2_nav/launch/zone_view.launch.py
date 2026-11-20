"""[코스 관람] robot4 코스 구역(a_course.yaml)을 로봇 없이 지도 위에 RViz로 보기 + 좌표 찍기.

    ros2 launch arti_pc2_nav zone_view.launch.py

보이는 것 (a_course.yaml = robot4가 실제로 쓰는 값. follow 팀 arti_common artworks.yaml과 따로)
    지도, 키오스크 반원(파랑), 작품 반원(초록), 출구 직사각형(주황, arti_common 공통),
    goal_pose = 흰 점 + 화살표 (로봇이 서는 자리와 정면. 키오스크는 이 자리까지 감, 변경 ⑪)
    로봇 위치(amcl_pose) — 로봇·localization이 떠 있을 때만
좌표 찍기
    RViz 위쪽 "Publish Point"로 지도를 클릭 →
    ros2 topic echo /robot4/clicked_point geometry_msgs/msg/PointStamped
    goal_pose는 자기 반원 안이어야 한다 (밖이면 코스 노드가 설정을 못 읽음)
    a_course.yaml을 고쳤으면 이 launch만 다시 켜면 된다 (빌드 불필요, symlink 설치)

옵션
    map:=<yaml>          지도 (기본 $ARTI_MAP, 없으면 워크스페이스 maps/gallery.yaml)
    config_file:=<yaml>  코스 설정 (기본 이 패키지 config/a_course.yaml)
    namespace:=/robot4   클릭 좌표·마커·로봇 위치 네임스페이스
지도는 /pc2_zone_view 네임스페이스의 map_server로 따로 띄워서 로봇 localization의 map과 겹치지 않는다.
"""
import os
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MAP_NS = '/pc2_zone_view'


def _make(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    pkg = get_package_share_directory('arti_pc2_nav')
    robot_ns = '/' + arg('namespace').strip('/')
    map_yaml = os.path.expanduser(arg('map') or os.environ.get('ARTI_MAP', '') or os.path.join(
        os.path.dirname(os.path.dirname(get_package_prefix('arti_pc2_nav'))), 'maps', 'gallery.yaml'))
    if not os.path.isfile(map_yaml):
        raise RuntimeError(f'지도 파일이 없음: {map_yaml} (map:=<yaml>로 지정)')
    config = arg('config_file') or os.path.join(pkg, 'config', 'a_course.yaml')

    with open(os.path.join(pkg, 'rviz', 'zone_view.rviz'), encoding='utf-8') as f:
        cfg = f.read().replace('<map_ns>', MAP_NS).replace('<robot_ns>', robot_ns)
    rviz_file = os.path.join(tempfile.gettempdir(), 'arti_pc2_zone_view%s.rviz' % robot_ns.replace('/', '_'))
    with open(rviz_file, 'w', encoding='utf-8') as f:
        f.write(cfg)

    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server', namespace=MAP_NS,
             output='log', parameters=[{'yaml_filename': map_yaml, 'frame_id': 'map'}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_zone_view',
             namespace=MAP_NS, output='log', parameters=[{'autostart': True, 'node_names': ['map_server']}]),
        Node(package='arti_pc2_nav', executable='zone_marker_node', name='arti_zone_markers',
             namespace=robot_ns, output='screen', parameters=[{'config_file': config, 'map_frame': 'map'}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_file], output='log'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='/robot4'),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('config_file', default_value=''),
        OpaqueFunction(function=_make),
    ])
