"""작품·안내 구역을 지도 위에 RViz로 확인 (T04-2).

    ros2 launch arti_follow zone_view.launch.py                    # localization이 떠 있을 때 (<ns>/map 사용)
    ros2 launch arti_follow zone_view.launch.py standalone:=true   # 로봇 없이 지도 파일만으로

보이는 것
    지도, 구역(파랑 반원 = 작품, 주황 반원 = 출구·키오스크, 화살표 = 정면, 글자 = 이름),
    관람객 좌표(target_person/point_map, 분홍 공), 로봇 위치(amcl_pose, 초록 화살표), LiDAR
좌표 재기
    RViz 상단 "Publish Point"로 지도를 클릭 → ros2 topic echo <ns>/clicked_point (standalone이어도 로봇 ns)
    구역 파일(arti_common/config/artworks.yaml)을 고쳤으면 이 launch만 다시 켜면 된다

옵션
    standalone:=true   지도·구역은 /zone_view 네임스페이스의 map_server로 따로 (로봇 localization과 안 겹침).
                       관람객 좌표·로봇 위치·LiDAR·클릭 좌표는 그대로 <ns>(로봇) 쪽을 본다
    map:=<yaml>        standalone일 때 지도 (기본 $ARTI_MAP)
    zones_file:=<경로> 구역 파일 (기본 arti_common/config/artworks.yaml)
    namespace:=/robot1 기본 $ARTI_ROBOT_NS
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def make(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    standalone = arg('standalone').lower() == 'true'
    robot_ns = '/' + arg('namespace').strip('/')     # 관람객 좌표·로봇 위치·LiDAR·클릭 좌표
    ns = '/zone_view' if standalone else robot_ns    # 지도·구역 마커
    if robot_ns == '/':
        raise RuntimeError('namespace가 비었음 → 저장소 폴더 안에서 실행($ARTI_ROBOT_NS)하거나 namespace:=/robot1')
    actions = []

    if standalone:
        map_yaml = os.path.expanduser(arg('map'))
        if not os.path.isfile(map_yaml):
            raise RuntimeError(f'지도 파일 없음: {map_yaml} → map:=<yaml> 또는 저장소 폴더에서 실행($ARTI_MAP)')
        actions += [
            Node(package='nav2_map_server', executable='map_server', name='map_server', namespace=ns,
                 output='log', parameters=[{'yaml_filename': map_yaml, 'frame_id': 'map'}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_zone_view',
                 namespace=ns, output='log', parameters=[{'autostart': True, 'node_names': ['map_server']}]),
        ]

    actions.append(Node(package='arti_follow', executable='zone_viz', namespace=ns, output='screen',
                        parameters=[{'zones_file': arg('zones_file')}]))

    share = get_package_share_directory('arti_follow')
    with open(os.path.join(share, 'rviz', 'zone_view.rviz'), encoding='utf-8') as f:
        cfg = f.read().replace('<map_ns>', ns).replace('<robot_ns>', robot_ns)
    path = os.path.join(tempfile.gettempdir(), 'arti_zone_view%s.rviz' % ns.replace('/', '_'))
    with open(path, 'w', encoding='utf-8') as f:
        f.write(cfg)
    # tf는 네임스페이스 안(/robot1/tf)에 있으므로 RViz가 그쪽을 보게 바꿔 준다
    actions.append(Node(package='rviz2', executable='rviz2', arguments=['-d', path], output='log',
                        remappings=[('/tf', robot_ns + '/tf'), ('/tf_static', robot_ns + '/tf_static')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable('ARTI_ROBOT_NS', default_value=''),
                              description='로봇 네임스페이스 (기본: $ARTI_ROBOT_NS)'),
        DeclareLaunchArgument('standalone', default_value='false',
                              description='true면 로봇 없이 지도 파일(map)로 map_server를 따로 띄움'),
        DeclareLaunchArgument('map', default_value=EnvironmentVariable('ARTI_MAP', default_value=''),
                              description='standalone일 때 지도 yaml (기본: $ARTI_MAP)'),
        DeclareLaunchArgument('zones_file', default_value='',
                              description='구역 파일 (비우면 arti_common/config/artworks.yaml)'),
        OpaqueFunction(function=make),
    ])
