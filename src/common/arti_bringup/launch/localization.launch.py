"""localization (AMCL + map_server) — 시작 위치를 자동으로 잡는다.

    ros2 launch arti_bringup localization.launch.py                  # namespace = $ARTI_ROBOT_NS, map = $ARTI_MAP
    ros2 launch arti_bringup localization.launch.py namespace:=/robot4

하는 일
    turtlebot4_navigation localization.launch.py를 띄우면서 params에 arti_bringup/config/localization.yaml을 넘긴다.
    AMCL 시작 위치(initial_pose)는 arti_common/config/init_pose.yaml의 robots.<namespace>.<start_point>(기본 dock)
    값으로 채운다 → 시작 위치는 init_pose.yaml 한 곳에서만 고친다. RViz "2D Pose Estimate"가 필요 없다.
    init_pose.yaml에 그 로봇이 없으면 시작 위치 없이 띄운다 (RViz로 직접 지정).

옵션
    namespace:=/robot1       기본 $ARTI_ROBOT_NS
    map:=<yaml>              기본 $ARTI_MAP (저장소 maps/gallery.yaml)
    start_point:=dock        init_pose.yaml에서 쓸 위치 이름 (기본 dock = 도크 위에서 띄울 때. 언독 뒤면 undock)
"""
import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
import yaml


def start_pose(ns, point):
    """init_pose.yaml의 robots.<ns>.<point> → (x, y, yaw[rad]). 없으면 None."""
    path = os.path.join(get_package_share_directory('arti_common'), 'config', 'init_pose.yaml')
    with open(path, encoding='utf-8') as f:
        p = ((yaml.safe_load(f) or {}).get('robots') or {}).get(ns, {}).get(point)
    if not p:
        return None
    return float(p['x']), float(p['y']), math.radians(float(p.get('yaw_deg', 0.0)))


def make(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    ns = '/' + arg('namespace').strip('/')
    if ns == '/':
        raise RuntimeError('namespace가 비었음 → 저장소 폴더 안에서 실행($ARTI_ROBOT_NS)하거나 namespace:=/robot1')
    with open(os.path.join(get_package_share_directory('arti_bringup'), 'config', 'localization.yaml'),
              encoding='utf-8') as f:
        params = yaml.safe_load(f)
    amcl = params['amcl']['ros__parameters']
    pose = start_pose(ns, arg('start_point'))
    if pose is None:
        amcl['set_initial_pose'] = False
        msg = f'[localization] init_pose.yaml에 {ns}.{arg("start_point")} 없음 → 시작 위치 없이 (RViz 2D Pose Estimate)'
    else:
        amcl['set_initial_pose'] = True
        amcl['initial_pose'] = {'x': pose[0], 'y': pose[1], 'z': 0.0, 'yaw': pose[2]}
        msg = (f'[localization] {ns} 시작 위치 = init_pose.yaml {arg("start_point")} '
               f'({pose[0]}, {pose[1]}, {math.degrees(pose[2]):.0f}°)')

    out = tempfile.NamedTemporaryFile('w', suffix='_localization.yaml', delete=False, encoding='utf-8')
    yaml.safe_dump(params, out, allow_unicode=True)
    out.close()
    tb4 = os.path.join(get_package_share_directory('turtlebot4_navigation'), 'launch', 'localization.launch.py')
    return [LogInfo(msg=msg), IncludeLaunchDescription(
        PythonLaunchDescriptionSource(tb4),
        launch_arguments={'namespace': ns, 'map': arg('map'), 'params': out.name}.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable('ARTI_ROBOT_NS', default_value=''),
                              description='로봇 네임스페이스 (기본: $ARTI_ROBOT_NS)'),
        DeclareLaunchArgument('map', default_value=EnvironmentVariable('ARTI_MAP', default_value=''),
                              description='지도 yaml (기본: $ARTI_MAP)'),
        DeclareLaunchArgument('start_point', default_value='dock',
                              description='시작 위치로 쓸 init_pose.yaml의 위치 이름'),
        OpaqueFunction(function=make),
    ])
