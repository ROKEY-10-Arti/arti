"""추종 테스트를 한 번에: 웹캠 사람 인식 + visitor_follower(실제 주행) + RViz + 자세한 로그 저장.

    ros2 launch arti_follow follow_test.launch.py
    → 인식 창에서 따라갈 사람을 마우스로 클릭하면 로봇이 따라간다

옵션
    video_source:=0          노트북 웹캠 번호, 또는 휴대폰 DroidCam URL (http://<폰IP>:4747/video)
    drive:=false             인식·계산·로그만 하고 cmd_vel은 보내지 않음
    test:=false              진단 로그·파일 저장 끄기
    tracker:=false           인식 모듈은 따로 띄울 때
    rviz:=true               RViz도 띄움 (기본은 인식 창 하나만 보임)
    namespace:=/robot1       기본은 $ARTI_ROBOT_NS (env.bash의 ARTI_DISCOVERY_SERVERS로 자동)
전제: AMCL(localization)이 이미 이 네임스페이스로 떠 있다 (map→odom).

뜨는 것
    phone_camera.launch.py (person_tracker)   camera_pub: 웹캠 → <ns>/top_cam/image_raw/compressed, top_cam_tf
    multi_tracker_node (person_tracker)       YOLO 인식 + LiDAR → <ns>/target_person/point_map
                                              인식 창: 사람 박스 + 로봇까지 거리(m) (타겟 초록, '~'는 LiDAR 없이 박스 크기로 추정)
    visitor_follower                          좌표 → <ns>/cmd_vel (화면 없음, 로그만)
    rviz2 (rviz:=true일 때만)                 웹캠·지도·LiDAR·인식 좌표·추종 판단 (rviz/follow_test.rviz)
인식 노드는 네임스페이스를 붙여 띄운다. 안 붙이면 설정 파일을 못 찾아 네임스페이스 없는 토픽을 구독해서
영상·scan을 못 받는다 (인식 창이 까맣게 나옴).

테스트 로그 (test:=true, 기본)  log_dir/<날짜_시각>/  — 영상은 저장하지 않음, 좌표·속도·시간만
    python3_*.log         노드 로그. 2초마다 토픽별 Hz·간격·지연·콜백 시간, 제어 주기·계산 시간, 정지 이유 요약
                          + 끊긴 순간마다 [늦음] [제어 밀림] [이유] [시간]
    follow_timing.csv     제어 주기(0.1 s)마다 한 줄
    run_info.txt          실행 인자, PC 시간 동기화(NTP) 상태
"""
import datetime
import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def pc_time_sync():
    """이 PC 시계가 NTP로 맞춰져 있는지 (timedatectl). 로봇 쪽 차이는 노드의 [시간] 로그로 본다."""
    try:
        out = subprocess.run(['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'],
                             capture_output=True, text=True, timeout=3).stdout.strip()
        return {'yes': '맞춰짐', 'no': '안 맞춰짐 (sudo timedatectl set-ntp true)'}.get(out, out or '알 수 없음')
    except Exception as e:
        return f'확인 실패 ({e})'


def tracker_actions(ns, video_source):
    """person_tracker: 카메라 launch + 인식 노드 (둘 다 같은 네임스페이스)."""
    try:
        share = get_package_share_directory('person_tracker')
    except PackageNotFoundError:
        raise RuntimeError('person_tracker 패키지가 없음 → colcon build 하거나 tracker:=false') from None
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'phone_camera.launch.py')),
        launch_arguments={'namespace': ns, 'video_source': video_source}.items())
    tracker = Node(package='person_tracker', executable='multi_tracker_node',
                   name='multi_person_tracker_node', namespace=ns, output='screen',
                   parameters=[{'show_info_window': False,      # 화면은 인식 영상 창 하나만
                                'show_distance': True}])        # 박스마다 거리(m)
    return [camera, TimerAction(period=2.0, actions=[tracker])]     # 카메라가 먼저 켜지게


def make_nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    ns = '/' + arg('namespace').strip('/')
    if ns == '/':
        raise RuntimeError('namespace가 비었음 → 저장소 폴더 안에서 실행($ARTI_ROBOT_NS)하거나 namespace:=/robot1')
    test = arg('test').lower() == 'true'
    drive = arg('drive').lower() == 'true'
    params = {'start_enabled': drive, 'debug_viz': True}
    env, actions = {}, []

    if test:
        run_dir = os.path.join(os.path.expanduser(arg('log_dir')),
                               datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
        os.makedirs(run_dir, exist_ok=True)
        sync = pc_time_sync()
        with open(os.path.join(run_dir, 'run_info.txt'), 'w', encoding='utf-8') as f:
            f.write(f'namespace={ns}\ndrive={drive}\ntracker={arg("tracker")}\n'
                    f'video_source={arg("video_source")}\nPC 시간 동기화(NTP): {sync}\n')
        params.update({'diag': True, 'diag_events': True,
                       'diag_csv': os.path.join(run_dir, 'follow_timing.csv')})
        env['ROS_LOG_DIR'] = run_dir                # 노드 로그 파일이 여기에 생김
        actions.append(LogInfo(msg=f'[테스트] 주행 {"켬" if drive else "끔"}, PC 시간 동기화 {sync}, '
                                   f'로그 저장 위치: {run_dir}'))

    if arg('tracker').lower() == 'true':
        actions += tracker_actions(ns, arg('video_source'))

    actions.append(Node(
        package='arti_follow', executable='visitor_follower', namespace=ns, output='screen',
        parameters=[params], additional_env=env))

    if arg('rviz').lower() == 'true':
        share = get_package_share_directory('arti_follow')
        with open(os.path.join(share, 'rviz', 'follow_test.rviz'), encoding='utf-8') as f:
            cfg = f.read().replace('<ns>', ns)
        path = os.path.join(tempfile.gettempdir(), 'arti_follow_test%s.rviz' % ns.replace('/', '_'))
        with open(path, 'w', encoding='utf-8') as f:
            f.write(cfg)
        # tf는 네임스페이스 안(/robot1/tf)에 있으므로 RViz가 그쪽을 보게 바꿔 준다
        actions.append(Node(
            package='rviz2', executable='rviz2', arguments=['-d', path], output='log',
            remappings=[('/tf', ns + '/tf'), ('/tf_static', ns + '/tf_static')]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable(
            'ARTI_ROBOT_NS', default_value=''), description='로봇 네임스페이스 (기본: $ARTI_ROBOT_NS)'),
        DeclareLaunchArgument('video_source', default_value='0',
                              description='인식 카메라: 웹캠 번호 또는 DroidCam URL'),
        DeclareLaunchArgument('drive', default_value='true', description='false면 cmd_vel을 보내지 않음'),
        DeclareLaunchArgument('test', default_value='true',
                              description='진단 로그를 켜고 log_dir에 저장'),
        DeclareLaunchArgument('tracker', default_value='true',
                              description='사람 인식 모듈(person_tracker)도 같이 띄우기'),
        DeclareLaunchArgument('rviz', default_value='false', description='RViz도 띄우기 (기본은 인식 창만)'),
        DeclareLaunchArgument('log_dir', default_value='~/arti_logs/follow',
                              description='테스트 로그를 저장할 폴더'),
        OpaqueFunction(function=make_nodes),
    ])
