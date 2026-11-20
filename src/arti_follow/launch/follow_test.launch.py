"""추종 테스트를 한 번에: 웹캠 사람 인식 + visitor_follower(실제 주행) + ros2 bag 녹화.

    ros2 launch arti_follow follow_test.launch.py
    → <ns>/arti/prepare_tour 액션을 보내면 undock → 초기 위치 → 출발 방향 보기 → 인식 기다림 (등록),
      그다음 <ns>/arti/start_tour 액션을 보내면 따라간다.
      관람객 등록: tracker:=gesture(기본)면 로봇 앞에서 손 제스처(주먹 → 손바닥 펴기),
      tracker:=front면 로봇 바로 앞 사람 자동 등록 또는 인식 창 클릭 → "인식되었습니다" 안내
      ros2 action send_goal /robot1/arti/prepare_tour arti_interfaces/action/PrepareTour "{start: true}" --feedback
      ros2 action send_goal /robot1/arti/start_tour arti_interfaces/action/StartTour "{artwork_ids: []}" --feedback

옵션
    drive:=false             인식·계산·로그만 하고 cmd_vel은 보내지 않음
    test:=true               ros2 bag 녹화 켜기 (기본 false, 2026-11-20부터 기본 끔)
    video_source:=<값>       카메라 ('0' 웹캠, DroidCam URL). 비우면 person_tracker phone_camera.yaml
    bag_dir:=<폴더>          bag 저장 폴더 (기본 ~/arti_bags/follow, 그 아래 <날짜_시각>/)
    tracker:=gesture         (기본) gesture_test_node: 손 제스처(주먹 → 손바닥)로 관람객 등록
                             (visitor_follower auto_register 끔)
    tracker:=front           front_tracker_node: 로봇 바로 앞 사람 자동 등록(register_front) + 클릭 등록
    tracker:=all             multi_tracker_node (예전 노드, 모델·모드 바꾸는 벤치마크 화면) + 앞 사람 자동 등록
    tracker:=camera          카메라만 띄우고 인식 노드는 뺌 (인식은 다른 PC·bag 재생에서)
    (gesture·front는 person_tracker BaseTrackerNode를 상속한 같은 구조, 좌표 토픽·형식은 셋 다 같음)
    tracker:=none            카메라·인식 노드 모두 안 띄움 (인식 모듈을 따로 띄울 때)
    auto_start:=true         액션 없이 켜자마자 관람 시작
    diag:=true               visitor_follower 진단 로그: 2 s마다 토픽별 수신 Hz·간격·지연, [늦음]·[이유] 즉시 로그
    namespace:=/robot1       기본은 $ARTI_ROBOT_NS (env.bash의 ARTI_DISCOVERY_SERVERS로 자동)
전제: AMCL(localization)이 이미 이 네임스페이스로 떠 있다 (map→odom).

뜨는 것
    phone_camera.launch.py (person_tracker)   top_cam_tf (+ tracker:=all·camera면 camera_pub: 웹캠 → 토픽)
    인식 노드 (person_tracker, tracker 인자) gesture·front: 웹캠 직접 읽기 → 화면 (영상 토픽 없음)
                                              YOLO 인식·ReID + LiDAR → <ns>/target_person/point_map
                                              인식 창: 사람 박스 + 로봇까지 거리(m) (타겟 초록, '~'는 LiDAR 없이 박스 크기로 추정)
    visitor_follower                          좌표 → <ns>/cmd_vel (화면 없음, 로그만)
인식 노드는 네임스페이스를 붙여 띄운다. 안 붙이면 설정 파일을 못 찾아 네임스페이스 없는 토픽을 구독해서
영상·scan을 못 받는다 (인식 창이 까맣게 나옴).

저장 (test:=true일 때, 기본은 녹화 안 함)  bag_dir/<날짜_시각>/  ros2 bag (mcap) 하나만. 로그·CSV 파일은 따로 만들지 않는다
    녹화: BAG_TOPICS + /rosout(노드 로그 메시지). 관람객 영상은 기본으로 저장하지 않음 — 좌표·거리·명령만
    다시 보기: ros2 bag info <폴더>,  ros2 bag play <폴더> + RViz,  노드 로그: ros2 bag play <폴더> 중 ros2 topic echo /rosout
"""
import datetime
import os
import subprocess

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
import yaml


# ros2 bag으로 녹화할 토픽 (네임스페이스 뒤 이름).
# 관람객 영상은 저장하지 않는다(개인정보 요구사항) → top_cam 이미지, oakd 이미지·depth는 넣지 않는다.
# 카메라 토픽은 녹화하지 않는다 (gesture·front는 영상 토픽 자체가 없음).
BAG_TOPICS = [
    'scan', 'tf', 'tf_static', 'odom', 'amcl_pose',              # 로봇 센서·위치
    'target_person/point_map', 'target_person/point_base',       # 인식 모듈 출력
    'target_person/marker', 'top_cam/pose_in_map', 'target_person/register_front', 'gesture/status',
    'cmd_vel', 'follow/state', 'initialpose', 'guide_message', 'dock_status',                  # visitor_follower 입출력
    'artwork_zone/state', 'arti/tour_end',                        # 작품 구역 (tour.launch.py), 관람 끝
]


# tracker 인자: all = 카메라 + 인식 노드, camera = 카메라만, none = 둘 다 안 띄움
TRACKER_MODES = ('gesture', 'front', 'all', 'camera', 'none')
# 인식 노드: (executable, node name, parameters)
# 재식별(놓친 뒤 같은 관람객인지) 기준을 person_tracker 기본값보다 엄격하게 (gesture·front = BaseTrackerNode 파라미터).
# 2026-11-20: 놓칠 때마다 다른 사람·벽을 관람객으로 다시 잡음 (ID 50→52→63→66→69→73→71). 대신 진짜 관람객도
# 다시 잡히기까지 조금 더 걸릴 수 있다 (너무 안 잡히면 값을 낮춘다). 괄호 = person_tracker 기본값
REID_STRICT = {
    'reid_match_thresh': 0.60,       # (0.50) 다시 잡을 때 특징 유사도 기준
    'search_min_sim': 0.58,          # (0.48) 탐색 중 후보가 넘어야 할 최소 유사도
    'recover_margin': 0.10,          # (0.06) 1위 후보가 2위보다 이만큼 앞서야 복귀 (비슷한 사람 둘이면 안 붙임)
    'recover_confirm_frames': 5,     # (2) 이만큼 연속 프레임 같은 후보여야 복귀 (15 fps면 약 0.3 s)
}
TRACKER_NODES = {
    'gesture': ('gesture_test_node', 'gesture_test_node', REID_STRICT),
    'front': ('front_tracker_node', 'front_tracker_node', REID_STRICT),
    'all': ('multi_tracker_node', 'multi_person_tracker_node',
            {'show_info_window': False,      # 화면은 인식 영상 창 하나만
             'show_distance': True}),        # 박스마다 거리(m)
}


def pc_time_sync():
    """이 PC 시계가 NTP로 맞춰져 있는지 (timedatectl). 로봇 쪽 차이는 노드의 [시간] 로그로 본다."""
    try:
        out = subprocess.run(['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'],
                             capture_output=True, text=True, timeout=3).stdout.strip()
        return {'yes': '맞춰짐', 'no': '안 맞춰짐 (sudo timedatectl set-ntp true)'}.get(out, out or '알 수 없음')
    except Exception as e:
        return f'확인 실패 ({e})'


# 카메라를 직접 읽는 인식 노드 (person_tracker BaseTrackerNode). 'all'(multi_tracker_node)은 camera_pub 토픽을 구독
DIRECT_CAMERA = ('gesture', 'front')


def tracker_actions(ns, mode, video_arg):
    """person_tracker: 탑캠 TF + 카메라 + 인식 노드 (같은 네임스페이스).
    gesture·front: 인식 노드가 웹캠을 직접 읽어 화면에 바로 보여 준다 (토픽·discovery와 상관없음).
                   영상 토픽은 내지 않고 camera_pub도 안 띄움 (웹캠은 한 프로세스만 열 수 있음)
    all: camera_pub(웹캠 → 토픽) + multi_tracker_node(토픽 구독). camera: camera_pub만 (인식은 다른 PC)
    video_source = video_source 인자, 비었으면 person_tracker/config/phone_camera.yaml 값 (노트북 웹캠 0)."""
    try:
        share = get_package_share_directory('person_tracker')
    except PackageNotFoundError:
        raise RuntimeError('person_tracker 패키지가 없음 → colcon build 하거나 tracker:=none') from None
    with open(os.path.join(share, 'config', 'phone_camera.yaml'), encoding='utf-8') as f:
        video_source = video_arg or str((yaml.safe_load(f) or {}).get('video_source', '0'))
    direct = mode in DIRECT_CAMERA
    camera = IncludeLaunchDescription(        # direct면 top_cam_tf만, 아니면 camera_pub도
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'phone_camera.launch.py')),
        launch_arguments={'namespace': ns, 'video_source': video_source,
                          'publish_camera': 'false' if direct else 'true'}.items())
    if mode not in TRACKER_NODES:
        return [camera]
    executable, name, params = TRACKER_NODES[mode]
    if direct:
        params = {**params, 'video_source': video_source}
    tracker = Node(package='person_tracker', executable=executable, name=name, namespace=ns,
                   output='screen', parameters=[params])
    if direct:
        return [camera, LogInfo(msg=f'[카메라] {name}가 웹캠({video_source})을 직접 읽어 화면에 표시 '
                                    f'(영상 토픽 없음, camera_pub 안 띄움)'), tracker]
    return [camera, TimerAction(period=2.0, actions=[tracker])]     # 카메라가 먼저 켜지게


def make_nodes(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    ns = '/' + arg('namespace').strip('/')
    if ns == '/':
        raise RuntimeError('namespace가 비었음 → 저장소 폴더 안에서 실행($ARTI_ROBOT_NS)하거나 namespace:=/robot1')
    test = arg('test').lower() == 'true'
    drive = arg('drive').lower() == 'true'
    tracker = arg('tracker').lower()
    if tracker not in TRACKER_MODES:
        raise RuntimeError(f'tracker:={tracker} → {" | ".join(TRACKER_MODES)} 중 하나')
    diag = arg('diag').lower() == 'true'
    params = {'start_enabled': drive, 'auto_start': arg('auto_start').lower() == 'true',
              'diag': diag, 'diag_events': diag,   # 진단: 수신 간격·지연·제어 주기 로그
              'auto_register': tracker in ('front', 'all')}   # 제스처 등록이면 앞 사람 등록 요청 안 보냄
    actions = []

    if test:                                        # ros2 bag 녹화 (폴더는 ros2 bag이 만든다)
        bag_dir = os.path.expanduser(arg('bag_dir'))
        os.makedirs(bag_dir, exist_ok=True)
        out = os.path.join(bag_dir, datetime.datetime.now().strftime('%Y%m%d_%H%M%S'))
        actions.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-s', 'mcap', '-o', out,
                 '--topics', '/rosout', *[f'{ns}/{t}' for t in BAG_TOPICS]],
            output='log'))
        actions.append(LogInfo(msg=f'[테스트] 주행 {"켬" if drive else "끔"}, PC 시간 동기화 {pc_time_sync()}, '
                                   f'bag 저장: {out}'))

    if tracker != 'none':
        actions += tracker_actions(ns, tracker, arg('video_source').strip())

    actions.append(Node(
        package='arti_follow', executable='visitor_follower', namespace=ns, output='screen',
        parameters=[params]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable(
            'ARTI_ROBOT_NS', default_value=''), description='로봇 네임스페이스 (기본: $ARTI_ROBOT_NS)'),
        DeclareLaunchArgument('drive', default_value='true', description='false면 cmd_vel을 보내지 않음'),
        DeclareLaunchArgument('test', default_value='false',
                              description='ros2 bag으로 BAG_TOPICS + /rosout을 bag_dir에 녹화 (카메라 영상 제외)'),
        DeclareLaunchArgument('tracker', default_value='gesture', choices=list(TRACKER_MODES),
                              description='gesture(기본): 제스처 등록, front: 앞 사람 자동·클릭 등록, '
                                          'all: 예전 multi_tracker_node, camera: 카메라만, none: 안 띄움'),
        DeclareLaunchArgument('diag', default_value='false',
                              description='true면 visitor_follower 진단 로그 (토픽별 수신 간격·지연, 제어 주기, 끊긴 순간)'),
        DeclareLaunchArgument('auto_start', default_value='false',
                              description='true면 액션 없이 켜자마자 prepare → start (undock → 초기 위치 → 인식 → 추종)'),
        DeclareLaunchArgument('video_source', default_value='',
                              description="카메라: '0' 웹캠 / DroidCam URL (비우면 person_tracker phone_camera.yaml)"),
        DeclareLaunchArgument('bag_dir', default_value='~/arti_bags/follow',
                              description='bag 저장 폴더 (그 아래 <날짜_시각>/)'),
        OpaqueFunction(function=make_nodes),
    ])
