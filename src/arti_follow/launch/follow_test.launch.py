"""추종 테스트를 한 번에: 웹캠 사람 인식 + visitor_follower(실제 주행) + ros2 bag 녹화.

    ros2 launch arti_follow follow_test.launch.py
    → <ns>/arti/start_tour 액션을 보내면 undock → 초기 위치 → 출발 방향 보기 → 인식 기다림,
      인식 기다림 단계에서 로봇 바로 앞 사람이 자동 등록(target_person/register_front)되면
      "인식되었습니다" 안내 후 따라간다 (인식 창에서 마우스로 클릭해 바꿀 수도 있음)
      ros2 action send_goal /robot1/arti/start_tour arti_interfaces/action/StartTour "{mode: FREE}" --feedback

옵션
    drive:=false             인식·계산·로그만 하고 cmd_vel은 보내지 않음
    test:=false              ros2 bag 녹화 끄기
    bag_dir:=<폴더>          bag 저장 폴더 (기본 ~/arti_bags/follow, 그 아래 <날짜_시각>/)
    tracker:=all             인식 노드로 multi_tracker_node (바로 앞 사람 자동 등록·클릭 등록)
    tracker:=camera          카메라만 띄우고 인식 노드(multi_tracker_node)는 뺌 (인식은 다른 PC·bag 재생에서, 바로 가기: cam_test.launch.py)
    tracker:=none            카메라·인식 노드 모두 안 띄움 (인식 모듈을 따로 띄울 때)
    tracker:=gesture         (기본) gesture_test_node: 손 제스처(주먹 → 손바닥)로 대상 등록
                             (visitor_follower auto_register 끔). 좌표 토픽·형식은 multi_tracker_node와 같음
    auto_start:=true         액션 없이 켜자마자 관람 시작
    follower:=nav2           visitor_follower 대신 nav2_follow (주행은 Nav2. 전제: nav2를 nav2_follow.yaml로 띄움)
    namespace:=/robot1       기본은 $ARTI_ROBOT_NS (env.bash의 ARTI_DISCOVERY_SERVERS로 자동)
전제: AMCL(localization)이 이미 이 네임스페이스로 떠 있다 (map→odom).

뜨는 것
    phone_camera.launch.py (person_tracker)   camera_pub: 웹캠 → <ns>/top_cam/image_raw/compressed, top_cam_tf
    multi_tracker_node (person_tracker)       YOLO 인식 + LiDAR → <ns>/target_person/point_map
      또는 gesture_test_node (tracker:=gesture) 제스처 등록 + 같은 인식·LiDAR → <ns>/target_person/point_map
                                              인식 창: 사람 박스 + 로봇까지 거리(m) (타겟 초록, '~'는 LiDAR 없이 박스 크기로 추정)
    visitor_follower                          좌표 → <ns>/cmd_vel (화면 없음, 로그만)
인식 노드는 네임스페이스를 붙여 띄운다. 안 붙이면 설정 파일을 못 찾아 네임스페이스 없는 토픽을 구독해서
영상·scan을 못 받는다 (인식 창이 까맣게 나옴).

저장 (test:=true, 기본)  bag_dir/<날짜_시각>/  ros2 bag (mcap) 하나만. 로그·CSV 파일은 따로 만들지 않는다
    녹화: BAG_TOPICS + /rosout(노드 로그 메시지). 관람객 영상은 저장하지 않음 — 카메라 영상·depth는 빼고 좌표·거리·명령만
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


# ros2 bag으로 녹화할 토픽 (네임스페이스 뒤 이름).
# 관람객 영상은 저장하지 않는다(개인정보 요구사항) → top_cam 이미지, oakd 이미지·depth는 넣지 않는다.
# 카메라 토픽을 녹화하려면 팀 합의 후에.
BAG_TOPICS = [
    'scan', 'tf', 'tf_static', 'odom', 'amcl_pose',              # 로봇 센서·위치
    'target_person/point_map', 'target_person/point_base',       # 인식 모듈 출력
    'target_person/marker', 'top_cam/pose_in_map',
    'cmd_vel', 'follow/state', 'follow/enable', 'initialpose', 'guide_message', 'dock_status',                  # visitor_follower 입출력
    'target_person/register_front', 'gesture/status',            # 바로 앞 사람 등록 요청, 제스처 상태
    'artwork_zone/state', 'artwork_zone/entered', 'arti/tour_end',  # 작품 구역 (tour.launch.py), 출구 진입, 관람 끝
    'goal_update',                                               # nav2_follow → Nav2 추종 목표 (follower:=nav2)
]


# tracker 인자: all = 카메라 + 인식 노드, gesture = 카메라 + 제스처 등록 노드, camera = 카메라만, none = 둘 다 안 띄움
TRACKER_MODES = ('all', 'gesture', 'camera', 'none')


def pc_time_sync():
    """이 PC 시계가 NTP로 맞춰져 있는지 (timedatectl). 로봇 쪽 차이는 노드의 [시간] 로그로 본다."""
    try:
        out = subprocess.run(['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'],
                             capture_output=True, text=True, timeout=3).stdout.strip()
        return {'yes': '맞춰짐', 'no': '안 맞춰짐 (sudo timedatectl set-ntp true)'}.get(out, out or '알 수 없음')
    except Exception as e:
        return f'확인 실패 ({e})'


def tracker_actions(ns, mode='all'):
    """person_tracker: 카메라 launch + 인식 노드 (둘 다 같은 네임스페이스). mode=camera면 카메라만.
    mode=gesture면 인식 노드로 gesture_test_node (제스처로 대상 등록, 등록 뒤 좌표 발행).
    카메라(video_source)는 person_tracker/config/phone_camera.yaml 값 (노트북 웹캠 0)."""
    try:
        share = get_package_share_directory('person_tracker')
    except PackageNotFoundError:
        raise RuntimeError('person_tracker 패키지가 없음 → colcon build 하거나 tracker:=none') from None
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'phone_camera.launch.py')),
        launch_arguments={'namespace': ns}.items())
    tracker = Node(package='person_tracker', executable='multi_tracker_node',
                   name='multi_person_tracker_node', namespace=ns, output='screen',
                   parameters=[{'show_info_window': False,      # 화면은 인식 영상 창 하나만
                                'show_distance': True}])        # 박스마다 거리(m)
    if mode == 'gesture':
        # 카메라 TF는 phone_camera.launch.py의 top_cam_tf가 발행 → publish_cam_tf는 기본(false) 그대로
        tracker = Node(package='person_tracker', executable='gesture_test_node',
                       name='gesture_test_node', namespace=ns, output='screen',
                       parameters=[{'use_localizer': True,             # LiDAR 거리 + map 좌표 변환
                                    'publish_target_topics': True}])   # → <ns>/target_person/point_map
    if mode == 'camera':
        return [camera]
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
    follower = arg('follower').lower()
    if follower not in ('visitor', 'nav2'):
        raise RuntimeError(f'follower:={follower} → visitor | nav2 중 하나')
    params = {'start_enabled': drive, 'auto_start': arg('auto_start').lower() == 'true'}
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
        actions += tracker_actions(ns, tracker)
    if tracker == 'gesture':
        params['auto_register'] = False             # 등록은 제스처로 (register_front는 multi_tracker_node만 받음)

    actions.append(Node(
        package='arti_follow', executable='nav2_follow' if follower == 'nav2' else 'visitor_follower',
        namespace=ns, output='screen',
        parameters=[params]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=EnvironmentVariable(
            'ARTI_ROBOT_NS', default_value=''), description='로봇 네임스페이스 (기본: $ARTI_ROBOT_NS)'),
        DeclareLaunchArgument('drive', default_value='true', description='false면 cmd_vel을 보내지 않음'),
        DeclareLaunchArgument('test', default_value='true',
                              description='ros2 bag으로 BAG_TOPICS + /rosout을 bag_dir에 녹화 (카메라 영상 제외)'),
        DeclareLaunchArgument('tracker', default_value='gesture', choices=list(TRACKER_MODES),
                              description='gesture(기본): 카메라+제스처 등록 노드, all: 카메라+multi_tracker_node, '
                                          'camera: 카메라만, none: 안 띄움'),
        DeclareLaunchArgument('auto_start', default_value='false',
                              description='true면 start_tour 액션 없이 켜자마자 관람 시작 (undock → 초기 위치 → 인식 → 추종)'),
        DeclareLaunchArgument('follower', default_value='visitor',
                              description='visitor(기본): visitor_follower (직접 주행), '
                                          'nav2: nav2_follow (주행은 Nav2, nav2_follow.yaml로 nav2가 떠 있어야 함)'),
        DeclareLaunchArgument('bag_dir', default_value='~/arti_bags/follow',
                              description='bag 저장 폴더 (그 아래 <날짜_시각>/)'),
        OpaqueFunction(function=make_nodes),
    ])
