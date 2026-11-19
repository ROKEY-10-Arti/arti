"""ArtI PC2 코스 주행 ROS2 노드 (ROS2 Jazzy / Nav2).

역할:
  - [PC3 -> PC2] /robot4/arti/prepare_tour (arti_interfaces/PrepareTour 액션 서버, 키오스크 "로봇 호출")
    안전 점검 -> Undock -> 키오스크 이동 -> 관람객 등록(person_tracker) -> success, 키오스크 앞 대기.
    feedback state: UNDOCKING -> MOVING -> ARRIVED -> REGISTERING -> REGISTERED
  - [PC3 -> PC2] /robot4/arti/start_tour (arti_interfaces/StartTour 액션 서버, 키오스크 "관람 시작")
    prepare_tour 성공(키오스크 앞 대기) 뒤에만. artwork_ids 작품을 YAML 코스 순서로 관람 -> 출구 -> 도킹 -> result.
    feedback state: MOVING(+이동할 작품) / EXPLAINING / ARTWORK_COMPLETED(+작품) / RETURNING
    이동 중 관람객이 멀어지면 멈추고 come_here 안내, 10초 안 오면 도킹 복귀 (visitor_lost).
    취소(Action Cancel)·Nav2 실패도 도킹 복귀. result는 도킹이 끝난 뒤.
  - 실제 판단은 course_engine.TourMachine 이 하고, 이 노드는
    센서 데이터를 넣어 주고(tick) 나온 Event를 ROS 동작으로 실행(_dispatch)합니다.

주요 토픽 (namespace = robot4 기준):
  구독  amcl_pose, odom, scan, battery_state, dock_status, tf
        target_person/point_map                  관람객 위치 (person_tracker, 등록·거리 감시에 사용)
  발행  /arti/course_status                      상태 JSON (디버그·로그용, PC3와의 약속은 액션 2개)
        guide_message                            안내 키('register','recognized','start','tour_end' ...) -> arti_audio
        arti/tour_end                            (Empty) person_tracker 등록 대상 지우기 (호출 시작·관람 없이 종료)
  액션  navigate_to_pose(Nav2), undock, dock(irobot_create_msgs)
        explain_artwork(arti_interfaces/ExplainArtwork, 작품 번호 int) -> arti_audio guide_audio_node

map 좌표 목표는 항상 실측 설정 파일에서 읽고, 절대 임의로 만들지 않습니다.
cmd_vel은 절대 발행하지 않습니다. 이동은 Nav2 컨트롤러만 담당합니다.
"""
from __future__ import annotations
import collections
import json
import math
import time
import uuid
from typing import Optional

import rclpy
from rclpy.time import Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import BatteryState, LaserScan
from std_msgs.msg import Empty, String

from .course_engine import TourMachine, Event
from .guided_tour_plan import Pose2D, load_course_config

from arti_interfaces.action import ExplainArtwork   # 작품 해설 액션 (서버: arti_audio guide_audio_node)
from arti_interfaces.action import PrepareTour      # 로봇 호출·등록 (클라이언트: PC3 키오스크)
from arti_interfaces.action import StartTour        # 코스관람 시작 (클라이언트: PC3 키오스크)

# prepare_tour feedback 순서 (팀 명세서 3.1)
PREPARE_STATES = ('UNDOCKING', 'MOVING', 'ARRIVED', 'REGISTERING', 'REGISTERED')

# 엔진 상태 -> start_tour feedback state (팀 명세서 3.2). 그 밖의 상태는 feedback 안 바꿈
TOUR_MOVING = ('NAV_ARTWORK', 'CANCELING', 'VERIFY_STOP', 'WAIT_VISITOR',
               'NAV_EXIT', 'CANCELING_EXIT', 'VERIFY_EXIT_STOP')
TOUR_RETURNING = ('FAREWELL', 'NAV_HOME', 'CANCELING_HOME', 'VERIFY_HOME_STOP', 'DOCKING', 'WAIT_DOCK_CONFIRM')

try:
    from irobot_create_msgs.action import Undock, Dock  # TurtleBot4(Create3) 도킹 액션
    from irobot_create_msgs.msg import DockStatus
except ImportError:
    Undock = None
    Dock = None
    DockStatus = None


class ACourseNode(Node):
    def __init__(self):
        super().__init__('a_course_node')
        # ------------------------------------------------------------
        # 파라미터
        # ------------------------------------------------------------
        self.declare_parameter('config_file', '')              # a_course.yaml 경로
        self.declare_parameter('enable_motion', False)         # 안전 잠금: true여야 요청 수락
        self.declare_parameter('simulation_mode', False)       # true면 해설을 2초 모의로 처리
        self.declare_parameter('skip_undock', False)           # 시뮬레이션 전용: 언도킹 생략
        self.declare_parameter('simulation_battery_percent', -1.0)  # 시뮬레이션 배터리 값
        self.declare_parameter('minimum_battery_percent', 30.0)     # 이 값 미만이면 출발 거부
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('undock_timeout', 25.0)
        self.declare_parameter('dock_timeout', 90.0)
        self.declare_parameter('require_dock_status', True)    # Dock 후 is_docked 토픽 확인 필수
        # 명시적 모의 전용; 물리 도킹 성공으로 COMPLETED 표시하지 않음.
        self.declare_parameter('simulate_docking', False)
        self.declare_parameter('require_tf', True)             # map->base_link TF 필수 여부
        # [실물 로봇] TurtleBot4 power_saver가 도킹 중 LiDAR를 꺼서
        # scan/AMCL/TF가 언도킹 전까지 오래된 값임. 언도킹 후 다시 들어올 때까지 대기.
        self.declare_parameter('post_undock_sensor_timeout', 30.0)
        self._await_sensors_since = None   # 언도킹 후 센서 대기 시작 시각 (None = 대기 아님)
        self._sensors_ok_since = None      # 센서가 모두 신선해진 시각 (안정화 시간 계산용)
        # LiDAR 재시작 후 Nav2가 새 map->odom TF를 쓰려면 몇 초 필요.
        # 너무 빨리 목표를 보내면 'Initial robot pose is not available'로 ABORT됨.
        self.declare_parameter('post_undock_settle_time', 3.0)
        # [prepare_tour] 관람객 등록 — 값은 arti_follow visitor_follower와 같게
        self.declare_parameter('visitor_topic', 'target_person/point_map')  # person_tracker 출력 (namespace 안)
        self.declare_parameter('register_confirm_time', 1.0)   # 이만큼 끊김 없이 보이면 등록 [s]
        self.declare_parameter('register_gap', 0.5)            # 인식이 이보다 오래 끊기면 확인 다시 [s]
        self.declare_parameter('register_timeout', 60.0)       # 키오스크 도착 후 등록 제한 -> 실패, 도킹 복귀 [s]
        self.declare_parameter('ready_timeout', 130.0)         # 등록 후 start_tour 대기 제한 -> 도킹 복귀 [s] (robot1과 같음)
        self.declare_parameter('skip_register', False)         # true면 인식 없이 등록 (person_tracker 없이 시험)
        self.declare_parameter('register_guide', 'register')            # guide_lines.yaml 키: 손바닥 안내
        self.declare_parameter('recognized_guide', 'recognized')        # 인식 완료
        self.declare_parameter('register_failed_guide', 'register_failed')  # 등록 실패 / 관람객 분실
        # [start_tour] 관람 중 관람객 거리 감시 (판단 값은 arti_follow와 같게. 주행은 Nav2라 주행 값은 안 씀)
        self.declare_parameter('watch_visitor', True)          # false면 감시 안 함
        self.declare_parameter('far_distance', 1.5)            # 이보다 멀면 Nav2 goal 취소(정지) + come_here [m]
        self.declare_parameter('resume_distance', 0.95)        # 멈춘 뒤 이 안으로 오면 같은 목표 다시 전송 [m] (0.8+0.15)
        self.declare_parameter('lost_timeout', 3.0)            # 인식이 이만큼 끊기면 안 보이는 것 [s] (arti_follow)
        self.declare_parameter('visitor_give_up', 10.0)        # 멈춘 뒤 이만큼 안 오면 관람 끝 -> 도킹 복귀 [s]
        self.declare_parameter('start_guide', 'start')         # guide_lines.yaml 키: 관람 시작
        self.declare_parameter('come_here_guide', 'come_here') # 따라오세요 (관람객이 멀어짐)

        # Nav2/AMCL lifecycle 상태 확인용 클라이언트
        # _lifecycle[name] = [GetState 클라이언트, 진행 중 future, active 여부]
        from lifecycle_msgs.srv import GetState
        self._lifecycle_type = GetState
        self._lifecycle_sent = {}          # name -> 요청 보낸 시각 (재요청 판단용)
        self._lifecycle = {name: [self.create_client(GetState, name+'/get_state'), None, False]
                           for name in ('amcl','map_server','bt_navigator','controller_server','planner_server')}

        # ------------------------------------------------------------
        # 센서 / 진행 상태 변수
        # ------------------------------------------------------------
        self._pose: Optional[Pose2D] = None   # 최신 map 좌표 위치 (AMCL 또는 TF)
        self._pose_stamp = 0.0
        self._scan_stamp = 0.0
        self._odom_stamp = 0.0
        self._speed = None                    # odom 속도 (정지 판정용)
        self._battery = None                  # 배터리 %
        self._battery_stamp = 0.0
        self._pending_cancel = False          # goal 수락 전에 취소 요청이 온 경우 표시
        self._goal_handle = None              # 현재 Nav2 goal 핸들
        self._undocking = False               # Undock 액션 진행 중
        self._undock_started = 0.0
        self._undock_goal = None
        self._undock_failed_latched = False   # 언도킹 실패 후 재요청 차단(노드 재시작 필요)
        self._docking_inflight = False        # Dock 액션 진행 중
        self._dock_goal = None
        self._dock_status_stamp = 0.0
        self._is_docked = None                # dock_status.is_docked (None = 아직 수신 안 함)
        self._pending_nav = None              # 언도킹 끝난 뒤 실행할 Event 목록(키오스크 이동)
        self._nav_inflight = False            # Nav2 goal 전송 후 수락 대기 중
        self._cancel_inflight = False         # Nav2 취소 요청 응답 대기 중
        self._tf_at = 0.0                     # 마지막으로 신선한 TF를 받은 시각
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        # 늦게 도착한 옛 콜백을 무시하기 위한 세대 번호
        self._session = 0                     # 코스 요청마다 +1
        self._nav_generation = 0              # Nav2 goal마다 +1

        # ------------------------------------------------------------
        # 코스 설정 로드 -> 상태 머신 생성 (실패 시 이동 차단)
        # ------------------------------------------------------------
        self._machine = None
        self._config_error = None
        try:
            filename = self.get_parameter('config_file').value
            if not filename:
                raise ValueError('config_file 미지정')
            self._machine = TourMachine(
                load_course_config(filename),
                simulate_explanation=bool(self.get_parameter('simulation_mode').value),
                dock_timeout=float(self.get_parameter('dock_timeout').value),
                register_confirm_time=float(self.get_parameter('register_confirm_time').value),
                register_gap=float(self.get_parameter('register_gap').value),
                register_timeout=float(self.get_parameter('register_timeout').value),
                ready_timeout=float(self.get_parameter('ready_timeout').value),
                skip_register=bool(self.get_parameter('skip_register').value),
                register_guide=str(self.get_parameter('register_guide').value),
                recognized_guide=str(self.get_parameter('recognized_guide').value),
                register_failed_guide=str(self.get_parameter('register_failed_guide').value),
                watch_visitor=bool(self.get_parameter('watch_visitor').value),
                far_distance=float(self.get_parameter('far_distance').value),
                resume_distance=float(self.get_parameter('resume_distance').value),
                lost_timeout=float(self.get_parameter('lost_timeout').value),
                visitor_give_up=float(self.get_parameter('visitor_give_up').value),
                start_guide=str(self.get_parameter('start_guide').value),
                come_here_guide=str(self.get_parameter('come_here_guide').value))
        except Exception as exc:
            self._config_error = str(exc)
            self.get_logger().error(f'좌표 설정 오류. 이동 차단: {self._config_error}')

        # ------------------------------------------------------------
        # 액션 클라이언트 / 퍼블리셔 / 구독
        # ------------------------------------------------------------
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._undock_client = ActionClient(self, Undock, 'undock') if Undock else None
        self._dock_client = ActionClient(self, Dock, 'dock') if Dock else None
        self._explain_client = ActionClient(self, ExplainArtwork, 'explain_artwork')
        self._explain_goal = None
        self._status_pub = self.create_publisher(String, '/arti/course_status', 10)
        self._guide_pub = self.create_publisher(String, 'guide_message', 10)
        self._tour_end_pub = self.create_publisher(Empty, 'arti/tour_end', 10)   # person_tracker 대상 지우기
        # [실물 로봇] AMCL은 amcl_pose를 reliable + transient_local 로 발행하므로 QoS를 맞춤
        # (기본 QoS로는 마지막 위치를 받지 못함)
        amcl_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._on_pose, amcl_qos)
        self.create_subscription(Odometry, 'odom', self._on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(BatteryState, 'battery_state', self._on_battery, qos_profile_sensor_data)
        if DockStatus is not None:
            self.create_subscription(DockStatus, 'dock_status', self._on_dock_status,
                                     qos_profile_sensor_data)
        # 관람객 위치 (person_tracker: 손바닥으로 대상이 정해진 뒤부터 발행). 등록 판정에 사용
        self.create_subscription(PointStamped, self.get_parameter('visitor_topic').value,
                                 self._on_visitor, 10)
        # [PC3 -> PC2] 로봇 호출·등록 액션. 실행 함수는 끝날 때까지 기다리므로 별도 스레드 그룹에서 돈다.
        # 실행 함수는 상태를 바꾸지 않고, _tick(기본 그룹)이 판단해 둔 결과(_prep)를 읽어 feedback/result로 보낸다.
        self._prep = None          # 진행 중인 prepare_tour: dict(start, feedback, outcome, cancel)
        self._prepare_server = ActionServer(
            self, PrepareTour, 'arti/prepare_tour', self._prepare_execute,
            goal_callback=self._prepare_goal, cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        # [PC3 -> PC2] 코스관람 시작 액션. 같은 방식: 판단은 _update_tour(_tick), 실행 함수는 중계만.
        self._tour = None          # 진행 중인 start_tour: dict(ids, started, feedback(deque), outcome, cancel)
        self._tour_server = ActionServer(
            self, StartTour, 'arti/start_tour', self._tour_execute,
            goal_callback=self._tour_goal, cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
        self.create_timer(.1, self._tick)   # 10Hz 메인 루프
        self.get_logger().info('PC2 node initialized (prepare_tour, start_tour 대기); '
                               'enable_motion must be true to accept requests')

    def _now(self):
        """현재 ROS 시각(초)."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------
    # 센서 콜백: 최신 값과 수신 시각만 저장
    # ------------------------------------------------------------

    def _on_pose(self, msg):
        """amcl_pose -> map 좌표 위치 저장."""
        # AMCL / map 좌표계 필수. odom 좌표계를 map으로 쓰면 안 됨.
        if msg.header.frame_id and msg.header.frame_id.lstrip('/') != self.get_parameter('map_frame').value.lstrip('/'):
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))   # 쿼터니언 -> yaw
        self._pose = Pose2D(p.x, p.y, yaw)
        self._pose_stamp = self._now()

    def _on_odom(self, msg):
        """odom -> 속도 크기 저장 (선속도와 각속도 중 큰 값)."""
        vel = msg.twist.twist
        self._speed = max(math.hypot(vel.linear.x, vel.linear.y), abs(vel.angular.z))
        self._odom_stamp = self._now()

    def _on_scan(self, _msg):
        """scan -> LiDAR 살아있는지 확인용 수신 시각만 저장."""
        self._scan_stamp = self._now()

    def _on_battery(self, msg):
        """battery_state -> % 로 저장 (0~1 비율이면 100을 곱함)."""
        if math.isfinite(msg.percentage) and msg.percentage >= 0:
            self._battery = 100.0*msg.percentage if msg.percentage <= 1.0 else msg.percentage
            self._battery_stamp = self._now()

    def _on_dock_status(self, msg):
        """dock_status -> 도킹 여부 저장."""
        self._is_docked = bool(msg.is_docked)
        self._dock_status_stamp = self._now()

    # ------------------------------------------------------------
    # 상태 발행
    # ------------------------------------------------------------

    def _emit(self, payload):
        """상태 dict를 JSON으로 /arti/course_status 에 발행 + 로그 출력."""
        obj = dict(payload)
        obj['simulation'] = bool(self.get_parameter('simulation_mode').value)
        data = json.dumps(obj, ensure_ascii=False)
        self._status_pub.publish(String(data=data))
        self.get_logger().info(data)

    def _reject(self, text):
        """요청 거절(REJECTED) 상태 발행."""
        self._emit({'request_id': '', 'state': 'REJECTED', 'detail': text, 'completed': []})

    def _check_ready(self):
        """출발 전 안전 점검. 문제가 있으면 이유 문자열, 모두 OK면 None."""
        now = self._now()
        if self._machine is None:
            return self._config_error or '코스 설정 오류'
        if not self.get_parameter('enable_motion').value:
            return 'enable_motion=false (안전 잠금)'
        docked = (self._is_docked is True and now - self._dock_status_stamp < 3.0)
        if self._pose is None:
            return 'AMCL map 위치 정보 없음 (RViz 2D Pose Estimate 필요)'
        if now - self._odom_stamp > 3.0:
            return 'Odometry 신호가 없음'
        # [실물 로봇] 도킹 중엔 LiDAR가 꺼져 있음(power_saver).
        # 아래 항목은 언도킹 후 _tick 에서 다시 확인합니다.
        if not docked:
            if now - self._pose_stamp > 3.0:
                return 'AMCL map 위치 정보가 신선하지 않음'
            if now - self._scan_stamp > 3.0:
                return 'LiDAR scan 신호가 없음'
            if self.get_parameter('require_tf').value and now - self._tf_at > 3.0:
                return 'map -> base_link TF 변환 미수신 또는 만료'
        if not self._nav_client.server_is_ready():
            return 'Nav2 navigate_to_pose 액션 미준비'
        if not all(v[2] for v in self._lifecycle.values()):
            return 'AMCL/map_server/Nav2 lifecycle not active'
        # 해설 수단 준비 확인 (simulation_mode면 생략)
        if not self.get_parameter('simulation_mode').value:
            if not self._explain_client.server_is_ready():
                return 'explain_artwork 액션 서버 미준비 (arti_audio guide_audio_node 실행 확인)'
        # 작품 4까지 다 돌고 나서야 Dock 서버가 없다는 걸 알면 안 되므로 출발 전에 확인.
        mock_dock = (self.get_parameter('simulation_mode').value and
                     self.get_parameter('simulate_docking').value)
        if not mock_dock and (self._dock_client is None or not self._dock_client.server_is_ready()):
            return 'Dock 액션 서버 미준비 (simulation_mode에서 simulate_docking:=true는 모의 검증 전용)'
        # 배터리 확인 (5초 이내 값만 사용)
        battery = self._battery if (now - self._battery_stamp < 5.0) else None
        if battery is None and self.get_parameter('simulation_mode').value:
            fallback = float(self.get_parameter('simulation_battery_percent').value)
            battery = fallback if fallback >= 0 else None
        if battery is None:
            return '배터리 정보 없음 (시뮬레이션이면 시험값을 명시해야 함)'
        if battery < self.get_parameter('minimum_battery_percent').value:
            return f'배터리 부족: {battery:.1f}%'
        return None

    # ------------------------------------------------------------
    # 코스 요청 -> 언도킹
    # ------------------------------------------------------------

    def _busy(self):
        """무언가 진행 중이면 True (중복 요청 거절용)."""
        return (self._undocking or self._await_sensors_since is not None or self._docking_inflight
                or self._dock_goal is not None or self._nav_inflight or self._goal_handle is not None
                or self._undock_failed_latched
                or (self._machine is not None and self._machine.state not in ('IDLE', 'DONE', 'ERROR')))

    def _begin_course(self, request_id: str) -> Optional[str]:
        """[prepare_tour] 점검 통과 시 언도킹부터 시작 (키오스크에서 등록 후 멈춤).
        거절하면 이유 문자열, 시작하면 None."""
        if self._busy():
            return '코스주행 중복 요청'
        reason = self._check_ready()
        if reason:
            return reason
        if self._is_docked is None or self._now()-self._dock_status_stamp > 3.0:
            return '최신 dock_status 없음'
        self._session += 1
        self._pending_cancel = False
        self._goal_handle = None
        try:
            # 언도킹 '전'의 정확한 도킹 위치를 먼저 기록.
            # Nav2 복귀 목표는 언도킹 '후' 충돌 없는 위치로 갱신됨.
            events = self._machine.start(request_id, self._now(), self._pose)
        except ValueError as exc:
            return str(exc)
        if self._is_docked is False:
            # 이미 언도킹 상태: 바로 키오스크로 출발
            self._machine.set_post_undock_pose(self._pose)
            self._dispatch(events)
        elif self.get_parameter('skip_undock').value:
            # 시뮬레이션 전용: 언도킹 생략
            if not self.get_parameter('simulation_mode').value:
                self._dispatch(self._machine.fail('실기에서는 도킹 해제를 임의로 건너뛸 수 없습니다'))
                return None
            self._emit({'request_id': request_id, 'state': 'UNDOCK_SKIPPED',
                        'detail': '시뮬레이터 시작 시 이미 undocked라고 가정', 'completed': []})
            self._dispatch(events)
        else:
            # 도킹 상태: Undock 액션 실행, 키오스크 이동 Event는 끝난 뒤로 보류
            if self._undock_client is None or not self._undock_client.server_is_ready():
                self._dispatch(self._machine.fail('Undock 액션 서버 미준비'))
                return None
            self._undocking = True
            self._undock_started = self._now()
            self._pending_nav = events
            future = self._undock_client.send_goal_async(Undock.Goal())
            future.add_done_callback(self._undock_accepted)
        return None

    # ------------------------------------------------------------
    # [PC3 -> PC2] prepare_tour 액션 (로봇 호출·등록)
    #   스레드 나눔: _prepare_execute(액션 스레드)는 기다리며 feedback/result만 보내고,
    #   시작·판단·취소는 전부 _update_prepare(_tick, 기본 스레드)가 한다.
    # ------------------------------------------------------------

    def _prepare_goal(self, goal_request):
        """goal 받을지: start=false거나 이미 호출 처리 중이면 거절 (액션 스레드, 읽기만)."""
        if not goal_request.start:
            self.get_logger().warn('prepare_tour 거절: start=false')
            return GoalResponse.REJECT
        if self._prep is not None:
            self.get_logger().warn('prepare_tour 거절: 이미 호출 처리 중')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _prepare_execute(self, goal):
        """[액션 스레드] prepare_tour goal 하나를 끝까지: 시작 요청 -> feedback 중계 -> result."""
        prep = {'start': None, 'feedback': None, 'outcome': None, 'cancel': False}
        self._prep = prep                       # _tick이 다음 주기에 시작 처리
        sent = -1                               # 마지막으로 보낸 feedback 순번 (PREPARE_STATES)
        try:
            while rclpy.ok():
                if goal.is_cancel_requested:
                    prep['cancel'] = True       # _tick이 복귀 처리 후 outcome = canceled
                outcome = prep['outcome']
                state = 'REGISTERED' if outcome and outcome[0] == 'success' else prep['feedback']
                if state in PREPARE_STATES and PREPARE_STATES.index(state) > sent:
                    # 짧게 지나간 단계(ARRIVED 등)도 빠짐없이 순서대로 보냄.
                    # 처음 보내는 게 UNDOCKING이 아니면(이미 언독 상태) MOVING부터.
                    idx = PREPARE_STATES.index(state)
                    first = sent + 1 if sent >= 0 else (0 if state == 'UNDOCKING' else 1)
                    for name in PREPARE_STATES[first:idx+1]:
                        goal.publish_feedback(PrepareTour.Feedback(state=name))
                    sent = idx
                if outcome is not None:
                    kind, message = outcome
                    if kind == 'success':
                        goal.succeed()
                    elif kind == 'canceled' and goal.is_cancel_requested:
                        goal.canceled()
                    else:
                        goal.abort()
                    self.get_logger().info(f'prepare_tour 끝: {kind} ({message})')
                    return PrepareTour.Result(success=kind == 'success', message=message)
                time.sleep(0.1)
            goal.abort()
            return PrepareTour.Result(success=False, message='shutdown')
        finally:
            self._prep = None

    def _update_prepare(self, now):
        """[_tick] 진행 중인 prepare_tour의 시작·feedback·결과·취소를 판단해 _prep에 적어 둔다."""
        prep = self._prep
        if prep is None or prep['outcome'] is not None:
            return
        m = self._machine
        # (1) 시작: 점검 -> 언독 -> 키오스크 (거절이면 바로 실패)
        if prep['start'] is None:
            self._tour_end_pub.publish(Empty())         # 지난 관람객 등록 지우기
            reason = self._begin_course(str(uuid.uuid4()))
            prep['start'] = reason or ''
            if reason:
                self._reject(reason)
                prep['outcome'] = ('fail', reason)
            return
        # (2) 취소 (PC3 Action Cancel): 언독 중이면 끝날 때까지 기다렸다가 복귀
        if prep['cancel'] and not (self._undocking or self._await_sensors_since is not None):
            if m.state not in ('IDLE', 'DONE', 'ERROR'):
                self._abort_home('canceled', now)
                prep['outcome'] = ('canceled', 'canceled')
                return
        # (3) 지금 단계 -> feedback 이름 / 끝났는지
        if self._undocking or self._await_sensors_since is not None:
            prep['feedback'] = 'UNDOCKING'
        elif m.state in ('NAV_KIOSK', 'CANCELING'):
            prep['feedback'] = 'MOVING'
        elif m.state == 'VERIFY_STOP':
            prep['feedback'] = 'ARRIVED'
        elif m.state == 'REGISTERING':
            prep['feedback'] = 'REGISTERING'
        elif m.state == 'WAIT_START':
            prep['feedback'] = 'REGISTERED'
            prep['outcome'] = ('success', 'registered')
        elif m.state == 'ERROR':
            prep['outcome'] = ('fail', m.fail_reason or 'error')
        elif m.end_reason:
            # 등록 실패·Nav2 실패 등으로 도킹 복귀 시작
            self._tour_end_pub.publish(Empty())
            prep['outcome'] = ('fail', m.end_reason)

    def _on_visitor(self, msg):
        """target_person/point_map 수신 -> 인식 시각·map 좌표 기록 (등록 판정, 관람 중 거리 감시)."""
        if self._machine is not None:
            self._machine.visitor_seen(self._now(), (msg.point.x, msg.point.y))

    def _abort_home(self, reason, now):
        """[_tick] 관람 그만두고 도킹 복귀: 해설 중이면 끊고, person_tracker 대상도 지움."""
        if self._explain_goal is not None:
            self._explain_goal.cancel_goal_async()
            self._explain_goal = None
        self._dispatch(self._machine.abort_to_home(reason, now))
        self._tour_end_pub.publish(Empty())

    # ------------------------------------------------------------
    # [PC3 -> PC2] start_tour 액션 (코스관람 시작)
    # ------------------------------------------------------------

    def _tour_goal(self, goal_request):
        """goal 받을지: 이미 관람 중이면 거절. 준비(WAIT_START) 여부는 _update_tour가 보고 실패로 알림."""
        if self._tour is not None:
            self.get_logger().warn('start_tour 거절: 이미 관람 중')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _tour_execute(self, goal):
        """[액션 스레드] start_tour goal 하나를 끝까지: feedback 중계 -> 도킹 후 result."""
        tour = {'ids': list(goal.request.artwork_ids), 'started': False, 'last': None,
                'done_count': 0, 'feedback': collections.deque(), 'outcome': None, 'cancel': False}
        self._tour = tour
        try:
            while rclpy.ok():
                if goal.is_cancel_requested:
                    tour['cancel'] = True
                while tour['feedback']:
                    state, art = tour['feedback'].popleft()
                    goal.publish_feedback(StartTour.Feedback(state=state, artwork_id=art))
                if tour['outcome'] is not None and not tour['feedback']:
                    kind, message = tour['outcome']
                    if kind == 'success':
                        goal.succeed()
                    elif kind == 'canceled' and goal.is_cancel_requested:
                        goal.canceled()
                    else:
                        goal.abort()
                    self.get_logger().info(f'start_tour 끝: {kind} ({message})')
                    return StartTour.Result(success=kind == 'success', message=message)
                time.sleep(0.1)
            goal.abort()
            return StartTour.Result(success=False, message='shutdown')
        finally:
            self._tour = None

    def _update_tour(self, now):
        """[_tick] 진행 중인 start_tour의 시작·feedback·결과·취소 판단."""
        tour = self._tour
        if tour is None or tour['outcome'] is not None:
            return
        m = self._machine
        # (1) 시작: prepare_tour로 등록된 뒤(WAIT_START)에만
        if not tour['started']:
            tour['started'] = True
            reason = self._check_ready() if m.state == 'WAIT_START' else 'not_prepared'
            if not reason:
                try:
                    self._dispatch(m.begin_tour(tour['ids'], now))
                except ValueError as exc:
                    reason = str(exc)
            if reason:
                self.get_logger().warn(f'start_tour 거절: {reason}')
                tour['outcome'] = ('fail', reason)
            return
        # (2) 취소: 도킹 복귀 (result는 도킹 뒤 canceled)
        if tour['cancel'] and not m.end_reason and m.state not in ('DONE', 'ERROR'):
            self._abort_home('canceled', now)
        # (3) 작품 설명 완료 -> ARTWORK_COMPLETED (작품마다 한 번)
        while tour['done_count'] < len(m.completed):
            tour['feedback'].append(('ARTWORK_COMPLETED', int(m.completed[tour['done_count']])))
            tour['done_count'] += 1
        # (4) 지금 상태 -> feedback (바뀔 때만)
        if m.state in TOUR_MOVING:
            moving_to = (m.plan.artworks[m.index].artwork_id
                         if 0 <= m.index < len(m.plan.artworks) and not m.exit_pending else 0)
            current = ('MOVING', int(moving_to))
        elif m.state == 'EXPLAINING':
            current = ('EXPLAINING', int(m.plan.artworks[m.index].artwork_id))
        elif m.state in TOUR_RETURNING:
            current = ('RETURNING', 0)
        else:
            current = tour['last']
        if current is not None and current != tour['last']:
            tour['feedback'].append(current)
            tour['last'] = current
        # (5) 끝: 도킹까지 끝난 뒤 result
        if m.state == 'DONE':
            if not m.end_reason:
                tour['outcome'] = ('success', 'completed')
            else:
                tour['outcome'] = ('canceled' if m.end_reason == 'canceled' else 'fail', m.end_reason)
        elif m.state == 'ERROR':
            tour['outcome'] = ('fail', m.fail_reason or 'error')

    def _undock_accepted(self, future):
        """Undock goal 수락 여부 확인 -> 결과 대기 등록."""
        try:
            gh = future.result()
            if not gh.accepted:
                raise RuntimeError('Undock goal rejected')
            self._undock_goal = gh
            if not self._undocking:
                # 그 사이 타임아웃 났으면 취소
                self._undock_failed_latched = True
                gh.cancel_goal_async()
                return
            gh.get_result_async().add_done_callback(self._undock_finished)
        except Exception as exc:
            self._undocking = False
            self._undock_failed_latched = True
            self._dispatch(self._machine.fail(f'Undock 실패: {exc}'))

    def _undock_finished(self, future):
        """Undock 완료 -> 바로 출발하지 않고 센서 재수신 대기 시작."""
        if not self._undocking:
            return  # 타임아웃 후 늦게 온 콜백; 운영자 개입 필요
        self._undocking = False
        self._undock_goal = None
        try:
            response = future.result()
            if response.status != GoalStatus.STATUS_SUCCEEDED:
                raise RuntimeError(f'Undock status {response.status}')
            # [실물 로봇] 언도킹 후 LiDAR가 다시 켜짐. _tick 이 scan + AMCL/TF가
            # 신선해질 때까지 기다린 뒤 복귀 위치 저장 및 Nav2 시작.
            self._await_sensors_since = self._now()
            self._emit({'request_id': self._machine.request_id, 'state': 'UNDOCKED_WAITING_SENSORS',
                        'detail': 'LiDAR/AMCL/TF 재수신 대기', 'completed': []})
        except Exception as exc:
            self._undock_failed_latched = True
            self._pending_nav = None
            self._dispatch(self._machine.fail(f'Undock 실패: {exc}'))

    def _finish_post_undock(self, now):
        """센서 안정화 완료 -> 복귀 위치 저장 후 보류해 둔 키오스크 이동 실행."""
        self._await_sensors_since = None
        events, self._pending_nav = self._pending_nav, None
        try:
            origin = self._machine.dock_origin
            # 현재 위치에서 도킹 원점을 바라보는 자세 (YAML dock.approach_pose가 있으면 그게 우선)
            home = Pose2D(self._pose.x, self._pose.y, math.atan2(origin.y-self._pose.y, origin.x-self._pose.x))
            self._machine.set_post_undock_pose(home)
            hg = self._machine.home_goal
            self._emit({'request_id': self._machine.request_id, 'state': 'UNDOCKED',
                        'detail': f'언도킹 위치 ({self._pose.x:.3f}, {self._pose.y:.3f}); 복귀 목표 ({hg.x:.3f}, {hg.y:.3f})',
                        'completed': []})
            self._dispatch(events)
        except Exception as exc:
            self._undock_failed_latched = True
            self._dispatch(self._machine.fail(f'Undock 후 처리 실패: {exc}'))

    # ------------------------------------------------------------
    # 상태 머신 Event -> 실제 ROS 동작
    # ------------------------------------------------------------

    def _dispatch(self, events: list[Event]):
        """TourMachine이 돌려준 Event를 하나씩 실행."""
        for event in events:
            if event.kind == 'status':
                self._emit(event.value)                         # 상태 발행
            elif event.kind == 'navigate':
                name, pose = event.value
                self._send_nav(name, pose)                      # Nav2 목표 전송
            elif event.kind == 'cancel':
                self._cancel_nav()                              # Nav2 목표 취소
            elif event.kind == 'guide':
                self._guide_pub.publish(String(data=str(event.value)))  # 안내 문구(TTS 노드가 재생)
            elif event.kind == 'dock':
                self._start_docking()                           # Dock 액션
            elif event.kind == 'explain':
                self._emit({'request_id': self._machine.request_id,
                            'state': 'EXPLANATION_REQUESTED',
                            'detail': str(event.value), 'completed': list(self._machine.completed)})
                if not self.get_parameter('simulation_mode').value:
                    self._start_explanation(int(event.value))   # 작품 번호로 해설 요청

    # ------------------------------------------------------------
    # Nav2 NavigateToPose
    # ------------------------------------------------------------

    def _send_nav(self, name, pose):
        """Pose2D 목표를 NavigateToPose goal로 전송."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.get_parameter('map_frame').value
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = pose.x
        goal.pose.pose.position.y = pose.y
        # yaw -> 쿼터니언 (z축 회전만)
        goal.pose.pose.orientation.z = math.sin(pose.yaw / 2)
        goal.pose.pose.orientation.w = math.cos(pose.yaw / 2)
        self._goal_handle = None
        self._nav_inflight = True
        self._cancel_inflight = False
        self._pending_cancel = False
        self._nav_generation += 1
        session = (self._session, self._nav_generation)   # 이 goal의 고유 번호
        self._emit({'request_id': self._machine.request_id, 'state': 'NAVIGATING',
                    'detail': str(name), 'completed': list(self._machine.completed)})
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._nav_accepted(f, session))

    def _nav_accepted(self, future, session):
        """Nav2 goal 수락 응답."""
        if session != (self._session, self._nav_generation):
            return   # 이미 새 goal을 보냈으면 옛 응답은 무시
        try:
            gh = future.result()
            self._nav_inflight = False
            if not gh.accepted:
                self._dispatch(self._machine.nav_result('REJECTED', self._now()))
                return
            self._goal_handle = gh
            gh.get_result_async().add_done_callback(lambda f: self._nav_finished(f, session))
            if self._pending_cancel:
                # 수락 전에 취소 요청이 와 있었으면 지금 취소
                self._cancel_nav()
        except Exception as exc:
            self._nav_inflight = False
            self._dispatch(self._machine.fail(f'Nav2 전송 실패: {exc}'))

    def _nav_finished(self, future, session):
        """Nav2 goal 최종 결과 -> TourMachine.nav_result 로 전달."""
        if session != (self._session, self._nav_generation):
            return
        self._goal_handle = None
        self._cancel_inflight = False
        try:
            status = future.result().status
            name = {GoalStatus.STATUS_SUCCEEDED:'SUCCEEDED',
                    GoalStatus.STATUS_CANCELED:'CANCELED',
                    GoalStatus.STATUS_ABORTED:'ABORTED'}.get(status, f'STATUS_{status}')
            self._dispatch(self._machine.nav_result(name, self._now()))
        except Exception as exc:
            self._dispatch(self._machine.fail(f'Nav2 결과 확인 실패: {exc}'))

    def _cancel_nav(self):
        """현재 Nav2 goal 취소. 아직 수락 전이면 수락되는 즉시 취소하도록 표시."""
        if self._cancel_inflight:
            return
        if self._goal_handle is None:
            self._pending_cancel = True
            return
        self._pending_cancel = False
        self._cancel_inflight = True
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self._on_cancel_ack)

    def _on_cancel_ack(self, future):
        """취소 요청 응답 확인 (결과 자체는 _nav_finished 로 CANCELED가 옴)."""
        self._cancel_inflight = False
        try:
            if not future.result().goals_canceling and self._machine.state == 'CANCELING':
                self._dispatch(self._machine.fail('Nav2 목표 취소 승인 실패'))
        except Exception as exc:
            self._dispatch(self._machine.fail(f'Nav2 목표 취소 응답 실패: {exc}'))

    # ------------------------------------------------------------
    # Dock 액션
    # ------------------------------------------------------------

    def _start_docking(self):
        """충전 스테이션 접근점 도착 후 Dock 액션 실행."""
        if self._machine.state != 'DOCKING':
            return
        if (self.get_parameter('simulation_mode').value and
                self.get_parameter('simulate_docking').value):
            # 모의 도킹 (시뮬레이션 전용)
            self._dispatch(self._machine.dock_simulated(self._now()))
            return
        if self._dock_client is None or not self._dock_client.server_is_ready():
            self._dispatch(self._machine.fail('Dock 액션 서버 없음'))
            return
        self._docking_inflight = True
        session = self._session
        self._emit({'request_id': self._machine.request_id, 'state': 'DOCK_REQUESTED',
                    'detail': 'irobot_create_msgs/action/Dock 요청',
                    'completed': list(self._machine.completed)})
        try:
            future = self._dock_client.send_goal_async(Dock.Goal())
            future.add_done_callback(lambda f: self._dock_accepted(f, session))
        except Exception as exc:
            self._docking_inflight = False
            self._dispatch(self._machine.fail(f'Dock 목표 전송 실패: {exc}'))

    def _dock_accepted(self, future, session):
        """Dock goal 수락 응답."""
        if session != self._session:
            return
        try:
            handle = future.result()
            if not handle.accepted:
                raise RuntimeError('Dock goal rejected')
            self._dock_goal = handle
            if self._machine.state != 'DOCKING':
                handle.cancel_goal_async()
                return
            handle.get_result_async().add_done_callback(
                lambda f: self._dock_finished(f, session))
        except Exception as exc:
            self._docking_inflight = False
            self._dispatch(self._machine.fail(f'Dock 요청 거절/오류: {exc}'))

    def _dock_finished(self, future, session):
        """Dock 결과. 성공 후에는 _tick 에서 dock_status.is_docked 를 확인해야 COMPLETED."""
        if session != self._session:
            return
        self._docking_inflight = False
        self._dock_goal = None
        if self._machine.state != 'DOCKING':
            return  # 이미 타임아웃으로 실패 처리됨; 늦은 응답으로 성공 표시하지 않음
        try:
            response = future.result()
            success = response.status == GoalStatus.STATUS_SUCCEEDED and bool(response.result.is_docked)
            self._dispatch(self._machine.docking_action_result(success, self._now()))
            if success and not self.get_parameter('require_dock_status').value:
                # Dock 액션은 성공했고, 이 설정에서는 dock_status 확인을 요구하지 않음.
                self._dispatch(self._machine.dock_confirmed(self._now()))
        except Exception as exc:
            self._dispatch(self._machine.fail(f'Dock 결과 조회 실패: {exc}'))

    # ------------------------------------------------------------
    # 메인 루프 (10Hz)
    # ------------------------------------------------------------

    def _tick(self):
        """0.1초마다: lifecycle 확인 -> TF 위치 갱신 -> 언도킹 후 대기 -> 상태 머신 진행."""
        if self._machine is None:
            return
        now = self._now()
        self._update_prepare(now)   # prepare_tour 시작·feedback·결과·취소
        self._update_tour(now)      # start_tour 시작·feedback·결과·취소
        # Nav2 이동 실패(WAIT_RECOVERY): PC3에 재시도 수단이 없으므로 도킹 복귀. 복귀 중 실패면 멈추고 ERROR
        if self._machine.state == 'WAIT_RECOVERY':
            if self._machine.returning:
                self._dispatch(self._machine.fail(f'복귀 실패: {self._machine.recover_reason}'))
            else:
                self.get_logger().warn(f'Nav2 실패 -> 도킹 복귀: {self._machine.recover_reason}')
                self._abort_home('nav_failed', now)

        # (1) Nav2/AMCL 노드 lifecycle 상태 폴링 (id 3 = active)
        for client, future, active in list(self._lifecycle.values()):
            key = next(k for k,v in self._lifecycle.items() if v[0] is client)
            # [실물 로봇] discovery server 환경에서 get_state 요청이 유실될 수 있음.
            # 무한 대기하지 않고 2초 뒤 다시 요청.
            if future is not None and not future.done() and now - self._lifecycle_sent.get(key, now) > 2.0:
                client.remove_pending_request(future)
                self._lifecycle[key][1] = future = None
            if future is not None and future.done():
                try: self._lifecycle[key][2] = future.result().current_state.id == 3
                except Exception: self._lifecycle[key][2] = False
                self._lifecycle[key][1] = None
            elif future is None and client.service_is_ready():
                self._lifecycle[key][1] = client.call_async(self._lifecycle_type.Request())
                self._lifecycle_sent[key] = now

        # (2) map -> base_link TF로 위치 갱신 (amcl_pose보다 자주 들어옴)
        try:
            tf = self._tf_buffer.lookup_transform(self.get_parameter('map_frame').value, 'base_link', Time())
            if tf.header.stamp.sec or tf.header.stamp.nanosec:
                tf_time = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
                age = self.get_clock().now().nanoseconds * 1e-9 - tf_time
                if 0 <= age < 3.0:
                    self._tf_at = now
                    p = tf.transform.translation
                    q = tf.transform.rotation
                    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
                    self._pose = Pose2D(p.x, p.y, yaw)
                    self._pose_stamp = now
        except Exception:
            pass

        # (3) [실물 로봇] 언도킹 후 센서 재수신 대기
        #     pose/scan/TF 모두 신선 상태가 settle_time(3초) 연속 유지되면 코스 시작.
        #     30초 안에 안 되면 실패.
        if self._await_sensors_since is not None:
            fresh = (self._pose is not None and now - self._pose_stamp < 3.0
                     and now - self._scan_stamp < 3.0
                     and (not self.get_parameter('require_tf').value or now - self._tf_at < 3.0))
            if not fresh:
                self._sensors_ok_since = None
            elif self._sensors_ok_since is None:
                self._sensors_ok_since = now
            elif now - self._sensors_ok_since >= float(self.get_parameter('post_undock_settle_time').value):
                self._sensors_ok_since = None
                self._finish_post_undock(now)
                return
            if now - self._await_sensors_since > float(self.get_parameter('post_undock_sensor_timeout').value):
                self._await_sensors_since = None
                self._pending_nav = None
                self._undock_failed_latched = True
                self._dispatch(self._machine.fail('언도킹 후 LiDAR/AMCL/TF 미수신 타임아웃'))
            return

        # (4) 언도킹 진행 중: 타임아웃 감시만
        if self._undocking:
            if now - self._undock_started > float(self.get_parameter('undock_timeout').value):
                self._undocking = False
                self._undock_failed_latched = True
                if self._undock_goal is not None:
                    self._undock_goal.cancel_goal_async()
                self._dispatch(self._machine.fail('Undock 타임아웃; 로봇 상태 수동 점검·노드 재시작 필요'))
            return

        # (5) 코스 진행 중: 상태 머신 tick
        if self._machine.state in ('NAV_KIOSK', 'NAV_ARTWORK', 'NAV_HOME', 'NAV_EXIT', 'CANCELING',
                                   'CANCELING_HOME', 'CANCELING_EXIT',
                                   'VERIFY_STOP', 'VERIFY_HOME_STOP', 'VERIFY_EXIT_STOP',
                                   'FAREWELL', 'EXPLAINING', 'REGISTERING', 'WAIT_START', 'WAIT_VISITOR',
                                   'DOCKING', 'WAIT_DOCK_CONFIRM'):
            pose_age = now - self._pose_stamp if self._pose else float('inf')
            odom_age = now - self._odom_stamp
            events = self._machine.tick(now, self._pose, pose_age, self._speed, odom_age)
            self._dispatch(events)
            # Dock 성공 후 dock_status.is_docked == True 확인되면 최종 완료
            if (self._machine.state == 'WAIT_DOCK_CONFIRM' and self._is_docked is True
                    and 0 <= now-self._dock_status_stamp < 3.0):
                self._dispatch(self._machine.dock_confirmed(now))
            # 오류 발생 시 진행 중인 액션 모두 취소 (로봇 정지)
            if self._machine.state == 'ERROR':
                if self._goal_handle is not None or self._nav_inflight:
                    self._cancel_nav()
                if self._explain_goal is not None:
                    self._explain_goal.cancel_goal_async()
                if self._dock_goal is not None:
                    self._dock_goal.cancel_goal_async()

    # ------------------------------------------------------------
    # 해설(TTS) 요청
    # ------------------------------------------------------------

    def _start_explanation(self, artwork_id):
        """작품 해설 요청: explain_artwork 액션 goal(작품 번호 int) 전송.
        guide_audio_node가 이 번호로 PC3 /arti/description에 설명을 묻고 읽어 준다."""
        if not self._explain_client.server_is_ready():
            self._dispatch(self._machine.fail('explain_artwork 액션 서버 미준비 (arti_audio guide_audio_node)'))
            return
        goal = ExplainArtwork.Goal()
        goal.artwork_id = artwork_id
        self.get_logger().info(f'작품 {artwork_id} 해설 요청 (explain_artwork)')
        token = self._session
        self._explain_client.send_goal_async(goal).add_done_callback(
            lambda f: self._explain_accepted(f, token, artwork_id))

    def _explain_accepted(self, future, token, artwork_id):
        """ExplainArtwork goal 수락 응답 (action 방식)."""
        if token != self._session or self._machine.state != 'EXPLAINING':
            return
        try:
            gh = future.result()
            if not gh.accepted:
                raise RuntimeError('ExplainArtwork goal rejected')
            self._explain_goal = gh
            gh.get_result_async().add_done_callback(
                lambda f: self._explain_finished(f, token, artwork_id))
        except Exception as exc:
            self._dispatch(self._machine.fail(f'해설 요청 실패: {exc}'))

    def _explain_finished(self, future, token, artwork_id):
        """ExplainArtwork 결과 (action 방식)."""
        if token != self._session:
            return
        self._explain_goal = None
        try:
            response = future.result()
            ok = (response.status == GoalStatus.STATUS_SUCCEEDED
                  and bool(response.result.completed))
            self._dispatch(self._machine.explanation_result(artwork_id, ok, self._now()))
        except Exception as exc:
            self._dispatch(self._machine.fail(f'해설 결과 실패: {exc}'))


def main(args=None):
    rclpy.init(args=args)
    node = ACourseNode()
    # prepare_tour 실행 함수가 기다리는 동안에도 _tick·콜백이 돌도록 스레드 여러 개
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass   # Ctrl+C 정상 종료
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()
