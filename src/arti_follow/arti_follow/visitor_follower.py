"""관람객 추종 주행 — 플로우차트(tracking_pc_ver3) '추적 모듈'의 움직임.

파일 구성 (노드는 visitor_follower 하나. 기능별로 파일만 나눔, 클래스 하나가 셋을 상속)
  visitor_follower.py  노드 본체: 파라미터, 구독·발행·액션 서버 등록, 입력 콜백, control/step(상태 분기), 진단, main
  prepare_tour.py      ① 호출·등록 (prepare_tour): run_prepare, undock, localize, face_start, wait_visitor, registered_wait
  start_tour.py        ② 관람 (start_tour): run_tour, follow, search, end_tour, 작품 설명 feedback
  motion.py            주행·추정 공통: PersonKF(칼만 필터), face, obstacles·field·dwa, ramp, drive

입력  target_person/point_map (PointStamped, frame_id map)  인식 모듈이 준 관람객 좌표
                                  → TF(map→odom)로 odom 좌표로 바꿔 씀 (localization 필요)
      scan (LaserScan)            벽·기둥 등 장애물 (OAK-D depth는 쓰지 않음, 2026-11-18 제거)
      tf, tf_static               좌표 변환
      dock_status (irobot_create_msgs/DockStatus)   도크에 붙어 있는지 (붙어 있을 때만 undock)
      arti/prepare_tour (arti_interfaces/PrepareTour, 액션 서버)   호출·관람객 등록 (undock → 위치 잡기 → 키오스크 보기 → 등록)
                                  feedback: state(UNDOCKING/MOVING/ARRIVED/REGISTERING/REGISTERED)
                                  result: success(등록 완료), message(실패 이유)
      arti/start_tour (arti_interfaces/StartTour, 액션 서버)   자유관람 시작 (artwork_ids = []만, prepare_tour 성공 뒤에만).
                                  취소하면 관람 끝. feedback: state(FOLLOW/EXPLAINING/ARTWORK_COMPLETED), artwork_id
                                  result: success, message(끝난 이유)   (명세: docs/common/ros2_interfaces.md)
      artwork_zone/explaining (Int32)    설명 중인 작품 번호 (0 = 없음) → feedback EXPLAINING + artwork_id
      artwork_zone/explained (Int32)     끝까지 설명한 작품 번호 → feedback ARTWORK_COMPLETED 한 번
      artwork_zone/entered (String)      관람객이 들어간 안내 구역 zone_id → end_zone(exit)이면 end_guide_wait 동안 서서
                                         종료 안내를 듣게 한 뒤 관람 끝(completed) → 도크 복귀
                                  작품 ID는 int (artworks.yaml artwork_id = PC3 작품 DB 번호)
출력  cmd_vel (TwistStamped), follow/state (String)
      undock (irobot_create_msgs/Undock, 액션 클라이언트)     Create 3 언독
      dock (irobot_create_msgs/Dock, 액션 클라이언트)         관람이 끝나면 언독 지점으로 돌아와 Create 3 도킹
      initialpose (PoseWithCovarianceStamped)      AMCL 초기 위치 = init_pose.yaml의 출발 지점(undock)
      guide_message (String)                       관람객을 인식하면 recognized_guide 키 → arti_audio가 말함
      arti/tour_end (Empty)                        관람이 끝나면(취소·LOST·실패) 한 번 → 인식 모듈·artwork_zone이
                                                   그 관람객 데이터(re-ID 특징, 좌표 기록 등)를 지운다. 로봇 1대 = 관람객 1명
      target_person/register_front (Empty)         WAIT_VISITOR에서 인식 모듈에 '바로 앞 사람을 등록' 요청 (클릭 없이 자동 등록)
진단  diag일 때 diag_period마다 토픽별 수신 주기·지연·콜백 시간, 제어 주기·계산 시간, 정지 이유를 로그로 낸다.
      (화면 로그만. 테스트 기록은 follow_test.launch.py의 ros2 bag)

상태 (플로우차트 순서). 시작은 READY — arti/prepare_tour 액션이 와야 움직인다 (auto_start면 켜자마자 prepare+start)
  READY       액션 기다림. cmd_vel을 보내지 않음 (텔레옵 등 다른 노드와 안 겹침)
  ── 여기부터 REGISTERED까지 = prepare_tour ──
  UNDOCK      Create 3 undock 액션 (로봇이 스스로 움직임 → cmd_vel 안 보냄). 관람은 항상 도크에서 시작 → 도크 밖이면 먼저 Create 3 dock으로 올린 뒤 undock
  LOCALIZE    (initialpose = start_point(dock)는 언독 직전 도크 위에서 이미 보냄) 바로 START_FACE (AMCL 수렴 안 기다림)
  START_FACE  언독 지점(face_from_point)에서 face_zone(기본 키오스크, arti_common artworks.yaml) 중심 쪽으로 돌 각도를 계산해
              odom 기준으로 그만큼 제자리 회전 (map 위치 안 씀). 다 돌거나 face_timeout 지나면 WAIT_VISITOR
  WAIT_VISITOR 관람객 인식 기다림 (이 전까지 들어온 좌표는 버림). 좌표가 register_confirm_time 동안 끊김 없이
              (register_gap 안) 들어오면 등록 완료(on_registered: registered=True, "인식되었습니다") → REGISTERED.
              확인하는 동안은 따라가지 않고 그 자리에서 바라본다. 끊기면 처음부터 다시 기다림
              auto_register면 좌표가 안 오는 동안 register_period마다 target_person/register_front를 보내
              인식 모듈이 로봇 바로 앞 사람을 대상으로 등록하게 한다 (첫 요청 + register_settle 전 좌표 = 예전 대상 → 버림)
  REGISTERED  등록 완료 → prepare_tour 성공. start_tour를 그 자리에 서서 기다림 (돌지도 따라가지도 않음, 좌표는 받아 둠).
              ready_timeout 안에 start_tour가 안 오면 관람 끝 (not_started)
  ── 여기부터 = start_tour ──
  FOLLOW  좌표가 들어옴 → 80 cm 거리 유지하며 이동
  SEARCH  lost_timeout 동안 안 들어옴 → 최종 좌표에 도착할 때까지 이동 (막히면 search_stall_time 뒤, 상한 search_move_time)
          → 거기서 2π/search_time 일정 속도로 odom 기준 한 바퀴(360°) 회전 탐색
  WAIT    회전 탐색 끝 → 정지 대기 (상태 노드가 '로봇 쪽으로 와 주세요' 안내)
  LOST    wait_time 지남 → 정지, 액션 끝 (reason lost). 다시 시작하려면 액션을 새로 보냄
  SEARCH·WAIT 중 좌표가 다시 들어오면 FOLLOW
  액션을 취소하면 언제든 정지 → READY

관람객 좌표: 인식 좌표는 튀므로 칼만 필터(등속 모델)로 위치·속도를 매끄럽게 추정하고,
      예측에서 너무 먼 좌표(이상값)는 버린다. 연속으로 버려지면 사람이 정말 옮긴 것으로 보고 다시 시작.
멈춤: 0.85 m 안에서 서고 0.95 m 밖에서 다시 출발 (경계를 둘로 나눠 섰다 갔다 방지), 감속은 linear_decel로 부드럽게.
주행: 장애물로 격자를 만들어 칸마다 목표까지 '돌아가는 거리'를 구하고(BFS),
      (v, w) 후보를 sim_time 동안 굴려 보며 안 부딪히고 끝점의 그 거리가 가장 작은 것을 고른다(DWA).
"""
import collections
import json
import math
import os
import signal
import threading
import time

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from arti_interfaces.action import PrepareTour, StartTour
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped, TwistStamped
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus
import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Int32, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
import yaml

from arti_follow.motion import apply, MotionMixin, PersonKF, yaw_of
from arti_follow.prepare_tour import PrepareMixin
from arti_follow.start_tour import RETURNING, TourMixin, TRACKING

LISTEN = ('WAIT_VISITOR', 'REGISTERED') + TRACKING   # 관람객 좌표를 받는 상태 (그 밖에는 버림)

def load_params(path=''):
    """파라미터 기본값을 config/visitor_follower.json에서 읽는다 (코드에 값을 두지 않음).
    묶음(tour, follow, avoid …)은 보기용이라 풀고, '_'로 시작하는 키(_설명)는 버린다.
    받음: path  JSON 경로 ('' = install/share/arti_follow/config/visitor_follower.json)
    돌려줌: {이름: 기본값} (같은 이름의 ROS 파라미터 -p 이름:=값 이 있으면 그 값이 우선)"""
    path = path or os.path.join(get_package_share_directory('arti_follow'), 'config', 'visitor_follower.json')
    with open(path, encoding='utf-8') as f:
        cfg = json.load(f)
    return {k: v for group, d in cfg.items() if not group.startswith('_') and isinstance(d, dict)
            for k, v in d.items() if not k.startswith('_')}



class Diag:
    """진단 통계 (diag:=true일 때만): 이름별로 (개수, 합, 최대, 최소)를 모았다가 take()로 꺼내고 비운다."""

    def __init__(self):
        """빈 통계로 시작. 모으기 시작한 시각을 기록한다."""
        self.s, self.start = {}, time.monotonic()

    def add(self, key, value=1.0):
        """통계에 값 하나 더하기.
        받음: key    이름 (예: 'scan.gap')
              value  값 (기본 1 = 개수 세기)"""
        c = self.s.setdefault(key, [0, 0.0, -math.inf, math.inf])
        c[0], c[1], c[2], c[3] = c[0] + 1, c[1] + value, max(c[2], value), min(c[3], value)

    def take(self):
        """모은 통계를 꺼내고 비운다 (report가 diag_period마다 부름).
        돌려줌: (통계 dict {이름: [개수, 합, 최대, 최소]}, 모은 시간 [s])"""
        s, span = self.s, time.monotonic() - self.start
        self.s, self.start = {}, time.monotonic()
        return s, span


class VisitorFollower(PrepareMixin, TourMixin, MotionMixin, Node):

    """관람객 추종 노드: 키오스크 액션(start_tour) 응대 + 상태 기계(step) + 주행(dwa)."""
    def __init__(self):
        """노드 준비: 파라미터·설정 파일 읽기, 구독·발행·액션 서버·타이머 등록. 상태 READY.
        구독  tf, tf_static, target_person/point_map, scan,
              dock_status, artwork_zone/explaining·explained
        발행  cmd_vel, follow/state, initialpose, guide_message, arti/tour_end
        액션  서버 arti/prepare_tour·arti/start_tour, 클라이언트 undock
        타이머 control_period(0.1 s)마다 control()"""
        super().__init__('visitor_follower')
        params_json = self.declare_parameter('params_json', '').value   # '' = 패키지의 config/visitor_follower.json
        self.p = {q.name: q.value for q in self.declare_parameters('', list(load_params(params_json).items()))}
        self.tf = Buffer()
        self.seen = -1e9                            # 관람객 좌표를 마지막으로 받은 시각
        self.kf = PersonKF(self.p['kf_accel_std'], self.p['kf_meas_std'], self.p['kf_gate'], self.p['kf_max_rejects'])
        self.lost_at, self.turn, self.spin_at, self.spin_speed, self.holding = 0.0, 1.0, None, 0.0, False
        self.wait_at, self.spun, self.spin_yaw = None, 0.0, 0.0     # SEARCH: 회전 끝난 시각, 돈 각도, 지난 yaw
        self.best_dist, self.progress_at = None, 0.0                # SEARCH 이동: 가장 짧았던 경로 거리, 그때 시각
        self.path_dist = None                                       # dwa가 구한 로봇 → 목표 경로 거리 [m]
        self.scan_pts, self.scan_time = np.zeros((0, 2)), -1e9
        self.scan_rx = -1e9                         # scan을 받은 시각 (TF 변환 실패로 버려도)
        self.sensor_stop_at = self.sensor_log_at = None   # 센서 때문에 정지 시작한 시각, 마지막 경고 로그 시각
        self.state, self.enabled, self.cmd = None, self.p['start_enabled'], (0.0, 0.0)
        self.cmd_hist = collections.deque(maxlen=60)  # 최근 cmd_vel (시각, v, w) 약 6 s → odom 지연 보정 (predict_yaw)
        self.dbg = {}                               # 이번 주기의 판단 과정 (diag)
        self.diag = Diag() if self.p['diag'] else None
        self.arrive, self.last_tick, self.last_cmd = {}, None, None
        self.gap_avg, self.last_reason = {}, None    # 토픽별 평소 수신 간격, 지난 주기 이유 (diag_events)
        if self.diag is not None:
            self.get_logger().info('파라미터 ' + ', '.join(f'{k}={v}' for k, v in self.p.items()))

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        def sub(kind, topic, fn, qos):               # 진단 켜면 수신 간격·지연·콜백 시간을 잰다
            return self.create_subscription(kind, topic, self.timed(topic, fn), qos)
        # tf는 네임스페이스 안(/robot1/tf)에 있으므로 상대 이름으로 받아 Buffer에 넣는다
        sub(TFMessage, 'tf', lambda m: [self.tf.set_transform(t, 'tf') for t in m.transforms], 100)
        sub(TFMessage, 'tf_static', lambda m: [self.tf.set_transform_static(t, 'tf') for t in m.transforms],
            QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        sub(PointStamped, 'target_person/point_map', self.on_person, 10)
        sub(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        sub(DockStatus, 'dock_status', self.on_dock_status, qos_profile_sensor_data)
        sub(Int32, 'artwork_zone/explaining', self.on_explaining, latched)
        sub(Int32, 'artwork_zone/explained', self.on_explained, 10)
        sub(String, 'artwork_zone/entered', self.on_zone_entered, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'follow/state', latched)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.guide_pub = self.create_publisher(String, 'guide_message', 10)
        self.tour_end_pub = self.create_publisher(Empty, 'arti/tour_end', 10)
        self.register_pub = self.create_publisher(Empty, 'target_person/register_front', 10)
        self.reg_first = self.reg_last = None       # 이번 관람에서 등록 요청을 처음·마지막으로 보낸 시각
        self.start = self.load_start_point(self.p['start_point'])          # AMCL 초기 위치 = 도크 (x, y, yaw) map
        self.face_from = self.load_start_point(self.p['face_from_point'])  # 언독 끝 자리 = 시작 회전 기준
        self.face_point = self.load_face_point()    # 시작 때 바라볼 점 (x, y) map

        # 관람 (prepare_tour → start_tour 액션). 액션 처리는 다른 스레드에서 기다리므로 제어 주기와 겹치는 상태 변경은 lock으로 막는다
        self.lock = threading.Lock()
        self.docked = None                          # dock_status (아직 못 받으면 None)
        self.docked_at = None                       # dock_status를 마지막으로 받은 시각 (undock 실패 로그용)
        self.active, self.end_reason = False, ''    # 관람(준비 포함) 중인지, 끝난 이유 (액션 result)
        self.ready_at = None                        # REGISTERED가 된 시각 (ready_timeout 기준)
        self.explaining, self.visited = 0, []      # 이번 관람: 설명 중 작품 번호(0 = 없음), 설명한 작품
        self.reported = 0                           # visited 중 ARTWORK_COMPLETED로 보고한 수
        self.seen_since = None                      # 등록 확인: 끊김 없이 보이기 시작한 시각
        self.face_until = self.loc_at = self.pose_sent = self.face_goal = self.face_log_at = self.loc_log_at = None
        group = ReentrantCallbackGroup()
        self.undock_client = ActionClient(self, Undock, 'undock', callback_group=group)
        self.dock_client = ActionClient(self, Dock, 'dock', callback_group=group)
        self.home = None                            # 언독이 끝난 곳 (x, y, yaw) odom = 돌아올 곳. 도크는 그 뒤쪽
        self.final_state, self.ret_at, self.dock_future, self.dock_result = 'READY', None, None, None
        self.tour_at = self.end_at = None           # start_tour 시작 시각, 출구 종료 예정 시각
        ActionServer(self, PrepareTour, 'arti/prepare_tour', self.run_prepare, callback_group=group,
                     goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT)
        ActionServer(self, StartTour, 'arti/start_tour', self.run_tour, callback_group=group,
                     goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.set_state('READY')
        if self.p['auto_start']:
            def once():
                timer.cancel()
                if self.run_prepare(None).success:
                    self.run_tour(None)
            timer = self.create_timer(1.0, once, callback_group=group)
        else:
            self.get_logger().info(f'호출 액션 기다림: {self.get_namespace().rstrip("/")}/arti/prepare_tour')

        self.create_timer(self.p['control_period'], self.control)
        if self.diag is not None:
            self.create_timer(self.p['diag_period'], self.report)

    def config_file(self, param, name):
        """설정 파일 경로: 파라미터에 경로가 있으면 그것, 비었으면 arti_common/config/<name>.
        받음: param  파라미터 이름 (예: 'zones_file')
              name   기본 파일 이름 (예: 'artworks.yaml')
        돌려줌: 파일 경로"""
        return self.p[param] or os.path.join(get_package_share_directory('arti_common'), 'config', name)

    def load_start_point(self, key):
        """init_pose.yaml에서 이 로봇의 위치 하나(key: dock / undock)를 읽는다.
        yaw_deg가 없으면 dock → 출발 지점 방향, dock도 없으면 0.
        받음: key, (파라미터) start_pose_file, 노드 네임스페이스
        돌려줌: (x, y, yaw) map 좌표, 못 읽으면 None (초기 위치 안 보냄)"""
        path = self.p['start_pose_file']
        try:
            path = self.config_file('start_pose_file', 'init_pose.yaml')
            with open(path, encoding='utf-8') as f:
                robot = (yaml.safe_load(f) or {}).get('robots', {}).get(self.get_namespace(), {})
            target = robot[key]
        except (OSError, KeyError, PackageNotFoundError, yaml.YAMLError) as e:
            self.get_logger().warn(f'{key} 위치를 못 읽음 ({path}, {self.get_namespace()}: {e!r})')
            return None
        x, y, dock = float(target['x']), float(target['y']), robot.get('dock')
        if 'yaw_deg' in target:
            yaw = math.radians(float(target['yaw_deg']))
        else:
            yaw = math.atan2(y - float(dock['y']), x - float(dock['x'])) if dock else 0.0
        self.get_logger().info(f'{key} 위치 ({x}, {y}, {math.degrees(yaw):.0f}°)')
        return x, y, yaw

    def load_face_point(self):
        """artworks.yaml에서 face_zone(기본 kiosk) 구역의 중심을 읽는다 (START_FACE에서 바라볼 곳).
        받음: (파라미터) face_zone, zones_file
        돌려줌: (x, y) map 좌표, 없으면 None (출발 지점 방향으로 돎)"""
        zid = self.p['face_zone']
        if not zid:
            return None
        path = self.p['zones_file']
        try:
            path = self.config_file('zones_file', 'artworks.yaml')
            with open(path, encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            rows = [r for r in (data.get('guide_zones') or []) + (data.get('artworks') or [])
                    if zid in (r.get('zone_id'), str(r.get('artwork_id')))]   # 작품은 번호 문자열로 ('2')
            r = rows[0]
            if 'x' in r:
                point = (float(r['x']), float(r['y']))
            else:                                   # 직사각형 구역 → 가운데
                point = ((float(r['x_min']) + float(r['x_max'])) / 2, (float(r['y_min']) + float(r['y_max'])) / 2)
        except (OSError, KeyError, IndexError, PackageNotFoundError, yaml.YAMLError, ValueError) as e:
            self.get_logger().warn(f'바라볼 구역 {zid}을 못 읽음 ({path}: {e!r}) → 출발 지점 방향으로 돎')
            return None
        self.get_logger().info(f'시작 때 바라볼 곳: {zid} {point}')
        return point

    # ---------- 입력 ----------

    def now(self):
        """지금 시각 (ROS 시계).
        돌려줌: 초 [s] (float)"""
        return self.get_clock().now().nanoseconds / 1e9

    def lookup(self, target, source, stamp=None):
        """TF에서 source 좌표계가 target 좌표계의 어디에 있는지 꺼낸다 (쓰기 편하게 2D만).
        받음: target  기준 좌표계 (예: 'odom')
              source  찾을 좌표계 (예: 'base_link')
              stamp   (선택) 이 시각의 값. 없으면 최신 값으로 다시 시도
        돌려줌: (x, y, yaw), 변환이 없으면 None (예외 대신)"""
        # odom 하고 base_link 시각을 받아서 x, y , vaw 만 뽑아서 
        for t in ([stamp] if stamp is not None else []) + [Time()]:
            try:
                tr = self.tf.lookup_transform(target, source, t).transform
                if t is not stamp and stamp is not None and self.diag is not None:
                    self.diag.add('tf_fallback:' + source)      # 그 시각 TF가 없어 최신 값으로 대신함
                return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)
            except Exception:
                pass
        if self.diag is not None:
            self.diag.add('tf_fail:' + source)
        return None

    def on_person(self, msg):
        """[구독] target_person/point_map (PointStamped, map) — 관람객 좌표가 올 때마다.
        받아도 되는 상태(WAIT_VISITOR, REGISTERED, FOLLOW, SEARCH, WAIT)일 때만: map → odom 변환 → 칼만 필터.
        받음: msg.header.frame_id, msg.header.stamp (그 시각 TF로 변환), msg.point.x, y
        바뀜: kf.x (위치·속도), seen (마지막으로 본 시각), seen_since (등록 확인: 계속 보이기 시작한 시각)
        로봇을 움직이거나 state를 바꾸지는 않는다 (그건 step)."""
        if self.state not in LISTEN:
            return                                  # 관람 전·undock·초기 위치·출발 방향 보기 중 좌표는 버림
        if self.state == 'WAIT_VISITOR' and self.p['auto_register']:
            stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
            if self.reg_first is None or stamp < self.reg_first + self.p['register_settle']:
                return                              # 등록 요청 전 좌표 = 예전 대상(또는 아무나)일 수 있음
        f = self.lookup(self.p['odom_frame'], msg.header.frame_id, Time.from_msg(msg.header.stamp))
        if f is None:
            return
        q = apply(f, np.array([[msg.point.x, msg.point.y]]))[0]
        now = self.now()
        if self.state == 'WAIT_VISITOR' and (self.seen_since is None or now - self.seen > self.p['register_gap']):
            self.seen_since = now                   # 등록 확인 시작 (처음이거나 끊겼다 다시 보임)
        if self.kf.x is None or now - self.seen >= self.p['lost_timeout']:
            self.kf.reset(q, now)                   # 처음이거나 한동안 못 봄 → 새로 시작
        else:
            self.kf.update(q, now)                  # 이상값이면 버리고 예측만 (사람은 보이는 중)
        self.seen = now

    def person(self):
        """칼만 필터에서 관람객 위치·속도를 꺼낸다. 속도는 max_person_speed(1.5 m/s)로 자름 (튀는 값 막기).
        받음: (kf.x)
        돌려줌: (위치 [x, y], 속도 [vx, vy])  odom, kf.t 시각 기준"""
        pos, vel = self.kf.x[:2].copy(), self.kf.x[2:].copy()
        speed = np.linalg.norm(vel)
        if speed > self.p['max_person_speed']:
            vel *= self.p['max_person_speed'] / speed
        return pos, vel

    def tf_age(self, target, source):
        """TF(target←source) 최신 값이 지금보다 몇 초 전 것인지 (Wi-Fi로 늦게 온 odom 확인용).
        돌려줌: 나이 [s], 변환이 없으면 inf"""
        try:
            stamp = self.tf.lookup_transform(target, source, Time()).header.stamp
        except Exception:
            return float('inf')
        return self.now() - (stamp.sec + stamp.nanosec * 1e-9)

    def on_dock_status(self, msg):
        """[구독] dock_status (DockStatus) — 도크에 붙어 있는지. 값이 바뀌면 로그 (undock 실패 원인 찾기).
        바뀜: docked, docked_at (받은 시각)"""
        if self.docked is not None and msg.is_docked != self.docked:
            self.get_logger().info(f'dock_status 바뀜: is_docked {self.docked} → {msg.is_docked} (상태 {self.state})')
        self.docked, self.docked_at = msg.is_docked, self.now()

    def on_scan(self, msg):
        """[구독] scan (LaserScan) — 라이다가 올 때마다.
        거리 → (x, y) 점, inf·nan·로봇 몸(0.2 m 안)·범위 밖은 버림 → 그 시각 TF로 odom 좌표.
        받음: msg.ranges, angle_min, angle_increment, range_max, header
        바뀜: scan_rx (TF와 상관없이 받은 시각), scan_pts (장애물 점 N×2), scan_time (쓴 시각 → step이 라이다 끊김 확인)"""
        self.scan_rx = self.now()
        f = self.lookup(self.p['odom_frame'], msg.header.frame_id, Time.from_msg(msg.header.stamp))
        if f is None:
            return
        r = np.asarray(msg.ranges)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r > self.p['scan_min_range']) & (r < msg.range_max)
        self.scan_pts = apply(f, np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], 1))
        self.scan_time = self.now()

    def control(self):
        """[타이머] control_period(0.1 s)마다. step()을 감싸는 껍데기:
        lock을 걸어 run_tour(다른 스레드)와 동시에 상태를 바꾸지 않게 하고, (diag) 걸린 시간을 기록한다."""
        t0 = time.monotonic()
        self.dbg = {}
        with self.lock:
            self.step()
        if self.diag is not None:
            self.record(t0)

    def step(self):
        """0.1 s마다 한 번: 지금 상태를 위에서부터 차례로 확인하고, 처음 맞는 곳에서 행동한 뒤 return.
        순서가 중요하다 (안전 확인이 따라가기보다 먼저). if마다 [번호] 주석 = 무엇을 묻고 어디로 가는지.
        받음: (state, kf.x, seen, scan_time, TF odom→base_link)
        단계별 함수: prepare_tour.py(localize·face_start·wait_visitor·registered_wait), start_tour.py(follow·search)
        내보냄: drive() → cmd_vel, set_state() → follow/state, 필요하면 end_tour()"""
        p, now = self.p, self.now()
        # [1] 지금 상태가 READY·UNDOCK·LOST인가? → 예: 아무것도 안 하고 끝 / 아니오: 아래로
        # 관람 전(READY)·관람 끝(LOST)·undock 중이면 손 놓기:
        # undock은 Create 3가 스스로 움직이므로 cmd_vel을 같이 보내면 둘이 부딪힌다. 텔레옵 등 다른 노드와도 안 겹치게
        if self.state in ('READY', 'UNDOCK', 'LOST'):
            self.dbg['reason'] = self.state.lower()
            return
        # [2] 지금 상태가 LOCALIZE인가? → 예: localize() / 아니오: 아래로
        # 초기 위치 알려 주기: initialpose 한 번 보내고 바로 START_FACE (기다리지 않음)
        if self.state == 'LOCALIZE':
            return self.localize(now)
        
        pose = self.lookup(p['odom_frame'], p['base_frame']) #로봇 위치가 나옴
        # [3] 센서 정상인가? (로봇 위치 TF가 있고, 라이다가 0.5 s 안에 왔나) → 아니오: 바로 정지 / 예: 아래로
        # 안전: 내 위치(TF)를 모르거나 라이다가 0.5 s 넘게 끊기면 장애물을 못 보는 상태 → 계산하지 말고 바로 정지
        if pose is None or now - self.scan_time > p['scan_timeout']: # 센서 확인
            self.dbg['reason'] = 'no_pose' if pose is None else 'scan_timeout' # 0.5초 넘게 안바뀌었으면 라이다 끊긴 것
            self.warn_sensor(now, self.dbg['reason'])   # diag와 상관없이 로그 (→ /rosout → bag)
            return self.drive(0.0, 0.0)
        if self.sensor_stop_at is not None:         # 센서가 돌아옴 → 한 번 알림
            self.get_logger().info(f'센서 정상 → 다시 진행 ({now - self.sensor_stop_at:.1f} s 동안 정지했음, 상태 {self.state})')
            self.sensor_stop_at = self.sensor_log_at = None
        here = np.array(pose[:2])

        # [3-1] 관람이 끝나 돌아가는 중인가? (RETURN·DOCK_FACE·DOCK) → 예: go_home() (start_tour.py) / 아니오: 아래로
        # 언독 지점(odom)까지 이동 → 도크 쪽으로 돌기 → Create 3 dock 액션 → READY
        if self.state in RETURNING:
            return self.go_home(now, pose)
        # [3-2] 관람객이 출구에 들어갔나? (end_at) → 예: end_guide_wait 동안 서서 관람객 쪽을 봄 → 관람 끝(completed) → RETURN
        if self.end_at is not None and self.state in TRACKING:
            if now >= self.end_at:
                self.end_at = None
                return self.end_tour('READY', 'completed')
            self.dbg['reason'] = 'end_guide'
            if self.kf.x is not None:
                last, vel = self.person()
                return self.face(pose, np.array(pose[:2]), last)
            return self.drive(*self.ramp(0.0, 0.0))
        # [4] 지금 상태가 START_FACE인가? → 예: face_start() (키오스크 쪽으로 돌기) / 아니오: 아래로
        # 출발 직후: 관람객이 키오스크 쪽에서 오므로 그쪽을 먼저 바라본다 (카메라에 잡히게)
        if self.state == 'START_FACE':
            return self.face_start(now)
        # [5] 지금 상태가 WAIT_VISITOR인가? → 예: wait_visitor() (1 s 확인 → 등록) / 아니오: 아래로
        # 등록 확인 중: 1 s 동안 계속 보여야 등록 (순간 오인식으로 엉뚱한 사람을 따라가지 않게). 확인 중에는 안 따라감
        if self.state == 'WAIT_VISITOR':
            return self.wait_visitor(now, pose, here)
        # [5-1] 지금 상태가 REGISTERED인가? → 예: registered_wait() (start_tour 기다리며 정지) / 아니오: 아래로
        # 등록은 끝났지만 키오스크가 아직 관람 시작(start_tour)을 안 보냄: 사람 때문에 움직이지 않고 서서 기다린다
        if self.state == 'REGISTERED':
            return self.registered_wait(now)
        # [6] 관람객 위치 수신 중인가? (kf.x가 있고 now - seen < 3 s) → 예: FOLLOW / 아니오: 아래로(놓침)
        # 따라가기: 사람 위치를 알고, 마지막으로 본 지 3 s 안이면 FOLLOW.
        # 3 s는 카메라가 잠깐 놓쳐도(가려짐·흔들림) 바로 탐색으로 넘어가지 않게 둔 여유. 그동안은 칼만 예측으로 따라감
        if self.kf.x is not None and now - self.seen < p['lost_timeout']:
            return self.follow(now, pose, here)      # [7]~[9] → start_tour.py follow()
        self.holding = False
        # [10] 사람 정보가 아예 없나? (kf.x == None) → 예: 정지 / 아니오: 아래로(탐색)
        # 사람을 한 번도 못 봤으면 탐색할 기준(마지막 위치)이 없다 → 그냥 정지
        if self.kf.x is None: #kf.x 없음 정지해야함
            self.dbg['reason'] = 'idle'
            return self.drive(*self.ramp(0.0, 0.0))

        # [11]~[14] 사람을 놓침 (3 s 넘게 좌표가 안 옴) → start_tour.py search(): 마지막 위치로 이동 → 회전 탐색 → 대기 → LOST
        return self.search(now, pose, here)

    def warn_sensor(self, now, reason):
        """step [3]에서 센서 때문에 정지할 때 1 s마다 WARN (diag가 꺼져 있어도). 로그는 /rosout으로 나가 bag에도 남는다.
        scan_timeout은 라이다가 아예 안 오는지(scan_rx), 와도 TF 변환을 못 해 버리는지 구분해서 적는다.
        받음: now, reason ('no_pose' / 'scan_timeout'), (scan_rx, scan_time, sensor_log_at)
        바뀜: sensor_stop_at (정지 시작 시각), sensor_log_at (마지막 로그 시각)"""
        if self.sensor_stop_at is None:
            self.sensor_stop_at = now
        if self.sensor_log_at is not None and now - self.sensor_log_at < 1.0:
            return
        self.sensor_log_at = now
        ns = self.get_namespace().rstrip('/')
        def ago(t):
            return f'{now - t:.1f} s 전' if t > 0 else '한 번도 없음'
        if reason == 'no_pose':
            text = (f'로봇 위치 TF({self.p["odom_frame"]}→{self.p["base_frame"]}) 없음 → 정지 '
                    f'(확인: ros2 topic hz {ns}/odom)')
        elif now - self.scan_rx > self.p['scan_timeout']:
            text = f'라이다 scan 안 옴 (마지막 수신 {ago(self.scan_rx)}) → 정지 (확인: ros2 topic info {ns}/scan)'
        else:
            text = (f'라이다 scan은 오지만 TF 변환 실패로 버림 (마지막 사용 {ago(self.scan_time)}) → 정지 '
                    f'(scan frame → {self.p["odom_frame"]} TF 확인)')
        self.get_logger().warn(f'{text}, 상태 {self.state}, {now - self.sensor_stop_at:.1f} s째')

    # ---------- 액션 (호출·등록 = prepare_tour.py, 관람 = start_tour.py) ----------

    def on_goal(self, _goal):
        """[액션] prepare_tour·start_tour goal이 오면 일단 다 받는다.
        여기서 거절하면 키오스크가 이유를 모르므로, 검사는 run_prepare·run_tour에서 하고 이유(message)와 함께 끝낸다.
        돌려줌: GoalResponse.ACCEPT"""
        return GoalResponse.ACCEPT                  # 받을 수 없는 goal은 run_tour에서 이유(message)와 함께 끝냄

    # ---------- 진단 (끊김 원인 찾기) ----------

    def timed(self, name, fn):
        """(diag) 구독 콜백을 감싸서 수신 간격, 지연(받은 시각 - header.stamp), 콜백 시간을 잰다. diag가 꺼져 있으면 그대로.
        받음: name 토픽 이름, fn 원래 콜백
        돌려줌: 감싼 콜백"""
        def cb(msg):
            if self.diag is None:
                return fn(msg)
            t0 = time.monotonic()
            if name in self.arrive:
                gap = t0 - self.arrive[name]
                self.diag.add(name + '.gap', gap)
                avg = self.gap_avg.get(name)
                if self.p['diag_events'] and avg is not None and gap > max(3 * avg, 0.25) and name != 'tf_static':
                    self.get_logger().warn(f'[늦음] {name}: {gap:.2f} s 만에 도착 (평소 {avg:.2f} s)')
                self.gap_avg[name] = gap if avg is None else 0.9 * avg + 0.1 * min(gap, 3 * avg)
            self.arrive[name] = t0
            stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
            if stamp is not None and (stamp.sec or stamp.nanosec):
                self.diag.add(name + '.lat', self.now() - (stamp.sec + stamp.nanosec * 1e-9))
            fn(msg)
            self.diag.add(name + '.cb', time.monotonic() - t0)
        return cb

    def record(self, t0):
        """(diag) 제어 한 주기를 진단 통계에 넣고, diag_events면 끊긴 순간을 바로 로그로.
        받음: t0  control() 시작 시각"""
        d, now, t1 = self.dbg, self.now(), time.monotonic()
        gap = t1 - self.last_tick if self.last_tick is not None else 0.0
        self.last_tick = t1
        step_ms = (t1 - t0) * 1000
        self.diag.add('control.gap', gap)
        self.diag.add('control.ms', step_ms)
        if 'field_ms' in d:
            self.diag.add('field.ms', d['field_ms'])
        reason = d.get('reason', '')
        if d.get('blocked'):
            reason += '+blocked'
        self.diag.add('reason:' + reason)
        if self.p['diag_events']:
            period = self.p['control_period']
            if gap > 1.5 * period:
                self.get_logger().warn(f'[제어 밀림] 주기 {gap:.3f} s (목표 {period}), 이번 계산 {step_ms:.1f} ms')
            if reason != self.last_reason:
                bad = reason.startswith(('no_pose', 'scan_timeout')) or 'blocked' in reason
                def ago(t):
                    return f'{now - t:.2f} s 전' if t > 0 else '아직 없음'
                self.say(bad, f'[이유] {self.last_reason} → {reason}  상태 {self.state}, '
                              f'v={self.cmd[0]:.2f} w={self.cmd[1]:.2f}, scan {ago(self.scan_time)}, '
                              f'좌표 {ago(self.seen)}')
        self.last_reason = reason

    def report(self):
        """(diag) diag_period(2 s)마다 요약 로그: 토픽별 수신 주기·지연, 제어 주기·계산 시간, 정지 이유.
        끊김과 관련 있어 보이는 값은 WARN으로."""
        s, span = self.diag.take()
        log, period = self.get_logger(), self.p['control_period']

        def stat(key, scale=1.0):
            n, total, mx, _ = s.get(key, (0, 0.0, 0.0, 0.0))
            return n, (total / n * scale if n else 0.0), mx * scale
        topics = sorted({k.rsplit('.', 1)[0] for k in s if k.endswith('.cb')})
        for name in topics:
            n, _, _ = stat(name + '.cb')
            _, _, gap = stat(name + '.gap')
            _, lat, lat_max = stat(name + '.lat')
            _, cb, cb_max = stat(name + '.cb', 1000)
            text = (f'[입력] {name}: {n / span:.1f} Hz, 간격 최대 {gap:.2f} s, 지연 평균/최대 {lat:.3f}/{lat_max:.3f} s, '
                    f'콜백 평균/최대 {cb:.1f}/{cb_max:.1f} ms')
            slow = gap > 3 * span / max(n, 1) or lat_max > 0.3 or cb_max > period * 1000 / 2
            self.say(slow, text)
            if name + '.lat' in s:                  # 가장 빨리 온 메시지의 지연 ≈ 두 컴퓨터 시계 차이 + 최소 전송 시간
                lat_min = s[name + '.lat'][3]
                if abs(lat_min) > 0.05:
                    ahead = '로봇 시계가 빠름 (stamp가 미래)' if lat_min < 0 else '로봇 시계가 느리거나 전송이 늦음'
                    self.say(True, f'[시간] {name}: 최소 지연 {lat_min * 1000:.0f} ms → {ahead}. '
                                   f'PC·로봇 시간 동기화(chrony/NTP) 확인')
        expected = ['scan', 'target_person/point_map', 'tf']
        for name in [t for t in expected if t not in topics]:
            log.warn(f'[입력] {name}: {span:.1f} s 동안 한 번도 안 옴')
        n, _, gap = stat('control.gap')
        _, ms, ms_max = stat('control.ms')
        _, fms, fms_max = stat('field.ms')
        text = (f'[제어] {n / span:.1f} Hz (목표 {1 / period:.0f}), 주기 최대 {gap:.3f} s, '
                f'계산 평균/최대 {ms:.1f}/{ms_max:.1f} ms (그중 field {fms:.1f}/{fms_max:.1f} ms)')
        self.say(gap > 1.5 * period or ms_max > 0.8 * period * 1000, text)
        nc, _, cgap = stat('cmd_vel.gap')
        reasons = {k.split(':', 1)[1]: v[0] for k, v in s.items() if k.startswith('reason:')}
        sent = f'cmd_vel 간격 최대 {cgap:.3f} s' if nc else 'cmd_vel 안 보냄 (enabled=false)'
        text = f'[출력] {sent}, 이유별 주기 수 {reasons}'
        bad = any(r.startswith(('no_pose', 'scan_timeout')) or 'blocked' in r for r in reasons)
        self.say(bad or cgap > 1.5 * period, text)
        tf = {k: v[0] for k, v in s.items() if k.startswith('tf_')}
        if tf:
            log.warn(f'[TF] 그 시각 값 없음(fallback)/실패 횟수 {tf}')

    def say(self, warn, text):
        # rclpy는 한 줄(호출 위치)에서 INFO/WARN을 섞어 쓰면 에러를 내서 따로 부른다
        """(diag) 로그 한 줄 출력.
        받음: warn  True면 WARN, 아니면 INFO
              text  문장"""
        if warn:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    def set_state(self, state):
        """상태를 바꾼다. 바뀔 때만 로그를 남기고 follow/state를 발행 (같은 상태로 또 불러도 아무것도 안 함).
        받음: state  'READY' / 'UNDOCK' / 'LOCALIZE' / 'START_FACE' / 'WAIT_VISITOR' / 'REGISTERED' / 'FOLLOW' / 'SEARCH' / 'WAIT' / 'LOST'
        내보냄: follow/state (latched) → artwork_zone 등"""
        if state != self.state:
            self.get_logger().info(f'상태 {self.state} → {state}')
            self.state = state
            self.state_pub.publish(String(data=state))


def main():
    # Ctrl+C 뒤에도 정지 명령을 보낼 수 있게 rclpy의 시그널 처리를 끈다
    """프로그램 시작점: 노드를 만들고 여러 스레드(MultiThreadedExecutor)로 돌린다
    (run_tour가 관람 내내 기다리는 동안에도 step·구독이 돌게). 끝날 때 정지 명령을 보낸다."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = VisitorFollower()
    executor = MultiThreadedExecutor()              # 액션(관람)이 기다리는 동안에도 제어 주기·구독이 돌게
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl+C가 두 번 와도 정지 명령은 끝까지 보냄
    finally:
        node.active = False                         # 준비·관람 액션 대기 루프 끝냄
        executor.shutdown(timeout_sec=1.0)
        node.drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
