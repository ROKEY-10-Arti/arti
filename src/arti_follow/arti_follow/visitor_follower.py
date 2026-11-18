"""관람객 추종 주행 — 플로우차트(tracking_pc_ver3) '추적 모듈'의 움직임.

입력  target_person/point_map (PointStamped, frame_id map)  인식 모듈이 준 관람객 좌표
                                  → TF(map→odom)로 odom 좌표로 바꿔 씀 (localization 필요)
      scan (LaserScan)            벽·기둥 등 장애물 (OAK-D depth는 쓰지 않음, 2026-11-18 제거)
      tf, tf_static               좌표 변환
      follow/enable (Bool)        false면 cmd_vel을 보내지 않음 (작품 설명 중, 도킹 등)
      dock_status (irobot_create_msgs/DockStatus)   도크에 붙어 있는지 (붙어 있을 때만 undock)
      arti/start_tour (arti_interfaces/StartTour, 액션 서버)   관람 시작 (mode FREE만). 취소하면 관람 끝
                                  feedback: state(MOVING/ARRIVED/FOLLOW/EXPLAINING), registered, 설명 중 작품, 설명한 수/전체
                                  result: success, message(끝난 이유), visited_ids(끝까지 설명한 작품)
      artwork_zone/explaining (String)   설명 중인 artwork_id ('' = 없음) → feedback EXPLAINING, current_artwork_id
      artwork_zone/explained (String)    끝까지 설명한 artwork_id → visited_ids
      artwork_zone/entered (String)      관람객이 들어간 안내 구역 zone_id. end_zone(exit)이면 관람 끝 → 복귀·도킹
      dock_status                        도크에 붙었는지, 도크가 보이는지
출력  cmd_vel (TwistStamped), follow/state (String)
      undock (irobot_create_msgs/Undock, 액션 클라이언트)     Create 3 언독
      dock (irobot_create_msgs/Dock, 액션 클라이언트)         Create 3 도킹 (관람 끝, 도크 앞에서)
      initialpose (PoseWithCovarianceStamped)      AMCL 초기 위치 = init_pose.yaml의 출발 지점(undock)
      guide_message (String)                       관람객을 인식하면 recognized_guide 키 → arti_audio가 말함
      arti/tour_end (Empty)                        관람이 끝나면(취소·LOST·실패) 한 번 → 인식 모듈·artwork_zone이
                                                   그 관람객 데이터(re-ID 특징, 좌표 기록 등)를 지운다. 로봇 1대 = 관람객 1명
      target_person/register_front (Empty)         WAIT_VISITOR에서 인식 모듈에 '바로 앞 사람을 등록' 요청
진단  diag일 때 diag_period마다 토픽별 수신 주기·지연·콜백 시간, 제어 주기·계산 시간, 정지 이유를 로그로 낸다.
      (화면 로그만. 테스트 기록은 follow_test.launch.py의 ros2 bag)

상태 (플로우차트 순서). 시작은 READY — arti/start_tour 액션이 와야 움직인다 (auto_start면 켜자마자)
  READY       액션 기다림. cmd_vel을 보내지 않음 (텔레옵 등 다른 노드와 안 겹침)
  UNDOCK      도크에 붙어 있으면 Create 3 undock 액션 (로봇이 스스로 움직임 → cmd_vel 안 보냄). 도크 밖이면 건너뜀
  LOCALIZE    AMCL에 initialpose(출발 지점 x, y, yaw_deg)를 보내고 map 위치가 그 근처로 잡힐 때까지 기다림
  START_FACE  제자리 회전으로 face_zone(기본 키오스크, arti_common artworks.yaml) 중심 쪽을 봄.
              face_zone이 없으면 출발 지점의 yaw_deg. 다 돌거나 face_timeout 지나면 WAIT_VISITOR
  WAIT_VISITOR 관람객 인식 기다림 (이 전까지 들어온 좌표는 버림). 좌표가 register_confirm_time 동안 끊김 없이
              (register_gap 안) 들어오면 등록 완료(on_registered: feedback registered, "인식되었습니다") → FOLLOW.
              확인하는 동안은 따라가지 않고 그 자리에서 바라본다. 끊기면 처음부터 다시 기다림
              auto_register면 WAIT_VISITOR 동안 register_period마다 target_person/register_front를 보내
              인식 모듈이 로봇 바로 앞 사람을 대상으로 등록하게 한다 (첫 요청 + register_settle 전 좌표 = 예전 대상 → 버림)
  FOLLOW  좌표가 들어옴 → 80 cm 거리 유지하며 이동
  SEARCH  lost_timeout 동안 안 들어옴 → 최종 좌표에 도착할 때까지(search_move_time 상한) 이동
          → 거기서 2π/search_time 일정 속도로 odom 기준 한 바퀴(360°) 회전 탐색
  WAIT    회전 탐색 끝 → 정지 대기 (상태 노드가 '로봇 쪽으로 와 주세요' 안내)
  LOST    wait_time 지남 → 정지, 액션 끝 (reason lost). 다시 시작하려면 액션을 새로 보냄
  SEARCH·WAIT 중 좌표가 다시 들어오면 FOLLOW
  RETURN      FOLLOW·SEARCH·WAIT 중 관람객이 end_zone(출구)에 들어감 → 관람객 데이터 삭제(arti/tour_end),
              init_pose.yaml의 return_point(undock 지점, 도크 앞)로 DWA 이동 (return_timeout 넘으면 return_failed)
              출구에서 바로 dock하면 Create 3가 도크를 못 찾으므로 언독 지점으로 먼저 간다
  DOCK_FACE   도착 → 제자리에서 dock_point(도크) 쪽을 봄
  DOCK        Create 3 dock 액션 (cmd_vel 안 보냄). 끝나면 액션 result success=true, message finished → READY
              (dock 실패 → dock_failed, 서버 없음 → no_dock_server)
  액션을 취소하면 언제든 정지 → READY

관람객 좌표: 인식 좌표는 튀므로 칼만 필터(등속 모델)로 위치·속도를 매끄럽게 추정하고,
      예측에서 너무 먼 좌표(이상값)는 버린다. 연속으로 버려지면 사람이 정말 옮긴 것으로 보고 다시 시작.
멈춤: 0.85 m 안에서 서고 0.95 m 밖에서 다시 출발 (경계를 둘로 나눠 섰다 갔다 방지), 감속은 linear_decel로 부드럽게.
주행: 장애물로 격자를 만들어 칸마다 목표까지 '돌아가는 거리'를 구하고(BFS),
      (v, w) 후보를 sim_time 동안 굴려 보며 안 부딪히고 끝점의 그 거리가 가장 작은 것을 고른다(DWA).
"""
import math
import os
import re
import signal
import threading
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from arti_interfaces.action import StartTour
import cv2
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
from std_msgs.msg import Bool, Empty, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
import yaml

PARAMS = {
    'odom_frame': 'odom', 'base_frame': 'base_link', 'map_frame': 'map',
    'auto_start': False,            # true면 액션 없이 켜자마자 관람 시작 (테스트용)
    'undock': True,                 # 관람 시작 때 도크에 붙어 있으면 undock
    'undock_timeout': 60.0,         # undock 액션을 이만큼 기다림 [s]
    'return_on_exit': True,         # 추종 중 관람객이 end_zone(출구)에 들어가면 관람 끝 → return_point로 돌아가 도킹
    'end_zone': 'exit',             # 관람 끝 구역 = artworks.yaml guide_zones의 zone_id (artwork_zone/entered로 알려 옴)
    'return_point': 'undock',       # init_pose.yaml robots.<네임스페이스>.<이것> = 도킹 전에 먼저 갈 도크 앞 지점
                                    #   (Create 3 dock은 도크가 보이는 가까운 곳에서만 찾으므로 출구에서 바로 하면 못 찾음)
    'dock_point': 'dock',           # init_pose.yaml robots.<네임스페이스>.<이것> = 도크 위치 (도킹 전에 이쪽을 봄)
    'return_arrive': 0.15,          # return_point에서 이 거리 안이면 도착 [m]
    'return_timeout': 90.0,         # 이 안에 return_point에 못 가면 관람 끝 (return_failed) [s]
    'dock': True,                   # 도착·회전 뒤 Create 3 dock 액션 (false면 도크 쪽을 보고 끝)
    'dock_timeout': 120.0,          # dock 액션을 이만큼 기다림 [s]
    'set_initial_pose': True,       # undock 뒤 AMCL initialpose를 출발 지점으로 보냄
    'localize_timeout': 10.0,       # map 위치가 출발 지점 근처로 잡히길 이만큼 기다림 [s]
    'localize_tolerance': 0.3,      # initialpose 뒤 map 위치가 출발 지점에서 이 안이면 잡힌 것 [m]
    'recognized_guide': 'recognized',   # 관람객 인식 때 guide_message로 보낼 키 (arti_audio guide_lines.yaml, '' = 말 안 함)
    'auto_register': True,          # WAIT_VISITOR에서 인식 모듈에 바로 앞 사람 등록 요청 (false면 인식 창 클릭으로만)
    'register_period': 1.0,         # 등록될 때까지 요청을 다시 보내는 주기 [s] (앞에 아무도 없으면 계속)
    'register_settle': 0.5,         # 첫 요청 뒤 이 시간 전 좌표는 예전 대상일 수 있어 버림 [s]
    'start_pose_file': '',          # '' = arti_common/config/init_pose.yaml
    'zones_file': '',               # '' = arti_common/config/artworks.yaml (face_zone을 찾는 곳)
    'face_zone': 'kiosk',           # 시작 때 바라볼 곳 = artworks.yaml 구역(zone_id·artwork_id)의 중심 ('' = 출발 지점 yaw_deg)
    'start_point': 'undock',        # init_pose.yaml robots.<네임스페이스>.<이것> = AMCL 초기 위치
    'face_tolerance': 0.1,          # 이 각도 안이면 다 돈 것 [rad] (약 6°)
    'register_confirm_time': 1.0,   # 관람객이 이만큼 끊김 없이 보여야 등록 완료 (순간 오인식 방지) [s]
    'register_gap': 0.5,            # 등록 확인 중 좌표 간격이 이보다 길면 끊긴 것 → 다시 기다림 [s]
    'face_timeout': 15.0,           # 이 시간 안에 못 돌면 포기하고 WAIT_VISITOR [s] (map TF가 없을 때 등)
    'target_distance': 0.8,         # 유지 거리 [m]
    'distance_tolerance': 0.05,     # 이 안이면 전진 안 함 [m]
    'restart_margin': 0.15,         # 멈춘 뒤에는 target_distance + 이만큼 멀어져야 다시 출발 [m] (섰다 갔다 방지)
    'lead_time': 0.3,               # 관람객 속도로 이만큼 앞을 보고 목표를 잡음 [s] (길면 좌표 잡음이 커짐)
                                    #   0.5 → 0.3 (2026-11-18 bag 재계산: 0.5 s 앞 예측 오차 p90 0.74 m → 0.3 s 0.58 m, 멈출 때 덜 다가옴)
    'kf_accel_std': 2.0,            # 칼만: 관람객 가속도 잡음 (클수록 빨리 따라가고 덜 매끄러움) [m/s^2] (1.0 → 2.0: 걷기 시작·멈춤에 빨리 반응)
    'kf_meas_std': 0.2,             # 칼만: 인식 좌표 잡음 (클수록 더 매끄럽고 느림) [m] (0.15 → 0.2: 이상값으로 버리는 좌표 4.7 % → 3.2 %)
    'kf_gate': 9.21,                # 칼만: 예측과 이만큼(마할라노비스 거리^2, 2차원 99 %) 넘게 다르면 이상값으로 버림
    'kf_max_rejects': 5,            # 연속으로 이만큼 버리면 사람이 정말 옮긴 것으로 보고 필터를 새로 시작
    'max_person_speed': 1.5,        # 추정 속도 상한 [m/s]
    'face_deadband': 0.15,          # 가까이 서 있을 때 이 각도 안이면 돌지 않음 [rad] (약 9°)
    'lost_timeout': 1.5,            # 좌표가 이만큼 안 오면 분실 → SEARCH [s] (3.0 → 1.5: 추종 중 정상 끊김은 1.2 s 이하)
    'search_time': 10.0,            # 최종 좌표에 도착한 뒤 한 바퀴 회전 탐색 시간 (플로우차트 10초) → 2π/10 = 0.63 rad/s [s]
    'search_spin_extra': 3.0,       # 한 바퀴(odom 기준 360°)를 다 못 돌아도 search_time + 이만큼 지나면 끝 [s]
    'search_move_time': 5.0,        # 최종 좌표로 이동하는 시간 상한. 0이면 도착할 때까지 [s] (0 → 5: 사람이 그 자리에 서 있으면 영원히 못 도착)
    'search_arrive': 0.35,          # 최종 좌표에서 이 거리 안이면 도착 [m] (0.5 → 0.35: 거의 제자리에서 돌던 문제)
    'wait_time': 10.0,              # 회전이 끝난 뒤 정지 대기 (플로우차트 10초) [s]
    'search_spin': 0.3,             # 회전 탐색 최소 속도 [rad/s] (보통은 2π/search_time)
    'turn_first_angle': 1.0,        # 정해진 지점으로 갈 때(SEARCH 이동·복귀) 목표 방향이 이보다 틀어져 있으면 먼저 제자리 회전 [rad] (약 57°)
                                    #   DWA는 후진이 없어 목표가 뒤쪽이면 모든 후보가 동점 → (0, 0)으로 멈춤 (2026-11-18 시뮬레이션)
    'max_linear': 0.3, 'max_angular': 0.8,          # [m/s], [rad/s] (TurtleBot 4 최대 약 0.31 m/s)
                                    #   max_angular 1.5 → 0.8: 0.6 rad/s 넘게 돌면 탑캠 영상이 흔들려 인식을 놓침 (bag: 놓침 직전 빠른 회전 5/24)
    'linear_accel': 0.8, 'angular_accel': 1.5,      # [m/s^2], [rad/s^2] (0.5 → 0.8: 빨리 0.3 m/s까지, 3.0 → 1.5: 회전이 덜 들쑥날쑥)
    'linear_decel': 0.8,            # 멈출 때 감속 [m/s^2] (0.4 → 0.8: 0.3 m/s에서 약 6 cm만 더 감, 너무 가까이 가는 것 줄임)
    'min_spin_cmd': 0.1,            # 서 있을 때(v≈0) 이보다 작은 회전 명령은 0으로 [rad/s]
                                    #   Create 3는 0 < |w| < 0.05 같은 작은 제자리 회전을 0.25~0.3 rad/s로 돌아 버림 (bag 4개 공통)
    'control_period': 0.1, 'sim_time': 1.5,         # 제어 주기, 후보를 굴려 볼 시간 [s]
    'v_samples': 7, 'w_samples': 15,
    'robot_radius': 0.2, 'safety_margin': 0.05,     # [m]
    'grid_resolution': 0.1,         # 길 찾기 격자 칸 [m]
    'w_heading': 0.3, 'w_clearance': 1.0, 'clearance_zone': 0.4,   # 비용: 방향, 장애물 근접(이 거리 안)
    'w_turn': 0.02,                 # 비용: 회전 (같은 조건이면 직진)
    'scan_min_range': 0.2,          # 이보다 가까운 LiDAR 점은 로봇 몸체(상판 기둥) [m]
    'scan_timeout': 0.5,            # scan이 이보다 오래 안 오면 정지 [s]
    'start_enabled': True,          # 시작할 때 cmd_vel을 보낼지 (false면 follow/enable true가 올 때까지 계산만)
    'diag': False,                  # 데이터 흐름 진단 로그 (끊김 원인 찾기)
    'diag_period': 2.0,             # 진단 요약을 내는 주기 [s]
    'diag_events': False,           # 끊김이 생긴 순간마다 바로 로그 (토픽 늦음, 제어 밀림, 정지 이유 바뀜)
}


def artwork_number(artwork_id):
    """artworks.yaml의 artwork_id → 액션의 int 작품 ID (끝 숫자: art_01 → 1). 숫자가 없으면 0."""
    m = re.search(r'(\d+)$', artwork_id)
    return int(m.group(1)) if m else 0


class Diag:
    """진단 통계: 이름별로 (개수, 합, 최대, 최소)를 모았다가 take()로 꺼내고 비운다."""

    def __init__(self):
        self.s, self.start = {}, time.monotonic()

    def add(self, key, value=1.0):
        c = self.s.setdefault(key, [0, 0.0, -math.inf, math.inf])
        c[0], c[1], c[2], c[3] = c[0] + 1, c[1] + value, max(c[2], value), min(c[3], value)

    def take(self):
        s, span = self.s, time.monotonic() - self.start
        self.s, self.start = {}, time.monotonic()
        return s, span


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def apply(f, pts):
    """(N,2) 점에 2D 변환 f=(x, y, yaw) 적용."""
    c, s = math.cos(f[2]), math.sin(f[2])
    return np.stack([f[0] + c * pts[:, 0] - s * pts[:, 1], f[1] + s * pts[:, 0] + c * pts[:, 1]], 1)


class PersonKF:
    """관람객 위치 칼만 필터 (등속 모델). 상태 x = [px, py, vx, vy] (odom)."""

    def __init__(self, accel_std, meas_std, gate, max_rejects):
        self.q, self.r, self.gate, self.max_rejects = accel_std ** 2, meas_std ** 2, gate, max_rejects
        self.x, self.P, self.t, self.rejects = None, None, None, 0

    def reset(self, z, t):
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        self.P = np.diag([self.r, self.r, 1.0, 1.0])   # 속도는 모름 → 크게
        self.t, self.rejects = t, 0

    def predict(self, t):
        dt = max(t - self.t, 0.0)
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        g = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x, self.P, self.t = F @ self.x, F @ self.P @ F.T + self.q * g @ g.T, t

    def update(self, z, t):
        """좌표 하나 반영 (reset 뒤에만 호출). 이상값으로 버렸으면 False (예측은 반영됨)."""
        self.predict(t)
        y = np.asarray(z, float) - self.x[:2]
        S = self.P[:2, :2] + self.r * np.eye(2)
        if y @ np.linalg.solve(S, y) > self.gate:
            self.rejects += 1
            if self.rejects < self.max_rejects:
                return False
            self.reset(z, t)                            # 계속 다른 곳 → 사람이 정말 옮김
            return True
        K = self.P[:, :2] @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ np.eye(2, 4)) @ self.P    # np.eye(2, 4) = H (상태에서 위치만 꺼냄)
        self.rejects = 0
        return True


class VisitorFollower(Node):

    def __init__(self):
        super().__init__('visitor_follower')
        self.p = {q.name: q.value for q in self.declare_parameters('', list(PARAMS.items()))}
        self.tf = Buffer()
        self.seen = -1e9                            # 관람객 좌표를 마지막으로 받은 시각
        self.kf = PersonKF(self.p['kf_accel_std'], self.p['kf_meas_std'], self.p['kf_gate'], self.p['kf_max_rejects'])
        self.lost_at, self.turn, self.spin_at, self.spin_speed, self.holding = 0.0, 1.0, None, 0.0, False
        self.wait_at, self.spun, self.spin_yaw = None, 0.0, 0.0     # SEARCH: 회전 끝난 시각, 돈 각도, 지난 yaw
        self.scan_pts, self.scan_time = np.zeros((0, 2)), -1e9
        self.state, self.enabled, self.cmd = None, self.p['start_enabled'], (0.0, 0.0)
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
        sub(Bool, 'follow/enable', self.on_enable, 10)
        sub(DockStatus, 'dock_status', self.on_dock_status, qos_profile_sensor_data)
        sub(String, 'artwork_zone/explaining', self.on_explaining, latched)
        sub(String, 'artwork_zone/explained', self.on_explained, 10)
        sub(String, 'artwork_zone/entered', self.on_zone_entered, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'follow/state', latched)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.guide_pub = self.create_publisher(String, 'guide_message', 10)
        self.tour_end_pub = self.create_publisher(Empty, 'arti/tour_end', 10)
        self.register_pub = self.create_publisher(Empty, 'target_person/register_front', 10)
        self.reg_first = self.reg_last = None       # 이번 관람에서 등록 요청을 처음·마지막으로 보낸 시각
        self.start = self.load_start_point()        # 초기 위치 (x, y, yaw) map
        self.face_point = self.load_face_point()    # 시작 때 바라볼 점 (x, y) map
        self.home = self.load_init_point(self.p['return_point'])   # 관람 끝에 돌아갈 도크 앞 (x, y) map
        self.dock_xy = self.load_init_point(self.p['dock_point'])  # 도크 (x, y) map — 도킹 전에 이쪽을 봄
        self.return_at = self.return_reason = None  # 복귀 시작 시각, 복귀 뒤 result message
        self.tour_end_sent = False                  # 이번 관람에서 arti/tour_end를 이미 보냈는지
        self.artwork_total = self.count_artworks()  # feedback total (FREE)

        # 관람 (start_tour 액션). 액션 처리는 다른 스레드에서 기다리므로 제어 주기와 겹치는 상태 변경은 lock으로 막는다
        self.lock = threading.Lock()
        self.docked = None                          # dock_status (아직 못 받으면 None)
        self.dock_visible = None                    # dock_status: 도크 IR이 보이는지 (도킹 전 로그용)
        self.active, self.end_reason = False, ''    # 관람 중인지, 끝난 이유 (액션 result)
        self.registered, self.explaining, self.visited = False, '', []   # 이번 관람: 인식됨, 설명 중 작품, 설명한 작품
        self.seen_since = None                      # 등록 확인: 끊김 없이 보이기 시작한 시각
        self.face_until = self.loc_at = self.pose_sent = None
        group = ReentrantCallbackGroup()
        self.undock_client = ActionClient(self, Undock, 'undock', callback_group=group)
        self.dock_client = ActionClient(self, Dock, 'dock', callback_group=group)
        ActionServer(self, StartTour, 'arti/start_tour', self.run_tour, callback_group=group,
                     goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.set_state('READY')
        if self.p['auto_start']:
            def once():
                timer.cancel()
                self.run_tour(None)
            timer = self.create_timer(1.0, once, callback_group=group)
        else:
            self.get_logger().info(f'관람 시작 액션 기다림: {self.get_namespace().rstrip("/")}/arti/start_tour')

        self.create_timer(self.p['control_period'], self.control)
        if self.diag is not None and self.p['diag']:
            self.create_timer(self.p['diag_period'], self.report)

    def config_file(self, param, name):
        return self.p[param] or os.path.join(get_package_share_directory('arti_common'), 'config', name)

    def load_start_point(self):
        """init_pose.yaml에서 이 로봇의 출발 지점 (x, y, yaw) map. 없으면 None (초기 위치 안 보냄).
        yaw_deg가 없으면 dock → 출발 지점 방향, dock도 없으면 0."""
        path = self.p['start_pose_file']
        try:
            path = self.config_file('start_pose_file', 'init_pose.yaml')
            with open(path, encoding='utf-8') as f:
                robot = (yaml.safe_load(f) or {}).get('robots', {}).get(self.get_namespace(), {})
            target = robot[self.p['start_point']]
        except (OSError, KeyError, PackageNotFoundError, yaml.YAMLError) as e:
            self.get_logger().warn(f'출발 지점을 못 읽음 ({path}, {self.get_namespace()}: {e!r}) → 초기 위치 안 보냄')
            return None
        x, y, dock = float(target['x']), float(target['y']), robot.get('dock')
        if 'yaw_deg' in target:
            yaw = math.radians(float(target['yaw_deg']))
        else:
            yaw = math.atan2(y - float(dock['y']), x - float(dock['x'])) if dock else 0.0
        self.get_logger().info(f'출발 지점 {self.p["start_point"]} ({x}, {y}, {math.degrees(yaw):.0f}°)')
        return x, y, yaw

    def load_init_point(self, key):
        """init_pose.yaml robots.<네임스페이스>.<key>의 (x, y) map. 없으면 None."""
        path = self.p['start_pose_file']
        try:
            path = self.config_file('start_pose_file', 'init_pose.yaml')
            with open(path, encoding='utf-8') as f:
                target = (yaml.safe_load(f) or {}).get('robots', {}).get(self.get_namespace(), {})[key]
            return float(target['x']), float(target['y'])
        except (OSError, KeyError, TypeError, ValueError, PackageNotFoundError, yaml.YAMLError) as e:
            self.get_logger().warn(f'{key} 위치를 못 읽음 ({path}, {self.get_namespace()}: {e!r})')
            return None

    def load_face_point(self):
        """artworks.yaml에서 face_zone 구역의 중심 (x, y) map. 없으면 None (출발 지점 yaw로 돎)."""
        zid = self.p['face_zone']
        if not zid:
            return None
        path = self.p['zones_file']
        try:
            path = self.config_file('zones_file', 'artworks.yaml')
            with open(path, encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
            rows = [r for r in (data.get('guide_zones') or []) + (data.get('artworks') or [])
                    if zid in (r.get('zone_id'), r.get('artwork_id'))]
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

    def count_artworks(self):
        """artworks.yaml의 작품 수 (feedback total). 못 읽으면 0."""
        try:
            with open(self.config_file('zones_file', 'artworks.yaml'), encoding='utf-8') as f:
                return len((yaml.safe_load(f) or {}).get('artworks') or [])
        except (OSError, PackageNotFoundError, yaml.YAMLError) as e:
            self.get_logger().warn(f'작품 수를 못 읽음 ({e!r}) → total 0')
            return 0

    # ---------- 입력 ----------

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def lookup(self, target, source, stamp=None):
        """source 좌표계를 target에서 본 (x, y, yaw). 그 시각 값이 없으면 최신 값."""
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
        if self.state not in ('WAIT_VISITOR', 'FOLLOW', 'SEARCH', 'WAIT'):
            return                                  # 관람 전·undock·초기 위치·출발 방향 보기 중 좌표는 버림
        if self.state == 'WAIT_VISITOR' and self.p['auto_register']:
            stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
            if self.reg_first is None or stamp < self.reg_first + self.p['register_settle']:
                return                              # 등록 요청 전 좌표 = 예전 대상일 수 있음
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
        """관람객 추정 위치(kf.t 시각, odom)와 속도. 속도는 max_person_speed로 자름."""
        pos, vel = self.kf.x[:2].copy(), self.kf.x[2:].copy()
        speed = np.linalg.norm(vel)
        if speed > self.p['max_person_speed']:
            vel *= self.p['max_person_speed'] / speed
        return pos, vel

    def on_scan(self, msg):
        f = self.lookup(self.p['odom_frame'], msg.header.frame_id, Time.from_msg(msg.header.stamp))
        if f is None:
            return
        r = np.asarray(msg.ranges)
        a = msg.angle_min + np.arange(r.size) * msg.angle_increment
        ok = np.isfinite(r) & (r > self.p['scan_min_range']) & (r < msg.range_max)
        self.scan_pts = apply(f, np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], 1))
        self.scan_time = self.now()

    def on_dock_status(self, msg):
        self.docked, self.dock_visible = msg.is_docked, msg.dock_visible

    def on_enable(self, msg):
        if not msg.data:
            self.drive(0.0, 0.0)                    # 멈춘 뒤 cmd_vel을 넘겨줌
        self.enabled = msg.data

    def on_explaining(self, msg):
        self.explaining = msg.data if self.active else ''

    def on_explained(self, msg):
        if self.active and msg.data not in self.visited:
            self.visited.append(msg.data)

    def on_zone_entered(self, msg):
        """artwork_zone이 알려 준 '관람객이 들어간 안내 구역'. end_zone(출구)이면 관람 끝 → 복귀·도킹."""
        p = self.p
        if not p['return_on_exit'] or msg.data != p['end_zone']:
            return
        with self.lock:
            if self.active and self.state in ('FOLLOW', 'SEARCH', 'WAIT'):
                self.begin_return('finished', f'관람객이 종료 구역({msg.data})에 들어감')

    # ---------- 상태 (플로우차트) ----------

    def control(self):
        t0 = time.monotonic()
        self.dbg = {}
        with self.lock:
            self.step()
        if self.diag is not None:
            self.record(t0)

    def step(self):
        p, now = self.p, self.now()
        if self.state in ('READY', 'UNDOCK', 'DOCK', 'LOST'):  # 관람 전·후, undock·dock 중(Create 3가 직접 움직임) → cmd_vel 안 보냄
            self.dbg['reason'] = self.state.lower()
            return
        if self.state == 'LOCALIZE':
            return self.localize(now)
        pose = self.lookup(p['odom_frame'], p['base_frame'])
        if pose is None or now - self.scan_time > p['scan_timeout']:
            self.dbg['reason'] = 'no_pose' if pose is None else 'scan_timeout'
            return self.drive(0.0, 0.0)             # 센서 없음 → 정지
        here = np.array(pose[:2])

        if self.state == 'START_FACE':
            return self.face_start(now)
        if self.state == 'RETURN':
            return self.return_home(now, pose)
        if self.state == 'DOCK_FACE':
            return self.face_dock(now)
        if self.state == 'WAIT_VISITOR':
            return self.wait_visitor(now, pose, here)
        if self.kf.x is not None and now - self.seen < p['lost_timeout']:
            self.set_state('FOLLOW')                # 80 cm 유지하며 이동
            last, vel = self.person()
            person = last + vel * (now - self.kf.t)     # 필터 위치를 지금 시각으로
            ahead = person + vel * p['lead_time']
            to = ahead - here
            d = np.linalg.norm(to)
            stop = p['target_distance'] + p['distance_tolerance']
            if d <= stop:
                self.holding = True
            elif d > p['target_distance'] + p['restart_margin']:
                self.holding = False
            if not self.holding:
                goal = ahead - to / d * p['target_distance']
                self.dbg['reason'] = 'follow'
                return self.drive(*self.dwa(pose, goal, ahead))
            self.dbg['reason'] = 'hold'             # 0.85 m 안 → 서서 바라봄
            return self.face(pose, here, person)
        self.holding = False
        if self.kf.x is None:
            self.dbg['reason'] = 'idle'
            return self.drive(*self.ramp(0.0, 0.0))

        last, vel = self.person()
        if self.state == 'FOLLOW':                  # 방금 놓침 → 마지막으로 가던 방향 기억
            self.lost_at, self.spin_at, self.wait_at = now, None, None
            d = last + vel - here
            self.turn = 1.0 if wrap(math.atan2(d[1], d[0]) - pose[2]) >= 0 else -1.0
        if self.spin_at is None:                    # 1) 최종 좌표에 도착할 때까지 이동
            cap = p['search_move_time']             # 0이면 시간 제한 없음
            if np.linalg.norm(last - here) > p['search_arrive'] and (cap <= 0 or now - self.lost_at < cap):
                self.set_state('SEARCH')
                self.dbg['reason'] = 'search_move'
                return self.go_to(pose, last)
            self.spin_at, self.spun, self.spin_yaw = now, 0.0, pose[2]   # 도착 → 여기서 한 바퀴
            self.spin_speed = float(np.clip(2 * math.pi / max(p['search_time'], 1e-3),
                                            p['search_spin'], p['max_angular']))
        if self.wait_at is None:                    # 2) 도착 지점에서 일정한 속도로 한 바퀴 (odom으로 돈 각도를 잼)
            self.spun += abs(float(wrap(pose[2] - self.spin_yaw)))
            self.spin_yaw = pose[2]
            if self.spun < 2 * math.pi and now - self.spin_at < p['search_time'] + p['search_spin_extra']:
                self.set_state('SEARCH')
                self.dbg['reason'] = 'search_spin'
                return self.drive(*self.ramp(0.0, self.turn * self.spin_speed))  # 마지막 방향 쪽으로 회전
            self.wait_at = now
            self.get_logger().info(f'회전 탐색 끝: {math.degrees(self.spun):.0f}° / {now - self.spin_at:.1f} s → 정지 대기')
        if now - self.wait_at >= p['wait_time']:    # 4) 대기도 지남 → 관람 끝
            return self.end_tour('LOST', 'lost')
        self.set_state('WAIT')                      # 3) 정지 대기
        self.dbg['reason'] = 'wait'
        self.drive(*self.ramp(0.0, 0.0))

    def request_register(self, now):
        """WAIT_VISITOR: 인식 모듈에 바로 앞 사람 등록을 요청 (등록돼 좌표가 올 때까지 register_period마다)."""
        p = self.p
        if self.state != 'WAIT_VISITOR' or not p['auto_register']:
            return
        if self.reg_last is not None and now - self.reg_last < p['register_period']:
            return
        if self.reg_first is None:
            self.reg_first = now
            self.get_logger().info('바로 앞 사람 등록 요청 (target_person/register_front)')
        self.reg_last = now
        self.register_pub.publish(Empty())

    def face(self, pose, here, person):
        """제자리에서 감속해 서서 person 쪽을 바라봄 (face_deadband 안이면 돌지 않음)."""
        turn = wrap(math.atan2(*(person - here)[::-1]) - pose[2])
        w = float(np.clip(1.5 * turn, -self.p['max_angular'], self.p['max_angular']))
        if abs(turn) < self.p['face_deadband']:
            w = 0.0
        return self.drive(*self.ramp(0.0, w))

    def wait_visitor(self, now, pose, here):
        """관람객 인식 기다림 → register_confirm_time 동안 끊김 없이 보이면 등록 완료 → FOLLOW.
        확인하는 동안은 따라가지 않고 바라보기만 한다 (등록 안 된 사람을 따라가지 않게)."""
        p = self.p
        if self.kf.x is None or now - self.seen > p['register_gap']:
            if self.kf.x is not None:
                self.get_logger().info('등록 확인 중 끊김 → 다시 기다림')
            self.kf.x, self.seen_since = None, None
            self.dbg['reason'] = 'wait_visitor'
            self.request_register(now)
            return self.drive(*self.ramp(0.0, 0.0))
        if now - self.seen_since < p['register_confirm_time']:
            self.dbg['reason'] = 'register_confirm'
            last, vel = self.person()
            return self.face(pose, here, last + vel * (now - self.kf.t))
        self.on_registered(now - self.seen_since)
        self.set_state('FOLLOW')                    # 다음 주기부터 추종
        return self.drive(*self.ramp(0.0, 0.0))

    def on_registered(self, confirmed):
        """등록 완료 순간에 할 일 (한곳에). 나중에 인식 모듈이 등록 신호를 주면 부르는 조건만 바꾼다."""
        self.registered = True                      # → start_tour feedback registered (키오스크 보고)
        self.get_logger().info(f'관람객 등록 완료 ({confirmed:.1f} s 끊김 없이 보임)')
        if self.p['recognized_guide']:
            self.guide_pub.publish(String(data=self.p['recognized_guide']))   # "인식되었습니다"

    def face_start(self, now):
        """제자리 회전으로 출발 지점 쪽을 본다 (map 좌표, localization 필요)."""
        p = self.p
        if self.face_point is None and self.start is None:   # 바라볼 곳을 모름 → 돌지 않음
            return self.set_state('WAIT_VISITOR')
        if self.face_until is None:
            self.face_until = now + p['face_timeout']
        pose = self.lookup(p['map_frame'], p['base_frame'])
        if pose is None or now > self.face_until:
            if now > self.face_until:
                self.get_logger().warn(f'시작 회전 {p["face_timeout"]} s 안에 못 끝냄'
                                       + (' (map TF 없음: localization 확인)' if pose is None else '')
                                       + ' → WAIT_VISITOR')
                self.set_state('WAIT_VISITOR')
            self.dbg['reason'] = 'start_face_wait'
            return self.drive(*self.ramp(0.0, 0.0))
        x, y, yaw = pose
        if self.face_point is not None:             # 바라볼 곳(키오스크 등) 쪽
            want = math.atan2(self.face_point[1] - y, self.face_point[0] - x)
        else:                                       # 없으면 출발 지점의 방향
            want = self.start[2]
        err = float(wrap(want - yaw))
        self.dbg['reason'] = 'start_face'
        if abs(err) < p['face_tolerance'] and abs(self.cmd[1]) < 0.2:
            self.get_logger().info(f'시작 회전 끝 (오차 {math.degrees(err):.0f}°) → 관람객 인식 기다림')
            self.set_state('WAIT_VISITOR')
            return self.drive(*self.ramp(0.0, 0.0))
        w = float(np.clip(1.5 * err, -p['max_angular'], p['max_angular']))
        return self.drive(*self.ramp(0.0, w))

    def localize(self, now):
        """AMCL에 initialpose를 보내고 map 위치가 출발 지점 근처로 잡힐 때까지 기다린다 (cmd_vel 안 보냄)."""
        p = self.p
        if self.start is None or not p['set_initial_pose']:
            return self.set_state('START_FACE')     # 초기 위치를 안 보냄 → 지금 AMCL 위치 그대로
        if self.loc_at is None:
            self.loc_at = now
        x, y, yaw = self.start
        if self.pose_sent is None:
            if self.pose_pub.get_subscription_count() == 0:     # AMCL이 아직 initialpose를 구독하지 않음
                self.dbg['reason'] = 'localize_wait_amcl'
                if now - self.loc_at > p['localize_timeout']:
                    self.end_tour('READY', 'no_localization')
                return
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = p['map_frame']
            msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            cov = [0.0] * 36
            cov[0] = cov[7] = 0.05                  # x, y 분산 [m^2]
            cov[35] = 0.05                          # yaw 분산 [rad^2]
            msg.pose.covariance = cov
            self.pose_pub.publish(msg)
            self.pose_sent = now
            self.get_logger().info(f'초기 위치 보냄 (map {x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)')
        self.dbg['reason'] = 'localize'
        pose = self.lookup(p['map_frame'], p['base_frame'])
        near = pose is not None and math.hypot(pose[0] - x, pose[1] - y) < p['localize_tolerance']
        if near and now - self.pose_sent > 1.0:     # AMCL이 새 위치로 map→odom을 낼 시간
            self.get_logger().info('초기 위치 잡힘 → 그 자리에서 바라볼 곳(face_zone) 쪽으로 회전')
            return self.set_state('START_FACE')
        if now - self.loc_at > p['localize_timeout']:
            if pose is None:
                return self.end_tour('READY', 'no_localization')
            self.get_logger().warn(f'map 위치가 출발 지점에서 {math.hypot(pose[0] - x, pose[1] - y):.2f} m '
                                   '떨어져 있음 (AMCL 확인) → 그래도 바라볼 곳 쪽으로 회전')
            self.set_state('START_FACE')

    # ---------- 관람 시작 (start_tour 액션) ----------

    def on_goal(self, _goal):
        return GoalResponse.ACCEPT                  # 받을 수 없는 goal은 run_tour에서 이유(message)와 함께 끝냄

    def goal_problem(self, goal):
        """이 goal로 관람을 시작할 수 없으면 이유, 되면 ''. lock 안에서 부른다."""
        if self.active:
            return 'busy'
        mode = (goal.request.mode if goal is not None else '').strip().upper()
        if mode not in ('', 'FREE'):
            return 'unsupported_mode'
        return ''

    def begin_tour(self):
        self.kf.x, self.seen, self.holding = None, -1e9, False
        self.face_until = self.loc_at = self.pose_sent = None
        self.reg_first = self.reg_last = None
        self.active, self.end_reason = True, ''
        self.return_at = self.return_reason = None
        self.tour_end_sent = False
        self.registered, self.explaining, self.visited = False, '', []
        self.seen_since = None
        self.get_logger().info('관람 시작 (FREE)')
        self.set_state('UNDOCK' if self.p['undock'] else 'LOCALIZE')

    def end_tour(self, state, reason):
        """관람 끝: 정지하고 state로 (READY 또는 LOST). reason은 액션 result.
        이 관람객 데이터(필터 위치·속도)를 지우고 arti/tour_end로 다른 노드도 지우게 한다."""
        self.active, self.end_reason = False, reason
        self.drive(0.0, 0.0)
        self.forget_visitor()
        self.get_logger().info(f'관람 끝 ({reason})')
        self.set_state(state)

    def forget_visitor(self):
        """이 관람객 데이터(필터 위치·속도)를 지우고 arti/tour_end로 다른 노드도 지우게 한다 (관람마다 한 번)."""
        self.kf.x, self.kf.P, self.seen, self.holding = None, None, -1e9, False
        if not self.tour_end_sent:
            self.tour_end_sent = True
            self.tour_end_pub.publish(Empty())
            self.get_logger().info('관람객 데이터 삭제 (arti/tour_end)')

    def begin_return(self, reason, why):
        """관람 끝 → 도크 앞(return_point)으로 복귀 → 도크 쪽 보기 → dock. 액션은 도킹이 끝날 때 reason으로 끝난다.
        관람객 데이터는 바로 지운다 (더는 따라가지 않음)."""
        self.return_reason, self.return_at = reason, self.now()
        self.face_until = None
        self.forget_visitor()
        if self.home is None:
            self.get_logger().warn(f'{why} → 돌아갈 곳({self.p["return_point"]})을 몰라 그 자리에서 끝냄')
            return self.end_tour('READY', reason)
        self.get_logger().info(f'{why} → 도크 앞 {self.p["return_point"]} {self.home}으로 복귀')
        self.set_state('RETURN')

    def run_tour(self, goal):
        """start_tour 액션 실행 (goal None = auto_start). 관람이 끝날 때까지 feedback으로 상태를 보낸다."""
        with self.lock:
            problem = self.goal_problem(goal)
            if not problem:
                self.begin_tour()
        if problem:
            mode = goal.request.mode if goal is not None else ''
            self.get_logger().warn(f'관람 시작 거절 ({problem}, mode "{mode}") — 이 로봇은 FREE만, 한 번에 한 관람')
            if goal is not None:
                goal.abort()
            return StartTour.Result(success=False, message=problem)
        if self.state == 'UNDOCK':
            self.send_feedback(goal)
            why = self.undock(goal)
            with self.lock:
                if why:
                    self.end_tour('READY', why)
                elif self.active:
                    self.set_state('LOCALIZE')
        while self.active:
            if goal is not None and goal.is_cancel_requested:
                with self.lock:
                    self.end_tour('READY', 'canceled')
                break
            self.send_feedback(goal)
            if self.state == 'DOCK':                # 도크 쪽을 다 봄 → Create 3 dock (이 스레드에서 기다림)
                why = self.dock(goal)
                with self.lock:
                    if self.active:
                        self.end_tour('READY', why or self.return_reason)
                continue
            time.sleep(0.2)
        reason = self.end_reason
        result = StartTour.Result(success=reason in ('canceled', 'finished'), message=reason,
                                  visited_ids=[artwork_number(a) for a in self.visited])
        self.visited = []                           # 관람 기록은 result로 넘기고 지운다 (관람객 데이터 남기지 않음)
        self.get_logger().info(f'관람 결과: success={result.success}, message={reason}, '
                               f'visited_ids={list(result.visited_ids)}')
        if goal is not None:
            if reason == 'canceled':
                goal.canceled()
            elif reason == 'finished':
                goal.succeed()
            else:
                goal.abort()
        return result

    def send_feedback(self, goal):
        """내부 상태 → 키오스크용 feedback (StartTour.action 주석 참고)."""
        if goal is None:
            return
        explaining = self.explaining if self.state in ('FOLLOW', 'SEARCH', 'WAIT') else ''
        if explaining:
            state = 'EXPLAINING'
        elif self.state == 'WAIT_VISITOR':
            state = 'ARRIVED'
        elif self.state in ('RETURN', 'DOCK_FACE', 'DOCK'):
            state = 'RETURNING'
        elif self.state in ('FOLLOW', 'SEARCH', 'WAIT', 'LOST'):
            state = 'FOLLOW'
        else:                                       # READY·UNDOCK·LOCALIZE·START_FACE
            state = 'MOVING'
        goal.publish_feedback(StartTour.Feedback(
            state=state, registered=self.registered,
            current_artwork_id=artwork_number(explaining) if explaining else 0,
            current_index=len(self.visited), total=self.artwork_total))

    def wait_future(self, future, timeout, goal):
        """future가 끝날 때까지 기다림 (다른 스레드가 채움). 시간 초과·취소면 False."""
        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > end or (goal is not None and goal.is_cancel_requested):
                return False
            time.sleep(0.1)
        return True

    def undock(self, goal):
        """도크에 붙어 있으면 Create 3 undock. 성공이면 '', 실패면 이유."""
        if self.docked is False:
            self.get_logger().info('이미 도크 밖 → undock 건너뜀')
            return ''
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'undock 액션 서버 없음 ({self.get_namespace().rstrip("/")}/undock)')
            return 'no_undock_server'
        self.get_logger().info('undock 시작')
        sent = self.undock_client.send_goal_async(Undock.Goal())
        if not self.wait_future(sent, 5.0, goal) or not sent.result().accepted:
            return 'canceled' if goal is not None and goal.is_cancel_requested else 'undock_failed'
        handle = sent.result()
        done = handle.get_result_async()
        if not self.wait_future(done, self.p['undock_timeout'], goal):
            handle.cancel_goal_async()
            return 'canceled' if goal is not None and goal.is_cancel_requested else 'undock_failed'
        res = done.result()
        ok = (not res.result.is_docked) if res.status == GoalStatus.STATUS_SUCCEEDED else self.docked is False
        self.get_logger().info('undock 끝' if ok else f'undock 실패 (status {res.status})')
        return '' if ok else 'undock_failed'

    def dock(self, goal):
        """Create 3 dock. 성공이면 '', 실패면 이유. dock:=false면 하지 않고 성공으로 본다."""
        if not self.p['dock']:
            self.get_logger().info('dock:=false → 도킹하지 않고 끝냄')
            return ''
        if self.docked:
            return ''
        if not self.dock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'dock 액션 서버 없음 ({self.get_namespace().rstrip("/")}/dock)')
            return 'no_dock_server'
        self.get_logger().info(f'dock 시작 (도크 보임: {self.dock_visible})')
        sent = self.dock_client.send_goal_async(Dock.Goal())
        if not self.wait_future(sent, 5.0, goal) or not sent.result().accepted:
            return 'canceled' if goal is not None and goal.is_cancel_requested else 'dock_failed'
        handle = sent.result()
        done = handle.get_result_async()
        if not self.wait_future(done, self.p['dock_timeout'], goal):
            handle.cancel_goal_async()
            return 'canceled' if goal is not None and goal.is_cancel_requested else 'dock_failed'
        res = done.result()
        ok = res.result.is_docked if res.status == GoalStatus.STATUS_SUCCEEDED else bool(self.docked)
        self.get_logger().info('dock 끝 (충전 위치)' if ok else f'dock 실패 (status {res.status})')
        return '' if ok else 'dock_failed'

    # ---------- 복귀 (관람 끝 → 도크 앞 → 도킹) ----------

    def return_home(self, now, pose):
        """RETURN: return_point(map)를 odom으로 바꿔 추종과 같은 DWA로 이동. 도착하면 DOCK_FACE."""
        p = self.p
        if now - self.return_at > p['return_timeout']:
            self.get_logger().warn(f'{p["return_timeout"]} s 안에 {p["return_point"]}에 못 감 → 그 자리에서 끝냄')
            return self.end_tour('READY', 'return_failed')
        f = self.lookup(p['odom_frame'], p['map_frame'])
        if f is None:                               # map→odom 없음 (localization 확인)
            self.dbg['reason'] = 'return_no_map'
            return self.drive(*self.ramp(0.0, 0.0))
        goal = apply(f, np.array([self.home]))[0]
        if np.linalg.norm(goal - np.array(pose[:2])) < p['return_arrive']:
            self.get_logger().info(f'도크 앞 도착 ({now - self.return_at:.1f} s) → 도크 쪽으로 회전')
            self.face_until = None
            self.set_state('DOCK_FACE')
            return self.drive(*self.ramp(0.0, 0.0))
        self.dbg['reason'] = 'return'
        return self.go_to(pose, goal)

    def go_to(self, pose, goal):
        """정해진 지점(odom)으로 이동: 목표 방향이 turn_first_angle보다 틀어져 있으면 제자리 회전, 아니면 DWA."""
        err = float(wrap(math.atan2(goal[1] - pose[1], goal[0] - pose[0]) - pose[2]))
        if abs(err) > self.p['turn_first_angle']:
            self.dbg['turn_first'] = True
            w = float(np.clip(1.5 * err, -self.p['max_angular'], self.p['max_angular']))
            return self.drive(*self.ramp(0.0, w))
        return self.drive(*self.dwa(pose, goal, goal))

    def face_dock(self, now):
        """DOCK_FACE: 제자리에서 도크(dock_point) 쪽을 본다. 도크를 모르면 return_point 방향의 반대(언독 반대).
        다 돌거나 face_timeout이 지나면 DOCK (Create 3 dock 액션은 run_tour 스레드가 보냄)."""
        p = self.p
        if self.face_until is None:
            self.face_until = now + p['face_timeout']
        pose = self.lookup(p['map_frame'], p['base_frame'])
        if pose is not None:
            x, y, yaw = pose
            if self.dock_xy is not None:
                want = math.atan2(self.dock_xy[1] - y, self.dock_xy[0] - x)
            elif self.start is not None:
                want = self.start[2] + math.pi
            else:
                want = yaw
            err = float(wrap(want - yaw))
            if not (abs(err) < p['face_tolerance'] and abs(self.cmd[1]) < 0.2) and now < self.face_until:
                self.dbg['reason'] = 'dock_face'
                return self.drive(*self.ramp(0.0, float(np.clip(1.5 * err, -p['max_angular'], p['max_angular']))))
            self.get_logger().info(f'도크 쪽 회전 끝 (오차 {math.degrees(err):.0f}°)')
        elif now < self.face_until:
            self.dbg['reason'] = 'dock_face_wait'
            return self.drive(*self.ramp(0.0, 0.0))
        self.drive(0.0, 0.0)
        self.set_state('DOCK')

    # ---------- 주행 ----------

    def obstacles(self):
        """장애물 점 (odom) = LiDAR만. OAK-D depth 바닥 점은 2026-11-18에 뺐다
        (멀리 있는 관람객을 따라갈 때 빈 바닥이 장애물로 잡혀 목표까지 길이 없다고 보고 멈춤)."""
        return self.scan_pts

    def dwa(self, pose, goal, face):
        """(v, w) 후보를 굴려 보고 안 부딪히면서 목표까지 돌아가는 거리가 가장 줄어드는 것."""
        p, (x0, y0, th0), (v0, w0) = self.p, pose, self.cmd
        dt = p['control_period']
        vs = np.linspace(max(0.0, v0 - p['linear_accel'] * dt),
                         min(p['max_linear'], v0 + p['linear_accel'] * dt), p['v_samples'])
        ws = np.linspace(max(-p['max_angular'], w0 - p['angular_accel'] * dt),
                         min(p['max_angular'], w0 + p['angular_accel'] * dt), p['w_samples'])
        if ws[0] <= 0.0 <= ws[-1]:
            ws = np.union1d(ws, [0.0])              # 정확히 0도 후보로 (없으면 w0 근처 0.01 같은 값이 계속 뽑혀 제자리에서 돎)
        V, W = (g.ravel() for g in np.meshgrid(vs, ws))
        th = th0 + W[:, None] * np.arange(1, int(p['sim_time'] / 0.1) + 1) * 0.1
        xs = x0 + np.cumsum(V[:, None] * np.cos(th) * 0.1, 1)
        ys = y0 + np.cumsum(V[:, None] * np.sin(th) * 0.1, 1)

        obs = self.obstacles()
        near = obs[np.hypot(obs[:, 0] - x0, obs[:, 1] - y0) < p['max_linear'] * p['sim_time'] + 0.6]
        clear, here_clear = np.full(V.size, np.inf), np.inf
        if len(near):
            clear = np.hypot(xs[..., None] - near[:, 0], ys[..., None] - near[:, 1]).min((1, 2))
            here_clear = np.hypot(near[:, 0] - x0, near[:, 1] - y0).min()
        # 여유를 지키는 경로, 또는 이미 붙어 있을 때 더 가까워지지 않는 경로(빠져나오기). 로봇 반지름 안은 금지
        r = p['robot_radius']
        safe = (clear > r + p['safety_margin']) | ((clear > r) & (clear >= here_clear - 0.005))
        if not safe.any():                          # 막힘 → 제자리에서 대상 쪽으로
            self.dbg['blocked'] = True
            turn = wrap(math.atan2(face[1] - y0, face[0] - x0) - th0)
            return 0.0, float(np.clip(1.5 * turn, -p['max_angular'], p['max_angular']))

        t0 = time.monotonic()
        field, origin = self.field(np.asarray(goal, float), np.array([x0, y0]), obs)
        self.dbg['field_ms'] = (time.monotonic() - t0) * 1000
        ex, ey, eth = xs[:, -1], ys[:, -1], th[:, -1]
        cost = self.sample(field, origin, ex, ey) \
            + p['w_clearance'] * np.clip(p['clearance_zone'] - clear, 0, None) \
            + p['w_turn'] * np.abs(W)                       # 비용이 같으면 덜 도는 쪽
        if math.hypot(goal[0] - x0, goal[1] - y0) < 0.3:   # 거의 도착 → 관람객 쪽을 바라봄
            cost += p['w_heading'] * np.abs(wrap(np.arctan2(face[1] - ey, face[0] - ex) - eth))
        cost[~safe] = np.inf
        i = int(np.argmin(cost))
        return float(V[i]), float(W[i])

    def field(self, goal, here, obs):
        """격자 칸마다 goal까지 장애물을 돌아가는 거리 [m]. 장애물(로봇 반지름+여유만큼 부풀림)·못 가는 칸은 50."""
        p = self.p
        res = p['grid_resolution']
        size = max(6.0, np.linalg.norm(goal - here) + 3.0)
        n, origin = int(size / res), (goal + here) / 2 - size / 2
        grid = np.zeros((n, n), np.uint8)
        c = np.floor((obs - origin) / res).astype(int)
        c = c[((c >= 0) & (c < n)).all(1)]
        grid[c[:, 1], c[:, 0]] = 1
        k = int(math.ceil((p['robot_radius'] + p['safety_margin']) / res))
        raw, grid = grid, cv2.dilate(grid, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1)))
        gx, gy = np.floor((goal - origin) / res).astype(int)
        rx, ry = np.floor((here - origin) / res).astype(int)
        grid[max(gy - 1, 0):gy + 2, max(gx - 1, 0):gx + 2] = 0      # 목표 자리는 열어 둔다
        win = np.s_[max(ry - k, 0):ry + k + 1, max(rx - k, 0):rx + k + 1]
        grid[win] = raw[win]                        # 로봇 주변은 실제 장애물 칸만 막음 (붙어 있어도 출발 가능)
        dist = np.full((n, n), np.inf)
        dist[gy, gx] = 0.0
        steps = [(dy, dx, math.hypot(dy, dx)) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx]
        frontier = [(gy, gx)]
        while frontier:                             # 목표에서부터 넓혀 가며 거리 채우기
            nxt = []
            for y, x in frontier:
                for dy, dx, w in steps:
                    yy, xx = y + dy, x + dx
                    if 0 <= yy < n and 0 <= xx < n and not grid[yy, xx] and dist[y, x] + w < dist[yy, xx]:
                        dist[yy, xx] = dist[y, x] + w
                        nxt.append((yy, xx))
            frontier = nxt
        dist *= res
        dist[~np.isfinite(dist)] = 50.0
        return dist, origin

    def sample(self, field, origin, x, y):
        """격자 거리를 칸 사이에서 매끄럽게 (bilinear) 읽는다 → 후보끼리 비용이 같아지지 않음."""
        n = field.shape[0]
        g = (np.stack([x, y], 1) - origin) / self.p['grid_resolution'] - 0.5
        i = np.clip(np.floor(g).astype(int), 0, n - 2)
        t = np.clip(g - i, 0.0, 1.0)
        f = [field[i[:, 1] + dy, i[:, 0] + dx] for dy in (0, 1) for dx in (0, 1)]
        return (f[0] * (1 - t[:, 0]) + f[1] * t[:, 0]) * (1 - t[:, 1]) + (f[2] * (1 - t[:, 0]) + f[3] * t[:, 0]) * t[:, 1]

    def ramp(self, v, w):
        """지난 명령에서 감속(linear_decel)·각가속(angular_accel) 한도 안으로만 바꿈 → 덜컹 안 함."""
        p, (v0, w0) = self.p, self.cmd
        dt = p['control_period']
        dv = p['linear_decel'] if abs(v) < abs(v0) else p['linear_accel']
        v = v0 + float(np.clip(v - v0, -dv * dt, dv * dt))
        w = w0 + float(np.clip(w - w0, -p['angular_accel'] * dt, p['angular_accel'] * dt))
        return v, w

    def drive(self, v, w):
        if abs(v) < 0.01 and abs(w) < self.p['min_spin_cmd']:
            w = 0.0                                 # Create 3는 작은 제자리 회전을 0.25 rad/s 이상으로 돌아 버림
        self.cmd = (v, w)
        if not self.enabled:
            self.cmd = (0.0, 0.0)
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p['base_frame']
        msg.twist.linear.x, msg.twist.angular.z = v, w
        self.cmd_pub.publish(msg)
        if self.diag is not None:
            t = time.monotonic()
            if self.last_cmd is not None:
                self.diag.add('cmd_vel.gap', t - self.last_cmd)
            self.last_cmd = t

    # ---------- 진단 (끊김 원인 찾기) ----------

    def timed(self, name, fn):
        """구독 콜백을 감싸서 수신 간격(gap), 지연(lat = 받은 시각 - header.stamp), 콜백 시간(cb)을 잰다."""
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
        """제어 한 주기를 진단 통계에 넣고, diag_events면 끊긴 순간을 바로 로그로."""
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
        """diag_period마다 요약 로그. 끊김과 관련 있어 보이는 값은 WARN으로."""
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
        if warn:
            self.get_logger().warn(text)
        else:
            self.get_logger().info(text)

    def set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'상태 {self.state} → {state}')
            self.state = state
            self.state_pub.publish(String(data=state))


def main():
    # Ctrl+C 뒤에도 정지 명령을 보낼 수 있게 rclpy의 시그널 처리를 끈다
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = VisitorFollower()
    executor = MultiThreadedExecutor()              # 액션(관람)이 기다리는 동안에도 제어 주기·구독이 돌게
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl+C가 두 번 와도 정지 명령은 끝까지 보냄
    finally:
        node.active = False                         # 관람 액션 대기 루프 끝냄
        executor.shutdown(timeout_sec=1.0)
        node.drive(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
