"""관람객 추종 주행 — 플로우차트(tracking_pc_ver3) '추적 모듈'의 움직임.

입력  target_person/point_map (PointStamped, frame_id map)  인식 모듈이 준 관람객 좌표
                                  → TF(map→odom)로 odom 좌표로 바꿔 씀 (localization 필요)
      scan (LaserScan)            벽·기둥 등 장애물
      oakd/stereo/image_raw/compressedDepth, oakd/stereo/camera_info   바닥에 떨어진 물체
      tf, tf_static               좌표 변환
      follow/enable (Bool)        false면 cmd_vel을 보내지 않음 (작품 설명 중, 도킹 등)
출력  cmd_vel (TwistStamped), follow/state (String)
진단  diag일 때 diag_period마다 토픽별 수신 주기·지연·콜백 시간, 제어 주기·계산 시간, 정지 이유를 로그로 낸다.
      diag_csv를 주면 제어 주기마다 한 줄씩 CSV로 저장 (launch/follow_test.launch.py)

상태 (플로우차트 순서)
  START_FACE  시작하자마자 제자리 회전으로 출발 지점(arti_common init_pose.yaml의 robots.<ns>.undock) 쪽을 봄
              (로봇이 이미 그 위에 있으면 dock → undock 방향). 다 돌거나 face_timeout 지나면 IDLE.
              그 사이 관람객 좌표가 들어오면 바로 FOLLOW
  IDLE    좌표 기다림
  FOLLOW  좌표가 들어옴 → 80 cm 거리 유지하며 이동
  SEARCH  lost_timeout 동안 안 들어옴 → 최종 좌표로 이동 → 멈춰서 남은 시간 안에 한 바퀴 회전 탐색
  WAIT    search_time 지남 → 정지 대기 (상태 노드가 '로봇 쪽으로 와 주세요' 안내)
  LOST    wait_time 지남 → 정지 (재등록은 인식 모듈·상태 노드)
  좌표가 다시 들어오면 언제든 FOLLOW

관람객 좌표: 인식 좌표는 튀므로 칼만 필터(등속 모델)로 위치·속도를 매끄럽게 추정하고,
      예측에서 너무 먼 좌표(이상값)는 버린다. 연속으로 버려지면 사람이 정말 옮긴 것으로 보고 다시 시작.
멈춤: 0.85 m 안에서 서고 0.95 m 밖에서 다시 출발 (경계를 둘로 나눠 섰다 갔다 방지), 감속은 linear_decel로 부드럽게.
주행: 장애물로 격자를 만들어 칸마다 목표까지 '돌아가는 거리'를 구하고(BFS),
      (v, w) 후보를 sim_time 동안 굴려 보며 안 부딪히고 끝점의 그 거리가 가장 작은 것을 고른다(DWA).
"""
import csv
import math
import os
import signal
import time

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import cv2
from geometry_msgs.msg import PointStamped, TwistStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
import yaml

PARAMS = {
    'odom_frame': 'odom', 'base_frame': 'base_link', 'map_frame': 'map',
    'face_on_start': True,          # 시작하자마자 출발 지점 쪽으로 제자리 회전
    'start_pose_file': '',          # '' = arti_common/config/init_pose.yaml
    'start_point': 'undock',        # init_pose.yaml robots.<네임스페이스>.<이것>을 바라봄
    'face_tolerance': 0.1,          # 이 각도 안이면 다 돈 것 [rad] (약 6°)
    'face_timeout': 15.0,           # 이 시간 안에 못 돌면 포기하고 IDLE [s] (map TF가 없을 때 등)
    'target_distance': 0.8,         # 유지 거리 [m]
    'distance_tolerance': 0.05,     # 이 안이면 전진 안 함 [m]
    'restart_margin': 0.15,         # 멈춘 뒤에는 target_distance + 이만큼 멀어져야 다시 출발 [m] (섰다 갔다 방지)
    'lead_time': 0.5,               # 관람객 속도로 이만큼 앞을 보고 목표를 잡음 [s] (길면 좌표 잡음이 커짐)
    'kf_accel_std': 1.0,            # 칼만: 관람객 가속도 잡음 (클수록 빨리 따라가고 덜 매끄러움) [m/s^2]
    'kf_meas_std': 0.15,            # 칼만: 인식 좌표 잡음 (클수록 더 매끄럽고 느림) [m]
    'kf_gate': 9.21,                # 칼만: 예측과 이만큼(마할라노비스 거리^2, 2차원 99 %) 넘게 다르면 이상값으로 버림
    'kf_max_rejects': 5,            # 연속으로 이만큼 버리면 사람이 정말 옮긴 것으로 보고 필터를 새로 시작
    'max_person_speed': 1.5,        # 추정 속도 상한 [m/s]
    'face_deadband': 0.15,          # 가까이 서 있을 때 이 각도 안이면 돌지 않음 [rad] (약 9°)
    'lost_timeout': 3.0,            # 좌표가 이만큼 안 오면 분실 → SEARCH [s]
    'search_time': 10.0,            # 이동·회전 탐색 시간 (플로우차트 10초) [s]
    'search_move_time': 4.0,        # 그중 최종 좌표로 이동하는 데 쓰는 최대 시간 → 남은 시간은 회전 [s]
    'wait_time': 10.0,              # 그다음 정지 대기 (플로우차트 10초) [s]
    'search_spin': 0.6,             # 회전 탐색 최소 속도. 남은 시간에 한 바퀴가 안 되면 더 빨리 돎 [rad/s]
    'max_linear': 0.3, 'max_angular': 1.5,          # [m/s], [rad/s] (TurtleBot 4 최대 약 0.31 m/s)
    'linear_accel': 0.5, 'angular_accel': 3.0,      # [m/s^2], [rad/s^2]
    'linear_decel': 0.4,            # 멈출 때 감속 [m/s^2] (0.3 m/s → 0.75 s, 약 11 cm 더 감)
    'control_period': 0.1, 'sim_time': 1.5,         # 제어 주기, 후보를 굴려 볼 시간 [s]
    'v_samples': 7, 'w_samples': 15,
    'robot_radius': 0.2, 'safety_margin': 0.05,     # [m]
    'grid_resolution': 0.1,         # 길 찾기 격자 칸 [m]
    'w_heading': 0.3, 'w_clearance': 1.0, 'clearance_zone': 0.4,   # 비용: 방향, 장애물 근접(이 거리 안)
    'w_turn': 0.02,                 # 비용: 회전 (같은 조건이면 직진)
    'scan_min_range': 0.2,          # 이보다 가까운 LiDAR 점은 로봇 몸체(상판 기둥) [m]
    'scan_timeout': 0.5,            # scan이 이보다 오래 안 오면 정지 [s]
    'use_depth': True,              # 바닥 물체 회피 켜기
    'depth_step': 8,                # depth 몇 픽셀마다 한 점
    'depth_max_range': 1.2,         # 이보다 먼 depth는 잡음이 커서 안 씀 (실측: 1 m 너머 ±0.1~0.3 m) [m]
    'floor_min_height': 0.05,       # 바닥 물체 높이 범위 (base_link 기준, 아래는 바닥 잡음) [m]
    'floor_max_height': 0.3,        #   (위는 LiDAR가 봄)
    'floor_min_points': 3,          # 10 cm 칸에 점이 이만큼 모여야 물체 (흩어진 잡음 제거)
    'depth_keep': 1.0,              # depth 장애물을 이 시간 동안 기억 [s]
    'start_enabled': True,          # 시작할 때 cmd_vel을 보낼지 (false면 follow/enable true가 올 때까지 계산만)
    'diag': False,                  # 데이터 흐름 진단 로그 (끊김 원인 찾기)
    'diag_period': 2.0,             # 진단 요약을 내는 주기 [s]
    'diag_csv': '',                 # 제어 주기마다 한 줄씩 저장할 CSV 경로 ('' = 저장 안 함)
    'diag_events': False,           # 끊김이 생긴 순간마다 바로 로그 (토픽 늦음, 제어 밀림, 정지 이유 바뀜)
}


CSV_COLUMNS = ['t', 'tick_gap_s', 'step_ms', 'field_ms', 'state', 'reason', 'enabled', 'v', 'w',
               'scan_age_s', 'person_age_s', 'depth_age_s', 'person_x', 'person_y', 'person_dist']


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
        self.scan_pts, self.scan_time = np.zeros((0, 2)), -1e9
        self.floor_pts, self.floor_time = np.zeros((0, 2)), -1e9
        self.cam_k = self.cam_tf = None
        self.state, self.enabled, self.cmd = None, self.p['start_enabled'], (0.0, 0.0)
        self.dbg = {}                               # 이번 주기의 판단 과정 (diag)
        self.diag = Diag() if self.p['diag'] or self.p['diag_csv'] else None
        self.arrive, self.last_tick, self.last_cmd = {}, None, None
        self.gap_avg, self.last_reason = {}, None    # 토픽별 평소 수신 간격, 지난 주기 이유 (diag_events)
        self.csv_file = self.csv = None
        if self.p['diag_csv']:
            self.csv_file = open(self.p['diag_csv'], 'w', newline='', encoding='utf-8')
            self.csv = csv.writer(self.csv_file)
            self.csv.writerow(CSV_COLUMNS)
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
        sub(CameraInfo, 'oakd/stereo/camera_info',
            lambda m: setattr(self, 'cam_k', (m.k[0], m.k[4], m.k[2], m.k[5])), 10)
        if self.p['use_depth']:
            sub(CompressedImage, 'oakd/stereo/image_raw/compressedDepth', self.on_depth, qos_profile_sensor_data)
        sub(Bool, 'follow/enable', self.on_enable, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'follow/state', latched)
        self.face_target = self.load_start_point() if self.p['face_on_start'] else None
        self.face_until = None                      # START_FACE 끝 시각 (첫 제어 주기에 정함)
        self.set_state('START_FACE' if self.face_target is not None else 'IDLE')
        self.create_timer(self.p['control_period'], self.control)
        if self.diag is not None and self.p['diag']:
            self.create_timer(self.p['diag_period'], self.report)

    def load_start_point(self):
        """init_pose.yaml에서 이 로봇의 출발 지점과 도크 (map). 없으면 None (회전 안 함)."""
        path = self.p['start_pose_file']
        try:
            path = path or os.path.join(get_package_share_directory('arti_common'), 'config', 'init_pose.yaml')
            with open(path, encoding='utf-8') as f:
                robot = (yaml.safe_load(f) or {}).get('robots', {}).get(self.get_namespace(), {})
            target = robot[self.p['start_point']]
        except (OSError, KeyError, PackageNotFoundError, yaml.YAMLError) as e:
            self.get_logger().warn(f'출발 지점을 못 읽음 ({path}, {self.get_namespace()}: {e!r}) → 시작 회전 안 함')
            return None
        dock = robot.get('dock')
        self.get_logger().info(f'시작 회전: 출발 지점 {self.p["start_point"]} ({target["x"]}, {target["y"]}) 쪽을 봄')
        return ((float(target['x']), float(target['y'])),
                (float(dock['x']), float(dock['y'])) if dock else None)

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
        f = self.lookup(self.p['odom_frame'], msg.header.frame_id, Time.from_msg(msg.header.stamp))
        if f is None:
            return
        q = apply(f, np.array([[msg.point.x, msg.point.y]]))[0]
        now = self.now()
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

    def on_depth(self, msg):
        """depth → 바닥 물체 점 (base_link 높이 floor_min~floor_max) → odom."""
        p = self.p
        if self.cam_tf is None:
            try:
                tr = self.tf.lookup_transform(p['base_frame'], msg.header.frame_id, Time()).transform
            except Exception:
                return
            x, y, z, w = tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w
            rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
            self.cam_tf = (rot, np.array([tr.translation.x, tr.translation.y, tr.translation.z]))
        f = self.lookup(p['odom_frame'], p['base_frame'], Time.from_msg(msg.header.stamp))
        # compressedDepth = 12바이트 헤더 + PNG(16비트, mm)
        depth = cv2.imdecode(np.frombuffer(bytes(msg.data[12:]), np.uint8), cv2.IMREAD_UNCHANGED)
        if self.cam_k is None or f is None or depth is None:
            return
        s, (fx, fy, cx, cy) = p['depth_step'], self.cam_k
        z = depth[::s, ::s] / 1000.0
        v, u = np.mgrid[0:depth.shape[0]:s, 0:depth.shape[1]:s]
        ok = (z > 0.2) & (z < p['depth_max_range'])
        cam = np.stack([(u[ok] - cx) * z[ok] / fx, (v[ok] - cy) * z[ok] / fy, z[ok]], 1)
        base = cam @ self.cam_tf[0].T + self.cam_tf[1]
        base = base[(base[:, 2] > p['floor_min_height']) & (base[:, 2] < p['floor_max_height'])]
        if len(base):                               # 10 cm 칸에 floor_min_points개 이상 모인 점만
            _, inv, cnt = np.unique(np.floor(base[:, :2] / 0.1).astype(int), axis=0,
                                    return_inverse=True, return_counts=True)
            base = base[cnt[inv.ravel()] >= p['floor_min_points']]
        self.floor_pts, self.floor_time = apply(f, base[:, :2]), self.now()

    def on_enable(self, msg):
        if not msg.data:
            self.drive(0.0, 0.0)                    # 멈춘 뒤 cmd_vel을 넘겨줌
        self.enabled = msg.data

    # ---------- 상태 (플로우차트) ----------

    def control(self):
        t0 = time.monotonic()
        self.dbg = {}
        self.step()
        if self.diag is not None:
            self.record(t0)

    def step(self):
        p, now = self.p, self.now()
        pose = self.lookup(p['odom_frame'], p['base_frame'])
        if pose is None or now - self.scan_time > p['scan_timeout']:
            self.dbg['reason'] = 'no_pose' if pose is None else 'scan_timeout'
            return self.drive(0.0, 0.0)             # 센서 없음 → 정지
        here = np.array(pose[:2])
        self.dbg['here'] = here

        if self.state == 'START_FACE' and self.kf.x is None:
            return self.face_start(now)
        if self.kf.x is not None and now - self.seen < p['lost_timeout']:
            self.set_state('FOLLOW')                # 80 cm 유지하며 이동
            last, vel = self.person()
            person = last + vel * (now - self.kf.t)     # 필터 위치를 지금 시각으로
            ahead = person + vel * p['lead_time']
            self.dbg['person'] = person
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
            turn = wrap(math.atan2(*(person - here)[::-1]) - pose[2])        # 충분히 가까움 → 감속해 서서 바라보기
            w = float(np.clip(1.5 * turn, -p['max_angular'], p['max_angular']))
            if abs(turn) < p['face_deadband']:
                w = 0.0
            return self.drive(*self.ramp(0.0, w))
        self.holding = False
        if self.kf.x is None:
            self.dbg['reason'] = 'idle'
            return self.drive(*self.ramp(0.0, 0.0))  # IDLE

        last, vel = self.person()
        self.dbg['person'] = last
        if self.state == 'FOLLOW':                  # 방금 놓침 → 마지막으로 가던 방향 기억
            self.lost_at, self.spin_at = now, None
            d = last + vel - here
            self.turn = 1.0 if wrap(math.atan2(d[1], d[0]) - pose[2]) >= 0 else -1.0
        t = now - self.lost_at
        if t < p['search_time']:
            self.set_state('SEARCH')
            if self.spin_at is None and np.linalg.norm(last - here) > 0.5 and t < p['search_move_time']:
                self.dbg['reason'] = 'search_move'
                return self.drive(*self.dwa(pose, last, last))  # 최종 좌표로 이동
            if self.spin_at is None:
                self.spin_at = now                  # 도착(또는 이동 시간 끝) → 남은 시간 안에 한 바퀴 (가속분 5 % 여유)
                left = max(p['search_time'] - t, 1e-3)
                self.spin_speed = float(np.clip(1.05 * 2 * math.pi / left, p['search_spin'], p['max_angular']))
            self.dbg['reason'] = 'search_spin'
            return self.drive(*self.ramp(0.0, self.turn * self.spin_speed))  # 마지막 방향 쪽으로 회전
        self.set_state('WAIT' if t < p['search_time'] + p['wait_time'] else 'LOST')
        self.dbg['reason'] = self.state.lower()
        self.drive(*self.ramp(0.0, 0.0))

    def face_start(self, now):
        """제자리 회전으로 출발 지점 쪽을 본다 (map 좌표, localization 필요)."""
        p = self.p
        if self.face_until is None:
            self.face_until = now + p['face_timeout']
        (tx, ty), dock = self.face_target
        pose = self.lookup(p['map_frame'], p['base_frame'])
        if pose is None or now > self.face_until:
            if now > self.face_until:
                self.get_logger().warn(f'시작 회전 {p["face_timeout"]} s 안에 못 끝냄'
                                       + (' (map TF 없음: localization 확인)' if pose is None else '') + ' → IDLE')
                self.set_state('IDLE')
            self.dbg['reason'] = 'start_face_wait'
            return self.drive(*self.ramp(0.0, 0.0))
        x, y, yaw = pose
        if math.hypot(tx - x, ty - y) < 0.15 and dock is not None:    # 이미 출발 지점 위 → dock에서 나가는 방향
            want = math.atan2(ty - dock[1], tx - dock[0])
        else:
            want = math.atan2(ty - y, tx - x)
        err = float(wrap(want - yaw))
        self.dbg['reason'] = 'start_face'
        if abs(err) < p['face_tolerance'] and abs(self.cmd[1]) < 0.2:
            self.get_logger().info(f'시작 회전 끝 (오차 {math.degrees(err):.0f}°)')
            self.set_state('IDLE')
            return self.drive(*self.ramp(0.0, 0.0))
        w = float(np.clip(1.5 * err, -p['max_angular'], p['max_angular']))
        return self.drive(*self.ramp(0.0, w))

    # ---------- 주행 ----------

    def obstacles(self):
        fresh = self.now() - self.floor_time < self.p['depth_keep']
        return np.concatenate([self.scan_pts] + ([self.floor_pts] if fresh else []))

    def dwa(self, pose, goal, face):
        """(v, w) 후보를 굴려 보고 안 부딪히면서 목표까지 돌아가는 거리가 가장 줄어드는 것."""
        p, (x0, y0, th0), (v0, w0) = self.p, pose, self.cmd
        dt = p['control_period']
        vs = np.linspace(max(0.0, v0 - p['linear_accel'] * dt),
                         min(p['max_linear'], v0 + p['linear_accel'] * dt), p['v_samples'])
        ws = np.linspace(max(-p['max_angular'], w0 - p['angular_accel'] * dt),
                         min(p['max_angular'], w0 + p['angular_accel'] * dt), p['w_samples'])
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
        """제어 한 주기를 통계와 CSV에 남긴다."""
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
        if self.csv is None:
            return
        person = d.get('person')

        def age(t):
            return round(now - t, 3) if t > 0 else ''
        self.csv.writerow([
            round(now, 3), round(gap, 3), round(step_ms, 1), round(d.get('field_ms', 0.0), 1),
            self.state, reason, int(self.enabled), round(self.cmd[0], 3), round(self.cmd[1], 3),
            age(self.scan_time), age(self.seen), age(self.floor_time),
            '' if person is None else round(float(person[0]), 3),
            '' if person is None else round(float(person[1]), 3),
            '' if person is None or 'here' not in d else round(float(np.linalg.norm(person - d['here'])), 3)])
        self.csv_file.flush()

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
        if self.p['use_depth']:
            expected.append('oakd/stereo/image_raw/compressedDepth')
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

    def close(self):
        if self.csv_file is not None:
            self.csv_file.close()

    def set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'상태 {self.state} → {state}')
            self.state = state
            self.state_pub.publish(String(data=state))


def main():
    # Ctrl+C 뒤에도 정지 명령을 보낼 수 있게 rclpy의 시그널 처리를 끈다
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = VisitorFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl+C가 두 번 와도 정지 명령은 끝까지 보냄
    finally:
        node.drive(0.0, 0.0)
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
