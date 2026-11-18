"""관람객 추종 노드 (Nav2 버전) — visitor_follower를 그대로 바꿔 끼우는 노드.

판단(누구를, 어디로)은 이 노드가, 주행(어떻게 가나)은 Nav2가 한다. 이 노드는 cmd_vel을 직접 보내지 않는다.
설정 값: config/nav2_follow.json (ROS 파라미터로 덮어쓸 수 있음). 주행 값: arti_bringup/config/nav2_follow.yaml
설계: ~/Desktop/nav2_follow_설계.docx, 시험: docs/follow/nav2_follow_test_plan.md

──────────────────────────────── 실행 흐름 ────────────────────────────────
[시작]   main() → Nav2Follow.__init__(): JSON 읽기 → 구독·발행·액션 등록 → 0.1 s 타이머 → 상태 READY
[스레드 4종이 동시에 돈다 (MultiThreadedExecutor)]
 ① 구독 콜백   on_person(좌표 → 칼만 필터), on_enable, on_dock_status, on_explaining·on_explained, on_zone_entered(출구 → begin_return)
 ② 관람 스레드  run_tour(): 키오스크 액션 하나를 끝까지 맡음
                 검사 → Nav2 서버 확인 → [UNDOCK] Create 3 undock → 0.2 s마다 feedback·취소 확인 → [DOCK] Create 3 dock → result
 ③ 타이머      control() → step(): 0.1 s마다 지금 상태를 보고 할 일 하나를 고름 (3절 상황판단)
                 LOCALIZE → localize()        initialpose 보내고 위치 확인
                 START_FACE → face_start()    Nav2: 같은 자리 + 키오스크 쪽 방향 (기본 BT)
                 WAIT_VISITOR → wait_visitor() 등록 확인, 확인 중에는 track(chase=False)로 사람 쪽 보기
                 RETURN → return_home()       Nav2: 도크 앞 + 도크 쪽 방향 (기본 BT)
                 사람 보임 → follow() → track() → send_follow()   (4절 추종주행: follow BT + goal_update)
                 사람 안 보임 → lost()        (5절 이탈처리: search BT → WAIT → LOST)
 ④ Nav2 콜백   Nav2Goal.on_accept / on_result: Nav2 목표의 결과를 status에 적기만 함 → 다음 step()이 보고 판단
[Nav2로 가는 길]  상태 함수 → Nav2Goal.send(pose, bt) / update(pose) / cancel() → navigate_to_pose 액션 / goal_update 토픽
[끝]     main()의 finally: Nav2 목표 취소(로봇 정지) → 노드 종료

상태 (visitor_follower와 같음, DOCK_FACE만 없음: RETURN 목표 방향을 도크 쪽으로 줌)
  READY → UNDOCK → LOCALIZE → START_FACE → WAIT_VISITOR → FOLLOW ⇄ SEARCH → WAIT → LOST
  FOLLOW·SEARCH·WAIT 중 출구 진입 → RETURN → DOCK → READY

토픽·액션 (<ns> 아래, visitor_follower와 같음)
  입력  target_person/point_map, tf, tf_static, follow/enable, dock_status, artwork_zone/explaining·explained·entered
        arti/start_tour (액션 서버)
  출력  follow/state (latched), initialpose, guide_message, arti/tour_end, target_person/register_front
  Nav2  navigate_to_pose (액션 클라이언트), goal_update (follow BT의 GoalUpdater가 받음)
  도크  undock, dock (Create 3 액션 클라이언트)
"""
import json
import math
import os
import re
import signal
import threading
import time

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from arti_interfaces.action import StartTour
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped
from irobot_create_msgs.action import Dock, Undock
from irobot_create_msgs.msg import DockStatus
from nav2_msgs.action import NavigateToPose
import numpy as np
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool, Empty, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
import yaml


# ============================================================ 0. 설정 읽기 · 작은 계산
def load_config(path):
    """JSON 설정 → 평평한 dict {이름: 값}. 묶음("follow" 등)은 보기용이라 풀고, '_'로 시작하는 설명은 버린다.
    [부름] Nav2Follow.__init__ 맨 처음 → 결과가 ROS 파라미터의 기본값이 됨"""
    with open(path, encoding='utf-8') as f:
        data = json.load(f)
    flat = {}
    for group in data.values():
        if isinstance(group, dict):
            flat.update({k: v for k, v in group.items() if not k.startswith('_')})
    return flat


def wrap(a):
    """각도를 -π ~ π로 (두 방향의 차이를 짧은 쪽으로 비교)."""
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_of(q):
    """쿼터니언 → yaw [rad]."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def artwork_number(artwork_id):
    """"art_02" → 2 (액션 feedback·result에서 쓰는 작품 ID, 숫자가 없으면 0)."""
    m = re.search(r'(\d+)$', artwork_id)
    return int(m.group(1)) if m else 0


# ============================================================ 1. 관람객 위치 (칼만 필터)
class PersonKF:
    """등속 모델 칼만 필터. 상태 x = [px, py, vx, vy] (map).
    [쓰는 곳] on_person()이 좌표를 넣고(reset/update) → person()이 꺼내 track()·lost()가 씀.
    Nav2는 목표의 잡음을 거르지 않으므로 이 노드에 남는다."""

    def __init__(self, accel_std, meas_std, gate, max_rejects):
        self.q, self.r = accel_std ** 2, meas_std ** 2  # 예측 불확실도(가속도 잡음), 측정 잡음
        self.gate, self.max_rejects = gate, max_rejects
        self.x, self.P, self.t, self.rejects = None, None, None, 0

    def reset(self, z, t):
        """이 좌표로 새로 시작 (속도 0, 속도 불확실도 크게)."""
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        self.P = np.diag([self.r, self.r, 1.0, 1.0])
        self.t, self.rejects = t, 0

    def predict(self, t):
        """지난 위치 + 속도 × 지난 시간 → 지금쯤 위치 (불확실도도 커짐)."""
        dt = max(t - self.t, 0.0)
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt                          # 위치 += 속도 × dt
        g = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x, self.P, self.t = F @ self.x, F @ self.P @ F.T + self.q * g @ g.T, t

    def update(self, z, t):
        """① 예측 → ② 측정과 비교 → 튄 값이면 버림 / 아니면 ③ 섞어서 보정. 돌려줌: 반영 True, 버림 False"""
        self.predict(t)
        y = np.asarray(z, float) - self.x[:2]           # 측정 - 예측
        S = self.P[:2, :2] + self.r * np.eye(2)
        if y @ np.linalg.solve(S, y) > self.gate:       # 예측 불확실도에 비해 너무 멂 → 튄 값
            self.rejects += 1
            if self.rejects < self.max_rejects:
                return False
            self.reset(z, t)                            # 계속 다른 곳 → 사람이 정말 옮김
            return True
        K = self.P[:, :2] @ np.linalg.inv(S)            # 예측과 측정을 섞는 비율
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ np.eye(2, 4)) @ self.P
        self.rejects = 0
        return True


# ============================================================ 2. Nav2 도우미
class Nav2Goal:
    """Nav2에 보낼 목표 하나를 관리한다. 규칙: Nav2 목표는 한 번에 하나.

    [부르는 쪽: ③ 타이머 스레드의 상태 함수]
      send(pose, bt)  새 목표 → navigate_to_pose. 돌고 있는 목표가 있으면 취소가 끝난 뒤 보냄
                      (bt_navigator는 다른 BT 파일로의 덮어쓰기를 거절하기 때문)
      update(pose)    follow BT가 도는 중 목표만 바꿈 → goal_update 토픽
      cancel()        목표를 버리고 Nav2를 멈춤
      set_enabled()   follow/enable: false면 Nav2만 멈추고 목표는 기억, true면 다시 보냄
    [④ Nav2 콜백]   on_accept → on_result가 status를 바꿈 → 다음 step()이 status를 보고 다음 상태로
      status          'idle' / 'running' / 'succeeded' / 'failed'
    ③과 ④가 다른 스레드라 모든 메서드를 lock으로 감싼다."""

    def __init__(self, node, group, enabled, frame):
        self.node, self.enabled, self.frame = node, enabled, frame
        self.client = ActionClient(node, NavigateToPose, 'navigate_to_pose', callback_group=group)
        self.update_pub = node.create_publisher(PoseStamped, 'goal_update', 10)
        self.lock = threading.RLock()
        self.status = 'idle'
        self.want = None            # 노드가 원하는 목표 (pose, bt). pose = (x, y, yaw) map
        self.last_time = -1e9       # 마지막으로 보낸(send·update) 시각 [s]
        self.handle = None          # Nav2에서 실행 중인 목표
        self.sent = None            # handle로 보낸 pose
        self.pending = None         # 앞 목표가 끝나길 기다리는 (goal msg, pose)
        self.sending = False        # send_goal_async 응답 기다리는 중

    def pose_msg(self, pose):
        """(x, y, yaw) → PoseStamped (map)."""
        msg = PoseStamped()
        msg.header.frame_id = self.frame
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = float(pose[0]), float(pose[1])
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        return msg

    def send(self, pose, bt, now):
        """[③] 새 목표 + BT 파일 → dispatch() (꺼져 있으면 기억만)."""
        with self.lock:
            self.want, self.status, self.last_time = (pose, bt), 'running', now
            if self.enabled:
                self.dispatch()

    def update(self, pose, now):
        """[③] 돌고 있는 follow BT의 목표만 바꿈 → goal_update → BT의 GoalUpdater (액션 재시작 없음)."""
        with self.lock:
            self.want, self.last_time = (pose, self.want[1]), now
            if self.enabled and self.handle is not None and self.pending is None:
                self.update_pub.publish(self.pose_msg(pose))

    def cancel(self):
        """[③] 목표를 버림 → Nav2 취소 → 로봇이 섬."""
        with self.lock:
            self.want, self.status, self.pending = None, 'idle', None
            if self.handle is not None:
                self.handle.cancel_goal_async()

    def set_enabled(self, enabled):
        """[① on_enable] false: Nav2만 취소(목표·status는 기억) / true: 기억한 목표를 다시 dispatch()."""
        with self.lock:
            if enabled == self.enabled:
                return
            self.enabled = enabled
            if not enabled:
                self.pending = None
                if self.handle is not None:
                    self.handle.cancel_goal_async()
            elif self.want is not None and self.status == 'running':
                self.dispatch()

    # ---- 안쪽 흐름: dispatch → (취소 끝나길 기다림) → flush → on_accept → on_result ----
    def dispatch(self):
        """want를 pending에 올림. 돌고 있는 목표가 있으면 취소 요청 (끝나면 on_result가 flush), 없으면 바로 flush."""
        pose, bt = self.want
        goal = NavigateToPose.Goal()
        goal.pose, goal.behavior_tree = self.pose_msg(pose), bt   # behavior_tree '' = Nav2 기본 BT
        self.pending = (goal, pose)
        if self.handle is not None:
            self.handle.cancel_goal_async()
        elif not self.sending:
            self.flush()

    def flush(self):
        """pending을 실제로 Nav2에 보냄 → 응답은 on_accept."""
        goal, self.sent = self.pending
        self.pending, self.sending = None, True
        self.client.send_goal_async(goal).add_done_callback(self.on_accept)

    def on_accept(self, future):
        """[④] Nav2가 목표를 받았는지. 받았으면 결과를 on_result로 기다림."""
        with self.lock:
            self.sending = False
            handle = future.result()
            if not handle.accepted:
                if self.pending is not None:
                    self.flush()                        # 그 사이 새 목표가 옴 → 그걸 보냄
                elif self.status == 'running' and self.enabled:
                    self.status = 'failed'
                    self.node.get_logger().warn('Nav2가 목표를 거절함')
                return
            self.handle = handle
            handle.get_result_async().add_done_callback(lambda f, h=handle: self.on_result(h, f))
            if self.pending is not None or self.want is None or not self.enabled:
                handle.cancel_goal_async()              # 보내는 동안 새 목표·취소·멈춤이 옴 → 이건 취소
            elif self.want[0] != self.sent:
                self.update_pub.publish(self.pose_msg(self.want[0]))   # 보내는 동안 바뀐 목표 반영

    def on_result(self, handle, future):
        """[④] 목표가 끝남 → 기다리던 목표가 있으면 flush, 없으면 결과를 status에 (step()이 읽음)."""
        with self.lock:
            if handle is not self.handle:
                return
            self.handle = None
            if self.pending is not None:
                return self.flush()                     # 취소가 끝남 → 다음 목표
            if self.status == 'running' and self.enabled and self.want is not None:
                ok = future.result().status == GoalStatus.STATUS_SUCCEEDED
                self.status = 'succeeded' if ok else 'failed'


# ============================================================ 노드
class Nav2Follow(Node):

    def __init__(self):
        """[시작] ① JSON 설정 → ROS 파라미터 ② 구독·발행·액션 ③ 위치·구역 파일 ④ 액션 서버·타이머 → READY."""
        super().__init__('nav2_follow')
        # ① 설정: JSON 값을 기본값으로 ROS 파라미터 선언 → launch·-p로 준 값이 있으면 그것이 우선
        path = self.declare_parameter('config_file', '').value or os.path.join(
            get_package_share_directory('arti_follow'), 'config', 'nav2_follow.json')
        self.p = {q.name: q.value for q in self.declare_parameters('', list(load_config(path).items()))}
        self.get_logger().info(f'설정 {path}')
        p = self.p

        # ② 통신: 구독 콜백은 ① 구독 스레드에서, 액션은 ② 관람 스레드에서 돈다
        self.lock = threading.Lock()                    # ② 관람 스레드와 ③ step()이 동시에 상태를 바꾸지 않게
        group = ReentrantCallbackGroup()                # 액션을 기다리는 동안에도 다른 콜백이 돌게
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tf = Buffer()                              # tf는 네임스페이스 안(/robot1/tf) → 상대 이름으로 받아 넣음
        self.create_subscription(TFMessage, 'tf', lambda m: [self.tf.set_transform(t, 'tf') for t in m.transforms], 100)
        self.create_subscription(TFMessage, 'tf_static',
                                 lambda m: [self.tf.set_transform_static(t, 'tf') for t in m.transforms],
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PointStamped, 'target_person/point_map', self.on_person, 10)
        self.create_subscription(Bool, 'follow/enable', self.on_enable, 10)
        self.create_subscription(DockStatus, 'dock_status', self.on_dock_status, qos_profile_sensor_data)
        self.create_subscription(String, 'artwork_zone/explaining', self.on_explaining, latched)
        self.create_subscription(String, 'artwork_zone/explained', self.on_explained, 10)
        self.create_subscription(String, 'artwork_zone/entered', self.on_zone_entered, 10)
        self.state_pub = self.create_publisher(String, 'follow/state', latched)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.guide_pub = self.create_publisher(String, 'guide_message', 10)
        self.tour_end_pub = self.create_publisher(Empty, 'arti/tour_end', 10)
        self.register_pub = self.create_publisher(Empty, 'target_person/register_front', 10)
        self.undock_client = ActionClient(self, Undock, 'undock', callback_group=group)
        self.dock_client = ActionClient(self, Dock, 'dock', callback_group=group)
        self.nav = Nav2Goal(self, group, p['start_enabled'], p['map_frame'])   # 주행은 전부 이것을 통해 Nav2로

        # ③ 위치·구역 (좌표는 YAML: init_pose.yaml, artworks.yaml), BT 파일 경로
        bt_dir = p['bt_dir'] or os.path.join(get_package_share_directory('arti_bringup'), 'behavior_trees')
        self.follow_bt = os.path.join(bt_dir, p['follow_bt'])
        self.search_bt = {True: os.path.join(bt_dir, p['search_left_bt']), False: os.path.join(bt_dir, p['search_right_bt'])}
        self.start = self.load_point(p['start_point'], with_yaw=True)     # (x, y, yaw) map
        self.home = self.load_point(p['return_point'])                   # (x, y)
        self.dock_xy = self.load_point(p['dock_point'])                  # (x, y)
        self.face_point, self.artwork_total = self.load_zones()

        # ④ 관람 상태 → 액션 서버(② 관람 스레드의 입구) → 타이머(③ step의 입구)
        self.kf = PersonKF(p['kf_accel_std'], p['kf_meas_std'], p['kf_gate'], p['kf_max_rejects'])
        self.state, self.docked = None, None
        self.active, self.end_reason = False, ''
        self.reset_tour()
        ActionServer(self, StartTour, 'arti/start_tour', self.run_tour, callback_group=group,
                     goal_callback=lambda _: GoalResponse.ACCEPT, cancel_callback=lambda _: CancelResponse.ACCEPT)
        self.set_state('READY')
        if p['auto_start']:                             # 테스트: 액션 없이 1 s 뒤 run_tour 한 번
            def once():
                timer.cancel()
                self.run_tour(None)
            timer = self.create_timer(1.0, once, callback_group=group)
        else:
            self.get_logger().info(f'관람 시작 액션 기다림: {self.get_namespace().rstrip("/")}/arti/start_tour')
        self.create_timer(p['control_period'], self.control)

    # ---------- 설정 파일 (__init__ ③에서 한 번) ----------
    def config_path(self, param, name):
        """파라미터 경로, 비었으면 arti_common/config/<name>."""
        return self.p[param] or os.path.join(get_package_share_directory('arti_common'), 'config', name)

    def load_point(self, key, with_yaw=False):
        """init_pose.yaml robots.<네임스페이스>.<key> → (x, y) 또는 (x, y, yaw). 없으면 None.
        yaw: yaw_deg, 없으면 dock → 그 지점 방향, dock도 없으면 0."""
        try:
            with open(self.config_path('start_pose_file', 'init_pose.yaml'), encoding='utf-8') as f:
                robot = (yaml.safe_load(f) or {}).get('robots', {}).get(self.get_namespace(), {})
            x, y = float(robot[key]['x']), float(robot[key]['y'])
        except (OSError, KeyError, TypeError, ValueError, PackageNotFoundError, yaml.YAMLError) as e:
            self.get_logger().warn(f'init_pose.yaml {self.get_namespace()}.{key}를 못 읽음 ({e!r})')
            return None
        if not with_yaw:
            return x, y
        if 'yaw_deg' in robot[key]:
            return x, y, math.radians(float(robot[key]['yaw_deg']))
        dock = robot.get('dock')
        return x, y, (math.atan2(y - float(dock['y']), x - float(dock['x'])) if dock else 0.0)

    def load_zones(self):
        """artworks.yaml → (face_zone 중심 (x, y) 또는 None, 작품 수)."""
        try:
            with open(self.config_path('zones_file', 'artworks.yaml'), encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        except (OSError, PackageNotFoundError, yaml.YAMLError) as e:
            self.get_logger().warn(f'artworks.yaml을 못 읽음 ({e!r})')
            return None, 0
        zone = self.p['face_zone']
        rows = [r for r in (data.get('guide_zones') or []) + (data.get('artworks') or [])
                if zone and zone in (r.get('zone_id'), r.get('artwork_id'))]
        point = None
        if rows:
            r = rows[0]
            point = ((float(r['x']), float(r['y'])) if 'x' in r else       # 원·반원 구역은 중심, 직사각형은 가운데
                     ((float(r['x_min']) + float(r['x_max'])) / 2, (float(r['y_min']) + float(r['y_max'])) / 2))
        return point, len(data.get('artworks') or [])

    # ---------- 공통 도구 ----------
    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def lookup(self, target, source, stamp=None):
        """TF target ← source를 (x, y, yaw)로. 그 시각이 없으면 최신 값, 그래도 없으면 None."""
        for t in ([stamp] if stamp is not None else []) + [Time()]:
            try:
                tr = self.tf.lookup_transform(target, source, t).transform
                return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)
            except Exception:                           # 변환 없음 → 다음 시도 / None
                pass
        return None

    def robot(self):
        """로봇 위치 (x, y, yaw) map, 모르면 None."""
        return self.lookup(self.p['map_frame'], self.p['base_frame'])

    def person(self, now):
        """칼만 → 지금 사람 위치, 속도 (속도는 max_person_speed로 자름). [쓰는 곳] track()"""
        pos, vel = self.kf.x[:2].copy(), self.kf.x[2:].copy()
        speed = np.linalg.norm(vel)
        if speed > self.p['max_person_speed']:
            vel *= self.p['max_person_speed'] / speed
        return pos + vel * (now - self.kf.t), vel       # 마지막 좌표 뒤 흐른 시간만큼 앞으로

    # ============================================================ ① 구독 콜백 (값만 바꾸고, 움직임은 step()이)
    def on_person(self, msg):
        """[① target_person/point_map] 좌표 → map으로 → 칼만 필터. 결과(kf.x, seen)는 다음 step()이 씀."""
        p = self.p
        if self.state not in ('WAIT_VISITOR', 'FOLLOW', 'SEARCH', 'WAIT'):
            return                                      # 관람 전·도킹·복귀 중 좌표는 버림
        if self.state == 'WAIT_VISITOR' and p['auto_register']:
            stamp = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
            if self.reg_first is None or stamp < self.reg_first + p['register_settle']:
                return                                  # 등록 요청 전 좌표 = 예전 대상일 수 있음
        x, y = msg.point.x, msg.point.y
        if msg.header.frame_id and msg.header.frame_id != p['map_frame']:   # map이 아니면 TF로 map 좌표로
            f = self.lookup(p['map_frame'], msg.header.frame_id, Time.from_msg(msg.header.stamp))
            if f is None:
                return
            c, s = math.cos(f[2]), math.sin(f[2])
            x, y = f[0] + c * x - s * y, f[1] + s * x + c * y
        now = self.now()
        if self.state == 'WAIT_VISITOR' and (self.seen_since is None or now - self.seen > p['register_gap']):
            self.seen_since = now                       # 등록 확인 시작 → wait_visitor()가 시간을 잼
        if self.kf.x is None or now - self.seen >= p['lost_timeout']:
            self.kf.reset((x, y), now)                  # 처음이거나 한동안 못 봄 → 새로 시작
        else:
            self.kf.update((x, y), now)                 # 튀는 값이면 필터가 버림
        self.seen = now                                 # step()이 '보이는 중인지' 판단하는 기준

    def on_enable(self, msg):
        """[① follow/enable] → Nav2Goal.set_enabled (false 멈춤 / true 다시)."""
        self.nav.set_enabled(msg.data)

    def on_dock_status(self, msg):
        """[① dock_status] → create3()가 undock·dock을 건너뛸지 판단."""
        self.docked = msg.is_docked

    def on_explaining(self, msg):
        """[① artwork_zone/explaining] → send_feedback()의 EXPLAINING."""
        self.explaining = msg.data if self.active else ''

    def on_explained(self, msg):
        """[① artwork_zone/explained] → run_tour() result의 visited_ids."""
        if self.active and msg.data not in self.visited:
            self.visited.append(msg.data)

    def on_zone_entered(self, msg):
        """[① artwork_zone/entered] 관람객이 end_zone(출구)에 들어감 → begin_return() → 다음 step()부터 RETURN."""
        if msg.data != self.p['end_zone']:
            return
        with self.lock:
            if self.active and self.state in ('FOLLOW', 'SEARCH', 'WAIT'):
                self.begin_return('finished', f'관람객이 종료 구역({msg.data})에 들어감')

    # ============================================================ ② 관람 스레드 (키오스크 액션)
    def run_tour(self, goal):
        """[② arti/start_tour 액션이 오면] 관람 하나를 처음부터 끝까지.
        (1) 검사·시작 → (2) Nav2 서버 확인 → (3) UNDOCK: Create 3 undock → LOCALIZE (이후 움직임은 ③ step())
        → (4) 0.2 s마다 feedback·취소 확인, DOCK이 되면 Create 3 dock → (5) active가 내려가면 result."""
        p = self.p
        # (1) 검사: 한 번에 한 관람, FREE만 → 통과하면 관람 값 초기화, UNDOCK(또는 LOCALIZE)
        with self.lock:
            problem = 'busy' if self.active else ''
            mode = (goal.request.mode if goal is not None else '').strip().upper()
            if not problem and mode not in ('', 'FREE'):
                problem = 'unsupported_mode'
            if not problem:
                self.reset_tour()
                self.active, self.end_reason = True, ''
                self.get_logger().info('관람 시작 (FREE)')
                self.set_state('UNDOCK' if p['undock'] else 'LOCALIZE')
        if problem:
            self.get_logger().warn(f'관람 시작 거절 ({problem}, mode "{mode}")')
            if goal is not None:
                goal.abort()
            return StartTour.Result(success=False, message=problem)
        # (2) Nav2가 없으면 움직일 수 없음 → no_nav2로 끝
        if not self.nav.client.wait_for_server(timeout_sec=p['nav_server_timeout']):
            self.get_logger().error(f'Nav2 navigate_to_pose 서버 없음 ({self.get_namespace().rstrip("/")}/navigate_to_pose)')
            with self.lock:
                self.end_tour('READY', 'no_nav2')
        # (3) undock은 Create 3가 스스로 움직임 → 여기서 끝날 때까지 기다림 → LOCALIZE부터는 ③ step()
        if self.state == 'UNDOCK':
            self.send_feedback(goal)
            why = self.create3(goal, self.undock_client, Undock, p['undock_timeout'], want_docked=False)
            with self.lock:
                if why:
                    self.end_tour('READY', why)
                elif self.active:
                    self.set_state('LOCALIZE')
        # (4) 관람 중: 취소 확인 + feedback. step()이 DOCK으로 바꾸면 여기서 Create 3 dock
        while self.active:
            if goal is not None and goal.is_cancel_requested:
                with self.lock:
                    self.end_tour('READY', 'canceled')
                break
            self.send_feedback(goal)
            if self.state == 'DOCK':
                why = self.create3(goal, self.dock_client, Dock, p['dock_timeout'], want_docked=True) if p['dock'] else ''
                with self.lock:
                    if self.active:
                        self.end_tour('READY', why or self.return_reason)
                continue
            time.sleep(0.2)
        # (5) end_tour()가 active를 내림 → 끝난 이유로 result
        reason = self.end_reason
        result = StartTour.Result(success=reason in ('canceled', 'finished'), message=reason,
                                  visited_ids=[artwork_number(a) for a in self.visited])
        self.visited = []                               # 관람 기록은 result로 넘기고 지움
        self.get_logger().info(f'관람 결과: success={result.success}, message={reason}, visited_ids={list(result.visited_ids)}')
        if goal is not None:
            {'canceled': goal.canceled, 'finished': goal.succeed}.get(reason, goal.abort)()
        return result

    def reset_tour(self):
        """[run_tour (1)] 한 관람에서 쓰는 값을 처음으로."""
        self.kf.x, self.seen, self.seen_since, self.goal_xy = None, -1e9, None, None
        self.reg_first = self.reg_last = None
        self.loc_at = self.pose_sent = self.face_at = None
        self.lost_at = self.wait_at = None
        self.return_at = self.return_reason = None
        self.registered, self.explaining, self.visited = False, '', []
        self.tour_end_sent = False

    def send_feedback(self, goal):
        """[run_tour (3)(4)] 내부 상태 → 키오스크 feedback (visitor_follower와 같은 대응)."""
        if goal is None:
            return
        explaining = self.explaining if self.state in ('FOLLOW', 'SEARCH', 'WAIT') else ''
        if explaining:
            state = 'EXPLAINING'
        elif self.state == 'WAIT_VISITOR':
            state = 'ARRIVED'
        elif self.state in ('RETURN', 'DOCK'):
            state = 'RETURNING'
        elif self.state in ('FOLLOW', 'SEARCH', 'WAIT', 'LOST'):
            state = 'FOLLOW'
        else:
            state = 'MOVING'                            # READY·UNDOCK·LOCALIZE·START_FACE
        goal.publish_feedback(StartTour.Feedback(
            state=state, registered=self.registered,
            current_artwork_id=artwork_number(explaining) if explaining else 0,
            current_index=len(self.visited), total=self.artwork_total))

    def create3(self, goal, client, kind, timeout, want_docked):
        """[run_tour (3) undock / (4) dock] Create 3 액션 → 끝까지 기다림 (시간 제한·관람 취소 확인).
        돌려줌: '' 성공(이미 그 상태면 건너뜀) / no_undock_server·undock_failed / no_dock_server·dock_failed / canceled"""
        name = 'dock' if want_docked else 'undock'
        if self.docked is want_docked:
            self.get_logger().info(f'이미 {name} 상태 → 건너뜀')
            return ''
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'{name} 액션 서버 없음')
            return f'no_{name}_server'
        self.get_logger().info(f'{name} 시작')
        sent = client.send_goal_async(kind.Goal())
        if not self.wait_future(sent, 5.0, goal) or not sent.result().accepted:
            return 'canceled' if goal is not None and goal.is_cancel_requested else f'{name}_failed'
        done = sent.result().get_result_async()
        if not self.wait_future(done, timeout, goal):
            sent.result().cancel_goal_async()
            return 'canceled' if goal is not None and goal.is_cancel_requested else f'{name}_failed'
        res = done.result()
        ok = (res.result.is_docked == want_docked) if res.status == GoalStatus.STATUS_SUCCEEDED else self.docked is want_docked
        self.get_logger().info(f'{name} 끝' if ok else f'{name} 실패 (status {res.status})')
        return '' if ok else f'{name}_failed'

    def wait_future(self, future, timeout, goal):
        """[create3] future가 끝날 때까지 기다림 (시간 초과·관람 취소면 False)."""
        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > end or (goal is not None and goal.is_cancel_requested):
                return False
            time.sleep(0.1)
        return True

    # ============================================================ 3. 상황판단 (③ 타이머 스레드)
    def control(self):
        """[③ 타이머, control_period마다] ② 관람 스레드와 겹치지 않게 lock → step()."""
        with self.lock:
            self.step()

    def step(self):
        """지금 상태에 맞는 함수 하나를 고른다. 위에서부터 차례로, 처음 맞는 곳에서 끝."""
        now = self.now()
        if self.state in ('READY', 'UNDOCK', 'DOCK', 'LOST'):
            return                                      # 액션 대기 / ② 관람 스레드가 Create 3 처리 중 / 관람 끝
        if self.state == 'LOCALIZE':
            return self.localize(now)                   # → START_FACE
        if self.state == 'START_FACE':
            return self.face_start(now)                 # → WAIT_VISITOR
        if self.state == 'WAIT_VISITOR':
            return self.wait_visitor(now)               # → FOLLOW
        if self.state == 'RETURN':
            return self.return_home(now)                # → DOCK (② 관람 스레드가 dock)
        if self.kf.x is not None and now - self.seen < self.p['lost_timeout']:
            return self.follow(now)                     # 사람이 보이는 중 → 4. 추종주행
        return self.lost(now)                           # 안 보임 → 5. 관람객 이탈처리

    def localize(self, now):
        """[step: LOCALIZE] (1) AMCL이 initialpose를 구독하면 출발 지점을 보냄 → (2) map 위치가 그 근처면 START_FACE.
        로봇은 움직이지 않는다. 위치를 아예 모르면 no_localization으로 관람 끝."""
        p = self.p
        if self.start is None:
            return self.set_state('START_FACE')         # 출발 지점을 모름 → 지금 AMCL 위치 그대로
        if self.loc_at is None:
            self.loc_at = now
        x, y, yaw = self.start
        if self.pose_sent is None:                      # (1) 한 번만
            if self.pose_pub.get_subscription_count() == 0:     # AMCL이 아직 구독 안 함
                if now - self.loc_at > p['localize_timeout']:
                    self.end_tour('READY', 'no_localization')
                return
            msg = PoseWithCovarianceStamped()
            msg.header.stamp, msg.header.frame_id = self.get_clock().now().to_msg(), p['map_frame']
            msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            cov = [0.0] * 36
            cov[0] = cov[7] = cov[35] = 0.05            # x, y, yaw 분산
            msg.pose.covariance = cov
            self.pose_pub.publish(msg)
            self.pose_sent = now
            self.get_logger().info(f'초기 위치 보냄 (map {x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)')
        pose = self.robot()                             # (2) 매 주기 확인
        if pose is not None and math.hypot(pose[0] - x, pose[1] - y) < p['localize_tolerance'] and now - self.pose_sent > 1.0:
            self.get_logger().info('초기 위치 잡힘')
            return self.set_state('START_FACE')
        if now - self.loc_at > p['localize_timeout']:
            if pose is None:
                return self.end_tour('READY', 'no_localization')   # map 위치를 아예 모름
            self.get_logger().warn(f'map 위치가 출발 지점에서 {math.hypot(pose[0] - x, pose[1] - y):.2f} m 떨어져 있음 → 그래도 진행')
            self.set_state('START_FACE')

    def face_start(self, now):
        """[step: START_FACE] (1) 처음: Nav2에 '같은 자리 + 키오스크 쪽 방향'(기본 BT) → 제자리 회전
        (2) 이후: Nav2 결과(④가 적은 status)가 오거나 face_timeout이면 WAIT_VISITOR."""
        if self.face_at is None:                        # (1)
            self.face_at = now
            pose = self.robot()
            if pose is None or (self.face_point is None and self.start is None):
                self.get_logger().warn('바라볼 곳이나 로봇 위치를 모름 → 돌지 않음')
                return self.set_state('WAIT_VISITOR')
            if self.face_point is not None:
                want = math.atan2(self.face_point[1] - pose[1], self.face_point[0] - pose[0])   # 키오스크 쪽
            else:
                want = self.start[2]                    # 출발 지점 방향
            self.nav.send((pose[0], pose[1], want), '', now)
            return
        if self.nav.status in ('succeeded', 'failed') or now - self.face_at > self.p['face_timeout']:   # (2)
            self.get_logger().info(f'시작 회전 끝 ({self.nav.status}) → 관람객 인식 기다림')
            self.nav.cancel()
            self.set_state('WAIT_VISITOR')

    def wait_visitor(self, now):
        """[step: WAIT_VISITOR] (1) 좌표가 없거나 끊김 → 등록 요청하며 기다림
        (2) 확인 중(register_confirm 전) → track(chase=False): 따라가지 않고 그 자리에서 사람 쪽 보기 (follow BT 시작)
        (3) 확인 끝 → 등록(음성) → FOLLOW (follow BT는 이미 돌고 있어 끊김 없이 추종)."""
        p = self.p
        if self.kf.x is None or now - self.seen > p['register_gap']:   # (1)
            if self.kf.x is not None:
                self.get_logger().info('등록 확인 중 끊김 → 다시 기다림')
            self.kf.x, self.seen_since, self.goal_xy = None, None, None
            if self.nav.want is not None:
                self.nav.cancel()                       # 바라보기 멈춤
            return self.request_register(now)
        if now - self.seen_since < p['register_confirm']:   # (2)
            return self.track(now, chase=False)
        self.registered = True                          # (3) → send_feedback의 registered
        self.get_logger().info(f'관람객 등록 완료 ({now - self.seen_since:.1f} s 끊김 없이 보임)')
        if p['recognized_guide']:
            self.guide_pub.publish(String(data=p['recognized_guide']))   # 음성 "인식되었습니다"
        self.set_state('FOLLOW')

    def request_register(self, now):
        """[wait_visitor (1)] 인식 모듈에 '바로 앞 사람 등록' 요청 (register_period마다)."""
        p = self.p
        if not p['auto_register'] or (self.reg_last is not None and now - self.reg_last < p['register_period']):
            return
        if self.reg_first is None:
            self.reg_first = now                        # on_person이 이 시각 전 좌표를 버림
            self.get_logger().info('바로 앞 사람 등록 요청 (target_person/register_front)')
        self.reg_last = now
        self.register_pub.publish(Empty())

    def begin_return(self, reason, why):
        """[① on_zone_entered] 관람 끝 → 관람객 데이터 삭제 → Nav2 목표 버림 → RETURN (다음 step()이 return_home)."""
        self.return_reason, self.return_at = reason, self.now()
        self.forget_visitor()
        self.nav.cancel()
        if self.home is None:
            self.get_logger().warn(f'{why} → 돌아갈 곳({self.p["return_point"]})을 몰라 그 자리에서 끝냄')
            return self.end_tour('READY', reason)
        self.get_logger().info(f'{why} → 도크 앞 {self.home}으로 복귀')
        self.set_state('RETURN')

    def return_home(self, now):
        """[step: RETURN] (1) 처음: Nav2에 '도크 앞(언독 지점) + 도크 쪽 방향'(기본 BT)
        (2) 성공 → DOCK (② 관람 스레드가 Create 3 dock) / 실패·return_timeout → return_failed."""
        if self.nav.want is None:                       # (1)
            if self.dock_xy is not None:
                yaw = math.atan2(self.dock_xy[1] - self.home[1], self.dock_xy[0] - self.home[0])   # 도크 쪽
            else:
                yaw = self.start[2] + math.pi if self.start is not None else 0.0
            self.nav.send((self.home[0], self.home[1], yaw), '', now)
        elif self.nav.status == 'succeeded':            # (2)
            self.get_logger().info(f'도크 앞 도착 ({now - self.return_at:.1f} s)')
            self.nav.cancel()
            self.set_state('DOCK')
        elif self.nav.status == 'failed' or now - self.return_at > self.p['return_timeout']:
            self.get_logger().warn(f'도크 앞으로 못 감 ({self.nav.status}, {now - self.return_at:.0f} s)')
            self.end_tour('READY', 'return_failed')

    def end_tour(self, state, reason):
        """[어디서든 관람이 끝날 때] Nav2 목표 취소 → 관람객 데이터 삭제(arti/tour_end) → state.
        active가 내려가면 ② run_tour (4)의 루프가 끝나 result를 보낸다."""
        self.active, self.end_reason = False, reason
        self.nav.cancel()
        self.forget_visitor()
        self.get_logger().info(f'관람 끝 ({reason})')
        self.set_state(state)

    def forget_visitor(self):
        """[end_tour, begin_return] 이 관람객 데이터를 지우고, 다른 노드도 지우게 arti/tour_end (관람마다 한 번)."""
        self.kf.x, self.seen, self.goal_xy = None, -1e9, None
        if not self.tour_end_sent:
            self.tour_end_sent = True
            self.tour_end_pub.publish(Empty())
            self.get_logger().info('관람객 데이터 삭제 (arti/tour_end)')

    def set_state(self, state):
        """[모든 상태 전환] 바뀔 때만 로그 + follow/state 발행 (artwork_zone이 구독)."""
        if state != self.state:
            self.get_logger().info(f'상태 {self.state} → {state}')
            self.state = state
            self.state_pub.publish(String(data=state))

    # ============================================================ 4. 추종주행 (step → follow → track → send_follow → Nav2)
    def follow(self, now):
        """[step: 사람이 보이는 중] FOLLOW로 두고 track(chase=True)."""
        self.set_state('FOLLOW')
        self.track(now, chase=True)

    def track(self, now, chase):
        """[follow, wait_visitor (2)] 목표 pose 계산 → send_follow().
        (1) 칼만에서 사람 위치·속도 → lead_time 앞 위치(ahead), 로봇까지 거리 d
        (2) 목표 위치: chase이고 d > target_distance + stop_margin(0.85 m) → 사람에서 로봇 쪽으로 target_distance 돌아온 점
                       아니면 그 자리 (이미 서 있으면 보낸 목표 그대로, 방금 안으로 들어왔으면 지금 위치)
        (3) 목표 방향: 사람 쪽 → Nav2(RotationShim)가 도착 후 그 방향으로 돎
        서기(0.85 m)·다시 출발(0.95 m)은 send_follow의 '0.15 m 넘게 바뀔 때만 보냄'에서 나온다 (별도 상태 없음)."""
        p = self.p
        pose = self.robot()
        if pose is None:
            return                                      # map 위치를 모르면 목표를 못 정함
        here = np.array(pose[:2])
        person, vel = self.person(now)                  # (1)
        ahead = person + vel * p['lead_time']           # 인식·통신이 늦는 만큼 앞을 봄
        d = float(np.linalg.norm(ahead - here))
        target = p['target_distance']
        if chase and d > target + p['stop_margin']:     # (2)
            pos = ahead - (ahead - here) / d * target
        elif self.goal_xy is not None and np.linalg.norm(self.goal_xy - here) <= p['update_min_move']:
            pos = self.goal_xy
        else:
            pos = here
        to = person - pos                               # (3)
        yaw = math.atan2(to[1], to[0]) if np.linalg.norm(to) > 1e-3 else pose[2]
        self.send_follow(pos, yaw, now)

    def send_follow(self, pos, yaw, now):
        """[track] follow BT로 넘기기.
        follow BT가 안 돌고 있음(시작·다른 BT에서 넘어옴·실패 뒤 resend_period) → Nav2Goal.send (navigate_to_pose 1번)
        돌고 있음 → 위치 update_min_move 또는 방향 update_min_yaw 넘게 바뀌었고 update_period 지났을 때만 Nav2Goal.update."""
        p = self.p
        want = self.nav.want
        if want is None or want[1] != self.follow_bt or (self.nav.status == 'failed' and now - self.nav.last_time >= p['resend_period']):
            self.nav.send((pos[0], pos[1], yaw), self.follow_bt, now)
            self.goal_xy = np.array(pos)
            return
        last = want[0]
        moved = math.hypot(pos[0] - last[0], pos[1] - last[1]) > p['update_min_move']
        turned = abs(wrap(yaw - last[2])) > p['update_min_yaw']
        if (moved or turned) and now - self.nav.last_time >= p['update_period']:   # 작은 흔들림은 무시
            self.nav.update((pos[0], pos[1], yaw), now)
            self.goal_xy = np.array(pos)

    # ============================================================ 5. 관람객 이탈처리 (step → lost)
    def lost(self, now):
        """[step: 사람이 lost_timeout 넘게 안 보임] 상태에 따라 한 단계씩:
        (1) FOLLOW였음(방금 놓침) → 사람이 가던 쪽 판단 → search BT(마지막 위치로 최대 5 s → 그쪽으로 한 바퀴) → SEARCH
        (2) SEARCH → Nav2 결과가 오거나 search_timeout → 취소 → WAIT
        (3) WAIT → wait_time 지나면 관람 끝(lost)
        어느 단계든 좌표가 다시 오면 step()이 follow()로 보냄 → send_follow가 follow BT로 바꿈."""
        p = self.p
        if self.kf.x is None:
            return                                      # 한 번도 못 봄 → 찾을 기준이 없음
        if self.state == 'FOLLOW':                      # (1)
            pose = self.robot()
            last, vel = self.kf.x[:2], self.kf.x[2:]
            heading = pose[2] if pose is not None else 0.0
            left = True
            if pose is not None:                        # 사람이 가던 쪽이 로봇 기준 왼쪽인지
                d = last + vel - np.array(pose[:2])
                left = wrap(math.atan2(d[1], d[0]) - pose[2]) >= 0
            self.nav.send((last[0], last[1], heading), self.search_bt[left], now)
            self.lost_at, self.wait_at, self.goal_xy = now, None, None
            self.get_logger().info(f'놓침 → 마지막 위치 ({last[0]:.2f}, {last[1]:.2f})로 가서 {"왼" if left else "오른"}쪽으로 한 바퀴')
            return self.set_state('SEARCH')
        if self.state == 'SEARCH':                      # (2)
            if self.nav.status in ('succeeded', 'failed') or now - self.lost_at > p['search_timeout']:
                self.get_logger().info(f'회전 탐색 끝 ({self.nav.status}, {now - self.lost_at:.1f} s) → 정지 대기')
                self.nav.cancel()
                self.wait_at = now
                self.set_state('WAIT')
            return
        if self.state == 'WAIT' and now - self.wait_at >= p['wait_time']:   # (3)
            self.end_tour('LOST', 'lost')


def main():
    """[시작] 노드 생성 → MultiThreadedExecutor (①~④ 스레드가 동시에 돌게) → spin.
    [끝] Ctrl+C → 관람 루프 종료 → Nav2 목표 취소(로봇 정지) → 종료."""
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # Ctrl+C 뒤에도 취소 요청을 보낼 수 있게
    node = Nav2Follow()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # Ctrl+C가 두 번 와도 정리는 끝까지
    finally:
        node.active = False                             # ② run_tour 루프 끝냄
        node.nav.cancel()                               # 로봇 정지
        time.sleep(0.2)                                 # 취소 요청이 나갈 시간
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
