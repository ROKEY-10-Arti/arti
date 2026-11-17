"""관람객 추종 — 주행을 Nav2에 맡기는 버전 (핵심만).

이 노드는 '어디로 갈지'만 정해서 Nav2 NavigateToPose로 보낸다. 경로 계획·장애물 회피(LiDAR, depth 포함)는
Nav2 costmap과 컨트롤러가 한다. Nav2가 안 움직이는 동작(가까이 서서 바라보기, 회전 탐색)만 직접 cmd_vel을 보낸다.
필요: Nav2 + localization(map → odom)이 실행 중이어야 함.

입력  target_person/point_map (PointStamped, map)  관람객 좌표
      tf, tf_static
      follow/enable (Bool)                         false면 Nav2 goal 취소 + cmd_vel 안 보냄
출력  navigate_to_pose (action goal), cmd_vel (TwistStamped, 서서 바라보기·회전 탐색 때만), follow/state (String)

상태  IDLE → FOLLOW(0.8 m 유지) → SEARCH(최종 좌표로 이동 → 회전) → WAIT → LOST
"""
import math

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, TwistStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

PARAMS = {
    'map_frame': 'map', 'base_frame': 'base_link',
    'target_distance': 0.8,         # 유지 거리 [m]
    'distance_tolerance': 0.05,     # target_distance + 이만큼 안이면 섬 [m]
    'restart_margin': 0.15,         # target_distance + 이만큼 밖이면 다시 출발 [m]
    'lead_time': 0.5,               # 관람객 속도로 이만큼 앞을 보고 목표를 잡음 [s]
    'kf_accel_std': 1.0, 'kf_meas_std': 0.15, 'kf_gate': 9.21, 'kf_max_rejects': 5,   # 칼만 필터
    'max_person_speed': 1.5,        # [m/s]
    'face_deadband': 0.15,          # [rad]
    'lost_timeout': 3.0, 'search_time': 10.0, 'search_move_time': 4.0, 'wait_time': 10.0,   # [s]
    'search_spin': 0.6,             # 회전 탐색 최소 속도 [rad/s]
    'max_angular': 1.5, 'angular_accel': 3.0,       # 직접 돌 때 [rad/s], [rad/s^2]
    'goal_update_dist': 0.3,        # 목표가 이만큼 바뀌어야 새 goal을 보냄 [m]
    'goal_update_period': 0.5,      # 새 goal은 최소 이 간격으로만 (너무 잦으면 Nav2가 계속 재계획) [s]
    'control_period': 0.1,
    'start_enabled': True,
}


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PersonKF:
    """관람객 칼만 필터 (등속 모델). 상태 x = [px, py, vx, vy]."""

    def __init__(self, accel_std, meas_std, gate, max_rejects):
        self.q, self.r, self.gate, self.max_rejects = accel_std ** 2, meas_std ** 2, gate, max_rejects
        self.x = self.P = self.t = None
        self.rejects = 0

    def reset(self, z, t):
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        self.P = np.diag([self.r, self.r, 1.0, 1.0])
        self.t, self.rejects = t, 0

    def update(self, z, t):
        dt = max(t - self.t, 0.0)                   # 예측
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        g = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x, self.P, self.t = F @ self.x, F @ self.P @ F.T + self.q * g @ g.T, t
        y = np.asarray(z, float) - self.x[:2]       # 보정
        S = self.P[:2, :2] + self.r * np.eye(2)
        if y @ np.linalg.solve(S, y) > self.gate:   # 이상값
            self.rejects += 1
            if self.rejects >= self.max_rejects:
                self.reset(z, t)
            return
        K = self.P[:, :2] @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ np.eye(2, 4)) @ self.P
        self.rejects = 0


class Nav2Follower(Node):

    def __init__(self):
        super().__init__('visitor_follower')
        self.p = p = {q.name: q.value for q in self.declare_parameters('', list(PARAMS.items()))}
        self.tf = Buffer()
        self.kf = PersonKF(p['kf_accel_std'], p['kf_meas_std'], p['kf_gate'], p['kf_max_rejects'])
        self.seen = -1e9
        self.lost_at, self.turn, self.spin_speed, self.holding = 0.0, 1.0, None, False
        self.state, self.enabled, self.cmd = None, p['start_enabled'], (0.0, 0.0)
        # Nav2 goal 관리
        self.nav_on = False                         # 지금 Nav2가 운전하길 원하는가
        self.goal, self.goal_sent, self.goal_handle, self.seq = None, -1e9, None, 0

        self.create_subscription(TFMessage, 'tf', lambda m: [self.tf.set_transform(t, 'tf') for t in m.transforms], 100)
        self.create_subscription(TFMessage, 'tf_static',
                                 lambda m: [self.tf.set_transform_static(t, 'tf') for t in m.transforms],
                                 QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PointStamped, 'target_person/point_map', self.on_person, 10)
        self.create_subscription(Bool, 'follow/enable', self.on_enable, 10)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.state_pub = self.create_publisher(String, 'follow/state',
                                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.set_state('IDLE')
        self.create_timer(p['control_period'], self.step)

    # ---------- 입력 ----------

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def lookup(self, target, source, stamp=None):
        for t in ([stamp] if stamp is not None else []) + [Time()]:
            try:
                tr = self.tf.lookup_transform(target, source, t).transform
                q = tr.rotation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                return tr.translation.x, tr.translation.y, yaw
            except Exception:
                pass
        return None

    def on_person(self, msg):
        f = self.lookup(self.p['map_frame'], msg.header.frame_id, Time.from_msg(msg.header.stamp))
        if f is None:
            return
        c, s = math.cos(f[2]), math.sin(f[2])
        z = (f[0] + c * msg.point.x - s * msg.point.y, f[1] + s * msg.point.x + c * msg.point.y)
        now = self.now()
        if self.kf.x is None or now - self.seen >= self.p['lost_timeout']:
            self.kf.reset(z, now)
        else:
            self.kf.update(z, now)
        self.seen = now

    def person(self, at=None):
        pos, vel = self.kf.x[:2].copy(), self.kf.x[2:].copy()
        speed = np.linalg.norm(vel)
        if speed > self.p['max_person_speed']:
            vel *= self.p['max_person_speed'] / speed
        if at is not None:
            pos += vel * (at - self.kf.t)
        return pos, vel

    def on_enable(self, msg):
        self.enabled = msg.data
        if not msg.data:
            self.stop_nav()
            self.publish(0.0, 0.0, force=True)      # 정지 한 번 보내고 cmd_vel을 넘겨줌

    # ---------- 상태 ----------

    def step(self):
        p, now = self.p, self.now()
        pose = self.lookup(p['map_frame'], p['base_frame'])
        if pose is None:
            self.stop_nav()
            return self.drive(0.0, 0.0)             # 위치 모름 → 정지
        here = np.array(pose[:2])

        if self.kf.x is not None and now - self.seen < p['lost_timeout']:
            self.set_state('FOLLOW')
            person, vel = self.person(at=now)
            ahead = person + vel * p['lead_time']
            to = ahead - here
            d = np.linalg.norm(to)
            if d <= p['target_distance'] + p['distance_tolerance']:
                self.holding = True
            elif d > p['target_distance'] + p['restart_margin']:
                self.holding = False
            if not self.holding:
                goal = ahead - to / d * p['target_distance']
                return self.go(goal, math.atan2(*(ahead - goal)[::-1]))   # 도착하면 관람객 쪽을 보도록
            self.stop_nav()
            return self.drive(0.0, self.face_w(pose, person))             # 서서 바라보기

        self.holding = False
        if self.kf.x is None:
            self.set_state('IDLE')
            self.stop_nav()
            return self.drive(0.0, 0.0)

        last, vel = self.person()
        if self.state == 'FOLLOW':                  # 방금 놓침 → 가던 방향 기억
            self.lost_at, self.spin_speed = now, None
            d = last + vel - here
            self.turn = 1.0 if wrap(math.atan2(d[1], d[0]) - pose[2]) >= 0 else -1.0
        t = now - self.lost_at
        if t < p['search_time']:
            self.set_state('SEARCH')
            if self.spin_speed is None and np.linalg.norm(last - here) > 0.5 and t < p['search_move_time']:
                return self.go(last, math.atan2(*(last - here)[::-1]))    # 최종 좌표로 이동
            self.stop_nav()
            if self.spin_speed is None:
                left = max(p['search_time'] - t, 1e-3)
                self.spin_speed = float(np.clip(1.05 * 2 * math.pi / left, p['search_spin'], p['max_angular']))
            return self.drive(0.0, self.turn * self.spin_speed)
        self.set_state('WAIT' if t < p['search_time'] + p['wait_time'] else 'LOST')
        self.stop_nav()
        self.drive(0.0, 0.0)

    # ---------- Nav2 ----------

    def go(self, goal, yaw):
        """Nav2에 goal을 보낸다. 목표가 충분히 바뀌었고 너무 잦지 않을 때만 새로 보냄 (Nav2가 이전 goal을 대체)."""
        if not self.enabled:
            return self.stop_nav()
        self.nav_on, self.cmd = True, (0.0, 0.0)   # 직접 보내던 cmd_vel은 멈추고 Nav2에 넘김
        now = self.now()
        if self.goal is not None and np.linalg.norm(goal - self.goal) < self.p['goal_update_dist']:
            return
        if now - self.goal_sent < self.p['goal_update_period'] or not self.nav.server_is_ready():
            return
        msg = NavigateToPose.Goal()
        msg.pose.header.frame_id = self.p['map_frame']
        msg.pose.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(goal[0]), float(goal[1])
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        self.seq += 1
        seq = self.seq
        self.nav.send_goal_async(msg).add_done_callback(lambda f: self.on_accepted(f, seq))
        self.goal, self.goal_sent = np.array(goal, float), now

    def on_accepted(self, future, seq):
        handle = future.result()
        if not handle.accepted:
            self.goal = None                        # 다음 주기에 다시 시도
            return
        if not self.nav_on:                         # 응답이 오기 전에 이미 멈추기로 함
            handle.cancel_goal_async()
            return
        if seq == self.seq:
            self.goal_handle = handle
            handle.get_result_async().add_done_callback(lambda f: self.on_result(f, seq))

    def on_result(self, future, seq):
        if seq == self.seq and future.result().status != GoalStatus.STATUS_SUCCEEDED:
            self.goal = None                        # 실패·취소 → 다음 주기에 다시 보냄

    def stop_nav(self):
        if self.nav_on and self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        self.nav_on, self.goal, self.goal_handle = False, None, None

    # ---------- 직접 회전 (Nav2가 운전하지 않을 때만) ----------

    def face_w(self, pose, target):
        turn = wrap(math.atan2(target[1] - pose[1], target[0] - pose[0]) - pose[2])
        if abs(turn) < self.p['face_deadband']:
            return 0.0
        return float(np.clip(1.5 * turn, -self.p['max_angular'], self.p['max_angular']))

    def drive(self, v, w):
        """각가속 한도 안에서 바꿔 보냄. (직진은 항상 0: 이동은 Nav2 몫)"""
        dw = self.p['angular_accel'] * self.p['control_period']
        w = self.cmd[1] + float(np.clip(w - self.cmd[1], -dw, dw))
        self.publish(v, w)

    def publish(self, v, w, force=False):
        self.cmd = (v, w) if self.enabled else (0.0, 0.0)
        if not (self.enabled or force) or self.nav_on:
            return
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.p['base_frame']
        msg.twist.linear.x, msg.twist.angular.z = float(v), float(w)
        self.cmd_pub.publish(msg)

    def set_state(self, state):
        if state != self.state:
            self.get_logger().info(f'상태 {self.state} → {state}')
            self.state = state
            self.state_pub.publish(String(data=state))


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Nav2Follower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_nav()
        node.publish(0.0, 0.0, force=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
