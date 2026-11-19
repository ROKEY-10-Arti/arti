"""주행·추정 공통 (visitor_follower 노드의 일부) — 등록 단계와 관람 단계가 같이 쓴다.

  PersonKF      관람객 좌표 칼만 필터 (등속 모델, 이상값 버림)
  wrap·yaw_of·apply   각도·좌표 변환 도우미
  MotionMixin   face(제자리 돌기), obstacles·field·dwa(장애물 회피 경로), ramp(가감속), drive(cmd_vel)
"""
import math
import time

import cv2
from geometry_msgs.msg import TwistStamped
import numpy as np


def wrap(a):
    """각도를 -π ~ π 범위로 맞춘다 (350° → -10°). 회전할 때 짧은 쪽으로 돌게 하려고.
    받음: a  각도 [rad] (숫자 또는 numpy 배열)
    돌려줌: -π ~ π 각도 [rad]"""
    return (a + np.pi) % (2 * np.pi) - np.pi


def yaw_of(q):
    """쿼터니언에서 바닥 위 방향(yaw)만 뽑는다. 로봇은 바닥에서만 움직이므로 기울기(roll, pitch)는 버린다.
    받음: q  geometry_msgs/Quaternion (x, y, z, w)
    돌려줌: yaw [rad] (0 = x축 방향, + = 왼쪽으로 돈 쪽)"""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def apply(f, pts):
    """점들을 다른 좌표계로 옮긴다 (2D 회전 + 이동). 예: 라이다 좌표 → odom 좌표.
    받음: f    변환 (x, y, yaw)  lookup()의 결과
          pts  점들 (N, 2) numpy 배열
    돌려줌: 옮긴 점들 (N, 2)"""
    c, s = math.cos(f[2]), math.sin(f[2])
    return np.stack([f[0] + c * pts[:, 0] - s * pts[:, 1], f[1] + s * pts[:, 0] + c * pts[:, 1]], 1)


class PersonKF:
    """관람객 위치 칼만 필터 (등속 모델: 사람이 같은 속도로 걷는다고 가정).
    상태 x = [px, py, vx, vy] (odom 좌표, m · m/s). 튀는 인식 좌표를 매끄럽게 하고, 위치만 받아도 속도를 추정한다."""

    def __init__(self, accel_std, meas_std, gate, max_rejects):
        """필터 준비. 아직 사람 정보 없음 (x = None).
        받음: accel_std    사람 가속도 잡음 [m/s²] (클수록 빨리 따라가고 덜 매끄러움)
              meas_std     인식 좌표 잡음 [m] (클수록 매끄럽고 느림)
              gate         이상값 기준 (예측과 이만큼 넘게 다르면 버림)
              max_rejects  연속으로 이만큼 버리면 사람이 정말 옮긴 것으로 보고 새로 시작"""
        self.q, self.r, self.gate, self.max_rejects = accel_std ** 2, meas_std ** 2, gate, max_rejects
        self.x, self.P, self.t, self.rejects = None, None, None, 0

    def reset(self, z, t):
        """이 좌표로 필터를 새로 시작한다 (처음 보거나 오래 못 봤을 때). 속도는 모르니 0, 불확실도는 크게.
        받음: z  좌표 (x, y) odom
              t  시각 [s]
        바뀜: x = [x, y, 0, 0], P, t"""
        self.x = np.array([z[0], z[1], 0.0, 0.0])
        self.P = np.diag([self.r, self.r, 1.0, 1.0])   # 속도는 모름 → 크게
        self.t, self.rejects = t, 0

    def predict(self, t):
        """지난 위치 + 속도 × 지난 시간 → 지금쯤 위치를 예측한다 (불확실도도 시간만큼 커짐).
        받음: t  예측할 시각 [s]
        바뀜: x, P, t"""
        dt = max(t - self.t, 0.0)
        F = np.eye(4)
        F[0, 2] = F[1, 3] = dt
        g = np.array([[dt * dt / 2, 0], [0, dt * dt / 2], [dt, 0], [0, dt]])
        self.x, self.P, self.t = F @ self.x, F @ self.P @ F.T + self.q * g @ g.T, t

    def update(self, z, t):
        """좌표 하나 반영 (reset 뒤에만): 예측 → 받은 좌표와 비교 → 너무 다르면 버림, 아니면 섞어서 보정.
        받음: z  좌표 (x, y) odom
              t  받은 시각 [s]
        돌려줌: 반영했으면 True, 이상값으로 버렸으면 False (예측은 반영됨)
              연속 max_rejects번 버리면 사람이 정말 옮긴 것으로 보고 reset"""
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


class MotionMixin:
    """VisitorFollower가 상속해서 쓰는 주행 함수 (self.p, self.cmd 등 노드 상태를 그대로 씀)."""

    def face(self, pose, here, person):
        """제자리에서 감속해 서서 사람 쪽으로 몸만 돌린다 (face_deadband 9° 안이면 안 돎 → 꿈틀거림 방지).
        받음: pose    로봇 (x, y, yaw) odom
              here    로봇 (x, y)
              person  사람 (x, y)
        내보냄: drive(ramp(0, w)) → cmd_vel (전진 0, 회전 w)"""
        turn = wrap(math.atan2(*(person - here)[::-1]) - pose[2])
        w = float(np.clip(1.5 * turn, -self.p['max_angular'], self.p['max_angular']))
        if abs(turn) < self.p['face_deadband']:
            w = 0.0
        return self.drive(*self.ramp(0.0, w))

    def obstacles(self):
        """지금 피해야 할 장애물 점 = 라이다 점 + depth_keep(1 s) 안의 바닥 물체 점.
        받음: (scan_pts, floor_pts, floor_time)
        돌려줌: 장애물 점 (N, 2) odom"""
        # depth(바닥 물체)는 초당 약 3번만 와서, 마지막으로 받은 걸 depth_keep(1 s) 동안 기억해서 쓴다.
        # 1 s 넘게 안 왔으면 오래된 점이라 버린다 (이미 치운 물체를 계속 피하지 않게).
        fresh = self.now() - self.floor_time < self.p['depth_keep']
        # 라이다 점(벽·기둥) + (최근이면) 바닥 물체 점을 한 배열로 합친다 → dwa·field가 장애물로 씀
        return np.concatenate([self.scan_pts] + ([self.floor_pts] if fresh else []))

    def dwa(self, pose, goal, face):
        """(v, w) 후보를 굴려 보고, 안 부딪히면서 목표까지 '돌아가는 거리'가 가장 줄어드는 것을 고른다 (DWA).
        ① 후보 105개 ② 1.5 s 굴려 봄 ③ 위험한 후보 버림 ④ 벌점 최소 — 단계별 입출력은 본문 주석.
        받음: pose (x, y, yaw) odom, goal 목표 (x, y), face 바라볼 곳 (x, y), (self.cmd: 지난 명령 = 지금 속도로 가정)
        돌려줌: (v [m/s], w [rad/s]). 모든 후보가 막혔으면 (0, face 쪽 회전)"""
        # ── 입력 풀기 ──────────────────────────────────────────────────────────────
        # pose      → x0, y0 [m], th0 [rad]   로봇 위치·방향 (odom)            예) 1.0, 0.0, 0.0
        # self.cmd  → v0 [m/s], w0 [rad/s]    지난번 보낸 명령 = 지금 속도로 가정  예) 0.20, 0.0
        p, (x0, y0, th0), (v0, w0) = self.p, pose, self.cmd
        dt = p['control_period']
        # ── ① 후보 만들기 ──────────────────────────────────────────────────────────
        # 입력: v0, w0, 가속 한계(linear_accel 0.5, angular_accel 3.0), 최대 속도(0.3, 1.5)
        # 출력: vs (7,)  0.1 s 안에 낼 수 있는 전진 속도 7개   예) 0.15 ~ 0.25
        #       ws (15,) 0.1 s 안에 낼 수 있는 회전 속도 15개  예) -0.3 ~ 0.3
        vs = np.linspace(max(0.0, v0 - p['linear_accel'] * dt),
                         min(p['max_linear'], v0 + p['linear_accel'] * dt), p['v_samples'])
        ws = np.linspace(max(-p['max_angular'], w0 - p['angular_accel'] * dt),
                         min(p['max_angular'], w0 + p['angular_accel'] * dt), p['w_samples'])
        # 출력: V, W (105,)  모든 조합을 한 줄로 편 것. 후보 i = (V[i], W[i])
        V, W = (g.ravel() for g in np.meshgrid(vs, ws))
        # ── ② 굴려 보기 (sim_time 1.5 s를 0.1 s씩 15칸) ─────────────────────────────
        # 입력: V, W, x0, y0, th0
        # 출력: th (105, 15)  후보별·시간별 방향 [rad]
        #       xs, ys (105, 15)  후보별·시간별 위치 [m]   행 = 후보, 열 = 0.1 ~ 1.5 s
        th = th0 + W[:, None] * np.arange(1, int(p['sim_time'] / 0.1) + 1) * 0.1
        xs = x0 + np.cumsum(V[:, None] * np.cos(th) * 0.1, 1)
        ys = y0 + np.cumsum(V[:, None] * np.sin(th) * 0.1, 1)

        # ── ③ 위험한 후보 버리기 ────────────────────────────────────────────────────
        # 입력: obstacles() → obs (N, 2)  라이다 + 바닥 물체 점 (odom)
        # 출력: near (M, 2)  로봇에서 1.05 m(0.3 × 1.5 + 0.6) 안의 점만 (먼 점은 궤적과 상관없어 계산 줄임)
        obs = self.obstacles()
        near = obs[np.hypot(obs[:, 0] - x0, obs[:, 1] - y0) < p['max_linear'] * p['sim_time'] + 0.6]
        # 출력: clear (105,)   후보별로 궤적 15점 중 장애물에 가장 가까이 간 거리 [m] (장애물 없으면 inf)
        #       here_clear     지금 로봇 위치에서 가장 가까운 장애물 거리 [m]
        clear, here_clear = np.full(V.size, np.inf), np.inf
        if len(near):
            clear = np.hypot(xs[..., None] - near[:, 0], ys[..., None] - near[:, 1]).min((1, 2))
            here_clear = np.hypot(near[:, 0] - x0, near[:, 1] - y0).min()
        # 여유를 지키는 경로, 또는 이미 붙어 있을 때 더 가까워지지 않는 경로(빠져나오기). 로봇 반지름 안은 금지
        # 출력: safe (105,) True/False  통과 조건 둘 중 하나
        #   - clear > 0.25 (로봇 반지름 0.2 + 여유 0.05): 여유 있게 지나감
        #   - 이미 벽에 붙어 있을 때: 몸(0.2)이 닿지 않고 지금보다 안 가까워짐 → 빠져나가는 길 (없으면 꼼짝 못 함)
        r = p['robot_radius']
        safe = (clear > r + p['safety_margin']) | ((clear > r) & (clear >= here_clear - 0.005))
        # 전부 위험 → 앞으로 안 가고 face(사람) 쪽으로 제자리 회전만. 출력: (0.0, w)  w = 1.5 × 어긋난 각도 (최대 ±1.5)
        if not safe.any():                          # 막힘 → 제자리에서 대상 쪽으로
            self.dbg['blocked'] = True
            turn = wrap(math.atan2(face[1] - y0, face[0] - x0) - th0)
            return 0.0, float(np.clip(1.5 * turn, -p['max_angular'], p['max_angular']))

        # ── ④ 점수(벌점) 매기고 고르기 — 작을수록 좋음 ─────────────────────────────
        # 입력: goal, 로봇 위치, obs
        # 출력: field (n, n)  칸마다 goal까지 장애물을 돌아가는 거리 [m],  origin 격자 원점 (x, y)
        t0 = time.monotonic()
        field, origin = self.field(np.asarray(goal, float), np.array([x0, y0]), obs)
        self.dbg['field_ms'] = (time.monotonic() - t0) * 1000
        # 출력: ex, ey, eth (105,)  후보별 1.5 s 뒤 도착 위치·방향 (② 결과의 마지막 열)
        ex, ey, eth = xs[:, -1], ys[:, -1], th[:, -1]
        # 출력: cost (105,)  후보별 벌점 =
        #   sample(field, 도착점)               도착점에서 goal까지 돌아가는 거리 [m]   (가장 중요)
        #   + 1.0  × max(0.4 - clear, 0)        장애물에 0.4 m 안으로 붙은 만큼
        #   + 0.02 × |W|                        회전 (비슷하면 직진)
        #   + (goal 0.3 m 안이면) 0.3 × |사람 쪽과 어긋난 각도|  사람을 바라보며 서게
        cost = self.sample(field, origin, ex, ey) \
            + p['w_clearance'] * np.clip(p['clearance_zone'] - clear, 0, None) \
            + p['w_turn'] * np.abs(W)                       # 비용이 같으면 덜 도는 쪽
        if math.hypot(goal[0] - x0, goal[1] - y0) < 0.3:   # 거의 도착 → 관람객 쪽을 바라봄
            cost += p['w_heading'] * np.abs(wrap(np.arctan2(face[1] - ey, face[0] - ex) - eth))
        cost[~safe] = np.inf
        # 출력: i  벌점이 가장 작은 후보 번호 (0 ~ 104). 위험한 후보는 cost = inf라 절대 안 뽑힘
        # 결과: (v [m/s], w [rad/s])  예) (0.25, 0.3) → drive()가 cmd_vel로 보냄. 다음 0.1 s만 쓰고 다시 계산
        i = int(np.argmin(cost))
        return float(V[i]), float(W[i])

    def field(self, goal, here, obs):
        """격자 칸마다 goal까지 장애물을 돌아가는 거리 [m] (dwa의 벌점용).
        10 cm 격자 → 장애물 칸을 로봇 반지름+여유만큼 부풀림 → 목표에서 BFS로 퍼져 나가며 거리 채움.
        받음: goal (x, y), here 로봇 (x, y), obs 장애물 점 (N, 2)
        돌려줌: (거리 지도 n×n, 지도 원점 (x, y)). 못 가는 칸은 50"""
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
        """거리 지도에서 점들의 값을 읽는다. 칸 사이는 주변 4칸을 섞어서(bilinear) → 후보끼리 점수가 같아지지 않게.
        받음: field 거리 지도, origin 지도 원점, x, y 점들 (배열)
        돌려줌: 점마다 거리 [m]"""
        n = field.shape[0]
        g = (np.stack([x, y], 1) - origin) / self.p['grid_resolution'] - 0.5
        i = np.clip(np.floor(g).astype(int), 0, n - 2)
        t = np.clip(g - i, 0.0, 1.0)
        f = [field[i[:, 1] + dy, i[:, 0] + dx] for dy in (0, 1) for dx in (0, 1)]
        return (f[0] * (1 - t[:, 0]) + f[1] * t[:, 0]) * (1 - t[:, 1]) + (f[2] * (1 - t[:, 0]) + f[3] * t[:, 0]) * t[:, 1]

    def ramp(self, v, w):
        """속도를 한 번에 바꾸지 않고 0.1 s에 바꿀 수 있는 만큼만 바꾼다 (덜컹 방지).
        감속 linear_decel, 가속 linear_accel, 회전 angular_accel 한도.
        받음: v, w  원하는 속도, (self.cmd: 지난 명령)
        돌려줌: (v, w)  이번에 보낼 속도"""
        #속도를 한번에 확 바꾸지 않고 조금씩 바꿔 주는 함
        p, (v0, w0) = self.p, self.cmd
        dt = p['control_period']
        dv = p['linear_decel'] if abs(v) < abs(v0) else p['linear_accel']
        v = v0 + float(np.clip(v - v0, -dv * dt, dv * dt))
        w = w0 + float(np.clip(w - w0, -p['angular_accel'] * dt, p['angular_accel'] * dt))
        return v, w

    def drive(self, v, w):
        """속도 명령을 로봇에 보낸다. enabled가 false면 보내지 않음. 보낸 값을 self.cmd에 기억 (ramp·dwa가 '지금 속도'로 씀).
        받음: v  전진 속도 [m/s], w  회전 속도 [rad/s] (+ 왼쪽)
        내보냄: cmd_vel (TwistStamped: twist.linear.x = v, twist.angular.z = w)"""
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
