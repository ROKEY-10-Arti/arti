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

try:                                                # 격자 최단 거리를 C로 계산 (없으면 numpy로 대신)
    from scipy.sparse import coo_matrix as _coo_matrix
    from scipy.sparse.csgraph import dijkstra as _csgraph_dijkstra
except ImportError:
    _coo_matrix = _csgraph_dijkstra = None

_SQRT2 = math.sqrt(2.0)


def grid_distances(free, gy, gx):
    """8방향 격자에서 (gy, gx)부터 모든 칸까지의 최단 거리 [칸] (대각선 √2). 못 가는 칸은 inf.
    (팀원 브랜치 fix/follow-stability-perf에서 가져옴. 예전 순수 Python BFS보다 3~7배 빠르고 결과는 같음)
    SciPy가 있으면: 이웃 칸끼리 잇는 그래프를 만들어 C 다익스트라 한 번.
    없으면: numpy로 격자 전체를 한꺼번에 '이웃 + 1(또는 √2)'로 줄여 가기를 바뀌지 않을 때까지 반복.
    받음: free  (n, n) bool, 지나갈 수 있는 칸 = True
          gy, gx  출발 칸 (목표)
    돌려줌: (n, n) 거리 [칸]"""
    n = free.shape[0]
    if _coo_matrix is not None:
        idx = np.arange(n * n, dtype=np.int32).reshape(n, n)
        rows, cols, weights = [], [], []
        for dy, dx, w in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, _SQRT2), (1, -1, _SQRT2)):   # 방향 4개 (양방향 그래프)
            y0, y1 = max(0, -dy), n - max(0, dy)
            x0, x1 = max(0, -dx), n - max(0, dx)
            both = free[y0:y1, x0:x1] & free[y0 + dy:y1 + dy, x0 + dx:x1 + dx]   # 두 칸 다 지나갈 수 있으면 연결
            if both.any():
                rows.append(idx[y0:y1, x0:x1][both])
                cols.append(idx[y0 + dy:y1 + dy, x0 + dx:x1 + dx][both])
                weights.append(np.full(rows[-1].size, w))
        if not rows:
            dist = np.full((n, n), np.inf)
            dist[gy, gx] = 0.0
            return dist
        graph = _coo_matrix((np.concatenate(weights), (np.concatenate(rows), np.concatenate(cols))),
                            shape=(n * n, n * n))
        return _csgraph_dijkstra(graph, directed=False, indices=gy * n + gx).reshape(n, n)
    d = np.full((n + 2, n + 2), np.inf)             # 테두리 1칸을 inf로 둘러 경계 검사 없이 계산
    d[1 + gy, 1 + gx] = 0.0
    for _ in range(2 * n):
        card = np.minimum(np.minimum(d[:-2, 1:-1], d[2:, 1:-1]), np.minimum(d[1:-1, :-2], d[1:-1, 2:])) + 1.0
        diag = np.minimum(np.minimum(d[:-2, :-2], d[:-2, 2:]), np.minimum(d[2:, :-2], d[2:, 2:])) + _SQRT2
        cur = d[1:-1, 1:-1]
        nxt = np.where(free, np.minimum(cur, np.minimum(card, diag)), cur)
        if np.array_equal(nxt, cur):
            break
        d[1:-1, 1:-1] = nxt
    return d[1:-1, 1:-1]


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


def find_people(r, a, ok, p):
    """라이다 한 장에서 '사람 다리처럼 생긴 묶음'을 찾는다 (라이다 좌표). 지도 빼기는 노드가 따로 한다.
    ① 묶기: scan 순서대로 이웃 점 사이가 seg_gap 넘게 벌어지거나 빈 ray가 2개 넘게 있으면 다른 묶음
    ② 모양: 점 leg_min_points개 이상, 양 끝 폭 leg_min_width ~ leg_max_width (벽처럼 긴 묶음은 버림)
    ③ 다리 둘 → 사람 하나: 다리 중심끼리 leg_pair_max 안이면 가운데 점. 짝이 없으면 다리 하나 = 사람 하나 (가려짐)
    받음: r 거리, a 각도 (scan 전체 배열), ok 쓸 수 있는 ray (bool), p 파라미터
    돌려줌: 사람 후보 (K, 2) 라이다 좌표"""
    idx = np.nonzero(ok)[0]
    if idx.size < p['leg_min_points']:
        return np.zeros((0, 2))
    pts = np.stack([r[idx] * np.cos(a[idx]), r[idx] * np.sin(a[idx])], 1)
    cut = (np.diff(idx) > 2) | (np.hypot(*np.diff(pts, axis=0).T) > p['seg_gap'])
    legs = []
    for seg in np.split(np.arange(idx.size), np.nonzero(cut)[0] + 1):
        if seg.size < p['leg_min_points']:
            continue
        width = float(np.hypot(*(pts[seg[-1]] - pts[seg[0]])))
        if not p['leg_min_width'] <= width <= p['leg_max_width']:
            continue
        # 앞으로 튀어나왔나: 다리는 양옆 ray(묶음 바로 밖)가 더 멀거나 비어 있다. 다리 사이로 보이는 벽 조각은
        # 양옆이 다리라 더 가까움 → 버림 (2026-11-20 가짜 scan 시험에서 벽 조각이 후보로 나옴)
        i0, i1 = idx[seg[0]], idx[seg[-1]]
        left = r[i0 - 1] if i0 > 0 and ok[i0 - 1] else np.inf
        right = r[i1 + 1] if i1 + 1 < r.size and ok[i1 + 1] else np.inf
        if left > r[i0] + p['seg_gap'] and right > r[i1] + p['seg_gap']:
            legs.append(pts[seg].mean(0))
    people, used = [], set()
    for i in range(len(legs)):                      # 가까운 다리끼리 짝 (다리는 많아야 수십 개라 반복문으로 충분)
        if i in used:
            continue
        best, best_d = None, p['leg_pair_max']
        for j in range(i + 1, len(legs)):
            d = float(np.hypot(*(legs[j] - legs[i])))
            if j not in used and d <= best_d:
                best, best_d = j, d
        if best is None:
            people.append(legs[i])
        else:
            used.add(best)
            people.append((legs[i] + legs[best]) / 2)
    return np.array(people).reshape(-1, 2)


class StaticClutter:
    """라이다 사람 후보 중 '계속 그 자리에 있는 것'(기둥·벽 모서리, 지도에 없는 받침대)을 배워서 뺀다.
    다리는 움직이고 모서리는 안 움직인다 (2026-11-20: 모서리를 다리로 잡음).
    - 배우기: 관람객에게서 먼(learn=True) 후보가 같은 칸(cell)에 끊김(1 s) 없이 hold_time 넘게 보이면 고정 물체
    - 관람객 근처에서는 새로 배우지 않음 → 서서 감상 중인 관람객 본인은 고정 물체가 되지 않음
      (이미 고정 물체인 칸은 관람객 근처여도 계속 보이면 유지)
    - forget_time 동안 안 보이면 잊음 (다른 관람객이 서 있다 떠난 자리)"""

    def __init__(self, cell, hold_time, forget_time):
        self.cell, self.hold, self.forget = cell, hold_time, forget_time
        self.cells = {}                             # (i, j) → [처음 본 시각, 마지막 본 시각]

    def filter(self, pts, learn, now):
        """받음: pts 후보 (K, 2) 고정 좌표계(map), learn (K,) bool 배워도 되는 후보, now [s]
        돌려줌: (K,) bool 남길 후보 (고정 물체 칸과 그 둘레 8칸에 있으면 False)"""
        keep = np.ones(len(pts), bool)
        for i, ((x, y), ok) in enumerate(zip(pts, learn)):
            k = (int(math.floor(x / self.cell)), int(math.floor(y / self.cell)))
            static = any(self.is_static((k[0] + dx, k[1] + dy), now) for dx in (-1, 0, 1) for dy in (-1, 0, 1))
            c = self.cells.get(k)
            if c is not None and now - c[1] <= 1.0:
                c[1] = now                          # 이어서 보임 (고정 물체든 배우는 중이든)
            elif ok:
                self.cells[k] = [now, now]          # 처음이거나 끊겼다 다시 → 새로 셈
            keep[i] = not static
        if len(self.cells) > 2000 or not int(now) % 5:   # 오래된 칸 정리 (가끔)
            self.cells = {k: c for k, c in self.cells.items() if now - c[1] <= self.forget}
        return keep

    def is_static(self, k, now):
        c = self.cells.get(k)
        return c is not None and c[1] - c[0] >= self.hold and now - c[1] <= self.forget


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

    def obstacles(self, here=None):
        """지금 피해야 할 장애물 점 = 라이다 점(관람객 다리 뺌) + 지도 벽 점 (map_obstacles, AMCL map→odom이 있을 때만).
        OAK-D depth 바닥 점은 뺐다 (2026-11-18, free_tracking 9715cbf): 관람객이 멀면 빈 바닥이 장애물로 잡혀
        DWA가 목표까지 길이 없다고 보고 조용히 멈춤 (docs/troubleshooting/follow/2026-11-18-far-visitor-stall-depth-floor.md)
        지도 벽 점 중 로봇에서 robot_radius 안의 것은 버린다: 로봇이 실제로 거기 있으니 AMCL 오차로 벽이 몸 안에
        들어온 것 → 그대로 두면 모든 궤적이 금지돼 꼼짝 못 함 (라이다가 보는 진짜 벽은 그대로 피함)
        받음: here 로봇 (x, y) odom (없으면 위 거르기 안 함), (scan_pts, map_obs)
        돌려줌: 장애물 점 (N, 2) odom"""
        p, pts = self.p, self.scan_pts
        # 추종 중: 관람객 본인의 다리 점은 장애물에서 뺀다 (person_obstacle_clear 안, 반지름 0.28로 커져 사람 주변이
        # 막힌 길이 됨). 단, 로봇 몸 가까이(robot_radius + 0.15 안)의 점은 사람 추정이 틀렸을 수 있어 그대로 둔다
        if p['person_obstacle_clear'] > 0 and here is not None and self.kf.x is not None and self.state == 'FOLLOW':
            last, vel = self.person()
            ps = last + vel * min(self.now() - self.kf.t, p['predict_max'])
            drop = (np.hypot(pts[:, 0] - ps[0], pts[:, 1] - ps[1]) < p['person_obstacle_clear']) \
                & (np.hypot(pts[:, 0] - here[0], pts[:, 1] - here[1]) > p['robot_radius'] + 0.15)
            pts = pts[~drop]
        if self.map_obs is None:
            return pts
        f = self.lookup(p['odom_frame'], p['map_frame'])
        if f is None:                               # AMCL 전 → 라이다만
            return pts
        wall = apply(f, self.map_obs)
        if here is not None:
            wall = wall[np.hypot(wall[:, 0] - here[0], wall[:, 1] - here[1]) > p['robot_radius']]
        return np.vstack([pts, wall])

    def dwa(self, pose, goal, face, keep_view=False):
        """(v, w) 후보를 굴려 보고, 안 부딪히면서 목표까지 '돌아가는 거리'가 가장 줄어드는 것을 고른다 (DWA).
        ① 후보 105개 ② 1.5 s 굴려 봄 ③ 위험한 후보 버림 ④ 벌점 최소 — 단계별 입출력은 본문 주석.
        받음: pose (x, y, yaw) odom, goal 목표 (x, y), face 바라볼 곳 (x, y), (self.cmd: 지난 명령 = 지금 속도로 가정)
        돌려줌: (v [m/s], w [rad/s]). 모든 후보가 막혔으면 (0, face 쪽 회전)"""
        # ── 입력 풀기 ──────────────────────────────────────────────────────────────
        # pose      → x0, y0 [m], th0 [rad]   로봇 위치·방향 (odom)            예) 1.0, 0.0, 0.0
        # self.cmd  → v0 [m/s], w0 [rad/s]    지난번 보낸 명령 = 지금 속도로 가정  예) 0.20, 0.0
        p, (x0, y0, th0), (v0, w0) = self.p, pose, self.cmd
        dt = p['control_period']
        self.path_dist = None                       # 이번에 못 구하면(전부 막힘) None

        # ── ① 후보 만들기 ──────────────────────────────────────────────────────────
        # 입력: v0, w0, 가속 한계(linear_accel 0.5, angular_accel 3.0), 최대 속도(0.3, 1.5)
        # 출력: vs (7,)  0.1 s 안에 낼 수 있는 전진 속도 7개   예) 0.15 ~ 0.25
        #       ws (15,) 0.1 s 안에 낼 수 있는 회전 속도 15개  예) -0.3 ~ 0.3
        vs = np.linspace(max(0.0, v0 - p['linear_accel'] * dt),
                         min(p['max_linear'], v0 + p['linear_accel'] * dt), p['v_samples'])
        ws = np.linspace(max(-p['max_angular'], w0 - p['angular_accel'] * dt),
                         min(p['max_angular'], w0 + p['angular_accel'] * dt), p['w_samples'])
        if ws[0] <= 0.0 <= ws[-1]:
            ws = np.union1d(ws, [0.0])              # 정확히 0도 후보로 (없으면 0.01 같은 값이 계속 뽑혀 제자리에서 돎)
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
        obs = self.obstacles((x0, y0))
        reach = p['max_linear'] * p['sim_time'] + 0.6
        near = obs[np.hypot(obs[:, 0] - x0, obs[:, 1] - y0) < reach]
        # 출력: clear (105,)   후보별로 궤적 15점 중 장애물에 가장 가까이 간 거리 [m] (장애물 없으면 inf)
        #       here_clear     지금 로봇 위치에서 가장 가까운 장애물 거리 [m]
        # 거리 변환(clearance_grid)으로 한 번에: 궤적 점 1575개 × 장애물 점 M개를 다 재던 것(지도 벽을 더해 M이 늘어
        # 느려짐)을 '격자 한 번 + 점 1575개 읽기'로 바꿈 (2026-11-20)
        clear, here_clear = np.full(V.size, np.inf), np.inf
        if len(near):
            t0 = time.monotonic()
            cgrid, corigin = self.clearance_grid(near, x0, y0, reach)
            clear = self.sample(cgrid, corigin, xs.ravel(), ys.ravel(), p['clear_resolution']) \
                .reshape(xs.shape).min(1)
            here_clear = float(self.sample(cgrid, corigin, np.array([x0]), np.array([y0]),
                                           p['clear_resolution'])[0])
            self.dbg['clear_ms'] = (time.monotonic() - t0) * 1000
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
        # 목표·로봇이 field_reuse_dist(10 cm) 넘게 안 움직였고 field_max_age(0.5 s) 안이면 지난 거리 지도를 다시 씀
        # (제어 밀림 줄이기, 2026-11-20). 안전 판정(③ clear)은 매번 새 장애물로 하므로 길 안내만 조금 늦게 바뀜
        t0 = time.monotonic()
        goal_a, here_a, fc = np.asarray(goal, float), np.array([x0, y0]), self.field_cache
        if fc is not None and t0 - fc[0] < p['field_max_age'] \
                and np.linalg.norm(goal_a - fc[1]) < p['field_reuse_dist'] \
                and np.linalg.norm(here_a - fc[2]) < p['field_reuse_dist']:
            field, origin = fc[3], fc[4]
            self.dbg['field_reused'] = True
        else:
            field, origin = self.field(goal_a, here_a, obs)
            self.field_cache = (t0, goal_a, here_a, field, origin)
            self.dbg['field_ms'] = (time.monotonic() - t0) * 1000
        # 지금 로봇 자리에서 goal까지 장애물을 돌아가는 거리 [m] → SEARCH 이동의 진척 판정 (못 가면 약 50)
        self.path_dist = float(self.sample(field, origin, np.array([x0]), np.array([y0]))[0])
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
        off = np.abs(wrap(np.arctan2(face[1] - ey, face[0] - ex) - eth))   # 후보별 도착 때 사람 쪽과 어긋난 각도 [rad]
        if math.hypot(goal[0] - x0, goal[1] - y0) < 0.3:   # 거의 도착 → 관람객 쪽을 바라봄
            cost += p['w_heading'] * off
        if keep_view:                                       # 추종 중: 사람이 탑캠 화면 밖으로 나가는 궤적에 벌점
            # + w_view × max(어긋난 각도 - view_half_angle, 0)  (2026-11-20: 이동 중 화면 밖으로 빠져 인식을 놓침)
            cost += p['w_view'] * np.clip(off - p['view_half_angle'], 0, None)
        cost[~safe] = np.inf
        # 출력: i  벌점이 가장 작은 후보 번호 (0 ~ 104). 위험한 후보는 cost = inf라 절대 안 뽑힘
        # 결과: (v [m/s], w [rad/s])  예) (0.25, 0.3) → drive()가 cmd_vel로 보냄. 다음 0.1 s만 쓰고 다시 계산
        i = int(np.argmin(cost))
        return float(V[i]), float(W[i])

    def field(self, goal, here, obs):
        """격자 칸마다 goal까지 장애물을 돌아가는 거리 [m] (dwa의 벌점용).
        grid_resolution(5 cm) 격자 → 장애물 칸을 로봇 반지름+여유만큼 부풀림 → 목표에서 최단 거리(grid_distances, SciPy 다익스트라).
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
        # 올림 전에 아주 작은 값을 빼서 소수점 오차로 한 칸 더 부풀지 않게 ((0.28 + 0.02) / 0.1 = 3.0000000000000004 → 4칸이
        # 되어 작품 4 쪽 통로가 막히고 도크 복귀가 60 s 동안 제자리, 2026-11-20)
        k = int(math.ceil((p['robot_radius'] + p['safety_margin']) / res - 1e-6))
        raw, grid = grid, cv2.dilate(grid, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1)))
        gx, gy = np.clip(np.floor((goal - origin) / res).astype(int), 0, n - 1)   # 반올림으로 격자 밖에 나가지 않게
        rx, ry = np.clip(np.floor((here - origin) / res).astype(int), 0, n - 1)
        g = max(1, int(round(0.1 / res)))           # 목표 자리는 ±10 cm 열어 둔다 (격자 크기와 상관없이)
        grid[max(gy - g, 0):gy + g + 1, max(gx - g, 0):gx + g + 1] = 0
        win = np.s_[max(ry - k, 0):ry + k + 1, max(rx - k, 0):rx + k + 1]
        grid[win] = raw[win]                        # 로봇 주변은 실제 장애물 칸만 막음 (붙어 있어도 출발 가능)
        dist = grid_distances(grid == 0, int(gy), int(gx))   # 목표에서 모든 칸까지 돌아가는 거리 [칸]
        dist *= res
        dist[~np.isfinite(dist)] = 50.0
        return dist, origin

    def clearance_grid(self, near, x0, y0, reach):
        """로봇 주변(±reach) 칸마다 '가장 가까운 장애물 점까지 거리' [m] (dwa의 안전 판정·벌점용).
        clear_resolution(1 cm) 격자에 장애물 점을 찍고 cv2.distanceTransform 한 번.
        격자 오차 = 장애물 점이 칸 안 어디든 있을 수 있음(최대 res × 0.71) + 칸 사이를 섞어 읽음(bilinear, 최대 res × 0.71)
        → res × 1.42를 빼서 실제보다 멀게 나오지 않게 한다
        (안전 판정이 예전 정확한 계산보다 느슨해지지 않음. 대신 최대 약 1.4 cm 보수적).
        받음: near 장애물 점 (M, 2) odom (로봇에서 reach 안), x0, y0 로봇 위치, reach 반경 [m]
        돌려줌: (거리 지도 n×n [m], 지도 원점 (x, y))"""
        res = self.p['clear_resolution']
        n = int(math.ceil(2 * reach / res)) + 2
        origin = np.array([x0 - n * res / 2, y0 - n * res / 2])
        img = np.ones((n, n), np.uint8)             # 1 = 빈칸, 0 = 장애물 (distanceTransform은 0까지 거리를 잰다)
        c = np.floor((near - origin) / res).astype(int)
        c = c[((c >= 0) & (c < n)).all(1)]
        img[c[:, 1], c[:, 0]] = 0
        dist = cv2.distanceTransform(img, cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * res
        return np.maximum(dist - res * 1.42, 0.0), origin

    def sample(self, field, origin, x, y, res=None):
        """거리 지도에서 점들의 값을 읽는다. 칸 사이는 주변 4칸을 섞어서(bilinear) → 후보끼리 점수가 같아지지 않게.
        받음: field 거리 지도, origin 지도 원점, x, y 점들 (배열), res 칸 크기 (없으면 grid_resolution)
        돌려줌: 점마다 거리 [m]"""
        n = field.shape[0]
        g = (np.stack([x, y], 1) - origin) / (res or self.p['grid_resolution']) - 0.5
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

    def go_to(self, pose, goal):
        """정해진 지점(odom)으로 이동 (SEARCH에서 마지막 위치로 갈 때).
        목표 방향이 turn_first_angle(약 57°)보다 틀어져 있으면 먼저 제자리 회전, 아니면 DWA.
        DWA는 후진이 없어 목표가 뒤쪽이면 모든 후보가 동점 → (0, 0)으로 멈추기 때문.
        받음: pose (x, y, yaw) odom, goal (x, y) odom
        내보냄: drive(...)"""
        err = float(wrap(math.atan2(goal[1] - pose[1], goal[0] - pose[0]) - pose[2]))
        if abs(err) > self.p['turn_first_angle']:
            self.dbg['turn_first'] = True
            w = float(np.clip(1.5 * err, -self.p['max_angular'], self.p['max_angular']))
            return self.drive(*self.ramp(0.0, w))
        return self.drive(*self.dwa(pose, goal, goal))

    def predict_yaw(self, yaw, age, now):
        """늦게 온 odom yaw에 아직 반영되지 않은 회전 명령을 더해 지금 yaw를 추정한다 (Wi-Fi 지연 보정).
        cmd_vel도 로봇까지 늦게 가므로(오는 지연 ≈ 가는 지연으로 봄) 최근 2×age 초 동안 보낸 명령이
        odom에 아직 안 들어 있다고 보고 Σ w·dt 를 더한다. 명령 기록은 drive()가 남김 (최근 약 6 s).
        받음: yaw  odom이 준 값 [rad], age  그 값이 몇 초 전 것인지 [s], now
        돌려줌: 추정한 지금 yaw [rad]"""
        if age <= 0.0 or not np.isfinite(age):
            return yaw
        since, add = now - 2.0 * age, 0.0
        hist = list(self.cmd_hist)
        for (t0, _, w), nxt in zip(hist, hist[1:] + [(now, 0.0, 0.0)]):
            a, b = max(t0, since), min(nxt[0], now)
            if b > a:
                add += w * (b - a)
        return yaw + add

    def drive(self, v, w):
        """속도 명령을 로봇에 보낸다. enabled가 false면 보내지 않음. 보낸 값을 self.cmd에 기억 (ramp·dwa가 '지금 속도'로 씀).
        서 있을 때(v≈0) min_spin_cmd보다 작은 회전은 0으로: Create 3는 아주 작은 제자리 회전을 0.25~0.3 rad/s로 돌아 버림.
        받음: v  전진 속도 [m/s], w  회전 속도 [rad/s] (+ 왼쪽)
        내보냄: cmd_vel (TwistStamped: twist.linear.x = v, twist.angular.z = w)"""
        if abs(v) < 0.01 and abs(w) < self.p['min_spin_cmd']:
            w = 0.0
        self.cmd = (v, w)
        self.cmd_hist.append((self.now(), v, w))    # odom 지연 보정용 (predict_yaw)
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
