"""작품 구역 판정 → 작품 설명·안내 음성 (T32 정지·구역 판정·설명).

입력  target_person/point_map (PointStamped, frame_id map)  인식 모듈이 준 관람객 좌표
      artwork_zone/reset (std_msgs/Empty)                   '이미 설명한 작품' 목록 비우기 (새 관람객)
출력  explain_artwork (arti_interfaces/action/ExplainArtwork)  액션 클라이언트 → arti_audio guide_audio_node
      guide_message (std_msgs/String)                         안내 구역(출구·키오스크) 안내 키
      artwork_zone/state (std_msgs/String)                    지금 판단 (바뀔 때만): idle, dwell art_01 3.2s, explaining art_01 ...

구역은 arti_common/config/artworks.yaml (map 좌표 반원). 코드에 좌표를 두지 않는다.

작품 구역
  관람객이 dwell_time(5 s) 동안 dwell_radius(0.6 m) 안에 머무름 (is_dwelling)
  → 그 위치가 작품 반원 안이면 (여럿이면 중심이 가장 가까운 작품, find_artwork_zone)
  → 이미 끝까지 설명한 작품이 아니면 explain_artwork goal
  설명 중 처음 위치에서 leave_distance(1 m) 넘게 벗어나거나 lost_timeout(1 s) 동안 좌표가 없으면 취소
안내 구역
  들어가면 바로 (멈추지 않아도) guide_message 한 번. 나갔다 다시 들어와도 guide_rearm_time 안에는 다시 안 함
"""
import collections
import math
import os
import signal

from ament_index_python.packages import get_package_share_directory
from arti_interfaces.action import ExplainArtwork
from geometry_msgs.msg import PointStamped
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty, String
import yaml

PARAMS = {
    'zones_file': '',              # '' = 설치된 share/arti_common/config/artworks.yaml
    'map_frame': 'map',
    'dwell_time': 5.0,             # 이만큼 머물러야 설명 [s] (T32)
    'dwell_radius': 0.6,           # 머무름 판정 반경 [m] (T32)
    'leave_distance': 1.0,         # 설명 중 이만큼 벗어나면 취소 [m] (T32)
    'lost_timeout': 1.0,           # 좌표가 이만큼 안 오면 머무름 초기화·설명 취소 [s]
    'guide_rearm_time': 30.0,      # 같은 안내 구역 안내를 다시 할 수 있는 간격 [s]
    'check_period': 0.2,           # 판정 주기 [s]
}


class Zone:
    """반원 구역: 중심에서 정면(facing) 쪽으로 반지름 radius."""

    def __init__(self, zid, row):
        self.id, self.name = zid, row.get('name', zid)
        self.x, self.y, self.r = float(row['x']), float(row['y']), float(row['radius'])
        a = math.radians(float(row['facing_deg']))
        self.fx, self.fy = math.cos(a), math.sin(a)
        self.guide = row.get('guide')

    def dist(self, p):
        return math.hypot(p[0] - self.x, p[1] - self.y)

    def contains(self, p):
        dx, dy = p[0] - self.x, p[1] - self.y
        return math.hypot(dx, dy) <= self.r and dx * self.fx + dy * self.fy >= 0.0


def load_zones(path):
    with open(path, encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    arts = [Zone(str(r['artwork_id']), r) for r in data.get('artworks') or []]
    guides = [Zone(str(r['zone_id']), r) for r in data.get('guide_zones') or []]
    return arts, guides


class ArtworkZone(Node):

    def __init__(self):
        super().__init__('artwork_zone')
        self.p = {q.name: q.value for q in self.declare_parameters('', list(PARAMS.items()))}
        path = self.p['zones_file'] or os.path.join(
            get_package_share_directory('arti_common'), 'config', 'artworks.yaml')
        self.arts, self.guides = load_zones(path)
        self.get_logger().info(
            f'구역 {path}: 작품 {[z.id for z in self.arts]}, 안내 {[(z.id, z.guide) for z in self.guides]}')

        self.track = collections.deque()       # (시각, (x, y)) 최근 관람객 좌표
        self.seen = -1e9
        self.explained = set()                 # 끝까지 설명한 작품 (중복 설명 안 함)
        self.active = None                     # 설명 중: {'id', 'anchor', 'handle', 'why'}
        self.inside = set()                    # 지금 들어가 있는 안내 구역
        self.guided_at = {}                    # 안내 구역별 마지막 안내 시각
        self.state = None
        self.warned_frame = False

        self.explain = ActionClient(self, ExplainArtwork, 'explain_artwork')
        self.guide_pub = self.create_publisher(String, 'guide_message', 10)
        self.state_pub = self.create_publisher(String, 'artwork_zone/state', 10)
        self.create_subscription(PointStamped, 'target_person/point_map', self.on_person, 10)
        self.create_subscription(Empty, 'artwork_zone/reset', self.on_reset, 10)
        self.create_timer(self.p['check_period'], self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_person(self, msg):
        if msg.header.frame_id != self.p['map_frame']:
            if not self.warned_frame:
                self.get_logger().warn(f'좌표 frame이 {msg.header.frame_id} → {self.p["map_frame"]}만 씀')
                self.warned_frame = True
            return
        now = self.now()
        self.track.append((now, (msg.point.x, msg.point.y)))
        while self.track and self.track[0][0] < now - self.p['dwell_time'] - 1.0:
            self.track.popleft()
        self.seen = now

    def on_reset(self, _msg):
        self.get_logger().info(f'설명한 작품 목록 비움 (전에는 {sorted(self.explained)})')
        self.explained.clear()

    # ---------- 판정 ----------

    def tick(self):
        now = self.now()
        if now - self.seen > self.p['lost_timeout']:
            self.track.clear()
            self.inside.clear()
            if self.active:
                self.cancel('좌표 끊김')
            return self.set_state('lost' if self.seen > 0 else 'idle')
        if not self.track:                      # 설명이 막 끝나 기록을 비움 → 다음 좌표 기다림
            return
        p = self.track[-1][1]

        self.check_guides(p, now)

        if self.active:                         # 설명 중 → 벗어났는지만 본다
            a = self.active
            if a['handle'] is not None and math.dist(p, a['anchor']) > self.p['leave_distance']:
                self.cancel(f'{self.p["leave_distance"]} m 넘게 벗어남')
            return self.set_state(f'explaining {a["id"]}')

        dwell = self.dwelling(now, p)
        if dwell is None:
            return self.set_state('moving')
        zone = self.find_artwork_zone(dwell)
        if zone is None:
            return self.set_state('dwell (작품 구역 밖)')
        if zone.id in self.explained:
            return self.set_state(f'dwell {zone.id} (이미 설명함)')
        self.start(zone, dwell)

    def dwelling(self, now, p):
        """dwell_time 동안 dwell_radius 안에 있었으면 그 평균 위치, 아니면 None (is_dwelling)."""
        if not self.track or self.track[0][0] > now - self.p['dwell_time']:
            return None                         # 아직 dwell_time만큼 못 봄
        pts = [q for t, q in self.track if t >= now - self.p['dwell_time']]
        if any(math.dist(q, p) > self.p['dwell_radius'] for q in pts):
            return None
        return (sum(q[0] for q in pts) / len(pts), sum(q[1] for q in pts) / len(pts))

    def find_artwork_zone(self, p):
        """p가 들어 있는 작품 반원 중 중심이 가장 가까운 것."""
        inside = [z for z in self.arts if z.contains(p)]
        return min(inside, key=lambda z: z.dist(p)) if inside else None

    def check_guides(self, p, now):
        for z in self.guides:
            if not z.contains(p):
                self.inside.discard(z.id)
                continue
            if z.id in self.inside:
                continue
            self.inside.add(z.id)
            if now - self.guided_at.get(z.id, -1e9) < self.p['guide_rearm_time']:
                continue
            self.guided_at[z.id] = now
            self.get_logger().info(f'안내 구역 {z.name}({z.id}) 진입 → guide_message "{z.guide}"')
            self.guide_pub.publish(String(data=z.guide))

    # ---------- 설명 액션 ----------

    def start(self, zone, anchor):
        if not self.explain.server_is_ready():
            return self.set_state(f'dwell {zone.id} (explain_artwork 서버 없음)')
        self.get_logger().info(f'{zone.name}({zone.id}) 앞에서 멈춤 → 설명 시작')
        self.active = {'id': zone.id, 'anchor': anchor, 'handle': None, 'why': None}
        goal = ExplainArtwork.Goal(artwork_id=zone.id)
        self.explain.send_goal_async(goal).add_done_callback(self.on_goal)
        self.set_state(f'explaining {zone.id}')

    def on_goal(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'설명 goal 거절됨: {self.active["id"]}')
            self.active = None
            return
        self.active['handle'] = handle
        if self.active['why']:                  # 수락되기 전에 취소 요청이 있었음
            handle.cancel_goal_async()
        handle.get_result_async().add_done_callback(self.on_result)

    def on_result(self, future):
        res = future.result().result
        a, self.active = self.active, None
        why = f' ({a["why"]})' if a['why'] else ''
        self.get_logger().info(f'설명 끝 {a["id"]}: completed={res.completed}, reason={res.reason}{why}')
        if res.completed:
            self.explained.add(a['id'])
        self.track.clear()                      # 다시 dwell_time 동안 머물러야 다음 설명

    def cancel(self, why):
        a = self.active
        if a['why']:
            return
        a['why'] = why
        self.get_logger().info(f'설명 취소 {a["id"]}: {why}')
        if a['handle'] is not None:
            a['handle'].cancel_goal_async()

    def set_state(self, text):
        if text != self.state:
            self.state = text
            self.state_pub.publish(String(data=text))


def main():
    rclpy.init()
    node = ArtworkZone()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)   # launch가 SIGINT를 한 번 더 보내도 정리는 끝까지
        if node.active and node.active['handle'] is not None:
            node.active['handle'].cancel_goal_async()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
