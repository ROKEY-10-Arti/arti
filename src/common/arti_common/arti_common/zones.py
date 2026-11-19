"""작품·안내 구역 (arti_common/config/artworks.yaml) 읽기와 안/밖 판정 — 두 로봇 공통.

robot1: arti_follow artwork_zone(관람객 위치로 설명·안내 판정), zone_viz(RViz 표시)
robot4: arti_pc2_nav (로봇 위치로 작품 도착 판정) — 같은 파일·같은 판정을 쓰면 두 로봇의 구역이 어긋나지 않는다.

  arts, guides = load_zones(default_zones_file())   # 작품 구역들, 안내 구역들 (출구·키오스크)
  zone.contains((x, y))                            # map 좌표 점이 구역 안인가
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
import yaml


def default_zones_file():
    """설치된 share/arti_common/config/artworks.yaml 경로."""
    return os.path.join(get_package_share_directory('arti_common'), 'config', 'artworks.yaml')


class Zone:
    """구역 하나. shape: semicircle(기본) 또는 rect.

    semicircle  x, y(중심), facing_deg(정면, 방 쪽), radius
    rect        x_min, x_max, y_min, y_max (map 축에 나란한 직사각형). 중심 = 가운데, 정면 없음
    """

    def __init__(self, zid, row):
        self.id, self.name = zid, row.get('name', str(zid))
        self.shape = row.get('shape', 'semicircle')
        self.guide = row.get('guide')
        if self.shape == 'rect':
            self.box = tuple(float(row[k]) for k in ('x_min', 'x_max', 'y_min', 'y_max'))
            self.x, self.y = (self.box[0] + self.box[1]) / 2, (self.box[2] + self.box[3]) / 2
            self.r, self.fx, self.fy = None, 0.0, 0.0
            return
        self.x, self.y, self.r = float(row['x']), float(row['y']), float(row['radius'])
        a = math.radians(float(row['facing_deg']))
        self.fx, self.fy = math.cos(a), math.sin(a)

    def dist(self, p):
        return math.hypot(p[0] - self.x, p[1] - self.y)

    def contains(self, p):
        if self.shape == 'rect':
            x0, x1, y0, y1 = self.box
            return x0 <= p[0] <= x1 and y0 <= p[1] <= y1
        dx, dy = p[0] - self.x, p[1] - self.y
        return math.hypot(dx, dy) <= self.r and dx * self.fx + dy * self.fy >= 0.0


def load_zones(path=None):
    """artworks.yaml → (작품 구역 목록, 안내 구역 목록). 작품 id = 작품 번호(int), 안내 구역 id = zone_id(str)."""
    with open(path or default_zones_file(), encoding='utf-8') as f:
        data = yaml.safe_load(f) or {}
    arts = [Zone(int(r['artwork_id']), r) for r in data.get('artworks') or []]   # 작품 번호 (int)
    guides = [Zone(str(r['zone_id']), r) for r in data.get('guide_zones') or []]
    return arts, guides
