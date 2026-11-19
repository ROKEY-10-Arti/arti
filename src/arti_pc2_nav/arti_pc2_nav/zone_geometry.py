"""반원 지오메트리 유틸 (RViz 표시용).

출구 반원(RViz 표시용)의 지름 끝점은 config/a_course.yaml의 exit.marker_diameter에 둔다 (코드에 좌표 없음).
실제 출구 도착 판정은 반원이 아니라 exit.goal_pose 기준 0.30m 거리로 한다.

두 점 A, B는 반원의 중심/반지름이 아니라 '지름(밑변)'의 양 끝점입니다.
반원은 A->B의 왼쪽(갤러리 안쪽)으로 열리므로 YAML에 적는 순서가 방향을 정합니다.
"""
import math
from .guided_tour_plan import Semicircle


def semicircle_from_diameter(a, b) -> Semicircle:
    """지름 끝점 A, B (map 좌표 (x, y))로 반원(중심, 반지름, 방향)을 계산. 방향 = A->B의 왼쪽 법선."""
    ax, ay = a
    bx, by = b
    cx, cy = (ax + bx) / 2.0, (ay + by) / 2.0     # 중심 = 지름의 중점
    dx, dy = bx - ax, by - ay
    radius = math.hypot(dx, dy) / 2.0             # 반지름 = 지름 길이 / 2
    if radius <= 0:
        raise ValueError('출구 지름 끝점 A, B가 같습니다')
    direction = math.atan2(dx, -dy)               # 왼쪽 법선 (-dy, dx)의 각도
    return Semicircle(cx, cy, radius, direction)


def boundary_points(zone: Semicircle, steps: int = 40):
    """반원의 호(arc) 위 점 목록 (steps+1개). 영역을 닫으려면 첫 점과 끝 점을 이으면 됩니다."""
    if steps < 2:
        raise ValueError('steps must be >= 2')
    # direction-90° 부터 direction+90° 까지 180°를 steps 등분
    return [
        (zone.cx + zone.radius * math.cos(zone.direction - math.pi/2 + math.pi*i/steps),
         zone.cy + zone.radius * math.sin(zone.direction - math.pi/2 + math.pi*i/steps))
        for i in range(steps + 1)
    ]
