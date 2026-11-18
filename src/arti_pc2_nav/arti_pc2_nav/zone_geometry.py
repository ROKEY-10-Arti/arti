"""RViz 표시용 반원 지오메트리 (출구 반원 끝점은 2026-11-17 갱신).

주의: 출구 반원은 RViz 마커 표시용입니다. 실제 코스 주행의 출구 도착 판정은
반원이 아니라 a_course.yaml의 exit.goal_pose 기준 0.30m 거리로 합니다.

두 점 A, B는 반원의 중심/반지름이 아니라 '지름(밑변)'의 양 끝점입니다.
갤러리 내부(-X 쪽)를 명시적으로 선택하므로, 두 점의 입력 순서로
방향을 추측하지 않습니다.
"""
import math
from .guided_tour_plan import Semicircle

# 출구 반원의 지름 양 끝점 (map 좌표, m)
EXIT_DIAMETER_A = (0.167, 1.95)
EXIT_DIAMETER_B = (0.147, 2.28)


def exit_semicircle() -> Semicircle:
    """지름 끝점 A, B로부터 출구 반원(중심, 반지름, 방향)을 계산."""
    ax, ay = EXIT_DIAMETER_A
    bx, by = EXIT_DIAMETER_B
    cx, cy = (ax + bx) / 2.0, (ay + by) / 2.0     # 중심 = 지름의 중점
    dx, dy = bx - ax, by - ay
    radius = math.hypot(dx, dy) / 2.0             # 반지름 = 지름 길이 / 2
    # 지름 A->B의 왼쪽 법선 = (-dy, dx), 대체로 -X 방향.
    # 즉 원래 의도대로 갤러리 내부(-X) 반평면을 유지합니다.
    direction = math.atan2(dx, -dy)
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
