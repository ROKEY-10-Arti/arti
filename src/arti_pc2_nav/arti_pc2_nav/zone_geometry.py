"""반원 지오메트리 유틸 (RViz 표시용).

키오스크·작품 반원의 호 위 점을 만든다. 출구는 반원이 아니라 arti_common 공통 직사각형이다.
"""
import math
from .guided_tour_plan import Semicircle  # noqa: F401 (타입 설명용)


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
