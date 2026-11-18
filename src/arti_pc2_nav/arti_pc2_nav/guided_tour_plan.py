"""PC2 코스 계획(데이터 구조) + 반원 지오메트리 + YAML/요청 파서.

ROS에 의존하지 않으므로 단독으로 테스트할 수 있습니다(test_full_course.py).

- 반원 안/밖 판정은 '관람객'이 아니라 '로봇 위치' 기준입니다.
- 좌표는 절대 임의로 만들지 않습니다. YAML(config/a_course.yaml)에
  실측한 map 좌표가 반드시 들어 있어야 하며, 없으면 즉시 예외로 막습니다.
"""
from __future__ import annotations
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Optional
import json
import math
import uuid
import yaml


def number(value: Any, name: str) -> float:
    """YAML 값이 유한한 숫자인지 검사해서 float로 반환 (bool/문자열/NaN 거부)."""
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValueError(f"{name}: 숫자 좌표가 필요합니다: {value!r}")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name}: NaN/Inf는 사용할 수 없습니다")
    return result


# ---------------------------------------------------------------------------
# 좌표 / 영역 자료형
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class Pose2D:
    """map 좌표계의 2D 자세 (x, y [m], yaw [rad])."""
    x: float
    y: float
    yaw: float = 0.0  # radian, map frame

    @classmethod
    def parse(cls, data: Any) -> 'Pose2D':
        """YAML의 {x, y, yaw} 딕셔너리를 Pose2D로 변환 (yaw 생략 시 0)."""
        if not isinstance(data, dict):
            raise ValueError('goal_pose: {x, y, yaw} 딕셔너리가 필요합니다')
        return cls(number(data.get('x'), 'goal_pose.x'),
                   number(data.get('y'), 'goal_pose.y'),
                   number(data.get('yaw', 0), 'goal_pose.yaw'))


@dataclass(frozen=True)
class Semicircle:
    """작품/키오스크 앞의 반원 영역.

    (cx, cy) 중심, radius 반지름, direction 방향(작품 정면 쪽) 반평면만 포함.
    로봇이 이 반원 안에 들어오면 '도착'으로 판정합니다.
    """
    cx: float
    cy: float
    radius: float
    direction: float  # 정면을 가리키는 각도(rad). cos/sin 방향 반평면만 허용.

    @classmethod
    def parse(cls, data: Any) -> 'Semicircle':
        """YAML의 {cx, cy, radius, direction} 딕셔너리를 Semicircle로 변환."""
        if not isinstance(data, dict):
            raise ValueError('zone: {cx, cy, radius, direction} 딕셔너리가 필요합니다')
        obj = cls(number(data.get('cx'), 'zone.cx'),
                  number(data.get('cy'), 'zone.cy'),
                  number(data.get('radius'), 'zone.radius'),
                  number(data.get('direction'), 'zone.direction'))
        if obj.radius <= 0:
            raise ValueError('반원의 반지름은 양수여야 합니다')
        return obj

    def contains(self, location: Pose2D | tuple[float, float], epsilon: float = 1e-7) -> bool:
        """점이 반원 안에 있는지 판정.

        조건 1) 중심까지 거리 <= 반지름 (원 안)
        조건 2) 중심->점 벡터와 direction 벡터의 내적 >= 0 (정면 쪽 반평면)
        """
        if isinstance(location, Pose2D):
            x, y = location.x, location.y
        else:
            x, y = location
        dx, dy = x - self.cx, y - self.cy
        return (dx * dx + dy * dy <= (self.radius + epsilon) ** 2
                and dx * math.cos(self.direction) + dy * math.sin(self.direction) >= -epsilon)


@dataclass(frozen=True)
class Artwork:
    """작품 1개: ID, Nav2 목표 자세, 도착 판정 반원, 해설 ID."""
    artwork_id: str
    goal: Pose2D
    zone: Semicircle
    explanation_id: str


@dataclass(frozen=True)
class CoursePlan:
    """전체 코스 계획: 키오스크 -> 작품들 -> (출구) -> (충전 스테이션 도킹)."""
    kiosk_goal: Pose2D
    kiosk_zone: Semicircle
    artworks: tuple[Artwork, ...]
    # [실물 로봇 추가] 출구 정지점(반원 없이 거리로 판정)과 고정 도킹 접근 위치.
    # YAML에 exit/dock 섹션이 없으면 None -> 기존 시뮬레이션 동작(출구 생략,
    # 언도킹 직후 위치로 복귀)과 동일하게 동작합니다.
    exit_goal: Optional[Pose2D] = None          # 작품 4 다음 들르는 출구 정지점
    farewell_text: str = '안녕히가세요'          # 출구 도착 시 TTS 인사말
    dock_station: Optional[Pose2D] = None        # 충전 스테이션 실측 위치(참고용)
    dock_approach: Optional[Pose2D] = None       # Nav2로 가는 스테이션 정면 접근점

    def selected(self, ids: Optional[tuple[str, ...]]) -> 'CoursePlan':
        """선택 관람(B 코스): 지정한 작품만 지정 순서로 남긴 새 계획을 반환.

        ids가 None이면 전체 관람(A 코스)이므로 그대로 반환.
        dataclasses.replace를 써서 exit/dock 필드는 그대로 유지합니다.
        """
        if ids is None:
            return self
        if not ids:
            raise ValueError('선택 작품 목록이 비었습니다')
        by_id = {art.artwork_id: art for art in self.artworks}
        if len(set(ids)) != len(ids):
            raise ValueError('선택 작품 목록에 중복된 ID가 있습니다')
        missing = [a for a in ids if a not in by_id]
        if missing:
            raise ValueError(f'등록되지 않은 작품 ID: {missing}')
        return replace(self, artworks=tuple(by_id[id_] for id_ in ids))


@dataclass(frozen=True)
class CourseRequest:
    """PC3(키오스크)에서 들어온 코스 요청."""
    request_id: str                          # 요청 추적용 ID (A/B 문자열이면 uuid 자동 생성)
    all_artworks: bool                       # True = 전체 관람(A)
    artwork_ids: Optional[tuple[str, ...]]   # 선택 관람(B) 작품 목록, A이면 None


# ---------------------------------------------------------------------------
# 파서
# ---------------------------------------------------------------------------

def parse_course_request(raw: str) -> CourseRequest:
    """코스 요청 문자열 파싱. PC3 문자열 A, B:1,3 또는 JSON 계약 모두 허용.

    예)  'A'                       -> 전체 관람
         'B:1,3'                   -> 1, 3번만 관람
         '{"mode":"COURSE","request_id":"r1","all_artworks":true}'
    """
    raw = raw.strip()
    if not raw:
        raise ValueError('코스 요청이 비었습니다')
    # --- 간단 문자열 형식 ---
    if raw.upper() in {'A', 'ALL'}:
        return CourseRequest(str(uuid.uuid4()), True, None)
    if raw.upper().startswith('B:'):
        ids = tuple(part.strip() for part in raw[2:].split(','))
        if not ids or any(not t for t in ids):
            raise ValueError('B:1,3 형식으로 선택 작품을 입력하세요')
        return CourseRequest(str(uuid.uuid4()), False, ids)
    # --- JSON 형식 ---
    try:
        data = json.loads(raw)
    except (ValueError, TypeError) as error:
        raise ValueError('요청은 A, B:1,3 또는 JSON이어야 합니다') from error
    if not isinstance(data, dict):
        raise ValueError('JSON 요청은 객체여야 합니다')
    if data.get('mode') != 'COURSE':
        raise ValueError('PC2는 mode=COURSE만 처리합니다')
    rid = data.get('request_id')
    if not isinstance(rid, str) or not rid.strip() or len(rid) > 128:
        raise ValueError('request_id는 1~128자 문자열이어야 합니다')
    all_ = data.get('all_artworks')
    if not isinstance(all_, bool):
        raise ValueError('all_artworks는 true/false이어야 합니다')
    ids = data.get('artwork_ids', [])
    if not isinstance(ids, list):
        raise ValueError('artwork_ids는 배열이어야 합니다')
    if any(not isinstance(x, (str, int)) or isinstance(x, bool) for x in ids):
        raise ValueError('artwork_ids에는 문자열 또는 정수만 허용합니다')
    normalized = tuple(str(x).strip() for x in ids)
    if not all_ and (not normalized or any(not x for x in normalized)):
        raise ValueError('선택 코스는 1개 이상의 작품 ID를 포함해야 합니다')
    if all_ and normalized:
        raise ValueError('전체 관람은 artwork_ids를 비워야 합니다')
    return CourseRequest(rid, all_, None if all_ else normalized)


def load_course_config(path: str | Path) -> CoursePlan:
    """config/a_course.yaml을 읽어 CoursePlan 생성.

    누락된 좌표는 즉시 실패시켜 로봇의 잘못된 이동을 방지.
    goal_pose가 자기 반원 밖에 있으면 '도착 판정 불가'이므로 역시 거부합니다.
    """
    data = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('YAML 최상위 값은 객체여야 합니다')

    # --- 키오스크 (코스 시작 지점) ---
    kiosk = data.get('kiosk')
    if not isinstance(kiosk, dict):
        raise ValueError('kiosk 설정이 없습니다')
    kiosk_goal = Pose2D.parse(kiosk.get('goal_pose'))
    kiosk_zone = Semicircle.parse(kiosk.get('zone'))
    if not kiosk_zone.contains(kiosk_goal):
        raise ValueError('키오스크 goal_pose가 키오스크 반원 바깥입니다')

    # --- 작품 목록 (YAML 순서 = 방문 순서) ---
    items = data.get('artworks')
    if not isinstance(items, list) or not items:
        raise ValueError('artworks는 하나 이상의 작품 목록이어야 합니다')
    works = []
    for item in items:
        if not isinstance(item, dict) or not str(item.get('artwork_id', '')).strip():
            raise ValueError('각 작품의 artwork_id가 필요합니다')
        art_id = str(item['artwork_id'])
        goal = Pose2D.parse(item.get('goal_pose'))
        zone = Semicircle.parse(item.get('zone'))
        if not zone.contains(goal):
            raise ValueError(f'{art_id}: goal_pose가 해당 작품의 반원 바깥입니다')
        works.append(Artwork(art_id, goal, zone,
                             str(item.get('explanation_id', art_id))))
    if len({a.artwork_id for a in works}) != len(works):
        raise ValueError('artwork_id가 중복되었습니다')

    # --- [실물 로봇 추가] 출구 정지점 (선택 섹션) ---
    exit_goal = None
    farewell = '안녕히가세요'
    exit_cfg = data.get('exit')
    if exit_cfg is not None:
        if not isinstance(exit_cfg, dict):
            raise ValueError('exit 설정은 객체여야 합니다')
        exit_goal = Pose2D.parse(exit_cfg.get('goal_pose'))
        farewell = str(exit_cfg.get('farewell', farewell))

    # --- [실물 로봇 추가] 충전 스테이션 / 도킹 접근점 (선택 섹션) ---
    station = approach = None
    dock_cfg = data.get('dock')
    if dock_cfg is not None:
        if not isinstance(dock_cfg, dict):
            raise ValueError('dock 설정은 객체여야 합니다')
        station = Pose2D.parse(dock_cfg.get('station'))
        approach = Pose2D.parse(dock_cfg.get('approach_pose'))
    return CoursePlan(kiosk_goal, kiosk_zone, tuple(works),
                      exit_goal, farewell, station, approach)


def resolve_selection(base: CoursePlan, request: CourseRequest) -> CoursePlan:
    """요청(A/B)에 맞게 이번 주행에 쓸 계획을 만든다."""
    return base.selected(request.artwork_ids)
