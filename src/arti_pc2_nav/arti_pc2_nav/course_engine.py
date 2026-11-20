"""ROS에 의존하지 않는 ArtI PC2 코스 상태 머신(TourMachine).

전체 흐름 (PC3 액션 2개):
  prepare_tour -> start():   UNDOCK(ROS 노드) -> KIOSK -> REGISTERING(관람객 인식 1초) -> WAIT_START
  start_tour   -> begin_tour(): 작품 1..N -> EXIT(인사) -> DOCK APPROACH -> Dock -> dock_status 확인 -> DONE
  등록 실패 / 대기 시간 초과 / 취소 / Nav2 실패 -> abort_to_home: 충전 스테이션으로 복귀(NAV_HOME -> Dock)

  start_tour(begin_tour(artwork_ids))는 WAIT_START에서 선택 작품 관람 시작:
  작품 1..N -> EXIT -> 복귀·도킹. 이동 중 관람객이 멀어지거나(far_distance) 안 보이면(lost_timeout)
  WAIT_VISITOR로 멈추고 come_here 안내 -> 가까이 오면(resume_distance) 이어서 이동,
  visitor_give_up(10초) 넘으면 register_failed 안내 후 도킹 복귀(end_reason visitor_lost).

- EXIT는 계획에 exit_goal이 있을 때만 사용합니다. 도착 판정 = arti_common 출구 직사각형 안 (없으면 목표점 0.30m).
- 관람객 인식(visitor_seen())은 등록 판정과 관람 중 거리 감시에 씀. 이동은 전부 Nav2.

설계 방식:
  이 클래스는 ROS를 직접 호출하지 않습니다. 각 메서드는 Event 리스트를 돌려주고,
  a_course_node.py의 _dispatch()가 그 Event를 실제 ROS 동작(Nav2 goal 전송,
  취소, 토픽 발행, Dock 액션)으로 바꿉니다. 그래서 ROS 없이 테스트가 가능합니다.

상태(state) 목록:
  IDLE              대기
  NAV_KIOSK         키오스크로 Nav2 이동 중
  NAV_ARTWORK       현재 작품(index)으로 Nav2 이동 중
  CANCELING         반원 진입 -> Nav2 goal 취소 결과 대기
  VERIFY_STOP       반원 안에서 완전 정지 확인 중
  FACE_KIOSK        [코스 관람 변경 ⑦] 키오스크 정지 후 등(웹캠)이 키오스크를 보게 제자리 회전 (prepare_tour)
  REGISTERING       키오스크 앞 정지 후 관람객 인식 기다림 (prepare_tour)
  WAIT_START        등록 완료, 키오스크 앞에서 start_tour 기다림 (prepare_tour 성공)
  WAIT_VISITOR      관람 중 이동하다 관람객이 멀어져 멈춤, 가까이 오길 기다림 (start_tour)
  EXPLAINING        작품 해설(TTS) 재생 중, 끝날 때까지 대기
  NAV_EXIT          출구 정지점으로 이동 중
  CANCELING_EXIT    출구 구역 진입 -> 취소 결과 대기
  VERIFY_EXIT_STOP  출구에서 정지 확인 중
  FAREWELL          '안녕히가세요' 재생 시간 대기
  NAV_HOME          충전 스테이션 접근점으로 이동 중
  CANCELING_HOME    접근점 0.30m 진입 -> 취소 결과 대기
  VERIFY_HOME_STOP  접근점에서 정지 확인 중
  DOCKING           Dock 액션 실행 중
  FACE_DOCK         [코스 관람 변경 ⑨] 도킹 접근 원에서 정지 후 정면이 충전 스테이션을 보게 제자리 회전 → Dock
  WAIT_DOCK_CONFIRM Dock 성공 후 dock_status.is_docked 확인 대기
  WAIT_RECOVERY     Nav2 이동 실패 (노드가 보고 바로 도킹 복귀 또는 ERROR로 바꿈)
  DONE / ERROR      종료
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Optional
from .guided_tour_plan import CoursePlan, Pose2D


@dataclass(frozen=True)
class Event:
    """상태 머신이 ROS 노드에게 요청하는 동작 1건.

    kind 별 value:
      status   -> 상태 JSON dict (/arti/course_status 로 발행)
      navigate -> (이름, Pose2D) : Nav2 NavigateToPose 목표 전송
      cancel   -> None           : 현재 Nav2 목표 취소
      guide    -> 안내 키 문자열  : guide_message 토픽 발행 (arti_audio guide_lines.yaml의 키)
      explain  -> 작품 번호(int)  : explain_artwork 액션으로 작품 해설 요청
      dock     -> None           : Dock 액션 실행
      spin     -> float          : 제자리 회전 [rad, +면 반시계] (Nav2 spin). 결과는 spin_result()
    """
    kind: str  # status | navigate | cancel | guide | explain | dock | spin
    value: object


class TourMachine:
    def __init__(self, plan: CoursePlan, *, stop_duration: float = 0.8,
                 stop_timeout: float = 20.0, navigation_timeout: float = 180.0,
                 explanation_timeout: float = 60.0, dock_timeout: float = 90.0,
                 confirmation_timeout: float = 12.0,
                 home_position_tolerance: float = 0.30,
                 simulate_explanation: bool = False, simulated_duration: float = 2.0,
                 exit_position_tolerance: float = 0.30,
                 farewell_duration: float = 2.0,
                 register_confirm_time: float = 1.0, register_gap: float = 0.5,
                 register_timeout: float = 60.0, ready_timeout: float = 130.0,
                 skip_register: bool = False,
                 register_guide: str = 'register', recognized_guide: str = 'recognized',
                 register_failed_guide: str = 'register_failed',
                 watch_visitor: bool = True, far_distance: float = 2.5,
                 resume_distance: float = 1.8, lost_timeout: float = 3.0,
                 visitor_give_up: float = 10.0,
                 start_guide: str = 'start', come_here_guide: str = 'come_here',
                 distance_window: float = 1.0, far_hold: float = 1.0,
                 near_hold: float = 0.5, come_here_interval: float = 8.0,
                 kiosk_face: bool = True, face_tolerance: float = 0.35, face_timeout: float = 20.0,
                 kiosk_stop_at_goal: bool = True, kiosk_face_target: str = 'plane',
                 kiosk_goal_tolerance: float = 0.30,
                 dock_face: bool = True,
                 explain_zone_margin: float = 0.15, explain_lost_grace: float = 3.0):
        # --- 설정값 (단위: 초 / m) ---
        self.base_plan = plan                      # YAML 원본 계획
        self.plan = plan                           # 이번 관람에 쓰는 계획(start_tour 작품 선택 반영)
        self.stop_duration = stop_duration         # 이 시간만큼 연속 정지해야 '정지'로 인정
        self.stop_timeout = stop_timeout           # 정지 확인 최대 대기 시간
        self.navigation_timeout = navigation_timeout   # 한 구간 Nav2 이동 최대 시간
        self.explanation_timeout = explanation_timeout # 해설 완료 응답 최대 대기
        self.dock_timeout = dock_timeout           # Dock 액션 최대 시간
        self.confirmation_timeout = confirmation_timeout  # Dock 후 is_docked 확인 최대 대기
        self.home_position_tolerance = home_position_tolerance  # 도킹 접근점 도착 허용 반경
        self.simulate_explanation = simulate_explanation   # True면 TTS 없이 2초 후 해설 완료 처리
        self.simulated_duration = simulated_duration
        self.exit_position_tolerance = exit_position_tolerance  # 출구 도착 허용 반경
        self.farewell_duration = farewell_duration  # 출구 인사 후 출발까지 대기 시간
        # --- [코스 관람 변경 ⑦] 키오스크 도착 후 방향 맞추기 (prepare_tour) ---
        # 반원에 들어서자마자 멈추므로 로봇은 키오스크 쪽(진행 방향)을 보고 선다. robot4 웹캠은 뒤를 보므로
        # 등이 키오스크(반원 중심)를 보게 제자리 회전한 뒤 등록을 시작한다.
        self.kiosk_face = kiosk_face               # False면 회전 없이 바로 등록
        self.face_tolerance = face_tolerance       # 목표 방향과 이 이하 차이면 회전 생략 [rad]
        self.face_timeout = face_timeout           # 회전 결과가 이 안에 안 오면 그대로 다음 단계(등록/Dock) [s]
        # --- [코스 관람 변경 ⑪] 키오스크: 찍은 자리(goal_pose)까지 가서 서고, 키오스크 면을 기준으로 돌기 ---
        self.kiosk_stop_at_goal = kiosk_stop_at_goal   # True: goal_pose까지 감 / False: 전처럼 반원 진입 즉시 정지
        self.kiosk_goal_tolerance = kiosk_goal_tolerance  # goal_pose 이 거리 안(반원 안)이면 도착으로 멈춤 [m]
        if kiosk_face_target not in ('plane', 'center'):
            raise ValueError(f"kiosk_face_target은 'plane' 또는 'center': {kiosk_face_target!r}")
        self.kiosk_face_target = kiosk_face_target     # 'plane': 등이 키오스크 면을 봄 / 'center': 등이 반원 중심을 봄(전)
        # --- [코스 관람 변경 ⑨] 도킹 접근 원에서 Dock 전에 방향 맞추기 ---
        # 왜: 접근 원(반경 home_position_tolerance 0.30 m)에 들어서자마자 멈추므로, 출구(북쪽)에서 내려오면
        #     로봇이 남쪽·비스듬히 서서 도크가 정면에 없다. Create 3 Dock은 앞쪽 IR로 도크 신호를 보며 다가가는데,
        #     정면에 없으면 찾다가 앞 벽에 범퍼가 닿아 2.7 s 만에 Dock 실패 → 그 자리 정지 (2026-11-20 실주행).
        # 해결: 정면이 충전 스테이션(a_course.yaml dock.station)을 보게 Nav2 spin으로 돈 뒤 Dock (키오스크 회전과 같은 방식).
        #       허용 각도·시간은 키오스크와 같은 face_tolerance / face_timeout. 회전 실패·시간 초과여도 Dock은 시도.
        self.dock_face = dock_face
        # --- [코스 관람 변경 ⑧] 해설 중 위치 감시 여유 ---
        # 반원에 들어서자마자 멈추므로 로봇은 호 바로 안쪽에 선다. 서 있어도 AMCL 위치가 몇 cm 흔들려서
        # 한 번만 밖으로 튀어도 실패하던 문제 (2026-11-20 작품 1에서 해설 0.7 s 만에 실패) → 여유를 둔다.
        self.explain_zone_margin = explain_zone_margin  # 반원 밖 이 거리까지는 안으로 봄 [m]
        self.explain_lost_grace = explain_lost_grace    # 밖이거나 위치가 끊긴 상태가 이만큼 계속돼야 실패 [s]
        self.explain_bad_since = None                   # 해설 중 '밖/끊김'이 시작된 시각
        # --- 관람객 등록 (prepare_tour) ---
        self.register_confirm_time = register_confirm_time  # 이만큼 끊김 없이 보여야 등록
        self.register_gap = register_gap           # 인식이 이보다 오래 끊기면 확인을 처음부터
        self.register_timeout = register_timeout   # 키오스크 도착 후 이 안에 등록 못 하면 실패 -> 복귀
        self.ready_timeout = ready_timeout         # 등록 후 이 안에 start_tour가 없으면 복귀
        self.skip_register = skip_register         # True면 인식 없이 바로 등록 (인식 노드 없이 시험)
        self.register_guide = register_guide       # guide_message 키: 등록 요청 / 인식 완료 / 등록 실패
        self.recognized_guide = recognized_guide
        self.register_failed_guide = register_failed_guide
        # --- 관람 중 관람객 거리 감시 (start_tour, 값은 arti_follow와 같게) ---
        self.watch_visitor = watch_visitor         # False면 감시 안 함 (인식 없이 시험)
        self.far_distance = far_distance           # 로봇-관람객 거리가 이보다 멀면 멈춤 [m]
        self.resume_distance = resume_distance     # 멈춘 뒤 이 안으로 오면 다시 출발 [m] (arti_follow 0.8+0.15)
        self.lost_timeout = lost_timeout           # 인식 좌표가 이만큼 안 오면 안 보이는 것 [s] (arti_follow)
        self.visitor_give_up = visitor_give_up     # 멈춘 뒤 이만큼 안 오면 관람 끝 -> 도킹 복귀 [s]
        self.start_guide = start_guide             # guide 키: 관람 시작 / 따라오세요
        self.come_here_guide = come_here_guide
        # 인식 위치가 튀는 것(카메라 추정·LiDAR 매칭이 번갈아 나옴) 때문에 멈춤·출발이 반복되지 않게:
        self.distance_window = distance_window     # 거리 = 최근 이 시간 동안 받은 좌표들의 거리 중앙값 [s]
        self.far_hold = far_hold                   # '멀다'가 이만큼 계속돼야 멈춤 [s]
        self.near_hold = near_hold                 # '가깝다'가 이만큼 계속돼야 다시 출발 [s]
        self.come_here_interval = come_here_interval  # '따라오세요' 음성 최소 간격 [s]
        # --- 실행 중 상태 ---
        self.state = 'IDLE'
        self.request_id = ''
        self.index = -1                    # -1 = 키오스크, 0..N-1 = 작품 순번, N = 모두 완료
        self.completed: list[int] = []     # 해설까지 끝난 작품 번호 목록
        self.dock_origin: Optional[Pose2D] = None  # robot pose before first undock (언도킹 전 도킹 위치)
        self.home_goal: Optional[Pose2D] = None    # safe Nav2 return pose after undock (복귀 목표)
        self.last_pose: Optional[Pose2D] = None    # 마지막으로 받은 신선한 AMCL 위치
        self.last_pose_at = float('-inf')
        self.still_since: Optional[float] = None   # 정지 상태가 시작된 시각
        self.state_since = 0.0                     # 현재 상태에 들어온 시각 (타임아웃 계산용)
        self.exit_pending = False                  # 출구 구간 진행 중 여부
        self.returning = False                     # 복귀(NAV_HOME 이후) 중 여부 (여기서 실패하면 ERROR)
        self.end_reason = ''                       # 관람 없이 복귀한 이유 (register_failed, canceled, not_started)
        self.fail_reason = ''                      # 마지막 ERROR 이유
        self.recover_reason = ''                   # 마지막 WAIT_RECOVERY 이유
        self.visitor_since: Optional[float] = None # 관람객이 끊김 없이 보이기 시작한 시각
        self.visitor_last = float('-inf')          # 마지막으로 관람객을 본 시각
        self.visitor_xy: Optional[tuple[float, float]] = None   # 마지막 관람객 map 좌표
        self.visitor_samples: list[tuple[float, float, float]] = []   # 최근 (시각, x, y) — 거리 중앙값용
        self.far_since: Optional[float] = None     # '멀다/안 보임'이 시작된 시각
        self.near_since: Optional[float] = None    # WAIT_VISITOR에서 '가깝다'가 시작된 시각
        self.come_here_at = float('-inf')          # 마지막 '따라오세요' 음성 시각
        self.touring = False                       # start_tour로 관람 중 (관람객 거리 감시)
        self.resume_state = ''                     # WAIT_VISITOR 끝나면 돌아갈 상태 (NAV_ARTWORK/NAV_EXIT)

    def _event(self, state: str, detail: str = '') -> Event:
        """/arti/course_status 로 나갈 상태 Event 생성."""
        return Event('status', {'request_id': self.request_id, 'state': state,
                                'detail': detail, 'completed': list(self.completed)})

    # ------------------------------------------------------------------
    # 시작 / 준비
    # ------------------------------------------------------------------

    def start(self, request_id: str, now: float, dock_origin: Pose2D) -> list[Event]:
        """[prepare_tour] 호출 시작: 키오스크로 이동 Event를 낸다. 키오스크에서 등록 후 WAIT_START로 멈춘다.

        request_id = 로그 추적용 ID, dock_origin = 언도킹 전(도킹 상태)의 로봇 위치.
        """
        if self.state not in ('IDLE', 'DONE', 'ERROR'):
            raise ValueError('이미 코스주행 중입니다')
        if not isinstance(dock_origin, Pose2D) or not all(math.isfinite(x) for x in
                                  (dock_origin.x, dock_origin.y, dock_origin.yaw)):
            raise ValueError('언도킹 출발 위치(map 좌표)가 필요합니다')
        self.plan = self.base_plan                 # 작품 선택은 begin_tour에서
        self.request_id = request_id
        self.completed = []
        self.index = -1
        self.dock_origin = dock_origin
        # YAML의 고정 도킹 접근점이 있으면 그것을 사용, 없으면(시뮬레이션 skip_undock)
        # 출발 위치를 복귀 목표로 사용.
        self.home_goal = self.plan.dock_approach or dock_origin
        self.exit_pending = False
        self.touring = False
        self.returning = False
        self.end_reason = ''
        self.fail_reason = ''
        self.state = 'NAV_KIOSK'
        self.state_since = now
        self.still_since = None
        self.last_pose = None
        self.last_pose_at = float('-inf')
        return [self._event('PREPARE_STARTED', '키오스크로 이동'),
                Event('navigate', ('kiosk', self.plan.kiosk_goal))]

    def set_post_undock_pose(self, pose: Pose2D):
        """언도킹 후, 실제로 도달 가능한 '도킹 전 접근 위치'를 저장.

        정확한 원래 도킹 지점은 Nav2가 아니라 Dock 액션이 찾아갑니다.
        (Nav2가 스테이션/장애물 안으로 경로를 짜면 안 되기 때문)
        YAML에 dock.approach_pose가 있으면 그 값이 우선합니다.
        """
        if not isinstance(pose, Pose2D) or not all(math.isfinite(x) for x in
                                                     (pose.x, pose.y, pose.yaw)):
            raise ValueError('언도킹 후 복귀 위치가 유효하지 않습니다')
        if self.state != 'NAV_KIOSK':
            raise ValueError('언도킹 완료 이전 상태에서만 접근 위치 저장 가능')
        self.home_goal = self.plan.dock_approach or pose

    # ------------------------------------------------------------------
    # 위치 판정 헬퍼
    # ------------------------------------------------------------------

    def _zone(self):
        """현재 목표 반원 (index<0 이면 키오스크, 아니면 현재 작품)."""
        return self.plan.kiosk_zone if self.index < 0 else self.plan.artworks[self.index].zone

    def _fresh_pose(self, now: float) -> bool:
        """최근 2초 이내에 받은 위치가 있는지."""
        return self.last_pose is not None and 0 <= now-self.last_pose_at < 2.0

    def _fresh_inside(self, now: float) -> bool:
        """신선한 위치가 현재 반원 안인지."""
        return self._fresh_pose(now) and self._zone().contains(self.last_pose)

    def _explain_ok(self, now: float) -> bool:
        """[코스 관람 변경 ⑧] [해설 중] 위치가 신선하고 현재 작품 반원(+ explain_zone_margin) 안인지."""
        if not self._fresh_pose(now):
            return False
        z, m = self._zone(), self.explain_zone_margin
        dx, dy = self.last_pose.x - z.cx, self.last_pose.y - z.cy
        return (math.hypot(dx, dy) <= z.radius + m
                and dx * math.cos(z.direction) + dy * math.sin(z.direction) >= -m)

    def _entry_inside(self, now: float) -> bool:
        """반원 진입 판정 (진입 시 Nav2 goal을 취소함).

        원래는 반원 안쪽으로 어느 정도 들어와야 취소하려는 의도였습니다.
        반원 호 위에 딱 걸쳐 멈추면 해설 중 AMCL 흔들림으로
        '작품 반원 이탈'이 잘못 감지될 수 있기 때문입니다.
        (현재 구현은 단순 contains 판정)
        """
        if not self._fresh_pose(now):
            return False
        zone, pose = self._zone(), self.last_pose
        return zone.contains(pose)

    def _near_kiosk_goal(self, now: float) -> bool:
        """[코스 관람 변경 ⑪] 키오스크 반원 안이고 goal_pose에서 kiosk_goal_tolerance 이내인지."""
        if not self._fresh_inside(now):
            return False
        g = self.plan.kiosk_goal
        return math.hypot(self.last_pose.x - g.x, self.last_pose.y - g.y) <= self.kiosk_goal_tolerance

    def _fresh_exit(self, now: float) -> bool:
        """출구 도착인지: 공통 출구 구역(arti_common, follow 팀과 같은 직사각형)이 있으면 그 안,
        없으면 출구 정지점 0.30m 이내."""
        goal = self.plan.exit_goal
        if not self._fresh_pose(now) or goal is None:
            return False
        if self.plan.exit_zone is not None:
            return self.plan.exit_zone.contains((self.last_pose.x, self.last_pose.y))
        return (math.hypot(self.last_pose.x-goal.x, self.last_pose.y-goal.y)
                <= self.exit_position_tolerance)

    def _go_home(self, now: float, detail: str) -> list[Event]:
        """충전 스테이션 접근점(home_goal)으로 이동 시작."""
        self.state = 'NAV_HOME'
        self.state_since = now
        self.returning = True
        return [self._event('RETURNING_HOME', detail),
                Event('navigate', ('dock_approach', self.home_goal))]

    def _fresh_home(self, now: float) -> bool:
        """도킹 접근점 0.30m 이내인지."""
        return (self._fresh_pose(now) and self.home_goal is not None and
                math.hypot(self.last_pose.x-self.home_goal.x,
                           self.last_pose.y-self.home_goal.y) <= self.home_position_tolerance)

    # ------------------------------------------------------------------
    # Nav2 이동 실패
    # ------------------------------------------------------------------

    def _recover(self, reason: str, now: float) -> list[Event]:
        """Nav2 이동 실패: goal 취소 후 WAIT_RECOVERY. 노드가 보고 도킹 복귀(abort_to_home) 또는 ERROR."""
        self.state = 'WAIT_RECOVERY'
        self.state_since = now
        self.recover_reason = reason
        return [self._event('RECOVERY_REQUIRED', reason), Event('cancel', None)]

    # ------------------------------------------------------------------
    # 주기 처리 (a_course_node가 0.1초마다 호출)
    # ------------------------------------------------------------------

    def tick(self, now: float, pose: Optional[Pose2D], pose_age: float,
             speed: Optional[float], odom_age: float) -> list[Event]:
        """현재 위치/속도로 상태를 진행시킨다.

        pose_age / odom_age = 해당 데이터가 몇 초 전 것인지.
        speed = odom 기준 max(선속도, |각속도|).
        """
        # 신선한 위치만 저장
        if pose is not None and 0 <= pose_age < 2.0:
            self.last_pose = pose
            self.last_pose_at = now-pose_age

        # (1) 이동 중: 목표 영역에 들어오면 Nav2 goal을 취소하고 정지 확인으로 넘어감
        if self.state in ('NAV_KIOSK', 'NAV_ARTWORK', 'NAV_HOME', 'NAV_EXIT'):
            # [실물 로봇] 도킹 접근점/출구 근처에서 Nav2가 미세 위치를 맞추다
            # progress checker("Failed to make progress")로 실패하는 문제가 있어,
            # 반원처럼 허용 반경 안에 들어오면 바로 취소합니다.
            # 마지막 정밀 접근은 Dock 액션이 수행합니다.
            if self.state == 'NAV_HOME' and self._fresh_home(now):
                self.state = 'CANCELING_HOME'
                self.state_since = now
                return [self._event('HOME_ZONE_ENTERED', 'Nav2 목표 취소 요청'), Event('cancel', None)]
            if self.state == 'NAV_EXIT' and self._fresh_exit(now):
                self.state = 'CANCELING_EXIT'
                self.state_since = now
                return [self._event('EXIT_ZONE_ENTERED', 'Nav2 목표 취소 요청'), Event('cancel', None)]
            # 키오스크/작품: 반원 진입 시 취소
            # [코스 관람 변경 ⑪] 키오스크는 kiosk_stop_at_goal이면 반원 진입으로 멈추지 않고 goal_pose(RViz로 찍은 자리)까지 간다
            #   → Nav2 SUCCEEDED가 오면 nav_result에서 정지 확인. (전: 반원 가장자리에서 멈춰 서는 자리·방향이 매번 달라
            #   관람 시작 때 그림 1 쪽으로 크게 휙 돎, 2026-11-20)
            kiosk_to_goal = self.state == 'NAV_KIOSK' and self.kiosk_stop_at_goal
            # 단, 찍은 자리가 벽·키오스크 가까이라 Nav2가 끝까지 못 들어가는 경우가 있어서 (2026-11-20 실주행:
            # 0.22 m 앞에서 "Failed to make progress" 반복) 반원 안 + goal_pose kiosk_goal_tolerance 이내면 도착으로 멈춘다
            if kiosk_to_goal and self._near_kiosk_goal(now):
                self.state = 'CANCELING'
                self.state_since = now
                return [self._event('KIOSK_GOAL_NEAR', f'키오스크 목표 {self.kiosk_goal_tolerance:.2f} m 이내; '
                                                       'Nav2 목표 취소 요청'), Event('cancel', None)]
            if (self.state not in ('NAV_HOME', 'NAV_EXIT') and not kiosk_to_goal
                    and self._entry_inside(now)):
                self.state = 'CANCELING'
                self.state_since = now
                return [self._event('ZONE_ENTERED', 'Nav2 목표 취소 요청'), Event('cancel', None)]
            # [start_tour] 작품·출구로 이동 중 관람객이 멀어지거나 안 보이면 멈추고 '따라오세요'
            if self.touring and self.watch_visitor and self.state in ('NAV_ARTWORK', 'NAV_EXIT'):
                why = self._visitor_problem(now)
                if not why:
                    self.far_since = None
                elif self.far_since is None:
                    self.far_since = now             # 처음 '멀다' → far_hold 동안 계속되는지 봄
                elif now-self.far_since >= self.far_hold:
                    self.far_since = None
                    self.near_since = None
                    self.resume_state = self.state
                    self.state = 'WAIT_VISITOR'
                    self.state_since = now
                    events = [self._event('VISITOR_WAIT', why), Event('cancel', None)]
                    if now-self.come_here_at >= self.come_here_interval:
                        self.come_here_at = now
                        events.append(Event('guide', self.come_here_guide))
                    return events
            if now-self.state_since >= self.navigation_timeout:
                return self._recover('Nav2 이동 시간 초과', now)

        # (2) 취소 결과 대기: 결과는 nav_result()로 들어옴. 12초 넘으면 실패
        elif self.state in ('CANCELING', 'CANCELING_HOME', 'CANCELING_EXIT'):
            if now-self.state_since > 12.0:
                return self.fail('Nav2 목표 취소 결과 시간 초과')

        # (3) 출구 인사 재생 시간이 지나면 도킹 접근점으로 출발
        elif self.state == 'FAREWELL':
            if now-self.state_since >= self.farewell_duration:
                self.exit_pending = False
                return self._go_home(now, '출구 인사 완료; 충전 스테이션 접근 위치로 이동')

        # (4) 정지 확인: 영역 안 + 속도 < 0.035 가 stop_duration(0.8초) 연속 유지돼야 통과
        elif self.state in ('VERIFY_STOP', 'VERIFY_HOME_STOP', 'VERIFY_EXIT_STOP'):
            if now-self.state_since > self.stop_timeout:
                return self.fail('목표 지역 내 정지 확인 시간 초과')
            if self.state == 'VERIFY_HOME_STOP':
                safe_location = self._fresh_home(now)
            elif self.state == 'VERIFY_EXIT_STOP':
                safe_location = self._fresh_exit(now)
            else:
                safe_location = self._fresh_inside(now)
            safe = (safe_location and speed is not None and 0 <= odom_age < 2.0 and speed < 0.035)
            if not safe:
                self.still_since = None      # 조건 깨지면 정지 타이머 리셋
                return []
            if self.still_since is None:
                self.still_since = now       # 정지 시작 시각 기록
                return []
            if now-self.still_since < self.stop_duration:
                return []                    # 아직 충분히 오래 정지하지 않음
            # --- 정지 확인 완료: 어디서 멈췄는지에 따라 다음 단계 ---
            if self.state == 'VERIFY_EXIT_STOP':
                # 출구 -> 안내 키(farewell, 기본 tour_end) 재생 (guide_message -> guide_audio_node)
                self.state = 'FAREWELL'
                self.state_since = now
                return [self._event('EXIT_ARRIVED', self.plan.farewell_text),
                        Event('guide', self.plan.farewell_text)]
            if self.state == 'VERIFY_HOME_STOP':
                # 도킹 접근점 -> (정면을 도크로 회전) -> Dock 액션
                turn = self._dock_turn()
                if turn is not None:
                    self.state = 'FACE_DOCK'
                    self.state_since = now
                    return [self._event('HOME_ARRIVED', f'충전 스테이션 접근 위치 도착; 도크 쪽으로 '
                                                        f'{math.degrees(turn):.0f}° 회전 후 Dock'),
                            Event('spin', turn)]
                return [self._event('HOME_ARRIVED', '충전 스테이션 접근 위치 도착; Dock 액션 실행')] + \
                    self._start_docking(now)
            if self.index < 0:
                # [prepare_tour] 키오스크 도착 -> (방향 맞추기) -> 관람객 등록 기다림
                turn = self._kiosk_turn()
                if turn is not None:
                    self.state = 'FACE_KIOSK'
                    self.state_since = now
                    return [self._event('KIOSK_FACING', f'{math.degrees(turn):.0f}° 회전 (웹캠을 키오스크 쪽으로)'),
                            Event('spin', turn)]
                return self._start_registering(now, '관람객 등록 시작')
            # 작품 -> 해설(TTS) 시작
            self.state = 'EXPLAINING'
            self.explain_bad_since = None
            self.state_since = now
            art = self.plan.artworks[self.index]
            return [self._event('ARTWORK_STOPPED', str(art.artwork_id)), Event('explain', art.artwork_id)]

        # (4-0) [코스 관람 변경 ⑦] 키오스크에서 회전 중: 결과는 spin_result(). 너무 오래 걸리면 그대로 등록
        elif self.state == 'FACE_KIOSK':
            if now-self.state_since >= self.face_timeout:
                return self._start_registering(now, f'회전 {self.face_timeout:.0f}초 초과; 그대로 등록 시작')

        # (4-0') [코스 관람 변경 ⑨] 도킹 접근점에서 도크 쪽으로 회전 중: 결과는 spin_result(). 너무 오래 걸리면 그대로 Dock
        elif self.state == 'FACE_DOCK':
            if now-self.state_since >= self.face_timeout:
                return ([self._event('DOCK_FACE_TIMEOUT', f'회전 {self.face_timeout:.0f}초 초과; 그대로 Dock')]
                        + self._start_docking(now))

        # (4-1) [prepare_tour] 관람객 등록: register_confirm_time 동안 끊김 없이 보이면 등록
        elif self.state == 'REGISTERING':
            seeing = (self.visitor_since is not None and now-self.visitor_last <= self.register_gap)
            if self.skip_register or (seeing and now-self.visitor_since >= self.register_confirm_time):
                self.state = 'WAIT_START'
                self.state_since = now
                detail = '인식 생략(skip_register)' if self.skip_register else '관람객 인식 완료'
                return [self._event('REGISTERED', detail), Event('guide', self.recognized_guide)]
            if now-self.state_since >= self.register_timeout:
                self.end_reason = 'register_failed'
                return ([self._event('REGISTER_FAILED', f'{self.register_timeout:.0f}초 안에 관람객 인식 못 함'),
                         Event('guide', self.register_failed_guide)]
                        + self._go_home(now, '등록 실패; 충전 스테이션으로 복귀'))

        # (4-2) [prepare_tour] 등록 후 start_tour 대기: 시간 초과면 복귀
        elif self.state == 'WAIT_START':
            if now-self.state_since >= self.ready_timeout:
                self.end_reason = 'not_started'
                return ([self._event('START_TIMEOUT', f'등록 후 {self.ready_timeout:.0f}초 동안 관람 시작 없음')]
                        + self._go_home(now, '관람 시작 없음; 충전 스테이션으로 복귀'))

        # (4-3) [start_tour] 관람객 기다림: 가까이 오면 이어서 이동, visitor_give_up 넘으면 도킹 복귀
        elif self.state == 'WAIT_VISITOR':
            if not self._visitor_near(now):
                self.near_since = None
            elif self.near_since is None:
                self.near_since = now                # 처음 '가깝다' → near_hold 동안 계속되는지 봄
            if self.near_since is not None and now-self.near_since >= self.near_hold:
                self.near_since = None
                self.state = self.resume_state
                self.state_since = now               # 이동 시간 초과는 다시 셈
                if self.state == 'NAV_EXIT':
                    target = ('exit', self.plan.exit_goal)
                else:
                    art = self.plan.artworks[self.index]
                    target = (art.artwork_id, art.goal)
                return [self._event('VISITOR_BACK', '관람객 가까이 옴; 이어서 이동'), Event('navigate', target)]
            if now-self.state_since >= self.visitor_give_up:
                self.end_reason = 'visitor_lost'
                self.exit_pending = False
                return ([self._event('VISITOR_LOST', f'{self.visitor_give_up:.0f}초 동안 관람객이 돌아오지 않음'),
                         Event('guide', self.register_failed_guide)]
                        + self._go_home(now, '관람객 분실; 충전 스테이션으로 복귀'))

        # (5) 해설 중: 반원 이탈 감시, 결과는 explanation_result()로 들어옴
        #     [코스 관람 변경 ⑧] 전: 한 번이라도 밖이면 즉시 실패 → 지금: 밖 0.15 m까지 봐주고, 3 s 넘게 계속돼야 실패
        elif self.state == 'EXPLAINING':
            if self._explain_ok(now):
                self.explain_bad_since = None
            elif self.explain_bad_since is None:
                self.explain_bad_since = now                   # 잠깐 튄 것일 수 있음 → explain_lost_grace 동안 지켜봄
            elif now-self.explain_bad_since >= self.explain_lost_grace:
                why = '위치 정보 소실' if not self._fresh_pose(now) else '현재 작품 반원 이탈'
                return self.fail(f'해설 중 {why} ({self.explain_lost_grace:.0f}초 넘게 계속)')
            if self.simulate_explanation and now-self.state_since >= self.simulated_duration:
                # simulation_mode: TTS 없이 2초 후 완료 처리
                return self.explanation_result(self.plan.artworks[self.index].artwork_id, True, now)
            if now-self.state_since >= self.explanation_timeout:
                return self.fail('해설 완료 응답 시간 초과')

        # (6) 도킹 타임아웃 감시
        elif self.state == 'DOCKING':
            if now-self.state_since >= self.dock_timeout:
                return self.fail('Dock 액션 시간 초과')
        elif self.state == 'WAIT_DOCK_CONFIRM':
            if now-self.state_since >= self.confirmation_timeout:
                return self.fail('Dock 액션 성공 이후 dock_status.is_docked 확인 실패')
        return []

    # ------------------------------------------------------------------
    # 외부 결과 콜백 (ROS 노드가 호출)
    # ------------------------------------------------------------------

    def _kiosk_turn(self) -> Optional[float]:
        """[코스 관람 변경 ⑦] 등(웹캠)이 키오스크(반원 중심)를 보게 돌 각도 [rad, -π~π]. 필요 없거나 끔/위치 모름이면 None.
        목표 방향 = 키오스크 → 로봇 방향 (멈춘 자리 기준이라 어느 쪽에서 들어와도 맞음).
        2026-11-20: 고정 goal_pose.yaw(91°)로 돌았더니 반원 옆에서 멈춘 경우 -85°만 돌아 등이 벽을 봄.
        로봇이 중심에 거의 붙어 있으면(0.05 m 안) 방향을 못 정하므로 goal_pose.yaw."""
        if not self.kiosk_face or self.last_pose is None:
            return None
        z, pose = self.plan.kiosk_zone, self.last_pose
        if self.kiosk_face_target == 'plane':
            # [코스 관람 변경 ⑪] 등이 키오스크 면을 똑바로 보게 = 정면이 반원이 열린 방향(zone.direction, 방 안쪽).
            #   서는 자리와 상관없이 방향이 항상 같다 → 관람 시작 때 그림 1 쪽으로 도는 각도가 매번 같고 작다.
            target = z.direction
        elif math.hypot(pose.x - z.cx, pose.y - z.cy) > 0.05:
            target = math.atan2(pose.y - z.cy, pose.x - z.cx)
        else:
            target = self.plan.kiosk_goal.yaw
        d = target - pose.yaw
        d = math.atan2(math.sin(d), math.cos(d))
        return d if abs(d) > self.face_tolerance else None

    def _start_registering(self, now: float, detail: str) -> list[Event]:
        """키오스크에서 관람객 등록 시작 (손바닥 안내 음성)."""
        self.state = 'REGISTERING'
        self.state_since = now
        self.visitor_since, self.visitor_last = None, float('-inf')   # 도착 전 인식은 안 셈
        return [self._event('KIOSK_STOPPED', detail), Event('guide', self.register_guide)]

    def _dock_turn(self) -> Optional[float]:
        """[코스 관람 변경 ⑨] 정면이 충전 스테이션(dock.station)을 보게 돌 각도 [rad, -π~π].
        필요 없거나(face_tolerance 이하) 끔/위치·스테이션 모름이면 None."""
        st, pose = self.plan.dock_station, self.last_pose
        if not self.dock_face or st is None or pose is None:
            return None
        if math.hypot(st.x - pose.x, st.y - pose.y) < 0.05:
            return None
        d = math.atan2(st.y - pose.y, st.x - pose.x) - pose.yaw
        d = math.atan2(math.sin(d), math.cos(d))
        return d if abs(d) > self.face_tolerance else None

    def _start_docking(self, now: float) -> list[Event]:
        """Create 3 Dock 액션 시작 (Nav2 없이 IR로 도크를 찾아 붙음)."""
        self.state = 'DOCKING'
        self.state_since = now
        return [Event('dock', None)]

    def spin_result(self, ok: bool, now: float) -> list[Event]:
        """Nav2 spin 결과. 실패해도 다음 단계는 진행한다 (방향만 덜 맞을 뿐).
        FACE_KIOSK → 관람객 등록 시작 / FACE_DOCK → Dock 액션."""
        if self.state == 'FACE_KIOSK':
            return self._start_registering(now, '관람객 등록 시작' if ok else '회전 실패; 그대로 등록 시작')
        if self.state == 'FACE_DOCK':
            note = [] if ok else [self._event('DOCK_FACE_FAILED', '회전 실패; 그대로 Dock')]
            return note + self._start_docking(now)
        return []            # 그 사이 취소·복귀로 상태가 바뀜

    def nav_result(self, result: str, now: float) -> list[Event]:
        """Nav2 NavigateToPose 결과 처리.

        result: 'SUCCEEDED' | 'CANCELED' | 'ABORTED' | 'REJECTED' | 'STATUS_n'
        우리가 영역 진입 시 직접 취소하므로 CANCELED도 정상 도착으로 볼 수 있음.
        """
        if self.state not in ('NAV_KIOSK', 'NAV_ARTWORK', 'NAV_HOME', 'NAV_EXIT', 'CANCELING',
                              'CANCELING_HOME', 'CANCELING_EXIT'):
            return []
        # --- 도킹 접근점에서 취소한 결과 ---
        if self.state == 'CANCELING_HOME':
            if result in ('CANCELED', 'SUCCEEDED') and self._fresh_home(now):
                return self._begin_stop(now, home=True)
            return self._recover(f'복귀 Nav2 {result}; 접근 위치 이탈', now)
        # --- 출구에서 취소한 결과: 성공으로 바꿔서 아래 NAV_EXIT 로직 재사용 ---
        if self.state == 'CANCELING_EXIT':
            self.state = 'NAV_EXIT'  # reuse NAV_EXIT success path below
            result = 'SUCCEEDED' if result in ('CANCELED', 'SUCCEEDED') else result
        if self.state == 'NAV_EXIT':
            if result == 'SUCCEEDED' and self._fresh_exit(now):
                self.state = 'VERIFY_EXIT_STOP'
                self.state_since = now
                self.still_since = None
                return [self._event('VERIFY_EXIT_STOP', '출구 목표점 도착 및 정지 확인')]
            return self._recover(f'출구 이동 Nav2 {result}; 출구 도착 미확인', now)
        # --- 도킹 접근점으로 가다가 Nav2가 스스로 끝낸 경우 ---
        if self.state == 'NAV_HOME':
            if result == 'SUCCEEDED' and self._fresh_home(now):
                return self._begin_stop(now, home=True)
            if result in ('ABORTED', 'REJECTED'):
                return self._recover(f'복귀 경로 Nav2 {result}', now)
            return self.fail(f'Nav2 {result}: 초기 언도킹 접근 위치 복귀 미확인')
        # --- 키오스크/작품 ---
        # Nav2 SUCCEEDED라도 반원 밖이면 도착으로 인정하지 않음
        # (xy_goal_tolerance 때문에 반원 밖에서 SUCCEEDED가 나올 수 있음)
        if result == 'SUCCEEDED' and self._fresh_inside(now):
            return self._begin_stop(now)
        if result == 'CANCELED' and self.state == 'CANCELING' and self._fresh_inside(now):
            return self._begin_stop(now)
        # [코스 관람 변경 ⑪] goal_pose까지 가다가 목표 바로 앞에서 Nav2가 포기(progress checker 등)해도
        #   이미 키오스크 반원 안이면 도착으로 본다 (반원 진입 취소를 안 쓰므로 그 대신)
        if (result == 'ABORTED' and self.state == 'NAV_KIOSK' and self.kiosk_stop_at_goal
                and self._fresh_inside(now)):
            return [self._event('KIOSK_NEAR_GOAL', 'Nav2 ABORTED이지만 키오스크 반원 안; 도착으로 처리')] + \
                self._begin_stop(now)
        if result in ('ABORTED', 'REJECTED'):
            return self._recover(f'Nav2 {result}; 운영자 복구 승인 필요', now)
        return self._recover(f'Nav2 {result}: 현재 반원 도착 미확인; 동일 목표 재검증 필요', now)

    def _begin_stop(self, now: float, *, home: bool = False) -> list[Event]:
        """정지 확인 단계로 진입 (home=True면 도킹 접근점용)."""
        self.state = 'VERIFY_HOME_STOP' if home else 'VERIFY_STOP'
        self.state_since = now
        self.still_since = None
        return [self._event(self.state, '목표 위치 내부 및 Odometry 연속 정지 확인')]

    def explanation_result(self, artwork_id: int, success: bool, now: float) -> list[Event]:
        """작품 해설(TTS) 완료 결과 처리 -> 다음 작품 / 출구 / 복귀로 진행."""
        if self.state != 'EXPLAINING':
            return []
        active = self.plan.artworks[self.index].artwork_id
        if artwork_id != active:
            return self.fail(f'잘못된 작품 해설 응답: {artwork_id} != {active}')
        # [코스 관람 변경 ⑧] 완료 순간에 잠깐 튄 것은 봐주고, '밖/끊김'이 explain_lost_grace 넘게 계속된 경우만 실패
        if (not self._explain_ok(now) and self.explain_bad_since is not None
                and now-self.explain_bad_since >= self.explain_lost_grace):
            return self.fail('해설 완료 시점에 현재 작품 반원 밖 또는 위치 정보 만료')
        if not success:
            return self.fail(f'작품 {active} 해설 실패')
        self.completed.append(active)
        events = [self._event('ARTWORK_COMPLETED', str(active))]
        self.index += 1
        if self.index == len(self.plan.artworks):
            # 마지막 작품까지 끝남
            if self.plan.exit_goal is not None:
                # [실물 로봇 추가] 출구 정지점으로 이동
                self.exit_pending = True
                self.state = 'NAV_EXIT'
                self.state_since = now
                events.extend([self._event('NAV_TO_EXIT', '모든 작품 방문 완료; 출구로 이동'),
                               Event('navigate', ('exit', self.plan.exit_goal))])
            else:
                # 출구 설정이 없으면 바로 복귀
                events.extend(self._go_home(now, '모든 작품 방문 완료; 언도킹 접근 위치 복귀'))
        else:
            # 다음 작품으로 이동
            self.state = 'NAV_ARTWORK'
            self.state_since = now
            nxt = self.plan.artworks[self.index]
            events.append(Event('navigate', (nxt.artwork_id, nxt.goal)))
        return events

    def docking_action_result(self, success: bool, now: float) -> list[Event]:
        """Dock 액션 결과. 성공해도 dock_status.is_docked 확인 전까지는 완료 아님."""
        if self.state != 'DOCKING':
            return []
        if not success:
            return self.fail('Dock 액션 실패/거절/취소')
        self.state = 'WAIT_DOCK_CONFIRM'
        self.state_since = now
        return [self._event('DOCK_ACTION_SUCCEEDED', 'dock_status.is_docked 확인 대기')]

    def dock_confirmed(self, now: float) -> list[Event]:
        """dock_status.is_docked == True 확인 -> 코스 최종 완료(COMPLETED)."""
        if self.state != 'WAIT_DOCK_CONFIRM':
            return []
        self.state = 'DONE'
        self.state_since = now
        # 안내 음성(tour_end)은 출구(FAREWELL)에서 이미 했으므로 도킹 후에는 하지 않음
        detail = (f'관람 없이 복귀 ({self.end_reason}) + 실제 도킹 완료' if self.end_reason
                  else '전체 작품 관람 + 원위치 복귀 + 실제 도킹 완료; PC3 예약 해제 요청')
        return [self._event('DOCKED', '도킹 액션 및 is_docked 상태 확인'),
                self._event('COMPLETED', detail)]

    def dock_simulated(self, now: float) -> list[Event]:
        """명시적으로 켠 모의 도킹(simulate_docking). 실제 물리 도킹 성공이 아님."""
        if self.state != 'DOCKING':
            return []
        self.state = 'DONE'
        self.state_since = now
        return [self._event('SIMULATED_DOCKING', '실제 Dock 액션 없이 시뮬레이션 완료'),
                self._event('SIMULATED_COMPLETED', '물리 도킹 검증 전; 실제 COMPLETED와 구분')]

    def fail(self, reason: str) -> list[Event]:
        """복구 불가 오류 -> ERROR 상태로 종료 (이미 종료 상태면 무시)."""
        if self.state in ('IDLE', 'DONE', 'ERROR'):
            return []
        self.state = 'ERROR'
        self.fail_reason = reason
        return [self._event('FAILED', reason)]

    # ------------------------------------------------------------------
    # [prepare_tour] 관람객 인식 / 도중 복귀
    # ------------------------------------------------------------------

    def visitor_seen(self, now: float, xy: Optional[tuple[float, float]] = None):
        """관람객 인식 1건 (노드가 target_person/point_map 받을 때마다 호출, xy = map 좌표).
        register_gap보다 오래 끊겼다가 다시 보이면 등록 확인 시간을 처음부터 센다."""
        if self.visitor_since is None or now-self.visitor_last > self.register_gap:
            self.visitor_since = now
        self.visitor_last = now
        if xy is not None:
            self.visitor_xy = (float(xy[0]), float(xy[1]))
            self.visitor_samples.append((now, self.visitor_xy[0], self.visitor_xy[1]))
            self.visitor_samples = [smp for smp in self.visitor_samples if now-smp[0] <= self.distance_window]

    def _visitor_distance(self, now: float) -> Optional[float]:
        """로봇-관람객 거리 [m]. 위치나 인식이 오래됐으면 None."""
        if (not self._fresh_pose(now) or self.visitor_xy is None
                or now-self.visitor_last > self.lost_timeout):
            return None
        # 최근 distance_window 동안 받은 좌표들의 거리 중앙값 (튀는 값 한두 개는 무시됨)
        recent = [smp for smp in self.visitor_samples if now-smp[0] <= self.distance_window] \
            or [(now, self.visitor_xy[0], self.visitor_xy[1])]
        dists = sorted(math.hypot(x-self.last_pose.x, y-self.last_pose.y) for _, x, y in recent)
        mid = len(dists) // 2
        return dists[mid] if len(dists) % 2 else (dists[mid-1]+dists[mid]) / 2.0

    def _visitor_problem(self, now: float) -> str:
        """이동 중 멈춰야 하는 이유 ('' = 괜찮음)."""
        if now-self.visitor_last > self.lost_timeout:
            return f'관람객이 {self.lost_timeout:.0f}초 넘게 안 보임'
        dist = self._visitor_distance(now)
        if dist is not None and dist > self.far_distance:
            return f'관람객이 멀어짐 ({dist:.2f} m > {self.far_distance:.2f} m)'
        return ''

    def _visitor_near(self, now: float) -> bool:
        """멈춘 뒤 다시 출발해도 되는지: 관람객이 보이고 resume_distance 안."""
        dist = self._visitor_distance(now)
        return dist is not None and dist <= self.resume_distance

    def begin_tour(self, artwork_ids, now: float) -> list[Event]:
        """[start_tour] WAIT_START(등록 완료)에서 관람 시작. artwork_ids = 작품 번호 목록.
        비어 있으면 전체 작품. 방문 순서는 YAML 코스 순서 (요청 순서와 상관없이 동선대로)."""
        if self.state != 'WAIT_START':
            raise ValueError('not_prepared')
        ids = tuple(int(i) for i in artwork_ids)
        plan = self.base_plan
        if ids:
            picked = {a.artwork_id for a in plan.selected(ids).artworks}   # 없는 번호·중복이면 ValueError
            plan = plan.selected(tuple(a.artwork_id for a in plan.artworks if a.artwork_id in picked))
        self.plan = plan
        self.completed = []
        self.index = 0
        self.touring = True
        self.far_since = None
        self.near_since = None
        self.come_here_at = float('-inf')
        self.state = 'NAV_ARTWORK'
        self.state_since = now
        self.still_since = None
        first = self.plan.artworks[0]
        return [self._event('TOUR_STARTED', ','.join(str(a.artwork_id) for a in self.plan.artworks)),
                Event('guide', self.start_guide), Event('navigate', (first.artwork_id, first.goal))]

    # 복귀 중(또는 이미 끝난) 상태: abort_to_home은 이유만 남기고 하던 복귀를 계속함
    RETURN_STATES = ('NAV_HOME', 'CANCELING_HOME', 'VERIFY_HOME_STOP', 'FACE_DOCK', 'DOCKING', 'WAIT_DOCK_CONFIRM')

    def redock(self, reason: str, now: float) -> list[Event]:
        """[코스 관람 변경 ④] [prepare_tour] 언독 직후, 키오스크로 출발하기 전에 그만둘 때: Nav2 없이 그 자리에서 Dock 액션으로 다시 도킹.
        전: 언독 후 센서 대기 시간 초과 → fail로 그 자리 정지 (사람이 손으로 도크에 올려야 했음).
        지금: 도크가 바로 앞이므로 Create 3 Dock만으로 돌아간다 (Nav2·AMCL이 준비 안 돼도 됨).
        (언독 후 Nav2·AMCL이 준비되지 않았거나 그 사이 취소됨. 도크가 바로 앞이라 Create3 Dock만으로 돌아갈 수 있다)
        reason은 end_reason에 남는다."""
        if self.state != 'NAV_KIOSK':
            raise ValueError(f'{self.state} 상태에서는 바로 재도킹할 수 없습니다 (언독 직후에만)')
        if not self.end_reason:
            self.end_reason = reason
        self.returning = True
        self.exit_pending = False
        self.state = 'DOCKING'
        self.state_since = now
        return [self._event('REDOCK', f'{reason}; 키오스크로 가지 않고 바로 재도킹'), Event('dock', None)]

    def abort_to_home(self, reason: str, now: float) -> list[Event]:
        """관람 시작 전·관람 중 어디서든 그만두고 충전 스테이션으로 복귀 (취소·Nav2 실패 등).
        새 Nav2 goal이 이전 goal을 대신하므로 따로 취소하지 않는다. reason은 end_reason에 남는다."""
        if self.state in ('IDLE', 'DONE', 'ERROR'):
            raise ValueError(f'{self.state} 상태에서는 복귀할 수 없습니다')
        if not self.end_reason:
            self.end_reason = reason
        if self.state in self.RETURN_STATES:
            return []
        self.exit_pending = False
        return self._go_home(now, f'{reason}; 충전 스테이션으로 복귀')
