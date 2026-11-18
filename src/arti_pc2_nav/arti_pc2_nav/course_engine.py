"""ROS에 의존하지 않는 ArtI PC2 코스 상태 머신(TourMachine).

전체 흐름:
  UNDOCK(ROS 노드가 처리) -> KIOSK -> 작품 1..N -> EXIT(인사) ->
  DOCK APPROACH(스테이션 정면) -> Dock 액션 -> dock_status 확인 -> COMPLETED

- EXIT는 계획에 exit_goal이 있을 때만 사용합니다(반원 없는 점 목표, 거리 판정).
- 객체 인식/추적 없음. 실제 이동은 전부 Nav2가 담당합니다.

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
  EXPLAINING        작품 해설(TTS) 재생 중, 끝날 때까지 대기
  NAV_EXIT          출구 정지점으로 이동 중
  CANCELING_EXIT    출구 0.30m 진입 -> 취소 결과 대기
  VERIFY_EXIT_STOP  출구에서 정지 확인 중
  FAREWELL          '안녕히가세요' 재생 시간 대기
  NAV_HOME          충전 스테이션 접근점으로 이동 중
  CANCELING_HOME    접근점 0.30m 진입 -> 취소 결과 대기
  VERIFY_HOME_STOP  접근점에서 정지 확인 중
  DOCKING           Dock 액션 실행 중
  WAIT_DOCK_CONFIRM Dock 성공 후 dock_status.is_docked 확인 대기
  WAIT_RECOVERY     오류 발생, 운영자 retry 승인 대기
  DONE / ERROR      종료
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Optional
from .guided_tour_plan import CoursePlan, CourseRequest, Pose2D, resolve_selection


@dataclass(frozen=True)
class Event:
    """상태 머신이 ROS 노드에게 요청하는 동작 1건.

    kind 별 value:
      status   -> 상태 JSON dict (/arti/course_status 로 발행)
      navigate -> (이름, Pose2D) : Nav2 NavigateToPose 목표 전송
      cancel   -> None           : 현재 Nav2 목표 취소
      guide    -> 문자열          : guide_message 토픽 발행 (TTS 인사 등)
      explain  -> explanation_id : 작품 해설 요청
      dock     -> None           : Dock 액션 실행
    """
    kind: str  # status | navigate | cancel | guide | explain | dock
    value: object


class TourMachine:
    def __init__(self, plan: CoursePlan, *, stop_duration: float = 0.8,
                 stop_timeout: float = 20.0, navigation_timeout: float = 180.0,
                 explanation_timeout: float = 60.0, dock_timeout: float = 90.0,
                 confirmation_timeout: float = 12.0,
                 home_position_tolerance: float = 0.30,
                 simulate_explanation: bool = False, simulated_duration: float = 2.0,
                 max_retries: int = 2, exit_position_tolerance: float = 0.30,
                 farewell_duration: float = 2.0):
        # --- 설정값 (단위: 초 / m) ---
        self.base_plan = plan                      # YAML 원본 계획
        self.plan = plan                           # 이번 주행에 쓰는 계획(A/B 선택 반영)
        self.stop_duration = stop_duration         # 이 시간만큼 연속 정지해야 '정지'로 인정
        self.stop_timeout = stop_timeout           # 정지 확인 최대 대기 시간
        self.navigation_timeout = navigation_timeout   # 한 구간 Nav2 이동 최대 시간
        self.explanation_timeout = explanation_timeout # 해설 완료 응답 최대 대기
        self.dock_timeout = dock_timeout           # Dock 액션 최대 시간
        self.confirmation_timeout = confirmation_timeout  # Dock 후 is_docked 확인 최대 대기
        self.home_position_tolerance = home_position_tolerance  # 도킹 접근점 도착 허용 반경
        self.simulate_explanation = simulate_explanation   # True면 TTS 없이 2초 후 해설 완료 처리
        self.simulated_duration = simulated_duration
        self.max_retries = max_retries             # 운영자 retry 최대 횟수(구간마다 리셋)
        self.exit_position_tolerance = exit_position_tolerance  # 출구 도착 허용 반경
        self.farewell_duration = farewell_duration  # 출구 인사 후 출발까지 대기 시간
        # --- 실행 중 상태 ---
        self.state = 'IDLE'
        self.request_id = ''
        self.index = -1                    # -1 = 키오스크, 0..N-1 = 작품 순번, N = 모두 완료
        self.completed: list[str] = []     # 해설까지 끝난 작품 ID 목록
        self.dock_origin: Optional[Pose2D] = None  # robot pose before first undock (언도킹 전 도킹 위치)
        self.home_goal: Optional[Pose2D] = None    # safe Nav2 return pose after undock (복귀 목표)
        self.last_pose: Optional[Pose2D] = None    # 마지막으로 받은 신선한 AMCL 위치
        self.last_pose_at = float('-inf')
        self.still_since: Optional[float] = None   # 정지 상태가 시작된 시각
        self.state_since = 0.0                     # 현재 상태에 들어온 시각 (타임아웃 계산용)
        self.retry_count = 0
        self.exit_pending = False                  # 출구 구간 진행 중 여부(retry 시 목표 결정용)

    def _event(self, state: str, detail: str = '') -> Event:
        """/arti/course_status 로 나갈 상태 Event 생성."""
        return Event('status', {'request_id': self.request_id, 'state': state,
                                'detail': detail, 'completed': list(self.completed)})

    # ------------------------------------------------------------------
    # 시작 / 준비
    # ------------------------------------------------------------------

    def start(self, request: CourseRequest, now: float, dock_origin: Pose2D) -> list[Event]:
        """코스 시작: 계획을 확정하고 첫 목표(키오스크)로 이동 Event를 낸다.

        dock_origin = 언도킹 전(도킹 상태)의 로봇 위치.
        """
        if self.state not in ('IDLE', 'DONE', 'ERROR'):
            raise ValueError('이미 코스주행 중입니다')
        if not isinstance(dock_origin, Pose2D) or not all(math.isfinite(x) for x in
                                  (dock_origin.x, dock_origin.y, dock_origin.yaw)):
            raise ValueError('언도킹 출발 위치(map 좌표)가 필요합니다')
        self.plan = resolve_selection(self.base_plan, request)
        self.request_id = request.request_id
        self.completed = []
        self.index = -1
        self.retry_count = 0
        self.dock_origin = dock_origin
        # YAML의 고정 도킹 접근점이 있으면 그것을 사용, 없으면(시뮬레이션 skip_undock)
        # 출발 위치를 복귀 목표로 사용.
        self.home_goal = self.plan.dock_approach or dock_origin
        self.exit_pending = False
        self.state = 'NAV_KIOSK'
        self.state_since = now
        self.still_since = None
        self.last_pose = None
        self.last_pose_at = float('-inf')
        return [self._event('STARTED', ','.join(a.artwork_id for a in self.plan.artworks)),
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

    def _fresh_exit(self, now: float) -> bool:
        """출구 정지점 0.30m 이내인지 (반원 대신 거리 판정)."""
        goal = self.plan.exit_goal
        return (self._fresh_pose(now) and goal is not None and
                math.hypot(self.last_pose.x-goal.x, self.last_pose.y-goal.y)
                <= self.exit_position_tolerance)

    def _go_home(self, now: float, detail: str) -> list[Event]:
        """충전 스테이션 접근점(home_goal)으로 이동 시작."""
        self.state = 'NAV_HOME'
        self.state_since = now
        return [self._event('RETURNING_HOME', detail),
                Event('navigate', ('dock_approach', self.home_goal))]

    def _fresh_home(self, now: float) -> bool:
        """도킹 접근점 0.30m 이내인지."""
        return (self._fresh_pose(now) and self.home_goal is not None and
                math.hypot(self.last_pose.x-self.home_goal.x,
                           self.last_pose.y-self.home_goal.y) <= self.home_position_tolerance)

    # ------------------------------------------------------------------
    # 오류 복구
    # ------------------------------------------------------------------

    def _recover(self, reason: str, now: float) -> list[Event]:
        """복구 가능한 오류: Nav2 취소 후 운영자 retry 대기(WAIT_RECOVERY).

        retry 횟수를 초과하면 ERROR로 종료.
        """
        if self.retry_count >= self.max_retries:
            return self.fail(f'복구 허용 횟수 초과: {reason}')
        self.state = 'WAIT_RECOVERY'
        self.state_since = now
        return [self._event('RECOVERY_REQUIRED', reason), Event('cancel', None)]

    def retry(self, now: float) -> list[Event]:
        """운영자가 승인한 재시도 (/arti/course_recovery {"action":"retry"}).

        실패했던 구간의 목표로 다시 Nav2 이동을 보냅니다.
        """
        if self.state != 'WAIT_RECOVERY':
            raise ValueError('복구 승인 대기 상태가 아닙니다')
        self.retry_count += 1
        # 어느 구간에서 실패했는지로 다시 갈 상태 결정
        if self.exit_pending:
            self.state = 'NAV_EXIT'
        else:
            self.state = 'NAV_KIOSK' if self.index < 0 else ('NAV_HOME' if self.index == len(self.plan.artworks) else 'NAV_ARTWORK')
        self.state_since = now
        self.still_since = None
        if self.state == 'NAV_EXIT':
            target = ('exit', self.plan.exit_goal)
        elif self.state == 'NAV_KIOSK':
            target = ('kiosk', self.plan.kiosk_goal)
        elif self.state == 'NAV_HOME':
            target = ('return_to_undock', self.home_goal)
        else:
            art = self.plan.artworks[self.index]
            target = (art.artwork_id, art.goal)
        return [self._event('RECOVERY_RETRY', f'재시도 {self.retry_count}/{self.max_retries}'),
                Event('navigate', target)]

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
            if self.state not in ('NAV_HOME', 'NAV_EXIT') and self._entry_inside(now):
                self.state = 'CANCELING'
                self.state_since = now
                return [self._event('ZONE_ENTERED', 'Nav2 목표 취소 요청'), Event('cancel', None)]
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
            self.retry_count = 0
            if self.state == 'VERIFY_EXIT_STOP':
                # 출구 -> '안녕히가세요' 안내(guide 토픽 -> TTS)
                self.state = 'FAREWELL'
                self.state_since = now
                return [self._event('EXIT_ARRIVED', self.plan.farewell_text),
                        Event('guide', self.plan.farewell_text)]
            if self.state == 'VERIFY_HOME_STOP':
                # 도킹 접근점 -> Dock 액션
                self.state = 'DOCKING'
                self.state_since = now
                return [self._event('HOME_ARRIVED', '충전 스테이션 접근 위치 도착; Dock 액션 실행'),
                        Event('dock', None)]
            if self.index < 0:
                # 키오스크 -> 첫 번째 작품으로 이동
                self.index = 0
                self.state = 'NAV_ARTWORK'
                self.state_since = now
                first = self.plan.artworks[0]
                return [self._event('KIOSK_STOPPED', '객체인식 단계는 PC2 단독 시뮬레이션에서 제외'),
                        Event('guide', 'start'), Event('navigate', (first.artwork_id, first.goal))]
            # 작품 -> 해설(TTS) 시작
            self.state = 'EXPLAINING'
            self.state_since = now
            art = self.plan.artworks[self.index]
            return [self._event('ARTWORK_STOPPED', art.artwork_id), Event('explain', art.explanation_id)]

        # (5) 해설 중: 반원 이탈 감시, 결과는 explanation_result()로 들어옴
        elif self.state == 'EXPLAINING':
            if not self._fresh_inside(now):
                return self.fail('해설 중 현재 작품 반원 이탈 또는 위치 정보 소실')
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
        if result in ('ABORTED', 'REJECTED'):
            return self._recover(f'Nav2 {result}; 운영자 복구 승인 필요', now)
        return self._recover(f'Nav2 {result}: 현재 반원 도착 미확인; 동일 목표 재검증 필요', now)

    def _begin_stop(self, now: float, *, home: bool = False) -> list[Event]:
        """정지 확인 단계로 진입 (home=True면 도킹 접근점용)."""
        self.state = 'VERIFY_HOME_STOP' if home else 'VERIFY_STOP'
        self.state_since = now
        self.still_since = None
        return [self._event(self.state, '목표 위치 내부 및 Odometry 연속 정지 확인')]

    def explanation_result(self, artwork_id: str, success: bool, now: float) -> list[Event]:
        """작품 해설(TTS) 완료 결과 처리 -> 다음 작품 / 출구 / 복귀로 진행."""
        if self.state != 'EXPLAINING':
            return []
        active = self.plan.artworks[self.index].artwork_id
        if artwork_id != active:
            return self.fail(f'잘못된 작품 해설 응답: {artwork_id} != {active}')
        if not self._fresh_inside(now):
            return self.fail('해설 완료 시점에 현재 작품 반원 밖 또는 위치 정보 만료')
        if not success:
            return self.fail(f'작품 {active} 해설 실패')
        self.completed.append(active)
        events = [self._event('ARTWORK_COMPLETED', active)]
        self.index += 1
        self.retry_count = 0
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
        return [self._event('DOCKED', '도킹 액션 및 is_docked 상태 확인'),
                Event('guide', 'tour_end'),
                self._event('COMPLETED', '전체 작품 관람 + 원위치 복귀 + 실제 도킹 완료; PC3 예약 해제 요청')]

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
        return [self._event('FAILED', reason)]
