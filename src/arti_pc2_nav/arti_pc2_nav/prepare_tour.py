"""① 호출 = prepare_tour 액션 (robot4 코스관람) — [뼈대] robot4 담당이 로직을 채운다.

형식은 robot1(arti_follow/prepare_tour.py)과 같고 로직만 다르다. 팀 명세: docs/common/ros2_interfaces.md 2·2.1·3.1

키오스크 흐름 (robot4)
  [호출] ─▶ /robot4/arti/prepare_tour {start: true}
            undock → (센서 대기) → 키오스크(만남 장소)로 이동 → 정지   ← robot4는 관람객 인식 없음
          ◀─ result success=true
  키오스크가 곧바로 자동으로 ─▶ /robot4/arti/start_tour {artwork_ids: [...]}   (start_tour.py)

feedback state (PrepareTour.action): UNDOCKING | MOVING | ARRIVED | REGISTERING | REGISTERED
  robot4 대응 예 (course_engine.TourMachine.state 기준, 담당자가 확정)
    undock 중 (a_course_node._undocking·_await_sensors_since) → UNDOCKING
    NAV_KIOSK, CANCELING                                    → MOVING
    VERIFY_STOP (키오스크)                                   → ARRIVED
    키오스크 정지 확인 끝                                     → REGISTERED → result success
result message 예: registered / not_requested / not_docked / undock_failed / not_ready(+_check_ready 이유) / canceled

노드에 붙이는 법 (담당자)
  1. class ACourseNode(PrepareMixin, TourMixin, Node): 로 상속하고 __init__ 끝에서
       group = ReentrantCallbackGroup(); self.setup_prepare_tour(group); self.setup_start_tour(group)
  2. main의 rclpy.spin(node) → MultiThreadedExecutor (run_prepare가 기다리는 동안 _tick·구독·undock 콜백이 돌아야 함).
     robot1 visitor_follower main 참고. 같은 값을 액션 스레드와 _tick이 같이 고치면 lock으로 묶는다.
  3. 상태 머신이 키오스크 정지 뒤 바로 작품으로 가지 않고 start_tour를 기다리게 (course_engine: 예 KIOSK_READY 상태)
  4. package.xml에 <exec_depend>arti_interfaces</exec_depend>
공통 설정: 작품·키오스크·출구 구역 = arti_common/config/artworks.yaml (arti_common.zones.load_zones),
           도크·출발 위치 = arti_common/config/init_pose.yaml robots./robot4. 작품 번호는 int (art_02 → 2).
"""

import time

from arti_interfaces.action import PrepareTour
from rclpy.action import ActionServer, CancelResponse, GoalResponse


class PrepareMixin:
    """ACourseNode가 상속해서 쓰는 호출(prepare_tour) 함수. TODO 부분을 robot4 담당이 채운다."""

    def setup_prepare_tour(self, callback_group):
        """__init__에서 한 번: /<ns>/arti/prepare_tour 액션 서버를 연다 (네임스페이스는 launch가 붙임)."""
        self._prepare_server = ActionServer(
            self, PrepareTour, 'arti/prepare_tour', self.run_prepare, callback_group=callback_group,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,      # 받을 수 없는 goal은 run_prepare에서 이유와 함께 끝냄
            cancel_callback=lambda _goal: CancelResponse.ACCEPT)

    def run_prepare(self, goal):
        """[액션 실행] 호출 → undock → 키오스크 이동·정지까지. 0.2 s마다 feedback, 끝나면 result.
        받음: goal (start, is_cancel_requested)
        돌려줌: PrepareTour.Result(success = 키오스크에서 출발 준비 완료, message = registered 또는 실패 이유)"""
        if not goal.request.start:
            goal.abort()
            return PrepareTour.Result(success=False, message='not_requested')
        why = self.prepare_begin()                      # TODO: 점검(_check_ready) + undock 시작 (+ 키오스크 이동 예약)
        if why:
            goal.abort()
            return PrepareTour.Result(success=False, message=why)
        while True:
            if goal.is_cancel_requested:
                self.prepare_cancel()                   # TODO: Nav2·undock 취소, 상태 머신 정리
                goal.canceled()
                return PrepareTour.Result(success=False, message='canceled')
            done, why = self.prepare_status()           # TODO: 키오스크 정지 확인 / 실패 여부
            if done:
                break
            self.send_prepare_feedback(goal)
            time.sleep(0.2)
        self.send_prepare_feedback(goal)
        if why:
            goal.abort()
            return PrepareTour.Result(success=False, message=why)
        goal.succeed()
        return PrepareTour.Result(success=True, message='registered')

    def send_prepare_feedback(self, goal):
        """내부 상태 → PrepareTour feedback state (위 대응 예 참고)."""
        goal.publish_feedback(PrepareTour.Feedback(state=self.prepare_feedback_state()))

    # ---------- robot4 담당이 채울 곳 ----------

    def prepare_begin(self):
        """호출을 시작한다. 돌려줌: '' (시작함) 또는 실패 이유 (예: not_docked, not_ready: 배터리 부족 …)."""
        raise NotImplementedError('robot4: _check_ready → undock 시작 (a_course_node._on_request 참고)')

    def prepare_status(self):
        """돌려줌: (끝났나, 실패 이유). 키오스크 정지 확인되면 (True, ''), 실패면 (True, 이유), 진행 중이면 (False, '')."""
        raise NotImplementedError('robot4: TourMachine.state가 키오스크 정지(예: KIOSK_READY)인지 / ERROR인지')

    def prepare_cancel(self):
        """취소 요청이 오면 정지·정리."""
        raise NotImplementedError('robot4: Nav2 goal·undock 취소, TourMachine.fail(...)')

    def prepare_feedback_state(self):
        """지금 상태 → 'UNDOCKING' | 'MOVING' | 'ARRIVED' | 'REGISTERING' | 'REGISTERED'."""
        raise NotImplementedError('robot4: 위 대응 예대로 TourMachine.state를 바꿔 돌려줌')
