"""② 관람 = start_tour 액션 (robot4 코스관람) — [뼈대] robot4 담당이 로직을 채운다.

형식은 robot1(arti_follow/start_tour.py)과 같고 로직만 다르다. 팀 명세: docs/common/ros2_interfaces.md 2·2.1·3.2

키오스크 흐름 (robot4): prepare_tour success 직후 키오스크가 자동으로
  ─▶ /robot4/arti/start_tour {artwork_ids: [1, 3, 4]}   (선택한 작품 번호, 이 순서대로)
      작품마다 이동 → 정지 → 해설 → 다음 작품 → 출구(인사) → 도크 복귀 → Dock
  ◀─ result success=true (도킹까지 완료) / false (실패 이유)

feedback (StartTour.action): state + artwork_id (관련 작품 없으면 0)
  state: MOVING | FOLLOW | EXPLAINING | ARTWORK_COMPLETED | RETURNING   (robot4는 FOLLOW 안 씀)
  robot4 대응 예 (course_engine.TourMachine.state 기준, 담당자가 확정)
    NAV_ARTWORK, CANCELING, VERIFY_STOP          → MOVING      (artwork_id = 가는 작품)
    EXPLAINING                                   → EXPLAINING  (artwork_id = 설명 중인 작품)
    explanation_result 성공 직후 (작품마다 한 번)   → ARTWORK_COMPLETED (artwork_id = 끝난 작품)
    NAV_EXIT … FAREWELL, NAV_HOME … WAIT_DOCK_CONFIRM → RETURNING
result message 예: completed / failed:<이유> / canceled
작품 번호: StartTour.artwork_ids는 int32. a_course.yaml artwork_id('1')와 맞출 때 str(n) 또는 int로 통일.

노드에 붙이는 법은 prepare_tour.py 맨 위 참고 (setup_start_tour(group)).
"""

import time

from arti_interfaces.action import StartTour
from rclpy.action import ActionServer, CancelResponse, GoalResponse


class TourMixin:
    """ACourseNode가 상속해서 쓰는 관람(start_tour) 함수. TODO 부분을 robot4 담당이 채운다."""

    def setup_start_tour(self, callback_group):
        """__init__에서 한 번: /<ns>/arti/start_tour 액션 서버를 연다."""
        self._start_server = ActionServer(
            self, StartTour, 'arti/start_tour', self.run_tour, callback_group=callback_group,
            goal_callback=lambda _goal: GoalResponse.ACCEPT,
            cancel_callback=lambda _goal: CancelResponse.ACCEPT)

    def run_tour(self, goal):
        """[액션 실행, 관람 내내] 고른 작품 순서대로 관람 → 출구 → 도킹까지. 0.2 s마다 feedback, 끝나면 result.
        받음: goal (artwork_ids, is_cancel_requested)
        돌려줌: StartTour.Result(success = 도킹까지 완료, message = completed 또는 실패 이유)"""
        why = self.tour_begin(list(goal.request.artwork_ids))   # TODO: 키오스크 대기 상태에서 작품 선택 적용 + 첫 작품 이동
        if why:
            goal.abort()
            return StartTour.Result(success=False, message=why)
        while True:
            if goal.is_cancel_requested:
                self.tour_cancel()                              # TODO: Nav2 취소, 해설 취소, 상태 머신 정리
                goal.canceled()
                return StartTour.Result(success=False, message='canceled')
            for state, artwork_id in self.tour_feedback():      # TODO: 이번 주기에 보낼 feedback들
                goal.publish_feedback(StartTour.Feedback(state=state, artwork_id=int(artwork_id)))
            done, why = self.tour_status()                      # TODO: 도킹 완료 / 실패 여부
            if done:
                break
            time.sleep(0.2)
        if why:
            goal.abort()
            return StartTour.Result(success=False, message=why)
        goal.succeed()
        return StartTour.Result(success=True, message='completed')

    # ---------- robot4 담당이 채울 곳 ----------

    def tour_begin(self, artwork_ids):
        """관람 시작. 받음: artwork_ids [1, 3, 4] (int). 돌려줌: '' (시작함) 또는 거절 이유 (예: not_prepared, unknown_artwork)."""
        raise NotImplementedError('robot4: 키오스크 대기 상태 확인 → 작품 선택(CoursePlan.selected) → 첫 작품 Nav2')

    def tour_feedback(self):
        """돌려줌: 이번에 보낼 [(state, artwork_id), ...]. ARTWORK_COMPLETED는 작품마다 한 번만 넣는다."""
        raise NotImplementedError('robot4: 위 대응 예대로 TourMachine.state·completed를 바꿔 돌려줌')

    def tour_status(self):
        """돌려줌: (끝났나, 실패 이유). 도킹 확인(DONE)이면 (True, ''), ERROR면 (True, 이유), 진행 중이면 (False, '')."""
        raise NotImplementedError('robot4: TourMachine.state == DONE / ERROR')

    def tour_cancel(self):
        """취소 요청이 오면 정지·정리."""
        raise NotImplementedError('robot4: Nav2 goal 취소, 해설 취소, TourMachine.fail(...)')
