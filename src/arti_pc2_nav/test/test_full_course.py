"""TourMachine 단위 테스트 (ROS 없이 실행).

실행: cd src/arti_pc2_nav && python3 -m pytest -q test/
흐름: start()(prepare_tour) -> 키오스크 -> 등록 -> WAIT_START -> begin_tour()(start_tour) -> 작품 -> 출구 -> 도킹.
반원 판정, 출구 직사각형(arti_common), 등록, 관람객 거리 감시, 도킹 확인, 도킹 복귀(실패·취소)를 검사합니다.
"""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# 출구 구역은 arti_common(공통, follow 팀과 같은 파일·코드)에서 읽는다 → 저장소 안의 소스를 바로 씀
sys.path.insert(0, str(ROOT.parent / 'common' / 'arti_common'))

from arti_pc2_nav.course_engine import TourMachine, Event  # noqa: E402
from arti_pc2_nav.guided_tour_plan import load_course_config, Pose2D  # noqa: E402

P = ROOT / 'config/a_course.yaml'
Z = str(ROOT.parent / 'common' / 'arti_common' / 'config' / 'artworks.yaml')


def machine(**kw):
    """도킹 위치 (0, 0)에서 prepare_tour를 시작한 상태 기계 (NAV_KIOSK)."""
    m = TourMachine(load_course_config(P, Z), **kw)
    m.start('test', 0, Pose2D(0, 0))
    return m


def stop_at(m, pose, t):
    """pose에서 반원 진입 -> Nav2 취소 결과 -> 정지 확인까지 진행. 마지막 tick의 Event 반환."""
    m.tick(t, pose, 0, 0, 0)
    m.nav_result('CANCELED', t)
    m.tick(t + .1, pose, 0, 0, 0)
    return m.tick(t + 1, pose, 0, 0, 0)


def ready(m, t=1.):
    """키오스크 도착 -> 등록(skip_register) -> WAIT_START."""
    m.skip_register = True
    stop_at(m, m.plan.kiosk_goal, t)
    m.tick(t + 1.1, m.plan.kiosk_goal, 0, 0, 0)
    assert m.state == 'WAIT_START', m.state
    return t + 2


class PlanTests(unittest.TestCase):
    """YAML 설정과 반원."""

    def setUp(self):
        self.m = machine()

    def test_half_plane(self):
        z = self.m.plan.artworks[0].zone
        self.assertFalse(z.contains((z.cx - .1, z.cy)))
        self.assertTrue(z.contains((z.cx + .1, z.cy)))

    def test_exit_zone_from_arti_common(self):
        """출구 = arti_common artworks.yaml의 exit 직사각형 (follow 팀과 같은 구역)."""
        z = self.m.plan.exit_zone
        self.assertIsNotNone(z)                             # exit.zone: arti_common
        self.assertEqual(z.shape, 'rect')
        self.assertEqual(z.box, (-0.82, 0.13, 1.36, 1.91))
        g = self.m.plan.exit_goal
        self.assertTrue(z.contains((g.x, g.y)))             # 목표점은 구역 안
        self.assertTrue(z.contains((-0.75, 1.40)))          # 목표점에서 0.30m 넘게 떨어져도 구역 안이면 도착
        self.assertFalse(z.contains((0.20, 1.60)))          # 오른쪽 벽 너머

    def test_artwork_ids_are_int(self):
        self.assertEqual([a.artwork_id for a in self.m.plan.artworks], [1, 2, 3, 4])

    def test_select_by_int_ids(self):
        plan = self.m.base_plan.selected((1, 3, 4))
        self.assertEqual([a.artwork_id for a in plan.artworks], [1, 3, 4])
        with self.assertRaises(ValueError):
            self.m.base_plan.selected((5,))

    def test_fixed_dock_approach_overrides_undock_pose(self):
        self.m.set_post_undock_pose(Pose2D(-0.111, -0.098))
        self.assertEqual(self.m.home_goal, Pose2D(-0.35, -0.0957, 0.0))


class PrepareTests(unittest.TestCase):
    """prepare_tour: 키오스크 -> 등록(REGISTERING) -> WAIT_START, 실패·시간 초과·취소는 도킹 복귀."""

    def setUp(self):
        self.m = machine(register_timeout=60, ready_timeout=130)

    def arrive(self, t=1.):
        """키오스크 정지 -> REGISTERING + 손바닥 안내. 다음 시각 반환."""
        ev = stop_at(self.m, self.m.plan.kiosk_goal, t)
        self.assertEqual(self.m.state, 'REGISTERING')
        self.assertIn(Event('guide', 'register'), ev)
        return t + 1

    def test_stop_requires_odom(self):
        m = self.m
        m.state = 'VERIFY_STOP'
        p = m.plan.kiosk_goal
        m.tick(1, p, 0, None, 99)
        m.tick(2, p, 0, None, 99)
        self.assertEqual(m.state, 'VERIFY_STOP')

    def test_register_after_one_second_of_visitor(self):
        m, t = self.m, self.arrive()
        k = m.plan.kiosk_goal
        for i in range(10):
            m.visitor_seen(t + i * .1)
            m.tick(t + i * .1, k, 0, 0, 0)
        self.assertEqual(m.state, 'REGISTERING')            # 0.9초: 아직
        m.visitor_seen(t + 1.0)
        ev = m.tick(t + 1.0, k, 0, 0, 0)
        self.assertEqual(m.state, 'WAIT_START')
        self.assertIn(Event('guide', 'recognized'), ev)

    def test_gap_restarts_confirmation(self):
        m, t = self.m, self.arrive()
        k = m.plan.kiosk_goal
        m.visitor_seen(t)
        m.visitor_seen(t + .8)                               # 0.8초 끊김 > register_gap 0.5 -> 다시 셈
        for dt in (1.2, 1.6):
            m.visitor_seen(t + dt)
            m.tick(t + dt, k, 0, 0, 0)
            self.assertEqual(m.state, 'REGISTERING')
        m.visitor_seen(t + 1.8)                              # 0.8초부터 1.0초 연속
        m.tick(t + 1.8, k, 0, 0, 0)
        self.assertEqual(m.state, 'WAIT_START')

    def test_register_timeout_returns_to_dock(self):
        m, t = self.m, self.arrive()
        ev = m.tick(t + 60, m.plan.kiosk_goal, 0, 0, 0)
        self.assertEqual(m.state, 'NAV_HOME')
        self.assertEqual(m.end_reason, 'register_failed')
        self.assertIn(Event('guide', 'register_failed'), ev)
        self.assertIn("'dock_approach'", str(ev))

    def test_ready_timeout_returns_to_dock(self):
        m = self.m
        ready(m)                                             # WAIT_START에 들어간 시각 = m.state_since
        since = m.state_since
        m.tick(since + 129.9, m.plan.kiosk_goal, 0, 0, 0)
        self.assertEqual(m.state, 'WAIT_START')
        m.tick(since + 130, m.plan.kiosk_goal, 0, 0, 0)      # 130초 지남
        self.assertEqual(m.state, 'NAV_HOME')
        self.assertEqual(m.end_reason, 'not_started')

    def test_cancel_returns_home_and_docks(self):
        m = self.m
        m.abort_to_home('canceled', 1)
        self.assertEqual(m.state, 'NAV_HOME')
        self.assertTrue(m.returning)
        h = m.home_goal
        m.tick(2, h, 0, 0, 0)
        m.nav_result('CANCELED', 2)
        m.tick(2.1, h, 0, 0, 0)
        m.tick(3, h, 0, 0, 0)
        self.assertEqual(m.state, 'DOCKING')
        m.docking_action_result(True, 4)
        ev = m.dock_confirmed(5)
        self.assertEqual(m.state, 'DONE')
        self.assertIn('canceled', str(ev))

    def test_nav_fail_while_returning_is_marked(self):
        """복귀 중 Nav2 실패 -> WAIT_RECOVERY + returning (노드가 이걸 보고 ERROR로 끝냄)."""
        m = self.m
        m.abort_to_home('canceled', 1)
        m.nav_result('ABORTED', 2)
        self.assertEqual(m.state, 'WAIT_RECOVERY')
        self.assertTrue(m.returning)


class TourTests(unittest.TestCase):
    """start_tour: WAIT_START에서 선택 작품 관람, 관람객 거리 감시(멈춤·재출발·분실), 출구, 도킹, 취소."""

    def setUp(self):
        self.m = machine()
        self.t = ready(self.m)

    def near(self, t, pose, d=0.5):
        """관람객을 로봇에서 d [m] 떨어진 곳에서 봄."""
        self.m.visitor_seen(t, (pose.x + d, pose.y))

    def test_not_prepared(self):
        m = TourMachine(load_course_config(P, Z))
        with self.assertRaises(ValueError):
            m.begin_tour([1], 0)

    def test_course_order_follows_yaml(self):
        ev = self.m.begin_tour([4, 1, 3], self.t)
        self.assertEqual([a.artwork_id for a in self.m.plan.artworks], [1, 3, 4])
        self.assertEqual(self.m.state, 'NAV_ARTWORK')
        self.assertIn(Event('guide', 'start'), ev)

    def test_empty_ids_means_all(self):
        self.m.begin_tour([], self.t)
        self.assertEqual(len(self.m.plan.artworks), 4)

    def test_unknown_id_rejected(self):
        with self.assertRaises(ValueError):
            self.m.begin_tour([9], self.t)
        self.assertEqual(self.m.state, 'WAIT_START')

    def test_explain_event_sends_int(self):
        m = self.m
        m.watch_visitor = False
        m.begin_tour([1], self.t)
        ev = stop_at(m, m.plan.artworks[0].goal, self.t + 1)
        self.assertIn(Event('explain', 1), ev)

    def test_far_visitor_pauses_then_resumes(self):
        m, t = self.m, self.t
        m.begin_tour([1, 3], t)
        p = Pose2D(-1.5, 0.5)                                 # 작품 1 반원 밖, 이동 중
        self.near(t + .5, p, 0.5)
        self.assertEqual(m.tick(t + .5, p, 0, .2, 0), [])
        self.near(t + 1, p, 3.0)                              # 3 m > far 2.5 (중앙값이 3 m가 되도록 1초 뒤 값만 남김)
        self.assertEqual(m.tick(t + 1.6, p, 0, .2, 0), [])    # 처음 '멀다' → far_hold 1초 기다림
        self.near(t + 2.4, p, 3.0)
        m.tick(t + 2.4, p, 0, .2, 0)
        self.assertEqual(m.state, 'NAV_ARTWORK')
        self.near(t + 2.7, p, 3.0)
        ev = m.tick(t + 2.7, p, 0, .2, 0)                     # 1초 넘게 계속 멂 → 멈춤
        self.assertEqual(m.state, 'WAIT_VISITOR')
        self.assertIn(Event('cancel', None), ev)
        self.assertIn(Event('guide', 'come_here'), ev)
        self.near(t + 4, p, 2.0)                              # 2.0 m: 아직 (resume 1.8)
        m.tick(t + 4, p, 0, 0, 0)
        self.assertEqual(m.state, 'WAIT_VISITOR')
        self.near(t + 5.2, p, 1.0)
        m.tick(t + 5.2, p, 0, 0, 0)                           # 처음 '가깝다' → near_hold 0.5초
        self.assertEqual(m.state, 'WAIT_VISITOR')
        self.near(t + 5.8, p, 1.0)
        ev = m.tick(t + 5.8, p, 0, 0, 0)
        self.assertEqual(m.state, 'NAV_ARTWORK')
        self.assertIn("Event(kind='navigate', value=(1,", str(ev))

    def test_jumpy_position_does_not_stop(self):
        """인식 위치가 가끔 튀어도(1.0 m 사이에 3.5 m 한 번씩) 중앙값이라 멈추지 않는다."""
        m, t = self.m, self.t
        m.begin_tour([1], t)
        p = Pose2D(-1.5, 0.5)
        for i in range(30):                                   # 3초, 0.1초 간격
            now = t + 0.1 * i
            self.near(now, p, 3.5 if i % 4 == 0 else 1.0)
            m.tick(now, p, 0, .2, 0)
            self.assertEqual(m.state, 'NAV_ARTWORK', i)

    def test_come_here_voice_has_cooldown(self):
        """멈춤이 반복돼도 '따라오세요'는 come_here_interval(8초)에 한 번만."""
        m, t = self.m, self.t
        m.begin_tour([1], t)
        p = Pose2D(-1.5, 0.5)
        guides = []
        now = t
        for cycle in range(3):                                # 멂(1.2초) → 가까움(0.7초) 반복
            for _ in range(13):
                self.near(now, p, 3.0)
                guides += [e for e in m.tick(now, p, 0, .2, 0) if e.kind == 'guide']
                now += 0.1
            for _ in range(7):
                self.near(now, p, 0.5)
                guides += [e for e in m.tick(now, p, 0, 0, 0) if e.kind == 'guide']
                now += 0.1
        self.assertEqual(guides, [Event('guide', 'come_here')])

    def test_lost_visitor_pauses(self):
        m, t = self.m, self.t
        m.begin_tour([1], t)
        p = Pose2D(-1.5, 0.5)
        self.near(t, p)
        m.tick(t + 2.9, p, 0, .2, 0)
        self.assertEqual(m.state, 'NAV_ARTWORK')
        m.tick(t + 3.1, p, 0, .2, 0)                          # 3초 넘게 안 보임 → far_hold 시작
        self.assertEqual(m.state, 'NAV_ARTWORK')
        m.tick(t + 4.2, p, 0, .2, 0)                          # 1초 더 → 멈춤
        self.assertEqual(m.state, 'WAIT_VISITOR')

    def test_give_up_after_10s_returns_to_dock(self):
        m, t = self.m, self.t
        m.begin_tour([1], t)
        p = Pose2D(-1.5, 0.5)
        self.near(t, p, 3.0)
        m.tick(t, p, 0, .2, 0)
        self.near(t + 1.1, p, 3.0)
        m.tick(t + 1.1, p, 0, .2, 0)
        self.assertEqual(m.state, 'WAIT_VISITOR')
        w = t + 1.1                                           # 멈춘 시각
        m.tick(w + 9.9, p, 0, 0, 0)
        self.assertEqual(m.state, 'WAIT_VISITOR')
        ev = m.tick(w + 10, p, 0, 0, 0)
        self.assertEqual(m.state, 'NAV_HOME')
        self.assertEqual(m.end_reason, 'visitor_lost')
        self.assertIn(Event('guide', 'register_failed'), ev)

    def test_no_watch_when_disabled(self):
        m = self.m
        m.watch_visitor = False
        m.begin_tour([1], self.t)
        m.tick(self.t + 20, Pose2D(-1.5, 0.5), 0, .2, 0)
        self.assertEqual(m.state, 'NAV_ARTWORK')

    def test_cancel_during_explaining_and_while_returning(self):
        m = self.m
        m.begin_tour([1], self.t)
        m.state = 'EXPLAINING'
        m.abort_to_home('canceled', self.t + 1)
        self.assertEqual(m.state, 'NAV_HOME')
        self.assertEqual(m.end_reason, 'canceled')
        self.assertEqual(m.abort_to_home('canceled', self.t + 2), [])   # 이미 복귀 중
        self.assertEqual(m.state, 'NAV_HOME')

    def test_exit_far_is_nav_failure(self):
        """출구 Nav2 SUCCEEDED인데 출구 직사각형 밖 -> WAIT_RECOVERY (노드가 도킹 복귀)."""
        m = self.m
        m.state, m.exit_pending = 'NAV_EXIT', True
        m.last_pose, m.last_pose_at = Pose2D(0.185, 1.5), 1
        m.nav_result('SUCCEEDED', 1)
        self.assertEqual(m.state, 'WAIT_RECOVERY')

    def test_exit_arrival_anywhere_in_rect(self):
        """출구 목표점에서 0.30m 넘게 떨어져도 직사각형 안에서 멈추면 출구 도착 (follow 팀과 같은 판정)."""
        m = self.m
        m.state, m.exit_pending, m.touring = 'NAV_EXIT', True, False
        p = Pose2D(-0.70, 1.45, 0)                          # 목표점 (-0.345, 1.635)에서 약 0.40 m
        m.tick(2, p, 0, 0, 0)
        self.assertEqual(m.state, 'CANCELING_EXIT')

    def test_full_tour_every_artwork_then_physical_dock(self):
        m, t = self.m, self.t
        m.watch_visitor = False
        m.begin_tour([], t)
        for art in m.plan.artworks:
            t += 2
            stop_at(m, art.goal, t)
            self.assertEqual(m.state, 'EXPLAINING')
            m.explanation_result(art.artwork_id, True, t + 1)
        self.assertEqual(m.completed, [1, 2, 3, 4])
        self.assertEqual(m.state, 'NAV_EXIT')
        t += 2
        ex = m.plan.exit_goal
        m.tick(t, ex, 0, 0, 0)
        self.assertEqual(m.state, 'CANCELING_EXIT')
        m.nav_result('CANCELED', t)
        self.assertEqual(m.state, 'VERIFY_EXIT_STOP')
        ev = m.tick(t + .1, ex, 0, 0, 0) + m.tick(t + 1, ex, 0, 0, 0)
        self.assertEqual(m.state, 'FAREWELL')
        self.assertIn(Event('guide', 'tour_end'), ev)
        ev = m.tick(t + 3.5, ex, 0, 0, 0)
        self.assertEqual(m.state, 'NAV_HOME')
        self.assertEqual(m.home_goal, m.plan.dock_approach)
        t += 4
        stop_at(m, m.home_goal, t)
        self.assertEqual(m.state, 'DOCKING')
        m.docking_action_result(True, t + 2)
        self.assertEqual(m.state, 'WAIT_DOCK_CONFIRM')
        ev = m.dock_confirmed(t + 3)
        self.assertEqual(m.state, 'DONE')
        self.assertEqual(m.end_reason, '')
        self.assertNotIn('guide', [e.kind for e in ev])       # 안내는 출구에서 한 번만

    def test_dock_needs_confirmation(self):
        m = self.m
        m.state = 'DOCKING'
        ev = m.docking_action_result(True, 1)
        self.assertEqual(m.state, 'WAIT_DOCK_CONFIRM')
        self.assertNotIn('COMPLETED', str(ev))
        self.assertIn('COMPLETED', str(m.dock_confirmed(2)))

    def test_dock_fail(self):
        m = self.m
        m.state = 'DOCKING'
        m.docking_action_result(False, 1)
        self.assertEqual(m.state, 'ERROR')


if __name__ == '__main__':
    unittest.main()
