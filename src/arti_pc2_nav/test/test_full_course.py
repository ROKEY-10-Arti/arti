"""TourMachine 단위 테스트 (ROS 없이 실행).

실행: cd src/arti_pc2_nav && python3 -m pytest -q test/
흐름: start()(prepare_tour) -> 키오스크 -> 등록 -> WAIT_START -> begin_tour()(start_tour) -> 작품 -> 출구 -> 도킹.
반원 판정, 출구 직사각형(arti_common), 등록, 관람객 거리 감시, 도킹 확인, 도킹 복귀(실패·취소)를 검사합니다.
"""
import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
# 출구 구역은 arti_common(공통, follow 팀과 같은 파일·코드)에서 읽는다 → 저장소 안의 소스를 바로 씀
sys.path.insert(0, str(ROOT.parent / 'common' / 'arti_common'))

from arti_pc2_nav.course_engine import TourMachine, Event  # noqa: E402
from arti_pc2_nav.guided_tour_plan import load_course_config, load_start_pose, Pose2D  # noqa: E402

P = ROOT / 'config/a_course.yaml'
Z = str(ROOT.parent / 'common' / 'arti_common' / 'config' / 'artworks.yaml')


def machine(**kw):
    """도킹 위치 (0, 0)에서 prepare_tour를 시작한 상태 기계 (NAV_KIOSK)."""
    m = TourMachine(load_course_config(P, Z), **kw)
    m.start('test', 0, Pose2D(0, 0))
    return m


def stop_at(m, pose, t):
    """pose에서 반원 진입 -> Nav2 취소 결과 -> 정지 확인까지 진행. 마지막 tick의 Event 반환.
    키오스크(kiosk_stop_at_goal)는 취소 없이 goal_pose까지 가므로 Nav2 SUCCEEDED로 도착."""
    m.tick(t, pose, 0, 0, 0)
    m.nav_result('SUCCEEDED' if m.state == 'NAV_KIOSK' else 'CANCELED', t)
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

    def test_start_pose_from_init_pose_yaml(self):
        """언독 후 initialpose = arti_common init_pose.yaml robots./robot4.undock (follow 팀과 같은 파일)."""
        path = ROOT.parent / 'common' / 'arti_common' / 'config' / 'init_pose.yaml'
        u = load_start_pose(path, 'robot4', 'undock')
        self.assertAlmostEqual(u.x, -0.12)
        self.assertAlmostEqual(u.y, 0.03)
        self.assertAlmostEqual(math.degrees(u.yaw), -179)
        d = load_start_pose(path, '/robot4', 'dock')               # yaw_deg 없음 → dock 자신 방향 0
        self.assertEqual((d.x, d.y), (0.304, -0.0957))
        self.assertIsNone(load_start_pose(path, '/robot9', 'undock'))

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

    def test_redock_right_after_undock(self):
        """언독 직후 Nav2·AMCL이 준비 안 됨 → 키오스크로 가지 않고 Dock 액션으로 바로 재도킹 → 끝나면 DONE."""
        m = self.m                                          # start() 직후 = NAV_KIOSK (노드는 언독 후 준비 대기 중)
        ev = m.redock('not_ready: Nav2', 5)
        self.assertEqual(m.state, 'DOCKING')
        self.assertEqual(m.end_reason, 'not_ready: Nav2')
        self.assertIn(Event('dock', None), ev)
        self.assertNotIn('navigate', [e.kind for e in ev])  # Nav2 안 씀
        m.docking_action_result(True, 30)
        m.dock_confirmed(31)
        self.assertEqual(m.state, 'DONE')

    def test_redock_only_before_leaving(self):
        m = self.m
        stop_at(m, m.plan.kiosk_goal, 2)                    # 키오스크 도착 뒤에는 재도킹 아님 (Nav2로 복귀)
        with self.assertRaises(ValueError):
            m.redock('canceled', 5)

    def arrive(self, t=1.):
        """키오스크 정지 -> REGISTERING + 손바닥 안내. 다음 시각 반환."""
        ev = stop_at(self.m, self.m.plan.kiosk_goal, t)
        self.assertEqual(self.m.state, 'REGISTERING')
        self.assertIn(Event('guide', 'register'), ev)
        return t + 1

    def test_kiosk_turn_before_register(self):
        """반원에 들어서자마자 키오스크를 보고 멈춤 → goal_pose.yaw(등을 키오스크로)까지 spin → 등록 시작."""
        m, g = self.m, self.m.plan.kiosk_goal
        facing = Pose2D(g.x, g.y, g.yaw + math.pi)                # 반대 방향으로 섬
        ev = stop_at(m, facing, 1)
        self.assertEqual(m.state, 'FACE_KIOSK')
        spin = [e.value for e in ev if e.kind == 'spin']
        self.assertEqual(len(spin), 1)
        self.assertAlmostEqual(abs(spin[0]), math.pi, places=3)   # 180° 회전
        self.assertNotIn(Event('guide', 'register'), ev)          # 돌기 전에는 등록 안내 안 함
        ev = m.spin_result(True, 5)
        self.assertEqual(m.state, 'REGISTERING')
        self.assertIn(Event('guide', 'register'), ev)

    def test_kiosk_goes_to_goal_not_zone_edge(self):
        """[⑪] 키오스크 반원에 들어서도 Nav2를 취소하지 않고 goal_pose까지 감 → SUCCEEDED로 정지 확인."""
        m, z = self.m, self.m.plan.kiosk_zone
        edge = Pose2D(z.cx + 0.45, z.cy + 0.30, 0.0)               # 반원 안 가장자리
        self.assertTrue(z.contains(edge))
        ev = m.tick(1, edge, 0, 0.2, 0)
        self.assertEqual(m.state, 'NAV_KIOSK')                    # 안 멈춤
        self.assertNotIn(Event('cancel', None), ev)
        m.nav_result('SUCCEEDED', 1.5)                            # goal_pose 도착
        self.assertEqual(m.state, 'VERIFY_STOP')
        old = machine(kiosk_stop_at_goal=False)                   # 전 방식: 진입 즉시 취소
        old.tick(1, edge, 0, 0.2, 0)
        self.assertEqual(old.state, 'CANCELING')

    def test_kiosk_aborted_inside_zone_counts_as_arrived(self):
        """[⑪] 목표 바로 앞에서 Nav2 ABORTED여도 반원 안이면 도착. 반원 밖이면 복구."""
        m, g = self.m, self.m.plan.kiosk_goal
        m.nav_result('ABORTED', 0)                                # 위치 정보 전 → 반원 판정 불가
        self.assertEqual(m.state, 'WAIT_RECOVERY')
        m = machine(kiosk_goal_tolerance=0.0)                     # 근처 도착 끄고 ABORTED 경로만 봄
        m.tick(1, Pose2D(g.x, g.y - 0.1, g.yaw), 0, 0, 0)
        m.nav_result('ABORTED', 2)
        self.assertEqual(m.state, 'VERIFY_STOP')
        m2 = machine()
        m2.tick(1, Pose2D(0, 0), 0, 0, 0)                          # 도크 앞, 반원 밖
        m2.nav_result('ABORTED', 2)
        self.assertEqual(m2.state, 'WAIT_RECOVERY')

    def test_kiosk_near_goal_counts_as_arrived(self):
        """[⑪] 2026-11-20 실주행: 찍은 자리 0.22 m 앞 (-1.04, 0.02)에서 Nav2가 더 못 감 → 0.30 m 안이면 도착."""
        m, g = self.m, self.m.plan.kiosk_goal
        p = Pose2D(-1.04, 0.02, 2.5)
        self.assertLess(math.hypot(p.x - g.x, p.y - g.y), 0.30)
        ev = m.tick(1, p, 0, 0.2, 0)
        self.assertEqual(m.state, 'CANCELING')
        self.assertIn(Event('cancel', None), ev)
        m.nav_result('CANCELED', 1.2)
        self.assertEqual(m.state, 'VERIFY_STOP')
        m.tick(1.3, p, 0, 0, 0)
        m.tick(2.2, p, 0, 0, 0)
        self.assertEqual(m.state, 'FACE_KIOSK')                     # → 회전 → 등록(스캔)
        m.spin_result(True, 4)
        self.assertEqual(m.state, 'REGISTERING')

    def test_kiosk_turn_to_plane_from_side(self):
        """[⑪] 어디에 서든 등이 키오스크 면을 보게 = 정면이 zone.direction."""
        m, z = self.m, self.m.plan.kiosk_zone
        p = Pose2D(z.cx + 0.45, z.cy + 0.30, math.radians(176))
        ev = stop_at(m, p, 1)
        self.assertEqual(m.state, 'FACE_KIOSK')
        turn = [e.value for e in ev if e.kind == 'spin'][0]
        self.assertAlmostEqual(math.cos(p.yaw + turn - z.direction), 1.0, places=6)

    def test_kiosk_turn_back_to_kiosk_from_side(self):
        """kiosk_face_target='center'(전): 반원 옆(동쪽)에서 서쪽을 보고 멈춤 → 등이 반원 중심을 보게 약 180°."""
        self.m = machine(register_timeout=60, ready_timeout=130, kiosk_face_target='center')
        m, z = self.m, self.m.plan.kiosk_zone
        p = Pose2D(z.cx + 0.45, z.cy + 0.30, math.radians(176))   # 실주행 2026-11-20과 비슷한 자리
        ev = stop_at(m, p, 1)
        self.assertEqual(m.state, 'FACE_KIOSK')
        turn = [e.value for e in ev if e.kind == 'spin'][0]
        final = p.yaw + turn
        away = math.atan2(p.y - z.cy, p.x - z.cx)                 # 키오스크 반대쪽
        self.assertAlmostEqual(math.cos(final - away), 1.0, places=6)
        self.assertGreater(abs(math.degrees(turn)), 140)

    def test_kiosk_turn_small_or_failed(self):
        m, g = self.m, self.m.plan.kiosk_goal
        stop_at(m, Pose2D(g.x, g.y, g.yaw + 0.2), 1)              # 0.2 rad 차이 → 안 돎
        self.assertEqual(m.state, 'REGISTERING')
        m2 = machine()
        stop_at(m2, Pose2D(g.x, g.y, g.yaw - 2.0), 1)
        self.assertEqual(m2.state, 'FACE_KIOSK')
        m2.spin_result(False, 3)                                  # 회전 실패해도 등록은 시작
        self.assertEqual(m2.state, 'REGISTERING')
        m3 = machine(face_timeout=5)
        stop_at(m3, Pose2D(g.x, g.y, g.yaw - 2.0), 1)
        m3.tick(7, Pose2D(g.x, g.y, g.yaw - 2.0), 0, 0, 0)        # 결과가 안 옴 → 시간 초과 후 등록
        self.assertEqual(m3.state, 'REGISTERING')
        m4 = machine(kiosk_face=False)
        stop_at(m4, Pose2D(g.x, g.y, g.yaw - 2.0), 1)
        self.assertEqual(m4.state, 'REGISTERING')

    def test_cancel_while_turning_goes_home(self):
        m, g = self.m, self.m.plan.kiosk_goal
        stop_at(m, Pose2D(g.x, g.y, g.yaw + math.pi), 1)
        ev = m.abort_to_home('canceled', 3)
        self.assertIn('navigate', [e.kind for e in ev])
        self.assertEqual(m.spin_result(True, 4), [])              # 늦게 온 회전 결과는 무시

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

    def explaining_on_arc(self):
        """작품 1 반원 호 바로 안쪽에서 멈춰 해설 시작 (실주행 2026-11-20). (machine, zone, 다음 시각)"""
        m, t = self.m, self.t
        m.watch_visitor = False
        m.begin_tour([1], t)
        z = m.plan.artworks[0].zone
        edge = Pose2D(z.cx + (z.radius - 0.01) * math.cos(z.direction), z.cy + (z.radius - 0.01) * math.sin(z.direction))
        stop_at(m, edge, t + 1)
        self.assertEqual(m.state, 'EXPLAINING')
        return m, z, t + 3

    def test_explaining_tolerates_amcl_jitter(self):
        """해설 중 AMCL이 호 밖으로 몇 cm 튀거나(여유 0.15 m) 3초 안에 돌아오면 계속 해설."""
        m, z, t = self.explaining_on_arc()
        out = lambda d: Pose2D(z.cx + (z.radius + d) * math.cos(z.direction), z.cy + (z.radius + d) * math.sin(z.direction))
        m.tick(t, out(0.05), 0, 0, 0)            # 5 cm 밖 → 여유 안
        self.assertEqual(m.state, 'EXPLAINING')
        m.tick(t + 0.5, out(0.30), 0, 0, 0)      # 30 cm 튐 → 지켜봄
        m.tick(t + 2.0, out(0.30), 0, 0, 0)
        self.assertEqual(m.state, 'EXPLAINING')
        m.tick(t + 2.5, out(0.0), 0, 0, 0)       # 돌아옴
        m.tick(t + 9.0, out(0.0), 0, 0, 0)
        self.assertEqual(m.state, 'EXPLAINING')
        m.explanation_result(1, True, t + 9.5)
        self.assertEqual(m.completed, [1])

    def test_explaining_fails_when_really_out_or_lost(self):
        m, z, t = self.explaining_on_arc()
        far = Pose2D(z.cx + (z.radius + 0.5) * math.cos(z.direction), z.cy + (z.radius + 0.5) * math.sin(z.direction))
        m.tick(t, far, 0, 0, 0)
        m.tick(t + 3.1, far, 0, 0, 0)            # 3초 넘게 밖 → 실패
        self.assertEqual(m.state, 'ERROR')
        self.assertIn('반원 이탈', m.fail_reason)
        m2, z2, t2 = self.__class__.explaining_on_arc(self._fresh())
        m2.tick(t2 + 1, None, 99, 0, 0)          # 위치 끊김 (2 s 넘으면 신선하지 않음)
        m2.tick(t2 + 6.5, None, 99, 0, 0)
        self.assertEqual(m2.state, 'ERROR')
        self.assertIn('소실', m2.fail_reason)

    def _fresh(self):
        """같은 setUp 상태의 새 테스트 객체 (machine을 하나 더 쓰려고)."""
        other = self.__class__(self._testMethodName)
        other.setUp()
        return other

    def home_stop(self, m, pose, t):
        """도킹 접근 원 안 pose에서 멈춤: NAV_HOME → 취소 → 정지 확인 통과. 마지막 Event 반환."""
        m.state, m.state_since, m.returning = 'NAV_HOME', t, True
        m.tick(t, pose, 0, 0, 0)
        self.assertEqual(m.state, 'CANCELING_HOME')
        m.nav_result('CANCELED', t)
        m.tick(t + .1, pose, 0, 0, 0)
        return m.tick(t + 1, pose, 0, 0, 0)

    def test_face_dock_before_docking(self):
        """[코스 관람] 출구(북쪽)에서 내려와 남쪽을 보고 접근 원에 멈춤 → 정면이 스테이션을 보게 약 +90° → Dock.
        (2026-11-20: 도크를 안 보고 Dock → 앞 벽에 부딪혀 실패)"""
        m, t = self.m, self.t
        h, st = m.home_goal, m.plan.dock_station
        pose = Pose2D(h.x + 0.05, h.y + 0.20, -math.pi / 2)
        ev = self.home_stop(m, pose, t)
        self.assertEqual(m.state, 'FACE_DOCK')
        turn = [e.value for e in ev if e.kind == 'spin'][0]
        want = math.atan2(st.y - pose.y, st.x - pose.x)
        self.assertAlmostEqual(math.cos(pose.yaw + turn - want), 1.0, places=6)
        self.assertNotIn(Event('dock', None), ev)                 # 돌기 전엔 Dock 안 함
        ev = m.spin_result(True, t + 4)
        self.assertEqual(m.state, 'DOCKING')
        self.assertIn(Event('dock', None), ev)

    def test_face_dock_skip_fail_timeout(self):
        m, t = self.m, self.t
        h, st = m.home_goal, m.plan.dock_station
        facing = math.atan2(st.y - h.y, st.x - h.x)
        ev = self.home_stop(m, Pose2D(h.x, h.y, facing + 0.1), t)   # 이미 도크를 봄 → 바로 Dock
        self.assertEqual(m.state, 'DOCKING')
        self.assertIn(Event('dock', None), ev)
        m2 = machine()
        m2.home_goal = h
        self.home_stop(m2, Pose2D(h.x, h.y, facing + 2.0), t)
        self.assertEqual(m2.state, 'FACE_DOCK')
        self.assertEqual(m2.abort_to_home('canceled', t + 2), [])  # 복귀 중이라 그대로 (노드가 spin 취소)
        m2.spin_result(False, t + 2.5)                              # 회전 실패여도 Dock 시도
        self.assertEqual(m2.state, 'DOCKING')
        m3 = machine(face_timeout=5)
        m3.home_goal = h
        p3 = Pose2D(h.x, h.y, facing + 2.0)
        self.home_stop(m3, p3, t)
        ev = m3.tick(t + 7, p3, 0, 0, 0)                            # 결과가 안 옴 → 시간 초과 후 Dock
        self.assertEqual(m3.state, 'DOCKING')
        self.assertIn(Event('dock', None), ev)
        m4 = machine(dock_face=False)
        m4.home_goal = h
        self.home_stop(m4, Pose2D(h.x, h.y, facing + 2.0), t)
        self.assertEqual(m4.state, 'DOCKING')

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
