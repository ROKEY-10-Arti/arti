"""① 호출·등록 = prepare_tour 액션 (visitor_follower 노드의 일부). 키오스크 "로봇 호출" 버튼.

  run_prepare        액션 실행: 검사 → begin_tour → undock → 등록(REGISTERED)까지 feedback → success
  상태별 동작 (step이 부름)  LOCALIZE localize · START_FACE face_start · WAIT_VISITOR wait_visitor
                             · REGISTERED registered_wait (start_tour 기다리며 정지)
"""
import math
import time

from action_msgs.msg import GoalStatus
from arti_interfaces.action import PrepareTour
from geometry_msgs.msg import PoseWithCovarianceStamped
from irobot_create_msgs.action import Dock, Undock
import numpy as np
from std_msgs.msg import Empty, String

from arti_follow.motion import wrap
from arti_follow.start_tour import RETURNING


class PrepareMixin:
    """VisitorFollower가 상속해서 쓰는 호출·등록 단계 함수."""

    def begin_tour(self):
        """관람 시작 준비: 지난 관람객 데이터(위치, 등록, 본 작품)를 지우고 active = True, 첫 상태로.
        바뀜: kf.x, seen, holding, 단계 타이머, explaining, visited, seen_since, active, end_reason
              state → UNDOCK (undock 파라미터 false면 LOCALIZE)"""
        self.kf.x, self.seen, self.holding = None, -1e9, False
        self.face_until = self.loc_at = self.pose_sent = self.loc_log_at = self.face_log_at = self.face_goal = None
        self.active, self.end_reason, self.ready_at = True, '', None
        self.home = None                            # 이번 관람에서 언독하면 그 자리를 기록
        self.tour_at = self.end_at = None
        self.explaining, self.visited = 0, []
        self.reported = 0                           # visited 중 ARTWORK_COMPLETED로 보고한 수
        self.seen_since = None
        self.reg_first = self.reg_last = None       # 앞 사람 등록 요청 (WAIT_VISITOR)
        self.get_logger().info('호출 → 관람 준비 시작 (undock → 위치 → 키오스크 → 등록)')
        self.set_state('UNDOCK' if self.p['undock'] else 'LOCALIZE')

    def run_prepare(self, goal):
        """[액션 실행] prepare_tour goal 하나: 호출 → undock → 위치 잡기 → 키오스크 보기 → 관람객 등록까지 (goal None = auto_start).
        begin_tour → undock → 0.2 s마다 feedback (취소 확인) → REGISTERED가 되면 성공.
        관람 중 호출은 검사하지 않는다: 관람 중이면 로봇이 도크 밖 → 키오스크가 [호출]을 끈다 (dock_status).
        start = false면 아무것도 안 하고 끝냄 (not_requested).
        움직임은 step()이 하고, 여기서는 시작·끝과 키오스크 보고만 한다.
        받음: goal (start, is_cancel_requested)
        돌려줌: PrepareTour.Result(success = 등록 완료, message = registered 또는 실패 이유)
        성공·실패 모두 result 값과 goal.succeed/abort/canceled 여부를 로그로 남긴다"""
        if goal is not None and not goal.request.start:
            self.get_logger().warn('호출 거절 (start = false)')
            goal.abort()
            return PrepareTour.Result(success=False, message='not_requested')
        with self.lock:
            if self.active:
                self.get_logger().warn(f'호출 거절 (이미 진행 중, state={self.state})')
                if goal is not None:
                    goal.abort()
                return PrepareTour.Result(success=False, message='busy')
            self.begin_tour()
            state = self.state
        if state == 'UNDOCK':
            self.send_prepare_feedback(goal)
            why = self.undock(goal)
            with self.lock:
                if why:
                    self.end_tour('READY', why)
                elif self.active:
                    self.home = self.lookup(self.p['odom_frame'], self.p['base_frame'])   # 관람 끝나면 돌아올 곳
                    self.set_state('LOCALIZE')
        while True:
            with self.lock:
                if not self.active or self.state == 'REGISTERED':
                    break
                if goal is not None and goal.is_cancel_requested and self.state not in RETURNING:
                    self.end_tour('READY', 'canceled')   # 언독했으면 돌아가 도킹한 뒤 result (루프가 계속 돎)
            self.send_prepare_feedback(goal)
            time.sleep(0.2)
        with self.lock:
            ok = self.active and self.state == 'REGISTERED'
            reason = self.end_reason or 'unknown'
            cur_state = self.state
        if ok:
            self.send_prepare_feedback(goal)        # 마지막 feedback REGISTERED
            self.get_logger().info('prepare_tour 성공 → result success=true, message=registered '
                                   '(goal=succeed, start_tour 대기)')
            if goal is not None:
                goal.succeed()
            return PrepareTour.Result(success=True, message='registered')
        if goal is None:
            outcome = 'auto_start (goal 없음)'
        elif reason == 'canceled':
            outcome = 'goal.canceled()'
        else:
            outcome = 'goal.abort()'
        self.get_logger().error(f'prepare_tour 실패 → result success=false, message={reason!r}, '
                                f'{outcome} / state={cur_state}')
        if goal is not None:
            goal.canceled() if reason == 'canceled' else goal.abort()
        return PrepareTour.Result(success=False, message=reason)

    def send_prepare_feedback(self, goal):
        """내부 상태 → prepare_tour feedback (PrepareTour.action).
        UNDOCK → UNDOCKING, LOCALIZE·START_FACE → MOVING, WAIT_VISITOR → ARRIVED (아무도 안 보임) /
        REGISTERING (1 s 확인 중), REGISTERED → REGISTERED.
        받음: goal, (state, seen_since, seen)
        내보냄: feedback (state)"""
        if goal is None:
            return
        with self.lock:
            cur_state = self.state
            seen_since = self.seen_since
            seen = self.seen
        if cur_state == 'UNDOCK':
            state = 'UNDOCKING'
        elif cur_state == 'WAIT_VISITOR':
            seeing = seen_since is not None and self.now() - seen <= self.p['register_gap']
            state = 'REGISTERING' if seeing else 'ARRIVED'
        elif cur_state == 'REGISTERED':
            state = 'REGISTERED'
        else:                                       # READY·LOCALIZE·START_FACE·RETURN·DOCK_FACE·DOCK
            state = 'MOVING'
        goal.publish_feedback(PrepareTour.Feedback(state=state))

    def undock(self, goal):
        """Create 3에 undock 액션을 보내고 끝날 때까지 기다린다 (로봇이 스스로 움직임).
        관람은 항상 도크에서 시작한다 (undock 뒤 initialpose = 언독 지점이 맞으려면 도크에서 출발해야 함).
        도크 밖이면 먼저 Create 3 dock 액션으로 도크에 올린 뒤 undock 한다 (dock_first).
        받음: goal (취소 확인용), (docked: dock_status)
        돌려줌: '' (성공) / 'dock_status_unavailable' / 'no_dock_server' / 'dock_failed' /
                'no_undock_server' / 'undock_failed' / 'canceled'"""
        # dock_status를 아직 못 받았으면 잠깐(최대 3 s) 기다린다 (노드를 막 켰을 때)
        end = time.monotonic() + 3.0
        while self.docked is None and time.monotonic() < end:
            if goal is not None and goal.is_cancel_requested:
                return 'canceled'
            time.sleep(0.1)
        if self.docked is None:
            self.get_logger().error('dock_status를 받지 못함 (3 s) → undock 요청을 보내지 않음 '
                                    '(ros2 topic echo /robot1/dock_status --once 확인)')
            return 'dock_status_unavailable'
        if self.docked is False:
            why = self.dock_first(goal)
            if why:
                return why
        self.send_initialpose()                     # 도크 위 = start_point(dock) → AMCL 초기 위치, 언독 중엔 odom으로 따라감
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'undock 액션 서버 없음 ({self.get_namespace().rstrip("/")}/undock)')
            return 'no_undock_server'
        age = self.now() - self.docked_at if self.docked_at is not None else float('nan')
        self.get_logger().info(f'undock 시작 (dock_status is_docked={self.docked}, {age:.1f} s 전에 받음)')
        log = self.get_logger()
        t0 = time.monotonic()
        sent = self.undock_client.send_goal_async(Undock.Goal())
        # ① Create 3가 요청을 받았는지. Wi-Fi가 느리면 응답이 수 초~수십 초 늦거나 사라진다 (2026-11-19: ping 0.5~1.5 s,
        #    응답 없이 13 s 뒤 실제로 언독됨). 그래서 undock_timeout까지 기다리고, 응답이 없어도
        #    dock_status가 False가 되면 실제로 언독된 것으로 본다 → 로봇이 멈출 때까지 기다린 뒤 성공
        end = time.monotonic() + self.p['undock_timeout']
        while not sent.done():
            if goal is not None and goal.is_cancel_requested:
                return 'canceled'
            if self.docked is False:
                log.warn(f'undock 요청 응답은 아직 없지만 dock_status is_docked=False ({time.monotonic() - t0:.1f} s) '
                         f'→ 언독된 것으로 보고 로봇이 멈추길 기다림 (네트워크 지연)')
                return self.wait_undock_motion(end, goal, t0)
            if time.monotonic() > end:
                log.error(f'undock 실패: {self.p["undock_timeout"]:.0f} s 동안 요청 응답도 없고 도크에서도 안 떨어짐 '
                          f'(네트워크·discovery 확인: ping 로봇 IP). 지금 dock_status is_docked={self.docked}')
                return 'undock_failed'
            time.sleep(0.1)
        if not sent.result().accepted:
            log.error(f'undock 실패: Create 3가 요청을 거절함 ({time.monotonic() - t0:.1f} s). '
                      f'지금 dock_status is_docked={self.docked} (도크 밖이면 거절됨 → 로봇을 도크에 다시 올리기)')
            return 'undock_failed'
        log.info(f'undock 요청 받아들여짐 ({time.monotonic() - t0:.1f} s) → 끝나길 기다림')
        handle = sent.result()
        done = handle.get_result_async()
        # ② 언독 동작이 끝났는지 (결과 응답도 늦거나 사라질 수 있음 → 그때는 dock_status False + 로봇 정지로 판단)
        if not self.wait_future(done, max(end - time.monotonic(), 0.0), goal):
            if goal is not None and goal.is_cancel_requested:
                handle.cancel_goal_async()
                return 'canceled'
            if self.docked is False:
                log.warn('undock 결과 응답은 없지만 dock_status is_docked=False → 로봇이 멈추길 기다림 (네트워크 지연)')
                return self.wait_undock_motion(time.monotonic() + 15.0, goal, t0)
            handle.cancel_goal_async()
            log.error(f'undock 실패: {self.p["undock_timeout"]:.0f} s 안에 안 끝남 → 취소 요청. '
                      f'지금 dock_status is_docked={self.docked}')
            return 'undock_failed'
        # ③ 결과: 성공 status이고 도크에서 떨어졌는지
        res = done.result()
        ok = (not res.result.is_docked) if res.status == GoalStatus.STATUS_SUCCEEDED else self.docked is False
        if ok:
            log.info(f'undock 끝 ({time.monotonic() - t0:.1f} s, status {res.status})')
        else:
            log.error(f'undock 실패: 결과 status {res.status} (4=성공, 5=취소, 6=중단), '
                      f'result is_docked={res.result.is_docked}, 지금 dock_status is_docked={self.docked}')
        return '' if ok else 'undock_failed'

    def dock_first(self, goal):
        """[UNDOCK 전] 도크 밖이면 Create 3 dock 액션으로 먼저 도크에 올린다 (로봇이 스스로 도크를 찾아 움직임).
        Create 3가 IR로 도크를 볼 수 있는 거리·방향이어야 한다 (멀거나 등지고 있으면 dock_failed).
        응답이 늦거나 사라져도 dock_status가 True가 되면 성공으로 본다 (Wi-Fi 지연 대비, undock과 같은 방식).
        받음: goal (취소 확인), (dock_timeout, docked)
        돌려줌: '' (도크에 올라감) / 'no_dock_server' / 'dock_failed' / 'canceled'"""
        log = self.get_logger()
        if not self.dock_client.wait_for_server(timeout_sec=5.0):
            log.error(f'dock 액션 서버 없음 ({self.get_namespace().rstrip("/")}/dock) → 로봇을 손으로 도크에 올리기')
            return 'no_dock_server'
        log.warn('도크 밖에서 호출됨 (dock_status is_docked=false) → 먼저 Create 3 dock으로 도크에 올림')
        t0 = time.monotonic()
        end = t0 + self.p['dock_timeout']
        sent = self.dock_client.send_goal_async(Dock.Goal())
        result = None
        while time.monotonic() < end:
            if goal is not None and goal.is_cancel_requested:
                return 'canceled'
            if self.docked is True:                     # 응답보다 dock_status가 먼저 올 수 있음
                time.sleep(1.0)                         # 도크에 완전히 붙고 멈출 시간
                log.info(f'dock 끝 ({time.monotonic() - t0:.1f} s) → 이어서 undock')
                return ''
            if sent.done() and result is None:
                handle = sent.result()
                if not handle.accepted:
                    log.error('dock 실패: Create 3가 요청을 거절함 → 로봇을 손으로 도크에 올리기')
                    return 'dock_failed'
                result = handle.get_result_async()
            if result is not None and result.done():
                res = result.result()
                if res.result.is_docked or self.docked:
                    log.info(f'dock 끝 ({time.monotonic() - t0:.1f} s, status {res.status}) → 이어서 undock')
                    time.sleep(1.0)
                    return ''
                log.error(f'dock 실패: 결과 status {res.status}, is_docked=False '
                          '(도크가 IR로 안 보이는 위치일 수 있음 → 도크 앞 1 m 안, 도크를 보게 두고 다시)')
                return 'dock_failed'
            time.sleep(0.1)
        log.error(f'dock 실패: {self.p["dock_timeout"]:.0f} s 안에 도크에 못 올라감 → 로봇을 손으로 도크에 올리기')
        return 'dock_failed'

    def wait_undock_motion(self, end, goal, t0):
        """응답 없이 언독된 경우: Create 3가 스스로 움직이는 중일 수 있으므로 멈출 때까지 기다린다
        (멈추기 전에 initialpose·회전 명령을 보내면 언독 동작과 겹침).
        odom→base_link가 1 s 동안 1 cm·1° 안이면 멈춘 것. end(시각)까지 못 멈추면 그래도 진행.
        받음: end (time.monotonic 기준 마감), goal (취소 확인), t0 (언독 요청 시각, 로그용)
        돌려줌: '' (진행) / 'canceled'"""
        log, last, still_since = self.get_logger(), None, None
        while time.monotonic() < end:
            if goal is not None and goal.is_cancel_requested:
                return 'canceled'
            pose = self.lookup(self.p['odom_frame'], self.p['base_frame'])
            now = time.monotonic()
            if pose is not None and last is not None and math.hypot(pose[0] - last[0], pose[1] - last[1]) < 0.01 \
                    and abs(float(wrap(pose[2] - last[2]))) < math.radians(1.0):
                still_since = still_since or now
                if now - still_since >= 1.0:
                    log.info(f'undock 끝 (응답 없이 dock_status·odom으로 판단, {now - t0:.1f} s)')
                    return ''
            else:
                still_since, last = None, pose
            time.sleep(0.2)
        log.warn('언독 뒤 로봇이 멈췄는지 확인 못 함 (odom TF 없음 또는 계속 움직임) → 그래도 다음 단계로')
        return ''

    def wait_future(self, future, timeout, goal):
        """future(서비스·액션 응답)가 끝날 때까지 기다린다 (다른 스레드가 채움).
        받음: future, timeout [s], goal (취소되면 그만 기다림)
        돌려줌: 끝났으면 True, 시간 초과·취소면 False"""
        end = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > end or (goal is not None and goal.is_cancel_requested):
                return False
            time.sleep(0.1)
        return True

    def send_initialpose(self):
        """AMCL에 initialpose = start_point(기본 dock) 한 번. 도크 위에서 언독 직전에 부른다
        (그 뒤 언독하는 동안은 AMCL이 odom으로 로봇 이동을 따라감).
        받음: (start, set_initial_pose, initialpose 구독자)
        내보냄: initialpose (map). pose_sent = 보낸 시각"""
        p = self.p
        self.pose_sent = self.now()
        if self.start is None or not p['set_initial_pose']:
            return
        x, y, yaw = self.start
        subscribers = self.pose_pub.get_subscription_count()
        if subscribers == 0:
            self.get_logger().warn('initialpose 구독자(AMCL) 없음 → 초기 위치 못 보냄 (localization이 떠 있는지 확인)')
            return
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = p['map_frame']
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05                      # x, y 분산 [m^2]
        cov[35] = 0.05                              # yaw 분산 [rad^2]
        msg.pose.covariance = cov
        self.pose_pub.publish(msg)
        self.get_logger().info(f'initialpose 발행: {p["start_point"]} (map {x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°, '
                               f'구독자 {subscribers})')

    def localize(self, now):
        """[LOCALIZE] 초기 위치는 언독 전(도크 위)에 이미 보냈다 → 바로 START_FACE.
        undock 파라미터가 false라 아직 안 보냈으면 여기서 한 번 보낸다. AMCL 수렴은 기다리지 않는다.
        받음: now, (pose_sent)
        내보냄: (필요하면) initialpose → state START_FACE"""
        if self.pose_sent is None:
            self.send_initialpose()
        self.set_state('START_FACE')

    def face_start(self, now):
        """[START_FACE] 언독 지점에서 face_zone(키오스크) 쪽으로 제자리 회전. 관람객이 키오스크에서 오므로 카메라에 잡히게.
        map 위치(AMCL)를 쓰지 않는다: 언독 지점(init_pose.yaml face_from_point, 기본 undock)과 키오스크 좌표가 정해져 있으므로
        돌 각도 = (언독 지점에서 키오스크 방향 + face_offset_deg) - 언독 지점 yaw 를 처음 한 번 계산하고,
        odom yaw로 그만큼 돌았는지 잰다. face_timeout(15 s) 안에 못 돌면 그대로 WAIT_VISITOR.
        받음: now, (face_point, start, TF odom→base_link)
        내보냄: drive(ramp(0, w)), 다 돌면 state WAIT_VISITOR"""
        p = self.p
        if self.face_from is None:                  # 언독 지점을 모름 → 돌 각도를 계산할 수 없음
            self.get_logger().error(f'START_FACE 불가: init_pose.yaml에 {p["face_from_point"]}이 없음 → 회전 생략')
            return self.set_state('WAIT_VISITOR')
        pose = self.lookup(p['odom_frame'], p['base_frame'])
        if pose is None:                            # step [3]에서 걸러지지만 혹시 모를 때
            return self.drive(*self.ramp(0.0, 0.0))
        # odom이 늦게 오면 (Wi-Fi 지연) 지금 각도를 모르는 채 돌게 되어 지나쳐 돈다
        # (2026-11-19 bag: odom 0.8~1.5 s 지연 + 0.8 rad/s → 48° 대신 약 100° 회전)
        # → odom 값에 그 뒤 보낸 회전 명령만큼 더해 지금 각도를 추정 (predict_yaw). 너무 오래된 값이면 멈추고 기다림
        age = self.tf_age(p['odom_frame'], p['base_frame'])
        if age > p['face_tf_max_age']:
            if self.face_log_at is None or now - self.face_log_at >= 1.0:
                self.face_log_at = now
                self.get_logger().warn(f'START_FACE: odom이 {age:.1f} s 늦게 옴 → 멈추고 기다림 (지나쳐 돌지 않게)')
            self.dbg['reason'] = 'start_face_tf_old'
            if self.face_until is not None:
                self.face_until += self.p['control_period']   # 기다린 시간은 face_timeout에서 빼 줌
            return self.drive(0.0, 0.0)
        yaw = self.predict_yaw(pose[2], age, now)   # odom 지연만큼 보낸 명령으로 보정한 지금 각도
        if self.face_until is None:                 # 처음 한 번: 돌 각도 계산 → odom 목표 yaw
            self.face_until = now + p['face_timeout']
            sx, sy, syaw = self.face_from
            if self.face_point is not None:
                want = math.atan2(self.face_point[1] - sy, self.face_point[0] - sx)
            else:
                want = syaw
            turn = float(wrap(want + math.radians(p['face_offset_deg']) - syaw))
            self.face_goal = yaw + turn
            self.get_logger().info(f'START_FACE: 언독 지점 yaw {math.degrees(syaw):.0f}° → 키오스크 쪽 '
                                   f'{math.degrees(want):.0f}° (보정 {p["face_offset_deg"]:+.0f}°) '
                                   f'= {math.degrees(turn):+.0f}° 회전 (+ 왼쪽, odom 기준)')
        err = float(wrap(self.face_goal - yaw))
        self.dbg['reason'] = 'start_face'
        if abs(err) < p['face_tolerance'] and abs(self.cmd[1]) < 0.2:
            self.get_logger().info(f'시작 회전 끝 (오차 {math.degrees(err):.0f}°) → 관람객 인식 기다림')
            self.set_state('WAIT_VISITOR')
            return self.drive(*self.ramp(0.0, 0.0))
        if now > self.face_until:
            self.get_logger().warn(f'시작 회전 {p["face_timeout"]} s 안에 못 끝냄 (남은 {math.degrees(err):.0f}°) '
                                   '→ WAIT_VISITOR')
            self.set_state('WAIT_VISITOR')
            return self.drive(*self.ramp(0.0, 0.0))
        w_max = min(p['max_angular'], p['face_max_speed'])   # 천천히 돌아야 odom이 조금 늦어도 덜 지나침
        w = float(np.clip(1.5 * err, -w_max, w_max))
        if self.face_log_at is None or now - self.face_log_at >= 1.0:
            self.face_log_at = now
            subscribers = self.cmd_pub.get_subscription_count()
            level = self.get_logger().warn if not self.enabled or subscribers == 0 else self.get_logger().info
            level(f'START_FACE 회전 명령: 남은 {math.degrees(err):.1f}°, angular.z={w:.3f} rad/s, '
                  f'enabled={self.enabled}, cmd_vel 구독자={subscribers}')
        return self.drive(*self.ramp(0.0, w))

    def wait_visitor(self, now, pose, here):
        """[WAIT_VISITOR] 관람객이 register_confirm_time(1 s) 동안 끊김 없이(register_gap 0.5 s 안) 보이면 등록 → FOLLOW.
        확인하는 동안은 따라가지 않고 바라보기만 한다 (등록 안 된 사람을 따라가지 않게). 끊기면 처음부터 다시.
        받음: now, pose (x, y, yaw), here (x, y), (kf.x, seen, seen_since)
        내보냄: 끊김 → 정지 / 확인 중 → face() / 1 s 채움 → on_registered(), state REGISTERED (prepare_tour 성공)"""
        p = self.p
        if self.kf.x is None or now - self.seen > p['register_gap']:
            if self.kf.x is not None:
                self.get_logger().info('등록 확인 중 끊김 → 다시 기다림')
            self.kf.x, self.seen_since = None, None
            self.dbg['reason'] = 'wait_visitor'
            self.request_register(now)              # 클릭 없이: 인식 모듈에 바로 앞 사람 등록 요청
            return self.drive(*self.ramp(0.0, 0.0))
        if now - self.seen_since < p['register_confirm_time']:
            self.dbg['reason'] = 'register_confirm'
            last, vel = self.person()
            return self.face(pose, here, last + vel * (now - self.kf.t))
        self.on_registered(now - self.seen_since)
        self.ready_at = now
        self.set_state('REGISTERED')                # prepare_tour 성공 → start_tour 기다림
        return self.drive(*self.ramp(0.0, 0.0))

    def request_register(self, now):
        """[WAIT_VISITOR] 인식 모듈에 '바로 앞 사람을 등록' 요청 (target_person/register_front, Empty).
        좌표가 안 오는 동안 register_period(1 s)마다 다시 보낸다 (앞에 아무도 없으면 계속).
        인식 모듈(multi_tracker_node)이 정면 ±30°, 2.5 m 안 가장 가까운 사람을 타겟으로 잡고 좌표를 보내기 시작한다.
        받음: now, (auto_register, reg_first, reg_last)
        내보냄: target_person/register_front"""
        p = self.p
        if not p['auto_register']:
            return
        if self.reg_last is not None and now - self.reg_last < p['register_period']:
            return
        if self.reg_first is None:
            self.reg_first = now
            self.get_logger().info('바로 앞 사람 등록 요청 (target_person/register_front) → 로봇 앞에 서 주세요')
        self.reg_last = now
        self.register_pub.publish(Empty())

    def registered_wait(self, now):
        """[REGISTERED] 등록 완료 뒤 start_tour(키오스크 '관람 시작')를 그 자리에 서서 기다린다.
        관람 시작 전에는 사람 때문에 움직이지 않는다 (돌지도 않음). 좌표(kf.x)는 계속 받아 둔다
        → start_tour가 오면 바로 따라가고, 그사이 사라졌으면 FOLLOW → SEARCH로 마지막 위치를 찾아간다.
        ready_timeout 안에 start_tour가 안 오면 관람 끝 (not_started).
        받음: now, (ready_at)
        내보냄: 시간 초과 → end_tour(not_started) / 그 밖 → 정지"""
        if now - self.ready_at > self.p['ready_timeout']:
            self.get_logger().warn(f'등록 뒤 {self.p["ready_timeout"]:.0f} s 동안 start_tour 없음 → 관람 끝')
            return self.end_tour('READY', 'not_started')
        self.dbg['reason'] = 'registered'
        return self.drive(*self.ramp(0.0, 0.0))

    def on_registered(self, confirmed):
        """등록 완료 순간에 할 일 (한곳에). 나중에 인식 모듈이 등록 신호를 주면 부르는 조건만 바꾼다.
        받음: confirmed  끊김 없이 보인 시간 [s] (로그용)
        내보냄: guide_message recognized_guide("recognized") → 음성 '인식되었습니다'
        (키오스크 보고는 wait_visitor가 바꾸는 state REGISTERED를 run_prepare가 보고 feedback·result로)"""
        self.get_logger().info(f'관람객 등록 완료 ({confirmed:.1f} s 끊김 없이 보임)')
        if self.p['recognized_guide']:
            self.guide_pub.publish(String(data=self.p['recognized_guide']))   # "인식되었습니다"
