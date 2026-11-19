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
from irobot_create_msgs.action import Undock
import numpy as np
from std_msgs.msg import String

from arti_follow.motion import wrap


DOCK_STATUS_WAIT = 3.0     # 호출 때 dock_status를 이만큼 기다림 (노드를 막 켰을 때) [s]


class PrepareMixin:
    """VisitorFollower가 상속해서 쓰는 호출·등록 단계 함수."""

    def begin_tour(self):
        """관람 시작 준비: 지난 관람객 데이터(위치, 등록, 본 작품)를 지우고 active = True, 첫 상태로.
        바뀜: kf.x, seen, holding, 단계 타이머, explaining, visited, seen_since, active, end_reason
              state → UNDOCK (undock 파라미터 false면 LOCALIZE)"""
        self.kf.x, self.seen, self.holding = None, -1e9, False
        self.face_until = self.loc_at = self.pose_sent = None
        self.active, self.end_reason, self.ready_at = True, '', None
        self.explaining, self.visited = 0, []
        self.reported = 0                           # visited 중 ARTWORK_COMPLETED로 보고한 수
        self.seen_since = None
        self.get_logger().info('호출 → 관람 준비 시작 (undock → 위치 → 키오스크 → 등록)')
        self.set_state('UNDOCK' if self.p['undock'] else 'LOCALIZE')

    def run_prepare(self, goal):
        """[액션 실행] prepare_tour goal 하나: 호출 → undock → 위치 잡기 → 키오스크 보기 → 관람객 등록까지 (goal None = auto_start).
        begin_tour → undock → 0.2 s마다 feedback (취소 확인) → REGISTERED가 되면 성공.
        관람 중 호출은 검사하지 않는다: 관람 중이면 로봇이 도크 밖 → 키오스크가 [호출]을 끈다 (dock_status).
        start = false면 아무것도 안 하고 끝냄 (not_requested).
        움직임은 step()이 하고, 여기서는 시작·끝과 키오스크 보고만 한다.
        받음: goal (start, is_cancel_requested)
        돌려줌: PrepareTour.Result(success = 등록 완료, message = registered 또는 실패 이유)"""
        if goal is not None and not goal.request.start:
            self.get_logger().warn('호출 거절 (start = false)')
            goal.abort()
            return PrepareTour.Result(success=False, message='not_requested')
        with self.lock:
            self.begin_tour()
        if self.state == 'UNDOCK':
            self.send_prepare_feedback(goal)
            why = self.undock(goal)
            with self.lock:
                if why:
                    self.end_tour('READY', why)
                elif self.active:
                    self.set_state('LOCALIZE')
        while self.active and self.state != 'REGISTERED':
            if goal is not None and goal.is_cancel_requested:
                with self.lock:
                    self.end_tour('READY', 'canceled')
                break
            self.send_prepare_feedback(goal)
            time.sleep(0.2)
        if self.active and self.state == 'REGISTERED':
            self.send_prepare_feedback(goal)        # 마지막 feedback REGISTERED
            self.get_logger().info('관람 준비 끝 (등록 완료) → start_tour 기다림')
            if goal is not None:
                goal.succeed()
            return PrepareTour.Result(success=True, message='registered')
        reason = self.end_reason
        self.get_logger().info(f'관람 준비 실패: {reason}')
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
        if self.state == 'UNDOCK':
            state = 'UNDOCKING'
        elif self.state == 'WAIT_VISITOR':
            seeing = self.seen_since is not None and self.now() - self.seen <= self.p['register_gap']
            state = 'REGISTERING' if seeing else 'ARRIVED'
        elif self.state == 'REGISTERED':
            state = 'REGISTERED'
        else:                                       # READY·LOCALIZE·START_FACE
            state = 'MOVING'
        goal.publish_feedback(PrepareTour.Feedback(state=state))

    def undock(self, goal):
        """Create 3에 undock 액션을 보내고 끝날 때까지 기다린다 (로봇이 스스로 움직임).
        관람은 항상 도크에서 시작한다 → 도크 밖이면 거절 (undock 뒤 initialpose = 도크 앞이 맞으려면 도크에서 출발해야 함).
        받음: goal (취소 확인용), (docked: dock_status)
        돌려줌: '' (성공) / 'not_docked' (도크 밖, dock_status 못 받음) / 'no_undock_server' / 'undock_failed' / 'canceled'"""
        end = time.monotonic() + DOCK_STATUS_WAIT
        while self.docked is None and time.monotonic() < end:   # 노드를 막 켜서 dock_status를 아직 못 받았을 때
            time.sleep(0.1)
        if self.docked is not True:
            self.get_logger().error('도크에 붙어 있지 않음 (dock_status '
                                    f'{"없음" if self.docked is None else "is_docked=false"}) → 호출 거절. '
                                    '관람은 도크에서 시작한다')
            return 'not_docked'
        if not self.undock_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'undock 액션 서버 없음 ({self.get_namespace().rstrip("/")}/undock)')
            return 'no_undock_server'
        self.get_logger().info('undock 시작')
        sent = self.undock_client.send_goal_async(Undock.Goal())
        if not self.wait_future(sent, 5.0, goal) or not sent.result().accepted:
            return 'canceled' if goal is not None and goal.is_cancel_requested else 'undock_failed'
        handle = sent.result()
        done = handle.get_result_async()
        if not self.wait_future(done, self.p['undock_timeout'], goal):
            handle.cancel_goal_async()
            return 'canceled' if goal is not None and goal.is_cancel_requested else 'undock_failed'
        res = done.result()
        ok = (not res.result.is_docked) if res.status == GoalStatus.STATUS_SUCCEEDED else self.docked is False
        self.get_logger().info('undock 끝' if ok else f'undock 실패 (status {res.status})')
        return '' if ok else 'undock_failed'

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

    def localize(self, now):
        """[LOCALIZE] AMCL에 initialpose(출발 지점)를 보내고 map 위치가 그 근처로 잡힐 때까지 기다린다.
        로봇을 움직이지 않는다 (지도에 '여기 있다' 핀만 꽂음). 로봇이 이미 출발 지점(도크 앞)에 있다는 전제.
        받음: now, (start, TF map→base_link)
        내보냄: initialpose 한 번 → 잡히면 state START_FACE, 위치를 아예 못 잡으면 end_tour(no_localization)"""
        p = self.p
        if self.start is None or not p['set_initial_pose']:
            return self.set_state('START_FACE')     # 초기 위치를 안 보냄 → 지금 AMCL 위치 그대로
        if self.loc_at is None:
            self.loc_at = now
        x, y, yaw = self.start
        if self.pose_sent is None:
            if self.pose_pub.get_subscription_count() == 0:     # AMCL이 아직 initialpose를 구독하지 않음
                self.dbg['reason'] = 'localize_wait_amcl'
                if now - self.loc_at > p['localize_timeout']:
                    self.end_tour('READY', 'no_localization')
                return
            msg = PoseWithCovarianceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = p['map_frame']
            msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            cov = [0.0] * 36
            cov[0] = cov[7] = 0.05                  # x, y 분산 [m^2]
            cov[35] = 0.05                          # yaw 분산 [rad^2]
            msg.pose.covariance = cov
            self.pose_pub.publish(msg)
            self.pose_sent = now
            self.get_logger().info(f'초기 위치 보냄 (map {x:.2f}, {y:.2f}, {math.degrees(yaw):.0f}°)')
        self.dbg['reason'] = 'localize'
        pose = self.lookup(p['map_frame'], p['base_frame'])
        near = pose is not None and math.hypot(pose[0] - x, pose[1] - y) < p['localize_tolerance']
        if near and now - self.pose_sent > 1.0:     # AMCL이 새 위치로 map→odom을 낼 시간
            self.get_logger().info('초기 위치 잡힘 → 그 자리에서 바라볼 곳(face_zone) 쪽으로 회전')
            return self.set_state('START_FACE')
        if now - self.loc_at > p['localize_timeout']:
            if pose is None:
                return self.end_tour('READY', 'no_localization')
            self.get_logger().warn(f'map 위치가 출발 지점에서 {math.hypot(pose[0] - x, pose[1] - y):.2f} m '
                                   '떨어져 있음 (AMCL 확인) → 그래도 바라볼 곳 쪽으로 회전')
            self.set_state('START_FACE')

    def face_start(self, now):
        """[START_FACE] 제자리 회전으로 face_zone(키오스크) 쪽을 본다. 관람객이 키오스크에서 오므로 카메라에 잡히게.
        map 좌표 필요 (그래서 LOCALIZE 뒤). face_timeout(15 s) 안에 못 돌면 포기하고 WAIT_VISITOR.
        받음: now, (face_point, start, TF map→base_link)
        내보냄: drive(ramp(0, w)), 다 돌면 state WAIT_VISITOR"""
        p = self.p
        if self.face_point is None and self.start is None:   # 바라볼 곳을 모름 → 돌지 않음
            return self.set_state('WAIT_VISITOR')
        if self.face_until is None:
            self.face_until = now + p['face_timeout']
        pose = self.lookup(p['map_frame'], p['base_frame'])
        if pose is None or now > self.face_until:
            if now > self.face_until:
                self.get_logger().warn(f'시작 회전 {p["face_timeout"]} s 안에 못 끝냄'
                                       + (' (map TF 없음: localization 확인)' if pose is None else '')
                                       + ' → WAIT_VISITOR')
                self.set_state('WAIT_VISITOR')
            self.dbg['reason'] = 'start_face_wait'
            return self.drive(*self.ramp(0.0, 0.0))
        x, y, yaw = pose
        if self.face_point is not None:             # 바라볼 곳(키오스크 등) 쪽
            want = math.atan2(self.face_point[1] - y, self.face_point[0] - x)
        else:                                       # 없으면 출발 지점의 방향
            want = self.start[2]
        err = float(wrap(want - yaw))
        self.dbg['reason'] = 'start_face'
        if abs(err) < p['face_tolerance'] and abs(self.cmd[1]) < 0.2:
            self.get_logger().info(f'시작 회전 끝 (오차 {math.degrees(err):.0f}°) → 관람객 인식 기다림')
            self.set_state('WAIT_VISITOR')
            return self.drive(*self.ramp(0.0, 0.0))
        w = float(np.clip(1.5 * err, -p['max_angular'], p['max_angular']))
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
            return self.drive(*self.ramp(0.0, 0.0))
        if now - self.seen_since < p['register_confirm_time']:
            self.dbg['reason'] = 'register_confirm'
            last, vel = self.person()
            return self.face(pose, here, last + vel * (now - self.kf.t))
        self.on_registered(now - self.seen_since)
        self.ready_at = now
        self.set_state('REGISTERED')                # prepare_tour 성공 → start_tour 기다림
        return self.drive(*self.ramp(0.0, 0.0))

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
