"""② 관람 = start_tour 액션 (visitor_follower 노드의 일부). 키오스크 "관람 시작" 버튼 (prepare_tour 성공 뒤).

  run_tour       액션 실행: REGISTERED 확인 → FOLLOW → 관람 끝까지 feedback → result
  상태별 동작 (step이 부름)  FOLLOW follow · SEARCH/WAIT/LOST search
  end_tour       관람 끝 (출구 completed·취소·실패·LOST·not_started 공통): 정지, 관람객 데이터 삭제, arti/tour_end
  on_explaining·on_explained   artwork_zone 작품 설명 → feedback EXPLAINING / ARTWORK_COMPLETED
"""
import math
import time

from arti_interfaces.action import StartTour
from irobot_create_msgs.action import Dock
import numpy as np
from std_msgs.msg import Empty, String

from arti_follow.motion import apply, wrap

RETURNING = ('RETURN', 'DOCK_FACE', 'DOCK')   # 관람 끝 → 도크 복귀 중
TRACKING = ('FOLLOW', 'SEARCH', 'WAIT')     # 관람 중 = 관람객을 따라가거나 찾는 상태 (start_tour 뒤)


class TourMixin:
    """VisitorFollower가 상속해서 쓰는 관람 단계 함수."""

    def run_tour(self, goal):
        """[액션 실행, 관람 내내] start_tour goal 하나: prepare_tour로 등록된 관람객을 끝까지 따라간다 (goal None = auto_start).
        FOLLOW → 0.2 s마다 feedback (취소 확인) → active가 내려가면 result.
        키오스크는 prepare_tour 성공 직후 start_tour를 자동 요청한다. 자유 관람의 artwork_ids는 비어 있다.
        움직임은 step()이 하고, 여기서는 시작·끝과 키오스크 보고만 한다.
        받음: goal (is_cancel_requested)
        돌려줌: StartTour.Result(success = 취소·출구 완료로 끝남, message = 끝난 이유: completed / canceled / lost ...)"""
        with self.lock:
            self.get_logger().info('관람 시작 → 추종')
            self.tour_at, self.end_at = self.now(), None
            self.set_state('FOLLOW')                # 다음 주기부터 step이 따라감 (안 보이면 SEARCH로)
        while self.active:
            if goal is not None and goal.is_cancel_requested and self.state not in RETURNING:
                with self.lock:
                    self.end_tour('READY', 'canceled')   # 돌아가 도킹할 때까지 루프가 계속 돌며 RETURNING feedback
            self.send_feedback(goal)
            time.sleep(0.2)
        reason = self.end_reason
        result = StartTour.Result(success=reason in ('canceled', 'completed'), message=reason)
        with self.lock:
            visited, self.visited = self.visited, []   # 관람 기록은 로그로 남기고 지운다 (관람객 데이터 남기지 않음)
        self.get_logger().info(f'관람 결과: success={result.success}, message={reason}, '
                               f'설명 완료 {visited}')
        if goal is not None:
            if reason == 'canceled':
                goal.canceled()
            elif reason == 'completed':
                goal.succeed()                      # 출구까지 관람 끝 (robot4 코스 완료와 같은 message)
            else:
                goal.abort()
        return result

    def send_feedback(self, goal):
        """내부 상태 → 키오스크용 feedback (StartTour.action, docs/common/ros2_interfaces.md).
        새로 끝까지 설명한 작품이 있으면 → ARTWORK_COMPLETED + 그 작품 번호 (작품마다 한 번)
        설명 중 → EXPLAINING, 그 밖 → FOLLOW (관람 중 상태는 FOLLOW·SEARCH·WAIT·LOST뿐).
        받음: goal, (state, explaining, visited, reported)
        내보냄: feedback (state, artwork_id)"""
        if goal is None:
            return
        if self.state in RETURNING:
            goal.publish_feedback(StartTour.Feedback(state='RETURNING', artwork_id=0))
            return
        if self.reported < len(self.visited):
            done = self.visited[self.reported]
            self.reported += 1
            goal.publish_feedback(StartTour.Feedback(state='ARTWORK_COMPLETED', artwork_id=done))
            return
        explaining = self.explaining if self.state in TRACKING else 0
        goal.publish_feedback(StartTour.Feedback(
            state='EXPLAINING' if explaining else 'FOLLOW', artwork_id=explaining))

    def follow(self, now, pose, here):
        """[FOLLOW] step [6]에서: 관람객 위치를 알고 마지막으로 본 지 lost_timeout(2.5 s) 안 → target_distance(0.8 m) 유지하며 따라간다.
        멀면 dwa로 이동, 0.85 m 안이면 서서 몸만 돌림 (0.95 m 밖이 되면 다시 출발).
        받음: now, pose (x, y, yaw), here (x, y), (kf.x·kf.t, holding)
        내보냄: set_state(FOLLOW), drive(dwa(...)) 또는 face()"""
        p = self.p
        self.set_state('FOLLOW')
        last, vel = self.person()
        # 마지막 좌표 뒤 흐른 시간만큼 → 지금 위치. 좌표가 끊긴 동안은 predict_max까지만 앞으로 밀어 봄
        # (lost_timeout 2.5 s 동안 오래된 속도로 엉뚱한 곳까지 쫓아가지 않게)
        person = last + vel * min(now - self.kf.t, p['predict_max'])
        ahead = person + vel * p['lead_time']       # 0.5 s 뒤 위치: 인식·통신·가속이 늦는 만큼 미리 향함
        to = ahead - here
        d = np.linalg.norm(to)
        stop = p['target_distance'] + p['distance_tolerance']
        # [7] 사람까지 거리 d가 0.85 m 이하인가? → 예: holding = True (서기)
        # [8] 아니면 0.95 m 넘나? → 예: holding = False (다시 출발) / 사이: 그대로
        # 0.85 m 안이면 서기, 0.95 m 밖이면 다시 출발. 기준을 둘로 나눠
        # 0.8 m 근처에서 좌표가 조금 흔들릴 때 섰다 갔다 덜덜 떨지 않게 한다 (사이 구간은 하던 대로)
        if d <= stop:
            self.holding = True
        elif d > p['target_distance'] + p['restart_margin']:
            self.holding = False
        # [9] 서 있는 중이 아닌가? (holding == False) → 예: dwa로 따라가기 / 아니오: 아래 face()
        # 멀면: 사람에서 로봇 쪽으로 0.8 m 돌아온 점을 목표로 장애물 피해서 이동 (사람 자체를 목표로 하면 부딪힘)
        if not self.holding:
            goal = self.reachable_goal(ahead - to / d * p['target_distance'])   # 벽에 붙은 목표면 밀어냄 (우회 가능하게)
            self.dbg['reason'] = 'follow'
            # 화면 유지 벌점은 카메라로 view_fresh_time 안에 본 관람객일 때만: 예측·라이다 보조·벽 좌표 대체로만 이어 가는 중에는
            # 그 위치를 화면에 두려고 크게 돌 이유가 약함 (jsbzzz feat/follow-perception-update, 2026-11-21)
            keep_view = now - self.cam_seen <= p['view_fresh_time']
            return self.drive(*self.dwa(pose, goal, ahead, keep_view=keep_view))
        # 가까우면: 앞으로는 안 가고 사람 쪽으로 몸만 돌림 (작품 감상 중 등)
        self.dbg['reason'] = 'hold'
        return self.face(pose, here, person)

    def search(self, now, pose, here):
        """[SEARCH → WAIT → LOST] step [10] 다음: 관람객을 놓침 (lost_timeout 넘게 좌표가 안 옴, kf.x는 있음).
        마지막 위치로 이동(도착·막힘·search_move_time 상한까지) → 거기서 odom 기준 한 바퀴(360°) 회전 → wait_time 동안 정지 대기
        → LOST (관람 끝). 좌표가 다시 오면 step [6]이 FOLLOW로 돌린다.
        받음: now, pose, here, (kf.x, lost_at, spin_at, spun, spin_yaw, wait_at, turn)
        내보냄: set_state(SEARCH/WAIT), drive(...), 끝나면 end_tour(LOST, lost)"""
        p = self.p
        # ---- 여기부터 = 사람을 놓침 (lost_timeout 넘게 좌표가 안 옴) ----
        last, vel = self.person()
        # [11] 방금 놓쳤나? (직전 상태가 FOLLOW) → 예: 놓친 시각·회전 방향 기억 (한 번만) / 아니오: 건너뜀
        # 방금 놓친 순간(직전까지 FOLLOW)에만 한 번: 놓친 시각과 사람이 가던 쪽(왼/오른)을 기억 → 나중에 그쪽으로 돈다
        if self.state == 'FOLLOW':
            prev_path = self.path_dist              # 놓치기 직전 추종 경로 거리 (벽을 돌아가던 중이었나 판단)
            self.lost_at, self.spin_at, self.wait_at = now, None, None
            self.best_dist, self.progress_at = None, now   # 이동 진척 (가장 짧았던 경로 거리, 그때 시각)
            self.path_dist = None                   # 이전 dwa(추종) 값은 목표가 달라서 버림
            # 탐색 목표 = 놓친 순간의 위치 (탐색 중 확인 전 새 좌표로 바뀌지 않게, A4).
            # 걷던 중(search_goal_ext_speed 넘게) 놓쳤으면 걷던 방향으로 최대 search_goal_ext_max 더 감 (모퉁이 너머로 사라진 경우)
            # (jsbzzz feat/follow-perception-update, 2026-11-21)
            ext = vel * min(p['predict_max'], 1.0) if float(np.linalg.norm(vel)) > p['search_goal_ext_speed'] else np.zeros(2)
            if np.linalg.norm(ext) > p['search_goal_ext_max']:
                ext *= p['search_goal_ext_max'] / float(np.linalg.norm(ext))
            # 벽 모서리에 붙어 있으면 로봇이 갈 수 있는 곳으로 밀어냄 → 길 찾기가 모서리를 돌아가는 길을 찾음
            self.search_goal = np.asarray(self.reachable_goal(last + ext), float)
            d = last + vel - here
            self.turn = 1.0 if wrap(math.atan2(d[1], d[0]) - pose[2]) >= 0 else -1.0
            # 먼저 돌아볼 곳 = 사람이 가던 쪽 (마지막 위치 + 속도 × search_look_ahead).
            # 놓친 이유는 대부분 사람이 탑캠 화면 옆으로 빠진 것 → 마지막 자리로 직진하면 카메라는 '있던 곳'을 봄
            # (2026-11-20: 다시 찾는 데 9~23 s). 서 있다 놓쳤으면(속도≈0) 마지막 자리 쪽을 본다.
            # 벽을 돌아가던 중(경로 거리가 직선보다 detour_margin 넘게 김)에 놓쳤으면 돌아보기를 건너뛰고 바로 이동:
            # 벽 앞에서 벽 너머를 돌아봐야 보이지 않음 (jsbzzz feat/follow-perception-update, 2026-11-21)
            straight = float(np.linalg.norm(last - here))
            detouring = prev_path is not None and prev_path < 49.0 and prev_path > straight + p['detour_margin']
            self.look_at = None if detouring else last + vel * p['search_look_ahead']
            self.get_logger().info(f'관람객 놓침 → {"벽을 돌아가던 중이라 바로 이동" if detouring else "가던 쪽으로 먼저 돌아봄"}, '
                                   f'그다음 마지막 위치까지 이동 ({straight:.2f} m)')
        # [11-1] 가던 쪽을 아직 안 돌아봤나? (look_at != None) → 제자리 회전, 그쪽을 보거나 search_look_time 지나면 끝
        if self.look_at is not None and self.spin_at is None:
            off = wrap(math.atan2(self.look_at[1] - here[1], self.look_at[0] - here[0]) - pose[2])
            if abs(off) > p['face_deadband'] and now - self.lost_at < p['search_look_time']:
                self.set_state('SEARCH')
                self.dbg['reason'] = 'search_look'
                return self.face(pose, here, self.look_at)
            self.look_at = None
            self.progress_at = now                  # 돌아보는 동안은 이동 막힘 판정에 넣지 않음
            self.get_logger().info(f'가던 쪽 돌아보기 끝 ({now - self.lost_at:.1f} s) → 마지막 위치까지 이동')
        # [12] 아직 회전 탐색 전인가? (spin_at == None) → 예: 마지막 위치로 이동 또는 회전 준비 / 아니오: 아래로
        # 1) 마지막으로 본 곳(last, 관람객 위치)까지 간다 (기둥·모퉁이 뒤로 사라졌으면 가야 보인다).
        #    끝나는 조건 셋 중 하나: 도착(직선거리 search_arrive 안) / 막힘(search_stall_time 동안 경로 거리가
        #    search_stall_dist만큼도 안 줄어듦 — 길이 없음, 또는 인식 안 되는 사람이 그 자리에 서 있음) / 안전 상한(search_move_time)
        #    진척은 직선거리가 아니라 경로 거리(dwa가 구한 '장애물을 돌아가는 거리')로 잰다:
        #    벽을 돌아가느라 직선거리가 잠깐 늘어도 경로 거리는 줄어드므로 막힘으로 잘못 보지 않는다
        if self.spin_at is None:
            last = self.search_goal if self.search_goal is not None else last
            dist = float(np.linalg.norm(last - here))
            path = self.path_dist                   # 지난 주기 dwa 값. None = 못 구함(전부 막힘·먼저 회전 중)
            reachable = path is not None and path < 49.0   # 약 50 = 길이 안 보임 → 진척 아님 (3 s 뒤 막힘)
            if reachable and (self.best_dist is None or path < self.best_dist - p['search_stall_dist']):
                self.best_dist, self.progress_at = path, now      # 경로 거리가 줄어듦 = 진척
            elif self.turning_first:
                self.progress_at = now              # 길 방향이 뒤쪽이라 먼저 도는 중 (거리는 안 줄어도 진척, go_to와 같은 기준)
            cap = p['search_move_time']             # 0이면 시간 상한 없음 (막힘 판정은 그대로)
            if dist <= p['search_arrive']:
                why = '도착'
            elif now - self.progress_at > p['search_stall_time']:
                why = f'{p["search_stall_time"]:.0f} s 동안 경로 거리 안 줄어듦 (막힘)'
            elif cap > 0 and now - self.lost_at >= cap:
                why = f'{cap:.0f} s 상한'
            else:
                self.set_state('SEARCH')
                self.dbg['reason'] = 'search_move'
                return self.go_to(pose, last)       # 목표가 뒤쪽이면 먼저 돌고, 아니면 DWA
            self.get_logger().info(f'마지막 위치 이동 끝: {why}, 남은 {dist:.2f} m, {now - self.lost_at:.1f} s → 회전 탐색')
            self.spin_at, self.spun, self.spin_yaw = now, 0.0, pose[2]   # 도착 → 여기서 한 바퀴
            self.spin_speed = float(np.clip(2 * math.pi / max(p['search_time'], 1e-3),
                                            p['search_spin'], p['max_angular']))
        # [13] 아직 한 바퀴를 다 안 돌았나? (odom yaw로 돈 각도를 잼) → 예: 제자리 회전 / 아니오: 정지 대기 시작
        # 2) 도착 지점에서 일정 속도(2π/search_time)로 한 바퀴: 카메라 화각 밖으로 나간 사람을 다시 화면에 넣으려고.
        #    사람이 가던 쪽부터 돈다. 시간이 아니라 실제로 돈 각도로 끝냄 (Create 3 회전 속도가 명령과 달라서)
        if self.wait_at is None:
            self.spun += abs(float(wrap(pose[2] - self.spin_yaw)))
            self.spin_yaw = pose[2]
            if self.spun < 2 * math.pi and now - self.spin_at < p['search_time'] + p['search_spin_extra']:
                self.set_state('SEARCH')
                self.dbg['reason'] = 'search_spin'
                return self.drive(*self.ramp(0.0, self.turn * self.spin_speed))
            self.wait_at = now
            self.get_logger().info(f'회전 탐색 끝: {math.degrees(self.spun):.0f}° / {now - self.spin_at:.1f} s → 정지 대기')
        # [14] 정지 대기 wait_time이 지났나? → 예: 관람 끝 LOST / 아니오: 아래 WAIT
        if now - self.wait_at >= p['wait_time']:
            if p['lost_guide']:                     # "관람객님을 찾지 못해 안내를 마칩니다" (요구사항: 분실 → 대기 → 안내)
                self.guide_pub.publish(String(data=p['lost_guide']))
            return self.end_tour('LOST', 'lost')
        # 3) 회전으로도 못 찾음 → 멈춰서 기다림 (관람객이 로봇 쪽으로 와 주길). 좌표가 다시 오면 위 FOLLOW로 돌아감
        self.set_state('WAIT')
        self.dbg['reason'] = 'wait'
        self.drive(*self.ramp(0.0, 0.0))

    def end_tour(self, state, reason):
        """관람 끝: 정지 → 관람객 데이터 삭제 → arti/tour_end 발행(다른 노드도 삭제) → state.
        받음: state   다음 상태 'READY' (취소·실패) 또는 'LOST' (분실)
              reason  끝난 이유 → 액션 result.message ('canceled', 'lost', 'undock_failed' ...)
        바뀜: active = False → run_prepare·run_tour 루프가 끝나 result를 보냄
        내보냄: cmd_vel 0, arti/tour_end, follow/state"""
        self.end_reason, self.end_at = reason, None
        self.drive(0.0, 0.0)
        self.kf.x, self.kf.P, self.seen, self.holding = None, None, -1e9, False
        self.anchor = self.search_goal = self.run_since = None
        self.tour_end_pub.publish(Empty())
        self.get_logger().info(f'관람 끝 ({reason}) → 관람객 데이터 삭제 (arti/tour_end)')
        if self.p['return_to_dock'] and self.home is not None and self.docked is not True \
                and not self.shutting_down:         # 노드가 꺼지는 중이면 돌아갈 수 없음 → 바로 끝
            # 언독했으면 돌아가 도킹한 뒤에 끝 (active는 그때 내림 → 액션 result도 도킹 뒤)
            self.final_state, self.ret_at, self.dock_future, self.dock_result = state, None, None, None
            self.ret_best = self.ret_progress_at = self.ret_warn_at = None
            self.return_lidar_only, self.turning_first, self.path_dist = False, False, None
            self.get_logger().info('언독 지점으로 돌아가 도킹 (RETURN)')
            return self.set_state('RETURN')
        self.active = False
        self.set_state(state)

    def go_home(self, now, pose):
        """[RETURN → DOCK_FACE → DOCK] step [3-1]: 관람이 끝난 뒤 도크로 돌아간다.
        목표는 return_target(): 지도의 언독 지점·도크(init_pose.yaml)를 지금 AMCL(map→odom)로 odom 좌표로 바꾼 값.
        AMCL이 없으면 예전처럼 언독 때 기록한 odom 값 (관람 중 바퀴 오차가 쌓여 20~30 cm 어긋남, 2026-11-20).
        RETURN     언독 지점까지 go_to. 도착(return_arrive 안)해야만 다음으로 (return_watch: 막히면 경고·지도 벽 빼고 계속).
                   멀리서 dock을 부르면 도크 IR이 안 닿음 (2026-11-21: 3.7 m 떨어진 곳에서 도킹 시도)
        DOCK_FACE  언독 지점에서 도크 쪽으로 제자리 회전
        DOCK       Create 3 dock 액션 (Create 3가 IR로 도크를 찾아 스스로 올라감) → 끝나면 finish_return()
        받음: now, pose (x, y, yaw) odom, (home, ret_at, dock_future)
        내보냄: drive(...), dock 액션, 끝나면 active = False, state = final_state"""
        p = self.p
        hx, hy, face_yaw, src = self.return_target()  # 매 주기 다시 계산 (AMCL이 보정하는 만큼 따라감)
        if self.ret_at is None:
            self.ret_at = now
            self.get_logger().info(f'도크 복귀 목표: 언독 지점 ({src} 기준) odom ({hx:.2f}, {hy:.2f}), '
                                   f'지금 {math.hypot(hx - pose[0], hy - pose[1]):.2f} m')
        if self.state == 'RETURN':
            dist = math.hypot(hx - pose[0], hy - pose[1])
            close = ''
            if dist > p['return_arrive']:
                self.dbg['reason'] = 'return'
                self.go_to(pose, (hx, hy))
                # return_near 안인데 더 못 다가가면 도착으로 본다 (2026-11-21 13:43: 언독 지점이 도크·벽에서 robot_radius
                # 여유 안이라 길 찾기가 목표를 로봇 앞으로 옮김(goal_snapped, 경로 0.05 m) → DWA는 '다 왔다'며 (0, 0),
                # 도착 판정은 0.15 m라 0.18 m 앞에서 90 s 넘게 서 있음). 도크 IR은 이 거리에서 충분히 닿는다
                if dist <= p['return_near']:
                    if self.dbg.get('goal_snapped') and self.path_dist is not None \
                            and self.path_dist < p['return_snap_done']:
                        close = ', 언독 지점이 장애물 여유 안이라 더 못 다가감'
                    elif self.ret_progress_at is not None and now - self.ret_progress_at >= p['return_stall_time']:
                        close = f', {p["return_stall_time"]:.0f} s 동안 더 못 다가감'
                if not close:
                    return self.return_watch(now, dist)
            self.get_logger().info(f'언독 지점 도착 (남은 {dist:.2f} m, {now - self.ret_at:.0f} s 걸림'
                                   f'{", 지도 벽 빼고 옴" if self.return_lidar_only else ""}{close}) → 도크 쪽으로 돌기')
            self.return_lidar_only = False
            self.ret_at = now
            self.set_state('DOCK_FACE')
        if self.state == 'DOCK_FACE':
            err = float(wrap(face_yaw - pose[2]))
            if abs(err) > p['face_tolerance'] and now - self.ret_at < p['face_timeout']:
                self.dbg['reason'] = 'dock_face'
                w = float(np.clip(1.5 * err, -p['max_angular'], p['max_angular']))
                return self.drive(*self.ramp(0.0, w))
            self.drive(0.0, 0.0)
            self.ret_at = now
            self.set_state('DOCK')
        # DOCK: cmd_vel은 보내지 않는다 (Create 3가 스스로 움직임)
        self.dbg['reason'] = 'dock'
        if self.dock_future is None:
            if not self.dock_client.server_is_ready():
                if now - self.ret_at > 5.0:
                    self.get_logger().error('dock 액션 서버 없음 → 도킹 못 함 (로봇을 손으로 도크에 올리기)')
                    return self.finish_return('no_dock_server')
                return
            self.get_logger().info('Create 3 dock 요청')
            self.dock_future = self.dock_client.send_goal_async(Dock.Goal())
            return
        if self.docked is True and now - self.ret_at > 2.0 and self.dock_result is None:
            return self.finish_return('docked')     # dock_status로 확인 (결과 응답이 늦을 때)
        if self.dock_future.done() and self.dock_result is None:
            handle = self.dock_future.result()
            if not handle.accepted:
                return self.finish_return('dock_rejected')
            self.dock_result = handle.get_result_async()
        if self.dock_result is not None and self.dock_result.done():
            res = self.dock_result.result()
            return self.finish_return('docked' if res.result.is_docked or self.docked else f'dock_status_{res.status}')
        if now - self.ret_at > p['dock_timeout']:
            return self.finish_return('dock_timeout')

    def return_watch(self, now, dist):
        """[RETURN] 진척 확인. 경로 거리(path_dist, 길이 없으면 직선 거리)가 return_stall_time 동안
        search_stall_dist만큼도 안 줄면 막힘: 이유를 경고하고, 처음 막혔을 때 지도 벽을 빼고(라이다만) 계속 간다
        (AMCL이 어긋나 지도 벽이 통로를 막는 경우, 2026-11-21). 그래도 막히면 return_stall_time마다 경고만 하고 계속 시도.
        도킹은 언독 지점에 도착한 뒤에만 한다. return_timeout마다 '아직 못 감' 경고.
        받음: now, dist 언독 지점까지 직선 거리 [m], (path_dist, turning_first, dbg: go_to·dwa가 이번 주기에 남긴 것)
        바뀜: ret_best, ret_progress_at, ret_warn_at, return_lidar_only"""
        p = self.p
        path = self.path_dist if self.path_dist is not None and self.path_dist < 49.0 else None
        measure = path if path is not None else dist
        if self.ret_progress_at is None or measure < self.ret_best - p['search_stall_dist']:
            self.ret_best, self.ret_progress_at = measure, now
            return
        if self.ret_warn_at is None or now - self.ret_warn_at >= p['return_timeout']:
            if now - self.ret_at >= p['return_timeout']:
                self.ret_warn_at = now
                self.get_logger().warn(f'언독 지점까지 아직 {dist:.2f} m ({now - self.ret_at:.0f} s째) → 도킹하지 않고 계속 감')
        if now - self.ret_progress_at < p['return_stall_time']:
            return
        if path is None:
            why = '길 없음'
        elif self.dbg.get('blocked'):
            why = '모든 궤적이 장애물에 걸림'
        elif self.turning_first:
            why = '길 방향으로 도는 중'
        else:
            why = '경로 거리가 안 줄어듦'
        if not self.return_lidar_only and self.map_obs is not None:
            self.return_lidar_only = True
            self.field_cache = None
            self.get_logger().warn(f'도크 복귀 막힘 ({why}, {p["return_stall_time"]:.0f} s, 남은 {dist:.2f} m) '
                                   f'→ 지도 벽 빼고 라이다만 보며 언독 지점까지 계속 감')
        else:
            self.get_logger().warn(f'도크 복귀 막힘 ({why}, {p["return_stall_time"]:.0f} s, 남은 {dist:.2f} m, '
                                   f'라이다만) → 계속 시도 (주변 사람·물체 확인)')
        self.ret_best, self.ret_progress_at = measure, now

    def return_target(self):
        """도크 복귀 목표 (odom): 언독 지점 (x, y)과 거기서 도크를 바라볼 방향 yaw.
        return_use_map이고 AMCL TF(map→odom)가 있으면 init_pose.yaml의 언독 지점(face_from)·도크(start)를
        지도 좌표 → odom으로 바꿔 쓴다 (AMCL이 라이다로 보정한 위치라 바퀴 오차가 쌓이지 않음).
        없으면 언독 때 기록한 odom 위치(home)와 그 반대 방향.
        돌려줌: (x, y, yaw, 'map' 또는 'odom')"""
        if self.p['return_use_map'] and self.face_from is not None and self.start is not None:
            f = self.lookup(self.p['odom_frame'], self.p['map_frame'])
            if f is not None:
                (ux, uy), (dx, dy) = apply(f, np.array([self.face_from[:2], self.start[:2]], float))
                return float(ux), float(uy), math.atan2(dy - uy, dx - ux), 'map'
        hx, hy, hyaw = self.home
        return hx, hy, hyaw + math.pi, 'odom'

    def finish_return(self, how):
        """도크 복귀 끝: 로그 → active = False (액션 result가 나감) → final_state.
        받음: how  'docked' / 'dock_timeout' / 'dock_rejected' / 'no_dock_server' ..."""
        text = f'도크 복귀 끝 ({how}, dock_status is_docked={self.docked}) → 관람 결과 {self.end_reason}'
        if how == 'docked':                         # 레벨마다 호출 줄을 따로 (rclpy: 같은 줄에서 레벨이 바뀌면 ValueError)
            self.get_logger().info(text)
        else:
            self.get_logger().warn(text)
        self.home, self.dock_future, self.dock_result, self.return_lidar_only = None, None, None, False
        self.active = False
        self.set_state(self.final_state)

    def on_zone_entered(self, msg):
        """[구독] artwork_zone/entered (String) — 관람객이 들어간 안내 구역 zone_id.
        관람 중(FOLLOW·SEARCH·WAIT)이고 end_zone(exit)이며 start_tour 뒤 end_min_time이 지났으면
        end_guide_wait 뒤에 관람 끝 (step [3-2]). 종료 안내 음성은 artwork_zone이 guide_message로 이미 보냄.
        받음: msg.data  'exit' / 'kiosk' ...
        바뀜: end_at (관람 끝 예정 시각)"""
        if not self.p['end_zone'] or msg.data != self.p['end_zone']:
            return
        with self.lock:
            self.exit_reached('진입')

    def exit_reached(self, how):
        """관람객이 출구(end_zone)에 들어감/지나감 → end_guide_wait 뒤 관람 끝 예약. lock을 잡은 채 부른다.
        받음: how  '진입' (artwork_zone/entered) 또는 '통과 …' (end_cross_check)
        바뀜: end_at
        돌려줌: 예약했으면 True"""
        p = self.p
        if self.end_at is not None or not self.active or self.state not in TRACKING or self.tour_at is None:
            return False
        now = self.now()
        if now - self.tour_at < p['end_min_time']:
            self.log_every('end_early', 5.0, f'출구({p["end_zone"]}) {how}했지만 관람 시작 {now - self.tour_at:.0f} s 뒤라 무시 '
                                              f'(end_min_time {p["end_min_time"]:.0f} s)', 'info')
            return False
        self.end_at = now + p['end_guide_wait']
        self.get_logger().info(f'관람객 출구({p["end_zone"]}) {how} → {p["end_guide_wait"]:.0f} s 종료 안내 뒤 관람 끝 → 도크 복귀')
        return True

    def end_cross_check(self, now):
        """[step 3-2 앞] 추적 중인 관람객 위치(칼만, 튄 좌표 확인·라이다 짝을 거친 값)가 지난 주기 뒤 출구 구역을
        지나갔는지 선분으로 판정한다. artwork_zone은 인식 좌표 한 점만 보므로, 좌표가 끊기거나 튀어 폭 0.55 m인
        출구 띠를 건너뛰면 관람이 끝나지 않고 복귀도 안 함 (2026-11-21 14:10·14:17: 출구 안 좌표 0개,
        y 2.48 → 1.15를 0.5 s에 건너뜀). 선분 길이가 end_cross_max를 넘으면(다른 사람으로 바뀐 순간 등) 판정 안 함.
        출구에서 종료 안내 음성(tour_end)은 artwork_zone이 못 냈을 수 있으므로 여기서 guide_message로 보냄.
        받음: now, (kf, seen, end_zone, end_prev, TF map→odom)
        바뀜: end_prev, (exit_reached → end_at)"""
        p = self.p
        if self.end_zone is None or self.end_at is not None or self.state not in TRACKING or self.kf.x is None \
                or now - self.seen >= p['lost_timeout']:
            self.end_prev = None
            return
        f = self.lookup(p['map_frame'], p['odom_frame'])
        if f is None:
            return
        last, _ = self.person()
        q = apply(f, last[None])[0]               # apply는 점 배열 (N, 2)
        prev, self.end_prev = self.end_prev, (q, now)
        if prev is None or now - prev[1] > p['end_cross_gap']:
            return
        seg = float(np.hypot(*(q - prev[0])))
        if seg > p['end_cross_max']:
            return
        n = max(2, int(math.ceil(seg / 0.05)) + 1)
        if not any(self.end_zone.contains(prev[0] + (q - prev[0]) * k / (n - 1)) for k in range(n)):
            return
        if self.exit_reached(f'통과 (추적 위치 ({prev[0][0]:.2f}, {prev[0][1]:.2f}) → ({q[0]:.2f}, {q[1]:.2f}))'):
            guide = self.end_zone.guide
            if guide:
                self.guide_pub.publish(String(data=guide))   # artwork_zone이 먼저 잡았으면 end_at이 있어 여기 안 옴

    def on_explaining(self, msg):
        """[구독] artwork_zone/explaining (Int32) — 설명 중인 작품 번호. 관람 중일 때만 저장.
        받음: msg.data  2 또는 0 (설명 끝)
        바뀜: explaining → send_feedback의 EXPLAINING"""
        self.explaining = msg.data if self.active else 0

    def on_explained(self, msg):
        """[구독] artwork_zone/explained (Int32) — 끝까지 설명한 작품 번호. 관람 중이고 처음일 때만 추가.
        받음: msg.data  2
        바뀜: visited → send_feedback이 ARTWORK_COMPLETED로 한 번씩 보고"""
        if self.active and msg.data not in self.visited:
            self.visited.append(msg.data)
