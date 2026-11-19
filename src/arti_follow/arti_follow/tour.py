"""② 관람 = start_tour 액션 (visitor_follower 노드의 일부). 키오스크 "관람 시작" 버튼 (prepare_tour 성공 뒤).

  run_tour       액션 실행: REGISTERED 확인 → FOLLOW → 관람 끝까지 feedback → result
  상태별 동작 (step이 부름)  FOLLOW follow · SEARCH/WAIT/LOST search
  end_tour       관람 끝 (취소·실패·LOST·not_started 공통): 정지, 관람객 데이터 삭제, arti/tour_end
  on_explaining·on_explained   artwork_zone 작품 설명 → feedback EXPLAINING / ARTWORK_COMPLETED
"""
import math
import time

from arti_interfaces.action import StartTour
import numpy as np
from std_msgs.msg import Empty

from arti_follow.motion import wrap

TRACKING = ('FOLLOW', 'SEARCH', 'WAIT')     # 관람 중 = 관람객을 따라가거나 찾는 상태 (start_tour 뒤)


class TourMixin:
    """VisitorFollower가 상속해서 쓰는 관람 단계 함수."""

    def run_tour(self, goal):
        """[액션 실행, 관람 내내] start_tour goal 하나: prepare_tour로 등록된 관람객을 끝까지 따라간다 (goal None = auto_start).
        FOLLOW → 0.2 s마다 feedback (취소 확인) → active가 내려가면 result.
        검사는 하지 않는다: 키오스크가 [관람 시작]을 prepare_tour 성공 뒤 한 번만, 120 s 안에만 켠다
        (로봇 ready_timeout 130 s가 키오스크 120 s보다 길어서 로봇이 먼저 끝나는 일 없음). artwork_ids는 보지 않음 (자유관람).
        움직임은 step()이 하고, 여기서는 시작·끝과 키오스크 보고만 한다.
        받음: goal (is_cancel_requested)
        돌려줌: StartTour.Result(success = 취소로 끝남, message = 끝난 이유)"""
        with self.lock:
            self.get_logger().info('관람 시작 → 추종')
            self.set_state('FOLLOW')                # 다음 주기부터 step이 따라감 (안 보이면 SEARCH로)
        while self.active:
            if goal is not None and goal.is_cancel_requested:
                with self.lock:
                    self.end_tour('READY', 'canceled')
                break
            self.send_feedback(goal)
            time.sleep(0.2)
        reason = self.end_reason
        result = StartTour.Result(success=reason == 'canceled', message=reason)
        with self.lock:
            visited, self.visited = self.visited, []   # 관람 기록은 로그로 남기고 지운다 (관람객 데이터 남기지 않음)
        self.get_logger().info(f'관람 결과: success={result.success}, message={reason}, '
                               f'설명 완료 {visited}')
        if goal is not None:
            goal.canceled() if reason == 'canceled' else goal.abort()
        return result

    def send_feedback(self, goal):
        """내부 상태 → 키오스크용 feedback (StartTour.action, docs/common/ros2_interfaces.md).
        새로 끝까지 설명한 작품이 있으면 → ARTWORK_COMPLETED + 그 작품 번호 (작품마다 한 번)
        설명 중 → EXPLAINING, 그 밖 → FOLLOW (관람 중 상태는 FOLLOW·SEARCH·WAIT·LOST뿐).
        받음: goal, (state, explaining, visited, reported)
        내보냄: feedback (state, artwork_id)"""
        if goal is None:
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
        """[FOLLOW] step [6]에서: 관람객 위치를 알고 마지막으로 본 지 lost_timeout(3 s) 안 → 0.8 m 유지하며 따라간다.
        멀면 dwa로 이동, 0.85 m 안이면 서서 몸만 돌림 (0.95 m 밖이 되면 다시 출발).
        받음: now, pose (x, y, yaw), here (x, y), (kf.x·kf.t, holding)
        내보냄: set_state(FOLLOW), drive(dwa(...)) 또는 face()"""
        p = self.p
        self.set_state('FOLLOW')
        last, vel = self.person()
        person = last + vel * (now - self.kf.t)     # 마지막 좌표 뒤 흐른 시간만큼 → 지금 위치
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
            goal = ahead - to / d * p['target_distance']
            self.dbg['reason'] = 'follow'
            return self.drive(*self.dwa(pose, goal, ahead))
        # 가까우면: 앞으로는 안 가고 사람 쪽으로 몸만 돌림 (작품 감상 중 등)
        self.dbg['reason'] = 'hold'
        return self.face(pose, here, person)

    def search(self, now, pose, here):
        """[SEARCH → WAIT → LOST] step [10] 다음: 관람객을 놓침 (lost_timeout 넘게 좌표가 안 옴, kf.x는 있음).
        마지막 위치로 이동 → 거기서 search_time 동안 한 바퀴 회전 → wait_time 동안 정지 대기 → LOST (관람 끝).
        좌표가 다시 오면 step [6]이 FOLLOW로 돌린다.
        받음: now, pose, here, (kf.x, lost_at, spin_at, turn)
        내보냄: set_state(SEARCH/WAIT), drive(...), 끝나면 end_tour(LOST, lost)"""
        p = self.p
        # ---- 여기부터 = 사람을 놓침 (3 s 넘게 좌표가 안 옴) ----
        last, vel = self.person()
        # [11] 방금 놓쳤나? (직전 상태가 FOLLOW) → 예: 놓친 시각·회전 방향 기억 (한 번만) / 아니오: 건너뜀
        # 방금 놓친 순간(직전까지 FOLLOW)에만 한 번: 놓친 시각과 사람이 가던 쪽(왼/오른)을 기억 → 나중에 그쪽으로 돈다
        if self.state == 'FOLLOW': # 놓친 순간 1회 
            self.lost_at, self.spin_at = now, None # lost-at은 언제 놓쳤나? spin_at은 회전 탐색은 아직 안 시작함
            d = last + vel - here
            self.turn = 1.0 if wrap(math.atan2(d[1], d[0]) - pose[2]) >= 0 else -1.0
        # [12] 아직 회전 탐색 전인가? (spin_at == None) → 예: 마지막 위치로 이동 또는 회전 준비 / 아니오: 아래로
        # 1) 아직 회전 탐색 전: 마지막으로 본 곳까지 먼저 간다 (기둥·모퉁이 뒤로 사라졌으면 가야 보인다)
        if self.spin_at is None:
            cap = p['search_move_time']             # 0이면 시간 제한 없음
            # [12-1] 마지막 위치까지 0.5 m 넘게 남았나? (시간 제한 안인가?) → 예: 그쪽으로 이동 / 아니오: 도착 → 회전 시작 시각 기록
            # 아직 0.5 m 넘게 남았고 (시간 제한이 있으면) 그 안이면 계속 이동
            if np.linalg.norm(last - here) > 0.5 and (cap <= 0 or now - self.lost_at < cap):
                self.set_state('SEARCH')
                self.dbg['reason'] = 'search_move'
                return self.drive(*self.dwa(pose, last, last))
            self.spin_at = now                      # 도착 → 여기서부터 search_time 동안 한 바퀴 (가속분 5 % 여유)
            self.spin_speed = float(np.clip(1.05 * 2 * math.pi / max(p['search_time'], 1e-3),
                                            p['search_spin'], p['max_angular']))
        t = now - self.spin_at
        # [13] 회전 탐색 10 s가 안 지났나? (t = now - spin_at < search_time) → 예: 제자리 회전 / 아니오(10 s 이상): 아래로
        # 2) 도착 뒤 10 s 동안 제자리 한 바퀴: 카메라 화각(88°) 밖으로 나간 사람을 다시 화면에 넣으려고.
        #    사람이 가던 쪽부터 돈다 (그쪽에 있을 가능성이 크다)
        if t < p['search_time']:
            self.set_state('SEARCH') # SEARCH 상태로 바꿈 
            self.dbg['reason'] = 'search_spin'
            return self.drive(*self.ramp(0.0, self.turn * self.spin_speed))
        # [14] 회전 10 s + 대기 10 s가 다 지났나? (t >= search_time + wait_time) → 예: 관람 끝 LOST / 아니오: 아래 WAIT
        # 4) 회전 10 s + 대기 10 s 다 지남 → 더 기다려도 못 찾는다고 보고 관람 끝 (키오스크에 result lost)
        if t >= p['search_time'] + p['wait_time']:
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
        self.active, self.end_reason = False, reason
        self.drive(0.0, 0.0)
        self.kf.x, self.kf.P, self.seen, self.holding = None, None, -1e9, False
        self.tour_end_pub.publish(Empty())
        self.get_logger().info(f'관람 끝 ({reason}) → 관람객 데이터 삭제 (arti/tour_end)')
        self.set_state(state)

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
