"""
앞 사람 자동 등록 / 클릭 등록 추적 노드 (front_tracker_node.py)

`BaseTrackerNode`를 상속받아 공용 영상 입력·YOLO+ReID 추적·LiDAR 위치추정·오버레이·`arti/tour_end` 초기화를 그대로 쓰고,
등록 방식만 '기존 방식'으로 구현합니다 (gesture_test_node는 같은 베이스에 손 제스처 등록).
1. `target_person/register_front` (std_msgs/Empty) 를 받으면 로봇 정면 ±register_front_angle_deg,
   register_max_dist 안에서 가장 가까운 사람을 타겟으로 등록 (없으면 해제).
   arti_follow `visitor_follower`가 관람객을 기다릴 때(WAIT_VISITOR) 1 s마다 보낸다.
2. 인식 창에서 사람을 마우스로 클릭하면 그 사람으로 등록 (베이스 `_on_mouse_click`).
출력은 베이스와 같다: <ns>/target_person/point_map, point_base, marker.
"""

import math
import time
from typing import List, Optional, Tuple

from person_tracker.env_bootstrap import bootstrap_venv
bootstrap_venv()

import cv2
import numpy as np
from std_msgs.msg import Empty

from person_tracker.base_tracker_node import BaseTrackerNode, run_tracker_node
from person_tracker.multi_tracker_core import UnifiedTrackedPerson


class FrontTrackerNode(BaseTrackerNode):
    """앞 사람 자동 등록(register_front 요청)·클릭 등록 + 추적·위치추정 노드."""

    def __init__(self):
        super().__init__('front_tracker_node', win_name='Arti Tracker (front / click register)')
        # target_person/register_front 요청 때 '바로 앞 사람'으로 볼 범위 (base_link 기준)
        self.declare_parameter('register_front_angle_deg', 30.0)  # 정면에서 좌우 이 각도 안
        self.declare_parameter('register_max_dist', 2.5)          # 이 거리 안 [m]
        self.register_front_angle = float(self.get_parameter('register_front_angle_deg').value)
        self.register_max_dist = float(self.get_parameter('register_max_dist').value)
        self.frame_size: Optional[Tuple[int, int]] = None          # 마지막 프레임 (w, h)
        self.create_subscription(Empty, 'target_person/register_front', self._on_register_front, 10)
        self.get_logger().info(
            f'FrontTrackerNode 시작 | 소스: {self.image_source_label} | 앞 사람 등록 범위 '
            f'±{self.register_front_angle:.0f}°, {self.register_max_dist:.1f} m | 클릭 등록 가능'
        )

    def _on_register_front(self, _msg):
        """로봇 정면 범위 안에서 가장 가까운 사람을 타겟으로 등록. 아무도 없으면 예전 타겟 해제."""
        if self.frame_size is None or self.tracker is None or self.localizer is None:
            return
        w, h = self.frame_size
        best, best_dist = None, float('inf')
        for person in self.tracker.last_detections:
            res = self.localizer.localize_target(
                person, w, h, publish=False, frame_stamp=self._current_frame_stamp
            )
            if res.base_xyz is None:
                continue
            bx, by = res.base_xyz[0], res.base_xyz[1]
            dist = math.hypot(bx, by)
            angle = math.degrees(math.atan2(by, bx))
            if abs(angle) <= self.register_front_angle and dist <= self.register_max_dist and dist < best_dist:
                best, best_dist = person, dist
        if best is None:
            if self.tracker.target_id is not None:
                self._reset_target('[앞 사람 등록] 앞에 사람 없음 → 예전 타겟 해제')
            return
        if best.track_id == self.tracker.target_id:
            return                                  # 이미 그 사람
        selected = self.tracker.select_target_by_id(best.track_id)
        self.get_logger().info(f'[앞 사람 등록] ID={selected} (거리 {best_dist:.2f} m)')

    def handle_frame(self, frame: np.ndarray, frame_stamp=None):
        """프레임 1장: 추적(ReID) → 타겟 3D 위치추정·발행 → 상태 갱신 → GUI."""
        t_start = time.perf_counter()
        now_wall = time.time()
        self.last_frame_wall_time = now_wall
        self._current_frame_stamp = frame_stamp
        self.frame_size = (frame.shape[1], frame.shape[0])
        self._update_perf_metrics(now_wall)

        detections: List[UnifiedTrackedPerson] = []
        target: Optional[UnifiedTrackedPerson] = None
        if self.tracker is not None:
            detections, target = self.tracker.process_frame(
                frame,
                auto_lock_first=False,
                spatial_estimator=(
                    self._estimate_candidate_spatial if self.localizer is not None else None
                ),
            )
        loc_res = self._localize_and_log_target(target, frame.shape, frame_stamp, now_wall)
        if self.tracker is None or self.tracker.target_id is None:
            phase = 'WAITING'
        else:
            phase = 'LOCKED' if target is not None else 'LOST_WAIT'
        self._update_state_store(phase=phase, target=target, loc_res=loc_res)

        total_ms = (time.perf_counter() - t_start) * 1000.0
        self.total_ms = 0.85 * self.total_ms + 0.15 * total_ms if self.total_ms > 0 else total_ms
        if self.show_gui:
            self._render_gui(frame, detections, phase)

    def _render_gui(self, frame: np.ndarray, detections: List[UnifiedTrackedPerson], phase: str):
        vis = frame.copy()
        self._draw_pose_and_mask_overlays(vis, frame, detections)
        for p in detections:
            x1, y1, x2, y2 = p.bbox
            if p.is_target:
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(vis, f'TARGET ID:{p.track_id}', (x1, max(20, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 255, 0), 2, cv2.LINE_AA)
                self._draw_target_loc_text(vis, p.bbox, self.last_loc_result)
            else:
                cv2.rectangle(vis, (x1, y1), (x2, y2), (170, 170, 170), 1)
                cv2.putText(vis, f'ID:{p.track_id} sim:{p.similarity:.2f}', (x1, max(20, y1 - 8)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.44, (180, 180, 180), 1, cv2.LINE_AA)
        h, w = vis.shape[:2]
        cv2.rectangle(vis, (0, 0), (w, 28), (20, 20, 20), -1)
        target_id = self.tracker.target_id if self.tracker is not None else None
        cv2.putText(vis, f'{phase} | target={target_id} | FPS {self.fps:.1f} | {self.total_ms:.0f} ms | '
                         f'click: register / R: reset',
                    (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 220, 255), 1, cv2.LINE_AA)
        self.last_display_frame = vis
        cv2.imshow(self.win_name, vis)
        key = cv2.waitKey(1) & 0xFF
        if key != 255:
            self._process_key(key)


def main(args=None):
    run_tracker_node(FrontTrackerNode, args=args)


if __name__ == '__main__':
    main()
