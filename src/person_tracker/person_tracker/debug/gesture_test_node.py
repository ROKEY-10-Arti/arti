"""
제스처 기반 타겟 등록 및 ReID 추적/위치 추정 테스트·실행 노드 (gesture_test_node.py)

`BaseTrackerNode`를 상속받아 공용 영상 입력·추적·위치추정·오버레이·선택적 상태 저장소
(`TrackerStateStore`, 테스트 노드 기본 ON) 위에 아래 제스처 특화 기능을 추가합니다:
1. 미등록(`target_id is None`) 시 사람별 손목 우선 ROI 생성 -> MediaPipe 21관절 검출 ->
   얼굴 오검출 제외 + Pose 손목 1:1 매칭 -> `주먹(4f) -> 손바닥 펴기(4f)` 타겟 잠금
2. `gesture/status` 토픽 발행 및 `TrackerStateStore` 스냅샷·이벤트 콜백 갱신
3. 상단 제스처 제어 바, 손 스켈레톤/진행도 배지, 별도 `ReID Monitor` 창·트랙바 제공
"""

import time
from typing import Dict, List, Optional, Tuple

from person_tracker.core.env_bootstrap import bootstrap_venv
bootstrap_venv()

import cv2
import numpy as np
from std_msgs.msg import String

from person_tracker.nodes.base_tracker_node import BaseTrackerNode, run_tracker_node
from person_tracker.core.gesture_core import (
    GESTURE_FIST,
    GESTURE_OPEN_PALM,
    HAND_CONNECTIONS,
    STATE_FIST_ARMED,
    STATE_TRIGGERED,
    GestureProgress,
    HandGestureDetector,
    HandGestureObservation,
    MultiTargetGestureManager,
    build_person_hand_rois,
    find_best_hand_owner_slot,
    infer_fallback_hand_slot,
)
from person_tracker.core.multi_tracker_core import UnifiedTrackedPerson
from person_tracker.debug.reid_visualizer import ReIDDebugVisualizer


class GestureTestNode(BaseTrackerNode):
    """
    손 제스처('주먹 -> 손바닥 펴기') 기반 타겟 지정 테스트·튜닝용 노드.
    - use_yolo_roi=True : YOLO로 사람 추적 후, 미등록 상태일 때만 각 사람 상반신/손 ROI에서 제스처 판별 -> 자동 타겟 잠금
    - use_yolo_roi=False: YOLO 없이 전체 화면에서 순수 MediaPipe 손 검출·제스처 상태머신만 단독 테스트
    - enable_state_store=True (기본): 동일 프로세스 `import` 모듈용 상태 스냅샷·콜백 활성화
    """

    BAR_HEIGHT = 38

    def __init__(self):
        super().__init__(
            'gesture_test_node',
            default_enable_state_store=True,
            win_name='Arti Gesture Test (Fist -> Open Palm)',
            create_tracker_on_init=False,
        )

        self.declare_parameter('use_yolo_roi', True)
        self.declare_parameter('fist_hold_frames', 4)
        self.declare_parameter('open_hold_frames', 4)
        self.declare_parameter('grace_frames', 3)
        self.declare_parameter('armed_timeout_sec', 2.5)
        self.declare_parameter('min_hand_conf', 0.60)
        self.declare_parameter('min_palm_scale_px', 12.0)
        self.declare_parameter('min_person_width_px', 44)
        self.declare_parameter('require_pose_wrist', False)
        self.declare_parameter('show_reid_viz', True)

        self.use_yolo_roi = bool(self.get_parameter('use_yolo_roi').value)
        fist_hold = int(self.get_parameter('fist_hold_frames').value)
        open_hold = int(self.get_parameter('open_hold_frames').value)
        grace = int(self.get_parameter('grace_frames').value)
        armed_timeout = float(self.get_parameter('armed_timeout_sec').value)
        min_hand_conf = float(self.get_parameter('min_hand_conf').value)
        self.min_palm_scale_px = float(self.get_parameter('min_palm_scale_px').value)
        self.min_person_width_px = int(self.get_parameter('min_person_width_px').value)
        self.require_pose_wrist = bool(self.get_parameter('require_pose_wrist').value)
        self.show_reid_viz = bool(self.get_parameter('show_reid_viz').value)

        self.gesture_detector = HandGestureDetector(
            num_hands=2,
            min_detection_confidence=min_hand_conf,
            min_presence_confidence=min_hand_conf,
        )
        self.gesture_mgr = MultiTargetGestureManager(
            fist_hold_frames=fist_hold,
            open_hold_frames=open_hold,
            grace_frames=grace,
            armed_timeout_sec=armed_timeout,
        )
        if self.use_yolo_roi:
            self._ensure_tracker()

        self.gesture_ms = 0.0
        self.last_status_str = 'IDLE'
        self.status_pub = self.create_publisher(String, 'gesture/status', 10)

        self.reid_win_name = 'ReID Similarity & 304D HSV Vector'
        self.reid_viz = ReIDDebugVisualizer()
        if self.show_gui and self.show_reid_viz:
            self._ensure_reid_window()

        self.get_logger().info(
            f'GestureTestNode 시작 | 소스: {self.image_source_label} | '
            f'YOLO+ROI: {self.use_yolo_roi} | StateStore: {self.enable_state_store} | '
            f'fist={fist_hold}, open={open_hold} | 모델: {self.gesture_detector.model_path}'
        )

    def _ensure_reid_window(self):
        """ReID 모니터 창과 4개 실시간 튜닝 트랙바(Match%, Coex%, Margin%, ConfirmF)를 생성·연동합니다."""
        cv2.namedWindow(self.reid_win_name)
        cfg = self._tracker_cfg
        m_val = int(round((self.tracker.reid_match_thresh if self.tracker else float(cfg['reid_match_thresh'])) * 100))
        c_val = int(round((self.tracker.coexist_min_sim if self.tracker else float(cfg['coexist_min_sim'])) * 100))
        g_val = int(round((self.tracker.recover_margin if self.tracker else float(cfg['recover_margin'])) * 100))
        f_val = int(self.tracker.recover_confirm_frames if self.tracker else int(cfg['recover_confirm_frames']))

        def _on_match(v):
            if self.tracker is not None:
                val = max(0.30, v / 100.0)
                self.tracker.reid_match_thresh = val
                self.tracker.search_min_sim = max(0.28, val - 0.02)

        def _on_coex(v):
            if self.tracker is not None:
                self.tracker.coexist_min_sim = max(0.50, v / 100.0)

        def _on_margin(v):
            if self.tracker is not None:
                self.tracker.recover_margin = max(0.0, v / 100.0)

        def _on_confirm(v):
            if self.tracker is not None:
                self.tracker.recover_confirm_frames = max(1, int(v))

        cv2.createTrackbar('Match%', self.reid_win_name, m_val, 80, _on_match)
        cv2.createTrackbar('Coex%', self.reid_win_name, c_val, 95, _on_coex)
        cv2.createTrackbar('Margin%', self.reid_win_name, g_val, 20, _on_margin)
        cv2.createTrackbar('ConfirmF', self.reid_win_name, f_val, 5, _on_confirm)

    def _on_tracker_reset(self, _reason: str):
        self.gesture_mgr.reset_all()
        self.reid_viz.reset()
        self._publish_status('SCANNING')

    def _toggle_yolo_roi(self):
        self.use_yolo_roi = not self.use_yolo_roi
        if self.use_yolo_roi:
            self._ensure_tracker()
        self.gesture_mgr.reset_all()
        self.get_logger().info(f'[모드 전환] use_yolo_roi={self.use_yolo_roi}')

    def _adjust_thresholds(self, d_fist: int = 0, d_open: int = 0):
        new_fist = max(1, min(15, self.gesture_mgr.fist_hold_frames + d_fist))
        new_open = max(1, min(15, self.gesture_mgr.open_hold_frames + d_open))
        self.gesture_mgr.set_thresholds(new_fist, new_open)
        self.get_logger().info(
            f'[제스처 기준] fist={self.gesture_mgr.fist_hold_frames}f, open={self.gesture_mgr.open_hold_frames}f'
        )

    def _handle_custom_mouse(self, x: int, y: int) -> bool:
        if y <= self.BAR_HEIGHT:
            slot_w = max(1, self.frame_width // 4)
            slot_idx = min(3, max(0, x // slot_w))
            if slot_idx == 0:
                self._toggle_yolo_roi()
            elif slot_idx == 1:
                self._reset_target()
            elif slot_idx == 2:
                self._adjust_thresholds(d_fist=1 if x >= (2 * slot_w + slot_w // 2) else -1)
            else:
                self._adjust_thresholds(d_open=1 if x >= (3 * slot_w + slot_w // 2) else -1)
            return True
        if not self.use_yolo_roi:
            return True
        return False

    def _handle_custom_key(self, key: int) -> bool:
        if key in (ord('y'), ord('Y')):
            self._toggle_yolo_roi()
            return True
        if key == ord('1'):
            self._adjust_thresholds(d_fist=-1)
            return True
        if key == ord('2'):
            self._adjust_thresholds(d_fist=1)
            return True
        if key == ord('3'):
            self._adjust_thresholds(d_open=-1)
            return True
        if key == ord('4'):
            self._adjust_thresholds(d_open=1)
            return True
        if key in (ord('v'), ord('V')):
            self.show_reid_viz = not self.show_reid_viz
            if self.show_gui:
                if self.show_reid_viz:
                    self._ensure_reid_window()
                else:
                    cv2.destroyWindow(self.reid_win_name)
            self.get_logger().info(f'[ReID 그래프/벡터 창] {self.show_reid_viz}')
            return True
        return False

    def _publish_status(self, status_text: str):
        if status_text != self.last_status_str:
            self.last_status_str = status_text
            self.status_pub.publish(String(data=status_text))

    @staticmethod
    def _draw_hand_skeleton(frame: np.ndarray, obs: HandGestureObservation):
        pts = obs.landmarks_px
        if obs.gesture == GESTURE_FIST:
            color = (0, 180, 255)
        elif obs.gesture == GESTURE_OPEN_PALM:
            color = (80, 255, 80)
        else:
            color = (180, 180, 180)

        for i1, i2 in HAND_CONNECTIONS:
            cv2.line(
                frame,
                (int(pts[i1, 0]), int(pts[i1, 1])),
                (int(pts[i2, 0]), int(pts[i2, 1])),
                color,
                2,
                cv2.LINE_AA,
            )
        for i in range(21):
            r = 4 if i in (0, 4, 8, 12, 16, 20) else 2
            cv2.circle(frame, (int(pts[i, 0]), int(pts[i, 1])), r, (255, 255, 255), -1)

        x1, y1, x2, y2 = obs.hand_bbox
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)
        f_bits = ''.join('1' if f else '0' for f in obs.finger_states)
        cv2.putText(
            frame,
            f'{obs.gesture} [{f_bits}]',
            (x1, max(48, y1 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            color,
            2,
            cv2.LINE_AA,
        )

    def _draw_top_bar(self, frame: np.ndarray, gesture_active: bool):
        _, w = frame.shape[:2]
        self.frame_width = w
        slot_w = w // 4

        items = [
            (
                f'[Y] YOLO+ROI:{"ON" if self.use_yolo_roi else "OFF"}',
                (35, 130, 35) if self.use_yolo_roi else (60, 60, 60),
            ),
            ('[R] Reset Target', (140, 80, 30)),
            (f'[1/2] Fist:{self.gesture_mgr.fist_hold_frames}f', (55, 55, 55)),
            (f'[3/4] Open:{self.gesture_mgr.open_hold_frames}f', (55, 55, 55)),
        ]
        for idx, (label, bg) in enumerate(items):
            x1 = idx * slot_w
            x2 = w if idx == 3 else (idx + 1) * slot_w
            cv2.rectangle(frame, (x1, 0), (x2, self.BAR_HEIGHT), bg, -1)
            cv2.rectangle(frame, (x1, 0), (x2, self.BAR_HEIGHT), (95, 95, 95), 1)
            cv2.putText(
                frame,
                label,
                (x1 + 8, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.44,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        adapt_str = ''
        if self.use_yolo_roi and self.tracker is not None and self.tracker.target_id is not None:
            adapt_str = f' | {self.tracker.metrics.adaptive_state}'
        pub_tag = ''
        if self.localizer is not None and not self.publish_target_topics:
            pub_tag = ' | PUB:OFF'
        mask_tag = '' if self.show_pose_mask else ' | MASK:OFF'
        state_txt = (
            f'FPS:{self.fps:4.1f} | Total:{self.total_ms:4.1f}ms | '
            f'Gesture:{self.gesture_ms:4.1f}ms | CPU:{self.cpu_percent:4.1f}% | '
            f'{"SCANNING" if gesture_active else "LOCKED"}{adapt_str}{pub_tag}{mask_tag}'
        )
        cv2.rectangle(frame, (0, self.BAR_HEIGHT), (w, self.BAR_HEIGHT + 24), (24, 24, 24), -1)
        cv2.putText(
            frame,
            state_txt,
            (8, self.BAR_HEIGHT + 17),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (0, 255, 255) if gesture_active else (80, 255, 120),
            1,
            cv2.LINE_AA,
        )

    @staticmethod
    def _draw_progress_badge(
        frame: np.ndarray,
        anchor_xy: Tuple[int, int],
        prog: GestureProgress,
        label_prefix: str = '',
    ):
        x, y = anchor_xy
        hand_tag = f'({prog.hand_label}) ' if prog.hand_label else ''
        prefix = f'{label_prefix.rstrip()} {hand_tag}' if label_prefix else hand_tag
        if prog.state == STATE_TRIGGERED:
            txt = f'{prefix}TARGET LOCKED!'
            col = (80, 255, 80)
        elif prog.state == STATE_FIST_ARMED:
            txt = f'{prefix}OPEN PALM ({prog.open_count}/{prog.open_target})'
            col = (0, 255, 255)
        else:
            txt = f'{prefix}Fist ({prog.fist_count}/{prog.fist_target})'
            col = (200, 200, 200) if prog.fist_count == 0 else (0, 180, 255)

        cv2.putText(
            frame,
            txt,
            (max(8, x), max(74, y)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            col,
            2,
            cv2.LINE_AA,
        )

    def _scan_person_gestures(
        self,
        frame: np.ndarray,
        detections: List[UnifiedTrackedPerson],
        now_wall: float,
    ) -> Tuple[
        Optional[UnifiedTrackedPerson],
        Dict[int, List[Tuple[Tuple[int, int, int, int], str]]],
        List[HandGestureObservation],
        Dict[int, GestureProgress],
        bool,
        float,
    ]:
        t_g0 = time.perf_counter()
        rois: Dict[int, List[Tuple[Tuple[int, int, int, int], str]]] = {}
        observations: List[HandGestureObservation] = []
        progresses: Dict[int, GestureProgress] = {}
        target: Optional[UnifiedTrackedPerson] = None
        registered_now = False

        active_ids = [p.track_id for p in detections]
        person_tuples = [(p.track_id, p.bbox, p.keypoints) for p in detections]
        assigned_slot_obs: Dict[int, Dict[str, List[HandGestureObservation]]] = {
            p.track_id: {} for p in detections
        }
        for person in detections:
            if (person.bbox[2] - person.bbox[0]) < self.min_person_width_px:
                continue
            person_rois = build_person_hand_rois(person.bbox, person.keypoints, frame.shape)
            rois[person.track_id] = person_rois
            for roi_box, _ in person_rois:
                for obs in self.gesture_detector.detect_in_roi(frame, roi=roi_box):
                    owner = find_best_hand_owner_slot(
                        obs,
                        person_tuples,
                        min_palm_scale_px=self.min_palm_scale_px,
                        min_person_width_px=self.min_person_width_px,
                        require_pose_wrist=self.require_pose_wrist,
                    )
                    if owner is not None:
                        owner_id, hand_slot, _ = owner
                        assigned_slot_obs[owner_id].setdefault(hand_slot, []).append(obs)

        for person in detections:
            per_slot_primary: Dict[str, HandGestureObservation] = {}
            for hand_slot, slot_list in assigned_slot_obs.get(person.track_id, {}).items():
                chosen = self.gesture_detector.pick_primary_observation(slot_list)
                if chosen is not None:
                    per_slot_primary[hand_slot] = chosen
                    observations.append(chosen)

            slot_progs = self.gesture_mgr.update_person_hands(
                person.track_id, per_slot_primary, now=now_wall
            )
            prog = self.gesture_mgr.pick_best_progress(
                slot_progs, fallback_subject_id=person.track_id
            )
            progresses[person.track_id] = prog
            triggered_prog = next((sp for sp in slot_progs if sp.just_triggered), None)
            if triggered_prog is not None:
                locked_id = self.tracker.select_target_by_id(person.track_id)
                person.is_target = True
                target = person
                registered_now = True
                progresses[person.track_id] = triggered_prog
                self.get_logger().info(
                    f'[제스처 타겟 등록] ID={locked_id} hand={triggered_prog.hand_label} (주먹->펴기 완료)'
                )
                self._publish_status(f'TRIGGERED:id={locked_id}')
                break

        self.gesture_mgr.prune_stale(active_ids, now=now_wall)
        t_gesture_ms = (time.perf_counter() - t_g0) * 1000.0
        return target, rois, observations, progresses, registered_now, t_gesture_ms

    def handle_frame(self, frame: np.ndarray, frame_stamp=None):
        """프레임 1장을 추적 -> 제스처 등록 -> 3D 위치추정 -> 선택적 상태 갱신 -> GUI 출력합니다."""
        t_start = time.perf_counter()
        now_wall = time.time()
        self.last_frame_wall_time = now_wall
        self._current_frame_stamp = frame_stamp
        self._update_perf_metrics(now_wall)

        detections: List[UnifiedTrackedPerson] = []
        target: Optional[UnifiedTrackedPerson] = None
        rois: Dict[int, List[Tuple[Tuple[int, int, int, int], str]]] = {}
        observations: List[HandGestureObservation] = []
        progresses: Dict[int, GestureProgress] = {}
        gesture_active = True
        registered_now = False
        phase = 'SCANNING'
        loc_res = None
        t_gesture_ms = 0.0

        if self.use_yolo_roi and self.tracker is not None:
            want_all_sims = self.show_gui and self.show_reid_viz
            detections, target = self.tracker.process_frame(
                frame,
                auto_lock_first=False,
                spatial_estimator=(
                    self._estimate_candidate_spatial if self.localizer is not None else None
                ),
                compute_all_sims=want_all_sims,
            )
            if self.tracker.target_id is not None:
                gesture_active = False
                phase = 'LOCKED' if target is not None else 'LOST_WAIT'
                self._publish_status(
                    f'LOCKED:id={self.tracker.target_id}' if target is not None else 'LOST_WAIT'
                )
            else:
                (
                    target,
                    rois,
                    observations,
                    progresses,
                    registered_now,
                    t_gesture_ms,
                ) = self._scan_person_gestures(frame, detections, now_wall)
                if registered_now:
                    gesture_active = False
                    phase = 'TRIGGERED'
                else:
                    armed = [
                        p.subject_id for p in progresses.values() if p.state == STATE_FIST_ARMED
                    ]
                    phase = 'FIST_ARMED' if armed else 'SCANNING'
                    self._publish_status(f'FIST_ARMED:ids={armed}' if armed else 'SCANNING')

            loc_res = self._localize_and_log_target(target, frame.shape, frame_stamp, now_wall)
        else:
            t_g0 = time.perf_counter()
            obs_list = self.gesture_detector.detect_in_roi(frame, roi=None)
            slot_lists: Dict[str, List[HandGestureObservation]] = {}
            mid_x = frame.shape[1] * 0.5
            for obs in obs_list:
                slot_lists.setdefault(infer_fallback_hand_slot(obs, mid_x), []).append(obs)
            per_slot_primary = {}
            for hand_slot, s_list in slot_lists.items():
                chosen = self.gesture_detector.pick_primary_observation(s_list)
                if chosen is not None:
                    per_slot_primary[hand_slot] = chosen
                    observations.append(chosen)
            slot_progs = self.gesture_mgr.update_person_hands(0, per_slot_primary, now=now_wall)
            prog = self.gesture_mgr.pick_best_progress(slot_progs, fallback_subject_id=0)
            progresses[0] = prog
            t_gesture_ms = (time.perf_counter() - t_g0) * 1000.0
            if prog.just_triggered:
                self.get_logger().info('[단독 제스처 트리거] 주먹 -> 손바닥 펴기 감지됨')
            phase = 'HAND_ONLY'
            self._publish_status(f'HAND_ONLY:{prog.state}:{prog.last_gesture}')

        total_ms = (time.perf_counter() - t_start) * 1000.0
        self.gesture_ms = (
            0.85 * self.gesture_ms + 0.15 * t_gesture_ms if self.gesture_ms > 0 else t_gesture_ms
        )
        self.total_ms = 0.85 * self.total_ms + 0.15 * total_ms if self.total_ms > 0 else total_ms

        self._update_state_store(
            phase=phase,
            target=target,
            loc_res=loc_res,
            registered_now=registered_now,
        )

        if self.show_gui:
            self._render_gui(
                frame, detections, rois, observations, progresses, gesture_active
            )

    def _render_gui(
        self,
        frame: np.ndarray,
        detections: List[UnifiedTrackedPerson],
        rois: Dict[int, List[Tuple[Tuple[int, int, int, int], str]]],
        observations: List[HandGestureObservation],
        progresses: Dict[int, GestureProgress],
        gesture_active: bool,
    ):
        vis = frame.copy()
        self._draw_pose_and_mask_overlays(vis, frame, detections)
        for p in detections:
            x1, y1, x2, y2 = p.bbox
            if p.is_target:
                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(
                    vis,
                    f'TARGET ID:{p.track_id} (Gesture PAUSED)',
                    (x1, max(78, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.50,
                    (0, 255, 0),
                    2,
                    cv2.LINE_AA,
                )
                self._draw_target_loc_text(vis, p.bbox, self.last_loc_result)
            else:
                cv2.rectangle(vis, (x1, y1), (x2, y2), (170, 170, 170), 1)
                if self.tracker is not None and self.tracker.target_id is not None:
                    cv2.putText(
                        vis,
                        f'ID:{p.track_id} sim:{p.similarity:.2f}',
                        (x1, max(78, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.44,
                        (180, 180, 180),
                        1,
                        cv2.LINE_AA,
                    )
                else:
                    for roi_box, roi_kind in rois.get(p.track_id, []):
                        roi_color = (255, 220, 40) if roi_kind == 'wrist' else (40, 160, 220)
                        cv2.rectangle(vis, (roi_box[0], roi_box[1]), (roi_box[2], roi_box[3]), roi_color, 1)
                    prog = progresses.get(p.track_id)
                    if prog is not None:
                        self._draw_progress_badge(
                            vis,
                            (x1, max(84, y1 - 12)),
                            prog,
                            label_prefix=f'ID:{p.track_id} ',
                        )

        if not self.use_yolo_roi and 0 in progresses:
            self._draw_progress_badge(vis, (12, 92), progresses[0], label_prefix='Hand ')

        for obs in observations:
            self._draw_hand_skeleton(vis, obs)

        self._draw_top_bar(vis, gesture_active=gesture_active)
        self.last_display_frame = vis
        cv2.imshow(self.win_name, vis)
        if self.show_reid_viz and self.use_yolo_roi and self.tracker is not None:
            reid_panel = self.reid_viz.render(self.tracker, detections)
            cv2.imshow(self.reid_win_name, reid_panel)
        key = cv2.waitKey(1) & 0xFF
        if key != 255:
            self._process_key(key)

    def destroy_node(self):
        self.gesture_detector.close()
        super().destroy_node()


def main(args=None):
    run_tracker_node(GestureTestNode, args=args)


if __name__ == '__main__':
    main()
