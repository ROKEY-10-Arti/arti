"""
제스처 기반 타겟 등록 및 ReID 추적/위치 추정 통합 실행 노드 (gesture_test_node.py)

[전체 실행 흐름 및 주요 호출 함수]
1. 영상 입력 수신:
   - ROS 이미지 토픽 (`_compressed_cb` / `_raw_cb`) 또는 로컬 웹캠 캡처 스레드 (`_capture_loop` -> `_poll_local_frame`)를 통해
     프레임을 수신하고 `handle_frame(frame)`을 호출합니다.
2. 사람 탐지·추적 및 ReID (`MultiModelTrackerCore.process_frame`):
   - `YOLO26n-pose` + ByteTrack 기반 `MODE_ADAPTIVE`로 프레임 내 사람을 추적하고,
     타겟 잠금 상태일 때는 304D HSV 부위 특징 유사도(`similarity`)와 공간 점수(`_estimate_candidate_spatial`)로
     SAFE / GUARD / SEARCH / RECOVERED 상태를 갱신합니다. (유실 시에는 수동 `[R]` 초기화 전까지 SEARCH 유지)
3. 미등록(`target_id is None`) 시 손 제스처 스캔 및 타겟 등록:
   - 사람별 손목 우선 ROI 생성 (`build_person_hand_rois`)
   - ROI 내부 MediaPipe 21관절 손 검출 (`HandGestureDetector.detect_in_roi`)
   - 얼굴 오검출 제외 + Pose 손목(9,10) 1:1 거리 매칭 (`find_best_hand_owner`)
   - 동일 인물에 복수 손이 매칭되면 대표 1개 선택 (`HandGestureDetector.pick_primary_observation`)
   - '주먹(Fist 4f) -> 손바닥 펴기(Open 4f)' 상태머신 갱신 (`MultiTargetGestureManager.update`)
   - `just_triggered=True` 발생 시 `MultiModelTrackerCore.select_target_by_id()`로 타겟 및 Anchor 특징 뱅크 고정
4. 3D 공간 좌표 추정 및 발행 (`LidarPersonLocalizer.localize_target`):
   - 타겟이 확정되면 LiDAR + 카메라 TF(폴백: BBox 높이 기반 거리)로 `base_link`/`map` 좌표를 계산하고
     `publish_target_topics=True`일 때 타겟 좌표/마커 토픽을 발행합니다.
5. 시각화 및 실시간 튜닝 (`_draw_top_bar`, `ReIDDebugVisualizer.render`, `_ensure_reid_window`):
   - 메인 GUI 창에 BBox, ROI, 손 21관절 스켈레톤, 제스처 진행도, 좌표 오버레이를 출력하고,
     별도 ReID 모니터 창에 유사도 시계열/부위 점수/304D 히트맵과 트랙바(`Match%`, `Coex%`, `Margin%`, `ConfirmF`)를 제공합니다.
"""

import os
import threading
import time
from typing import Dict, List, Optional, Tuple, Union

from person_tracker.env_bootstrap import bootstrap_venv, resolve_image_topic
bootstrap_venv()

import cv2
from cv_bridge import CvBridge
import numpy as np
import psutil
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String

from person_tracker.gesture_core import (
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
    find_best_hand_owner,
)
from person_tracker.lidar_person_localizer import LidarPersonLocalizer, TargetLocalizationResult
from person_tracker.multi_tracker_core import MultiModelTrackerCore, UnifiedTrackedPerson
from person_tracker.reid_visualizer import ReIDDebugVisualizer


class GestureTestNode(Node):
    """
    손 제스처('주먹 -> 손바닥 펴기') 기반 타겟 지정 테스트·튜닝용 노드.
    - use_yolo_roi=True : YOLO로 사람 추적 후, 미등록 상태일 때만 각 사람 상반신/손 ROI에서 제스처 판별 -> 자동 타겟 잠금
    - use_yolo_roi=False: YOLO 없이 전체 화면에서 순수 MediaPipe 손 검출·제스처 상태머신만 단독 테스트
    - video_source 지정 시 ROS 카메라 토픽 대신 로컬 웹캠을 직접 열어 단독 테스트 가능
    """

    BAR_HEIGHT = 38

    def __init__(self):
        super().__init__('gesture_test_node')

        self.declare_parameter('use_compressed', True)
        self.declare_parameter('image_topic', '')
        self.declare_parameter(
            'video_source',
            '',
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter('use_yolo_roi', True)
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('device', '')
        self.declare_parameter('fist_hold_frames', 4)
        self.declare_parameter('open_hold_frames', 4)
        self.declare_parameter('grace_frames', 3)
        self.declare_parameter('armed_timeout_sec', 2.5)
        self.declare_parameter('min_hand_conf', 0.60)
        self.declare_parameter('min_palm_scale_px', 12.0)
        self.declare_parameter('min_person_width_px', 44)
        self.declare_parameter('require_pose_wrist', False)
        self.declare_parameter('use_localizer', True)
        self.declare_parameter('publish_target_topics', True)
        self.declare_parameter('publish_cam_tf', False)
        self.declare_parameter('show_gui', True)
        self.declare_parameter('show_reid_viz', True)

        # 트래커 / ReID / EMA 주요 임계 파라미터 (launch/CLI에서 --ros-args -p 로 일괄 조정 가능)
        tracker_param_defaults = {
            'tracker_family_idx': 3,
            'tracker_task_idx': 3,
            'tracker_mode': 5,
            'tracker_conf': 0.15,
            'tracker_iou': 0.55,
            'reid_match_thresh': 0.50,
            'search_min_sim': 0.48,
            'coexist_min_sim': 0.74,
            'recover_margin': 0.06,
            'recover_confirm_frames': 2,
            'id_switch_thresh': 0.48,
            'safe_mismatch_limit': 4,
            'ema_min_sim': 0.60,
            'ema_alpha': 0.04,
            'ema_require_single': True,
        }
        for p_name, p_default in tracker_param_defaults.items():
            self.declare_parameter(p_name, p_default)
        self._tracker_cfg = {
            p_name: self.get_parameter(p_name).value
            for p_name in tracker_param_defaults
        }

        self.use_yolo_roi = bool(self.get_parameter('use_yolo_roi').value)
        self.use_localizer = bool(self.get_parameter('use_localizer').value)
        self.publish_target_topics = bool(self.get_parameter('publish_target_topics').value)
        self.show_gui = bool(self.get_parameter('show_gui').value)
        self.show_reid_viz = bool(self.get_parameter('show_reid_viz').value)
        imgsz = int(self.get_parameter('imgsz').value)
        device = str(self.get_parameter('device').value)
        fist_hold = int(self.get_parameter('fist_hold_frames').value)
        open_hold = int(self.get_parameter('open_hold_frames').value)
        grace = int(self.get_parameter('grace_frames').value)
        armed_timeout = float(self.get_parameter('armed_timeout_sec').value)
        min_hand_conf = float(self.get_parameter('min_hand_conf').value)
        self.min_palm_scale_px = float(self.get_parameter('min_palm_scale_px').value)
        self.min_person_width_px = int(self.get_parameter('min_person_width_px').value)
        self.require_pose_wrist = bool(self.get_parameter('require_pose_wrist').value)
        raw_source = str(self.get_parameter('video_source').value).strip()

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
        self.tracker: Optional[MultiModelTrackerCore] = None
        self.tracker_imgsz = imgsz
        self.tracker_device = device
        if self.use_yolo_roi:
            self._ensure_tracker()

        self.bridge = CvBridge()
        self.frame_width = 640
        self.prev_time = time.time()
        self.fps = 0.0
        self.gesture_ms = 0.0
        self.total_ms = 0.0
        self.process = psutil.Process(os.getpid())
        self.cpu_percent = 0.0
        self.last_sys_check = 0.0

        self.last_frame_wall_time = 0.0
        self.last_display_frame: Optional[np.ndarray] = None
        self.last_status_str = 'IDLE'

        self.status_pub = self.create_publisher(String, 'gesture/status', 10)
        self.localizer: Optional[LidarPersonLocalizer] = (
            LidarPersonLocalizer(self) if self.use_localizer else None
        )
        self.last_loc_result: Optional[TargetLocalizationResult] = None
        self.last_loc_log_time = 0.0

        self._cap = None
        self._cap_lock = threading.Lock()
        self._latest_cam_frame: Optional[np.ndarray] = None
        self._stop_event = threading.Event()
        self._cap_thread: Optional[threading.Thread] = None
        self._cam_timer = None
        self.sub = None
        self.image_source_label = ''

        if raw_source:
            src: Union[int, str] = int(raw_source) if raw_source.isdigit() else raw_source
            self.image_source_label = f'webcam:{src}'
            self._start_local_capture(src)
        else:
            ns = self.get_namespace()
            use_compressed = bool(self.get_parameter('use_compressed').value)
            custom_topic = str(self.get_parameter('image_topic').value)
            image_topic = resolve_image_topic(ns, use_compressed, custom_topic)
            self.image_source_label = image_topic
            if use_compressed:
                self.sub = self.create_subscription(
                    CompressedImage, image_topic, self._compressed_cb, qos_profile_sensor_data
                )
            else:
                self.sub = self.create_subscription(
                    Image, image_topic, self._raw_cb, qos_profile_sensor_data
                )

        self.win_name = 'Arti Gesture Test (Fist -> Open Palm)'
        self.reid_win_name = 'ReID Similarity & 304D HSV Vector'
        self.reid_viz = ReIDDebugVisualizer()
        if self.show_gui:
            cv2.namedWindow(self.win_name)
            cv2.setMouseCallback(self.win_name, self._on_mouse_click)
            if self.show_reid_viz:
                self._ensure_reid_window()

        self.get_logger().info(
            f'GestureTestNode 시작 | 소스: {self.image_source_label} | '
            f'YOLO+ROI: {self.use_yolo_roi} | fist={fist_hold}, open={open_hold} | '
            f'모델: {self.gesture_detector.model_path}'
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

    def _ensure_tracker(self):
        if self.tracker is None:
            cfg = self._tracker_cfg
            self.tracker = MultiModelTrackerCore(
                family_idx=int(cfg['tracker_family_idx']),
                task_idx=int(cfg['tracker_task_idx']),
                mode=int(cfg['tracker_mode']),
                imgsz=self.tracker_imgsz,
                device=self.tracker_device,
                conf=float(cfg['tracker_conf']),
                iou=float(cfg['tracker_iou']),
                reid_match_thresh=float(cfg['reid_match_thresh']),
                search_min_sim=float(cfg['search_min_sim']),
                coexist_min_sim=float(cfg['coexist_min_sim']),
                recover_margin=float(cfg['recover_margin']),
                recover_confirm_frames=int(cfg['recover_confirm_frames']),
                id_switch_thresh=float(cfg['id_switch_thresh']),
                safe_mismatch_limit=int(cfg['safe_mismatch_limit']),
                ema_min_sim=float(cfg['ema_min_sim']),
                ema_alpha=float(cfg['ema_alpha']),
                ema_require_single=bool(cfg['ema_require_single']),
            )

    def _start_local_capture(self, src: Union[int, str]):
        cap = cv2.VideoCapture(src, cv2.CAP_V4L2) if isinstance(src, int) else cv2.VideoCapture(src)
        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(src)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        else:
            self.get_logger().error(f'로컬 카메라를 열 수 없습니다: {src}')
        self._cap = cap
        self._cap_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._cap_thread.start()
        self._cam_timer = self.create_timer(1.0 / 20.0, self._poll_local_frame)

    def _capture_loop(self):
        while not self._stop_event.is_set():
            if self._cap is None or not self._cap.isOpened():
                time.sleep(0.2)
                continue
            ret, frame = self._cap.read()
            if ret and frame is not None:
                if frame.shape[1] != 640 or frame.shape[0] != 480:
                    frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_AREA)
                with self._cap_lock:
                    self._latest_cam_frame = frame
            else:
                time.sleep(0.02)

    def _poll_local_frame(self):
        with self._cap_lock:
            frame = self._latest_cam_frame
            self._latest_cam_frame = None
        if frame is not None:
            self.handle_frame(frame)

    def _compressed_cb(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                self.handle_frame(frame)
        except Exception as exc:
            self.get_logger().warn(f'Compressed decode error: {exc}', throttle_duration_sec=5.0)

    def _raw_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.handle_frame(frame)
        except Exception as exc:
            self.get_logger().warn(f'Raw convert error: {exc}', throttle_duration_sec=5.0)

    def _reset_target(self, reason: str = '[타겟/제스처 초기화] 대기 상태로 복귀합니다.'):
        """등록된 타겟, 제스처 상태머신 및 ReID 시계열 그래프를 초기 대기 상태로 되돌립니다."""
        if self.tracker is not None:
            self.tracker.select_target_by_id(None)
        self.gesture_mgr.reset_all()
        self.reid_viz.reset()
        self.last_loc_result = None
        self.get_logger().info(reason)

    def _toggle_yolo_roi(self):
        self.use_yolo_roi = not self.use_yolo_roi
        if self.use_yolo_roi:
            self._ensure_tracker()
        self.gesture_mgr.reset_all()
        self.get_logger().info(f'[모드 전환] YOLO+ROI = {self.use_yolo_roi}')

    def _adjust_thresholds(self, d_fist: int = 0, d_open: int = 0):
        f = max(1, min(15, self.gesture_mgr.fist_hold_frames + d_fist))
        o = max(1, min(15, self.gesture_mgr.open_hold_frames + d_open))
        self.gesture_mgr.set_thresholds(f, o)
        self.get_logger().info(f'[임계 프레임 변경] fist={f}, open={o}')

    def _on_mouse_click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if y <= self.BAR_HEIGHT:
            slot_w = max(1, self.frame_width // 4)
            slot = min(3, max(0, x // slot_w))
            if slot == 0:
                self._toggle_yolo_roi()
            elif slot == 1:
                self._reset_target()
            elif slot == 2:
                self._adjust_thresholds(d_fist=1 if x > slot_w * 2 + slot_w // 2 else -1)
            else:
                self._adjust_thresholds(d_open=1 if x > slot_w * 3 + slot_w // 2 else -1)
            return

        if self.use_yolo_roi and self.tracker is not None:
            selected = self.tracker.select_target_by_point(x, y)
            if selected is not None:
                self.gesture_mgr.reset_all()
                self.get_logger().info(f'[수동 클릭 잠금] ID={selected}')
            else:
                self._reset_target()

    def _process_key(self, key: int):
        if key in (ord('y'), ord('Y')):
            self._toggle_yolo_roi()
        elif key in (ord('r'), ord('R')):
            self._reset_target()
        elif key == ord('1'):
            self._adjust_thresholds(d_fist=-1)
        elif key == ord('2'):
            self._adjust_thresholds(d_fist=1)
        elif key == ord('3'):
            self._adjust_thresholds(d_open=-1)
        elif key == ord('4'):
            self._adjust_thresholds(d_open=1)
        elif key in (ord('d'), ord('D')) and self.tracker is not None:
            dev = self.tracker.toggle_device()
            self.get_logger().info(f'[장치 전환] {dev}')
        elif key in (ord('v'), ord('V')):
            self.show_reid_viz = not self.show_reid_viz
            if self.show_gui:
                if self.show_reid_viz:
                    self._ensure_reid_window()
                else:
                    cv2.destroyWindow(self.reid_win_name)
            self.get_logger().info(f'[ReID 그래프/벡터 창] {self.show_reid_viz}')
        elif key in (ord('p'), ord('P')):
            self.publish_target_topics = not self.publish_target_topics
            self.get_logger().info(f'[타겟 토픽 발행] publish_target_topics={self.publish_target_topics}')

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
        state_txt = (
            f'FPS:{self.fps:4.1f} | Total:{self.total_ms:4.1f}ms | '
            f'Gesture:{self.gesture_ms:4.1f}ms | CPU:{self.cpu_percent:4.1f}% | '
            f'{"SCANNING" if gesture_active else "LOCKED"}{adapt_str}{pub_tag}'
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
        bar_w = 120
        bar_h = 8
        cv2.rectangle(frame, (x, y), (x + bar_w, y + bar_h), (40, 40, 40), -1)
        fill_w = int(bar_w * max(0.0, min(1.0, prog.progress)))
        if prog.state == STATE_TRIGGERED:
            bar_color = (60, 255, 80)
        elif prog.state == STATE_FIST_ARMED:
            bar_color = (0, 200, 255)
        else:
            bar_color = (200, 200, 200)
        if fill_w > 0:
            cv2.rectangle(frame, (x, y), (x + fill_w, y + bar_h), bar_color, -1)
        cv2.rectangle(frame, (x, y), (x + bar_w, y + bar_h), (120, 120, 120), 1)

        if prog.state == STATE_FIST_ARMED:
            txt = f'{label_prefix}ARMED! Open ({prog.open_count}/{prog.open_target})'
        elif prog.state == STATE_TRIGGERED:
            txt = f'{label_prefix}TRIGGERED!'
        else:
            txt = f'{label_prefix}Fist ({prog.fist_count}/{prog.fist_target})'
        cv2.putText(
            frame,
            txt,
            (x, max(78, y - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            bar_color,
            1,
            cv2.LINE_AA,
        )

    def _estimate_candidate_spatial(self, person: UnifiedTrackedPerson, w: int, h: int):
        """ReID 후보 검증용 3D 위치 추정 콜백(토픽 발행 없이 map 또는 base_link XY 좌표와 센서 출처 반환)."""
        if self.localizer is None:
            return None
        res = self.localizer.localize_target(person, w, h, publish=False)
        if res.map_xyz is not None:
            return (res.map_xyz[0], res.map_xyz[1]), 'map', res.source
        return (res.base_xyz[0], res.base_xyz[1]), 'base_link', res.source

    def handle_frame(self, frame: np.ndarray):
        """
        프레임 1장을 입력받아 전체 추적 -> 제스처 등록 -> 3D 위치 추정 -> GUI 렌더링을 수행하는 메인 파이프라인.

        [단계별 처리 원리]
        1. 추적/ReID (`tracker.process_frame`): 사람 BBox·Pose 키포인트 추적 및 타겟 잠금 시 SAFE/GUARD/SEARCH 수행.
           (유실 시에도 타임아웃 자동 해제 없이 `SEARCH` 상태를 유지하며 수동 `[R]` 초기화 전까지 재식별 대기)
        2. 제스처 스캔 (`target_id is None`일 때만 실행):
           - 인원별 손목 우선 ROI 생성(`build_person_hand_rois`) -> 손 검출(`detect_in_roi`)
           - `find_best_hand_owner()`로 얼굴 오검출 제외 + Pose 손목(9,10) 1:1 최단거리 주인 배정
           - 인원별 대표 손(`pick_primary_observation`)으로 `gesture_mgr.update()` 전이 판정 ->
             `just_triggered` 시 `select_target_by_id()`로 타겟 및 Anchor 특징 뱅크 고정.
        3. 위치 추정 및 발행: 확정된 타겟에 대해 `localizer.localize_target(..., publish=publish_target_topics)` 호출.
        4. GUI 출력: 메인 영상 오버레이 + `[V]` 활성화 시 `reid_viz.render()` 실시간 표시.
        """
        t_start = time.perf_counter()
        now_wall = time.time()
        self.last_frame_wall_time = now_wall

        dt = now_wall - self.prev_time
        self.prev_time = now_wall
        if dt > 0:
            inst = 1.0 / dt
            self.fps = 0.9 * self.fps + 0.1 * inst if self.fps > 0 else inst

        if now_wall - self.last_sys_check >= 0.5:
            self.last_sys_check = now_wall
            self.cpu_percent = psutil.cpu_percent(interval=None)

        detections: List[UnifiedTrackedPerson] = []
        rois: Dict[int, List[Tuple[Tuple[int, int, int, int], str]]] = {}
        observations: List[HandGestureObservation] = []
        progresses: Dict[int, GestureProgress] = {}
        gesture_active = True
        t_gesture_ms = 0.0

        if self.use_yolo_roi and self.tracker is not None:
            # ReID 모니터 창이 켜져 있을 때만 비타겟 포함 전원의 유사도·히스토그램 계산 활성화
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
                if target is not None:
                    self._publish_status(f'LOCKED:id={self.tracker.target_id}')
                else:
                    self._publish_status('LOST_WAIT')

            # 타겟 미등록 상태에서만 사람별 손 ROI 스캔 및 제스처 상태머신 구동
            if self.tracker.target_id is None:
                t_g0 = time.perf_counter()
                active_ids = [p.track_id for p in detections]
                person_tuples = [(p.track_id, p.bbox, p.keypoints) for p in detections]
                # A의 손 ROI에서 B의 손이 검출되어도 실제 Pose 손목 거리가 더 가까운 주인 ID에 귀속
                assigned_obs: Dict[int, List[HandGestureObservation]] = {
                    p.track_id: [] for p in detections
                }
                for person in detections:
                    if (person.bbox[2] - person.bbox[0]) < self.min_person_width_px:
                        continue
                    person_rois = build_person_hand_rois(
                        person.bbox, person.keypoints, frame.shape
                    )
                    rois[person.track_id] = person_rois
                    for roi_box, _ in person_rois:
                        for obs in self.gesture_detector.detect_in_roi(frame, roi=roi_box):
                            owner = find_best_hand_owner(
                                obs,
                                person_tuples,
                                min_palm_scale_px=self.min_palm_scale_px,
                                min_person_width_px=self.min_person_width_px,
                                require_pose_wrist=self.require_pose_wrist,
                            )
                            if owner is not None:
                                assigned_obs[owner[0]].append(obs)

                for person in detections:
                    primary = self.gesture_detector.pick_primary_observation(
                        assigned_obs.get(person.track_id, [])
                    )
                    if primary is not None:
                        observations.append(primary)
                    prog = self.gesture_mgr.update(person.track_id, primary, now=now_wall)
                    progresses[person.track_id] = prog
                    if prog.just_triggered:
                        locked_id = self.tracker.select_target_by_id(person.track_id)
                        person.is_target = True
                        target = person
                        gesture_active = False
                        self.get_logger().info(
                            f'[제스처 타겟 등록] ID={locked_id} (주먹->펴기 완료)'
                        )
                        self._publish_status(f'TRIGGERED:id={locked_id}')
                        break
                self.gesture_mgr.prune_stale(active_ids, now=now_wall)
                t_gesture_ms = (time.perf_counter() - t_g0) * 1000.0
                if self.tracker.target_id is None:
                    armed = [p.subject_id for p in progresses.values() if p.state == STATE_FIST_ARMED]
                    self._publish_status(f'FIST_ARMED:ids={armed}' if armed else 'SCANNING')

            if target is not None and self.localizer is not None:
                h_f, w_f = frame.shape[:2]
                loc_res = self.localizer.localize_target(
                    target, w_f, h_f, publish=self.publish_target_topics
                )
                self.last_loc_result = loc_res
                if now_wall - self.last_loc_log_time >= 1.0:
                    self.last_loc_log_time = now_wall
                    bx, by, _ = loc_res.base_xyz
                    if loc_res.map_xyz is not None:
                        mx, my, _ = loc_res.map_xyz
                        map_str = f'X={mx:+.2f}m, Y={my:+.2f}m [{loc_res.map_source}]'
                    else:
                        map_str = 'map TF 대기중'
                    self.get_logger().info(
                        f'[타겟 위치 ID={loc_res.track_id} | {loc_res.source}] '
                        f'base: ({bx:+.2f},{by:+.2f})m ({loc_res.base_dist_m:.2f}m) | map: {map_str}'
                    )
            else:
                self.last_loc_result = None
        else:
            t_g0 = time.perf_counter()
            obs_list = self.gesture_detector.detect_in_roi(frame, roi=None)
            observations.extend(obs_list)
            primary = self.gesture_detector.pick_primary_observation(obs_list)
            prog = self.gesture_mgr.update(0, primary, now=now_wall)
            progresses[0] = prog
            t_gesture_ms = (time.perf_counter() - t_g0) * 1000.0
            if prog.just_triggered:
                self.get_logger().info('[단독 제스처 트리거] 주먹 -> 손바닥 펴기 감지됨')
            self._publish_status(f'HAND_ONLY:{prog.state}:{prog.last_gesture}')

        total_ms = (time.perf_counter() - t_start) * 1000.0
        self.gesture_ms = (
            0.85 * self.gesture_ms + 0.15 * t_gesture_ms if self.gesture_ms > 0 else t_gesture_ms
        )
        self.total_ms = 0.85 * self.total_ms + 0.15 * total_ms if self.total_ms > 0 else total_ms

        if self.show_gui:
            vis = frame.copy()
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
                    if self.last_loc_result is not None:
                        lr = self.last_loc_result
                        bx, by, _ = lr.base_xyz
                        coord_txt = f'Bot:({bx:+.2f},{by:+.2f})m {lr.base_dist_m:.2f}m [{lr.source}]'
                        if lr.map_xyz is not None:
                            mx, my, _ = lr.map_xyz
                            coord_txt += f' Map:({mx:+.2f},{my:+.2f})m'
                        cv2.putText(
                            vis,
                            coord_txt,
                            (x1, min(vis.shape[0] - 8, y2 + 16)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.42,
                            (50, 255, 255),
                            1,
                            cv2.LINE_AA,
                        )
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

    def refresh_idle_gui(self):
        if not self.show_gui:
            return
        if time.time() - self.last_frame_wall_time < 0.4:
            return
        if self.last_display_frame is not None:
            idle = self.last_display_frame.copy()
        else:
            idle = np.zeros((480, 640, 3), dtype=np.uint8)
        h, w = idle.shape[:2]
        cv2.rectangle(idle, (0, h - 32), (w, h), (20, 20, 20), -1)
        cv2.putText(
            idle,
            f'WAITING FOR SOURCE: {self.image_source_label}',
            (10, h - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.46,
            (0, 220, 255),
            1,
            cv2.LINE_AA,
        )
        cv2.imshow(self.win_name, idle)
        key = cv2.waitKey(1) & 0xFF
        if key != 255:
            self._process_key(key)

    def destroy_node(self):
        self._stop_event.set()
        if self._cap_thread is not None and self._cap_thread.is_alive():
            self._cap_thread.join(timeout=1.0)
        if self._cap is not None and self._cap.isOpened():
            self._cap.release()
        self.gesture_detector.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureTestNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.refresh_idle_gui()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
