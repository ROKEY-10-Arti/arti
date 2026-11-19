"""
공용 ROS 2 사람 추적·위치추정 베이스 노드 (base_tracker_node.py)

`GestureTestNode`, `MultiModelTrackerNode` 등이 공통으로 사용하는:
- 영상 입력(ROS Compressed/Raw 토픽 및 선택적 로컬 웹캠 캡처 + 촬영 타임스탬프 유지)
- `MultiModelTrackerCore` 초기화·실행·마우스 클릭 타겟 선택·`arti/tour_end` 초기화
- `LidarPersonLocalizer` 타임스탬프 동기화 3D 위치 추정·발행·1초 주기 로그
- Pose 스켈레톤·관절 마커 및 3부위 히스토그램 마스크 반투명/외곽선 오버레이
- 선택적 상태/콜백 모듈(`TrackerStateStore`, `enable_state_store` 파라미터로 부하 제어)
- GUI 대기 화면(`refresh_idle_gui`) 및 공통 키 입력(`R`/`D`/`M`/`P`)·실행 루프(`run_tracker_node`)
를 제공합니다.
"""

import os
import threading
import time
from typing import Callable, List, Optional, Tuple, Union

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
from std_msgs.msg import Empty

from person_tracker.env_bootstrap import resolve_image_topic
from person_tracker.feature_extractors import PoseKeypointFeatureExtractor
from person_tracker.lidar_person_localizer import LidarPersonLocalizer, TargetLocalizationResult
from person_tracker.multi_tracker_core import MultiModelTrackerCore, UnifiedTrackedPerson
from person_tracker.tracker_state import TrackerState, TrackerStateStore


class BaseTrackerNode(Node):
    """사람 추적·위치추정 ROS 2 노드의 공용 베이스 클래스."""

    MASK_OVERLAY_ALPHA = 0.34

    # Pose 스켈레톤 연결선 및 히스토그램 3부위 마스크 색상 (BGR)
    POSE_EDGES = [
        # 상반신 (Torso / Arms: 하늘·파랑 계열)
        (5, 6, (255, 200, 50)),
        (5, 11, (255, 200, 50)),
        (6, 12, (255, 200, 50)),
        (11, 12, (255, 200, 50)),
        (5, 7, (255, 170, 50)),
        (7, 9, (255, 170, 50)),
        (6, 8, (255, 170, 50)),
        (8, 10, (255, 170, 50)),
        # 중하반신 (허벅지/바지: 주황색)
        (11, 13, (0, 165, 255)),
        (12, 14, (0, 165, 255)),
        # 최하단 (정강이/신발: 마젠타색)
        (13, 15, (255, 50, 255)),
        (14, 16, (255, 50, 255)),
    ]
    PART_MASK_COLORS = {
        'active': ((255, 200, 50), (0, 165, 255), (255, 50, 255)),
        'muted': ((170, 140, 70), (70, 120, 170), (160, 70, 160)),
    }

    TRACKER_PARAM_DEFAULTS = {
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

    def __init__(
        self,
        node_name: str,
        *,
        default_enable_state_store: bool = False,
        win_name: str = 'Arti Person Tracker',
        create_tracker_on_init: bool = True,
    ):
        super().__init__(node_name)

        self.declare_parameter('use_compressed', True)
        self.declare_parameter('image_topic', '')
        self.declare_parameter(
            'video_source',
            '',
            ParameterDescriptor(dynamic_typing=True),
        )
        self.declare_parameter('imgsz', 320)
        self.declare_parameter('device', '')
        self.declare_parameter('use_localizer', True)
        self.declare_parameter('publish_target_topics', True)
        self.declare_parameter('publish_cam_tf', False)
        self.declare_parameter('show_gui', True)
        self.declare_parameter('show_pose_mask', True)
        self.declare_parameter('enable_state_store', bool(default_enable_state_store))

        for p_name, p_default in self.TRACKER_PARAM_DEFAULTS.items():
            self.declare_parameter(p_name, p_default)
        self._tracker_cfg = {
            p_name: self.get_parameter(p_name).value
            for p_name in self.TRACKER_PARAM_DEFAULTS
        }

        self.use_localizer = bool(self.get_parameter('use_localizer').value)
        self.publish_target_topics = bool(self.get_parameter('publish_target_topics').value)
        self.show_gui = bool(self.get_parameter('show_gui').value)
        self.show_pose_mask = bool(self.get_parameter('show_pose_mask').value)
        self.enable_state_store = bool(self.get_parameter('enable_state_store').value)

        self.tracker: Optional[MultiModelTrackerCore] = None
        self.tracker_imgsz = int(self.get_parameter('imgsz').value)
        self.tracker_device = str(self.get_parameter('device').value)
        if create_tracker_on_init:
            self._ensure_tracker()

        self.state_store = TrackerStateStore(enabled=self.enable_state_store)
        self.bridge = CvBridge()
        self.frame_width = 640
        self.prev_time = time.time()
        self.fps = 0.0
        self.total_ms = 0.0
        self.process = psutil.Process(os.getpid())
        self.cpu_percent = 0.0
        self.last_sys_check = 0.0
        self.last_frame_wall_time = 0.0
        self.last_display_frame: Optional[np.ndarray] = None

        # 관람 끝(visitor_follower 등) -> 관람객 타겟·ReID 특징 초기화
        self.create_subscription(Empty, 'arti/tour_end', self._on_tour_end, 10)
        self.localizer: Optional[LidarPersonLocalizer] = (
            LidarPersonLocalizer(self) if self.use_localizer else None
        )
        self.last_loc_result: Optional[TargetLocalizationResult] = None
        self.last_loc_log_time = 0.0
        self._current_frame_stamp = None

        self._cap = None
        self._cap_lock = threading.Lock()
        self._latest_cam_frame: Optional[np.ndarray] = None
        self._latest_cam_stamp = None
        self._stop_event = threading.Event()
        self._cap_thread: Optional[threading.Thread] = None
        self._cam_timer = None
        self.sub = None
        self.image_source_label = ''

        raw_source = str(self.get_parameter('video_source').value).strip()
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

        self.win_name = win_name
        if self.show_gui:
            cv2.namedWindow(self.win_name)
            cv2.setMouseCallback(self.win_name, self._on_mouse_click)

    # ---------- 외부 모듈(ROS 외 Python import 등) 연동용 선택적 상태 API ----------

    def get_tracker_state(self) -> TrackerState:
        """현재 인식·추적 스냅샷을 반환합니다 (`enable_state_store=False`면 초기 IDLE 스냅샷)."""
        return self.state_store.state

    def register_state_hook(self, event_name: str, callback: Callable[..., None]):
        """상태 전이 이벤트(`on_target_registered`, `on_target_lost` 등) 콜백을 등록합니다."""
        self.state_store.register_hook(event_name, callback)

    def unregister_state_hook(self, event_name: str, callback: Callable[..., None]):
        self.state_store.unregister_hook(event_name, callback)

    # ---------- 코어·입력 초기화 ----------

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
                stamp_msg = self.get_clock().now().to_msg()
                with self._cap_lock:
                    self._latest_cam_frame = frame
                    self._latest_cam_stamp = stamp_msg
            else:
                time.sleep(0.02)

    def _poll_local_frame(self):
        with self._cap_lock:
            frame = self._latest_cam_frame
            frame_stamp = self._latest_cam_stamp
            self._latest_cam_frame = None
            self._latest_cam_stamp = None
        if frame is not None:
            self.handle_frame(frame, frame_stamp=frame_stamp)

    def _compressed_cb(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is not None:
                self.handle_frame(frame, frame_stamp=msg.header.stamp)
        except Exception as exc:
            self.get_logger().warn(f'Compressed decode error: {exc}', throttle_duration_sec=5.0)

    def _raw_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.handle_frame(frame, frame_stamp=msg.header.stamp)
        except Exception as exc:
            self.get_logger().warn(f'Raw convert error: {exc}', throttle_duration_sec=5.0)

    # ---------- 상태 리셋·입력 이벤트 훅 ----------

    def _reset_target(self, reason: str = '[타겟 초기화] 대기 상태로 복귀합니다.'):
        """등록된 타겟·로컬라이저 캐시·상태 저장소를 초기 대기 상태로 되돌립니다."""
        if self.tracker is not None:
            self.tracker.select_target_by_id(None)
        self.last_loc_result = None
        self._on_tracker_reset(reason)
        self.state_store.reset(reason)
        self.get_logger().info(reason)

    def _on_tracker_reset(self, reason: str):
        """서브클래스별 추가 리셋 훅."""

    def _on_tour_end(self, _msg):
        self._reset_target('[관람 끝] 타겟과 re-ID 특징을 지웠습니다.')

    def _handle_custom_mouse(self, x: int, y: int) -> bool:
        """서브클래스가 상단 바 클릭 등을 먼저 처리할 때 True 반환."""
        return False

    def _on_mouse_click(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if self._handle_custom_mouse(x, y):
            return
        if self.tracker is None:
            return
        selected_id = self.tracker.select_target_by_point(x, y)
        if selected_id is not None:
            self.get_logger().info(f'[수동 클릭 잠금] ID={selected_id}')
        else:
            self._reset_target('[수동 클릭 해제] 대기 상태로 복귀합니다.')

    def _handle_custom_key(self, key: int) -> bool:
        """서브클래스 전용 단축키 처리 훅."""
        return False

    def _process_key(self, key: int):
        if self._handle_custom_key(key):
            return
        if key in (ord('r'), ord('R')):
            self._reset_target()
        elif key in (ord('d'), ord('D')) and self.tracker is not None:
            dev = self.tracker.toggle_device()
            self.get_logger().info(f'[장치 전환] {dev}')
        elif key in (ord('m'), ord('M')):
            self.show_pose_mask = not self.show_pose_mask
            self.get_logger().info(f'[포즈/마스크 오버레이] show_pose_mask={self.show_pose_mask}')
        elif key in (ord('p'), ord('P')):
            self.publish_target_topics = not self.publish_target_topics
            self.get_logger().info(f'[타겟 토픽 발행] publish_target_topics={self.publish_target_topics}')

    # ---------- 추적·위치추정·상태 갱신 공용 헬퍼 ----------

    def _update_perf_metrics(self, now_wall: float):
        dt = now_wall - self.prev_time
        self.prev_time = now_wall
        if dt > 0:
            inst = 1.0 / dt
            self.fps = 0.9 * self.fps + 0.1 * inst if self.fps > 0 else inst
        if now_wall - self.last_sys_check >= 0.5:
            self.last_sys_check = now_wall
            self.cpu_percent = psutil.cpu_percent(interval=None)

    def _estimate_candidate_spatial(self, person: UnifiedTrackedPerson, w: int, h: int):
        """ReID 후보 검증용 3D 위치 추정 콜백(토픽 발행 없이 map 또는 base_link XY 좌표와 센서 출처 반환)."""
        if self.localizer is None:
            return None
        res = self.localizer.localize_target(
            person, w, h, publish=False, frame_stamp=self._current_frame_stamp
        )
        if res.map_xyz is not None:
            return (res.map_xyz[0], res.map_xyz[1]), 'map', res.source
        return (res.base_xyz[0], res.base_xyz[1]), 'base_link', res.source

    def _localize_and_log_target(
        self,
        target: Optional[UnifiedTrackedPerson],
        frame_shape: Tuple[int, ...],
        frame_stamp,
        now_wall: float,
    ) -> Optional[TargetLocalizationResult]:
        if target is None or self.localizer is None:
            self.last_loc_result = None
            return None

        h_f, w_f = frame_shape[:2]
        loc_res = self.localizer.localize_target(
            target,
            w_f,
            h_f,
            publish=self.publish_target_topics,
            frame_stamp=frame_stamp,
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
            sync_str = (
                f' ({loc_res.sync_dt_ms:+.0f}ms)'
                if loc_res.sync_dt_ms is not None
                else ''
            )
            self.get_logger().info(
                f'[타겟 위치 ID={loc_res.track_id} | {loc_res.source}{sync_str}] '
                f'base: ({bx:+.2f},{by:+.2f})m ({loc_res.base_dist_m:.2f}m) | map: {map_str}'
            )
        return loc_res

    def _update_state_store(
        self,
        *,
        phase: str,
        target: Optional[UnifiedTrackedPerson],
        loc_res: Optional[TargetLocalizationResult],
        registered_now: bool = False,
    ):
        """`enable_state_store=True`일 때만 스냅샷을 갱신하고 이벤트 콜백을 디스패치합니다."""
        if not self.state_store.enabled:
            return
        tracker = self.tracker
        self.state_store.update(
            phase=phase,
            target_id=tracker.target_id if tracker is not None else None,
            target_visible=(target is not None),
            adaptive_state=tracker.metrics.adaptive_state if tracker is not None else 'IDLE',
            ema_status=tracker.metrics.ema_status if tracker is not None else 'IDLE',
            similarity=target.similarity if target is not None else 0.0,
            visible_parts=target.visible_parts if target is not None else (False, False, False),
            base_xyz=loc_res.base_xyz if loc_res is not None else None,
            map_xyz=loc_res.map_xyz if loc_res is not None else None,
            loc_source=loc_res.source if loc_res is not None else '',
            sync_dt_ms=loc_res.sync_dt_ms if loc_res is not None else None,
            fps=self.fps,
            registered_now=registered_now,
        )

    # ---------- 공용 시각화 오버레이 ----------

    def _draw_pose_and_mask_overlays(
        self,
        vis: np.ndarray,
        src_frame: np.ndarray,
        detections: List[UnifiedTrackedPerson],
    ):
        """히스토그램 추출용 3부위 마스크를 반투명 블렌딩+외곽선으로 표시하고 Pose 관절 마커를 그립니다."""
        if not self.show_pose_mask or self.tracker is None or len(detections) == 0:
            return

        extractor = self.tracker.current_extractor
        in_guard = (
            self.tracker.mode == self.tracker.MODE_ADAPTIVE
            and self.tracker.metrics.adaptive_state == 'GUARD'
        )
        target_locked = self.tracker.target_id is not None

        mask_overlay = vis.copy()
        contour_items = []
        for p in detections:
            exclude_boxes = (
                [o.bbox for o in detections if o.track_id != p.track_id] if in_guard else None
            )
            part_masks = extractor.extract_part_masks(
                src_frame,
                p.bbox,
                mask_poly=p.mask_poly,
                keypoints=p.keypoints,
                exclude_boxes=exclude_boxes,
            )
            if part_masks is None:
                continue
            (rx1, ry1, rx2, ry2), upper_m, mid_m, lower_m = part_masks
            sub = mask_overlay[ry1:ry2, rx1:rx2]
            palette = self.PART_MASK_COLORS[
                'active' if (p.is_target or not target_locked) else 'muted'
            ]
            for part_mask, color in zip((upper_m, mid_m, lower_m), palette):
                if part_mask.any():
                    sub[part_mask > 0] = color
                    contour_items.append((rx1, ry1, part_mask, color))

        if contour_items:
            cv2.addWeighted(
                mask_overlay,
                self.MASK_OVERLAY_ALPHA,
                vis,
                1.0 - self.MASK_OVERLAY_ALPHA,
                0,
                vis,
            )
            for rx1, ry1, part_mask, color in contour_items:
                contours, _ = cv2.findContours(
                    part_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )
                valid_cnts = [c for c in contours if cv2.contourArea(c) >= 12.0]
                if valid_cnts:
                    cv2.drawContours(
                        vis,
                        valid_cnts,
                        -1,
                        color,
                        1,
                        lineType=cv2.LINE_AA,
                        offset=(rx1, ry1),
                    )

        kp_thresh = PoseKeypointFeatureExtractor.KP_CONF_THRESH
        for p in detections:
            if p.keypoints is None or len(p.keypoints) < 17:
                continue
            kp = p.keypoints
            emph = p.is_target or not target_locked
            for i1, i2, edge_color in self.POSE_EDGES:
                if float(kp[i1, 2]) >= kp_thresh and float(kp[i2, 2]) >= kp_thresh:
                    pt1 = (int(kp[i1, 0]), int(kp[i1, 1]))
                    pt2 = (int(kp[i2, 0]), int(kp[i2, 1]))
                    cv2.line(
                        vis,
                        pt1,
                        pt2,
                        edge_color if emph else (150, 150, 150),
                        2 if emph else 1,
                        cv2.LINE_AA,
                    )

            for idx in range(5, 17):
                if float(kp[idx, 2]) >= kp_thresh:
                    pt = (int(kp[idx, 0]), int(kp[idx, 1]))
                    radius = 4 if emph else 2
                    color = (0, 255, 255) if emph else (190, 190, 190)
                    if emph:
                        cv2.circle(vis, pt, radius + 1, (30, 30, 30), 1, cv2.LINE_AA)
                    cv2.circle(vis, pt, radius, color, -1, cv2.LINE_AA)

    @staticmethod
    def _draw_target_loc_text(
        vis: np.ndarray,
        bbox: Tuple[int, int, int, int],
        loc_res: Optional[TargetLocalizationResult],
    ):
        if loc_res is None:
            return
        x1, _, _, y2 = bbox
        bx, by, _ = loc_res.base_xyz
        sync_tag = f' {loc_res.sync_dt_ms:+.0f}ms' if loc_res.sync_dt_ms is not None else ''
        coord_txt = f'Bot:({bx:+.2f},{by:+.2f})m {loc_res.base_dist_m:.2f}m [{loc_res.source}{sync_tag}]'
        if loc_res.map_xyz is not None:
            mx, my, _ = loc_res.map_xyz
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

    def handle_frame(self, frame: np.ndarray, frame_stamp=None):
        """서브클래스가 프레임 단위 추적 파이프라인을 구현합니다."""
        raise NotImplementedError

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
        super().destroy_node()


def run_tracker_node(node_cls: Callable[[], BaseTrackerNode], args=None):
    """ROS 2 스핀 + idle GUI 갱신 공통 실행 엔트리포인트."""
    rclpy.init(args=args)
    node = node_cls()
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
