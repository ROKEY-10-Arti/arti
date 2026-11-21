from collections import deque
from dataclasses import dataclass
import math
import os
import threading
import time
from typing import Optional, Tuple

from builtin_interfaces.msg import Time as BuiltinTime
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PointStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from visualization_msgs.msg import Marker

from person_tracker.core.env_bootstrap import load_phone_camera_config
from person_tracker.core.multi_tracker_core import UnifiedTrackedPerson
from person_tracker.nodes.top_cam_tf_node import make_static_transform, quaternion_to_euler


@dataclass
class TargetLocalizationResult:
    track_id: int
    source: str                      # 'LiDAR' 또는 'CamEst'
    cam_bearing_deg: float           # 상단 캠 기준 수평 각도 (+왼쪽, -오른쪽)
    cam_dist_m: float                # 상단 캠 기준 수평 거리 (m)
    cam_xyz: Tuple[float, float, float]
    base_xyz: Tuple[float, float, float]
    base_dist_m: float               # 로봇(base_link) 중심 기준 수평 거리 (m)
    base_angle_deg: float            # 로봇(base_link) 정면 기준 각도 (+왼쪽, -오른쪽)
    map_xyz: Optional[Tuple[float, float, float]]
    map_source: str                  # 'AMCL' | 'Init+Odom' | 'None'
    matched_rays: int                # 매칭된 라이다 포인트 개수
    sync_dt_ms: Optional[float] = None  # 이미지 대비 선택된 scan 시각 차이 [ms]
    clock_offset_ms: Optional[float] = None  # 호스트 간 시계 오차 보정량(scan - cam) [ms]


def _stamp_to_sec(stamp) -> float:
    if stamp is None:
        return 0.0
    return float(getattr(stamp, 'sec', 0)) + float(getattr(stamp, 'nanosec', 0)) * 1e-9


@dataclass
class CameraPoseStatus:
    base_xyz: Optional[Tuple[float, float, float]]
    odom_xyz: Optional[Tuple[float, float, float]]
    map_xyz: Optional[Tuple[float, float, float]]
    map_yaw_deg: Optional[float]
    map_source: str                  # 'AMCL' | 'Init+Odom' | 'None'
    lidar_hz: float
    lidar_frame: str


def transform_to_matrix(tf_msg: TransformStamped) -> np.ndarray:
    """Convert geometry_msgs/TransformStamped into a 4x4 homogeneous matrix."""
    tr = tf_msg.transform.translation
    rot = tf_msg.transform.rotation
    qx, qy, qz, qw = rot.x, rot.y, rot.z, rot.w

    # Quaternion to 3x3 rotation matrix
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    mat = np.eye(4, dtype=np.float64)
    mat[0, 0] = 1.0 - 2.0 * (yy + zz)
    mat[0, 1] = 2.0 * (xy - wz)
    mat[0, 2] = 2.0 * (xz + wy)
    mat[1, 0] = 2.0 * (xy + wz)
    mat[1, 1] = 1.0 - 2.0 * (xx + zz)
    mat[1, 2] = 2.0 * (yz - wx)
    mat[2, 0] = 2.0 * (xz - wy)
    mat[2, 1] = 2.0 * (yz + wx)
    mat[2, 2] = 1.0 - 2.0 * (xx + yy)

    mat[0, 3] = tr.x
    mat[1, 3] = tr.y
    mat[2, 3] = tr.z
    return mat


def apply_transform_point(mat: np.ndarray, pt: Tuple[float, float, float]) -> Tuple[float, float, float]:
    vec = np.array([pt[0], pt[1], pt[2], 1.0], dtype=np.float64)
    out = mat @ vec
    return float(out[0]), float(out[1]), float(out[2])


class LidarPersonLocalizer:
    """
    상단 웹캠(top_cam_link)의 YOLO 바운딩 박스 각도와
    2D 라이다(/robot4/scan, rplidar_link) 점군을 TF 행렬로 정합하여
    특정 타겟 인물의:
      1) 상단 카메라 기준 3D 좌표 (top_cam_link)
      2) 로봇 몸체 기준 3D 좌표 및 거리/방위각 (base_link)
      3) 저장된 맵 기준 절대 3D 좌표 (map)
    를 실시간으로 계산하고 ROS 2 토픽/마커로 발행하는 모듈.
    """

    def __init__(self, node: Node):
        self.node = node
        cfg = self._load_yaml_defaults()

        ns = node.get_namespace()
        if ns == '/':
            cfg_ns = str(os.environ.get('ARTI_ROBOT_NS') or cfg.get('namespace', '/robot4')).rstrip('/')
            self.ns_prefix = cfg_ns
        else:
            self.ns_prefix = ns.rstrip('/')

        frames_cfg = cfg.get('frames', {})
        ext_cfg = cfg.get('extrinsics', {})
        intr_cfg = cfg.get('camera_intrinsics', {})
        init_cfg = cfg.get('initial_pose', {})

        self.parent_frame = str(frames_cfg.get('parent_frame', 'oakd_rgb_camera_frame'))
        self.camera_link_frame = str(frames_cfg.get('camera_link_frame', 'top_cam_link'))
        self.optical_frame = str(frames_cfg.get('optical_frame', 'top_cam_optical_frame'))
        self.base_frame = 'base_link'
        self.odom_frame = 'odom'
        self.map_frame = 'map'

        init_x = float(init_cfg.get('x', 0.0))
        init_y = float(init_cfg.get('y', 0.0))
        init_yaw = float(init_cfg.get('yaw', 0.0))
        if bool(init_cfg.get('use_degrees', True)):
            init_yaw = math.radians(init_yaw)
        self.init_map_xy_yaw = (init_x, init_y, init_yaw)
        self._initial_odom_inv: Optional[np.ndarray] = None

        self.hfov_deg = float(intr_cfg.get('hfov_deg', 65.0))

        # 실행 시 노드 파라미터로도 카메라 위치/각도를 덮어쓸 수 있도록 지원
        def _param_or_cfg(p_name: str, default_val):
            if not node.has_parameter(p_name):
                node.declare_parameter(p_name, default_val)
            return node.get_parameter(p_name).value

        cam_x = float(_param_or_cfg('cam_x', float(ext_cfg.get('x', 0.0))))
        cam_y = float(_param_or_cfg('cam_y', float(ext_cfg.get('y', 0.0))))
        cam_z = float(_param_or_cfg('cam_z', float(ext_cfg.get('z', 0.17))))
        roll = float(_param_or_cfg('cam_roll', float(ext_cfg.get('roll', 0.0))))
        pitch = float(_param_or_cfg('cam_pitch', float(ext_cfg.get('pitch', 0.0))))
        yaw = float(_param_or_cfg('cam_yaw', float(ext_cfg.get('yaw', 0.0))))
        use_deg = bool(_param_or_cfg('cam_use_degrees', bool(ext_cfg.get('use_degrees', True))))
        publish_cam_tf = bool(_param_or_cfg('publish_cam_tf', True))
        self.scan_sync_slop_sec = float(_param_or_cfg('scan_sync_slop_sec', 0.25))
        self.require_scan_sync = bool(_param_or_cfg('require_scan_sync', True))
        self.compensate_clock_offset = bool(_param_or_cfg('compensate_clock_offset', True))
        self.clock_offset_threshold_sec = float(_param_or_cfg('clock_offset_threshold_sec', 0.12))
        self.lidar_cluster_mode = str(_param_or_cfg('lidar_cluster_mode', 'front_valid')).strip().lower()
        self.lidar_cluster_tol_m = max(0.10, float(_param_or_cfg('lidar_cluster_tol_m', 0.35)))
        self.lidar_min_cluster_rays = max(1, int(_param_or_cfg('lidar_min_cluster_rays', 2)))
        self.use_pose_sector = bool(_param_or_cfg('use_pose_sector', True))
        self.use_kinematic_gating = bool(_param_or_cfg('use_kinematic_gating', True))
        # 관측 전용(publish_target_topics=false)일 때는 기본적으로 도킹 중인 로봇의 라이다 모터를 임의로 켜지 않음
        publish_topics = bool(_param_or_cfg('publish_target_topics', True))
        self.auto_start_lidar_motor = bool(
            _param_or_cfg('auto_start_lidar_motor', publish_topics)
        )

        self.xyz = (cam_x, cam_y, cam_z)
        if use_deg:
            self.rpy_rad = (math.radians(roll), math.radians(pitch), math.radians(yaw))
        else:
            self.rpy_rad = (roll, pitch, yaw)

        self.tf_topic = f'{self.ns_prefix}/tf' if self.ns_prefix else '/tf'
        self.tf_static_topic = f'{self.ns_prefix}/tf_static' if self.ns_prefix else '/tf_static'
        self.scan_topic = f'{self.ns_prefix}/scan' if self.ns_prefix else '/scan'
        self.start_motor_srv = f'{self.ns_prefix}/start_motor' if self.ns_prefix else '/start_motor'

        # 1. TF Buffer 및 /robot4/tf, /robot4/tf_static 직접 구독
        self.tf_buffer = Buffer()
        static_qos = QoSProfile(
            depth=20,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.tf_static_pub = (
            node.create_publisher(TFMessage, self.tf_static_topic, static_qos)
            if publish_cam_tf
            else None
        )
        if publish_cam_tf:
            self._publish_static_cam_tf()

        self.tf_sub = node.create_subscription(
            TFMessage, self.tf_topic, self._on_tf_msg, qos_profile_sensor_data
        )
        self.tf_static_sub = node.create_subscription(
            TFMessage, self.tf_static_topic, self._on_tf_static_msg, static_qos
        )

        # 2. 2D LiDAR (/robot4/scan) 버퍼 구독 및 도킹 시 모터 자동 기동 클라이언트
        self.latest_scan: Optional[LaserScan] = None
        self.last_scan_WallTime: float = 0.0
        self.scan_fps: float = 0.0
        self._scan_lock = threading.Lock()
        self._scan_buffer: deque = deque(maxlen=40)
        self._skew_samples: deque = deque(maxlen=60)
        self._last_skew_scan_sec: float = -1.0
        self.last_clock_offset_sec: float = 0.0
        self._last_skew_log_wall: float = 0.0
        self.scan_sub = node.create_subscription(
            LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data
        )
        self.motor_client = (
            node.create_client(Empty, self.start_motor_srv)
            if self.auto_start_lidar_motor
            else None
        )
        self._motor_requested = False
        self._startup_time = time.time()
        self._watchdog_timer = (
            node.create_timer(2.5, self._check_lidar_motor)
            if self.auto_start_lidar_motor
            else None
        )

        # 3. 타겟 인물 위치 발행 토픽 (로봇 기준 / 맵 기준 / RViz 마커)
        self.pub_point_base = node.create_publisher(
            PointStamped, f'{self.ns_prefix}/target_person/point_base', 10
        )
        self.pub_point_map = node.create_publisher(
            PointStamped, f'{self.ns_prefix}/target_person/point_map', 10
        )
        self.pub_marker = node.create_publisher(
            Marker, f'{self.ns_prefix}/target_person/marker', 10
        )

        # 타겟 이동 방향/속도 추적용 상태 (odom 고정 좌표계 기준)
        self._last_track_id: Optional[int] = None
        self._last_odom_xy: Optional[np.ndarray] = None
        self._last_odom_vel: np.ndarray = np.zeros(2, dtype=np.float64)
        self._last_valid_time: float = 0.0
        self._last_cam_dist: Optional[float] = None

    def _load_yaml_defaults(self) -> dict:
        return load_phone_camera_config()

    def _publish_static_cam_tf(self):
        now = self.node.get_clock().now().to_msg()
        t_link = make_static_transform(
            now, self.parent_frame, self.camera_link_frame, self.xyz, self.rpy_rad
        )
        t_opt = make_static_transform(
            now,
            self.camera_link_frame,
            self.optical_frame,
            (0.0, 0.0, 0.0),
            (-math.pi / 2.0, 0.0, -math.pi / 2.0),
        )
        msg = TFMessage(transforms=[t_link, t_opt])
        self.tf_static_pub.publish(msg)
        for t in msg.transforms:
            self.tf_buffer.set_transform_static(t, 'local_static')

    def _on_tf_msg(self, msg: TFMessage):
        for t in msg.transforms:
            self.tf_buffer.set_transform(t, 'default_authority')

    def _on_tf_static_msg(self, msg: TFMessage):
        for t in msg.transforms:
            self.tf_buffer.set_transform_static(t, 'default_authority')

    def _on_scan(self, msg: LaserScan):
        now = time.time()
        if self.last_scan_WallTime > 0.0:
            dt = now - self.last_scan_WallTime
            if dt > 0:
                inst = 1.0 / dt
                self.scan_fps = 0.85 * self.scan_fps + 0.15 * inst if self.scan_fps > 0 else inst
        self.last_scan_WallTime = now
        self.latest_scan = msg
        scan_sec = _stamp_to_sec(msg.header.stamp)
        if scan_sec <= 0.0:
            scan_sec = self.node.get_clock().now().nanoseconds * 1e-9
        with self._scan_lock:
            self._scan_buffer.append((scan_sec, now, msg))

    def _estimate_clock_offset_sec(
        self,
        frame_sec: float,
        frame_rx_wall: float,
        live: list,
    ) -> float:
        """수신 wall-clock 기준 동시 도착 스캔과 이미지 간 영구 시계 오차(scan - cam)를 추정합니다."""
        if not self.compensate_clock_offset or frame_sec <= 0.0 or not live:
            return 0.0

        nearest_s, nearest_w, _ = min(live, key=lambda item: abs(item[1] - frame_rx_wall))
        if (
            abs(nearest_w - frame_rx_wall) <= 0.18
            and nearest_s > 0.0
            and nearest_s != self._last_skew_scan_sec
        ):
            sample = (nearest_s - frame_sec) - (nearest_w - frame_rx_wall)
            if abs(sample) < 30.0:
                self._skew_samples.append(sample)
                self._last_skew_scan_sec = nearest_s

        if not self._skew_samples:
            return 0.0
        est = float(np.median(self._skew_samples))
        if len(self._skew_samples) < 3 and abs(est) < 0.35:
            return 0.0
        if abs(est) < self.clock_offset_threshold_sec:
            self.last_clock_offset_sec = 0.0
            return 0.0
        self.last_clock_offset_sec = est
        if (frame_rx_wall - self._last_skew_log_wall) >= 10.0:
            self._last_skew_log_wall = frame_rx_wall
            self.node.get_logger().warn(
                f'[시간 오차 보정] scan-camera 시계 차이 {est * 1000.0:+.0f}ms 감지 -> 자동 오프셋 매칭 적용중'
            )
        return est

    def _select_scan_for_stamp(
        self,
        frame_stamp: Optional[BuiltinTime],
        frame_rx_wall: Optional[float] = None,
    ) -> Tuple[Optional[LaserScan], Optional[float], bool, float]:
        """이미지 타임스탬프와 가장 가까운 LiDAR 스캔을 선택합니다(호스트 시계 오차 자동 보정 포함)."""
        now_wall = time.time()
        rx_wall = frame_rx_wall if (frame_rx_wall is not None and frame_rx_wall > 0.0) else now_wall
        with self._scan_lock:
            live = [(s_sec, w_t, m) for (s_sec, w_t, m) in self._scan_buffer if (now_wall - w_t) < 2.5]
        if not live:
            if self.latest_scan is not None and (now_wall - self.last_scan_WallTime) < 2.5:
                return self.latest_scan, None, False, 0.0
            return None, None, False, 0.0

        frame_sec = _stamp_to_sec(frame_stamp)
        if frame_sec <= 0.0:
            return live[-1][2], None, False, 0.0

        clock_offset = self._estimate_clock_offset_sec(frame_sec, rx_wall, live)
        target_sec = frame_sec + clock_offset
        best_sec, _, best_msg = min(live, key=lambda item: abs(item[0] - target_sec))
        abs_dt = abs(best_sec - target_sec)
        dt_ms = (best_sec - target_sec) * 1000.0
        if abs_dt <= self.scan_sync_slop_sec:
            return best_msg, dt_ms, True, clock_offset
        if not self.require_scan_sync:
            return best_msg, dt_ms, False, clock_offset
        return None, dt_ms, False, clock_offset

    def _compute_half_sector(
        self,
        target: UnifiedTrackedPerson,
        half_w: float,
        fx: float,
    ) -> float:
        """타겟 bbox(및 가능 시 Pose 좌우 몸통 관절 폭)에서 LiDAR 매칭 수평 반각을 계산합니다."""
        x1, _, x2, _ = target.bbox
        alpha_left = math.atan2(-(x1 - half_w), fx)
        alpha_right = math.atan2(-(x2 - half_w), fx)
        bbox_span = abs(alpha_left - alpha_right)
        half_sector = max(math.radians(3.5), min(math.radians(18.0), bbox_span * 0.35))

        if not self.use_pose_sector or target.keypoints is None or len(target.keypoints) < 13:
            return half_sector

        kpts = target.keypoints
        left_xs = [
            float(kpts[idx][0])
            for idx in (5, 11)
            if len(kpts[idx]) >= 3 and float(kpts[idx][2]) >= 0.35 and x1 <= float(kpts[idx][0]) <= x2
        ]
        right_xs = [
            float(kpts[idx][0])
            for idx in (6, 12)
            if len(kpts[idx]) >= 3 and float(kpts[idx][2]) >= 0.35 and x1 <= float(kpts[idx][0]) <= x2
        ]
        if not left_xs or not right_xs:
            return half_sector

        pad = max(12.0, 0.12 * max(1.0, float(x2 - x1)))
        sx1 = max(float(x1), min(left_xs + right_xs) - pad)
        sx2 = min(float(x2), max(left_xs + right_xs) + pad)
        if sx2 <= sx1:
            return half_sector
        torso_span = abs(math.atan2(-(sx1 - half_w), fx) - math.atan2(-(sx2 - half_w), fx))
        sector_span = min(bbox_span * 0.70, max(math.radians(5.0), torso_span * 1.15))
        return max(math.radians(3.0), min(math.radians(15.0), sector_span * 0.5))

    def _select_cluster_distance(self, dists: np.ndarray) -> float:
        """섹터 내 라이다 거리 배열에서 배경 벽면 대신 최전방 유효 다리/몸통 클러스터 거리를 선택합니다."""
        if self.lidar_cluster_mode == 'p25' or dists.size <= 1:
            p25 = float(np.percentile(dists, 25))
            cluster = dists[(dists >= p25 - 0.15) & (dists <= p25 + 0.25)]
            return float(np.mean(cluster)) if cluster.size > 0 else p25

        s = np.sort(dists)
        tol = self.lidar_cluster_tol_m
        min_rays = self.lidar_min_cluster_rays
        for anchor in s:
            cluster = s[(s >= anchor) & (s <= anchor + tol)]
            if cluster.size >= min_rays:
                return float(np.mean(cluster))
        # 전 구간에 2개 이상 모인 클러스터가 없을 때만 가장 가까운 포인트로 폴백
        return float(s[0])

    def _check_lidar_motor(self):
        """도킹 스테이션 위에서 라이다 모터가 멈춰 있을 때 자동으로 1회 기동."""
        if not self.auto_start_lidar_motor or self.motor_client is None or self._motor_requested:
            return
        if time.time() - self.last_scan_WallTime > 2.2:
            self._motor_requested = True
            if self.motor_client.service_is_ready():
                self.node.get_logger().info(
                    f'[LiDAR] {self.scan_topic} 수신 대기 중 -> {self.start_motor_srv} 모터 기동 요청'
                )
                self.motor_client.call_async(Empty.Request())

    def _lookup(
        self, target_frame: str, source_frame: str, stamp: Optional[Time] = None
    ) -> Optional[TransformStamped]:
        for t in ([stamp] if stamp is not None else []) + [Time()]:
            try:
                if self.tf_buffer.can_transform(target_frame, source_frame, t):
                    return self.tf_buffer.lookup_transform(target_frame, source_frame, t)
            except Exception:
                pass
        return None

    def _get_map_from_base_matrix(
        self, stamp: Optional[Time] = None
    ) -> Tuple[Optional[np.ndarray], str]:
        """
        1순위: AMCL이 발행하는 map -> base_link TF 사용 ('AMCL')
        2순위: 도킹 등으로 라이다가 0Hz라 AMCL TF가 아직 없을 때,
               initial_pose + (시작 시점 대비 odom 변화량)으로 map 좌표 계산 ('Init+Odom')
        """
        tf_map_base = self._lookup(self.map_frame, self.base_frame, stamp)
        if tf_map_base is not None:
            return transform_to_matrix(tf_map_base), 'AMCL'

        tf_odom_base = self._lookup(self.odom_frame, self.base_frame, stamp)
        if tf_odom_base is not None:
            mat_odom_base = transform_to_matrix(tf_odom_base)
            if self._initial_odom_inv is None:
                self._initial_odom_inv = np.linalg.inv(mat_odom_base)
            # 시작 시점 대비 상대 이동 행렬 (delta_odom)
            mat_delta = mat_odom_base @ self._initial_odom_inv

            ix, iy, iyaw = self.init_map_xy_yaw
            c, s = math.cos(iyaw), math.sin(iyaw)
            mat_init = np.array(
                [
                    [c, -s, 0.0, ix],
                    [s,  c, 0.0, iy],
                    [0.0, 0.0, 1.0, 0.0],
                    [0.0, 0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )
            return mat_init @ mat_delta, 'Init+Odom'

        return None, 'None'

    def get_camera_pose_status(self) -> CameraPoseStatus:
        tf_base = self._lookup(self.base_frame, self.camera_link_frame)
        tf_odom = self._lookup(self.odom_frame, self.camera_link_frame)
        tf_map = self._lookup(self.map_frame, self.camera_link_frame)

        base_xyz = None
        if tf_base is not None:
            tr = tf_base.transform.translation
            base_xyz = (tr.x, tr.y, tr.z)

        odom_xyz = None
        if tf_odom is not None:
            tr = tf_odom.transform.translation
            odom_xyz = (tr.x, tr.y, tr.z)

        map_xyz = None
        map_yaw_deg = None
        map_source = 'None'
        if tf_map is not None:
            tr = tf_map.transform.translation
            rot = tf_map.transform.rotation
            _, _, yaw = quaternion_to_euler(rot.x, rot.y, rot.z, rot.w)
            map_xyz = (tr.x, tr.y, tr.z)
            map_yaw_deg = math.degrees(yaw)
            map_source = 'AMCL'
        else:
            mat_map_base, src = self._get_map_from_base_matrix()
            if mat_map_base is not None and base_xyz is not None:
                map_xyz = apply_transform_point(mat_map_base, base_xyz)
                map_yaw_deg = math.degrees(math.atan2(mat_map_base[1, 0], mat_map_base[0, 0]))
                map_source = src

        scan_fresh = (time.time() - self.last_scan_WallTime) < 2.5
        lidar_hz = self.scan_fps if scan_fresh else 0.0
        lidar_frame = self.latest_scan.header.frame_id if self.latest_scan is not None else 'rplidar_link'

        return CameraPoseStatus(
            base_xyz=base_xyz,
            odom_xyz=odom_xyz,
            map_xyz=map_xyz,
            map_yaw_deg=map_yaw_deg,
            map_source=map_source,
            lidar_hz=lidar_hz,
            lidar_frame=lidar_frame,
        )

    def reset_kinematics(self, track_id: Optional[int] = None):
        """타겟 이동 속도·위치 이력을 초기화합니다 (관람 종료/타겟 변경 시 호출)."""
        self._last_track_id = track_id
        self._last_odom_xy = None
        self._last_odom_vel = np.zeros(2, dtype=np.float64)
        self._last_valid_time = 0.0
        self._last_cam_dist = None

    def _reset_kinematics(self, track_id: Optional[int] = None):
        self.reset_kinematics(track_id)

    def _cluster_sector_points(
        self, xc: np.ndarray, yc: np.ndarray, gap: float = 0.25
    ):
        """부채꼴 내 라이다 점을 거리순으로 묶어 클러스터 목록[(dist, cx, cy, width, count)]을 반환."""
        dists = np.hypot(xc, yc)
        order = np.argsort(dists)
        ds, xs, ys = dists[order], xc[order], yc[order]

        cuts = np.nonzero(np.diff(ds) > gap)[0] + 1
        clusters = []
        for idx in np.split(np.arange(ds.size), cuts):
            if idx.size == 0:
                continue
            cx = float(np.mean(xs[idx]))
            cy = float(np.mean(ys[idx]))
            width = float(math.hypot(float(np.ptp(xs[idx])), float(np.ptp(ys[idx]))))
            clusters.append((float(np.mean(ds[idx])), cx, cy, width, int(idx.size)))
        return clusters

    def _score_odom_candidate(
        self, cand_xy: np.ndarray, dt: float
    ) -> Tuple[bool, float]:
        """직전 이동 방향·속도 대비 후보 좌표(odom)의 물리적 자연스러움을 평가 (통과 여부, 점수)."""
        if self._last_odom_xy is None or dt <= 1e-3 or dt > 2.5:
            return True, 0.0

        prev_xy = self._last_odom_xy
        prev_vel = self._last_odom_vel
        pred_xy = prev_xy + prev_vel * min(dt, 0.8)
        err = cand_xy - pred_xy
        disp = cand_xy - prev_xy
        disp_norm = float(np.linalg.norm(disp))
        speed = float(np.linalg.norm(prev_vel))

        # 1) 저속/정지 상태: 방향 벡터가 노이즈이므로 원형 게이팅만 적용
        if speed < 0.25:
            lim = 0.45 + 1.0 * dt
            err_norm = float(np.linalg.norm(err))
            return (err_norm <= lim), (err_norm / lim)

        # 2) 이동 중: 진행 방향(전방/후방)과 측면으로 분해한 비대칭 게이팅
        u = prev_vel / speed
        e_par = float(np.dot(err, u))                       # + 진행 방향, - 반대 방향
        e_perp = float(np.linalg.norm(err - e_par * u))     # 좌우 이탈

        lim_par = (0.55 + 1.2 * dt) if e_par >= 0.0 else (0.22 + 0.30 * dt)
        lim_lat = 0.40 + 0.8 * dt
        norm_d = math.hypot(e_par / lim_par, e_perp / lim_lat)
        if norm_d > 1.0:
            return False, float('inf')

        # 3) 급가속 및 갑작스러운 역방향 급변 차단
        dt_eff = max(dt, 0.05)
        v_inst = disp / dt_eff
        v_inst_norm = float(np.linalg.norm(v_inst))
        accel = float(np.linalg.norm(v_inst - prev_vel)) / dt_eff
        if accel > 4.5 and disp_norm > 0.30:
            return False, float('inf')

        cos_sim = float(np.dot(prev_vel, v_inst)) / (speed * v_inst_norm + 1e-6)
        if cos_sim < -0.30 and v_inst_norm > 0.40 and disp_norm > 0.25:
            return False, float('inf')

        return True, norm_d

    def _update_kinematics(self, odom_xy: np.ndarray, now: float, cam_dist: float):
        if self._last_odom_xy is not None and 0.02 < (now - self._last_valid_time) < 2.0:
            dt = now - self._last_valid_time
            inst_vel = (odom_xy - self._last_odom_xy) / dt
            vel = 0.6 * self._last_odom_vel + 0.4 * inst_vel
            spd = float(np.linalg.norm(vel))
            if spd > 1.6:
                vel *= 1.6 / spd
            self._last_odom_vel = vel
        else:
            self._last_odom_vel = np.zeros(2, dtype=np.float64)
        self._last_odom_xy = odom_xy
        self._last_valid_time = now
        self._last_cam_dist = cam_dist

    def localize_target(
        self,
        target: UnifiedTrackedPerson,
        frame_width: int,
        frame_height: int,
        publish: bool = True,
        frame_stamp: Optional[BuiltinTime] = None,
        frame_rx_wall: Optional[float] = None,
        use_kinematics: bool = True,
    ) -> TargetLocalizationResult:
        x1, y1, x2, y2 = target.bbox
        kin_active = self.use_kinematic_gating and bool(use_kinematics)
        now_wall = frame_rx_wall if (frame_rx_wall is not None and frame_rx_wall > 0.0) else time.time()
        if kin_active and (
            self._last_track_id != target.track_id
            or (self._last_valid_time > 0.0 and (now_wall - self._last_valid_time) > 2.2)
        ):
            self.reset_kinematics(target.track_id)

        occluded = False
        cx, cy = target.center

        half_w = max(1.0, frame_width * 0.5)
        fx = half_w / math.tan(math.radians(self.hfov_deg * 0.5))
        fy = fx

        # top_cam_link 기준 (+X 정면, +Y 왼쪽, +Z 위):
        # 이미지 오른쪽(cx > W/2)은 -Y 방향, 이미지 왼쪽(cx < W/2)은 +Y 방향
        alpha_c = math.atan2(-(cx - half_w), fx)
        half_sector = self._compute_half_sector(target, half_w, fx)

        cam_dist_m: Optional[float] = None
        matched_rays = 0
        source = 'CamEst'
        frame_time = Time.from_msg(frame_stamp) if _stamp_to_sec(frame_stamp) > 0.0 else None

        # 1. 이미지 시각과 동기화된 라이다 스캔을 선택해 top_cam_link 좌표계로 변환
        scan, sync_dt_ms, _, clock_offset = self._select_scan_for_stamp(
            frame_stamp, frame_rx_wall=frame_rx_wall
        )
        scan_time: Optional[Time] = None
        if scan is not None:
            lidar_frame = scan.header.frame_id or 'rplidar_link'
            scan_time = (
                Time.from_msg(scan.header.stamp)
                if _stamp_to_sec(scan.header.stamp) > 0.0
                else frame_time
            )
            tf_cam_from_lidar = self._lookup(self.camera_link_frame, lidar_frame, scan_time)
            if tf_cam_from_lidar is not None:
                mat_cam_from_lidar = transform_to_matrix(tf_cam_from_lidar)
            else:
                # TF 미수신 시 로봇 기본 구조 기반 폴백 (rplidar_link는 base_link 대비 +90deg 회전)
                mat_cam_from_lidar = np.array(
                    [
                        [0.0, -1.0, 0.0, 0.02],
                        [1.0, 0.0, 0.0, 0.00],
                        [0.0, 0.0, 1.0, -0.221],
                        [0.0, 0.0, 0.0, 1.0],
                    ],
                    dtype=np.float64,
                )

            ranges = np.asarray(scan.ranges, dtype=np.float64)
            n_pts = len(ranges)
            if n_pts > 0:
                angles = scan.angle_min + np.arange(n_pts, dtype=np.float64) * scan.angle_increment
                r_min = max(0.12, float(scan.range_min))
                r_max = min(12.0, float(scan.range_max) if scan.range_max > 0 else 12.0)
                valid = np.isfinite(ranges) & (ranges >= r_min) & (ranges <= r_max)

                if np.any(valid):
                    rv = ranges[valid]
                    av = angles[valid]
                    pts_lidar = np.vstack(
                        (rv * np.cos(av), rv * np.sin(av), np.zeros_like(rv), np.ones_like(rv))
                    )
                    pts_cam = mat_cam_from_lidar @ pts_lidar
                    xc = pts_cam[0, :]
                    yc = pts_cam[1, :]

                    front_mask = xc > 0.12
                    if np.any(front_mask):
                        xc = xc[front_mask]
                        yc = yc[front_mask]
                        betas = np.arctan2(yc, xc)
                        diffs = np.abs(np.arctan2(np.sin(betas - alpha_c), np.cos(betas - alpha_c)))
                        in_sector = diffs <= half_sector

                        if np.any(in_sector):
                            if not kin_active:
                                dists = np.hypot(xc[in_sector], yc[in_sector])
                                matched_rays = int(dists.size)
                                cam_dist_m = self._select_cluster_distance(dists)
                                source = 'LiDAR'
                            else:
                                tf_odom_from_cam = self._lookup(
                                    self.odom_frame,
                                    self.camera_link_frame,
                                    scan_time if (abs(clock_offset) > 0.0 and scan_time is not None) else (scan_time or frame_time),
                                )
                                mat_odom_from_cam = (
                                    transform_to_matrix(tf_odom_from_cam)
                                    if tf_odom_from_cam is not None
                                    else None
                                )
                                clusters = self._cluster_sector_points(
                                    xc[in_sector], yc[in_sector], gap=0.25
                                )
                                # 1-ray 단발 노이즈에 첫 프레임이 잠기지 않도록 다중 ray 클러스터 우선
                                valid_clusters = [
                                    c for c in clusters if c[4] >= self.lidar_min_cluster_rays
                                ]
                                cand_clusters = valid_clusters if valid_clusters else clusters
                                dt = now_wall - self._last_valid_time if self._last_valid_time > 0.0 else 0.0
                                best_cand = None  # (score, dist, count, odom_xy)

                                for c_dist, cx_c, cy_c, c_width, c_cnt in cand_clusters:
                                    # 사람 체폭을 크게 넘는 넓은 벽/전시대 클러스터 페널티
                                    width_pen = 0.6 if c_width > 1.0 else 0.0

                                    if mat_odom_from_cam is not None:
                                        ox, oy, _ = apply_transform_point(
                                            mat_odom_from_cam, (cx_c, cy_c, 0.0)
                                        )
                                        cand_xy = np.array([ox, oy], dtype=np.float64)
                                        ok, score = self._score_odom_candidate(cand_xy, dt)
                                        if not ok:
                                            continue
                                    else:
                                        cand_xy = None
                                        lim_fb = 0.45 + 1.2 * max(dt, 0.05)
                                        if (
                                            self._last_cam_dist is not None
                                            and abs(c_dist - self._last_cam_dist) > lim_fb
                                        ):
                                            continue
                                        score = (
                                            abs(c_dist - self._last_cam_dist)
                                            if self._last_cam_dist is not None
                                            else c_dist
                                        )

                                    total_score = score + width_pen
                                    if best_cand is None or total_score < best_cand[0]:
                                        best_cand = (total_score, c_dist, c_cnt, cand_xy)

                                if best_cand is not None:
                                    _, cam_dist_m, matched_rays, best_odom_xy = best_cand
                                    source = 'LiDAR'
                                    if best_odom_xy is not None:
                                        self._update_kinematics(best_odom_xy, now_wall, cam_dist_m)
                                    else:
                                        self._last_cam_dist = cam_dist_m
                                        self._last_valid_time = now_wall
                                else:
                                    # 부채꼴에 라이다 점은 있으나 모두 앞 장애물(물리적 불가)로 탈락 → 가림
                                    occluded = True

        # 2. 라이다 포인트가 없을 때 폴백 (단, 앞 장애물 가림 판정 시에는 bbox 추정·발행 모두 보류)
        if cam_dist_m is None:
            if occluded and self._last_cam_dist is not None:
                cam_dist_m = self._last_cam_dist
                source = 'Occluded'
            else:
                bbox_h = max(24.0, float(y2 - y1))
                cam_dist_m = float(np.clip((1.65 * fy) / bbox_h, 0.35, 8.0))
                source = 'CamEst'


        # 상단 카메라(top_cam_link) 기준 타겟 3D 좌표
        cam_x = cam_dist_m * math.cos(alpha_c)
        cam_y = cam_dist_m * math.sin(alpha_c)
        cam_z = 0.0
        cam_xyz = (cam_x, cam_y, cam_z)

        # 3. 로봇 몸체(base_link) 기준 타겟 3D 좌표 변환
        tf_base_from_cam = self._lookup(self.base_frame, self.camera_link_frame, frame_time)
        if tf_base_from_cam is not None:
            mat_base_from_cam = transform_to_matrix(tf_base_from_cam)
            base_xyz = apply_transform_point(mat_base_from_cam, cam_xyz)
        else:
            # base_link -> top_cam_link 기본 오프셋 폴백 (-0.06m, 0.0m, +0.414m)
            base_xyz = (cam_x - 0.060, cam_y, cam_z)

        base_dist_m = math.hypot(base_xyz[0], base_xyz[1])
        base_angle_deg = math.degrees(math.atan2(base_xyz[1], base_xyz[0]))

        # 4. 저장된 맵(map) 기준 타겟 3D 좌표 변환 (AMCL 또는 도킹 시 Init+Odom 폴백)
        # 시계 오차가 보정된 경우 로봇 TF(scan_time)와 출력 stamp(scan.header.stamp)를 로봇 시계에 정렬
        use_scan_clock = (abs(clock_offset) > 0.0 and scan is not None and scan_time is not None)
        robot_lookup_time = scan_time if use_scan_clock else frame_time
        map_xyz: Optional[Tuple[float, float, float]] = None
        mat_map_from_base, map_source = self._get_map_from_base_matrix(robot_lookup_time)
        if mat_map_from_base is not None:
            map_xyz = apply_transform_point(mat_map_from_base, base_xyz)

        res = TargetLocalizationResult(
            track_id=target.track_id,
            source=source,
            cam_bearing_deg=math.degrees(alpha_c),
            cam_dist_m=cam_dist_m,
            cam_xyz=cam_xyz,
            base_xyz=base_xyz,
            base_dist_m=base_dist_m,
            base_angle_deg=base_angle_deg,
            map_xyz=map_xyz,
            map_source=map_source,
            matched_rays=matched_rays,
            sync_dt_ms=sync_dt_ms,
            clock_offset_ms=(clock_offset * 1000.0 if abs(clock_offset) > 0.0 else None),
        )
        if publish and not occluded:
            out_stamp = scan.header.stamp if use_scan_clock else frame_stamp
            self._publish_target_outputs(res, out_stamp=out_stamp)
        return res

    def _publish_target_outputs(
        self,
        res: TargetLocalizationResult,
        out_stamp: Optional[BuiltinTime] = None,
    ):
        now_msg = (
            out_stamp
            if _stamp_to_sec(out_stamp) > 0.0
            else self.node.get_clock().now().to_msg()
        )

        pt_base = PointStamped()
        pt_base.header.stamp = now_msg
        pt_base.header.frame_id = self.base_frame
        pt_base.point.x = res.base_xyz[0]
        pt_base.point.y = res.base_xyz[1]
        pt_base.point.z = 0.0
        self.pub_point_base.publish(pt_base)

        if res.map_xyz is not None:
            pt_map = PointStamped()
            pt_map.header.stamp = now_msg
            pt_map.header.frame_id = self.map_frame
            pt_map.point.x = res.map_xyz[0]
            pt_map.point.y = res.map_xyz[1]
            pt_map.point.z = 0.0
            self.pub_point_map.publish(pt_map)

        marker = Marker()
        marker.header.stamp = now_msg
        if res.map_xyz is not None:
            marker.header.frame_id = self.map_frame
            mx, my = res.map_xyz[0], res.map_xyz[1]
        else:
            marker.header.frame_id = self.base_frame
            mx, my = res.base_xyz[0], res.base_xyz[1]

        marker.ns = 'target_person'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = mx
        marker.pose.position.y = my
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 1.0
        marker.color.a = 0.85
        if res.source == 'LiDAR':
            marker.color.r = 0.1
            marker.color.g = 1.0
            marker.color.b = 0.3
        else:
            marker.color.r = 1.0
            marker.color.g = 0.75
            marker.color.b = 0.1
        self.pub_marker.publish(marker)
