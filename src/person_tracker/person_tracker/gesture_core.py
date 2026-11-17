from dataclasses import dataclass
import math
import time
from typing import Dict, List, Optional, Tuple

import cv2
import mediapipe as mp
from mediapipe.tasks.python import BaseOptions
from mediapipe.tasks.python.vision import HandLandmarker, HandLandmarkerOptions, RunningMode
import numpy as np

from person_tracker.env_bootstrap import resolve_model_path


HAND_CONNECTIONS: Tuple[Tuple[int, int], ...] = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
)

GESTURE_IDLE = 'IDLE'
GESTURE_FIST = 'FIST'
GESTURE_OPEN_PALM = 'OPEN_PALM'
GESTURE_UNKNOWN = 'UNKNOWN'

STATE_IDLE = 'IDLE'
STATE_FIST_ARMED = 'FIST_ARMED'
STATE_TRIGGERED = 'TRIGGERED'


@dataclass
class HandGestureObservation:
    """단일 손 검출 결과 및 손가락 개폐/제스처 판별 정보."""

    gesture: str                            # FIST | OPEN_PALM | UNKNOWN
    handedness: str                         # Left | Right | Unknown
    score: float
    landmarks_px: np.ndarray                # (21, 2) int32 원본 프레임 좌표
    hand_bbox: Tuple[int, int, int, int]    # (x1, y1, x2, y2)
    finger_states: Tuple[bool, bool, bool, bool, bool]  # [엄지, 검지, 중지, 약지, 소지]
    extended_count: int
    palm_scale_px: float


@dataclass
class GestureProgress:
    """특정 대상(subject_id)에 대한 다중 프레임 '주먹->펴기' 상태머신 결과."""

    subject_id: int
    state: str                              # IDLE | FIST_ARMED | TRIGGERED
    last_gesture: str
    fist_count: int
    fist_target: int
    open_count: int
    open_target: int
    progress: float                         # 0.0 ~ 1.0 전체 진행률
    just_triggered: bool
    observation: Optional[HandGestureObservation] = None


def _dist(pt_a: np.ndarray, pt_b: np.ndarray) -> float:
    return float(math.hypot(float(pt_a[0] - pt_b[0]), float(pt_a[1] - pt_b[1])))


def classify_hand_gesture(
    landmarks_px: np.ndarray,
) -> Tuple[str, Tuple[bool, bool, bool, bool, bool], int, float]:
    """21개 손 랜드마크(px)로 5개 손가락 펼침 여부와 FIST/OPEN_PALM/UNKNOWN을 판정합니다."""
    wrist = landmarks_px[0]
    middle_mcp = landmarks_px[9]
    palm_scale = _dist(wrist, middle_mcp)
    if palm_scale < 6.0:
        return GESTURE_UNKNOWN, (False, False, False, False, False), 0, palm_scale

    index_mcp = landmarks_px[5]
    pinky_mcp = landmarks_px[17]
    thumb_ip = landmarks_px[3]
    thumb_tip = landmarks_px[4]

    thumb_open = (
        _dist(thumb_tip, pinky_mcp) > _dist(thumb_ip, pinky_mcp) * 1.10
        and _dist(thumb_tip, index_mcp) > palm_scale * 0.42
    )

    four_open: List[bool] = []
    for mcp_i, pip_i, tip_i in ((5, 6, 8), (9, 10, 12), (13, 14, 16), (17, 18, 20)):
        d_tip = _dist(wrist, landmarks_px[tip_i])
        d_pip = _dist(wrist, landmarks_px[pip_i])
        d_mcp = _dist(wrist, landmarks_px[mcp_i])
        is_open = (d_tip > d_pip * 1.15) and (d_tip > d_mcp * 1.30)
        four_open.append(is_open)

    finger_states = (thumb_open, four_open[0], four_open[1], four_open[2], four_open[3])
    four_count = sum(1 for v in four_open if v)
    total_count = sum(1 for v in finger_states if v)

    if four_count == 0 and (not thumb_open or _dist(thumb_tip, index_mcp) < palm_scale * 0.78):
        gesture = GESTURE_FIST
    elif four_count == 4 or (four_count >= 3 and thumb_open and total_count >= 4):
        gesture = GESTURE_OPEN_PALM
    else:
        gesture = GESTURE_UNKNOWN

    return gesture, finger_states, total_count, palm_scale


class HandGestureDetector:
    """MediaPipe HandLandmarker 기반 손 검출 및 주먹/손바닥 분류기."""

    def __init__(
        self,
        model_name: str = 'hand_landmarker.task',
        num_hands: int = 2,
        min_detection_confidence: float = 0.60,
        min_presence_confidence: float = 0.60,
        min_tracking_confidence: float = 0.55,
    ):
        model_path = resolve_model_path(model_name)
        options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=RunningMode.IMAGE,
            num_hands=num_hands,
            min_hand_detection_confidence=min_detection_confidence,
            min_hand_presence_confidence=min_presence_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self._landmarker = HandLandmarker.create_from_options(options)
        self.model_path = model_path

    def detect_in_roi(
        self,
        frame_bgr: np.ndarray,
        roi: Optional[Tuple[int, int, int, int]] = None,
    ) -> List[HandGestureObservation]:
        """프레임 전체 또는 지정된 ROI 내부에서 손을 검출하고 원본 좌표계로 복원합니다."""
        h, w = frame_bgr.shape[:2]
        if roi is None:
            rx1, ry1, rx2, ry2 = 0, 0, w, h
        else:
            rx1 = max(0, int(roi[0]))
            ry1 = max(0, int(roi[1]))
            rx2 = min(w, int(roi[2]))
            ry2 = min(h, int(roi[3]))

        rw, rh = rx2 - rx1, ry2 - ry1
        if rw < 24 or rh < 24:
            return []

        crop_bgr = frame_bgr[ry1:ry2, rx1:rx2]
        crop_rgb = np.ascontiguousarray(cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2RGB))
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=crop_rgb)
        result = self._landmarker.detect(mp_image)

        observations: List[HandGestureObservation] = []
        if not result.hand_landmarks:
            return observations

        for idx, lm_list in enumerate(result.hand_landmarks):
            pts = np.empty((21, 2), dtype=np.int32)
            for i, lm in enumerate(lm_list):
                pts[i, 0] = int(round(rx1 + lm.x * rw))
                pts[i, 1] = int(round(ry1 + lm.y * rh))

            x1 = max(0, int(np.min(pts[:, 0])))
            y1 = max(0, int(np.min(pts[:, 1])))
            x2 = min(w - 1, int(np.max(pts[:, 0])))
            y2 = min(h - 1, int(np.max(pts[:, 1])))

            handedness = 'Unknown'
            score = 0.0
            if result.handedness and idx < len(result.handedness) and result.handedness[idx]:
                cat = result.handedness[idx][0]
                handedness = cat.category_name or 'Unknown'
                score = float(cat.score)

            gesture, finger_states, ext_count, palm_scale = classify_hand_gesture(pts)
            observations.append(
                HandGestureObservation(
                    gesture=gesture,
                    handedness=handedness,
                    score=score,
                    landmarks_px=pts,
                    hand_bbox=(x1, y1, x2, y2),
                    finger_states=finger_states,
                    extended_count=ext_count,
                    palm_scale_px=palm_scale,
                )
            )

        return observations

    @staticmethod
    def pick_primary_observation(
        observations: List[HandGestureObservation],
    ) -> Optional[HandGestureObservation]:
        """ROI 내 여러 손 중 FIST/OPEN_PALM 판정이 명확하고 크기가 큰 손 1개를 선택합니다."""
        if not observations:
            return None

        def _rank(obs: HandGestureObservation) -> Tuple[int, float]:
            decisive = 1 if obs.gesture in (GESTURE_FIST, GESTURE_OPEN_PALM) else 0
            return (decisive, obs.palm_scale_px * max(0.2, obs.score))

        return max(observations, key=_rank)

    def close(self):
        if self._landmarker is not None:
            self._landmarker.close()
            self._landmarker = None


class FistOpenStateMachine:
    """
    단일 대상의 '주먹(FIST_HOLD) -> 손바닥 펴기(OPEN_HOLD)' 다중 프레임 제스처 상태머신.
    중간 1~2프레임 흔들림(UNKNOWN/미검출)은 grace_frames 동안 유지해 오작동을 줄입니다.
    """

    def __init__(
        self,
        subject_id: int,
        fist_hold_frames: int = 4,
        open_hold_frames: int = 4,
        grace_frames: int = 3,
        armed_timeout_sec: float = 2.5,
        cooldown_sec: float = 1.5,
    ):
        self.subject_id = subject_id
        self.fist_hold_frames = max(1, int(fist_hold_frames))
        self.open_hold_frames = max(1, int(open_hold_frames))
        self.grace_frames = max(0, int(grace_frames))
        self.armed_timeout_sec = max(0.5, float(armed_timeout_sec))
        self.cooldown_sec = max(0.2, float(cooldown_sec))

        self.state = STATE_IDLE
        self.fist_count = 0
        self.open_count = 0
        self.miss_count = 0
        self.armed_time = 0.0
        self.last_trigger_time = 0.0
        self.last_seen_time = time.time()

    def configure(self, fist_hold_frames: int, open_hold_frames: int):
        self.fist_hold_frames = max(1, int(fist_hold_frames))
        self.open_hold_frames = max(1, int(open_hold_frames))

    def reset(self):
        self.state = STATE_IDLE
        self.fist_count = 0
        self.open_count = 0
        self.miss_count = 0
        self.armed_time = 0.0

    def update(
        self,
        obs: Optional[HandGestureObservation],
        now: Optional[float] = None,
    ) -> GestureProgress:
        t = time.time() if now is None else now
        gesture = obs.gesture if obs is not None else GESTURE_IDLE
        if obs is not None:
            self.last_seen_time = t

        just_triggered = False

        if self.state == STATE_TRIGGERED:
            if (t - self.last_trigger_time) >= self.cooldown_sec:
                self.reset()
            return self._snapshot(gesture, just_triggered=False, obs=obs)

        if self.state == STATE_IDLE:
            if gesture == GESTURE_FIST:
                self.fist_count += 1
                self.miss_count = 0
                if self.fist_count >= self.fist_hold_frames:
                    self.state = STATE_FIST_ARMED
                    self.armed_time = t
                    self.open_count = 0
                    self.miss_count = 0
            else:
                self.miss_count += 1
                if self.miss_count > self.grace_frames:
                    self.fist_count = 0
                    self.miss_count = 0

        elif self.state == STATE_FIST_ARMED:
            if (t - self.armed_time) > self.armed_timeout_sec:
                self.reset()
            elif gesture == GESTURE_OPEN_PALM:
                self.open_count += 1
                self.miss_count = 0
                if self.open_count >= self.open_hold_frames:
                    self.state = STATE_TRIGGERED
                    self.last_trigger_time = t
                    just_triggered = True
            elif gesture == GESTURE_FIST:
                self.armed_time = t
                self.open_count = 0
                self.miss_count = 0
            else:
                self.miss_count += 1
                if self.miss_count > self.grace_frames:
                    self.open_count = max(0, self.open_count - 1)

        return self._snapshot(gesture, just_triggered=just_triggered, obs=obs)

    def _snapshot(
        self,
        last_gesture: str,
        just_triggered: bool,
        obs: Optional[HandGestureObservation],
    ) -> GestureProgress:
        if self.state == STATE_TRIGGERED:
            progress = 1.0
        elif self.state == STATE_FIST_ARMED:
            progress = 0.5 + 0.5 * min(1.0, self.open_count / float(self.open_hold_frames))
        else:
            progress = 0.5 * min(1.0, self.fist_count / float(self.fist_hold_frames))

        return GestureProgress(
            subject_id=self.subject_id,
            state=self.state,
            last_gesture=last_gesture,
            fist_count=self.fist_count,
            fist_target=self.fist_hold_frames,
            open_count=self.open_count,
            open_target=self.open_hold_frames,
            progress=progress,
            just_triggered=just_triggered,
            observation=obs,
        )


class MultiTargetGestureManager:
    """사람 ID별로 독립적인 FistOpenStateMachine을 관리하고 오래 안 보인 대상을 정리합니다."""

    def __init__(
        self,
        fist_hold_frames: int = 4,
        open_hold_frames: int = 4,
        grace_frames: int = 3,
        armed_timeout_sec: float = 2.5,
        cooldown_sec: float = 1.5,
        stale_sec: float = 4.0,
    ):
        self.fist_hold_frames = fist_hold_frames
        self.open_hold_frames = open_hold_frames
        self.grace_frames = grace_frames
        self.armed_timeout_sec = armed_timeout_sec
        self.cooldown_sec = cooldown_sec
        self.stale_sec = stale_sec
        self._machines: Dict[int, FistOpenStateMachine] = {}

    def set_thresholds(self, fist_hold_frames: int, open_hold_frames: int):
        self.fist_hold_frames = max(1, int(fist_hold_frames))
        self.open_hold_frames = max(1, int(open_hold_frames))
        for sm in self._machines.values():
            sm.configure(self.fist_hold_frames, self.open_hold_frames)

    def reset_all(self):
        self._machines.clear()

    def update(
        self,
        subject_id: int,
        obs: Optional[HandGestureObservation],
        now: Optional[float] = None,
    ) -> GestureProgress:
        sm = self._machines.get(subject_id)
        if sm is None:
            sm = FistOpenStateMachine(
                subject_id=subject_id,
                fist_hold_frames=self.fist_hold_frames,
                open_hold_frames=self.open_hold_frames,
                grace_frames=self.grace_frames,
                armed_timeout_sec=self.armed_timeout_sec,
                cooldown_sec=self.cooldown_sec,
            )
            self._machines[subject_id] = sm
        return sm.update(obs, now=now)

    def prune_stale(self, active_ids: List[int], now: Optional[float] = None):
        t = time.time() if now is None else now
        active_set = set(active_ids)
        stale_keys = [
            sid for sid, sm in self._machines.items()
            if sid not in active_set and (t - sm.last_seen_time) > self.stale_sec
        ]
        for sid in stale_keys:
            del self._machines[sid]


def build_person_gesture_roi(
    bbox: Tuple[int, int, int, int],
    keypoints: Optional[np.ndarray],
    frame_shape: Tuple[int, ...],
) -> Tuple[int, int, int, int]:
    """
    사람 바운딩 박스와 Pose 키포인트로 폴백용 상반신 정사각/여유 ROI를 계산합니다.
    얼굴 키포인트(0~4)가 확인되면 머리 상단을 제외해 얼굴 오검출을 줄입니다.
    """
    h, w = frame_shape[:2]
    x1, y1, x2, y2 = bbox
    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)

    pad_x = int(bw * 0.22)
    rx1 = x1 - pad_x
    rx2 = x2 + pad_x
    ry1 = y1 + int(bh * 0.14)
    ry2 = y1 + int(bh * 0.78)

    # 얼굴(코 0, 눈 1·2)이 보이면 ROI 상단을 코 아래쪽 부근으로 내려 머리/얼굴 포함 최소화
    if keypoints is not None and len(keypoints) >= 5:
        face_ys = [float(keypoints[i, 1]) for i in (0, 1, 2) if float(keypoints[i, 2]) >= 0.30]
        if face_ys:
            ry1 = max(ry1, int(max(face_ys) + bh * 0.03))

    if keypoints is not None and len(keypoints) >= 11:
        wrist_margin = max(40, int(bw * 0.38))
        for kp_idx in (7, 8, 9, 10):
            if float(keypoints[kp_idx, 2]) >= 0.25:
                kx, ky = int(keypoints[kp_idx, 0]), int(keypoints[kp_idx, 1])
                rx1 = min(rx1, kx - wrist_margin)
                rx2 = max(rx2, kx + wrist_margin)
                ry1 = min(ry1, ky - wrist_margin)
                ry2 = max(ry2, ky + wrist_margin)

    rx1 = max(0, rx1)
    ry1 = max(0, ry1)
    rx2 = min(w, rx2)
    ry2 = min(h, ry2)
    if ry2 <= ry1 + 24:
        ry1 = max(0, y1)
    return (rx1, ry1, rx2, ry2)


def _clamp_square_roi(
    cx: float,
    cy: float,
    half_size: int,
    frame_shape: Tuple[int, ...],
) -> Optional[Tuple[int, int, int, int]]:
    h, w = frame_shape[:2]
    half = max(36, int(half_size))
    x1 = max(0, int(round(cx - half)))
    y1 = max(0, int(round(cy - half)))
    x2 = min(w, int(round(cx + half)))
    y2 = min(h, int(round(cy + half)))
    if (x2 - x1) < 32 or (y2 - y1) < 32:
        return None
    return (x1, y1, x2, y2)


def build_person_hand_rois(
    bbox: Tuple[int, int, int, int],
    keypoints: Optional[np.ndarray],
    frame_shape: Tuple[int, ...],
) -> List[Tuple[Tuple[int, int, int, int], str]]:
    """
    YOLO-Pose 팔꿈치(7,8)->손목(9,10) 연장선 기반의 정사각 손 ROI 목록을 우선 반환합니다.
    손목 키포인트가 없을 때만 상반신 폴백 ROI를 반환합니다.
    반환: [((x1, y1, x2, y2), 'wrist'|'upper_fallback'), ...]
    """
    x1, _, x2, _ = bbox
    bw = max(1, x2 - x1)
    hand_rois: List[Tuple[Tuple[int, int, int, int], str]] = []

    if keypoints is not None and len(keypoints) >= 11:
        # 어깨 폭이 보이면 손 ROI 크기의 기준으로 사용 (대략 어깨 폭의 0.55~0.65배 반폭)
        if float(keypoints[5, 2]) >= 0.30 and float(keypoints[6, 2]) >= 0.30:
            shoulder_w = _dist(keypoints[5, :2], keypoints[6, :2])
            half_sz = max(44, int(shoulder_w * 0.62))
        else:
            half_sz = max(44, int(bw * 0.42))

        for elbow_i, wrist_i in ((7, 9), (8, 10)):
            if float(keypoints[wrist_i, 2]) < 0.25:
                continue
            wx, wy = float(keypoints[wrist_i, 0]), float(keypoints[wrist_i, 1])
            cx, cy = wx, wy
            if float(keypoints[elbow_i, 2]) >= 0.25:
                ex, ey = float(keypoints[elbow_i, 0]), float(keypoints[elbow_i, 1])
                # 팔꿈치 -> 손목 방향 25% 앞쪽이 손바닥 중심에 가까움
                cx = wx + 0.25 * (wx - ex)
                cy = wy + 0.25 * (wy - ey)
            sq = _clamp_square_roi(cx, cy, half_sz, frame_shape)
            if sq is not None:
                hand_rois.append((sq, 'wrist'))

    if hand_rois:
        return hand_rois
    return [(build_person_gesture_roi(bbox, keypoints, frame_shape), 'upper_fallback')]


def is_false_face_detection(
    obs: HandGestureObservation,
    bbox: Tuple[int, int, int, int],
    keypoints: Optional[np.ndarray],
) -> bool:
    """
    검출된 손이 Pose 얼굴 영역(코·눈·귀)과 겹치면서 근처에 실제 손목(9,10)이 없으면
    얼굴 오검출로 판정해 True를 반환합니다.
    """
    hx1, hy1, hx2, hy2 = obs.hand_bbox
    hcx = 0.5 * (hx1 + hx2)
    hcy = 0.5 * (hy1 + hy2)

    x1, y1, x2, y2 = bbox
    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)

    in_face_zone = False
    face_radius = max(24.0, bw * 0.30)
    if keypoints is not None and len(keypoints) >= 5:
        face_pts = [keypoints[i, :2] for i in (0, 1, 2, 3, 4) if float(keypoints[i, 2]) >= 0.30]
        if face_pts:
            fc = np.mean(np.array(face_pts, dtype=np.float32), axis=0)
            if math.hypot(hcx - float(fc[0]), hcy - float(fc[1])) <= face_radius:
                in_face_zone = True
    elif (x1 + bw * 0.15) <= hcx <= (x2 - bw * 0.15) and y1 <= hcy <= (y1 + bh * 0.26):
        in_face_zone = True

    if not in_face_zone:
        return False

    # 얼굴 바로 옆으로 실제로 손을 들어 올린 경우(손목 키포인트가 근접)는 통과
    if keypoints is not None and len(keypoints) >= 11:
        hand_wrist_px = obs.landmarks_px[0]
        wrist_tol = max(45.0, obs.palm_scale_px * 1.6, bw * 0.40)
        for wrist_i in (9, 10):
            if float(keypoints[wrist_i, 2]) >= 0.25:
                if _dist(hand_wrist_px, keypoints[wrist_i, :2]) <= wrist_tol:
                    return False
    return True


def match_hand_to_person_wrist(
    obs: HandGestureObservation,
    bbox: Tuple[int, int, int, int],
    keypoints: Optional[np.ndarray],
    min_palm_scale_px: float = 12.0,
    min_person_width_px: int = 44,
    require_pose_wrist: bool = False,
) -> Optional[float]:
    """
    검출된 손(MediaPipe 손목 landmark[0])과 사람의 YOLO-Pose 손목(9, 10)을 거리 매칭합니다.
    - 너무 멀리 있는 사람/작은 손(노이즈·원거리 얼굴 조각)을 크기 하한으로 제외합니다.
    - Pose 손목이 보이면 반드시 허용 반경 이내여야 통과시키며, 매칭 거리(px)를 반환합니다.
    - 조건 불충족 시 None을 반환합니다.
    """
    x1, y1, x2, y2 = bbox
    bw = max(1, x2 - x1)
    bh = max(1, y2 - y1)

    if bw < min_person_width_px:
        return None
    if obs.palm_scale_px < min_palm_scale_px or obs.palm_scale_px > bw * 0.85:
        return None

    if is_false_face_detection(obs, bbox, keypoints):
        return None

    visible_wrists = []
    if keypoints is not None and len(keypoints) >= 11:
        for wrist_i in (9, 10):
            if float(keypoints[wrist_i, 2]) >= 0.25:
                visible_wrists.append(keypoints[wrist_i, :2])

    if visible_wrists:
        if float(keypoints[5, 2]) >= 0.30 and float(keypoints[6, 2]) >= 0.30:
            shoulder_w = _dist(keypoints[5, :2], keypoints[6, :2])
            wrist_tol = max(32.0, shoulder_w * 0.72, obs.palm_scale_px * 2.2)
        else:
            wrist_tol = max(32.0, bw * 0.45, obs.palm_scale_px * 2.2)

        hand_wrist_px = obs.landmarks_px[0]
        min_d = min(_dist(hand_wrist_px, w_pt) for w_pt in visible_wrists)
        if min_d <= wrist_tol:
            return float(min_d)
        return None

    if require_pose_wrist:
        return None

    hx1, hy1, hx2, hy2 = obs.hand_bbox
    hcx = 0.5 * (hx1 + hx2)
    hcy = 0.5 * (hy1 + hy2)
    if (x1 - bw * 0.25) <= hcx <= (x2 + bw * 0.25) and y1 <= hcy <= (y1 + bh * 0.85):
        # 손목 키포인트 기반 매칭보다 항상 후순위가 되도록 페널티 거리 부여
        return float(max(60.0, bw * 0.85))
    return None


def find_best_hand_owner(
    obs: HandGestureObservation,
    persons: List[Tuple[int, Tuple[int, int, int, int], Optional[np.ndarray]]],
    min_palm_scale_px: float = 12.0,
    min_person_width_px: int = 44,
    require_pose_wrist: bool = False,
) -> Optional[Tuple[int, float]]:
    """
    검출된 손 1개에 대해 전체 사람 목록 중 손목 거리가 가장 가까운 주인(track_id, dist_px)을 찾습니다.
    """
    best_id: Optional[int] = None
    best_dist = float('inf')
    for track_id, bbox, keypoints in persons:
        d = match_hand_to_person_wrist(
            obs,
            bbox,
            keypoints,
            min_palm_scale_px=min_palm_scale_px,
            min_person_width_px=min_person_width_px,
            require_pose_wrist=require_pose_wrist,
        )
        if d is not None and d < best_dist:
            best_dist = d
            best_id = track_id
    if best_id is None:
        return None
    return best_id, best_dist
