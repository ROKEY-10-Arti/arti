"""
다중 YOLO 모델 + ByteTrack/BoT-SORT + Adaptive ReID 오인식 방지 추적 코어 (multi_tracker_core.py)

[전체 프레임 처리 흐름 (`MultiModelTrackerCore.process_frame`)]
1. 1차 다중 객체 탐지·추적 (`self.model.track`):
   - 선택된 YOLO 버전(v8n/11n/26n)과 태스크(Det/Seg/Pose)에 ByteTrack 또는 BoT-SORT를 결합해
     프레임 내 모든 사람의 `track_id`, `bbox`, `keypoints`/`mask_poly`를 획득합니다.
2. 외형 특징 추출 및 유사도 산출 (`current_extractor.extract` -> `compute_dual_similarity_detail`):
   - 타겟 `anchor_bank`가 고정된 상태에서 REID 활성 모드이거나 시각화 요청(`compute_all_sims=True`) 시
     모든 검출 인원의 304D HSV `PartFeatureBank`와 Anchor/Recent 최고 유사도(`similarity`, `part_sims`)를 계산합니다.
3. 모델/트래커 전환 시 타겟 승계 (`pending_tracker_rebind`):
   - 모델/모드 전환 직후 1프레임에 한해 직전 BBox와의 IoU(0.65) + 외형 유사도(0.35)로 동일 인물을 재연결합니다.
4. MODE_ADAPTIVE (모드 5) 상태 머신 — 오인식 방지 및 타겟 복구:
   - [COEX 기록]: 현재 타겟과 충분히 떨어져(`abs(dx) > 0.9 * tw`, `IoU < 0.05`) 공존 중인 다른 ID를
     `coexisting_non_target_ids`에 등록해 복귀 시 동일인 오인을 막습니다.
   - [SAFE 상태]: 타겟 주변이 한산할 때 외형을 검사하며, 다수 인원(`len >= 2`) 상황에서
     유사도가 `id_switch_thresh`(0.48) 미만인 프레임이 `safe_mismatch_limit`(4회) 연속 발생하면
     ByteTrack ID 탈취로 간주하고 `target_id = -1`로 조기 이탈해 `SEARCH`로 전환합니다.
   - [GUARD 상태]: 타겟 주변이 혼잡(`_is_crowded_near_target`)하면 타인 박스를 마스킹(`exclude_boxes`)한 뒤
     외형(0.60) + 3D LiDAR 공간 점수(0.40) + 관성 보너스(+0.04)로 즉시 교차 ID를 재판별합니다.
   - [SEARCH -> RECOVERED 상태]: 타겟 유실 시 전체 검출 후보를 점수화(`_score_candidate`)한 뒤,
     `COEX` 차단(`>= coexist_min_sim`일 때만 `eligible`), 최소 유사도/점수, 1-2위 격차(`recover_margin`),
     연속 확인(`recover_confirm_frames=2`)을 모두 통과한 후보만 타겟으로 복귀시킵니다.
5. 결과 조립 및 EMA 성능 통계 갱신 (`UnifiedTrackedPerson`, `MultiTrackerMetrics`).
"""

from dataclasses import dataclass
import math
import os
import tempfile
import time
from typing import Callable, Dict, List, Optional, Tuple
import numpy as np
import torch
from ultralytics import YOLO

from person_tracker.core.env_bootstrap import resolve_model_path
from person_tracker.core.feature_extractors import (
    BaseFeatureExtractor,
    BBoxFeatureExtractor,
    PartFeatureBank,
    PoseKeypointFeatureExtractor,
    SegMaskFeatureExtractor,
)


@dataclass
class UnifiedTrackedPerson:
    track_id: int
    bbox: Tuple[int, int, int, int]
    center: Tuple[int, int]
    bottom_center: Tuple[int, int]
    norm_offset: Tuple[float, float]
    area_ratio: float
    is_target: bool
    similarity: float = 1.0
    mask_poly: Optional[np.ndarray] = None
    keypoints: Optional[np.ndarray] = None  # (17, 3) [x, y, conf]
    visible_parts: Tuple[bool, bool, bool] = (True, True, True)
    part_sims: Tuple[Optional[float], Optional[float], Optional[float], Optional[float]] = (
        None,
        None,
        None,
        None,
    )
    feature_bank: Optional[PartFeatureBank] = None


@dataclass
class MultiTrackerMetrics:
    model_label: str = 'yolov8n-pose.pt'
    task: str = 'pose'
    device: str = 'CPU'
    preprocess_ms: float = 0.0
    inference_ms: float = 0.0
    post_track_ms: float = 0.0
    reid_ms: float = 0.0
    total_ms: float = 0.0
    max_infer_fps: float = 0.0
    adaptive_state: str = 'IDLE'
    search_reason: str = ''
    ema_status: str = 'IDLE'


class MultiModelTrackerCore:
    """
    3종 YOLO 버전(YOLOv8n / YOLO11n / YOLO26n) x 3종 태스크(Detect / Segment / Pose) = 9개 모델과
    5가지 추적 알고리즘을 자유롭게 실시간 교차 조합할 수 있는 모듈형 코어
    """

    FAMILIES = {
        1: ('v8n', 'yolov8n'),
        2: ('11n', 'yolo11n'),
        3: ('26n', 'yolo26n'),
    }

    TASKS = {
        1: ('Det', '', 'detect'),
        2: ('Seg', '-seg', 'segment'),
        3: ('Pose', '-pose', 'pose'),
    }

    MODE_BYTETRACK = 1
    MODE_TASK_REID = 2
    MODE_BOTSORT = 3
    MODE_BOT_REID = 4
    MODE_ADAPTIVE = 5

    MODE_NAMES = {
        1: '1:ByteTrack',
        2: '2:Task-ReID',
        3: '3:BoT-SORT',
        4: '4:BoT+ReID',
        5: '5:Adaptive',
    }

    BOTSORT_MODES = {MODE_BOTSORT, MODE_BOT_REID}
    REID_ACTIVE_MODES = {MODE_TASK_REID, MODE_BOT_REID}

    def __init__(
        self,
        family_idx: int = 1,   # 기본: 1 (YOLOv8n, 8-GT 종합 1위)
        task_idx: int = 3,     # 기본: 3 (Pose)
        mode: int = 5,         # 기본: 5 (Adaptive)
        imgsz: int = 320,
        device: str = '',
        conf: float = 0.15,
        iou: float = 0.55,
        reid_match_thresh: float = 0.50,
        search_min_sim: float = 0.48,
        coexist_min_sim: float = 0.74,
        recover_margin: float = 0.06,
        recover_confirm_frames: int = 2,
        id_switch_thresh: float = 0.48,
        safe_mismatch_limit: int = 4,
        ema_min_sim: float = 0.60,
        ema_alpha: float = 0.04,
        ema_require_single: bool = True,
        pose_mode: str = 'edges',
        edge_score_mode: Optional[str] = None,
        seg_mode: str = 'baseline',
    ):
        self.family_idx = family_idx if family_idx in self.FAMILIES else 1
        self.task_idx = task_idx if task_idx in self.TASKS else 3
        self.mode = mode if mode in self.MODE_NAMES else 5
        self.imgsz = imgsz
        self.conf = float(conf)
        self.iou = float(iou)

        if not device:
            self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        else:
            self.device = device

        # 태스크별 특징 추출기 플러그인 매핑
        self.extractors: Dict[str, BaseFeatureExtractor] = {
            'detect': BBoxFeatureExtractor(),
            'segment': SegMaskFeatureExtractor(seg_mode=seg_mode),
            'pose': PoseKeypointFeatureExtractor(
                pose_mode=pose_mode, edge_score_mode=edge_score_mode
            ),
        }

        # 모델 캐시 (한 번 로드한 모델은 메모리에 유지하여 즉시 전환)
        self.model_cache: Dict[str, YOLO] = {}
        self.botsort_cfg_path = self._create_botsort_reid_config()

        self.target_id: Optional[int] = None
        self.anchor_bank: Optional[PartFeatureBank] = None
        self.recent_bank: Optional[PartFeatureBank] = None
        self.last_target_bbox: Optional[Tuple[int, int, int, int]] = None
        self.pending_tracker_rebind: bool = False
        self.last_detections: List[UnifiedTrackedPerson] = []
        self.last_frame: Optional[np.ndarray] = None

        # 오인식 예방 상태 (SAFE 이탈 검사 + COEX 공존 비타겟 + SEARCH 연속 확정)
        self.safe_mismatch_count: int = 0
        self.safe_mismatch_limit: int = max(1, int(safe_mismatch_limit))
        self.coexisting_non_target_ids: set[int] = set()
        self.recover_candidate_id: Optional[int] = None
        self.recover_confirm_count: int = 0
        self.recover_confirm_frames: int = max(1, int(recover_confirm_frames))
        self.last_search_reason: str = ''

        # 라이다/맵 3D 위치 기반 타겟 기억 상태 (다른 사람과 혼동 방지용)
        self.last_target_world_xy: Optional[Tuple[float, float]] = None
        self.last_target_world_frame: str = ''
        self.last_target_seen_time: float = 0.0

        self.reid_match_thresh = float(reid_match_thresh)
        self.search_min_sim = float(search_min_sim)
        self.coexist_min_sim = float(coexist_min_sim)
        self.recover_margin = float(recover_margin)
        self.id_switch_thresh = float(id_switch_thresh)
        self.ema_min_sim = float(ema_min_sim)
        self.ema_alpha = float(ema_alpha)
        self.ema_require_single = bool(ema_require_single)
        self.backfill_confirm_frames: int = 2
        self._fallback_upgrade_streak: int = 0
        self._backfill_streaks: Dict[str, int] = {'upper': 0, 'mid': 0, 'lower': 0}

        self.current_model_name = self._build_model_filename(self.family_idx, self.task_idx)
        self.model = self._get_or_load_model(self.current_model_name)
        self.metrics = MultiTrackerMetrics(
            model_label=self.current_model_name,
            task=self.current_task_name,
            device=self.device.upper(),
        )

    @classmethod
    def _build_model_filename(cls, family_idx: int, task_idx: int) -> str:
        _, prefix = cls.FAMILIES[family_idx]
        _, suffix, _ = cls.TASKS[task_idx]
        return f'{prefix}{suffix}.pt'

    @property
    def current_task_name(self) -> str:
        return self.TASKS[self.task_idx][2]

    @property
    def current_extractor(self) -> BaseFeatureExtractor:
        return self.extractors[self.current_task_name]

    def _get_or_load_model(self, model_filename: str) -> YOLO:
        if model_filename not in self.model_cache:
            resolved = resolve_model_path(model_filename)
            self.model_cache[model_filename] = YOLO(resolved)
        return self.model_cache[model_filename]

    def _create_botsort_reid_config(self) -> str:
        cfg_content = (
            "tracker_type: botsort\n"
            "track_high_thresh: 0.25\n"
            "track_low_thresh: 0.1\n"
            "new_track_thresh: 0.25\n"
            "track_buffer: 90\n"
            "match_thresh: 0.8\n"
            "fuse_score: True\n"
            "gmc_method: sparseOptFlow\n"
            "proximity_thresh: 0.5\n"
            "appearance_thresh: 0.65\n"
            "with_reid: True\n"
            "model: auto\n"
        )
        fd, path = tempfile.mkstemp(prefix='botsort_multi_', suffix='.yaml')
        with os.fdopen(fd, 'w') as f:
            f.write(cfg_content)
        return path

    def set_model(self, family_idx: Optional[int] = None, task_idx: Optional[int] = None) -> str:
        """YOLO 버전(v8n/11n/26n) 또는 태스크(Det/Seg/Pose)를 실시간 교체하고 타겟을 승계"""
        new_f = family_idx if family_idx in self.FAMILIES else self.family_idx
        new_t = task_idx if task_idx in self.TASKS else self.task_idx
        if new_f == self.family_idx and new_t == self.task_idx:
            return self.current_model_name

        self.family_idx = new_f
        self.task_idx = new_t
        self.current_model_name = self._build_model_filename(self.family_idx, self.task_idx)
        self.model = self._get_or_load_model(self.current_model_name)
        self.model.predictor = None
        if self.target_id is not None:
            self.pending_tracker_rebind = True
        return self.current_model_name

    def set_mode(self, new_mode: int):
        if new_mode not in self.MODE_NAMES or new_mode == self.mode:
            return
        prev_is_botsort = (self.mode in self.BOTSORT_MODES)
        new_is_botsort = (new_mode in self.BOTSORT_MODES)
        self.mode = new_mode

        if prev_is_botsort != new_is_botsort:
            self.model.predictor = None
            if self.target_id is not None:
                self.pending_tracker_rebind = True

    def toggle_device(self) -> str:
        if torch.cuda.is_available():
            self.device = 'cpu' if 'cuda' in self.device else 'cuda:0'
            self.model.predictor = None
            if self.target_id is not None:
                self.pending_tracker_rebind = True
        else:
            self.device = 'cpu'
        return self.device

    @staticmethod
    def _bbox_iou(boxA: Tuple[int, int, int, int], boxB: Tuple[int, int, int, int]) -> float:
        xA = max(boxA[0], boxB[0])
        yA = max(boxA[1], boxB[1])
        xB = min(boxA[2], boxB[2])
        yB = min(boxA[3], boxB[3])
        inter = max(0, xB - xA) * max(0, yB - yA)
        if inter <= 0:
            return 0.0
        areaA = max(1, (boxA[2] - boxA[0]) * (boxA[3] - boxA[1]))
        areaB = max(1, (boxB[2] - boxB[0]) * (boxB[3] - boxB[1]))
        return float(inter) / float(areaA + areaB - inter)

    @classmethod
    def _is_crowded_near_target(
        cls, target: UnifiedTrackedPerson, detections: List[UnifiedTrackedPerson]
    ) -> bool:
        """타겟 주변 혼잡 여부 판별: 타인과 IoU > 0.02 또는 중심 거리가 BBox 폭 1.2배/높이 0.8배 이내면 True."""
        tx1, ty1, tx2, ty2 = target.bbox
        tw = max(1, tx2 - tx1)
        tcx, tcy = target.center
        for other in detections:
            if other.track_id == target.track_id:
                continue
            if cls._bbox_iou(target.bbox, other.bbox) > 0.02:
                return True
            ocx, ocy = other.center
            if abs(tcx - ocx) < tw * 1.2 and abs(tcy - ocy) < (ty2 - ty1) * 0.8:
                return True
        return False

    def _compute_spatial_score(
        self,
        person: UnifiedTrackedPerson,
        frame_w: int,
        frame_h: int,
        spatial_estimator: Optional[
            Callable[[UnifiedTrackedPerson, int, int], Optional[Tuple[Tuple[float, float], str, str]]]
        ],
    ) -> Tuple[float, Optional[float]]:
        """
        직전 확인된 타겟의 3D(map 또는 base_link) 위치와 경과 시간을 바탕으로
        후보가 물리적 이동 반경 안에 있는지 [0.0 ~ 1.0] 공간 점수와 거리(m)를 반환.

        [알고리즘 원리]
        - 보행 속도(1.35 m/s)와 센서 오차 여유(0.65 m)를 합산해 허용 반경
          `allowed_radius = 0.65 + 1.35 * min(dt, 6.0)`을 구합니다.
        - 3D 유클리드 거리 `dist_m`를 가우시안 감쇠 `exp(-0.5 * (dist_m / sigma)^2)`로 변환하며,
          LiDAR 실측 거리가 허용 반경의 1.4배를 초과하면 순간이동 불가능으로 보고 0.20배 패널티를 줍니다.
        """
        if (
            spatial_estimator is None
            or self.last_target_world_xy is None
            or not self.last_target_world_frame
        ):
            return 1.0, None

        dt = max(0.0, time.time() - self.last_target_seen_time)
        if dt > 8.0:
            return 1.0, None

        est = spatial_estimator(person, frame_w, frame_h)
        if est is None:
            return 1.0, None

        cand_xy, cand_frame, source = est
        if cand_frame != self.last_target_world_frame:
            return 1.0, None

        dist_m = math.hypot(
            cand_xy[0] - self.last_target_world_xy[0],
            cand_xy[1] - self.last_target_world_xy[1],
        )
        # 사람 보행 속도(~1.35m/s) + 센서 측정 여유(0.65m)
        allowed_radius = 0.65 + 1.35 * min(dt, 6.0)
        sigma = max(0.55, 0.55 * allowed_radius)
        score = math.exp(-0.5 * (dist_m / sigma) ** 2)

        # 라이다 실측 거리가 물리적 이동 가능 반경을 크게 벗어나면 강한 패널티
        if source == 'LiDAR' and dist_m > allowed_radius * 1.4:
            score *= 0.20

        return float(score), dist_m

    def _score_candidate(
        self,
        person: UnifiedTrackedPerson,
        frame_w: int,
        frame_h: int,
        spatial_estimator,
    ) -> float:
        """SEARCH 복귀 후보 종합 점수: 3D 위치 정보가 있으면 `0.65 * 외형유사도 + 0.35 * 공간점수`, 없으면 외형유사도 단독 사용."""
        sp_score, _ = self._compute_spatial_score(person, frame_w, frame_h, spatial_estimator)
        if self.last_target_world_xy is None or spatial_estimator is None:
            return person.similarity
        return 0.65 * person.similarity + 0.35 * sp_score

    def _reset_bank_completion_streaks(self):
        self._fallback_upgrade_streak = 0
        for part in ('upper', 'mid', 'lower'):
            self._backfill_streaks[part] = 0

    def _update_target_banks_safely(
        self,
        extractor: BaseFeatureExtractor,
        current_target: UnifiedTrackedPerson,
        t_bank: Optional[PartFeatureBank],
        detections: List[UnifiedTrackedPerson],
    ) -> str:
        """
        안전 추적 구간에서 `recent_bank` EMA 갱신, BBox 폴백 승격(`UPGRADE`),
        초기 미관측(`None`) 부위 보충(`BACKFILL`)을 수행하고 `ema_status` 문자열을 반환합니다.
        """
        single_ok = (not self.ema_require_single) or (len(detections) == 1)
        if not single_ok:
            self._reset_bank_completion_streaks()
            return f'HOLD({len(detections)}P)'

        # 1. 등록 시 Pose 관절 미검출로 BBox 폴백 잠금된 경우 -> 정상 Pose 관절 2프레임 연속 검출 시 1회 승격
        if (
            self.anchor_bank is not None
            and self.anchor_bank.is_fallback
            and t_bank is not None
            and not t_bank.is_fallback
        ):
            self._fallback_upgrade_streak += 1
            if self._fallback_upgrade_streak >= self.backfill_confirm_frames:
                self.anchor_bank = t_bank.copy()
                self.recent_bank = t_bank.copy()
                self._reset_bank_completion_streaks()
                current_target.similarity, current_target.part_sims = (
                    extractor.compute_dual_similarity_detail(
                        self.anchor_bank, self.recent_bank, t_bank
                    )
                )
                return 'UPGRADE'
            return f'HOLD(UPG {self._fallback_upgrade_streak}/{self.backfill_confirm_frames})'

        self._fallback_upgrade_streak = 0
        if current_target.similarity < self.ema_min_sim:
            for part in ('upper', 'mid', 'lower'):
                self._backfill_streaks[part] = 0
            return 'HOLD(sim)'

        extractor.update_recent_bank(self.recent_bank, t_bank, alpha=self.ema_alpha)

        # 2. 등록 시 가려짐/화면 잘림으로 None이었던 부위가 안전 구간에서 연속 관측되면 1회 보충
        backfilled = False
        if t_bank is not None and not t_bank.is_fallback:
            for part in ('upper', 'mid', 'lower'):
                need_recent = self.recent_bank is not None and getattr(self.recent_bank, part) is None
                need_anchor = self.anchor_bank is not None and getattr(self.anchor_bank, part) is None
                cand_hist = getattr(t_bank, part)
                if (need_recent or need_anchor) and cand_hist is not None:
                    self._backfill_streaks[part] += 1
                    if self._backfill_streaks[part] >= self.backfill_confirm_frames:
                        if extractor.backfill_part(self.recent_bank, part, cand_hist):
                            backfilled = True
                        if extractor.backfill_part(self.anchor_bank, part, cand_hist):
                            backfilled = True
                        self._backfill_streaks[part] = 0
                else:
                    self._backfill_streaks[part] = 0
            if t_bank.edges:
                if extractor.backfill_edges(self.recent_bank, t_bank.edges):
                    backfilled = True
                if extractor.backfill_edges(self.anchor_bank, t_bank.edges):
                    backfilled = True
            if backfilled:
                current_target.similarity, current_target.part_sims = (
                    extractor.compute_dual_similarity_detail(
                        self.anchor_bank, self.recent_bank, t_bank
                    )
                )
        else:
            for part in ('upper', 'mid', 'lower'):
                self._backfill_streaks[part] = 0

        return 'BACKFILL' if backfilled else 'ACTIVE'

    def _lock_target_bank(self, frame: np.ndarray, person: UnifiedTrackedPerson):
        bank = self.current_extractor.extract(
            frame, person.bbox, person.mask_poly, person.keypoints
        )
        if bank is not None:
            self.anchor_bank = bank.copy()
            self.recent_bank = bank.copy()
            person.visible_parts = bank.visible_parts

    def _clear_target(self):
        self.target_id = None
        self.anchor_bank = None
        self.recent_bank = None
        self.last_target_bbox = None
        self.pending_tracker_rebind = False
        self.safe_mismatch_count = 0
        self.coexisting_non_target_ids.clear()
        self.recover_candidate_id = None
        self.recover_confirm_count = 0
        self.last_search_reason = ''
        self.last_target_world_xy = None
        self.last_target_world_frame = ''
        self.last_target_seen_time = 0.0
        self._reset_bank_completion_streaks()

    def _activate_target(self, person: UnifiedTrackedPerson) -> int:
        """새 타겟 ID와 Anchor 특징 뱅크를 잠그고 SAFE/COEX/SEARCH 카운터를 초기 상태로 리셋합니다."""
        self.target_id = person.track_id
        self.last_target_bbox = person.bbox
        self.pending_tracker_rebind = False
        self.safe_mismatch_count = 0
        self.coexisting_non_target_ids.clear()
        self.recover_candidate_id = None
        self.recover_confirm_count = 0
        self.last_search_reason = ''
        self.last_target_world_xy = None
        self.last_target_world_frame = ''
        self.last_target_seen_time = 0.0
        self._reset_bank_completion_streaks()
        if self.last_frame is not None:
            self._lock_target_bank(self.last_frame, person)
        return self.target_id

    def select_target_by_id(self, track_id: Optional[int]) -> Optional[int]:
        """Select and lock a tracked person by track_id, or clear target when None."""
        if track_id is None:
            self._clear_target()
            return None
        for person in self.last_detections:
            if person.track_id == track_id:
                return self._activate_target(person)
        self._clear_target()
        return None

    def select_target_by_point(self, x: int, y: int) -> Optional[int]:
        for person in self.last_detections:
            x1, y1, x2, y2 = person.bbox
            if x1 <= x <= x2 and y1 <= y <= y2:
                return self._activate_target(person)

        self._clear_target()
        return None

    def process_frame(
        self,
        frame: np.ndarray,
        auto_lock_first: bool = False,
        spatial_estimator: Optional[
            Callable[[UnifiedTrackedPerson, int, int], Optional[Tuple[Tuple[float, float], str, str]]]
        ] = None,
        compute_all_sims: bool = False,
    ) -> Tuple[List[UnifiedTrackedPerson], Optional[UnifiedTrackedPerson]]:
        """
        단일 프레임을 추적·ReID 재식별 처리하고 검출 인원 리스트 및 현재 확정 타겟을 반환합니다.

        [모드 5 (Adaptive ReID) 핵심 동작 원리]
        1. `COEX` 등록: 타겟이 확실히 보일 때 멀리 떨어져 공존 중인 타 ID를 `coexisting_non_target_ids`에 기억.
        2. `SAFE`: 한산한 장면에서 `len(detections) >= 2`이고 유사도가 `id_switch_thresh`(0.48) 미만인 프레임이
           4회 연속되면 ByteTrack ID 탈취로 판단해 `target_id = -1`로 조기 이탈 후 `SEARCH` 진입.
           (단독 1인 추적 중 유사도 >= 0.72일 때만 `recent_bank` EMA 갱신)
        3. `GUARD`: 타겟 주변이 혼잡할 때 겹친 타인 박스를 제외한 특징 + 공간 점수(`0.60*sim + 0.40*spatial`)로
           즉시 교차 ID를 보정(+0.04 현재 ID 관성 보너스).
        4. `SEARCH`: 타겟 유실 시 `COEX` 차단(`>= coexist_min_sim`일 때만 `eligible`), 최소 유사도/점수,
           1-2위 격차(`recover_margin`), 연속 프레임 확정(`recover_confirm_frames`) 통과 시에만 `RECOVERED`.
        """
        t_start = time.perf_counter()
        self.last_frame = frame
        h, w = frame.shape[:2]

        tracker_cfg = (
            self.botsort_cfg_path if self.mode in self.BOTSORT_MODES else 'bytetrack.yaml'
        )

        track_kwargs = dict(
            persist=True,
            classes=[0],
            imgsz=self.imgsz,
            conf=self.conf,
            iou=self.iou,
            tracker=tracker_cfg,
            device=self.device,
            verbose=False,
        )
        if self.current_task_name == 'segment':
            track_kwargs['retina_masks'] = False

        results = self.model.track(frame, **track_kwargs)
        t_after_yolo = time.perf_counter()

        detections: List[UnifiedTrackedPerson] = []
        target_person: Optional[UnifiedTrackedPerson] = None
        adaptive_state = 'IDLE'
        extractor = self.current_extractor

        res = results[0]
        if res.boxes is not None and res.boxes.id is not None:
            boxes = res.boxes.xyxy.cpu().numpy().astype(int)
            ids = res.boxes.id.cpu().numpy().astype(int)
            n_det = len(ids)

            polys = (
                res.masks.xy
                if (self.current_task_name == 'segment' and res.masks is not None)
                else [None] * n_det
            )
            kpts = (
                res.keypoints.data.cpu().numpy()
                if (self.current_task_name == 'pose' and res.keypoints is not None)
                else [None] * n_det
            )

            person_banks: Dict[int, Optional[PartFeatureBank]] = {}
            ema_status = 'IDLE'
            compute_all_now = (
                self.anchor_bank is not None
                and (self.mode in self.REID_ACTIVE_MODES or compute_all_sims)
            )

            for (x1, y1, x2, y2), track_id, poly, kp in zip(boxes, ids, polys, kpts):
                track_id = int(track_id)
                bbox = (int(x1), int(y1), int(x2), int(y2))
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                norm_x = (cx - (w / 2.0)) / (w / 2.0)
                norm_y = (cy - (h / 2.0)) / (h / 2.0)
                area_ratio = float((x2 - x1) * (y2 - y1)) / float(w * h)

                sim = 1.0
                vis_parts = (True, True, True)
                part_sims: Tuple[Optional[float], Optional[float], Optional[float], Optional[float]] = (
                    None,
                    None,
                    None,
                    None,
                )
                bank: Optional[PartFeatureBank] = None
                if compute_all_now:
                    bank = extractor.extract(frame, bbox, poly, kp)
                    person_banks[track_id] = bank
                    if bank is not None:
                        vis_parts = bank.visible_parts
                    sim, part_sims = extractor.compute_dual_similarity_detail(
                        self.anchor_bank, self.recent_bank, bank
                    )

                person = UnifiedTrackedPerson(
                    track_id=track_id,
                    bbox=bbox,
                    center=(cx, cy),
                    bottom_center=(cx, int(y2)),
                    norm_offset=(norm_x, norm_y),
                    area_ratio=area_ratio,
                    is_target=(track_id == self.target_id),
                    similarity=sim,
                    mask_poly=poly,
                    keypoints=kp,
                    visible_parts=vis_parts,
                    part_sims=part_sims,
                    feature_bank=bank,
                )
                detections.append(person)

            if auto_lock_first and self.target_id is None and len(detections) > 0:
                first_p = detections[0]
                self.target_id = first_p.track_id
                self.last_target_bbox = first_p.bbox
                self._lock_target_bank(frame, first_p)

            if self.target_id is not None:
                # 1. 모델/태스크/트래커 전환 직후 1회 타겟 자동 승계 (위치 IoU 65% + 특징 유사도 35%)
                if self.pending_tracker_rebind and len(detections) > 0:
                    best_p = None
                    best_score = -1.0
                    for p in detections:
                        iou_score = (
                            self._bbox_iou(self.last_target_bbox, p.bbox)
                            if self.last_target_bbox is not None
                            else 0.0
                        )
                        sim_score = p.similarity
                        if not compute_all_now and self.anchor_bank is not None:
                            b_tmp = extractor.extract(frame, p.bbox, p.mask_poly, p.keypoints)
                            sim_score = extractor.compute_dual_similarity(
                                self.anchor_bank, self.recent_bank, b_tmp
                            )
                            p.similarity = sim_score
                        score = 0.65 * iou_score + 0.35 * sim_score
                        if score > best_score:
                            best_score = score
                            best_p = p
                    if best_p is not None and best_score >= 0.25:
                        self.target_id = best_p.track_id
                        self.pending_tracker_rebind = False
                        # 태스크가 바뀐 경우 새 태스크 추출기로 최근 뱅크 갱신
                        new_bank = extractor.extract(
                            frame, best_p.bbox, best_p.mask_poly, best_p.keypoints
                        )
                        if new_bank is not None:
                            self.recent_bank = new_bank.copy()

                current_target = next(
                    (p for p in detections if p.track_id == self.target_id), None
                )

                # 2. 모드 2 (Task-ReID) & 모드 4 (BoT+ReID)
                if self.mode in self.REID_ACTIVE_MODES and self.anchor_bank is not None:
                    if current_target is not None:
                        _, cur_dist = self._compute_spatial_score(
                            current_target, w, h, spatial_estimator
                        )
                        eff_switch_thresh = (
                            self.id_switch_thresh * 0.85
                            if (cur_dist is not None and cur_dist <= 0.55)
                            else self.id_switch_thresh
                        )
                        if current_target.similarity < eff_switch_thresh:
                            current_target.is_target = False
                            current_target = None

                    if current_target is None and len(detections) > 0:
                        best_candidate = max(
                            detections,
                            key=lambda p: self._score_candidate(p, w, h, spatial_estimator),
                        )
                        cand_score = self._score_candidate(
                            best_candidate, w, h, spatial_estimator
                        )
                        if (
                            best_candidate.similarity >= self.reid_match_thresh * 0.85
                            and cand_score >= self.reid_match_thresh
                        ):
                            self.target_id = best_candidate.track_id
                            best_candidate.is_target = True
                            current_target = best_candidate

                    if current_target is not None:
                        ema_status = self._update_target_banks_safely(
                            extractor,
                            current_target,
                            person_banks.get(current_target.track_id),
                            detections,
                        )

                # 3. 모드 5 (Adaptive 스위칭)
                elif self.mode == self.MODE_ADAPTIVE and self.anchor_bank is not None:
                    # [COEX 등록] 타겟과 동시 존재하며 겹치지 않고 멀리 떨어진 ID는 비타겟(COEX)으로 기억
                    if current_target is not None:
                        tx1, _, tx2, _ = current_target.bbox
                        tw = max(1, tx2 - tx1)
                        tcx, _ = current_target.center
                        for other in detections:
                            if other.track_id == current_target.track_id:
                                continue
                            if self._bbox_iou(current_target.bbox, other.bbox) >= 0.05:
                                # 타겟과 BBox가 겹치면 ID 교차 가능성이 있으므로 COEX 고정 목록에서 해제
                                self.coexisting_non_target_ids.discard(other.track_id)
                            elif abs(tcx - other.center[0]) > tw * 0.9:
                                self.coexisting_non_target_ids.add(other.track_id)

                    if current_target is not None and not self._is_crowded_near_target(
                        current_target, detections
                    ):
                        t_bank = person_banks.get(current_target.track_id)
                        if t_bank is None:
                            t_bank = extractor.extract(
                                frame,
                                current_target.bbox,
                                current_target.mask_poly,
                                current_target.keypoints,
                            )
                        current_target.feature_bank = t_bank
                        if t_bank is not None:
                            current_target.visible_parts = t_bank.visible_parts
                        current_target.similarity, current_target.part_sims = (
                            extractor.compute_dual_similarity_detail(
                                self.anchor_bank, self.recent_bank, t_bank
                            )
                        )
                        # [SAFE 이탈 검사] 다수 인원(len>=2)일 때 외형 불일치가 4프레임 누적되면
                        # anchor_bank는 유지한 채 target_id=-1로 해제하여 SEARCH 복귀 루프로 강제 진입
                        if len(detections) >= 2 and current_target.similarity < self.id_switch_thresh:
                            self.safe_mismatch_count += 1
                            if self.safe_mismatch_count >= self.safe_mismatch_limit:
                                self.target_id = -1
                                current_target.is_target = False
                                current_target = None
                                self.safe_mismatch_count = 0
                            else:
                                adaptive_state = 'SAFE'
                        else:
                            adaptive_state = 'SAFE'
                            self.safe_mismatch_count = 0
                            self.recover_candidate_id = None
                            self.recover_confirm_count = 0
                            # 안전 추적 구간에서 EMA 갱신 + 빈 부위(None) 보충 + BBox 폴백 승격 처리
                            ema_status = self._update_target_banks_safely(
                                extractor, current_target, t_bank, detections
                            )

                    if current_target is None:
                        adaptive_state = 'SEARCH'
                        ema_status = 'HOLD(SEARCH)'
                        scored_search: List[Tuple[float, UnifiedTrackedPerson]] = []
                        for p in detections:
                            b = person_banks.get(p.track_id)
                            if b is None:
                                b = extractor.extract(frame, p.bbox, p.mask_poly, p.keypoints)
                            p.feature_bank = b
                            if b is not None:
                                p.visible_parts = b.visible_parts
                            p.similarity, p.part_sims = extractor.compute_dual_similarity_detail(
                                self.anchor_bank, self.recent_bank, b
                            )
                            sc = self._score_candidate(p, w, h, spatial_estimator)
                            scored_search.append((sc, p))

                        if scored_search:
                            scored_search.sort(key=lambda item: item[0], reverse=True)
                            # [eligible 필터] 비공존(CAND)은 통과, 공존 비타겟(COEX)은 매우 높은 확신(>=coexist_min_sim)일 때만 허용
                            eligible = [
                                (sc, p) for sc, p in scored_search
                                if (p.track_id not in self.coexisting_non_target_ids)
                                or (p.similarity >= self.coexist_min_sim and sc >= self.coexist_min_sim)
                            ]
                            if not eligible:
                                top_sc, top_p = scored_search[0]
                                self.recover_candidate_id = None
                                self.recover_confirm_count = 0
                                self.last_search_reason = (
                                    f'COEX ID:{top_p.track_id} {top_p.similarity:.2f}<{self.coexist_min_sim:.2f}'
                                )
                            else:
                                cand_score, best_candidate = eligible[0]
                                second_score = eligible[1][0] if len(eligible) >= 2 else 0.0
                                is_coexist = best_candidate.track_id in self.coexisting_non_target_ids
                                # COEX 예외 통과자와 일반 CAND 후보에 서로 다른 복귀 문턱값 적용
                                min_sim_req = self.coexist_min_sim if is_coexist else self.search_min_sim
                                min_score_req = (
                                    self.coexist_min_sim if is_coexist else self.reid_match_thresh
                                )
                                # 1·2위 점수 차(margin_val)가 recover_margin 이상이어야 쌍둥이/모호한 복귀 방지
                                margin_val = cand_score - second_score
                                margin_ok = len(eligible) < 2 or margin_val >= self.recover_margin

                                if best_candidate.similarity < min_sim_req:
                                    self.recover_candidate_id = None
                                    self.recover_confirm_count = 0
                                    self.last_search_reason = (
                                        f'ID:{best_candidate.track_id} sim {best_candidate.similarity:.2f}<{min_sim_req:.2f}'
                                    )
                                elif cand_score < min_score_req:
                                    self.recover_candidate_id = None
                                    self.recover_confirm_count = 0
                                    self.last_search_reason = (
                                        f'ID:{best_candidate.track_id} score {cand_score:.2f}<{min_score_req:.2f}'
                                    )
                                elif not margin_ok:
                                    self.recover_candidate_id = None
                                    self.recover_confirm_count = 0
                                    self.last_search_reason = (
                                        f'Ambiguous ID:{best_candidate.track_id} gap {margin_val:.2f}<{self.recover_margin:.2f}'
                                    )
                                else:
                                    # 동일 후보가 recover_confirm_frames(기본 2프레임) 연속 1위를 유지해야 최종 복구 확정
                                    if self.recover_candidate_id == best_candidate.track_id:
                                        self.recover_confirm_count += 1
                                    else:
                                        self.recover_candidate_id = best_candidate.track_id
                                        self.recover_confirm_count = 1
                                    if self.recover_confirm_count >= self.recover_confirm_frames:
                                        self.target_id = best_candidate.track_id
                                        best_candidate.is_target = True
                                        current_target = best_candidate
                                        self.coexisting_non_target_ids.discard(best_candidate.track_id)
                                        self.recover_candidate_id = None
                                        self.recover_confirm_count = 0
                                        self.safe_mismatch_count = 0
                                        self.last_search_reason = f'RECOVERED ID:{best_candidate.track_id}'
                                        adaptive_state = 'RECOVERED'
                                    else:
                                        self.last_search_reason = (
                                            f'Confirm ID:{best_candidate.track_id} '
                                            f'({self.recover_confirm_count}/{self.recover_confirm_frames})'
                                        )
                        else:
                            self.recover_candidate_id = None
                            self.recover_confirm_count = 0
                            self.last_search_reason = 'No person in frame'

                    elif self._is_crowded_near_target(current_target, detections):
                        adaptive_state = 'GUARD'
                        ema_status = 'HOLD(GUARD)'
                        self.safe_mismatch_count = 0
                        self.recover_candidate_id = None
                        self.recover_confirm_count = 0
                        scored_list = []
                        for p in detections:
                            other_boxes = [o.bbox for o in detections if o.track_id != p.track_id]
                            b = extractor.extract(
                                frame, p.bbox, p.mask_poly, p.keypoints, exclude_boxes=other_boxes
                            )
                            person_banks[p.track_id] = b
                            p.feature_bank = b
                            if b is not None:
                                p.visible_parts = b.visible_parts
                            p.similarity, p.part_sims = extractor.compute_dual_similarity_detail(
                                self.anchor_bank, self.recent_bank, b
                            )
                            sp_score, sp_dist = self._compute_spatial_score(
                                p, w, h, spatial_estimator
                            )
                            comb = (
                                0.60 * p.similarity + 0.40 * sp_score
                                if (self.last_target_world_xy is not None and spatial_estimator is not None)
                                else p.similarity
                            )
                            # 현재 track_id에는 미세한 관성 보너스만 부여 (+0.04), 단 라이다상 급점프(>0.9m) 시 제거
                            if p.track_id == current_target.track_id and (sp_dist is None or sp_dist <= 0.90):
                                comb += 0.04
                            scored_list.append((comb, p))

                        scored_list.sort(key=lambda item: item[0], reverse=True)
                        best_comb, best_candidate = scored_list[0]
                        if (
                            best_candidate.track_id != current_target.track_id
                            and best_candidate.similarity >= self.search_min_sim
                            and best_comb >= self.reid_match_thresh
                        ):
                            self.target_id = best_candidate.track_id
                            current_target = best_candidate

            for p in detections:
                p.is_target = (p.track_id == self.target_id)
                if p.is_target:
                    target_person = p
                    self.last_target_bbox = p.bbox
                    if spatial_estimator is not None:
                        est = spatial_estimator(p, w, h)
                        if est is not None:
                            self.last_target_world_xy, self.last_target_world_frame, _ = est
                            self.last_target_seen_time = time.time()
        else:
            ema_status = (
                'HOLD(EMPTY)'
                if (self.mode == self.MODE_ADAPTIVE and self.target_id is not None)
                else 'IDLE'
            )
            if self.mode == self.MODE_ADAPTIVE and self.target_id is not None:
                adaptive_state = 'SEARCH'
                self.recover_candidate_id = None
                self.recover_confirm_count = 0
                self.last_search_reason = 'No person in frame'

        t_end = time.perf_counter()

        spd = res.speed
        pre_ms = float(spd.get('preprocess', 0.0))
        inf_ms = float(spd.get('inference', 0.0))
        post_ms = float(spd.get('postprocess', 0.0))
        reid_ms = (t_end - t_after_yolo) * 1000.0
        total_ms = (t_end - t_start) * 1000.0

        alpha = 0.15
        m = self.metrics
        m.model_label = self.current_model_name
        m.task = self.current_task_name.upper()
        m.device = self.device.upper()
        m.preprocess_ms = (1 - alpha) * m.preprocess_ms + alpha * pre_ms if m.total_ms > 0 else pre_ms
        m.inference_ms = (1 - alpha) * m.inference_ms + alpha * inf_ms if m.total_ms > 0 else inf_ms
        m.post_track_ms = (1 - alpha) * m.post_track_ms + alpha * post_ms if m.total_ms > 0 else post_ms
        m.reid_ms = (1 - alpha) * m.reid_ms + alpha * reid_ms if m.total_ms > 0 else reid_ms
        m.total_ms = (1 - alpha) * m.total_ms + alpha * total_ms if m.total_ms > 0 else total_ms
        m.max_infer_fps = 1000.0 / max(0.1, m.total_ms)
        m.adaptive_state = adaptive_state
        if adaptive_state == 'SAFE' or adaptive_state == 'GUARD' or adaptive_state == 'IDLE':
            self.last_search_reason = ''
        m.search_reason = self.last_search_reason
        m.ema_status = ema_status

        self.last_detections = detections
        return detections, target_person
