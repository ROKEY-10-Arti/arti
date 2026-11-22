"""
부위별 외형 특징 추출(304D HSV) 및 ReID 유사도 비교 모듈 (feature_extractors.py)

[전체 처리 흐름 및 주요 클래스/함수]
1. 태스크별 부위 특징 추출 (`BaseFeatureExtractor.extract` 구현체):
   - Pose 태스크 (`PoseKeypointFeatureExtractor`): COCO 17관절(어깨 5/6, 골반 11/12, 무릎 13/14, 발목 15/16)로
     실제 해부학적 상의/허벅지/정강이 마스크를 생성합니다. (관절 부족 시 `BBoxFeatureExtractor` 폴백)
   - Segment 태스크 (`SegMaskFeatureExtractor`): 인스턴스 폴리곤 마스크(1px 침식)를 높이 비율 3등분합니다.
   - Detect 태스크 (`BBoxFeatureExtractor`): 박스 중앙 영역(가로 60%, 세로 80%)을 3등분합니다.
   - 겹침/가려짐 구간에서는 `_apply_exclude_boxes()`로 타인 박스 픽셀을 마스킹에서 제거해 특징 오염을 막습니다.
2. 304차원 HSV 히스토그램 생성 (`BaseFeatureExtractor.calc_hsv_hist`):
   - 유채색 구분용 `18(Hue) x 16(Sat)` 2D 히스토그램(288D, 가중치 0.72)과
     무채색(검정/흰색/회색) 구분용 `16(Value)` 1D 히스토그램(16D, 가중치 0.28)을 L1 정규화 후 결합해
     총 304차원 벡터를 생성하고 `PartFeatureBank(full, upper, mid, lower)`로 반환합니다.
3. 부위별 1:1 및 이중 뱅크(Anchor + Recent) 유사도 계산 (`compute_dual_similarity_detail` -> `compare_banks_detail` -> `hist_sim`):
   - `Bhattacharyya` 거리를 `max(0.0, 1.0 - dist)` 유사도로 변환하며,
     가시 부위 가중 평균(상반신 0.44, 중반신 0.36, 하반신 0.20)과 전체(`full`) 유사도의 최댓값을 사용합니다.
   - `PoseKeypointFeatureExtractor`는 `ALLOW_CROSS_PART = False`를 적용해 상의-하의 교차 매칭 오류를 차단합니다.
4. 최근 외형 지수이동평균(EMA) 갱신 (`BaseFeatureExtractor.update_recent_bank`):
   - 조명/각도 변화에 적응하기 위해 안정 추적 구간에서 `recent_bank`를 `alpha=0.04` 비율로 점진 갱신합니다.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import cv2
import numpy as np


@dataclass
class PartFeatureBank:
    """
    모든 태스크(Detect / Segment / Pose)가 공통으로 반환하는 3부위 + 전체 특징 뱅크
      - full : 전체 가시 영역 히스토그램
      - upper: 상반신(어깨~골반 / 상의) 히스토그램
      - mid  : 중하반신(골반~무릎 / 바지) 히스토그램
      - lower: 최하단(무릎~발목 / 정강이·신발) 히스토그램
      - edges: (선택) Pose 골격 연결선(마커-마커 링크)별 개별 304D 히스토그램 사전
    """
    full: np.ndarray
    upper: Optional[np.ndarray]
    mid: Optional[np.ndarray]
    lower: Optional[np.ndarray]
    visible_parts: Tuple[bool, bool, bool] = (True, True, True)
    is_fallback: bool = False
    edges: Optional[Dict[str, np.ndarray]] = field(default=None)

    def copy(self) -> 'PartFeatureBank':
        return PartFeatureBank(
            full=self.full.copy(),
            upper=self.upper.copy() if self.upper is not None else None,
            mid=self.mid.copy() if self.mid is not None else None,
            lower=self.lower.copy() if self.lower is not None else None,
            visible_parts=self.visible_parts,
            is_fallback=self.is_fallback,
            edges={k: v.copy() for k, v in self.edges.items()} if self.edges else None,
        )


class BaseFeatureExtractor:
    """태스크별 특징 추출기의 공통 베이스 클래스 (Strategy Pattern)"""

    ALLOW_CROSS_PART = True

    @staticmethod
    def calc_hsv_hist(hsv_roi: np.ndarray, mask_roi: np.ndarray) -> Optional[np.ndarray]:
        """
        마스크 영역(`mask_roi`) 내부 픽셀만 사용해 304차원 L1 정규화 HSV 특징 벡터를 생성합니다.

        [알고리즘 원리]
        - 유효 픽셀이 12개 미만이면 통계 불안정으로 None을 반환합니다.
        - 색상+채도(H: 18bin, S: 16bin -> 288D) L1 정규화 히스토그램(가중치 0.72)과
          밝기(V: 16bin -> 16D) L1 정규화 히스토그램(가중치 0.28)을 연결합니다.
        - V 성분을 포함시켜 검정/회색/흰색 같은 무채색 의상도 구별할 수 있게 합니다.
        """
        if hsv_roi.size == 0 or mask_roi.size == 0:
            return None
        if cv2.countNonZero(mask_roi) < 12:
            return None
        # 색상+채도(H-S) 2D 히스토그램 + 무채색(검정/흰색/회색 옷) 구분용 밝기(V) 1D 히스토그램 결합
        hs_hist = cv2.calcHist([hsv_roi], [0, 1], mask_roi, [18, 16], [0, 180, 0, 256])
        cv2.normalize(hs_hist, hs_hist, alpha=1.0, norm_type=cv2.NORM_L1)
        v_hist = cv2.calcHist([hsv_roi], [2], mask_roi, [16], [0, 256])
        cv2.normalize(v_hist, v_hist, alpha=1.0, norm_type=cv2.NORM_L1)
        combined = np.concatenate([0.72 * hs_hist.ravel(), 0.28 * v_hist.ravel()]).astype(np.float32)
        return combined

    @staticmethod
    def hist_sim(h1: Optional[np.ndarray], h2: Optional[np.ndarray]) -> float:
        """Bhattacharyya 거리 d in [0, 1]을 (1.0 - d) 코사인/분포 유사도 [0.0 ~ 1.0]으로 변환합니다."""
        if h1 is None or h2 is None:
            return 0.0
        dist = cv2.compareHist(h1, h2, cv2.HISTCMP_BHATTACHARYYA)
        return float(max(0.0, 1.0 - dist))

    @staticmethod
    def _apply_exclude_boxes(
        mask_roi: np.ndarray,
        bbox: Tuple[int, int, int, int],
        exclude_boxes: Optional[List[Tuple[int, int, int, int]]],
    ) -> np.ndarray:
        """다른 사람 박스와 겹치는 영역을 마스크에서 제외해 외형 오염을 방지합니다."""
        if not exclude_boxes:
            return mask_roi
        x1, y1, x2, y2 = bbox
        cleaned = mask_roi.copy()
        rh, rw = cleaned.shape[:2]
        for ox1, oy1, ox2, oy2 in exclude_boxes:
            ix1 = max(x1, ox1) - x1
            iy1 = max(y1, oy1) - y1
            ix2 = min(x2, ox2) - x1
            iy2 = min(y2, oy2) - y1
            if ix2 > ix1 and iy2 > iy1:
                cleaned[max(0, iy1):min(rh, iy2), max(0, ix1):min(rw, ix2)] = 0
        # 제외 후 남은 픽셀이 너무 적으면 원본 유지
        if cv2.countNonZero(cleaned) < 25:
            return mask_roi
        return cleaned

    def _prepare_part_masks(
        self,
        frame: np.ndarray,
        bbox: Tuple[int, int, int, int],
        mask_poly: Optional[np.ndarray] = None,
        keypoints: Optional[np.ndarray] = None,
        exclude_boxes: Optional[List[Tuple[int, int, int, int]]] = None,
    ) -> Optional[
        Tuple[Tuple[int, int, int, int], np.ndarray, np.ndarray, np.ndarray, np.ndarray, bool]
    ]:
        """ROI 좌표 `(x1, y1, x2, y2)`, `roi_hsv`, 3부위 마스크 `(upper, mid, lower)`, `is_fallback`을 반환합니다."""
        raise NotImplementedError

    def extract_part_masks(
        self,
        frame: np.ndarray,
        bbox: Tuple[int, int, int, int],
        mask_poly: Optional[np.ndarray] = None,
        keypoints: Optional[np.ndarray] = None,
        exclude_boxes: Optional[List[Tuple[int, int, int, int]]] = None,
    ) -> Optional[Tuple[Tuple[int, int, int, int], np.ndarray, np.ndarray, np.ndarray]]:
        """시각화/디버깅용으로 특징 추출에 실제로 사용되는 `(roi_box, upper_mask, mid_mask, lower_mask)`를 반환합니다."""
        prepared = self._prepare_part_masks(
            frame,
            bbox,
            mask_poly=mask_poly,
            keypoints=keypoints,
            exclude_boxes=exclude_boxes,
        )
        if prepared is None:
            return None
        roi_box, _, upper_mask, mid_mask, lower_mask, _ = prepared
        rx1, ry1, rx2, ry2 = roi_box
        target_shape = (max(1, ry2 - ry1), max(1, rx2 - rx1))
        if upper_mask.shape[:2] != target_shape:
            wh = (target_shape[1], target_shape[0])
            upper_mask = cv2.resize(upper_mask, wh, interpolation=cv2.INTER_NEAREST)
            mid_mask = cv2.resize(mid_mask, wh, interpolation=cv2.INTER_NEAREST)
            lower_mask = cv2.resize(lower_mask, wh, interpolation=cv2.INTER_NEAREST)
        return roi_box, upper_mask, mid_mask, lower_mask

    @staticmethod
    def _split_mask_by_ratios(
        mask: np.ndarray, upper_ratio: float, mid_ratio: float
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """단일 마스크를 높이 비율(`upper_ratio`, `mid_ratio`) 기준으로 3개 부위 마스크로 분할합니다."""
        h = mask.shape[0]
        y_u, y_m = int(h * upper_ratio), int(h * mid_ratio)
        upper_m = np.zeros_like(mask)
        mid_m = np.zeros_like(mask)
        lower_m = np.zeros_like(mask)
        upper_m[:y_u, :] = mask[:y_u, :]
        mid_m[y_u:y_m, :] = mask[y_u:y_m, :]
        lower_m[y_m:, :] = mask[y_m:, :]
        return upper_m, mid_m, lower_m

    def extract(
        self,
        frame: np.ndarray,
        bbox: Tuple[int, int, int, int],
        mask_poly: Optional[np.ndarray] = None,
        keypoints: Optional[np.ndarray] = None,
        exclude_boxes: Optional[List[Tuple[int, int, int, int]]] = None,
    ) -> Optional[PartFeatureBank]:
        """태스크별(BBox/Segment/Pose) 사람 ROI에서 304D HSV PartFeatureBank를 추출하는 공통 인터페이스."""
        prepared = self._prepare_part_masks(
            frame,
            bbox,
            mask_poly=mask_poly,
            keypoints=keypoints,
            exclude_boxes=exclude_boxes,
        )
        if prepared is None:
            return None
        _, roi_hsv, upper_mask, mid_mask, lower_mask, is_fallback = prepared
        combined_mask = cv2.bitwise_or(upper_mask, cv2.bitwise_or(mid_mask, lower_mask))
        full_hist = self.calc_hsv_hist(roi_hsv, combined_mask)
        if full_hist is None:
            return None

        upper_h = self.calc_hsv_hist(roi_hsv, upper_mask)
        mid_h = self.calc_hsv_hist(roi_hsv, mid_mask)
        lower_h = self.calc_hsv_hist(roi_hsv, lower_mask)

        return PartFeatureBank(
            full=full_hist,
            upper=upper_h,
            mid=mid_h,
            lower=lower_h,
            visible_parts=(upper_h is not None, mid_h is not None, lower_h is not None),
            is_fallback=is_fallback,
        )

    EDGE_WEIGHTS: Dict[str, float] = {
        'torso': 0.28,
        'shoulder': 0.14,
        'l_side': 0.14,
        'r_side': 0.14,
        'l_uarm': 0.16,
        'r_uarm': 0.16,
        'l_farm': 0.08,
        'r_farm': 0.08,
        'hip': 0.08,
        'l_thigh': 0.16,
        'r_thigh': 0.16,
        'l_calf': 0.12,
        'r_calf': 0.12,
    }
    SYMMETRIC_EDGE_PAIRS: Tuple[Tuple[str, str], ...] = (
        ('l_side', 'r_side'),
        ('l_uarm', 'r_uarm'),
        ('l_farm', 'r_farm'),
        ('l_thigh', 'r_thigh'),
        ('l_calf', 'r_calf'),
    )

    def _compare_edges(
        self, ref_edges: Optional[Dict[str, np.ndarray]], cur_edges: Optional[Dict[str, np.ndarray]]
    ) -> Tuple[Optional[float], int]:
        """마커 간 연결선(`edges`) 1:1 유사도 + 미검출 측면 대칭 폴백 가중 평균을 계산합니다."""
        if not ref_edges or not cur_edges:
            return None, 0

        weighted_scores: List[Tuple[float, float]] = []
        matched_keys = set()
        for key, weight in self.EDGE_WEIGHTS.items():
            r_h = ref_edges.get(key)
            c_h = cur_edges.get(key)
            if r_h is not None and c_h is not None:
                weighted_scores.append((self.hist_sim(r_h, c_h), weight))
                matched_keys.add(key)

        # 한쪽 측면만 관측되어 반대쪽 키가 비어 있을 때만 좌우 대칭 연결선 보완 비교
        for left_k, right_k in self.SYMMETRIC_EDGE_PAIRS:
            if left_k in matched_keys or right_k in matched_keys:
                continue
            w = self.EDGE_WEIGHTS.get(left_k, 0.12) * 0.85
            if left_k in ref_edges and right_k in cur_edges:
                weighted_scores.append((self.hist_sim(ref_edges[left_k], cur_edges[right_k]), w))
                matched_keys.add(f'{left_k}~{right_k}')
            elif right_k in ref_edges and left_k in cur_edges:
                weighted_scores.append((self.hist_sim(ref_edges[right_k], cur_edges[left_k]), w))
                matched_keys.add(f'{right_k}~{left_k}')

        if not weighted_scores:
            return None, 0
        total_w = sum(w for _, w in weighted_scores)
        if total_w <= 1e-8:
            return None, 0
        return float(sum(s * w for s, w in weighted_scores) / total_w), len(weighted_scores)

    def compare_banks_detail(
        self, ref: Optional[PartFeatureBank], cur: Optional[PartFeatureBank]
    ) -> Tuple[float, Optional[float], Optional[float], Optional[float], float]:
        """
        부위별 유사도와 최종 종합 점수를 함께 반환합니다:
        (total_score, upper_sim, mid_sim, lower_sim, full_sim)

        [비교 원리]
        - 기준(`ref`)과 현재(`cur`) 모두 추출된 부위끼리만 1:1 유사도(`upper`, `mid`, `lower`)를 계산하고,
          가용 부위 가중치(상의 0.44, 허벅지 0.36, 정강이 0.20)로 `aligned_score`를 산출합니다.
        - `edges`가 추출된 경우 마커-마커 연결선 1:1 비교 점수(`edge_score`)를 결합합니다.
        - `score = max(full_score, aligned_score)`를 사용해 일부 관절 가림 시에도 전체(`full`) 점수로 보완합니다.
        - Pose 추출기는 `ALLOW_CROSS_PART = False`이므로 상의가 가려졌을 때 중/하반신과 교차 비교하지 않습니다.
        """
        if ref is None or cur is None:
            return 0.0, None, None, None, 0.0

        upper_sim = (
            self.hist_sim(ref.upper, cur.upper)
            if (ref.upper is not None and cur.upper is not None)
            else None
        )
        mid_sim = (
            self.hist_sim(ref.mid, cur.mid)
            if (ref.mid is not None and cur.mid is not None)
            else None
        )
        lower_sim = (
            self.hist_sim(ref.lower, cur.lower)
            if (ref.lower is not None and cur.lower is not None)
            else None
        )

        part_scores = []
        for sim_val, weight in (
            (upper_sim, 0.44),
            (mid_sim, 0.36),
            (lower_sim, 0.20),
        ):
            if sim_val is not None:
                part_scores.append((sim_val, weight))

        # 가용 부위만으로 가중 평균을 구하되, 가림/자세 변화 보완을 위해 full 유사도와 최댓값 선택
        full_score = self.hist_sim(ref.full, cur.full)
        if part_scores:
            aligned_score = sum(s * w for s, w in part_scores) / sum(w for _, w in part_scores)
            score = max(full_score, aligned_score)
        else:
            score = full_score

        edge_score_mode = getattr(self, 'edge_score_mode', 'none')
        if edge_score_mode != 'none' and not ref.is_fallback and not cur.is_fallback:
            edge_score, n_edges = self._compare_edges(ref.edges, cur.edges)
            if edge_score is not None and n_edges >= 1:
                if edge_score_mode == 'blend' and n_edges >= 2:
                    score = 0.55 * edge_score + 0.45 * score
                elif edge_score_mode == 'hybrid' and n_edges >= 2:
                    score = 0.70 * score + 0.30 * edge_score
                else:
                    score = max(score, edge_score)

        # Pose는 상의-하의 교차 매칭 오류를 막기 위해 ALLOW_CROSS_PART=False로 비활성화됨
        if self.ALLOW_CROSS_PART and cur.upper is None:
            cross_candidates = [score]
            if ref.mid is not None:
                cross_candidates.append(self.hist_sim(ref.mid, cur.full) * 0.92)
            if ref.lower is not None:
                cross_candidates.append(self.hist_sim(ref.lower, cur.full) * 0.90)
            score = max(cross_candidates)

        return float(score), upper_sim, mid_sim, lower_sim, float(full_score)

    def compare_banks(
        self, ref: Optional[PartFeatureBank], cur: Optional[PartFeatureBank]
    ) -> float:
        """
        현재 보이는 부위(visible_parts)끼리 1:1 매칭을 우선 수행합니다.
        """
        score, _, _, _, _ = self.compare_banks_detail(ref, cur)
        return score

    def compute_dual_similarity_detail(
        self,
        anchor_bank: Optional[PartFeatureBank],
        recent_bank: Optional[PartFeatureBank],
        cur_bank: Optional[PartFeatureBank],
    ) -> Tuple[float, Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]]:
        """
        초기 등록 외형(`anchor_bank`)과 최근 적응 외형(`recent_bank`) 중 더 높은 종합 유사도와
        해당 비교의 세부 부위 점수 `(upper, mid, lower, full)`을 반환합니다.

        - 초기 등록 자세(Anchor)와 현재 조명/자세 변화(Recent)를 모두 허용해 재인식률을 유지합니다.
        """
        if cur_bank is None:
            return 0.0, (None, None, None, None)

        anchor_detail = self.compare_banks_detail(anchor_bank, cur_bank)
        recent_detail = self.compare_banks_detail(recent_bank, cur_bank)

        if recent_detail[0] > anchor_detail[0]:
            return float(recent_detail[0]), (
                recent_detail[1],
                recent_detail[2],
                recent_detail[3],
                recent_detail[4],
            )
        return float(anchor_detail[0]), (
            anchor_detail[1],
            anchor_detail[2],
            anchor_detail[3],
            anchor_detail[4],
        )

    def compute_dual_similarity(
        self,
        anchor_bank: Optional[PartFeatureBank],
        recent_bank: Optional[PartFeatureBank],
        cur_bank: Optional[PartFeatureBank],
    ) -> float:
        score, _ = self.compute_dual_similarity_detail(anchor_bank, recent_bank, cur_bank)
        return score

    @staticmethod
    def backfill_part(
        bank: Optional[PartFeatureBank],
        part_name: str,
        hist: Optional[np.ndarray],
    ) -> bool:
        """비어 있던(`None`) 부위 특징을 1회 보충하고 `visible_parts`를 갱신합니다."""
        if bank is None or hist is None or getattr(bank, part_name) is not None:
            return False
        setattr(bank, part_name, hist.copy())
        bank.visible_parts = (
            bank.upper is not None,
            bank.mid is not None,
            bank.lower is not None,
        )
        return True

    @staticmethod
    def backfill_edges(
        bank: Optional[PartFeatureBank],
        cur_edges: Optional[Dict[str, np.ndarray]],
    ) -> bool:
        """안전 추적 구간에서 비어 있던 골격 연결선 히스토그램을 보충합니다."""
        if bank is None or not cur_edges:
            return False
        if bank.edges is None:
            bank.edges = {}
        added = False
        for key, hist in cur_edges.items():
            if key not in bank.edges and hist is not None:
                bank.edges[key] = hist.copy()
                added = True
        return added

    @staticmethod
    def update_recent_bank(
        recent_bank: Optional[PartFeatureBank],
        cur_bank: Optional[PartFeatureBank],
        alpha: float = 0.04,
    ):
        """유효한 부위 히스토그램에 대해 EMA(`(1 - alpha) * recent + alpha * cur`)로 최근 외형을 완만히 갱신합니다."""
        if recent_bank is None or cur_bank is None:
            return
        recent_bank.full = (1.0 - alpha) * recent_bank.full + alpha * cur_bank.full
        if recent_bank.upper is not None and cur_bank.upper is not None:
            recent_bank.upper = (1.0 - alpha) * recent_bank.upper + alpha * cur_bank.upper
        if recent_bank.mid is not None and cur_bank.mid is not None:
            recent_bank.mid = (1.0 - alpha) * recent_bank.mid + alpha * cur_bank.mid
        if recent_bank.lower is not None and cur_bank.lower is not None:
            recent_bank.lower = (1.0 - alpha) * recent_bank.lower + alpha * cur_bank.lower
        if recent_bank.edges and cur_bank.edges:
            for key, cur_e in cur_bank.edges.items():
                ref_e = recent_bank.edges.get(key)
                if ref_e is not None and cur_e is not None:
                    recent_bank.edges[key] = (1.0 - alpha) * ref_e + alpha * cur_e


class BBoxFeatureExtractor(BaseFeatureExtractor):
    """[Detect 태스크용] 바운딩 박스 내부 중앙 크롭 + 3등분 특징 추출기"""

    def _prepare_part_masks(
        self,
        frame: np.ndarray,
        bbox: Tuple[int, int, int, int],
        mask_poly: Optional[np.ndarray] = None,
        keypoints: Optional[np.ndarray] = None,
        exclude_boxes: Optional[List[Tuple[int, int, int, int]]] = None,
    ) -> Optional[
        Tuple[Tuple[int, int, int, int], np.ndarray, np.ndarray, np.ndarray, np.ndarray]
    ]:
        """
        BBox 중앙 크롭(가로 20%~80%, 세로 12%~92%)에서 배경 노이즈를 줄이고
        세로 비율(0.38 / 0.72) 기준 3등분 마스크를 생성합니다.
        """
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = bbox
        bw, bh = x2 - x1, y2 - y1
        if bw < 16 or bh < 28:
            return None

        cx1 = max(0, int(x1 + bw * 0.20))
        cx2 = min(w, int(x2 - bw * 0.20))
        cy1 = max(0, int(y1 + bh * 0.12))
        cy2 = min(h, int(y2 - bh * 0.08))
        if cx2 - cx1 < 8 or cy2 - cy1 < 16:
            return None

        roi_hsv = cv2.cvtColor(frame[cy1:cy2, cx1:cx2], cv2.COLOR_BGR2HSV)
        # 무채색(S=0~20) 옷도 포함되도록 S 하한을 0으로 두고, 극단적 암부 노이즈(V<15)만 제외
        mask = cv2.inRange(roi_hsv, np.array([0, 0, 15]), np.array([180, 255, 255]))
        mask = self._apply_exclude_boxes(mask, (cx1, cy1, cx2, cy2), exclude_boxes)
        if cv2.countNonZero(mask) < 12:
            return None

        upper_m, mid_m, lower_m = self._split_mask_by_ratios(mask, 0.38, 0.72)
        return (cx1, cy1, cx2, cy2), roi_hsv, upper_m, mid_m, lower_m, False


class SegMaskFeatureExtractor(BaseFeatureExtractor):
    """
    [Segment 태스크용] 폴리곤 실루엣 마스킹(배경 0%) + 3부위 특징 추출기
      - `seg_mode='baseline'`: 기존 높이 비율(0.35 / 0.70) 3등분
      - `seg_mode='geom_fast'`: 대용량 ROI 다운스케일 가속 + 화면 하/상단 잘림·종횡비 기반 해부학 부위 매핑(근접 시 상체 전용화, 전신 시 얼굴/머리 상단 제외)
    """

    def __init__(self, seg_mode: str = 'baseline', max_roi_dim: int = 128):
        self.seg_mode = seg_mode
        self.max_roi_dim = int(max_roi_dim)

    def _split_geom_aware(
        self,
        final_mask: np.ndarray,
        bbox: Tuple[int, int, int, int],
        frame_shape: Tuple[int, int],
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        fh, fw = frame_shape
        x1, y1, x2, y2 = bbox
        bw = max(1, x2 - x1)
        bh = max(1, y2 - y1)
        mh, mw = final_mask.shape[:2]

        margin_edge = max(6, int(fh * 0.015))
        clipped_bottom = y2 >= (fh - margin_edge)
        clipped_top = y1 <= margin_edge
        aspect = float(bh) / float(bw)
        area_ratio = float(bw * bh) / float(max(1, fw * fh))

        upper_m = np.zeros_like(final_mask)
        mid_m = np.zeros_like(final_mask)
        lower_m = np.zeros_like(final_mask)

        # 1) 극단적 근접(상/하단 동시 잘림 또는 화면 32% 이상 차지 & 납작한 상체 크롭) -> 상체 전용
        if (clipped_top and clipped_bottom) or (area_ratio >= 0.32 and aspect < 1.35):
            y_start = 0 if clipped_top else int(mh * 0.08)
            upper_m[y_start:, :] = final_mask[y_start:, :]
            return upper_m, mid_m, lower_m

        # 2) 근접으로 하단이 화면 밖으로 잘린 경우: 어깨/몸통 폭 기반 전신 예상 높이 H_est로 가시 구간만 매핑
        if clipped_bottom and not clipped_top and aspect < 1.85:
            h_est_px = max(float(bh) * 1.12, float(bw) * 2.25)
            scale_y = float(mh) / float(bh)
            h_est_m = h_est_px * scale_y
            y_head = int(round(0.13 * h_est_m))
            y_waist = int(round(0.48 * h_est_m))
            y_knee = int(round(0.75 * h_est_m))

            u_s, u_e = min(mh, y_head), min(mh, y_waist)
            if u_e > u_s:
                upper_m[u_s:u_e, :] = final_mask[u_s:u_e, :]
            m_s, m_e = min(mh, y_waist), min(mh, y_knee)
            if m_e > m_s:
                mid_m[m_s:m_e, :] = final_mask[m_s:m_e, :]
            l_s = min(mh, y_knee)
            if mh > l_s:
                lower_m[l_s:, :] = final_mask[l_s:, :]

            # 머리만 크게 잡힌 예외 구간 등에서 상체가 너무 적으면 전체 마스크를 upper로 보전
            if cv2.countNonZero(upper_m) < 20 and cv2.countNonZero(mid_m) == 0:
                upper_m[:, :] = final_mask
            return upper_m, mid_m, lower_m

        # 3) 상단이 잘리고 하단만 보이는 경우: 발바닥(하단) 기준 역산 매핑
        if clipped_top and not clipped_bottom and aspect < 1.85:
            h_est_px = max(float(bh) * 1.12, float(bw) * 2.25)
            scale_y = float(mh) / float(bh)
            h_est_m = h_est_px * scale_y
            y_knee = max(0, int(round(mh - 0.25 * h_est_m)))
            y_waist = max(0, int(round(mh - 0.52 * h_est_m)))
            if y_waist > 0:
                upper_m[:y_waist, :] = final_mask[:y_waist, :]
            if y_knee > y_waist:
                mid_m[y_waist:y_knee, :] = final_mask[y_waist:y_knee, :]
            if mh > y_knee:
                lower_m[y_knee:, :] = final_mask[y_knee:, :]
            return upper_m, mid_m, lower_m

        # 4) 일반 전신/준전신 뷰: 최상단 머리/머리카락(0~13%) 제외 후 상/중/하 해부학 비율 분할
        y_head = int(mh * 0.13)
        y_waist = int(mh * 0.48)
        y_knee = int(mh * 0.75)
        upper_m[y_head:y_waist, :] = final_mask[y_head:y_waist, :]
        mid_m[y_waist:y_knee, :] = final_mask[y_waist:y_knee, :]
        lower_m[y_knee:, :] = final_mask[y_knee:, :]
        return upper_m, mid_m, lower_m

    def _prepare_part_masks(
        self,
        frame: np.ndarray,
        bbox: Tuple[int, int, int, int],
        mask_poly: Optional[np.ndarray] = None,
        keypoints: Optional[np.ndarray] = None,
        exclude_boxes: Optional[List[Tuple[int, int, int, int]]] = None,
        allow_downscale: bool = False,
    ) -> Optional[
        Tuple[Tuple[int, int, int, int], np.ndarray, np.ndarray, np.ndarray, np.ndarray, bool]
    ]:
        h, w = frame.shape[:2]
        x1, y1, x2, y2 = (
            max(0, bbox[0]),
            max(0, bbox[1]),
            min(w, bbox[2]),
            min(h, bbox[3]),
        )
        bw, bh = x2 - x1, y2 - y1
        if bw < 12 or bh < 20:
            return None

        roi_bgr = frame[y1:y2, x1:x2]
        scale = 1.0
        rw, rh = bw, bh
        if (
            self.seg_mode == 'geom_fast'
            and allow_downscale
            and max(bw, bh) > self.max_roi_dim
        ):
            scale = float(self.max_roi_dim) / float(max(bw, bh))
            rw = max(12, int(round(bw * scale)))
            rh = max(20, int(round(bh * scale)))
            roi_bgr = cv2.resize(roi_bgr, (rw, rh), interpolation=cv2.INTER_AREA)

        roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        seg_mask = np.zeros((rh, rw), dtype=np.uint8)

        if mask_poly is not None and len(mask_poly) >= 3:
            local_poly = mask_poly.astype(np.float32).copy()
            local_poly[:, 0] = np.clip((local_poly[:, 0] - x1) * scale, 0, rw - 1)
            local_poly[:, 1] = np.clip((local_poly[:, 1] - y1) * scale, 0, rh - 1)
            cv2.fillPoly(seg_mask, [local_poly.astype(np.int32)], 255)
            if rw > 24 and rh > 40:
                seg_mask = cv2.erode(seg_mask, np.ones((3, 3), np.uint8), iterations=1)
        else:
            seg_mask[int(rh * 0.1):int(rh * 0.9), int(rw * 0.15):int(rw * 0.85)] = 255

        valid_color = cv2.inRange(roi_hsv, np.array([0, 0, 15]), np.array([180, 255, 255]))
        final_mask = cv2.bitwise_and(seg_mask, valid_color)
        if cv2.countNonZero(final_mask) < 20:
            final_mask = seg_mask

        if exclude_boxes:
            scaled_ex = []
            for ox1, oy1, ox2, oy2 in exclude_boxes:
                scaled_ex.append((
                    int(round(x1 + (ox1 - x1) * scale)),
                    int(round(y1 + (oy1 - y1) * scale)),
                    int(round(x1 + (ox2 - x1) * scale)),
                    int(round(y1 + (oy2 - y1) * scale)),
                ))
            final_mask = self._apply_exclude_boxes(
                final_mask, (x1, y1, x1 + rw, y1 + rh), scaled_ex
            )

        if cv2.countNonZero(final_mask) < 12:
            return None

        if self.seg_mode == 'geom_fast':
            upper_m, mid_m, lower_m = self._split_geom_aware(
                final_mask, (x1, y1, x2, y2), (h, w)
            )
        else:
            upper_m, mid_m, lower_m = self._split_mask_by_ratios(final_mask, 0.35, 0.70)
        return (x1, y1, x2, y2), roi_hsv, upper_m, mid_m, lower_m, False

    def extract(
        self,
        frame: np.ndarray,
        bbox: Tuple[int, int, int, int],
        mask_poly: Optional[np.ndarray] = None,
        keypoints: Optional[np.ndarray] = None,
        exclude_boxes: Optional[List[Tuple[int, int, int, int]]] = None,
    ) -> Optional[PartFeatureBank]:
        prepared = self._prepare_part_masks(
            frame,
            bbox,
            mask_poly=mask_poly,
            keypoints=keypoints,
            exclude_boxes=exclude_boxes,
            allow_downscale=True,
        )
        if prepared is None:
            return None
        _, roi_hsv, upper_mask, mid_mask, lower_mask, is_fallback = prepared
        combined_mask = cv2.bitwise_or(upper_mask, cv2.bitwise_or(mid_mask, lower_mask))
        full_hist = self.calc_hsv_hist(roi_hsv, combined_mask)
        if full_hist is None:
            return None

        upper_h = self.calc_hsv_hist(roi_hsv, upper_mask)
        mid_h = self.calc_hsv_hist(roi_hsv, mid_mask)
        lower_h = self.calc_hsv_hist(roi_hsv, lower_mask)

        return PartFeatureBank(
            full=full_hist,
            upper=upper_h,
            mid=mid_h,
            lower=lower_h,
            visible_parts=(upper_h is not None, mid_h is not None, lower_h is not None),
            is_fallback=is_fallback,
        )


class PoseKeypointFeatureExtractor(BaseFeatureExtractor):
    """
    [Pose 태스크용] COCO 17개 관절 좌표 및 가시성(conf)을 기반으로
    실제 해부학적 부위와 골격 연결선(마커-마커 링크)을 마스킹해 추출하는 추출기
      - `pose_mode='baseline'`: 기존 몸통 사각형 + 허벅지/정강이 스트립
      - `pose_mode='expanded'`: 상완(5-7, 6-8)·측면 측면시(sh_w<12)·단측 어깨 근접 보완(폴백 대폭 감소)
      - `pose_mode='edges'`   : `expanded` 마스킹 + 각 마커 연결선별 개별 304D 히스토그램(`edges`) 비교
    """

    # COCO Keypoint 인덱스:
    # 5:L-Shoulder, 6:R-Shoulder, 7:L-Elbow, 8:R-Elbow, 9:L-Wrist, 10:R-Wrist,
    # 11:L-Hip, 12:R-Hip, 13:L-Knee, 14:R-Knee, 15:L-Ankle, 16:R-Ankle
    KP_CONF_THRESH = 0.35
    ALLOW_CROSS_PART = False

    def __init__(
        self,
        pose_mode: str = 'edges',
        edge_score_mode: Optional[str] = None,
        max_roi_dim: int = 160,
    ):
        self.pose_mode = pose_mode
        if edge_score_mode is not None:
            self.edge_score_mode = edge_score_mode
        else:
            self.edge_score_mode = 'max' if pose_mode == 'edges' else 'none'
        self.max_roi_dim = int(max_roi_dim)

    def _build_pose_masks(
        self,
        frame: np.ndarray,
        bbox: Tuple[int, int, int, int],
        keypoints: Optional[np.ndarray],
        exclude_boxes: Optional[List[Tuple[int, int, int, int]]] = None,
        collect_edges: bool = False,
        allow_downscale: bool = False,
    ) -> Optional[
        Tuple[
            Tuple[int, int, int, int],
            np.ndarray,
            np.ndarray,
            np.ndarray,
            np.ndarray,
            bool,
            Optional[Dict[str, np.ndarray]],
        ]
    ]:
        if keypoints is None or len(keypoints) < 17:
            fb = BBoxFeatureExtractor()._prepare_part_masks(
                frame, bbox, exclude_boxes=exclude_boxes
            )
            return (*fb[:5], True, None) if fb is not None else None

        h, w = frame.shape[:2]
        x1, y1, x2, y2 = (
            max(0, bbox[0]),
            max(0, bbox[1]),
            min(w, bbox[2]),
            min(h, bbox[3]),
        )
        bw, bh = x2 - x1, y2 - y1
        if bw < 12 or bh < 20:
            return None

        roi_bgr = frame[y1:y2, x1:x2]
        scale = 1.0
        rw, rh = bw, bh
        if (
            self.pose_mode != 'baseline'
            and allow_downscale
            and max(bw, bh) > self.max_roi_dim
        ):
            scale = float(self.max_roi_dim) / float(max(bw, bh))
            rw = max(12, int(round(bw * scale)))
            rh = max(20, int(round(bh * scale)))
            roi_bgr = cv2.resize(roi_bgr, (rw, rh), interpolation=cv2.INTER_AREA)

        roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
        valid_color = cv2.inRange(roi_hsv, np.array([0, 0, 15]), np.array([180, 255, 255]))
        if exclude_boxes:
            scaled_ex = [
                (
                    int(round(x1 + (ox1 - x1) * scale)),
                    int(round(y1 + (oy1 - y1) * scale)),
                    int(round(x1 + (ox2 - x1) * scale)),
                    int(round(y1 + (oy2 - y1) * scale)),
                )
                for ox1, oy1, ox2, oy2 in exclude_boxes
            ]
            valid_mask = self._apply_exclude_boxes(
                valid_color, (x1, y1, x1 + rw, y1 + rh), scaled_ex
            )
        else:
            valid_mask = valid_color

        kp = keypoints.copy()
        kp[:, 0] = np.clip((kp[:, 0] - x1) * scale, 0, rw - 1)
        kp[:, 1] = np.clip((kp[:, 1] - y1) * scale, 0, rh - 1)

        def is_vis(idx: int) -> bool:
            return float(kp[idx, 2]) >= self.KP_CONF_THRESH

        def pt(idx: int) -> Tuple[int, int]:
            return (int(kp[idx, 0]), int(kp[idx, 1]))

        expanded = self.pose_mode in ('expanded', 'edges')
        limb_thickness = max(8, int(rw * 0.24)) if expanded else max(6, int(rw * 0.22))
        arm_thickness = max(7, int(rw * 0.20))

        upper_mask = np.zeros((rh, rw), dtype=np.uint8)
        torso_mask = np.zeros((rh, rw), dtype=np.uint8) if collect_edges else None

        torso_pts = [pt(i) for i in (5, 6, 12, 11) if is_vis(i)]
        if len(torso_pts) >= 3:
            poly = np.array(torso_pts, dtype=np.int32)
            cv2.fillPoly(upper_mask, [poly], 255)
            if expanded:
                cv2.polylines(
                    upper_mask, [poly], True, 255, thickness=max(4, limb_thickness // 2)
                )
            if torso_mask is not None:
                torso_mask[:, :] = upper_mask
        elif (is_vis(5) or is_vis(6)) and (is_vis(11) or is_vis(12)):
            p_sh = pt(5) if is_vis(5) else pt(6)
            p_hip = pt(11) if is_vis(11) else pt(12)
            cv2.line(upper_mask, p_sh, p_hip, 255, thickness=limb_thickness * 2)
            if torso_mask is not None:
                torso_mask[:, :] = upper_mask
        elif is_vis(5) and is_vis(6):
            p5, p6 = pt(5), pt(6)
            sh_w = abs(p5[0] - p6[0])
            if expanded:
                # 측면 자세(sh_w < 12)에서도 최소 어깨 폭(0.34*rw)을 확보해 몸통 마스크 생성
                half_w = max(sh_w // 2, max(10, int(rw * 0.22)))
                x_mid = (p5[0] + p6[0]) // 2
                xl = max(0, x_mid - half_w)
                xr = min(rw - 1, x_mid + half_w)
                y_top = min(p5[1], p6[1])
                y_bot = min(rh - 1, int(max(p5[1], p6[1]) + max(18.0, sh_w * 1.15, rh * 0.45)))
                if y_bot > y_top + 2 and xr > xl + 2:
                    cv2.rectangle(upper_mask, (xl, y_top), (xr, y_bot), 255, -1)
                    if torso_mask is not None:
                        torso_mask[:, :] = upper_mask
            elif sh_w >= 12:
                y_bot = min(rh - 1, int(max(p5[1], p6[1]) + max(16.0, sh_w * 1.15)))
                if y_bot > max(p5[1], p6[1]) + 2:
                    chest_quad = np.array(
                        [p5, p6, (p6[0], y_bot), (p5[0], y_bot)], dtype=np.int32
                    )
                    cv2.fillPoly(upper_mask, [chest_quad], 255)
        elif expanded and (is_vis(5) or is_vis(6)):
            # 근접/측면으로 어깨 1개만 보이는 경우: 어깨에서 반대쪽 흉부 영역까지 폭넓게 상의 추출
            p_sh = pt(5) if is_vis(5) else pt(6)
            xl = max(0, min(p_sh[0], int(rw * 0.22)))
            xr = min(rw - 1, max(p_sh[0], int(rw * 0.78)))
            y_top = max(0, p_sh[1] - max(3, int(rh * 0.03)))
            y_bot = min(rh - 1, p_sh[1] + max(24, int(rh * 0.62)))
            if xr > xl + 4 and y_bot > y_top + 4:
                cv2.rectangle(upper_mask, (xl, y_top), (xr, y_bot), 255, -1)
                if torso_mask is not None:
                    torso_mask[:, :] = upper_mask

        if expanded:
            # 상완(소매: 5->7, 6->8)도 upper_mask에 포함해 상체 픽셀 커버리지·안정성 향상
            for sh_i, elb_i in ((5, 7), (6, 8)):
                if is_vis(sh_i) and is_vis(elb_i):
                    cv2.line(upper_mask, pt(sh_i), pt(elb_i), 255, thickness=arm_thickness)
            # 어깨·몸통 없이 전완(7->9, 8->10) 또는 양 팔꿈치(7, 8)만 극근접 관측된 예외 상황 보완
            if cv2.countNonZero(cv2.bitwise_and(upper_mask, valid_mask)) < 20:
                if is_vis(7) and is_vis(8):
                    p7, p8 = pt(7), pt(8)
                    xl = max(0, min(p7[0], p8[0]))
                    xr = min(rw - 1, max(p7[0], p8[0]))
                    yt = max(0, min(p7[1], p8[1]) - max(6, int(rh * 0.12)))
                    yb = min(rh - 1, max(p7[1], p8[1]) + max(10, int(rh * 0.16)))
                    if xr > xl + 8 and yb > yt + 8:
                        cv2.rectangle(upper_mask, (xl, yt), (xr, yb), 255, -1)
                        if torso_mask is not None:
                            torso_mask[:, :] = upper_mask
                for elb_i, wr_i in ((7, 9), (8, 10)):
                    if is_vis(elb_i) and is_vis(wr_i):
                        cv2.line(upper_mask, pt(elb_i), pt(wr_i), 255, thickness=arm_thickness)

        # 2. 중하반신(Mid/Thighs) 마스크: 골반(11->13, 12->14)에서 무릎까지 선분 스트립
        mid_mask = np.zeros((rh, rw), dtype=np.uint8)
        for hip_i, knee_i in ((11, 13), (12, 14)):
            if is_vis(hip_i) and is_vis(knee_i):
                cv2.line(mid_mask, pt(hip_i), pt(knee_i), 255, thickness=limb_thickness)

        # 3. 최하단(Lower/Calves & Shoes) 마스크: 무릎(13->15, 14->16)에서 발목까지 선분 스트립
        lower_mask = np.zeros((rh, rw), dtype=np.uint8)
        for knee_i, ank_i in ((13, 15), (14, 16)):
            if is_vis(knee_i) and is_vis(ank_i):
                cv2.line(lower_mask, pt(knee_i), pt(ank_i), 255, thickness=limb_thickness)
            elif is_vis(ank_i):
                cv2.circle(lower_mask, pt(ank_i), limb_thickness, 255, -1)

        upper_mask = cv2.bitwise_and(upper_mask, valid_mask)
        mid_mask = cv2.bitwise_and(mid_mask, valid_mask)
        lower_mask = cv2.bitwise_and(lower_mask, valid_mask)

        combined_mask = cv2.bitwise_or(upper_mask, cv2.bitwise_or(mid_mask, lower_mask))
        if cv2.countNonZero(combined_mask) < 20:
            fb = BBoxFeatureExtractor()._prepare_part_masks(
                frame, bbox, exclude_boxes=exclude_boxes
            )
            return (*fb[:5], True, None) if fb is not None else None

        edge_hists: Optional[Dict[str, np.ndarray]] = None
        if collect_edges:
            edge_hists = {}
            if torso_mask is not None:
                tm = cv2.bitwise_and(torso_mask, valid_mask)
                th = self.calc_hsv_hist(roi_hsv, tm)
                if th is not None:
                    edge_hists['torso'] = th

            link_specs = (
                ('shoulder', 5, 6, arm_thickness),
                ('l_side', 5, 11, arm_thickness),
                ('r_side', 6, 12, arm_thickness),
                ('hip', 11, 12, arm_thickness),
                ('l_uarm', 5, 7, arm_thickness),
                ('r_uarm', 6, 8, arm_thickness),
                ('l_farm', 7, 9, arm_thickness),
                ('r_farm', 8, 10, arm_thickness),
                ('l_thigh', 11, 13, limb_thickness),
                ('r_thigh', 12, 14, limb_thickness),
                ('l_calf', 13, 15, limb_thickness),
                ('r_calf', 14, 16, limb_thickness),
            )
            tmp_mask = np.empty((rh, rw), dtype=np.uint8)
            for edge_name, i1, i2, thick in link_specs:
                if is_vis(i1) and is_vis(i2):
                    tmp_mask.fill(0)
                    cv2.line(tmp_mask, pt(i1), pt(i2), 255, thickness=thick)
                    cv2.bitwise_and(tmp_mask, valid_mask, dst=tmp_mask)
                    if cv2.countNonZero(tmp_mask) >= 30:
                        eh = self.calc_hsv_hist(roi_hsv, tmp_mask)
                        if eh is not None:
                            edge_hists[edge_name] = eh

        return (x1, y1, x2, y2), roi_hsv, upper_mask, mid_mask, lower_mask, False, edge_hists

    def _prepare_part_masks(
        self,
        frame: np.ndarray,
        bbox: Tuple[int, int, int, int],
        mask_poly: Optional[np.ndarray] = None,
        keypoints: Optional[np.ndarray] = None,
        exclude_boxes: Optional[List[Tuple[int, int, int, int]]] = None,
    ) -> Optional[
        Tuple[Tuple[int, int, int, int], np.ndarray, np.ndarray, np.ndarray, np.ndarray, bool]
    ]:
        """YOLO-Pose 17관절 좌표를 로컬 ROI 좌표로 변환한 뒤 3개 해부학 부위별 마스크를 생성합니다."""
        built = self._build_pose_masks(
            frame,
            bbox,
            keypoints=keypoints,
            exclude_boxes=exclude_boxes,
            collect_edges=False,
            allow_downscale=False,
        )
        if built is None:
            return None
        return built[:6]

    def extract(
        self,
        frame: np.ndarray,
        bbox: Tuple[int, int, int, int],
        mask_poly: Optional[np.ndarray] = None,
        keypoints: Optional[np.ndarray] = None,
        exclude_boxes: Optional[List[Tuple[int, int, int, int]]] = None,
    ) -> Optional[PartFeatureBank]:
        collect_edges = (self.pose_mode == 'edges')
        built = self._build_pose_masks(
            frame,
            bbox,
            keypoints=keypoints,
            exclude_boxes=exclude_boxes,
            collect_edges=collect_edges,
            allow_downscale=(self.pose_mode != 'baseline'),
        )
        if built is None:
            return None
        _, roi_hsv, upper_mask, mid_mask, lower_mask, is_fallback, edge_hists = built
        combined_mask = cv2.bitwise_or(upper_mask, cv2.bitwise_or(mid_mask, lower_mask))
        full_hist = self.calc_hsv_hist(roi_hsv, combined_mask)
        if full_hist is None:
            return None

        upper_h = self.calc_hsv_hist(roi_hsv, upper_mask)
        mid_h = self.calc_hsv_hist(roi_hsv, mid_mask)
        lower_h = self.calc_hsv_hist(roi_hsv, lower_mask)

        return PartFeatureBank(
            full=full_hist,
            upper=upper_h,
            mid=mid_h,
            lower=lower_h,
            visible_parts=(upper_h is not None, mid_h is not None, lower_h is not None),
            is_fallback=is_fallback,
            edges=edge_hists,
        )
