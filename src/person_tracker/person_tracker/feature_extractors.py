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

from dataclasses import dataclass
from typing import List, Optional, Tuple
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
    """
    full: np.ndarray
    upper: Optional[np.ndarray]
    mid: Optional[np.ndarray]
    lower: Optional[np.ndarray]
    visible_parts: Tuple[bool, bool, bool] = (True, True, True)
    is_fallback: bool = False

    def copy(self) -> 'PartFeatureBank':
        return PartFeatureBank(
            full=self.full.copy(),
            upper=self.upper.copy() if self.upper is not None else None,
            mid=self.mid.copy() if self.mid is not None else None,
            lower=self.lower.copy() if self.lower is not None else None,
            visible_parts=self.visible_parts,
            is_fallback=self.is_fallback,
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

    def compare_banks_detail(
        self, ref: Optional[PartFeatureBank], cur: Optional[PartFeatureBank]
    ) -> Tuple[float, Optional[float], Optional[float], Optional[float], float]:
        """
        부위별 유사도와 최종 종합 점수를 함께 반환합니다:
        (total_score, upper_sim, mid_sim, lower_sim, full_sim)

        [비교 원리]
        - 기준(`ref`)과 현재(`cur`) 모두 추출된 부위끼리만 1:1 유사도(`upper`, `mid`, `lower`)를 계산하고,
          가용 부위 가중치(상의 0.44, 허벅지 0.36, 정강이 0.20)로 `aligned_score`를 산출합니다.
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
    """[Segment 태스크용] 폴리곤 실루엣 마스킹(배경 0%) + 3부위 특징 추출기"""

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
        Segment 폴리곤 마스크를 3x3 커널로 1회 침식해 경계 배경을 덜어낸 뒤
        높이 비율(0.35 / 0.70) 기준 상·중·하 부위별 마스크를 생성합니다.
        """
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

        roi_hsv = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
        seg_mask = np.zeros((bh, bw), dtype=np.uint8)

        if mask_poly is not None and len(mask_poly) >= 3:
            local_poly = mask_poly.astype(np.int32).copy()
            local_poly[:, 0] = np.clip(local_poly[:, 0] - x1, 0, bw - 1)
            local_poly[:, 1] = np.clip(local_poly[:, 1] - y1, 0, bh - 1)
            cv2.fillPoly(seg_mask, [local_poly], 255)
            if bw > 24 and bh > 40:
                seg_mask = cv2.erode(seg_mask, np.ones((3, 3), np.uint8), iterations=1)
        else:
            seg_mask[int(bh * 0.1):int(bh * 0.9), int(bw * 0.15):int(bw * 0.85)] = 255

        valid_color = cv2.inRange(roi_hsv, np.array([0, 0, 15]), np.array([180, 255, 255]))
        final_mask = cv2.bitwise_and(seg_mask, valid_color)
        if cv2.countNonZero(final_mask) < 20:
            final_mask = seg_mask
        final_mask = self._apply_exclude_boxes(final_mask, (x1, y1, x2, y2), exclude_boxes)
        if cv2.countNonZero(final_mask) < 12:
            return None

        upper_m, mid_m, lower_m = self._split_mask_by_ratios(final_mask, 0.35, 0.70)
        return (x1, y1, x2, y2), roi_hsv, upper_m, mid_m, lower_m, False


class PoseKeypointFeatureExtractor(BaseFeatureExtractor):
    """
    [Pose 태스크용] COCO 17개 관절 좌표 및 가시성(conf)을 기반으로
    실제 해부학적 부위(어깨~골반=상의, 골반~무릎=허벅지, 무릎~발목=정강이)만 정확히 마스킹해 추출하는 추출기
    """

    # COCO Keypoint 인덱스:
    # 5:L-Shoulder, 6:R-Shoulder, 11:L-Hip, 12:R-Hip, 13:L-Knee, 14:R-Knee, 15:L-Ankle, 16:R-Ankle
    KP_CONF_THRESH = 0.35
    ALLOW_CROSS_PART = False

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
        YOLO-Pose 17관절 좌표를 로컬 ROI 좌표로 변환한 뒤 3개 해부학 부위별 마스크를 생성합니다.

        [마스킹 원리]
        - `upper_mask`: 어깨(5,6)-골반(11,12) 3점 이상 가시 시 사각형 다각형 채우기, 한쪽만 보이면 두꺼운 골격선.
        - `mid_mask`: 골반->무릎(11->13, 12->14) 선분 스트립 (`thickness = max(6, bw*0.22)`).
        - `lower_mask`: 무릎->발목(13->15, 14->16) 선분 또는 발목 원형 패치.
        - `valid_color`(V>=15) 및 `exclude_boxes`(타인 BBox 제외)를 적용해 배경과 교차 오염을 제거하고,
          관절이 너무 적으면 `BBoxFeatureExtractor` 중앙 크롭 방식으로 폴백합니다.
        """
        if keypoints is None or len(keypoints) < 17:
            fb = BBoxFeatureExtractor()._prepare_part_masks(
                frame, bbox, exclude_boxes=exclude_boxes
            )
            return (*fb[:5], True) if fb is not None else None

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

        roi_hsv = cv2.cvtColor(frame[y1:y2, x1:x2], cv2.COLOR_BGR2HSV)
        valid_color = cv2.inRange(roi_hsv, np.array([0, 0, 15]), np.array([180, 255, 255]))

        # 로컬 ROI 좌표로 변환된 키포인트 (17, 3) -> [lx, ly, conf]
        kp = keypoints.copy()
        kp[:, 0] = np.clip(kp[:, 0] - x1, 0, bw - 1)
        kp[:, 1] = np.clip(kp[:, 1] - y1, 0, bh - 1)

        def is_vis(idx: int) -> bool:
            return float(kp[idx, 2]) >= self.KP_CONF_THRESH

        def pt(idx: int) -> Tuple[int, int]:
            return (int(kp[idx, 0]), int(kp[idx, 1]))

        limb_thickness = max(6, int(bw * 0.22))

        # 1. 상의(Upper/Torso) 마스크: 어깨(5,6)와 골반(11,12)을 잇는 몸통 다각형 또는 두꺼운 골격선
        upper_mask = np.zeros((bh, bw), dtype=np.uint8)
        torso_pts = [pt(i) for i in (5, 6, 12, 11) if is_vis(i)]
        if len(torso_pts) >= 3:
            cv2.fillPoly(upper_mask, [np.array(torso_pts, dtype=np.int32)], 255)
        elif (is_vis(5) or is_vis(6)) and (is_vis(11) or is_vis(12)):
            p_sh = pt(5) if is_vis(5) else pt(6)
            p_hip = pt(11) if is_vis(11) else pt(12)
            cv2.line(upper_mask, p_sh, p_hip, 255, thickness=limb_thickness * 2)
        elif is_vis(5) and is_vis(6):
            # 근접 촬영 등으로 골반(11,12)이 화면 아래로 잘렸을 때 어깨선 아래 상의 영역만 추출 (얼굴 제외)
            p5, p6 = pt(5), pt(6)
            sh_w = abs(p5[0] - p6[0])
            if sh_w >= 12:
                y_bot = min(bh - 1, int(max(p5[1], p6[1]) + max(16.0, sh_w * 1.15)))
                if y_bot > max(p5[1], p6[1]) + 2:
                    chest_quad = np.array(
                        [p5, p6, (p6[0], y_bot), (p5[0], y_bot)], dtype=np.int32
                    )
                    cv2.fillPoly(upper_mask, [chest_quad], 255)

        # 2. 중하반신(Mid/Thighs) 마스크: 골반(11->13, 12->14)에서 무릎까지 선분 스트립
        mid_mask = np.zeros((bh, bw), dtype=np.uint8)
        for hip_i, knee_i in ((11, 13), (12, 14)):
            if is_vis(hip_i) and is_vis(knee_i):
                cv2.line(mid_mask, pt(hip_i), pt(knee_i), 255, thickness=limb_thickness)

        # 3. 최하단(Lower/Calves & Shoes) 마스크: 무릎(13->15, 14->16)에서 발목까지 선분 스트립
        lower_mask = np.zeros((bh, bw), dtype=np.uint8)
        for knee_i, ank_i in ((13, 15), (14, 16)):
            if is_vis(knee_i) and is_vis(ank_i):
                cv2.line(lower_mask, pt(knee_i), pt(ank_i), 255, thickness=limb_thickness)
            elif is_vis(ank_i):
                cv2.circle(lower_mask, pt(ank_i), limb_thickness, 255, -1)

        upper_mask = self._apply_exclude_boxes(
            cv2.bitwise_and(upper_mask, valid_color), (x1, y1, x2, y2), exclude_boxes
        )
        mid_mask = self._apply_exclude_boxes(
            cv2.bitwise_and(mid_mask, valid_color), (x1, y1, x2, y2), exclude_boxes
        )
        lower_mask = self._apply_exclude_boxes(
            cv2.bitwise_and(lower_mask, valid_color), (x1, y1, x2, y2), exclude_boxes
        )

        # 전체(Full) 마스크는 관절로 확인된 모든 부위의 합집합
        combined_mask = cv2.bitwise_or(upper_mask, cv2.bitwise_or(mid_mask, lower_mask))
        if cv2.countNonZero(combined_mask) < 20:
            # 관절이 너무 안 보일 때는 중앙 영역 폴백 (`is_fallback=True` 표시로 이후 Pose 승격 허용)
            fb = BBoxFeatureExtractor()._prepare_part_masks(
                frame, bbox, exclude_boxes=exclude_boxes
            )
            return (*fb[:5], True) if fb is not None else None

        return (x1, y1, x2, y2), roi_hsv, upper_mask, mid_mask, lower_mask, False
