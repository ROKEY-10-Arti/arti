"""
ReID 디버그 시각화 렌더러 (reid_visualizer.py)

[전체 렌더링 파이프라인 (`ReIDDebugVisualizer.render`)]
1. 시계열 이력 관리 (`sim_history`):
   - 타겟이 잠금(`target_id is not None and anchor_bank is not None`)되어 있는 동안
     매 프레임 각 트랙 ID의 `similarity`를 최대 `history_len`(120 프레임) 길이의 덱에 누적합니다.
2. 상단 상태 헤더 및 Search Status 출력:
   - 현재 Adaptive 상태(`IDLE`/`SAFE`/`GUARD`/`SEARCH`/`RECOVERED`), 타겟 ID, 실시간 임계값
     (`reid_match_thresh`, `coexist_min_sim`, `recover_margin`, `recover_confirm_frames`)과
     탐색 보류/확정 사유(`metrics.search_reason`)를 표시합니다.
3. 유사도 시계열 선 그래프:
   - 최근 120프레임 동안 타겟 및 경쟁 후보들의 유사도 추이를 그리고, `reid_match_thresh`/`coexist_min_sim` 수평 기준선을 표시합니다.
4. 인원별 부위 유사도 막대 (`Per-ID Part Similarity`):
   - 상위 3명(타겟 우선, 이후 유사도 내림차순)에 대해 `[TGT]`/`[COEX]`/`[CAND]` 태그와
     종합(`Tot`), 상의(`U`), 허벅지(`M`), 정강이(`L`), 전체(`F`) 유사도를 가로 막대로 출력합니다.
5. 304차원 HSV 벡터 히트맵 (`_render_hsv_vector_patch`):
   - 등록 시점 `Anchor`, 시간 적응 EMA `Recent`, 비교 대상(`Other` / `Target`) 각각을
     가로 1행 4부위(`Upper` / `Mid` / `Lower` / `Full`) 18x16 Hue-Saturation 컬러 히트맵 + 16-bin Value 막대로 대조합니다.
6. 하단 가이드 패널:
   - 상태 정의, 태그 조건 및 트랙바 튜닝 파라미터 설명을 고정 범례로 표시합니다.
"""

from collections import deque
from typing import Deque, Dict, List, Optional, Tuple

import cv2
import numpy as np

from person_tracker.feature_extractors import PartFeatureBank
from person_tracker.multi_tracker_core import MultiModelTrackerCore, UnifiedTrackedPerson


class ReIDDebugVisualizer:
    """
    ReID 유사도 시계열 그래프, 부위별 점수 막대, 304차원 HSV 벡터(18x16 H-S + 16 V) 히트맵을
    별도 OpenCV 창으로 실시간 시각화하는 디버그 렌더러.
    """

    PALETTE: Tuple[Tuple[int, int, int], ...] = (
        (80, 255, 100),   # 초록
        (0, 200, 255),    # 주황/노랑
        (255, 130, 80),   # 하늘
        (230, 100, 255),  # 보라
        (100, 140, 255),  # 코랄
    )

    def __init__(self, width: int = 720, height: int = 708, history_len: int = 120):
        self.width = width
        self.height = height
        self.history_len = history_len
        self.frame_idx = 0
        self.sim_history: Dict[int, Deque[Tuple[int, float]]] = {}

    def reset(self):
        self.sim_history.clear()
        self.frame_idx = 0

    def _color_for_id(self, track_id: int, is_target: bool) -> Tuple[int, int, int]:
        if is_target:
            return (80, 255, 100)
        return self.PALETTE[abs(track_id) % len(self.PALETTE)]

    @staticmethod
    def _render_hsv_vector_patch(vec304: Optional[np.ndarray], patch_w: int = 126, patch_h: int = 84) -> np.ndarray:
        """
        304차원 특징 벡터(0:288 -> 18x16 H-S, 288:304 -> 16 V)를
        실제 Hue-Saturation 컬러 히트맵 + 밝기(V) 스트립 이미지로 변환합니다.

        [시각화 원리]
        - 앞 288차원을 `(18 Hue, 16 Sat)` 분포 행렬로 복원하고 최댓값 기준 `sqrt` 감마 정규화로 작은 빈도 색상도 보이게 합니다.
        - 각 셀에 실제 `(Hue, Sat, Value=28+227*norm)` 색상을 채운 뒤 x축=Hue(0..180), y축=Sat(255..0)로 전치해 렌더링합니다.
        - 뒤 16차원은 무채색 밝기(V) 16단계 수평 막대로 우측 스트립에 표시합니다.
        """
        canvas = np.full((patch_h, patch_w + 26, 3), 26, dtype=np.uint8)
        if vec304 is None or len(vec304) < 304:
            cv2.putText(
                canvas,
                'N/A',
                (max(8, patch_w // 2 - 8), patch_h // 2 + 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (120, 120, 120),
                1,
            )
            return canvas

        hs = vec304[:288].reshape(18, 16)
        v_bins = vec304[288:304]

        hs_max = float(np.max(hs))
        hs_norm = np.sqrt(np.clip(hs / hs_max, 0.0, 1.0)) if hs_max > 1e-8 else np.zeros_like(hs)

        # (18 H rows x 16 S cols) -> 각 셀을 실제 (H, S, V) 컬러로 칠함
        cell_hsv = np.zeros((18, 16, 3), dtype=np.uint8)
        for h_i in range(18):
            hue_val = int((h_i + 0.5) * 10)  # OpenCV Hue: 0..179
            for s_i in range(16):
                sat_val = int((s_i + 0.5) * 16)
                val_bright = int(28 + 227 * hs_norm[h_i, s_i])
                cell_hsv[h_i, s_i] = (hue_val, min(255, sat_val), min(255, val_bright))

        hs_bgr = cv2.cvtColor(cell_hsv, cv2.COLOR_HSV2BGR)
        # x축: H(0..180), y축: S(255..0) 형태로 보기 쉽게 전치
        hs_bgr = np.transpose(hs_bgr, (1, 0, 2))[::-1, :, :]
        hs_resized = cv2.resize(hs_bgr, (patch_w, patch_h), interpolation=cv2.INTER_NEAREST)
        canvas[:, :patch_w] = hs_resized

        # 오른쪽 20px: V(밝기) 16개 빈 세로 막대 (아래=어두움, 위=밝음)
        v_max = float(np.max(v_bins))
        v_norm = np.clip(v_bins / v_max, 0.0, 1.0) if v_max > 1e-8 else np.zeros_like(v_bins)
        bar_x0 = patch_w + 6
        bin_h = max(1, patch_h // 16)
        for i in range(16):
            y_bot = patch_h - i * bin_h
            y_top = max(0, y_bot - bin_h + 1)
            fill_w = int(round(18 * v_norm[i]))
            gray = int(35 + 220 * (i / 15.0))
            if fill_w > 0:
                cv2.rectangle(canvas, (bar_x0, y_top), (bar_x0 + fill_w, y_bot - 1), (gray, gray, gray), -1)
        cv2.rectangle(canvas, (0, 0), (patch_w - 1, patch_h - 1), (80, 80, 80), 1)
        cv2.rectangle(canvas, (bar_x0, 0), (bar_x0 + 18, patch_h - 1), (80, 80, 80), 1)
        return canvas

    def render(
        self,
        tracker: MultiModelTrackerCore,
        detections: List[UnifiedTrackedPerson],
    ) -> np.ndarray:
        """
        현재 프레임의 시계열 유사도, 부위별 막대, 304D 벡터 비교 패널 이미지를 생성합니다.

        [구성 영역]
        1. 상단 헤더: Adaptive State, Target ID, 4대 임계 파라미터 및 `Search Status` 사유
        2. 120f 시계열 그래프: `reid_match_thresh` / `coexist_min_sim` 기준선과 ID별 유사도 곡선
        3. 중단 부위 막대: 상위 3인(`TGT`/`COEX`/`CAND`)의 `Tot / U / M / L / F` 점수 막대
        4. 하단 304D 히트맵: `Anchor` / `Recent (EMA)` / `Other(or Target)` 행별 4부위(`Upper`/`Mid`/`Lower`/`Full`) 대조
        5. 최하단 범례(Guide)
        """
        self.frame_idx += 1
        panel = np.full((self.height, self.width, 3), 20, dtype=np.uint8)

        # 활성 ID 기록 갱신 (타겟이 잠겨 있을 때만 의미 있는 similarity 기록; 미등록 상태에서는 그래프 초기화)
        locked = tracker.target_id is not None and tracker.anchor_bank is not None
        if locked:
            for p in detections:
                dq = self.sim_history.setdefault(p.track_id, deque(maxlen=self.history_len))
                dq.append((self.frame_idx, float(p.similarity)))
            # 그래프 윈도우(120프레임) 밖으로 사라진 오래된 트랙 ID 이력 정리
            stale = [
                tid for tid, dq in self.sim_history.items()
                if not dq or (self.frame_idx - dq[-1][0]) > self.history_len
            ]
            for tid in stale:
                del self.sim_history[tid]
        else:
            self.sim_history.clear()

        # 1. 상단 헤더 + EMA 상태 + SEARCH 보류/확정 사유
        status_txt = (
            f'State:{tracker.metrics.adaptive_state} Tgt:{tracker.target_id} | '
            f'EMA:{tracker.metrics.ema_status}(>={tracker.ema_min_sim:.2f},a={tracker.ema_alpha:.2f}) | '
            f'M:{tracker.reid_match_thresh:.2f} C:{tracker.coexist_min_sim:.2f} '
            f'G:{tracker.recover_margin:.2f} F:{tracker.recover_confirm_frames}f'
        )
        cv2.putText(panel, status_txt, (12, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 255), 1, cv2.LINE_AA)
        reason_txt = tracker.metrics.search_reason
        if reason_txt:
            r_col = (80, 255, 120) if reason_txt.startswith('RECOVERED') else (0, 200, 255)
            cv2.putText(panel, f'Search Status: {reason_txt}', (12, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.43, r_col, 1, cv2.LINE_AA)

        # 2. 시계열 선 그래프 (y=46..206)
        gx1, gy1, gx2, gy2 = 46, 46, self.width - 18, 204
        cv2.rectangle(panel, (gx1, gy1), (gx2, gy2), (32, 32, 32), -1)
        cv2.rectangle(panel, (gx1, gy1), (gx2, gy2), (75, 75, 75), 1)

        for val, lbl, col in (
            (0.25, '0.25', (65, 65, 65)),
            (tracker.reid_match_thresh, f'{tracker.reid_match_thresh:.2f}', (60, 170, 80)),
            (tracker.coexist_min_sim, f'{tracker.coexist_min_sim:.2f}', (60, 145, 220)),
        ):
            v_clip = max(0.0, min(1.0, float(val)))
            yy = int(gy2 - v_clip * (gy2 - gy1))
            cv2.line(panel, (gx1, yy), (gx2, yy), col, 1, cv2.LINE_AA)
            cv2.putText(panel, lbl, (6, yy + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1)

        if locked and self.sim_history:
            min_f = max(1, self.frame_idx - self.history_len + 1)
            span_f = max(1, self.history_len - 1)
            for tid, dq in sorted(self.sim_history.items()):
                is_tgt = (tid == tracker.target_id)
                col = self._color_for_id(tid, is_tgt)
                pts = []
                for f_i, s_val in dq:
                    px = int(gx1 + ((f_i - min_f) / span_f) * (gx2 - gx1))
                    py = int(gy2 - np.clip(s_val, 0.0, 1.0) * (gy2 - gy1))
                    pts.append((px, py))
                for i in range(1, len(pts)):
                    if dq[i][0] - dq[i - 1][0] <= 3:
                        cv2.line(panel, pts[i - 1], pts[i], col, 2 if is_tgt else 1, cv2.LINE_AA)
                if pts:
                    cv2.circle(panel, pts[-1], 3, col, -1)
        else:
            cv2.putText(
                panel,
                'Lock a target (Fist -> Open) to plot live ReID curves',
                (gx1 + 110, (gy1 + gy2) // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (140, 140, 140),
                1,
                cv2.LINE_AA,
            )

        # 3. 중단: 인원별 부위 유사도 막대 (y=218..365)
        cv2.putText(
            panel,
            'Per-ID Part Similarity (Total / Upper / Mid / Lower / Full)',
            (12, 226),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            (210, 220, 255),
            1,
            cv2.LINE_AA,
        )
        sorted_dets = sorted(detections, key=lambda p: (not p.is_target, -p.similarity))
        row_y = 240
        for p in sorted_dets[:3]:
            col = self._color_for_id(p.track_id, p.is_target)
            tag = 'TGT' if p.is_target else ('COEX' if p.track_id in tracker.coexisting_non_target_ids else 'CAND')
            cv2.putText(
                panel,
                f'ID:{p.track_id} [{tag}]',
                (12, row_y + 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                col,
                1,
                cv2.LINE_AA,
            )
            u_s, m_s, l_s, f_s = p.part_sims
            metrics = [
                ('Tot', p.similarity),
                ('U', u_s),
                ('M', m_s),
                ('L', l_s),
                ('F', f_s),
            ]
            bx = 128
            for name, val in metrics:
                bw = 104
                cv2.rectangle(panel, (bx, row_y + 3), (bx + bw, row_y + 18), (42, 42, 42), -1)
                if val is not None:
                    fw = int(bw * max(0.0, min(1.0, val)))
                    bar_col = (60, 230, 90) if val >= tracker.reid_match_thresh else (60, 150, 230)
                    if fw > 0:
                        cv2.rectangle(panel, (bx, row_y + 3), (bx + fw, row_y + 18), bar_col, -1)
                    txt = f'{name}:{val:.2f}'
                else:
                    txt = f'{name}:--'
                cv2.rectangle(panel, (bx, row_y + 3), (bx + bw, row_y + 18), (85, 85, 85), 1)
                cv2.putText(panel, txt, (bx + 4, row_y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (245, 245, 245), 1)
                bx += bw + 10
            row_y += 34

        # 4. 하단: 304D HSV 벡터(18x16 H-S + 16 V) 1행 4부위(Upper / Mid / Lower / Full) 비교 (y=344..602)
        anchor_bank: Optional[PartFeatureBank] = tracker.anchor_bank
        recent_bank: Optional[PartFeatureBank] = tracker.recent_bank
        compare_person: Optional[UnifiedTrackedPerson] = None
        has_other = False
        if sorted_dets:
            non_targets = [p for p in sorted_dets if not p.is_target and p.feature_bank is not None]
            if non_targets:
                compare_person = non_targets[0]
                has_other = True
            else:
                compare_person = sorted_dets[0]

        cv2.putText(
            panel,
            '304D HSV Heatmap (18x16 H-S + 16 V)',
            (12, 354),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            (210, 220, 255),
            1,
            cv2.LINE_AA,
        )
        if compare_person is not None and compare_person.feature_bank is not None:
            sim_col = (80, 255, 120) if compare_person.is_target else (0, 200, 255)
            cv2.putText(
                panel,
                f'Total Sim: {compare_person.similarity:.3f}',
                (390, 354),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.46,
                sim_col,
                1,
                cv2.LINE_AA,
            )
        if anchor_bank is not None and recent_bank is not None:
            ar_sim = tracker.current_extractor.hist_sim(anchor_bank.full, recent_bank.full)
            cv2.putText(
                panel,
                f'A-R Sim: {ar_sim:.3f}',
                (556, 354),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.44,
                (190, 220, 255),
                1,
                cv2.LINE_AA,
            )

        col_xs = (120, 268, 416, 564)
        col_titles = ('Upper', 'Mid', 'Lower', 'Full')
        for cx, c_title in zip(col_xs, col_titles):
            cv2.putText(panel, c_title, (cx, 373), cv2.FONT_HERSHEY_SIMPLEX, 0.39, (215, 225, 235), 1, cv2.LINE_AA)

        def _render_bank_row(
            row_label: str,
            sub_label: str,
            label_color: Tuple[int, int, int],
            bank: Optional[PartFeatureBank],
            y0: int,
        ):
            cv2.putText(panel, row_label, (12, y0 + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.41, label_color, 1, cv2.LINE_AA)
            if sub_label:
                cv2.putText(panel, sub_label, (12, y0 + 44), cv2.FONT_HERSHEY_SIMPLEX, 0.36, (175, 185, 195), 1, cv2.LINE_AA)
            vecs = (
                (bank.upper, bank.mid, bank.lower, bank.full)
                if bank is not None
                else (None, None, None, None)
            )
            for cx, vec in zip(col_xs, vecs):
                patch = self._render_hsv_vector_patch(vec, patch_w=108, patch_h=64)
                ph, pw = patch.shape[:2]
                panel[y0:y0 + ph, cx:cx + pw] = patch

        _render_bank_row('Anchor', '(Registered)', (120, 235, 255), anchor_bank, 380)
        _render_bank_row('Recent', '(EMA)', (140, 255, 180), recent_bank, 456)

        if compare_person is not None and compare_person.feature_bank is not None:
            if has_other:
                cmp_tag = 'COEX' if compare_person.track_id in tracker.coexisting_non_target_ids else 'CAND'
                cmp_title = f'Other ID:{compare_person.track_id}'
                cmp_sub = f'[{cmp_tag}] {compare_person.similarity:.2f}'
                cmp_col = self._color_for_id(compare_person.track_id, False)
            else:
                cmp_title = f'Target ID:{compare_person.track_id}'
                cmp_sub = '(Current)'
                cmp_col = (80, 255, 100)
            _render_bank_row(cmp_title, cmp_sub, cmp_col, compare_person.feature_bank, 532)
        else:
            _render_bank_row('Other', '(None)', (130, 130, 130), None, 532)

        # 5. 하단 지표 안내 (Legend / Guide)
        cv2.rectangle(panel, (10, 614), (self.width - 10, self.height - 8), (28, 28, 28), -1)
        cv2.rectangle(panel, (10, 614), (self.width - 10, self.height - 8), (65, 65, 65), 1)
        guide_lines = [
            'Guide | State: SAFE(locked) / GUARD(crowded) / SEARCH(lost) / RECOVERED',
            'Tags  | [TGT]=Target  [COEX]=Separate stranger (needs >=Coex%)  [CAND]=New/returning (needs >=Match%)',
            'Bars  | Tot=Combined  U=Upper(Torso)  M=Mid(Thighs)  L=Lower(Calves)  F=Full | 304D=18x16 H-S + 16 V',
            'Tune  | Sliders above: Match% (re-lock), Coex% (stranger block), Margin% (1st-2nd gap), ConfirmF (frames)',
        ]
        for idx, g_txt in enumerate(guide_lines):
            cv2.putText(
                panel,
                g_txt,
                (16, 632 + idx * 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.37,
                (185, 200, 210),
                1,
                cv2.LINE_AA,
            )

        return panel
