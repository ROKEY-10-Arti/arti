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

from person_tracker.core.feature_extractors import PartFeatureBank, PoseKeypointFeatureExtractor
from person_tracker.core.multi_tracker_core import MultiModelTrackerCore, UnifiedTrackedPerson


DEFAULT_POSE_EDGES: Tuple[Tuple[int, int, Tuple[int, int, int]], ...] = (
    (5, 6, (255, 200, 50)),
    (5, 11, (255, 200, 50)),
    (6, 12, (255, 200, 50)),
    (11, 12, (255, 200, 50)),
    (5, 7, (255, 170, 50)),
    (7, 9, (255, 170, 50)),
    (6, 8, (255, 170, 50)),
    (8, 10, (255, 170, 50)),
    (11, 13, (0, 165, 255)),
    (12, 14, (0, 165, 255)),
    (13, 15, (255, 50, 255)),
    (14, 16, (255, 50, 255)),
)
DEFAULT_PART_MASK_COLORS = {
    'active': ((255, 200, 50), (0, 165, 255), (255, 50, 255)),
    'muted': ((170, 140, 70), (70, 120, 170), (160, 70, 160)),
}


def draw_pose_and_mask_overlays(
    vis: np.ndarray,
    src_frame: np.ndarray,
    detections: List[UnifiedTrackedPerson],
    tracker: Optional[MultiModelTrackerCore],
    mask_alpha: float = 0.34,
    kp_thresh: float = PoseKeypointFeatureExtractor.KP_CONF_THRESH,
    pose_edges=DEFAULT_POSE_EDGES,
    part_mask_colors=DEFAULT_PART_MASK_COLORS,
):
    """히스토그램 추출용 3부위 마스크(Det/Seg/Pose 공통) 반투명·외곽선 및 Pose 관절 마커를 그립니다."""
    if tracker is None or len(detections) == 0:
        return

    extractor = tracker.current_extractor
    in_guard = (
        tracker.mode == tracker.MODE_ADAPTIVE
        and tracker.metrics.adaptive_state == 'GUARD'
    )
    target_locked = tracker.target_id is not None

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
        palette = part_mask_colors['active' if (p.is_target or not target_locked) else 'muted']
        for part_mask, color in zip((upper_m, mid_m, lower_m), palette):
            if part_mask.any():
                sub[part_mask > 0] = color
                contour_items.append((rx1, ry1, part_mask, color))

    if contour_items:
        cv2.addWeighted(mask_overlay, mask_alpha, vis, 1.0 - mask_alpha, 0, vis)
        for rx1, ry1, part_mask, color in contour_items:
            contours, _ = cv2.findContours(part_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            valid_cnts = [c for c in contours if cv2.contourArea(c) >= 12.0]
            if valid_cnts:
                cv2.drawContours(
                    vis, valid_cnts, -1, color, 1, lineType=cv2.LINE_AA, offset=(rx1, ry1)
                )

    for p in detections:
        if p.keypoints is None or len(p.keypoints) < 17:
            continue
        kp = p.keypoints
        emph = p.is_target or not target_locked
        for i1, i2, edge_color in pose_edges:
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


def create_reid_trackbars(win_name: str, tracker_getter, defaults: Optional[dict] = None):
    """ReID 모니터 창에 Match% / Coex% / Margin% / ConfirmF 실시간 튜닝 트랙바를 생성합니다."""
    cv2.namedWindow(win_name)
    t0 = tracker_getter()
    d = defaults or {}
    m_val = int(round((t0.reid_match_thresh if t0 else float(d.get('reid_match_thresh', 0.50))) * 100))
    c_val = int(round((t0.coexist_min_sim if t0 else float(d.get('coexist_min_sim', 0.74))) * 100))
    g_val = int(round((t0.recover_margin if t0 else float(d.get('recover_margin', 0.06))) * 100))
    f_val = int(t0.recover_confirm_frames if t0 else int(d.get('recover_confirm_frames', 2)))

    def _on_match(v):
        t = tracker_getter()
        if t is not None:
            val = max(0.30, v / 100.0)
            t.reid_match_thresh = val
            t.search_min_sim = max(0.28, val - 0.02)

    def _on_coex(v):
        t = tracker_getter()
        if t is not None:
            t.coexist_min_sim = max(0.50, v / 100.0)

    def _on_margin(v):
        t = tracker_getter()
        if t is not None:
            t.recover_margin = max(0.0, v / 100.0)

    def _on_confirm(v):
        t = tracker_getter()
        if t is not None:
            t.recover_confirm_frames = max(1, int(v))

    cv2.createTrackbar('Match%', win_name, m_val, 80, _on_match)
    cv2.createTrackbar('Coex%', win_name, c_val, 95, _on_coex)
    cv2.createTrackbar('Margin%', win_name, g_val, 20, _on_margin)
    cv2.createTrackbar('ConfirmF', win_name, f_val, 5, _on_confirm)


def draw_model_mode_bars(
    frame: np.ndarray,
    tracker: Optional[MultiModelTrackerCore],
    y_top: int,
    row_h: int = 30,
) -> int:
    """모델 버전(Q/W/E) + 태스크(A/S/F) 1줄, 추적 알고리즘(1~5) 1줄을 그리고 하단 y 좌표를 반환합니다."""
    if tracker is None:
        return y_top
    w = frame.shape[1]
    slot_w = max(1, w // 6)
    family_keys = {1: 'Q', 2: 'W', 3: 'E'}
    task_keys = {1: 'A', 2: 'S', 3: 'F'}
    y1 = y_top + row_h

    for f_idx in (1, 2, 3):
        x1 = (f_idx - 1) * slot_w
        x2 = f_idx * slot_w
        is_act = (tracker.family_idx == f_idx)
        bg = (180, 100, 20) if is_act else (38, 38, 38)
        cv2.rectangle(frame, (x1, y_top), (x2, y1), bg, -1)
        cv2.rectangle(frame, (x1, y_top), (x2, y1), (85, 85, 85), 1)
        short_name = MultiModelTrackerCore.FAMILIES[f_idx][0]
        cv2.putText(
            frame,
            f'[{family_keys[f_idx]}] YOLO{short_name}',
            (x1 + 6, y_top + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (255, 255, 255) if is_act else (180, 180, 180),
            2 if is_act else 1,
            cv2.LINE_AA,
        )

    for t_idx in (1, 2, 3):
        slot = 3 + (t_idx - 1)
        x1 = slot * slot_w
        x2 = w if t_idx == 3 else (slot + 1) * slot_w
        is_act = (tracker.task_idx == t_idx)
        bg = (140, 50, 140) if is_act else (38, 38, 38)
        cv2.rectangle(frame, (x1, y_top), (x2, y1), bg, -1)
        cv2.rectangle(frame, (x1, y_top), (x2, y1), (85, 85, 85), 1)
        t_label = MultiModelTrackerCore.TASKS[t_idx][0]
        cv2.putText(
            frame,
            f'[{task_keys[t_idx]}] {t_label}',
            (x1 + 8, y_top + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.41,
            (255, 255, 255) if is_act else (180, 180, 180),
            2 if is_act else 1,
            cv2.LINE_AA,
        )

    y2 = y1 + row_h
    tab_w = max(1, w // 5)
    for m_idx in range(1, 6):
        x1 = (m_idx - 1) * tab_w
        x2 = w if m_idx == 5 else m_idx * tab_w
        is_act = (tracker.mode == m_idx)
        bg = (40, 155, 40) if is_act else (50, 50, 50)
        cv2.rectangle(frame, (x1, y1), (x2, y2), bg, -1)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (90, 90, 90), 1)
        cv2.putText(
            frame,
            MultiModelTrackerCore.MODE_NAMES[m_idx],
            (x1 + 6, y1 + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.40,
            (255, 255, 255) if is_act else (185, 185, 185),
            2 if is_act else 1,
            cv2.LINE_AA,
        )
    return y2


def handle_model_mode_click(
    x: int,
    y: int,
    frame_width: int,
    y_top: int,
    tracker: Optional[MultiModelTrackerCore],
    row_h: int = 30,
) -> Optional[str]:
    """draw_model_mode_bars 영역 클릭 시 모델·태스크·알고리즘을 전환하고 안내 문자열을 반환합니다."""
    if tracker is None or y < y_top or y > y_top + 2 * row_h:
        return None
    if y <= y_top + row_h:
        slot_w = max(1, frame_width // 6)
        slot_idx = min(5, max(0, x // slot_w))
        if slot_idx < 3:
            new_model = tracker.set_model(family_idx=slot_idx + 1)
            return f'[모델 버전 변경] -> {new_model}'
        new_model = tracker.set_model(task_idx=(slot_idx - 3) + 1)
        return f'[모델 태스크 변경] -> {new_model}'

    tab_w = max(1, frame_width // 5)
    clicked_mode = min(5, max(1, (x // tab_w) + 1))
    tracker.set_mode(clicked_mode)
    return f'[추적 알고리즘 변경] -> {MultiModelTrackerCore.MODE_NAMES[clicked_mode]}'


def handle_model_mode_key(
    key: int,
    tracker: Optional[MultiModelTrackerCore],
) -> Optional[str]:
    """Q/W/E, A/S/F, 1~5 단축키로 모델·태스크·알고리즘을 전환하고 로그 문구를 반환합니다."""
    if tracker is None:
        return None
    if key in (ord('1'), ord('2'), ord('3'), ord('4'), ord('5')):
        m_idx = int(chr(key))
        tracker.set_mode(m_idx)
        return f'[추적 알고리즘 변경] -> {MultiModelTrackerCore.MODE_NAMES[m_idx]}'
    family_map = {ord('q'): 1, ord('Q'): 1, ord('w'): 2, ord('W'): 2, ord('e'): 3, ord('E'): 3}
    if key in family_map:
        return f'[모델 버전 변경] -> {tracker.set_model(family_idx=family_map[key])}'
    task_map = {ord('a'): 1, ord('A'): 1, ord('s'): 2, ord('S'): 2, ord('f'): 3, ord('F'): 3}
    if key in task_map:
        return f'[모델 태스크 변경] -> {tracker.set_model(task_idx=task_map[key])}'
    return None


def render_benchmark_panel(
    tracker: Optional[MultiModelTrackerCore],
    cam_fps: float,
    total_ms: float,
    gesture_ms: float,
    cpu_percent: float,
    ram_mb: float,
    vram_mb: Optional[float],
    num_persons: int,
    cam_status,
    loc_res,
    bench_stats: dict,
    extra_tag: str = '',
) -> np.ndarray:
    """연산 속도·메모리·위치 추정 좌표 및 모델별 누적 비교표를 렌더링합니다."""
    lines: List[Tuple[str, Tuple[int, int, int]]] = []
    cur_key = ''
    if tracker is not None:
        m = tracker.metrics
        mode_name = MultiModelTrackerCore.MODE_NAMES.get(tracker.mode, str(tracker.mode))
        cur_key = f'{m.model_label} | {mode_name}'
        vram_str = f' | VRAM:{vram_mb:.0f}MB' if vram_mb is not None else ''
        adapt_str = f' [{m.adaptive_state} | EMA:{m.ema_status}]' if tracker.mode == 5 else ''
        lines.extend([
            (f'Model  : {m.model_label} | {mode_name} | {m.device}', (0, 255, 255)),
            (f'Keys   : Q/W/E=Ver  A/S/F=Task  1-5=Alg  M=Mask  V=ReID  B=Bench  C=Clr', (160, 210, 230)),
            (f'Cam FPS: {cam_fps:4.1f} | Max Infer: {m.max_infer_fps:5.1f} FPS{extra_tag}', (0, 255, 0)),
            (f'Total  : {total_ms:5.1f} ms | Gesture: {gesture_ms:4.1f} ms', (255, 255, 255)),
            (f' - YOLO ({m.task[:3]}) : {m.inference_ms:5.1f} ms (Pre:{m.preprocess_ms:3.1f})', (200, 220, 255)),
            (f' - Track+ReID : {m.post_track_ms + m.reid_ms:5.1f} ms (ReID:{m.reid_ms:3.1f})', (200, 220, 255)),
            (f'Sys CPU: {cpu_percent:4.1f}% | RAM:{ram_mb:.0f}MB{vram_str}', (180, 255, 180)),
            (f'Persons: {num_persons} | Target:{tracker.target_id}{adapt_str}', (255, 220, 120)),
        ])
    else:
        lines.extend([
            ('Model  : YOLO+ROI OFF (Hand-Only Mode)', (0, 220, 255)),
            (f'Cam FPS: {cam_fps:4.1f} | Total: {total_ms:5.1f} ms | Gesture: {gesture_ms:4.1f} ms', (255, 255, 255)),
            (f'Sys CPU: {cpu_percent:4.1f}% | RAM:{ram_mb:.0f}MB', (180, 255, 180)),
        ])

    if cam_status is not None and cam_status.map_xyz is not None:
        mx, my, _ = cam_status.map_xyz
        yaw_d = cam_status.map_yaw_deg or 0.0
        lines.append((
            f'Cam(Map): X={mx:+.2f}m Y={my:+.2f}m ({yaw_d:+.0f}deg) [{cam_status.map_source}] | Scan:{cam_status.lidar_hz:.1f}Hz',
            (220, 210, 255),
        ))
    elif cam_status is not None:
        lines.append((f'Cam TF: Waiting... | LiDAR Scan:{cam_status.lidar_hz:.1f}Hz', (200, 190, 230)))

    if loc_res is not None:
        bx, by, _ = loc_res.base_xyz
        sync_tag = f' {loc_res.sync_dt_ms:+.0f}ms' if loc_res.sync_dt_ms is not None else ''
        bot_color = (80, 255, 120) if loc_res.source == 'LiDAR' else (80, 220, 255)
        lines.append((
            f'Target(Bot): x={bx:+.2f}m y={by:+.2f}m ({loc_res.base_dist_m:.2f}m) [{loc_res.source}{sync_tag}]',
            bot_color,
        ))
        if loc_res.map_xyz is not None:
            tmx, tmy, _ = loc_res.map_xyz
            lines.append((f'Target(Map): X={tmx:+.2f}m Y={tmy:+.2f}m [{loc_res.map_source}]', (120, 255, 255)))
    else:
        lines.append(('Target(Bot): - (Gesture or Click to Lock)', (160, 160, 160)))

    bench_rows = list(bench_stats.items())[-10:]
    extra_h = (30 + len(bench_rows) * 19) if bench_rows else 0
    panel_h = max(260, 26 + len(lines) * 21 + extra_h)
    panel = np.full((panel_h, 620, 3), 24, dtype=np.uint8)

    for idx, (text, color) in enumerate(lines):
        cv2.putText(
            panel,
            text,
            (14, 22 + idx * 21),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.44,
            color,
            1,
            cv2.LINE_AA,
        )

    if bench_rows:
        y_tbl = 22 + len(lines) * 21 + 8
        cv2.line(panel, (12, y_tbl - 14), (608, y_tbl - 14), (70, 70, 70), 1)
        cv2.putText(
            panel,
            'Benchmark Compare (Q/W/E x A/S/F x 1-5) [C:Clear]',
            (14, y_tbl),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (255, 210, 100),
            1,
            cv2.LINE_AA,
        )
        for r_i, (lbl, (inf_ms, trk_ms, tot_ms, mf)) in enumerate(bench_rows):
            cur = (lbl == cur_key)
            row_txt = f'{"*" if cur else " "} {lbl:<26} Inf:{inf_ms:4.1f}ms Trk:{trk_ms:4.1f}ms Tot:{tot_ms:4.1f}ms ({mf:4.0f}FPS)'
            cv2.putText(
                panel,
                row_txt,
                (14, y_tbl + 19 + r_i * 19),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.39,
                (120, 255, 160) if cur else (195, 205, 215),
                1,
                cv2.LINE_AA,
            )
    return panel


def compose_dashboard_frame(
    main_vis: np.ndarray,
    reid_panel: Optional[np.ndarray] = None,
    bench_panel: Optional[np.ndarray] = None,
    max_w: int = 1680,
    max_h: int = 940,
) -> Tuple[np.ndarray, float]:
    """메인 영상(좌상), 벤치마크 지표(좌하), ReID 그래프·히트맵(우측)을 1개 대시보드 이미지로 합칩니다."""
    h_m, w_m = main_vis.shape[:2]

    if bench_panel is not None:
        h_b, w_b = bench_panel.shape[:2]
        if w_b > w_m:
            new_hb = max(1, int(round(h_b * (w_m / float(w_b)))))
            bench_fit = cv2.resize(bench_panel, (w_m, new_hb), interpolation=cv2.INTER_AREA)
        elif w_b < w_m:
            bench_fit = np.full((h_b, w_m, 3), 24, dtype=np.uint8)
            bench_fit[:, :w_b] = bench_panel
        else:
            bench_fit = bench_panel
        div_h = np.full((2, w_m, 3), 70, dtype=np.uint8)
        left_col = np.vstack((main_vis, div_h, bench_fit))
    else:
        left_col = main_vis

    if reid_panel is not None:
        h_l, w_l = left_col.shape[:2]
        h_r, w_r = reid_panel.shape[:2]
        h_tot = max(h_l, h_r)
        if h_l < h_tot:
            pad_l = np.full((h_tot - h_l, w_l, 3), 24, dtype=np.uint8)
            left_col = np.vstack((left_col, pad_l))
        if h_r < h_tot:
            pad_r = np.full((h_tot - h_r, w_r, 3), 24, dtype=np.uint8)
            right_col = np.vstack((reid_panel, pad_r))
        else:
            right_col = reid_panel
        div_v = np.full((h_tot, 2, 3), 70, dtype=np.uint8)
        canvas = np.hstack((left_col, div_v, right_col))
    else:
        canvas = left_col

    ch, cw = canvas.shape[:2]
    scale = 1.0
    if cw > max_w or ch > max_h:
        scale = min(max_w / float(cw), max_h / float(ch))
        nw = max(1, int(round(cw * scale)))
        nh = max(1, int(round(ch * scale)))
        canvas = cv2.resize(canvas, (nw, nh), interpolation=cv2.INTER_AREA)
    return canvas, scale


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
