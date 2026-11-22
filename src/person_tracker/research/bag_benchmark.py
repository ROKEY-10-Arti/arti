#!/usr/bin/env python3
"""
ROS 2 Bag 기반 person_tracker 자동 벤치마크 · 시계열 · 그래프 · 키프레임 스냅샷 추출 도구

기능:
  1. 1개 또는 여러 개의 ROS 2 Bag(.mcap / .db3) 폴더를 지정해 오프라인 고속 비교
  2. 모델 버전(v8n/11n/26n) × 태스크(Det/Seg/Pose) 속도 분해(Mean/P95) · 부위 가용성 · 폴백률 · 진성 유지율 정량 비교
  3. 추적 알고리즘(1:ByteTrack ~ 5:Adaptive) True Lock / 오인식(Hijack) / 재등장 복구 성능 비교
  4. 3D LiDAR Localizer (Raw vs Kinematic+MapFilter+시간창 EMA) 전/후 거리·튐 비교
  5. 프레임별 전체 지표 시계열 CSV(timeseries/*.csv) 및 그래프 7종 생성
  6. 동일 프레임 [마스크 + 304D HSV 히스토그램], [실제 추출 픽셀(배경 제거)], [버전 비교], [전·중·후 3프레임 시퀀스] 스냅샷 자동 저장

사용 예시:
  python3 src/person_tracker/research/bag_benchmark.py \
      --bags ~/bag/model_benchmark ~/bag/complete_bag/test_full \
      --models quick --modes 1,3,5 --reid-scale
"""

import argparse
from collections import deque
import csv
import datetime
import math
import os
import re
import sys
import time
from typing import Dict, List, Optional, Tuple

# 실제 로봇 네트워크와 절대 충돌하지 않도록 오프라인 격리 환경 기본 적용
os.environ.pop('ROS_DISCOVERY_SERVER', None)
os.environ.pop('ROS_SUPER_CLIENT', None)
os.environ.setdefault('ROS_DOMAIN_ID', '98')
os.environ.setdefault('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')

# 패키지 경로 및 venv 자동 부트스트랩
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.dirname(_THIS_DIR)
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)

from person_tracker.core.env_bootstrap import bootstrap_venv
bootstrap_venv()

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions

from person_tracker.core.lidar_person_localizer import LidarPersonLocalizer
from person_tracker.core.multi_tracker_core import MultiModelTrackerCore, UnifiedTrackedPerson
from person_tracker.debug.reid_visualizer import (
    ReIDDebugVisualizer,
    compose_dashboard_frame,
    draw_pose_and_mask_overlays,
    render_benchmark_panel,
)


MODEL_SPECS: Dict[str, Tuple[int, int]] = {
    'v8n-det': (1, 1),
    'v8n-seg': (1, 2),
    'v8n-pose': (1, 3),
    '11n-det': (2, 1),
    '11n-seg': (2, 2),
    '11n-pose': (2, 3),
    '26n-det': (3, 1),
    '26n-seg': (3, 2),
    '26n-pose': (3, 3),
}

PRESETS: Dict[str, List[str]] = {
    'quick': ['v8n-pose', '11n-pose', '26n-det', '26n-seg', '26n-pose'],
    'versions': ['v8n-pose', '11n-pose', '26n-pose'],
    'tasks': ['26n-det', '26n-seg', '26n-pose'],
    'all9': list(MODEL_SPECS.keys()),
}

REID_SCALE_KWARGS = {
    'ema_min_sim': 0.52,
    'ema_alpha': 0.08,
    'search_min_sim': 0.44,
    'reid_match_thresh': 0.46,
}

TASK_COMPARE_MODELS = ('26n-det', '26n-seg', '26n-pose')
VERSION_COMPARE_MODELS = ('v8n-pose', '11n-pose', '26n-pose')
HIJACK_SIM_THRESH = 0.40


def _fmt_opt(val: Optional[float], digits: int = 2) -> str:
    if val is None or (isinstance(val, float) and math.isnan(val)):
        return '--'
    return f'{val:.{digits}f}'


def _round_opt(val: Optional[float], digits: int = 4):
    if val is None or (isinstance(val, float) and math.isnan(val)):
        return ''
    return round(float(val), digits)


def _safe_filename(name: str) -> str:
    return re.sub(r'[^a-zA-Z0-9._-]+', '_', name).strip('_')


def _bbox_iou(box_a: Tuple[int, int, int, int], box_b: Tuple[int, int, int, int]) -> float:
    ax1, ay1, ax2, ay2 = box_a
    bx1, by1, bx2, by2 = box_b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw = max(0, ix2 - ix1)
    ih = max(0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0:
        return 0.0
    area_a = max(1, (ax2 - ax1) * (ay2 - ay1))
    area_b = max(1, (bx2 - bx1) * (by2 - by1))
    return float(inter) / float(area_a + area_b - inter)


def _load_gt_csv(path: str) -> Dict[int, dict]:
    """gt_annotator.py가 생성한 GT CSV를 frame_idx -> dict 매핑으로 로드합니다."""
    gt_map: Dict[int, dict] = {}
    with open(path, 'r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            src = row.get('source', '') or ''
            if src == 'unlabeled':
                continue
            fidx = int(row['frame_idx'])
            pres = int(row['gt_present'])
            bbox = None
            if pres == 1 and row.get('x1', '') != '':
                bbox = (
                    int(float(row['x1'])),
                    int(float(row['y1'])),
                    int(float(row['x2'])),
                    int(float(row['y2'])),
                )
            gt_map[fidx] = {
                'present': pres,
                'bbox': bbox,
                'corrected': int(row.get('corrected', 0) or 0),
                'source': src,
            }
    return gt_map


def _detect_storage_id(bag_dir: str) -> str:
    meta_path = os.path.join(bag_dir, 'metadata.yaml')
    if os.path.isfile(meta_path):
        try:
            with open(meta_path, 'r', encoding='utf-8') as f:
                meta = yaml.safe_load(f) or {}
            sid = meta.get('rosbag2_bagfile_information', {}).get('storage_identifier', '')
            if sid:
                return str(sid)
        except Exception:
            pass
    for fn in os.listdir(bag_dir):
        if fn.endswith('.mcap'):
            return 'mcap'
        if fn.endswith('.db3'):
            return 'sqlite3'
    return 'mcap'


def _auto_pick_topics(bag_dir: str, storage_id: str, custom_image_topic: str = ''):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag_dir, storage_id=storage_id), ConverterOptions('', ''))
    topics_info = r.get_all_topics_and_types()
    tmap = {t.name: t.type for t in topics_info}

    img_topic = custom_image_topic.strip()
    if not img_topic:
        priorities = [
            lambda name, typ: 'top_cam' in name and 'CompressedImage' in typ,
            lambda name, typ: 'oakd/rgb/image_raw/compressed' in name and 'CompressedImage' in typ,
            lambda name, typ: 'oakd/rgb/preview/image_raw/compressed' in name and 'CompressedImage' in typ,
            lambda name, typ: 'image_raw/compressed' in name and 'CompressedImage' in typ,
            lambda name, typ: 'image_raw' in name and typ.endswith('/Image'),
        ]
        for pred in priorities:
            cands = [n for n, tp in tmap.items() if pred(n, tp)]
            if cands:
                img_topic = sorted(cands)[0]
                break

    if not img_topic or img_topic not in tmap:
        raise RuntimeError(f'이미지 토픽을 찾지 못했습니다 ({bag_dir}). 사용 가능: {list(tmap.keys())}')

    parts = [p for p in img_topic.split('/') if p]
    ns = '/' + parts[0] if (len(parts) >= 2 and parts[0].startswith('robot')) else ''
    scan_topic = f'{ns}/scan' if f'{ns}/scan' in tmap else ('/scan' if '/scan' in tmap else '')
    tf_topic = f'{ns}/tf' if f'{ns}/tf' in tmap else ('/tf' if '/tf' in tmap else '')
    tf_static_topic = (
        f'{ns}/tf_static' if f'{ns}/tf_static' in tmap else ('/tf_static' if '/tf_static' in tmap else '')
    )
    return {
        'ns': ns or '/robot1',
        'image_topic': img_topic,
        'image_type': tmap[img_topic],
        'scan_topic': scan_topic,
        'tf_topic': tf_topic,
        'tf_static_topic': tf_static_topic,
    }


def _load_bag_stream(
    bag_dir: str,
    storage_id: str,
    topics: dict,
    max_frames: int = 0,
    stride: int = 1,
):
    """Bag의 TF/Scan/Image 이벤트를 순서대로 메모리에 로드합니다 (이미지는 압축 바이트로 유지)."""
    r = SequentialReader()
    r.open(StorageOptions(uri=bag_dir, storage_id=storage_id), ConverterOptions('', ''))
    msg_types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}

    wanted = {
        topics['image_topic'],
        topics['scan_topic'],
        topics['tf_topic'],
        topics['tf_static_topic'],
    }
    wanted.discard('')

    events = []
    img_raw_count = 0
    img_kept = 0
    t0_ns: Optional[int] = None

    while r.has_next():
        topic, data, t_ns = r.read_next()
        if topic not in wanted:
            continue
        if t0_ns is None:
            t0_ns = t_ns
        rel_sec = (t_ns - t0_ns) * 1e-9

        if topic == topics['image_topic']:
            img_raw_count += 1
            if stride > 1 and ((img_raw_count - 1) % stride != 0):
                continue
            msg = deserialize_message(data, msg_types[topic])
            events.append(('image', rel_sec, msg))
            img_kept += 1
            if max_frames > 0 and img_kept >= max_frames:
                break
        elif topic == topics['scan_topic']:
            events.append(('scan', rel_sec, deserialize_message(data, msg_types[topic])))
        elif topic == topics['tf_topic']:
            events.append(('tf', rel_sec, deserialize_message(data, msg_types[topic])))
        elif topic == topics['tf_static_topic']:
            events.append(('tf_static', rel_sec, deserialize_message(data, msg_types[topic])))

    return events, img_kept


def _decode_image(msg, is_compressed: bool) -> Optional[np.ndarray]:
    if is_compressed:
        arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)
    h, w = int(msg.height), int(msg.width)
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    enc = str(msg.encoding).lower()
    if 'rgb8' in enc:
        return cv2.cvtColor(buf.reshape((h, w, 3)), cv2.COLOR_RGB2BGR)
    if 'bgr8' in enc:
        return buf.reshape((h, w, 3)).copy()
    return None


def _draw_main_overlay(
    frame: np.ndarray,
    tracker: MultiModelTrackerCore,
    detections: List[UnifiedTrackedPerson],
    loc_res,
    label_header: str,
    sub_header: str = '',
) -> np.ndarray:
    vis = frame.copy()
    if tracker.current_task_name == 'segment':
        for p in detections:
            if p.mask_poly is not None and len(p.mask_poly) >= 3:
                cv2.polylines(
                    vis,
                    [p.mask_poly.astype(np.int32)],
                    isClosed=True,
                    color=(0, 230, 80) if p.is_target else (140, 120, 90),
                    thickness=2 if p.is_target else 1,
                )
    draw_pose_and_mask_overlays(vis, frame, detections, tracker)
    for p in detections:
        x1, y1, x2, y2 = p.bbox
        if p.is_target:
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                vis,
                f'TGT ID:{p.track_id} sim:{p.similarity:.2f}',
                (x1, max(48, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.50,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            if loc_res is not None:
                cv2.putText(
                    vis,
                    f'Dist:{loc_res.base_dist_m:.2f}m [{loc_res.source}]',
                    (x1, min(vis.shape[0] - 8, y2 + 16)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.44,
                    (50, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
        else:
            cv2.rectangle(vis, (x1, y1), (x2, y2), (170, 170, 170), 1)
            cv2.putText(
                vis,
                f'ID:{p.track_id} sim:{p.similarity:.2f}',
                (x1, max(48, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (180, 180, 180),
                1,
                cv2.LINE_AA,
            )
    banner_h = 44 if sub_header else 26
    cv2.rectangle(vis, (0, 0), (vis.shape[1], banner_h), (22, 22, 22), -1)
    cv2.putText(vis, label_header, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 255), 1, cv2.LINE_AA)
    if sub_header:
        cv2.putText(vis, sub_header, (8, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.40, (210, 235, 210), 1, cv2.LINE_AA)
    return vis


def _compute_target_mask_info(
    frame: np.ndarray,
    tracker: MultiModelTrackerCore,
    detections: List[UnifiedTrackedPerson],
    tgt: Optional[UnifiedTrackedPerson],
):
    """타겟의 실제 추출 마스크(Upper/Mid/Lower/Full)와 BBox 대비 전경 픽셀 비율(%)을 반환합니다."""
    if tgt is None:
        return None, None
    in_guard = (
        tracker.mode == tracker.MODE_ADAPTIVE
        and tracker.metrics.adaptive_state == 'GUARD'
    )
    exclude_boxes = [o.bbox for o in detections if o.track_id != tgt.track_id] if in_guard else None
    part_masks = tracker.current_extractor.extract_part_masks(
        frame,
        tgt.bbox,
        mask_poly=tgt.mask_poly,
        keypoints=tgt.keypoints,
        exclude_boxes=exclude_boxes,
    )
    if part_masks is None:
        return None, None
    (rx1, ry1, rx2, ry2), upper_m, mid_m, lower_m = part_masks
    combined = cv2.bitwise_or(upper_m, cv2.bitwise_or(mid_m, lower_m))
    bx1, by1, bx2, by2 = tgt.bbox
    bbox_area = max(1, (bx2 - bx1) * (by2 - by1))
    fg_pct = 100.0 * float(cv2.countNonZero(combined)) / float(bbox_area)
    return (part_masks, combined), round(min(100.0, fg_pct), 1)


def _render_extracted_pixels_strip(
    frame: np.ndarray,
    main_vis: np.ndarray,
    tracker: MultiModelTrackerCore,
    detections: List[UnifiedTrackedPerson],
    tgt: Optional[UnifiedTrackedPerson],
    run_name: str,
    fg_pct: Optional[float],
) -> np.ndarray:
    """
    [Overlay | Upper | Mid | Lower | Full Extracted Pixels] 5칸 스트립을 생성합니다.
    검정 배경 위에 실제로 히스토그램 계산에 들어가는 전경 픽셀만 남겨서 배경 오염 여부를 직관적으로 비교합니다.
    """
    cell_w, cell_h = 210, 220

    def _fit_cell(im: np.ndarray, title: str, badge_col=(40, 40, 40)) -> np.ndarray:
        canvas = np.full((cell_h, cell_w, 3), 16, dtype=np.uint8)
        body_h = cell_h - 24
        ih, iw = im.shape[:2]
        if ih > 0 and iw > 0:
            scale = min((cell_w - 8) / float(iw), (body_h - 6) / float(ih))
            nw = max(1, int(round(iw * scale)))
            nh = max(1, int(round(ih * scale)))
            resized = cv2.resize(im, (nw, nh), interpolation=cv2.INTER_AREA)
            x0 = (cell_w - nw) // 2
            y0 = 24 + (body_h - nh) // 2
            canvas[y0:y0 + nh, x0:x0 + nw] = resized
        cv2.rectangle(canvas, (0, 0), (cell_w, 22), badge_col, -1)
        cv2.putText(canvas, title, (6, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.39, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.rectangle(canvas, (0, 0), (cell_w - 1, cell_h - 1), (65, 65, 65), 1)
        return canvas

    overlay_cell = _fit_cell(main_vis, f'{run_name} Overlay', (35, 75, 120))
    if tgt is None:
        empty_msg = np.full((180, 180, 3), 18, dtype=np.uint8)
        cv2.putText(empty_msg, 'NO TARGET', (32, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 120), 1)
        return np.hstack([
            overlay_cell,
            _fit_cell(empty_msg, 'Upper (N/A)'),
            _fit_cell(empty_msg, 'Mid (N/A)'),
            _fit_cell(empty_msg, 'Lower (N/A)'),
            _fit_cell(empty_msg, 'Full FG (N/A)'),
        ])

    mask_bundle, _ = _compute_target_mask_info(frame, tracker, detections, tgt)
    if mask_bundle is None:
        empty_msg = np.full((180, 180, 3), 18, dtype=np.uint8)
        cv2.putText(empty_msg, 'NO MASK', (42, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (120, 120, 120), 1)
        return np.hstack([
            overlay_cell,
            _fit_cell(empty_msg, 'Upper (N/A)'),
            _fit_cell(empty_msg, 'Mid (N/A)'),
            _fit_cell(empty_msg, 'Lower (N/A)'),
            _fit_cell(empty_msg, 'Full FG (N/A)'),
        ])

    (rx1, ry1, rx2, ry2), upper_m, mid_m, lower_m = mask_bundle[0]
    combined = mask_bundle[1]
    roi_bgr = frame[ry1:ry2, rx1:rx2]
    is_fb = bool(tgt.feature_bank is not None and tgt.feature_bank.is_fallback)

    def _masked_crop(m_arr: np.ndarray, lbl: str) -> np.ndarray:
        nz = cv2.countNonZero(m_arr)
        if nz < 12 or roi_bgr.size == 0:
            blank = np.full((180, 180, 3), 18, dtype=np.uint8)
            cv2.putText(blank, 'N/A (<12px)', (34, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (110, 110, 190), 1)
            return _fit_cell(blank, f'{lbl}: N/A', (55, 35, 35))
        masked = cv2.bitwise_and(roi_bgr, roi_bgr, mask=m_arr)
        return _fit_cell(masked, f'{lbl} ({nz}px)')

    fb_tag = ' [FB]' if is_fb else ''
    fg_str = f'{fg_pct:.0f}%' if fg_pct is not None else '-'
    full_cell = _fit_cell(
        cv2.bitwise_and(roi_bgr, roi_bgr, mask=combined),
        f'Full FG:{fg_str}{fb_tag}',
        (95, 50, 35) if is_fb else (35, 95, 50),
    )
    return np.hstack([
        overlay_cell,
        _masked_crop(upper_m, 'Upper'),
        _masked_crop(mid_m, 'Mid'),
        _masked_crop(lower_m, 'Lower'),
        full_cell,
    ])


def _pick_auto_key_centers(
    frame_records: List[dict],
    n_imgs: int,
    seq_delta: int,
    max_centers: int = 3,
) -> List[Tuple[str, int]]:
    """
    기준 모델 시계열에서 비교용 대표 중심 프레임(혼잡/복귀/이탈/움직임)을 자동 선정합니다.
    반환: [(event_tag, frame_idx), ...]
    """
    if n_imgs <= 0 or not frame_records:
        return []

    margin_min = max(1, min(seq_delta + 1, max(1, n_imgs // 6)))
    margin_max = max(margin_min, n_imgs - min(seq_delta, max(0, n_imgs // 6)))
    cands = [r for r in frame_records if margin_min <= r['frame_idx'] <= margin_max]
    if not cands:
        cands = list(frame_records)

    chosen: List[Tuple[str, int]] = []
    min_gap = max(seq_delta * 2, 10)

    def _try_add(tag: str, fidx: int) -> bool:
        if len(chosen) >= max_centers:
            return False
        if any(abs(fidx - prev_f) < min_gap for _, prev_f in chosen):
            return False
        chosen.append((tag, int(fidx)))
        return True

    # 1) GUARD 또는 다수 인원 공존 시점 중 타인 유사도(other_max_sim)가 가장 높은 프레임
    crowd = [
        r for r in cands
        if (r['adaptive_state'] == 'GUARD' or r['num_persons'] >= 2) and r['other_max_sim'] != ''
    ]
    if crowd:
        best_crowd = max(crowd, key=lambda r: float(r['other_max_sim']))
        _try_add('crowd', best_crowd['frame_idx'])

    # 2) RECOVERED 복귀 또는 SEARCH 이탈 시점
    rec = [r for r in cands if r['adaptive_state'] in ('RECOVERED', 'SEARCH')]
    if rec:
        rec_first = next((r for r in rec if r['adaptive_state'] == 'RECOVERED'), rec[0])
        _try_add(rec_first['adaptive_state'].lower(), rec_first['frame_idx'])

    # 3) 타겟 이동량(bbox_move_px)이 가장 큰 움직임 프레임
    moving = [r for r in cands if r['locked'] == 1 and r['bbox_move_px'] != '' and float(r['bbox_move_px']) > 1.0]
    if moving:
        best_move = max(moving, key=lambda r: float(r['bbox_move_px']))
        _try_add('motion', best_move['frame_idx'])

    # 4) 이벤트가 부족하면 전체 구간 1/4, 1/2, 3/4 지점으로 보충
    for frac, tag in ((0.25, 'q1'), (0.50, 'mid'), (0.75, 'q3')):
        if len(chosen) >= max_centers:
            break
        fidx = max(1, min(n_imgs, int(round(n_imgs * frac))))
        _try_add(tag, fidx)

    if not chosen:
        chosen.append(('sample', cands[len(cands) // 2]['frame_idx']))

    return sorted(chosen, key=lambda item: item[1])


def _expand_seq_indices(key_centers: List[Tuple[str, int]], seq_delta: int, n_imgs: int) -> set:
    out = set()
    for _, c in key_centers:
        for d in (-seq_delta, 0, seq_delta):
            f = max(1, min(n_imgs, c + d))
            out.add(f)
    return out


def _run_single_experiment(
    events,
    topics: dict,
    *,
    run_name: str,
    family_idx: int,
    task_idx: int,
    mode: int,
    device: str,
    use_kinematics: bool,
    reid_kwargs: dict,
    lock_point: Optional[Tuple[int, int]],
    save_dashboard_snaps: bool,
    enable_reid_viz: bool,
    snap_dir: str,
    capture_frame_indices: Optional[set] = None,
    auto_cache_all_for_centers: bool = False,
    capture_pixel_strips: bool = False,
    seq_delta: int = 10,
    gt_map: Optional[Dict[int, dict]] = None,
    gt_iou_thresh: float = 0.5,
):
    is_comp = 'CompressedImage' in topics['image_type']
    has_scan = bool(topics['scan_topic'])

    safe_node = re.sub(r'[^a-zA-Z0-9_]', '_', run_name)
    node = Node(f'bench_{safe_node}_{int(time.time() * 1000) % 100000}', namespace=topics['ns'])
    node.declare_parameter('publish_cam_tf', False)
    node.declare_parameter('publish_target_topics', False)
    node.declare_parameter('auto_start_lidar_motor', False)
    node.declare_parameter('use_kinematic_gating', bool(use_kinematics))
    node.declare_parameter('filter_map_walls', bool(use_kinematics))
    node.declare_parameter('filter_long_segments', bool(use_kinematics))
    loc = LidarPersonLocalizer(node) if has_scan else None

    tracker = MultiModelTrackerCore(
        family_idx=family_idx,
        task_idx=task_idx,
        mode=mode,
        imgsz=320,
        device=device,
        **reid_kwargs,
    )
    reid_viz = ReIDDebugVisualizer() if (save_dashboard_snaps or enable_reid_viz) else None

    total_imgs = 0
    det_frames = 0
    locked_frames = 0
    true_locked_frames = 0
    hijack_frames = 0
    crowd_frames = 0
    safe_frames = 0
    guard_frames = 0
    search_frames = 0
    recovered_count = 0
    direct_id_switches = 0
    prev_locked_id: Optional[int] = None
    prev_state: str = 'IDLE'
    prev_tgt_center: Optional[Tuple[int, int]] = None
    locked_once = False
    search_streak = 0
    recovery_durations: List[int] = []

    pre_ms_list = []
    inf_ms_list = []
    post_ms_list = []
    reid_ms_list = []
    trk_ms_list = []
    tot_ms_list = []
    tgt_sims = []
    other_sims = []
    u_sims, m_sims, l_sims, f_sims = [], [], [], []
    u_valid_cnt = 0
    m_valid_cnt = 0
    l_valid_cnt = 0
    fallback_cnt = 0
    fg_pct_list = []
    margin_list = []

    loc_counts = {'LiDAR': 0, 'Occluded': 0, 'CamEst': 0}
    lidar_jumps = 0
    far_camest = 0
    amcl_wall_or_out = 0
    prev_lidar_dist: Optional[float] = None
    prev_lidar_id: Optional[int] = None

    gt_pos_frames = 0
    gt_neg_frames = 0
    gt_true_lock_frames = 0
    gt_hijack_frames = 0
    gt_miss_frames = 0
    gt_false_lock_frames = 0
    gt_fail_episodes = 0
    prev_gt_err = False
    gt_ious: List[float] = []
    has_pos_gt = bool(
        gt_map
        and any(v.get('present') == 1 and v.get('bbox') is not None for v in gt_map.values())
    )

    frame_records: List[dict] = []
    series_t = []
    series_sim = []
    series_margin = []
    series_tot_ms = []
    series_dist = []
    series_src = []

    captured_frames: Dict[int, dict] = {}
    snap_counts: Dict[str, int] = {}

    ring_buf: deque = deque(maxlen=max(2, seq_delta + 1))
    pending_after: set = set()
    event_hint_captured: Dict[str, int] = {}

    for ev_kind, rel_sec, msg in events:
        if ev_kind == 'tf' and loc is not None:
            loc._on_tf_msg(msg)
        elif ev_kind == 'tf_static' and loc is not None:
            loc._on_tf_static_msg(msg)
        elif ev_kind == 'scan' and loc is not None:
            loc._on_scan(msg)
        elif ev_kind == 'image':
            frame = _decode_image(msg, is_comp)
            if frame is None:
                continue
            total_imgs += 1
            h, w = frame.shape[:2]
            rx_wall = (
                loc.last_scan_WallTime
                if (loc is not None and loc.last_scan_WallTime > 0.0)
                else time.time()
            )

            def _spatial_est(person, pw, ph):
                if loc is None:
                    return None
                res_s = loc.localize_target(
                    person,
                    pw,
                    ph,
                    publish=False,
                    frame_stamp=msg.header.stamp,
                    frame_rx_wall=rx_wall,
                    use_kinematics=False,
                )
                if res_s.map_xyz is not None:
                    return (res_s.map_xyz[0], res_s.map_xyz[1]), 'map', res_s.source
                return (res_s.base_xyz[0], res_s.base_xyz[1]), 'base_link', res_s.source

            t_f0 = time.perf_counter()
            gt_entry = gt_map.get(total_imgs) if gt_map is not None else None
            eff_lock_point = lock_point
            gt_lock_bbox = None
            if (
                eff_lock_point is None
                and not locked_once
                and gt_entry is not None
                and gt_entry['present'] == 1
                and gt_entry['bbox'] is not None
            ):
                gt_lock_bbox = gt_entry['bbox']

            dets, tgt = tracker.process_frame(
                frame,
                auto_lock_first=(lock_point is None and not has_pos_gt and not locked_once),
                spatial_estimator=_spatial_est if loc is not None else None,
                compute_all_sims=True,
            )
            if not locked_once and dets:
                sel = None
                if eff_lock_point is not None:
                    sel = tracker.select_target_by_point(eff_lock_point[0], eff_lock_point[1])
                elif gt_lock_bbox is not None:
                    best_det = max(dets, key=lambda p: _bbox_iou(p.bbox, gt_lock_bbox))
                    if _bbox_iou(best_det.bbox, gt_lock_bbox) >= min(0.3, gt_iou_thresh):
                        sel = tracker.select_target_by_id(best_det.track_id)
                if sel is not None:
                    locked_once = True
                    tgt = next((p for p in dets if p.track_id == sel), None)
                    if tgt is not None:
                        tgt.is_target = True
                elif tgt is not None:
                    locked_once = True
            elif tgt is not None:
                locked_once = True

            if dets:
                det_frames += 1
            if len(dets) >= 2:
                crowd_frames += 1

            st = tracker.metrics.adaptive_state
            loc_res = None
            frame_other_sims = []
            u_s = m_s = l_s = f_s = None
            bbox_move_px: Optional[float] = None
            is_fb = False
            u_vis = m_vis = l_vis = False
            fg_pct: Optional[float] = None

            if tgt is not None:
                locked_frames += 1
                sim_val = float(tgt.similarity)
                if sim_val >= HIJACK_SIM_THRESH:
                    true_locked_frames += 1
                else:
                    hijack_frames += 1

                # SEARCH -> RECOVERED 복귀와 비-SEARCH 직접 ID 탈취(direct_id_switches)를 엄밀히 분리
                came_from_search = (prev_state == 'SEARCH' or st == 'RECOVERED')
                if came_from_search and search_streak > 0:
                    recovery_durations.append(search_streak)
                    search_streak = 0
                if (
                    prev_locked_id is not None
                    and tgt.track_id != prev_locked_id
                    and not came_from_search
                ):
                    direct_id_switches += 1
                prev_locked_id = tgt.track_id

                if prev_tgt_center is not None:
                    bbox_move_px = math.hypot(
                        float(tgt.center[0] - prev_tgt_center[0]),
                        float(tgt.center[1] - prev_tgt_center[1]),
                    )
                prev_tgt_center = tgt.center

                u_vis, m_vis, l_vis = (bool(x) for x in tgt.visible_parts)
                if u_vis:
                    u_valid_cnt += 1
                if m_vis:
                    m_valid_cnt += 1
                if l_vis:
                    l_valid_cnt += 1
                if tgt.feature_bank is not None and tgt.feature_bank.is_fallback:
                    is_fb = True
                    fallback_cnt += 1

                _, fg_pct = _compute_target_mask_info(frame, tracker, dets, tgt)
                if fg_pct is not None:
                    fg_pct_list.append(fg_pct)

                tgt_sims.append(sim_val)
                u_s, m_s, l_s, f_s = tgt.part_sims
                if u_s is not None:
                    u_sims.append(float(u_s))
                if m_s is not None:
                    m_sims.append(float(m_s))
                if l_s is not None:
                    l_sims.append(float(l_s))
                if f_s is not None:
                    f_sims.append(float(f_s))

                for p in dets:
                    if not p.is_target:
                        sv = float(p.similarity)
                        other_sims.append(sv)
                        frame_other_sims.append(sv)

                if loc is not None:
                    loc_res = loc.localize_target(
                        tgt,
                        w,
                        h,
                        publish=False,
                        frame_stamp=msg.header.stamp,
                        frame_rx_wall=rx_wall,
                        use_kinematics=use_kinematics,
                    )
                    loc_counts[loc_res.source] = loc_counts.get(loc_res.source, 0) + 1
                    if loc_res.source == 'LiDAR':
                        if (
                            prev_lidar_dist is not None
                            and prev_lidar_id == tgt.track_id
                            and abs(loc_res.cam_dist_m - prev_lidar_dist) > 0.8
                        ):
                            lidar_jumps += 1
                        prev_lidar_dist = loc_res.cam_dist_m
                        prev_lidar_id = tgt.track_id
                    elif loc_res.source == 'CamEst' and loc_res.cam_dist_m > 2.0:
                        far_camest += 1
                    if (
                        loc_res.source != 'Occluded'
                        and loc_res.map_xyz is not None
                        and loc_res.map_source == 'AMCL'
                        and loc._is_map_point_wall_or_outside(loc_res.map_xyz[0], loc_res.map_xyz[1], 0.0)
                    ):
                        amcl_wall_or_out += 1
            else:
                prev_tgt_center = None
                if st == 'SEARCH':
                    search_streak += 1
                for p in dets:
                    frame_other_sims.append(float(p.similarity))

            frame_tot_ms = (time.perf_counter() - t_f0) * 1000.0
            m = tracker.metrics
            if m.inference_ms > 0.0:
                pre_ms_list.append(float(m.preprocess_ms))
                inf_ms_list.append(float(m.inference_ms))
                post_ms_list.append(float(m.post_track_ms))
                reid_ms_list.append(float(m.reid_ms))
                trk_ms_list.append(float(m.post_track_ms + m.reid_ms))
                tot_ms_list.append(float(frame_tot_ms))

            if st == 'SAFE':
                safe_frames += 1
            elif st == 'GUARD':
                guard_frames += 1
            elif st == 'SEARCH':
                search_frames += 1
            elif st == 'RECOVERED':
                recovered_count += 1
            prev_state = st

            other_max = max(frame_other_sims) if frame_other_sims else None
            other_avg = float(np.mean(frame_other_sims)) if frame_other_sims else None
            frame_margin = (
                (float(tgt.similarity) - other_max)
                if (tgt is not None and other_max is not None)
                else None
            )
            if frame_margin is not None:
                margin_list.append(float(frame_margin))

            gt_iou_val: Optional[float] = None
            gt_status_str = ''
            if gt_map is not None:
                if gt_entry is not None:
                    is_err = False
                    if gt_entry['present'] == 1 and gt_entry['bbox'] is not None:
                        gt_pos_frames += 1
                        gt_iou_val = _bbox_iou(tgt.bbox, gt_entry['bbox']) if tgt is not None else 0.0
                        if tgt is None:
                            gt_miss_frames += 1
                            gt_status_str = 'MISS'
                            is_err = True
                        elif gt_iou_val >= gt_iou_thresh:
                            gt_true_lock_frames += 1
                            gt_status_str = 'TRUE_LOCK'
                            gt_ious.append(gt_iou_val)
                        else:
                            gt_hijack_frames += 1
                            gt_status_str = 'HIJACK'
                            is_err = True
                            gt_ious.append(gt_iou_val)
                    else:
                        gt_neg_frames += 1
                        if tgt is not None:
                            gt_false_lock_frames += 1
                            gt_status_str = 'FALSE_LOCK'
                            is_err = True
                        else:
                            gt_status_str = 'TN'
                    if is_err and not prev_gt_err:
                        gt_fail_episodes += 1
                    prev_gt_err = is_err
                else:
                    gt_status_str = 'UNLABELED'

            rec = {
                'frame_idx': total_imgs,
                't_sec': round(rel_sec, 4),
                'run_name': run_name,
                'model': tracker.current_model_name,
                'task': tracker.current_task_name,
                'mode': MultiModelTrackerCore.MODE_NAMES[mode],
                'use_kinematics': int(bool(use_kinematics)),
                'num_persons': len(dets),
                'locked': 1 if tgt is not None else 0,
                'true_locked': 1 if (tgt is not None and tgt.similarity >= HIJACK_SIM_THRESH) else 0,
                'hijacked': 1 if (tgt is not None and tgt.similarity < HIJACK_SIM_THRESH) else 0,
                'target_id': tgt.track_id if tgt is not None else -1,
                'adaptive_state': st,
                'ema_status': m.ema_status,
                'pose_fallback': 1 if is_fb else 0,
                'u_valid': 1 if u_vis else 0,
                'm_valid': 1 if m_vis else 0,
                'l_valid': 1 if l_vis else 0,
                'mask_fg_pct': fg_pct if fg_pct is not None else '',
                'tgt_sim': _round_opt(tgt.similarity if tgt is not None else None, 4),
                'tgt_u_sim': _round_opt(u_s, 4),
                'tgt_m_sim': _round_opt(m_s, 4),
                'tgt_l_sim': _round_opt(l_s, 4),
                'tgt_f_sim': _round_opt(f_s, 4),
                'other_max_sim': _round_opt(other_max, 4),
                'other_avg_sim': _round_opt(other_avg, 4),
                'sim_margin': _round_opt(frame_margin, 4),
                'bbox_x1': tgt.bbox[0] if tgt is not None else '',
                'bbox_y1': tgt.bbox[1] if tgt is not None else '',
                'bbox_x2': tgt.bbox[2] if tgt is not None else '',
                'bbox_y2': tgt.bbox[3] if tgt is not None else '',
                'bbox_cx': tgt.center[0] if tgt is not None else '',
                'bbox_cy': tgt.center[1] if tgt is not None else '',
                'bbox_area_ratio': _round_opt(tgt.area_ratio if tgt is not None else None, 5),
                'bbox_move_px': _round_opt(bbox_move_px, 2),
                'pre_ms': round(float(m.preprocess_ms), 2),
                'infer_ms': round(float(m.inference_ms), 2),
                'post_track_ms': round(float(m.post_track_ms), 2),
                'reid_ms': round(float(m.reid_ms), 2),
                'total_ms': round(float(frame_tot_ms), 2),
                'loc_source': loc_res.source if loc_res is not None else '',
                'loc_dist_m': _round_opt(loc_res.cam_dist_m if loc_res is not None else None, 3),
                'loc_base_x': _round_opt(loc_res.base_xyz[0] if loc_res is not None else None, 3),
                'loc_base_y': _round_opt(loc_res.base_xyz[1] if loc_res is not None else None, 3),
                'loc_map_x': _round_opt(
                    loc_res.map_xyz[0] if (loc_res is not None and loc_res.map_xyz is not None) else None, 3
                ),
                'loc_map_y': _round_opt(
                    loc_res.map_xyz[1] if (loc_res is not None and loc_res.map_xyz is not None) else None, 3
                ),
                'search_reason': m.search_reason,
                'gt_present': gt_entry['present'] if gt_entry is not None else '',
                'gt_iou': _round_opt(gt_iou_val, 4) if gt_map is not None else '',
                'gt_status': gt_status_str,
            }
            frame_records.append(rec)

            series_t.append(rel_sec)
            series_sim.append(float(tgt.similarity) if tgt is not None else float('nan'))
            series_margin.append(float(frame_margin) if frame_margin is not None else float('nan'))
            series_tot_ms.append(float(frame_tot_ms))
            series_dist.append(float(loc_res.cam_dist_m) if loc_res is not None else float('nan'))
            series_src.append(loc_res.source if loc_res is not None else 'NONE')

            reid_panel = reid_viz.render(tracker, dets) if reid_viz is not None else None

            def _build_overlay():
                hdr = f'{run_name} ({tracker.current_model_name}) | f#{total_imgs} ({rel_sec:.1f}s) | {st}'
                fb_s = ' [FB]' if is_fb else ''
                fg_s = f' FG:{fg_pct:.0f}%' if fg_pct is not None else ''
                sub = (
                    f'Sim:{_fmt_opt(tgt.similarity if tgt else None)}{fb_s}{fg_s} '
                    f'(U:{_fmt_opt(u_s)} M:{_fmt_opt(m_s)} L:{_fmt_opt(l_s)}) | '
                    f'Inf:{m.inference_ms:.1f}ms Trk:{m.post_track_ms + m.reid_ms:.1f}ms'
                )
                return _draw_main_overlay(frame, tracker, dets, loc_res, hdr, sub)

            def _make_item():
                mv = _build_overlay()
                px_strip = (
                    _render_extracted_pixels_strip(frame, mv, tracker, dets, tgt, run_name, fg_pct)
                    if capture_pixel_strips
                    else None
                )
                return {
                    'main_vis': mv,
                    'reid_panel': reid_panel,
                    'pixel_strip': px_strip,
                    't_sec': rel_sec,
                    'rec': rec,
                }

            want_cap = bool(capture_frame_indices and total_imgs in capture_frame_indices)
            if auto_cache_all_for_centers:
                cur_item = _make_item()
                ring_buf.append((total_imgs, cur_item))

                trigger_key = None
                if st == 'GUARD' and event_hint_captured.get('guard', 0) < 3:
                    trigger_key = 'guard'
                elif st in ('SEARCH', 'RECOVERED') and event_hint_captured.get(st.lower(), 0) < 3:
                    trigger_key = st.lower()
                elif len(dets) >= 2 and other_max is not None and other_max >= 0.30 and event_hint_captured.get('crowd', 0) < 3:
                    trigger_key = 'crowd'
                elif bbox_move_px is not None and bbox_move_px >= 12.0 and event_hint_captured.get('motion', 0) < 3:
                    trigger_key = 'motion'

                if want_cap or total_imgs in pending_after:
                    captured_frames[total_imgs] = cur_item
                if trigger_key is not None:
                    event_hint_captured[trigger_key] = event_hint_captured.get(trigger_key, 0) + 1
                    captured_frames[total_imgs] = cur_item
                    if len(ring_buf) >= 1:
                        prev_f, prev_item = ring_buf[0]
                        captured_frames[prev_f] = prev_item
                    pending_after.add(total_imgs + seq_delta)
            elif want_cap:
                captured_frames[total_imgs] = _make_item()

            if save_dashboard_snaps and reid_panel is not None and dets:
                ev_tag = None
                if st in ('GUARD', 'SEARCH', 'RECOVERED') and snap_counts.get(st, 0) < 2:
                    ev_tag = st
                elif loc_res is not None and loc_res.source == 'Occluded' and snap_counts.get('Occluded', 0) < 2:
                    ev_tag = 'Occluded'
                elif total_imgs == 15 and snap_counts.get('SAFE', 0) < 1 and tgt is not None:
                    ev_tag = 'SAFE'

                if ev_tag is not None:
                    snap_counts[ev_tag] = snap_counts.get(ev_tag, 0) + 1
                    main_vis = (
                        captured_frames[total_imgs]['main_vis']
                        if total_imgs in captured_frames
                        else _build_overlay()
                    )
                    bench_panel = render_benchmark_panel(
                        tracker=tracker,
                        cam_fps=1000.0 / max(frame_tot_ms, 1.0),
                        total_ms=frame_tot_ms,
                        gesture_ms=0.0,
                        cpu_percent=0.0,
                        ram_mb=0.0,
                        vram_mb=None,
                        num_persons=len(dets),
                        cam_status=loc.get_camera_pose_status() if loc is not None else None,
                        loc_res=loc_res,
                        bench_stats={},
                    )
                    dash, _ = compose_dashboard_frame(main_vis, reid_panel=reid_panel, bench_panel=bench_panel)
                    out_png = os.path.join(snap_dir, f'{run_name}_{ev_tag}_f{total_imgs:04d}.png')
                    cv2.imwrite(out_png, dash)

    node.destroy_node()

    avg_pre = float(np.mean(pre_ms_list)) if pre_ms_list else 0.0
    avg_inf = float(np.mean(inf_ms_list)) if inf_ms_list else 0.0
    p95_inf = float(np.percentile(inf_ms_list, 95)) if inf_ms_list else 0.0
    avg_post = float(np.mean(post_ms_list)) if post_ms_list else 0.0
    avg_reid = float(np.mean(reid_ms_list)) if reid_ms_list else 0.0
    avg_trk = float(np.mean(trk_ms_list)) if trk_ms_list else 0.0
    avg_tot = float(np.mean(tot_ms_list)) if tot_ms_list else 0.0
    p95_tot = float(np.percentile(tot_ms_list, 95)) if tot_ms_list else 0.0
    max_fps = 1000.0 / max(avg_inf + avg_trk, 0.1)

    avg_tgt_sim = float(np.mean(tgt_sims)) if tgt_sims else 0.0
    std_tgt_sim = float(np.std(tgt_sims)) if tgt_sims else 0.0
    min_tgt_sim = float(np.min(tgt_sims)) if tgt_sims else 0.0
    avg_oth_sim = float(np.mean(other_sims)) if other_sims else float('nan')
    max_oth_sim = float(np.max(other_sims)) if other_sims else float('nan')
    avg_margin = float(np.mean(margin_list)) if margin_list else (
        (avg_tgt_sim - avg_oth_sim) if not math.isnan(avg_oth_sim) else float('nan')
    )
    min_margin = float(np.min(margin_list)) if margin_list else float('nan')

    lk_denom = max(locked_frames, 1)
    summary = {
        'run_name': run_name,
        'model': tracker.current_model_name,
        'task': tracker.current_task_name,
        'mode': MultiModelTrackerCore.MODE_NAMES[mode],
        'use_kinematics': use_kinematics,
        'frames': total_imgs,
        'det_pct': round(100.0 * det_frames / max(total_imgs, 1), 1),
        'crowd_pct': round(100.0 * crowd_frames / max(total_imgs, 1), 1),
        'locked_pct': round(100.0 * locked_frames / max(det_frames, 1), 1),
        'true_lock_pct': round(100.0 * true_locked_frames / max(total_imgs, 1), 1),
        'hijack_pct': round(100.0 * hijack_frames / max(total_imgs, 1), 1),
        'safe_pct': round(100.0 * safe_frames / max(total_imgs, 1), 1),
        'search_pct': round(100.0 * search_frames / max(total_imgs, 1), 1),
        'guard_pct': round(100.0 * guard_frames / max(total_imgs, 1), 1),
        'recovered': recovered_count,
        'avg_rec_frames': round(float(np.mean(recovery_durations)), 1) if recovery_durations else '',
        'direct_sw': direct_id_switches,
        'pre_ms': round(avg_pre, 2),
        'infer_ms': round(avg_inf, 2),
        'infer_p95_ms': round(p95_inf, 2),
        'post_track_ms': round(avg_post, 2),
        'reid_ms': round(avg_reid, 2),
        'trk_reid_ms': round(avg_trk, 2),
        'total_ms': round(avg_tot, 2),
        'total_p95_ms': round(p95_tot, 2),
        'max_fps': round(max_fps, 1),
        'u_valid_pct': round(100.0 * u_valid_cnt / lk_denom, 1),
        'm_valid_pct': round(100.0 * m_valid_cnt / lk_denom, 1),
        'l_valid_pct': round(100.0 * l_valid_cnt / lk_denom, 1),
        'fallback_pct': round(100.0 * fallback_cnt / lk_denom, 1),
        'mask_fg_pct': round(float(np.mean(fg_pct_list)), 1) if fg_pct_list else '',
        'avg_tgt_sim': round(avg_tgt_sim, 3),
        'std_tgt_sim': round(std_tgt_sim, 3),
        'min_tgt_sim': round(min_tgt_sim, 3),
        'avg_u_sim': round(float(np.mean(u_sims)), 3) if u_sims else '',
        'avg_m_sim': round(float(np.mean(m_sims)), 3) if m_sims else '',
        'avg_l_sim': round(float(np.mean(l_sims)), 3) if l_sims else '',
        'avg_f_sim': round(float(np.mean(f_sims)), 3) if f_sims else '',
        'avg_other_sim': round(avg_oth_sim, 3) if not math.isnan(avg_oth_sim) else '',
        'max_other_sim': round(max_oth_sim, 3) if not math.isnan(max_oth_sim) else '',
        'sim_margin': round(avg_margin, 3) if not math.isnan(avg_margin) else '',
        'min_margin': round(min_margin, 3) if not math.isnan(min_margin) else '',
        'lidar_cnt': loc_counts['LiDAR'],
        'occluded_cnt': loc_counts['Occluded'],
        'camest_cnt': loc_counts['CamEst'],
        'lidar_jumps': lidar_jumps,
        'far_camest': far_camest,
        'amcl_wall_out': amcl_wall_or_out,
        'gt_pos_frames': gt_pos_frames if gt_map is not None else '',
        'gt_neg_frames': gt_neg_frames if gt_map is not None else '',
        'gt_true_lock_pct': round(100.0 * gt_true_lock_frames / gt_pos_frames, 1) if gt_pos_frames > 0 else '',
        'gt_hijack_pct': round(100.0 * gt_hijack_frames / gt_pos_frames, 1) if gt_pos_frames > 0 else '',
        'gt_miss_pct': round(100.0 * gt_miss_frames / gt_pos_frames, 1) if gt_pos_frames > 0 else '',
        'gt_false_lock_pct': round(100.0 * gt_false_lock_frames / gt_neg_frames, 1) if gt_neg_frames > 0 else '',
        'gt_mean_iou': round(float(np.mean(gt_ious)), 3) if gt_ious else '',
        'gt_fail_episodes': gt_fail_episodes if gt_map is not None else '',
    }
    return summary, {
        't': series_t,
        'sim': series_sim,
        'margin': series_margin,
        'tot_ms': series_tot_ms,
        'dist': series_dist,
        'src': series_src,
        'records': frame_records,
        'captured': captured_frames,
    }


def _save_csv(path: str, rows: List[dict]):
    if not rows:
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def _plot_model_charts(model_rows: List[dict], model_series: Dict[str, dict], charts_dir: str):
    if not model_rows:
        return
    labels = [r['run_name'] for r in model_rows]
    infs = [r['infer_ms'] for r in model_rows]
    posts = [r['post_track_ms'] for r in model_rows]
    reids = [r['reid_ms'] for r in model_rows]
    fpss = [r['max_fps'] for r in model_rows]
    tlocks = [r['true_lock_pct'] for r in model_rows]
    hijacks = [r['hijack_pct'] for r in model_rows]
    sims = [r['avg_tgt_sim'] for r in model_rows]

    # 1) 지연시간 분해(Infer / Post+Track / ReID) & Max FPS
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.5, 4.8))
    x = np.arange(len(labels))
    infs_arr = np.array(infs)
    posts_arr = np.array(posts)
    reids_arr = np.array(reids)
    ax1.bar(x, infs_arr, label='YOLO Infer (ms)', color='#4C8BF5')
    ax1.bar(x, posts_arr, bottom=infs_arr, label='Post + Track (ms)', color='#F4B400')
    ax1.bar(x, reids_arr, bottom=infs_arr + posts_arr, label='ReID Extract (ms)', color='#EA4335')
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, rotation=28, ha='right', fontsize=9)
    ax1.set_ylabel('Latency (ms)')
    ax1.set_title('Model & Task Latency Breakdown (Infer / Post+Track / ReID)')
    ax1.legend(fontsize=8)
    ax1.grid(axis='y', alpha=0.3)

    ax2.bar(x, fpss, color='#0F9D58')
    for xi, fv in zip(x, fpss):
        ax2.text(xi, fv + 0.5, f'{fv:.1f}', ha='center', va='bottom', fontsize=8)
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels, rotation=28, ha='right', fontsize=9)
    ax2.set_ylabel('Max Infer FPS')
    ax2.set_title('Model & Task Throughput (FPS)')
    ax2.grid(axis='y', alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(charts_dir, 'chart_models_latency_fps.png'), dpi=150)
    plt.close(fig)

    # 2) 진성 유지율(True Lock vs Hijack) & 평균 유사도
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.5, 4.8))
    ax1.bar(x - 0.17, tlocks, width=0.34, label='True Lock % (sim>=0.40)', color='#0F9D58')
    ax1.bar(x + 0.17, hijacks, width=0.34, label='Hijack % (sim<0.40)', color='#DB4437')
    for xi, lv in zip(x, tlocks):
        ax1.text(xi - 0.17, lv + 1.0, f'{lv:.1f}%', ha='center', va='bottom', fontsize=7.5)
    ax1.set_ylim(0, 112)
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, rotation=28, ha='right', fontsize=9)
    ax1.set_ylabel('Ratio of All Frames (%)')
    ax1.set_title('True Target Lock vs False Hijack Ratio')
    ax1.legend(fontsize=8)
    ax1.grid(axis='y', alpha=0.3)

    ax2.bar(x, sims, color='#AB47BC')
    for xi, sv in zip(x, sims):
        ax2.text(xi, sv + 0.015, f'{sv:.2f}', ha='center', va='bottom', fontsize=8)
    ax2.set_ylim(0, 1.05)
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels, rotation=28, ha='right', fontsize=9)
    ax2.set_ylabel('Avg Target Similarity')
    ax2.set_title('Average Target ReID Similarity')
    ax2.grid(axis='y', alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(charts_dir, 'chart_models_tracking.png'), dpi=150)
    plt.close(fig)

    # 3) 부위별(Upper / Mid / Lower / Full / Total) 유사도 비교 차트
    fig, ax = plt.subplots(figsize=(12.5, 4.8))
    part_defs = (
        ('avg_tgt_sim', 'Total', '#2E7D32'),
        ('avg_u_sim', 'Upper (Torso)', '#1E88E5'),
        ('avg_m_sim', 'Mid (Thighs)', '#FB8C00'),
        ('avg_l_sim', 'Lower (Calves)', '#8E24AA'),
        ('avg_f_sim', 'Full', '#546E7A'),
    )
    width = 0.15
    for idx, (col_key, lbl, color) in enumerate(part_defs):
        vals = [float(r[col_key]) if r[col_key] != '' else 0.0 for r in model_rows]
        offset = (idx - 2) * width
        ax.bar(x + offset, vals, width=width, label=lbl, color=color, alpha=0.9)
    ax.set_ylim(0, 1.05)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=24, ha='right', fontsize=9)
    ax.set_ylabel('Average Similarity')
    ax.set_title('Per-Part ReID Similarity by Model / Task (Background Mixing vs Part Alignment)')
    ax.legend(ncol=5, fontsize=8, loc='upper right')
    ax.grid(axis='y', alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(charts_dir, 'chart_models_part_sim.png'), dpi=150)
    plt.close(fig)

    # 4) 부위 가용성(U/M/L Valid %) & Pose Fallback % & 전경 마스크 비율(MaskFG %)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.5, 4.8))
    u_vals = [r['u_valid_pct'] for r in model_rows]
    m_vals = [r['m_valid_pct'] for r in model_rows]
    l_vals = [r['l_valid_pct'] for r in model_rows]
    fb_vals = [r['fallback_pct'] for r in model_rows]
    ax1.bar(x - 0.24, u_vals, width=0.16, label='Upper Valid %', color='#1E88E5')
    ax1.bar(x - 0.08, m_vals, width=0.16, label='Mid Valid %', color='#FB8C00')
    ax1.bar(x + 0.08, l_vals, width=0.16, label='Lower Valid %', color='#8E24AA')
    ax1.bar(x + 0.24, fb_vals, width=0.16, label='Pose->BBox Fallback %', color='#E53935')
    ax1.set_ylim(0, 112)
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, rotation=24, ha='right', fontsize=9)
    ax1.set_ylabel('Percentage of Locked Frames (%)')
    ax1.set_title('Body-Part Extraction Availability & Pose Fallback Rate')
    ax1.legend(fontsize=8)
    ax1.grid(axis='y', alpha=0.3)

    fg_vals = [float(r['mask_fg_pct']) if r['mask_fg_pct'] != '' else 0.0 for r in model_rows]
    margs = [float(r['sim_margin']) if r['sim_margin'] != '' else 0.0 for r in model_rows]
    ax2_r = ax2.twinx()
    ax2.bar(x - 0.17, fg_vals, width=0.34, label='Mask FG Ratio (% of BBox)', color='#00897B')
    ax2_r.bar(x + 0.17, margs, width=0.34, label='Avg Sim Margin', color='#5E35B1')
    ax2.set_ylim(0, 105)
    ax2_r.set_ylim(min(-0.1, min(margs) - 0.05) if margs else -0.1, max(0.7, max(margs) + 0.1) if margs else 0.7)
    ax2.set_xticks(x)
    ax2.set_xticklabels(labels, rotation=24, ha='right', fontsize=9)
    ax2.set_ylabel('Mask Foreground Ratio (%)')
    ax2_r.set_ylabel('Sim Margin (Tgt - MaxOther)')
    ax2.set_title('Foreground Mask Coverage vs ReID Margin')
    h1, l1 = ax2.get_legend_handles_labels()
    h2, l2 = ax2_r.get_legend_handles_labels()
    ax2.legend(h1 + h2, l1 + l2, fontsize=8, loc='upper right')
    ax2.grid(axis='y', alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(charts_dir, 'chart_models_parts_and_fg.png'), dpi=150)
    plt.close(fig)

    # 5) 동일 시점(시간축) 모델별 Sim / Sim Margin / Latency 시계열 비교 차트
    if model_series:
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(13, 9.2), sharex=True)
        for mk in labels:
            s = model_series.get(mk)
            if not s:
                continue
            t = s['t']
            ax1.plot(t, s['sim'], label=mk, linewidth=1.25, alpha=0.88)
            ax2.plot(t, s['margin'], label=mk, linewidth=1.25, alpha=0.88)
            ax3.plot(t, s['tot_ms'], label=mk, linewidth=1.1, alpha=0.80)

        ax1.axhline(HIJACK_SIM_THRESH, color='#DB4437', linestyle='--', linewidth=0.9, label=f'Hijack < {HIJACK_SIM_THRESH}')
        ax1.set_ylim(0, 1.05)
        ax1.set_ylabel('Target Sim')
        ax1.set_title('Synchronized Frame-by-Frame Comparison Across Models')
        ax1.legend(ncol=min(len(labels) + 1, 6), fontsize=8, loc='lower left')
        ax1.grid(True, alpha=0.3)

        ax2.axhline(0.0, color='#666666', linestyle='--', linewidth=0.9)
        ax2.set_ylabel('Sim Margin\n(Tgt - MaxOther)')
        ax2.grid(True, alpha=0.3)

        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Frame Latency (ms)')
        ax3.grid(True, alpha=0.3)

        fig.tight_layout()
        fig.savefig(os.path.join(charts_dir, 'chart_models_timeseries.png'), dpi=150)
        plt.close(fig)


def _plot_mode_charts(mode_rows: List[dict], mode_series: Dict[str, dict], charts_dir: str):
    if not mode_rows:
        return
    labels = [r['mode'] for r in mode_rows]
    tlocks = [r['true_lock_pct'] for r in mode_rows]
    hijacks = [r['hijack_pct'] for r in mode_rows]
    recs = [r['recovered'] for r in mode_rows]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.5, 4.8))
    x = np.arange(len(labels))
    ax1.bar(x - 0.18, tlocks, width=0.34, label='True Lock % (sim>=0.40)', color='#0F9D58')
    ax1.bar(x + 0.18, hijacks, width=0.34, label='Hijack % (sim<0.40)', color='#DB4437')
    for xi, tv, hv, rv in zip(x, tlocks, hijacks, recs):
        ax1.text(xi - 0.18, tv + 1.0, f'{tv:.1f}%\n(rec={rv})', ha='center', va='bottom', fontsize=7.5)
        if hv > 0.5:
            ax1.text(xi + 0.18, hv + 1.0, f'{hv:.1f}%', ha='center', va='bottom', fontsize=7.5, color='#B71C1C')
    ax1.set_ylim(0, 114)
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, rotation=15, ha='right', fontsize=9)
    ax1.set_ylabel('Ratio of All Frames (%)')
    ax1.set_title('Tracking Algorithm: True Lock vs Hijack & Recovery')
    ax1.legend(fontsize=8)
    ax1.grid(axis='y', alpha=0.3)

    ax2.axhline(HIJACK_SIM_THRESH, color='#DB4437', linestyle='--', linewidth=1.0, label=f'Hijack Thresh ({HIJACK_SIM_THRESH})')
    for name, s in mode_series.items():
        ax2.plot(s['t'], s['sim'], label=name, linewidth=1.25, alpha=0.88)
    ax2.set_ylim(0, 1.05)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Target Similarity')
    ax2.set_title('Target Similarity Over Time by Algorithm')
    ax2.legend(fontsize=8)
    ax2.grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(os.path.join(charts_dir, 'chart_modes_comparison.png'), dpi=150)
    plt.close(fig)


def _plot_localizer_chart(loc_series: Dict[str, dict], charts_dir: str):
    if not loc_series:
        return
    fig, ax = plt.subplots(figsize=(11, 4.5))
    colors = {'Raw (kin=False)': '#DB4437', 'Filtered (kin=True)': '#0F9D58'}
    for name, s in loc_series.items():
        t = np.array(s['t'])
        d = np.array(s['dist'])
        ax.plot(t, d, label=name, linewidth=1.3, color=colors.get(name, None), alpha=0.85)
        if 'kin=True' in name:
            occ_idx = [i for i, src in enumerate(s['src']) if src == 'Occluded']
            if occ_idx:
                ax.scatter(t[occ_idx], d[occ_idx], color='#F4B400', s=14, label='Occluded (Hold)', zorder=5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Estimated Distance (m)')
    ax.set_title('3D LiDAR Distance Estimation: Raw vs Kinematic+Map+EMA Filter')
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(charts_dir, 'chart_localizer_distance.png'), dpi=150)
    plt.close(fig)


def _fit_width(img: np.ndarray, target_w: int) -> np.ndarray:
    h, w = img.shape[:2]
    if w == target_w:
        return img
    nh = max(1, int(round(h * (target_w / float(w)))))
    return cv2.resize(img, (target_w, nh), interpolation=cv2.INTER_AREA)


def _save_side_by_side_hist_cards(
    captured_map: Dict[str, Dict[int, dict]],
    group_keys: List[str],
    prefix: str,
    tag: str,
    center_f: int,
    snap_dir: str,
) -> Optional[str]:
    if len(group_keys) < 2 or not all(center_f in captured_map.get(k, {}) for k in group_keys):
        return None
    card_w = 560
    cards = []
    for k in group_keys:
        item = captured_map[k][center_f]
        top_im = _fit_width(item['main_vis'], card_w)
        reid_im = item.get('reid_panel')
        card = np.vstack([top_im, _fit_width(reid_im, card_w)]) if reid_im is not None else top_im
        cards.append(card)
    max_h = max(c.shape[0] for c in cards)
    padded = []
    for c in cards:
        if c.shape[0] < max_h:
            c = np.vstack([c, np.full((max_h - c.shape[0], c.shape[1], 3), 20, dtype=np.uint8)])
        padded.append(c)
    sep = np.full((max_h, 6, 3), 55, dtype=np.uint8)
    joined = padded[0]
    for c in padded[1:]:
        joined = np.hstack([joined, sep, c])
    fname = f'{prefix}_{tag}_f{center_f:04d}.png'
    cv2.imwrite(os.path.join(snap_dir, fname), joined)
    return fname


def _save_all_comparison_snapshots(
    captured_map: Dict[str, Dict[int, dict]],
    key_centers: List[Tuple[str, int]],
    seq_delta: int,
    n_imgs: int,
    snap_dir: str,
) -> List[str]:
    """
    저장되는 비교 스냅샷 4종:
      1) compare_tasks_hist_<tag>_fXXXX.png : Det/Seg/Pose [마스크 + 304D HSV 히스토그램] 나란히 비교
      2) compare_pixels_<tag>_fXXXX.png     : Det/Seg/Pose [실제 추출 전경 픽셀(배경 검정 제거)] 비교
      3) compare_versions_<tag>_fXXXX.png   : v8n / 11n / 26n [마스크 + 304D HSV 히스토그램] 비교
      4) compare_seq_<tag>_fXXXX.png        : [BEFORE f-Δ | CENTER f | AFTER f+Δ] × (Det/Seg/Pose) 그리드
    """
    saved: List[str] = []
    if not key_centers:
        return saved

    task_keys = [k for k in TASK_COMPARE_MODELS if k in captured_map]
    ver_keys = [k for k in VERSION_COMPARE_MODELS if k in captured_map]
    cell_w = 480

    for tag, center_f in key_centers:
        # 1) Det/Seg/Pose [마스크 + 히스토그램] 비교
        fn1 = _save_side_by_side_hist_cards(captured_map, task_keys, 'compare_tasks_hist', tag, center_f, snap_dir)
        if fn1:
            saved.append(fn1)

        # 2) v8n / 11n / 26n 버전 비교
        fn2 = _save_side_by_side_hist_cards(captured_map, ver_keys, 'compare_versions', tag, center_f, snap_dir)
        if fn2:
            saved.append(fn2)

        # 3) Det/Seg/Pose 실제 추출 픽셀(배경 오염 여부) 비교
        if len(task_keys) >= 2 and all(
            center_f in captured_map[k] and captured_map[k][center_f].get('pixel_strip') is not None
            for k in task_keys
        ):
            strips = [captured_map[k][center_f]['pixel_strip'] for k in task_keys]
            w0 = max(s.shape[1] for s in strips)
            fitted = [_fit_width(s, w0) for s in strips]
            h_sep = np.full((6, w0, 3), 70, dtype=np.uint8)
            pix_grid = fitted[0]
            for s_im in fitted[1:]:
                pix_grid = np.vstack([pix_grid, h_sep, s_im])
            fn3 = f'compare_pixels_{tag}_f{center_f:04d}.png'
            cv2.imwrite(os.path.join(snap_dir, fn3), pix_grid)
            saved.append(fn3)

        # 4) 전·중·후 3프레임(f-Δ, f, f+Δ) × (Det/Seg/Pose) 움직임 그리드
        seq_fs = [
            ('BEFORE', max(1, center_f - seq_delta)),
            ('CENTER', center_f),
            ('AFTER', min(n_imgs, center_f + seq_delta)),
        ]
        if len(task_keys) >= 2 and all(all(f in captured_map[k] for _, f in seq_fs) for k in task_keys):
            rows_img = []
            for k in task_keys:
                row_cells = []
                for step_lbl, fidx in seq_fs:
                    im = _fit_width(captured_map[k][fidx]['main_vis'].copy(), cell_w)
                    badge = f'{step_lbl} (f{fidx})'
                    cv2.rectangle(im, (cell_w - 148, 0), (cell_w, 22), (45, 70, 120), -1)
                    cv2.putText(
                        im,
                        badge,
                        (cell_w - 142, 16),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.42,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )
                    row_cells.append(im)
                h_row = max(c.shape[0] for c in row_cells)
                fitted_cells = []
                for c in row_cells:
                    if c.shape[0] < h_row:
                        c = np.vstack([c, np.full((h_row - c.shape[0], c.shape[1], 3), 20, dtype=np.uint8)])
                    fitted_cells.append(c)
                v_sep = np.full((h_row, 4, 3), 60, dtype=np.uint8)
                rows_img.append(np.hstack([fitted_cells[0], v_sep, fitted_cells[1], v_sep, fitted_cells[2]]))
            h_sep = np.full((6, rows_img[0].shape[1], 3), 60, dtype=np.uint8)
            grid = rows_img[0]
            for r_im in rows_img[1:]:
                grid = np.vstack([grid, h_sep, r_im])
            fn4 = f'compare_seq_{tag}_f{center_f:04d}.png'
            cv2.imwrite(os.path.join(snap_dir, fn4), grid)
            saved.append(fn4)

    return saved


def _build_auto_insights(
    model_rows: List[dict],
    mode_rows: List[dict],
    loc_rows: List[dict],
) -> List[str]:
    insights: List[str] = []
    if model_rows:
        fastest = max(model_rows, key=lambda r: float(r['max_fps']))
        best_true = max(model_rows, key=lambda r: (float(r['true_lock_pct']), float(r['avg_tgt_sim'])))
        with_marg = [r for r in model_rows if r['sim_margin'] != '']
        insights.append(
            f'- **최고 연산 속도**: `{fastest["run_name"]}` (Infer `{fastest["infer_ms"]}ms`, Post+Trk `{fastest["post_track_ms"]}ms`, Max `{fastest["max_fps"]} FPS`)'
        )
        insights.append(
            f'- **최고 진성 유지율 (`True Lock %`, sim>={HIJACK_SIM_THRESH})**: `{best_true["run_name"]}` (`{best_true["true_lock_pct"]}%`, Avg Sim `{best_true["avg_tgt_sim"]}`)'
        )
        if with_marg:
            best_m = max(with_marg, key=lambda r: float(r['sim_margin']))
            insights.append(
                f'- **최고 타인 변별 마진 (`Sim Margin`)**: `{best_m["run_name"]}` (평균 `{best_m["sim_margin"]}`, 최소 `{best_m["min_margin"]}`)'
            )
        pose_rows = [r for r in model_rows if r['task'] == 'pose']
        if pose_rows:
            p_desc = ', '.join(
                f'`{r["run_name"]}` M={r["m_valid_pct"]}%/L={r["l_valid_pct"]}%/FB={r["fallback_pct"]}%'
                for r in pose_rows
            )
            insights.append(f'- **Pose 부위 가용성 & BBox 폴백 비율**: {p_desc}')
    if mode_rows:
        m_desc = ', '.join(
            f'`{r["mode"]}` TrueLock={r["true_lock_pct"]}% (Hijack={r["hijack_pct"]}%, Rec={r["recovered"]})'
            for r in mode_rows
        )
        insights.append(f'- **추적 알고리즘 효과**: {m_desc}')
    if len(loc_rows) >= 2:
        raw_r, flt_r = loc_rows[0], loc_rows[1]
        insights.append(
            f'- **LiDAR 필터링 효과**: Jumps `{raw_r["lidar_jumps"]} -> {flt_r["lidar_jumps"]}`, '
            f'FarCamEst `{raw_r["far_camest"]} -> {flt_r["far_camest"]}`, '
            f'AMCL Wall/Out `{raw_r["amcl_wall_out"]} -> {flt_r["amcl_wall_out"]}`, '
            f'Occluded Hold `{flt_r["occluded_cnt"]}f`'
        )
    return insights


def _write_markdown_report(
    path: str,
    bag_dir: str,
    topics: dict,
    total_frames: int,
    model_rows: List[dict],
    mode_rows: List[dict],
    loc_rows: List[dict],
    key_centers: List[Tuple[str, int]],
    seq_delta: int,
    saved_snaps: List[str],
    gt_iou_thresh: float = 0.5,
):
    lines = [
        f'# Bag Benchmark Report: `{os.path.basename(bag_dir.rstrip("/"))}`',
        '',
        f'- **Bag Path**: `{bag_dir}`',
        f'- **Image Topic**: `{topics["image_topic"]}` (`{total_frames}` frames evaluated)',
        f'- **Scan / TF**: `{topics["scan_topic"] or "None"}` / `{topics["tf_topic"] or "None"}`',
        f'- **Keyframes (`Δ={seq_delta}f`)**: `{", ".join(f"{t}@f{f}" for t, f in key_centers) or "None"}`',
        '',
        '## 0. 핵심 관찰 요약 (Auto Insights)',
        '',
        *_build_auto_insights(model_rows, mode_rows, loc_rows),
        '',
    ]

    if model_rows:
        lines += [
            '## 1. 모델별 연산 · 후처리 속도 분해 (`5:Adaptive`)',
            '',
            '| Model | Pre (ms) | Infer Mean / P95 (ms) | Post+Trk (ms) | ReID (ms) | Total Mean / P95 (ms) | Max FPS |',
            '|---|---:|---:|---:|---:|---:|---:|',
        ]
        for r in model_rows:
            lines.append(
                f'| {r["run_name"]} (`{r["model"]}`) | {r["pre_ms"]} | {r["infer_ms"]} / {r["infer_p95_ms"]} | '
                f'{r["post_track_ms"]} | {r["reid_ms"]} | {r["total_ms"]} / {r["total_p95_ms"]} | {r["max_fps"]} |'
            )
        lines += [
            '',
            '![Latency Breakdown](charts/chart_models_latency_fps.png)',
            '',
            '## 2. 특징 추출 용이성 · 부위 가용성 · 배경 분리 (`5:Adaptive`)',
            '',
            '| Model | U / M / L Valid % | Pose Fallback % | Mask FG % | Avg Tgt Sim ± Std (Min) | U / M / L / F Avg Sim |',
            '|---|---:|---:|---:|---:|---|',
        ]
        for r in model_rows:
            uv = f'{r["u_valid_pct"]}% / {r["m_valid_pct"]}% / {r["l_valid_pct"]}%'
            sim_str = f'{r["avg_tgt_sim"]} ± {r["std_tgt_sim"]} ({r["min_tgt_sim"]})'
            part_str = f'{r["avg_u_sim"] or "-"} / {r["avg_m_sim"] or "-"} / {r["avg_l_sim"] or "-"} / {r["avg_f_sim"] or "-"}'
            fg_str = f'{r["mask_fg_pct"]}%' if r['mask_fg_pct'] != '' else '-'
            lines.append(
                f'| {r["run_name"]} | {uv} | {r["fallback_pct"]}% | {fg_str} | {sim_str} | {part_str} |'
            )
        lines += [
            '',
            '![Part Sim](charts/chart_models_part_sim.png)',
            '![Part Availability & FG](charts/chart_models_parts_and_fg.png)',
            '',
            '## 3. 재식별 변별력 · 혼잡 · 오인식 방어 (`5:Adaptive`)',
            '',
            f'> `True Lock %`: 전체 프레임 중 유사도 `>={HIJACK_SIM_THRESH}`로 실제 타겟을 유지한 비율 · `Hijack %`: 유사도 `<{HIJACK_SIM_THRESH}` 오탐 잠금 비율',
            '',
            '| Model | Det % | >=2P % | Locked % | True Lock % | Hijack % | Direct Sw | Recovered (Avg f) | Other Avg / Max | Margin Mean / Min |',
            '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|',
        ]
        for r in model_rows:
            rec_s = f'{r["recovered"]} ({r["avg_rec_frames"]}f)' if r['avg_rec_frames'] != '' else str(r['recovered'])
            oth_s = f'{r["avg_other_sim"]} / {r["max_other_sim"]}' if r['avg_other_sim'] != '' else '-'
            mrg_s = f'{r["sim_margin"]} / {r["min_margin"]}' if r['sim_margin'] != '' else '-'
            lines.append(
                f'| {r["run_name"]} | {r["det_pct"]}% | {r["crowd_pct"]}% | {r["locked_pct"]}% | '
                f'**{r["true_lock_pct"]}%** | {r["hijack_pct"]}% | {r["direct_sw"]} | {rec_s} | {oth_s} | {mrg_s} |'
            )
        lines += [
            '',
            '![True Lock vs Hijack](charts/chart_models_tracking.png)',
            '![Synchronized Time Series](charts/chart_models_timeseries.png)',
            '',
        ]

    if mode_rows:
        lines += [
            '## 4. 추적 알고리즘 비교 (`yolo26n-pose`)',
            '',
            '| Mode | Locked % | True Lock % | Hijack % | SEARCH % | GUARD % | Direct Sw | Recovered (Avg f) | Avg Tgt Sim | Total (ms) |',
            '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|',
        ]
        for r in mode_rows:
            rec_s = f'{r["recovered"]} ({r["avg_rec_frames"]}f)' if r['avg_rec_frames'] != '' else str(r['recovered'])
            lines.append(
                f'| {r["mode"]} | {r["locked_pct"]}% | **{r["true_lock_pct"]}%** | {r["hijack_pct"]}% | '
                f'{r["search_pct"]}% | {r["guard_pct"]}% | {r["direct_sw"]} | {rec_s} | {r["avg_tgt_sim"]} | {r["total_ms"]} |'
            )
        lines += [
            '',
            '![Mode Comparison](charts/chart_modes_comparison.png)',
            '',
        ]

    if loc_rows:
        lines += [
            '## 5. 3D LiDAR Localizer 비교 (`Raw` vs `Kinematic+Map+EMA`)',
            '',
            '| Config | LiDAR | Occluded (Hold) | CamEst | LiDAR Jumps (>0.8m) | Far CamEst (>2.0m) | AMCL Wall/Out |',
            '|---|---:|---:|---:|---:|---:|---:|',
        ]
        for r in loc_rows:
            lines.append(
                f'| {r["run_name"]} | {r["lidar_cnt"]} | {r["occluded_cnt"]} | {r["camest_cnt"]} | '
                f'{r["lidar_jumps"]} | {r["far_camest"]} | {r["amcl_wall_out"]} |'
            )
        lines += [
            '',
            '![Localizer Distance](charts/chart_localizer_distance.png)',
            '',
        ]

    gt_rows = [r for r in (model_rows + mode_rows) if r.get('gt_pos_frames', '') != '']
    if gt_rows:
        lines += [
            f'## 5.5 Ground Truth(GT) 기반 실제 추적 정확도 (IoU >= {gt_iou_thresh})',
            '',
            '| Experiment | Mode | GT Pos / Neg (f) | GT True Lock % | GT Hijack % | GT Miss % | GT False Lock % | Mean IoU | Fail Episodes (예상 개입) |',
            '|---|---|---:|---:|---:|---:|---:|---:|---:|',
        ]
        for r in gt_rows:
            fl_s = f'{r["gt_false_lock_pct"]}%' if r['gt_false_lock_pct'] != '' else '-'
            iou_s = str(r['gt_mean_iou']) if r['gt_mean_iou'] != '' else '-'
            tl_s = f'**{r["gt_true_lock_pct"]}%**' if r['gt_true_lock_pct'] != '' else '-'
            hj_s = f'{r["gt_hijack_pct"]}%' if r['gt_hijack_pct'] != '' else '-'
            ms_s = f'{r["gt_miss_pct"]}%' if r['gt_miss_pct'] != '' else '-'
            lines.append(
                f'| {r["run_name"]} | {r["mode"]} | {r["gt_pos_frames"]} / {r["gt_neg_frames"]} | '
                f'{tl_s} | {hj_s} | {ms_s} | {fl_s} | {iou_s} | {r["gt_fail_episodes"]} |'
            )
        lines.append('')

    if saved_snaps:
        lines += ['## 6. 시각 검증 스냅샷 (`snapshots/`)', '']
        for fn in saved_snaps:
            lines.append(f'- `{fn}`')
            lines.append(f'  ![{fn}](snapshots/{fn})')
        lines.append('')

    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))


def evaluate_bag(bag_dir: str, out_root: str, args):
    bag_dir = os.path.abspath(os.path.expanduser(bag_dir))
    bag_name = os.path.basename(bag_dir.rstrip('/'))
    bag_out = os.path.join(out_root, bag_name)
    charts_dir = os.path.join(bag_out, 'charts')
    snap_dir = os.path.join(bag_out, 'snapshots')
    ts_dir = os.path.join(bag_out, 'timeseries')
    os.makedirs(charts_dir, exist_ok=True)
    os.makedirs(snap_dir, exist_ok=True)
    os.makedirs(ts_dir, exist_ok=True)

    storage_id = _detect_storage_id(bag_dir)
    topics = _auto_pick_topics(bag_dir, storage_id, custom_image_topic=args.image_topic)
    print(f'\n=== Bag: {bag_name} ({storage_id}) | img={topics["image_topic"]} scan={topics["scan_topic"] or "None"} ===')
    events, n_imgs = _load_bag_stream(
        bag_dir, storage_id, topics, max_frames=args.max_frames, stride=args.stride
    )
    print(f'Loaded {n_imgs} image frames ({len(events)} total messages).')

    reid_kwargs = dict(REID_SCALE_KWARGS) if args.reid_scale else {}
    gt_map = None
    if getattr(args, 'gt', ''):
        gt_path = os.path.abspath(os.path.expanduser(args.gt))
        gt_map = _load_gt_csv(gt_path)
        print(f'Loaded GT annotations: {len(gt_map)} frames from {gt_path} (IoU >= {args.gt_iou})')

    lock_point = None
    if args.lock_point:
        px, py = args.lock_point.split(',')
        lock_point = (int(px), int(py))

    if args.models in PRESETS:
        model_keys = list(PRESETS[args.models])
    else:
        model_keys = [k.strip() for k in args.models.split(',') if k.strip() in MODEL_SPECS]

    manual_centers: List[Tuple[str, int]] = []
    if args.snap_frames.strip():
        for raw_f in args.snap_frames.split(','):
            raw_f = raw_f.strip()
            if raw_f.isdigit():
                f_val = max(1, min(n_imgs, int(raw_f)))
                manual_centers.append(('user', f_val))

    exec_order = list(model_keys)
    ref_model = '26n-pose' if '26n-pose' in exec_order else (exec_order[0] if exec_order else '')
    if ref_model and exec_order[0] != ref_model:
        exec_order.remove(ref_model)
        exec_order.insert(0, ref_model)

    key_centers: List[Tuple[str, int]] = list(manual_centers)
    cap_frames: set = (
        _expand_seq_indices(key_centers, args.seq_delta, n_imgs)
        if (args.save_snapshots and key_centers)
        else set()
    )
    fallback_centers = [
        (tag, max(1, min(n_imgs, int(round(n_imgs * frac)))))
        for frac, tag in ((0.25, 'q1'), (0.50, 'mid'), (0.75, 'q3'))
    ]
    ref_pre_caps = (
        cap_frames
        if cap_frames
        else (_expand_seq_indices(fallback_centers, args.seq_delta, n_imgs) if args.save_snapshots else set())
    )

    compare_models_set = set(TASK_COMPARE_MODELS) | set(VERSION_COMPARE_MODELS)
    model_rows_map: Dict[str, dict] = {}
    model_series_map: Dict[str, dict] = {}
    captured_map: Dict[str, Dict[int, dict]] = {}

    for mk in exec_order:
        f_idx, t_idx = MODEL_SPECS[mk]
        is_ref = (mk == ref_model)
        is_cmp = (mk in compare_models_set)
        print(f'  [Model] {mk} (mode=5:Adaptive) ...')
        row, s = _run_single_experiment(
            events,
            topics,
            run_name=mk,
            family_idx=f_idx,
            task_idx=t_idx,
            mode=5,
            device=args.device,
            use_kinematics=True,
            reid_kwargs=reid_kwargs,
            lock_point=lock_point,
            save_dashboard_snaps=(args.save_snapshots and is_ref),
            enable_reid_viz=(args.save_snapshots and is_cmp),
            snap_dir=snap_dir,
            capture_frame_indices=(
                (ref_pre_caps if is_ref else cap_frames)
                if (args.save_snapshots and is_cmp)
                else None
            ),
            auto_cache_all_for_centers=(args.save_snapshots and is_ref and not manual_centers),
            capture_pixel_strips=(args.save_snapshots and mk in TASK_COMPARE_MODELS),
            seq_delta=args.seq_delta,
            gt_map=gt_map,
            gt_iou_thresh=args.gt_iou,
        )
        model_rows_map[mk] = row
        model_series_map[mk] = s
        if s['captured']:
            captured_map[mk] = s['captured']
        _save_csv(os.path.join(ts_dir, f'model_{_safe_filename(mk)}.csv'), s['records'])

        if is_ref and args.save_snapshots and not key_centers:
            auto_centers = _pick_auto_key_centers(s['records'], n_imgs, args.seq_delta, max_centers=3)
            valid_centers = []
            for tag, cf in auto_centers:
                needed = {max(1, cf - args.seq_delta), cf, min(n_imgs, cf + args.seq_delta)}
                if needed.issubset(set(s['captured'].keys())):
                    valid_centers.append((tag, cf))
            for tag, cf in fallback_centers:
                if len(valid_centers) >= 2:
                    break
                if all(abs(cf - prev_f) >= max(args.seq_delta, 8) for _, prev_f in valid_centers):
                    needed = {max(1, cf - args.seq_delta), cf, min(n_imgs, cf + args.seq_delta)}
                    if needed.issubset(set(s['captured'].keys())):
                        valid_centers.append((tag, cf))
            key_centers = sorted(valid_centers, key=lambda x: x[1])
            cap_frames = _expand_seq_indices(key_centers, args.seq_delta, n_imgs)
            print(f'  [Keyframes] Selected for visual comparison (Δ={args.seq_delta}f): {key_centers}')

    model_rows = [model_rows_map[mk] for mk in model_keys if mk in model_rows_map]
    model_series = {mk: model_series_map[mk] for mk in model_keys if mk in model_series_map}

    saved_snaps: List[str] = []
    if args.save_snapshots:
        saved_snaps = _save_all_comparison_snapshots(captured_map, key_centers, args.seq_delta, n_imgs, snap_dir)

    mode_rows = []
    mode_series = {}
    mode_list = [int(x.strip()) for x in args.modes.split(',') if x.strip().isdigit()]
    for m_idx in mode_list:
        m_label = MultiModelTrackerCore.MODE_NAMES[m_idx]
        print(f'  [Mode] 26n-pose + {m_label} ...')
        row, s = _run_single_experiment(
            events,
            topics,
            run_name=f'mode_{m_idx}',
            family_idx=3,
            task_idx=3,
            mode=m_idx,
            device=args.device,
            use_kinematics=True,
            reid_kwargs=reid_kwargs,
            lock_point=lock_point,
            save_dashboard_snaps=False,
            enable_reid_viz=False,
            snap_dir=snap_dir,
            gt_map=gt_map,
            gt_iou_thresh=args.gt_iou,
        )
        mode_rows.append(row)
        mode_series[m_label] = s
        _save_csv(os.path.join(ts_dir, f'mode_{m_idx}_{_safe_filename(m_label)}.csv'), s['records'])

    loc_rows = []
    loc_series = {}
    if args.compare_localizer and topics['scan_topic']:
        for lbl, kin_flag in (('Raw (kin=False)', False), ('Filtered (kin=True)', True)):
            print(f'  [Localizer] {lbl} ...')
            row, s = _run_single_experiment(
                events,
                topics,
                run_name=lbl,
                family_idx=3,
                task_idx=3,
                mode=5,
                device=args.device,
                use_kinematics=kin_flag,
                reid_kwargs=reid_kwargs,
                lock_point=lock_point,
                save_dashboard_snaps=False,
                enable_reid_viz=False,
                snap_dir=snap_dir,
                gt_map=gt_map,
                gt_iou_thresh=args.gt_iou,
            )
            loc_rows.append(row)
            loc_series[lbl] = s
            _save_csv(os.path.join(ts_dir, f'localizer_{_safe_filename(lbl)}.csv'), s['records'])

    _save_csv(os.path.join(bag_out, 'summary_models.csv'), model_rows)
    _save_csv(os.path.join(bag_out, 'summary_modes.csv'), mode_rows)
    _save_csv(os.path.join(bag_out, 'summary_localizer.csv'), loc_rows)
    _write_markdown_report(
        os.path.join(bag_out, 'summary.md'),
        bag_dir,
        topics,
        n_imgs,
        model_rows,
        mode_rows,
        loc_rows,
        key_centers,
        args.seq_delta,
        saved_snaps,
        gt_iou_thresh=args.gt_iou,
    )
    _plot_model_charts(model_rows, model_series, charts_dir)
    _plot_mode_charts(mode_rows, mode_series, charts_dir)
    _plot_localizer_chart(loc_series, charts_dir)
    print(f'Done! Report saved to: {bag_out}')


def main():
    parser = argparse.ArgumentParser(
        description='ROS 2 Bag 기반 person_tracker 모델·알고리즘·LiDAR 자동 벤치마크 및 차트/스냅샷 추출기'
    )
    parser.add_argument(
        '--bags',
        nargs='+',
        required=True,
        help='평가할 ROS 2 bag 폴더 경로 (1개 이상 공백으로 구분)',
    )
    parser.add_argument(
        '--out',
        default='',
        help='결과 저장 폴더 (비우면 ./bench_results/<타임스탬프>)',
    )
    parser.add_argument(
        '--models',
        default='quick',
        help='모델 프리셋(quick|versions|tasks|all9) 또는 콤마 목록(예: v8n-pose,11n-pose,26n-det,26n-seg,26n-pose)',
    )
    parser.add_argument(
        '--modes',
        default='1,3,5',
        help='비교할 추적 알고리즘 번호 콤마 목록 (1:ByteTrack, 2:Task-ReID, 3:BoT-SORT, 4:BoT+ReID, 5:Adaptive)',
    )
    parser.add_argument('--image-topic', default='', help='직접 지정할 이미지 토픽 (비우면 자동 탐색)')
    parser.add_argument('--max-frames', type=int, default=0, help='최대 평가 프레임 수 (0=전체)')
    parser.add_argument('--stride', type=int, default=1, help='프레임 샘플링 간격 (기본 1=매 프레임)')
    parser.add_argument('--device', default='', help='추론 장치 (cpu / cuda:0 / 비우면 자동)')
    parser.add_argument('--lock-point', default='', help='첫 타겟 선택 클릭 좌표 x,y (비우면 첫 검출 인물 자동 잠금)')
    parser.add_argument(
        '--snap-frames',
        default='',
        help='비교 스냅샷 중심 프레임 번호 콤마 목록 (예: 40,120,250 / 비우면 이벤트 자동 선정)',
    )
    parser.add_argument(
        '--seq-delta',
        type=int,
        default=10,
        help='전·중·후 3프레임 시퀀스 비교 간격 Δ (기본 10프레임 -> f-10, f, f+10)',
    )
    parser.add_argument('--reid-scale', action='store_true', help='follow_test REID_SCALE 완화 파라미터 적용')
    parser.add_argument('--no-localizer-compare', dest='compare_localizer', action='store_false')
    parser.add_argument('--no-snapshots', dest='save_snapshots', action='store_false')
    parser.add_argument(
        '--gt',
        default='',
        help='gt_annotator.py로 생성한 정답(GT) CSV 파일 경로 (진짜 True Lock / Hijack / Miss / Fail Episodes 집계)',
    )
    parser.add_argument('--gt-iou', type=float, default=0.5, help='GT BBox 매칭 IoU 임계값 (기본 0.5)')
    args = parser.parse_args()

    out_root = os.path.abspath(
        os.path.expanduser(args.out)
        if args.out
        else os.path.join(
            os.getcwd(),
            'bench_results',
            datetime.datetime.now().strftime('%Y%m%d_%H%M%S'),
        )
    )
    os.makedirs(out_root, exist_ok=True)

    rclpy.init()
    try:
        for bag_path in args.bags:
            evaluate_bag(bag_path, out_root, args)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
