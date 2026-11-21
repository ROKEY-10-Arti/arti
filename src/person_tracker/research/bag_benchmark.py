#!/usr/bin/env python3
"""
ROS 2 Bag 기반 person_tracker 자동 벤치마크 · 그래프 · 키프레임 스냅샷 추출 도구

기능:
  1. 1개 또는 여러 개의 ROS 2 Bag(.mcap / .db3) 폴더를 지정해 오프라인 고속 비교
  2. 모델 버전(v8n/11n/26n) × 태스크(Det/Seg/Pose) 연산 속도·유사도·유지율 정량 비교
  3. 추적 알고리즘(1:ByteTrack ~ 5:Adaptive) 비교 및 시계열 그래프 생성
  4. 3D LiDAR Localizer (Raw vs Kinematic+MapFilter+시간창 EMA) 전/후 거리·튐 비교
  5. CSV, Markdown 요약표(summary.md), matplotlib 비교 차트(.png),
     Det/Seg/Pose 3부위 마스크 비교 컷 및 상태 전환 대시보드 스냅샷(.png) 자동 저장

사용 예시:
  python3 src/person_tracker/research/bag_benchmark.py \
      --bags ~/bag/complete_bag/test_full ~/bag/complete_bag/zzz3
"""

import argparse
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
        # top_cam compressed 우선 -> oakd compressed -> raw 순으로 탐색
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
                (x1, max(36, y1 - 8)),
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
                (x1, max(36, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (180, 180, 180),
                1,
                cv2.LINE_AA,
            )
    cv2.rectangle(vis, (0, 0), (vis.shape[1], 26), (22, 22, 22), -1)
    cv2.putText(vis, label_header, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
    return vis


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
    save_snapshots: bool,
    snap_dir: str,
    capture_frame_indices: Optional[set] = None,
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
    reid_viz = ReIDDebugVisualizer() if save_snapshots else None

    total_imgs = 0
    det_frames = 0
    locked_frames = 0
    safe_frames = 0
    guard_frames = 0
    search_frames = 0
    recovered_count = 0
    id_switches = 0
    prev_locked_id: Optional[int] = None
    locked_once = False

    inf_ms_list = []
    trk_ms_list = []
    tot_ms_list = []
    tgt_sims = []
    other_sims = []

    loc_counts = {'LiDAR': 0, 'Occluded': 0, 'CamEst': 0}
    lidar_jumps = 0
    far_camest = 0
    amcl_wall_or_out = 0
    prev_lidar_dist: Optional[float] = None
    prev_lidar_id: Optional[int] = None

    series_t = []
    series_sim = []
    series_dist = []
    series_src = []
    captured_task_frames: Dict[int, np.ndarray] = {}
    snap_counts: Dict[str, int] = {}

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
            rx_wall = (loc.last_scan_WallTime if (loc is not None and loc.last_scan_WallTime > 0.0) else time.time())

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
            dets, tgt = tracker.process_frame(
                frame,
                auto_lock_first=(lock_point is None and not locked_once),
                spatial_estimator=_spatial_est if loc is not None else None,
                compute_all_sims=True,
            )
            if lock_point is not None and not locked_once and dets:
                sel = tracker.select_target_by_point(lock_point[0], lock_point[1])
                if sel is not None:
                    locked_once = True
                    tgt = next((p for p in dets if p.track_id == sel), None)
                    if tgt is not None:
                        tgt.is_target = True
            elif tgt is not None:
                locked_once = True

            if dets:
                det_frames += 1

            loc_res = None
            if tgt is not None:
                locked_frames += 1
                if prev_locked_id is not None and tgt.track_id != prev_locked_id:
                    id_switches += 1
                prev_locked_id = tgt.track_id
                tgt_sims.append(float(tgt.similarity))
                for p in dets:
                    if not p.is_target:
                        other_sims.append(float(p.similarity))

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

            frame_tot_ms = (time.perf_counter() - t_f0) * 1000.0
            m = tracker.metrics
            if m.inference_ms > 0.0:
                inf_ms_list.append(float(m.inference_ms))
                trk_ms_list.append(float(m.post_track_ms + m.reid_ms))
                tot_ms_list.append(float(frame_tot_ms))

            st = m.adaptive_state
            if st == 'SAFE':
                safe_frames += 1
            elif st == 'GUARD':
                guard_frames += 1
            elif st == 'SEARCH':
                search_frames += 1
            elif st == 'RECOVERED':
                recovered_count += 1

            series_t.append(rel_sec)
            series_sim.append(float(tgt.similarity) if tgt is not None else float('nan'))
            series_dist.append(float(loc_res.cam_dist_m) if loc_res is not None else float('nan'))
            series_src.append(loc_res.source if loc_res is not None else 'NONE')

            # 태스크별 마스크 비교 컷 캡처
            if capture_frame_indices and total_imgs in capture_frame_indices and dets:
                hdr = f'{tracker.current_model_name} | {MultiModelTrackerCore.MODE_NAMES[mode]} | Frame #{total_imgs}'
                captured_task_frames[total_imgs] = _draw_main_overlay(frame, tracker, dets, loc_res, hdr)

            # 상태 전환 키프레임 대시보드 자동 캡처
            if save_snapshots and reid_viz is not None and dets:
                ev_tag = None
                if st in ('GUARD', 'SEARCH', 'RECOVERED') and snap_counts.get(st, 0) < 2:
                    ev_tag = st
                elif loc_res is not None and loc_res.source == 'Occluded' and snap_counts.get('Occluded', 0) < 2:
                    ev_tag = 'Occluded'
                elif total_imgs == 15 and snap_counts.get('SAFE', 0) < 1 and tgt is not None:
                    ev_tag = 'SAFE'

                reid_panel = reid_viz.render(tracker, dets)
                if ev_tag is not None:
                    snap_counts[ev_tag] = snap_counts.get(ev_tag, 0) + 1
                    hdr = f'{run_name} | {ev_tag} | f={total_imgs} ({rel_sec:.1f}s)'
                    main_vis = _draw_main_overlay(frame, tracker, dets, loc_res, hdr)
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

    avg_inf = float(np.mean(inf_ms_list)) if inf_ms_list else 0.0
    avg_trk = float(np.mean(trk_ms_list)) if trk_ms_list else 0.0
    avg_tot = float(np.mean(tot_ms_list)) if tot_ms_list else 0.0
    max_fps = 1000.0 / max(avg_inf + avg_trk, 0.1)
    avg_tgt_sim = float(np.mean(tgt_sims)) if tgt_sims else 0.0
    avg_oth_sim = float(np.mean(other_sims)) if other_sims else float('nan')
    margin = (avg_tgt_sim - avg_oth_sim) if not math.isnan(avg_oth_sim) else float('nan')

    summary = {
        'run_name': run_name,
        'model': tracker.current_model_name,
        'task': tracker.current_task_name,
        'mode': MultiModelTrackerCore.MODE_NAMES[mode],
        'use_kinematics': use_kinematics,
        'frames': total_imgs,
        'det_pct': round(100.0 * det_frames / max(total_imgs, 1), 1),
        'locked_pct': round(100.0 * locked_frames / max(det_frames, 1), 1),
        'search_pct': round(100.0 * search_frames / max(total_imgs, 1), 1),
        'guard_pct': round(100.0 * guard_frames / max(total_imgs, 1), 1),
        'recovered': recovered_count,
        'id_switches': id_switches,
        'infer_ms': round(avg_inf, 2),
        'trk_reid_ms': round(avg_trk, 2),
        'total_ms': round(avg_tot, 2),
        'max_fps': round(max_fps, 1),
        'avg_tgt_sim': round(avg_tgt_sim, 3),
        'avg_other_sim': round(avg_oth_sim, 3) if not math.isnan(avg_oth_sim) else '',
        'sim_margin': round(margin, 3) if not math.isnan(margin) else '',
        'lidar_cnt': loc_counts['LiDAR'],
        'occluded_cnt': loc_counts['Occluded'],
        'camest_cnt': loc_counts['CamEst'],
        'lidar_jumps': lidar_jumps,
        'far_camest': far_camest,
        'amcl_wall_out': amcl_wall_or_out,
    }
    return summary, {
        't': series_t,
        'sim': series_sim,
        'dist': series_dist,
        'src': series_src,
        'task_frames': captured_task_frames,
    }


def _save_csv(path: str, rows: List[dict]):
    if not rows:
        return
    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)


def _plot_model_charts(model_rows: List[dict], charts_dir: str):
    if not model_rows:
        return
    labels = [r['run_name'] for r in model_rows]
    infs = [r['infer_ms'] for r in model_rows]
    trks = [r['trk_reid_ms'] for r in model_rows]
    fpss = [r['max_fps'] for r in model_rows]
    locks = [r['locked_pct'] for r in model_rows]
    sims = [r['avg_tgt_sim'] for r in model_rows]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.8))
    x = np.arange(len(labels))
    ax1.bar(x, infs, label='YOLO Infer (ms)', color='#4C8BF5')
    ax1.bar(x, trks, bottom=infs, label='Track + ReID (ms)', color='#F4B400')
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, rotation=28, ha='right', fontsize=9)
    ax1.set_ylabel('Latency (ms)')
    ax1.set_title('Model & Task Latency Breakdown')
    ax1.legend()
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

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.8))
    ax1.bar(x, locks, color='#4285F4')
    for xi, lv in zip(x, locks):
        ax1.text(xi, lv + 1.0, f'{lv:.1f}%', ha='center', va='bottom', fontsize=8)
    ax1.set_ylim(0, 112)
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, rotation=28, ha='right', fontsize=9)
    ax1.set_ylabel('Locked Ratio (% of detected frames)')
    ax1.set_title('Target Lock Retention by Model/Task')
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


def _plot_mode_charts(mode_rows: List[dict], mode_series: Dict[str, dict], charts_dir: str):
    if not mode_rows:
        return
    labels = [r['mode'] for r in mode_rows]
    locks = [r['locked_pct'] for r in mode_rows]
    switches = [r['id_switches'] for r in mode_rows]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.8))
    x = np.arange(len(labels))
    ax1.bar(x - 0.18, locks, width=0.35, label='Locked %', color='#0F9D58')
    ax1_r = ax1.twinx()
    ax1_r.bar(x + 0.18, switches, width=0.35, label='ID Switches', color='#DB4437')
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, rotation=15, ha='right', fontsize=9)
    ax1.set_ylabel('Locked %')
    ax1_r.set_ylabel('ID Switches')
    ax1.set_title('Tracking Algorithm Comparison')
    ax1.grid(axis='y', alpha=0.3)

    for name, s in mode_series.items():
        ax2.plot(s['t'], s['sim'], label=name, linewidth=1.2, alpha=0.85)
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


def _save_task_comparison_strips(task_frames_map: Dict[str, Dict[int, np.ndarray]], snap_dir: str):
    keys = [k for k in ('26n-det', '26n-seg', '26n-pose') if k in task_frames_map]
    if len(keys) < 2:
        return
    common_idxs = sorted(set.intersection(*(set(task_frames_map[k].keys()) for k in keys)))
    for fidx in common_idxs:
        imgs = [task_frames_map[k][fidx] for k in keys]
        h0 = imgs[0].shape[0]
        fitted = []
        for im in imgs:
            if im.shape[0] != h0:
                nw = int(round(im.shape[1] * (h0 / float(im.shape[0]))))
                im = cv2.resize(im, (nw, h0))
            fitted.append(im)
        strip = np.hstack(fitted)
        cv2.imwrite(os.path.join(snap_dir, f'compare_tasks_f{fidx:04d}.png'), strip)


def _write_markdown_report(
    path: str,
    bag_dir: str,
    topics: dict,
    total_frames: int,
    model_rows: List[dict],
    mode_rows: List[dict],
    loc_rows: List[dict],
):
    lines = [
        f'# Bag Benchmark Report: `{os.path.basename(bag_dir.rstrip("/"))}`',
        '',
        f'- **Bag Path**: `{bag_dir}`',
        f'- **Image Topic**: `{topics["image_topic"]}` (`{total_frames}` frames evaluated)',
        f'- **Scan / TF**: `{topics["scan_topic"] or "None"}` / `{topics["tf_topic"] or "None"}`',
        '',
    ]
    if model_rows:
        lines += [
            '## 1. Model & Task Comparison (`5:Adaptive`)',
            '',
            '| Model | Infer (ms) | Track+ReID (ms) | Total (ms) | Max FPS | Det % | Locked % | ID Sw | Avg Tgt Sim | Other Sim | Margin |',
            '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|',
        ]
        for r in model_rows:
            lines.append(
                f'| {r["run_name"]} (`{r["model"]}`) | {r["infer_ms"]} | {r["trk_reid_ms"]} | {r["total_ms"]} | '
                f'{r["max_fps"]} | {r["det_pct"]}% | {r["locked_pct"]}% | {r["id_switches"]} | '
                f'{r["avg_tgt_sim"]} | {r["avg_other_sim"]} | {r["sim_margin"]} |'
            )
        lines.append('')
    if mode_rows:
        lines += [
            '## 2. Tracking Algorithm Comparison (`yolo26n-pose`)',
            '',
            '| Mode | Locked % | SEARCH % | GUARD % | ID Sw | Recovered | Avg Tgt Sim | Total (ms) |',
            '|---|---:|---:|---:|---:|---:|---:|---:|',
        ]
        for r in mode_rows:
            lines.append(
                f'| {r["mode"]} | {r["locked_pct"]}% | {r["search_pct"]}% | {r["guard_pct"]}% | '
                f'{r["id_switches"]} | {r["recovered"]} | {r["avg_tgt_sim"]} | {r["total_ms"]} |'
            )
        lines.append('')
    if loc_rows:
        lines += [
            '## 3. 3D LiDAR Localizer Comparison (`Raw` vs `Kinematic+Map+EMA`)',
            '',
            '| Config | LiDAR | Occluded (Hold) | CamEst | LiDAR Jumps (>0.8m) | Far CamEst (>2.0m) | AMCL Wall/Out |',
            '|---|---:|---:|---:|---:|---:|---:|',
        ]
        for r in loc_rows:
            lines.append(
                f'| {r["run_name"]} | {r["lidar_cnt"]} | {r["occluded_cnt"]} | {r["camest_cnt"]} | '
                f'{r["lidar_jumps"]} | {r["far_camest"]} | {r["amcl_wall_out"]} |'
            )
        lines.append('')

    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))


def evaluate_bag(bag_dir: str, out_root: str, args):
    bag_dir = os.path.abspath(os.path.expanduser(bag_dir))
    bag_name = os.path.basename(bag_dir.rstrip('/'))
    bag_out = os.path.join(out_root, bag_name)
    charts_dir = os.path.join(bag_out, 'charts')
    snap_dir = os.path.join(bag_out, 'snapshots')
    os.makedirs(charts_dir, exist_ok=True)
    os.makedirs(snap_dir, exist_ok=True)

    storage_id = _detect_storage_id(bag_dir)
    topics = _auto_pick_topics(bag_dir, storage_id, custom_image_topic=args.image_topic)
    print(f'\n=== Bag: {bag_name} ({storage_id}) | img={topics["image_topic"]} scan={topics["scan_topic"] or "None"} ===')
    events, n_imgs = _load_bag_stream(
        bag_dir, storage_id, topics, max_frames=args.max_frames, stride=args.stride
    )
    print(f'Loaded {n_imgs} image frames ({len(events)} total messages).')

    reid_kwargs = dict(REID_SCALE_KWARGS) if args.reid_scale else {}
    lock_point = None
    if args.lock_point:
        px, py = args.lock_point.split(',')
        lock_point = (int(px), int(py))

    # 모델 목록 해석
    if args.models in PRESETS:
        model_keys = PRESETS[args.models]
    else:
        model_keys = [k.strip() for k in args.models.split(',') if k.strip() in MODEL_SPECS]

    # Det/Seg/Pose 동일 프레임 비교용 프레임 번호 2개 선정
    cap_frames = {max(1, n_imgs // 4), max(2, n_imgs // 2)} if (args.save_snapshots and n_imgs >= 4) else set()

    model_rows = []
    task_frames_map: Dict[str, Dict[int, np.ndarray]] = {}
    for mk in model_keys:
        f_idx, t_idx = MODEL_SPECS[mk]
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
            save_snapshots=(args.save_snapshots and mk == '26n-pose'),
            snap_dir=snap_dir,
            capture_frame_indices=cap_frames if mk in ('26n-det', '26n-seg', '26n-pose') else None,
        )
        model_rows.append(row)
        if s['task_frames']:
            task_frames_map[mk] = s['task_frames']

    _save_task_comparison_strips(task_frames_map, snap_dir)

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
            save_snapshots=False,
            snap_dir=snap_dir,
        )
        mode_rows.append(row)
        mode_series[m_label] = s

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
                save_snapshots=False,
                snap_dir=snap_dir,
            )
            loc_rows.append(row)
            loc_series[lbl] = s

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
    )
    _plot_model_charts(model_rows, charts_dir)
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
    parser.add_argument('--reid-scale', action='store_true', help='follow_test REID_SCALE 완화 파라미터 적용')
    parser.add_argument('--no-localizer-compare', dest='compare_localizer', action='store_false')
    parser.add_argument('--no-snapshots', dest='save_snapshots', action='store_false')
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
