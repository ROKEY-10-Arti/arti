#!/usr/bin/env python3
"""
Human-in-the-Loop Ground Truth (GT) 클릭 교정 어노테이터

ROS 2 Bag을 재생하면서 보조 트래커(기본 YOLO26n-seg + Adaptive)가 타겟을 따라가도록 두고,
트래커가 오인식(Hijack)하거나 타겟을 놓칠 때 사용자가 실제 타겟을 마우스로 클릭해
즉시 교정함으로써 정량 평가용 GT CSV(`gt_present, x1, y1, x2, y2, corrected`)를 생성합니다.

조작법:
  - Left-Click (클릭)        : 클릭한 위치의 검출 인물을 실제 타겟(GT)으로 지정 + 보조 트래커 즉시 재잠금
  - Left-Drag  (드래그>=12px): 미검출 프레임 등에서 수동 GT BBox 직접 그리기
  - Right-Click 또는 N       : 현재 프레임 타겟 부재/완전 가림(gt_present=0) 표시
  - Space                    : 재생 / 일시정지 토글 (프레임 1에서 일시정지 상태로 시작)
  - A / D (또는 Left/Right)  : 이전 / 다음 프레임 이동
  - C                        : 보조 트래커 타겟 잠금 해제 (Anchor 초기화)
  - S                        : 현재까지 GT CSV 즉시 저장
  - Q / ESC                  : GT CSV 저장 후 종료

사용 예시:
  python3 src/person_tracker/research/gt_annotator.py \
      --bag ~/bag3/pose_estimate2 \
      --out ~/bench_gt/pose_estimate2_gt.csv
"""

import argparse
import csv
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.dirname(_THIS_DIR)
if _PKG_DIR not in sys.path:
    sys.path.insert(0, _PKG_DIR)
if _THIS_DIR not in sys.path:
    sys.path.insert(0, _THIS_DIR)

from person_tracker.core.env_bootstrap import bootstrap_venv
bootstrap_venv()

import cv2
import numpy as np

from person_tracker.core.multi_tracker_core import MultiModelTrackerCore, UnifiedTrackedPerson
from bag_benchmark import (
    MODEL_SPECS,
    REID_SCALE_KWARGS,
    _auto_pick_topics,
    _decode_image,
    _detect_storage_id,
    _load_bag_stream,
)


def _point_in_bbox(x: int, y: int, bbox: Tuple[int, int, int, int]) -> bool:
    x1, y1, x2, y2 = bbox
    return x1 <= x <= x2 and y1 <= y <= y2


def _find_det_near_point(
    dets: List[UnifiedTrackedPerson],
    x: int,
    y: int,
    max_dist_px: float = 80.0,
) -> Optional[UnifiedTrackedPerson]:
    """클릭 좌표를 포함하는 BBox(가장 작은 면적 우선) 또는 반경 내 가장 가까운 검출을 반환합니다."""
    containing = [p for p in dets if _point_in_bbox(x, y, p.bbox)]
    if containing:
        return min(
            containing,
            key=lambda p: (p.bbox[2] - p.bbox[0]) * (p.bbox[3] - p.bbox[1]),
        )
    if not dets:
        return None
    best = min(
        dets,
        key=lambda p: ((p.center[0] - x) ** 2 + (p.center[1] - y) ** 2) ** 0.5,
    )
    dist = ((best.center[0] - x) ** 2 + (best.center[1] - y) ** 2) ** 0.5
    return best if dist <= max_dist_px else None


def load_gt_csv(path: str) -> Dict[int, dict]:
    """저장된 GT CSV를 frame_idx -> dict 매핑으로 로드합니다."""
    out: Dict[int, dict] = {}
    if not os.path.exists(path):
        return out
    with open(path, 'r', newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
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
            out[fidx] = {
                'frame_idx': fidx,
                't_sec': float(row.get('t_sec', 0.0) or 0.0),
                'gt_present': pres,
                'bbox': bbox,
                'corrected': int(row.get('corrected', 0) or 0),
                'source': row.get('source', 'unlabeled') or 'unlabeled',
                'helper_track_id': int(row.get('helper_track_id', -1) or -1),
            }
    return out


def save_gt_csv(
    path: str,
    image_events: List[Tuple[float, object]],
    annotations: Dict[int, dict],
) -> dict:
    """모든 이미지 프레임 기준으로 정렬된 GT CSV를 저장하고 요약 통계를 반환합니다."""
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    fieldnames = [
        'frame_idx',
        't_sec',
        'gt_present',
        'x1',
        'y1',
        'x2',
        'y2',
        'corrected',
        'source',
        'helper_track_id',
    ]
    stats = {
        'total_frames': len(image_events),
        'labeled': 0,
        'present': 0,
        'absent': 0,
        'human_corrections': 0,
        'tracker_auto': 0,
    }
    rows = []
    for idx0, (rel_sec, _) in enumerate(image_events):
        fidx = idx0 + 1
        ann = annotations.get(fidx)
        if ann is None or ann.get('source', 'unlabeled') == 'unlabeled':
            rows.append({
                'frame_idx': fidx,
                't_sec': round(rel_sec, 4),
                'gt_present': 0,
                'x1': '',
                'y1': '',
                'x2': '',
                'y2': '',
                'corrected': 0,
                'source': 'unlabeled',
                'helper_track_id': -1,
            })
            continue
        pres = int(ann['gt_present'])
        bbox = ann.get('bbox') if pres == 1 else None
        corr = int(ann.get('corrected', 0))
        src = ann.get('source', 'tracker')
        stats['labeled'] += 1
        if pres == 1:
            stats['present'] += 1
        else:
            stats['absent'] += 1
        if corr == 1:
            stats['human_corrections'] += 1
        if src == 'tracker':
            stats['tracker_auto'] += 1
        rows.append({
            'frame_idx': fidx,
            't_sec': round(rel_sec, 4),
            'gt_present': pres,
            'x1': bbox[0] if bbox is not None else '',
            'y1': bbox[1] if bbox is not None else '',
            'x2': bbox[2] if bbox is not None else '',
            'y2': bbox[3] if bbox is not None else '',
            'corrected': corr,
            'source': src,
            'helper_track_id': int(ann.get('helper_track_id', -1)),
        })

    with open(path, 'w', newline='', encoding='utf-8') as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)
    return stats


def _render_ui_frame(
    frame: np.ndarray,
    fidx: int,
    total_frames: int,
    rel_sec: float,
    dets: List[UnifiedTrackedPerson],
    tgt: Optional[UnifiedTrackedPerson],
    ann: Optional[dict],
    paused: bool,
    drag_rect: Optional[Tuple[int, int, int, int]],
    tracker_state: str,
    corrections_cnt: int,
) -> np.ndarray:
    vis = frame.copy()
    h, w = vis.shape[:2]

    # 보조 트래커 검출 BBox 표시
    for p in dets:
        x1, y1, x2, y2 = p.bbox
        if tgt is not None and p.track_id == tgt.track_id:
            cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 200, 255), 2)
            cv2.putText(
                vis,
                f'TRK#{p.track_id} sim:{p.similarity:.2f}',
                (x1, max(54, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (0, 200, 255),
                2,
                cv2.LINE_AA,
            )
        else:
            cv2.rectangle(vis, (x1, y1), (x2, y2), (160, 160, 160), 1)
            cv2.putText(
                vis,
                f'ID:{p.track_id}',
                (x1, max(54, y1 - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (190, 190, 190),
                1,
                cv2.LINE_AA,
            )

    # 현재 GT 지정 상태 오버레이 (굵은 초록색 또는 빨강 ABSENT 배지)
    gt_status_str = 'UNLABELED'
    if ann is not None and ann.get('source', 'unlabeled') != 'unlabeled':
        if ann['gt_present'] == 1 and ann.get('bbox') is not None:
            gx1, gy1, gx2, gy2 = ann['bbox']
            src = ann.get('source', 'tracker')
            col = (0, 255, 0) if ann.get('corrected') else (80, 230, 120)
            cv2.rectangle(vis, (gx1, gy1), (gx2, gy2), col, 3)
            cv2.putText(
                vis,
                f'GT [{src}]',
                (gx1, min(h - 10, gy2 + 18)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                col,
                2,
                cv2.LINE_AA,
            )
            gt_status_str = f'GT:PRESENT ({src})'
        elif ann['gt_present'] == 0:
            gt_status_str = 'GT:ABSENT (occluded/out)'
            cv2.putText(
                vis,
                'GT: TARGET ABSENT',
                (12, h - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (60, 80, 255),
                2,
                cv2.LINE_AA,
            )

    if drag_rect is not None:
        dx1, dy1, dx2, dy2 = drag_rect
        cv2.rectangle(vis, (dx1, dy1), (dx2, dy2), (255, 0, 255), 2)

    # 상단 HUD 배너
    cv2.rectangle(vis, (0, 0), (w, 48), (20, 20, 20), -1)
    state_lbl = 'PAUSED' if paused else 'PLAY'
    top_txt = f'[{state_lbl}] f {fidx}/{total_frames} ({rel_sec:.1f}s) | TRK:{tracker_state} | {gt_status_str} | Clicks:{corrections_cnt}'
    cv2.putText(vis, top_txt, (8, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 255), 1, cv2.LINE_AA)
    help_txt = 'L-Click:Select GT  L-Drag:Box  R-Click/N:Absent  Space:Play/Pause  A/D:Step  C:ClearTrk  S:Save  Q:Save&Quit'
    cv2.putText(vis, help_txt, (8, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.37, (200, 220, 200), 1, cv2.LINE_AA)
    return vis


def run_annotator(args):
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        raise RuntimeError(
            'GUI 디스플레이(DISPLAY/WAYLAND_DISPLAY)가 감지되지 않았습니다. '
            '로컬 X11/Wayland 환경이나 X-forwarding 환경에서 실행해 주세요.'
        )

    bag_dir = os.path.abspath(os.path.expanduser(args.bag))
    bag_name = os.path.basename(bag_dir.rstrip('/'))
    out_csv = (
        os.path.abspath(os.path.expanduser(args.out))
        if args.out
        else os.path.expanduser(f'~/bench_gt/{bag_name}_gt.csv')
    )

    storage_id = _detect_storage_id(bag_dir)
    topics = _auto_pick_topics(bag_dir, storage_id, custom_image_topic=args.image_topic)
    events, n_imgs = _load_bag_stream(
        bag_dir, storage_id, topics, max_frames=args.max_frames, stride=args.stride
    )
    image_events = [(rel_sec, msg) for kind, rel_sec, msg in events if kind == 'image']
    if not image_events:
        raise RuntimeError(f'이미지 토픽 프레임을 찾지 못했습니다: {bag_dir}')

    is_comp = 'CompressedImage' in topics['image_type']
    f_idx, t_idx = MODEL_SPECS[args.model]
    reid_kwargs = dict(REID_SCALE_KWARGS) if args.reid_scale else {}
    tracker = MultiModelTrackerCore(
        family_idx=f_idx,
        task_idx=t_idx,
        mode=args.mode,
        imgsz=320,
        device=args.device,
        **reid_kwargs,
    )

    annotations: Dict[int, dict] = {}
    if args.resume and os.path.exists(os.path.expanduser(args.resume)):
        annotations = load_gt_csv(os.path.expanduser(args.resume))
        print(f'Resumed {len(annotations)} frame records from {args.resume}')

    # 프레임별 최신 검출·디코드 캐시
    frame_cache: Dict[int, Tuple[np.ndarray, List[UnifiedTrackedPerson], Optional[UnifiedTrackedPerson]]] = {}
    max_processed_fidx = 0

    def _ensure_frame(fidx: int):
        nonlocal max_processed_fidx
        if fidx in frame_cache:
            return frame_cache[fidx]
        rel_sec, msg = image_events[fidx - 1]
        img = _decode_image(msg, is_comp)
        if img is None:
            raise RuntimeError(f'프레임 {fidx} 이미지 디코딩 실패')
        dets, tgt = tracker.process_frame(
            img,
            auto_lock_first=False,
            compute_all_sims=True,
        )
        max_processed_fidx = max(max_processed_fidx, fidx)
        frame_cache[fidx] = (img, dets, tgt)
        # 메모리 보호: 오래된 디코드 이미지는 400장 초과 시 정리
        if len(frame_cache) > 400:
            oldest = min(k for k in frame_cache if k != fidx)
            frame_cache.pop(oldest, None)
        return frame_cache[fidx]

    def _auto_propagate_from_tracker(fidx: int, tgt: Optional[UnifiedTrackedPerson], rel_sec: float):
        """사용자가 아직 수동 교정하지 않은 프레임에만 보조 트래커 결과를 기록합니다."""
        existing = annotations.get(fidx)
        if existing is not None and existing.get('corrected', 0) == 1:
            return
        if tgt is not None:
            annotations[fidx] = {
                'frame_idx': fidx,
                't_sec': rel_sec,
                'gt_present': 1,
                'bbox': tuple(int(v) for v in tgt.bbox),
                'corrected': 0,
                'source': 'tracker',
                'helper_track_id': int(tgt.track_id),
            }
        elif tracker.target_id is not None or any(
            v.get('gt_present') == 1 for v in annotations.values()
        ):
            # 이미 타겟을 한번 지정한 뒤에 놓쳤다면 일단 unlabeled로 두거나 이전 상태 표시
            if existing is None:
                annotations[fidx] = {
                    'frame_idx': fidx,
                    't_sec': rel_sec,
                    'gt_present': 0,
                    'bbox': None,
                    'corrected': 0,
                    'source': 'tracker',
                    'helper_track_id': -1,
                }

    cur_fidx = 1
    paused = True
    drag_start: Optional[Tuple[int, int]] = None
    drag_cur: Optional[Tuple[int, int]] = None

    win_name = f'Arti GT Annotator - {bag_name} ({args.model})'
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

    def _on_mouse(event, x, y, flags, param):
        nonlocal drag_start, drag_cur
        img, dets, tgt = _ensure_frame(cur_fidx)
        rel_sec, _ = image_events[cur_fidx - 1]
        if event == cv2.EVENT_LBUTTONDOWN:
            drag_start = (x, y)
            drag_cur = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and drag_start is not None:
            drag_cur = (x, y)
        elif event == cv2.EVENT_LBUTTONUP and drag_start is not None:
            sx, sy = drag_start
            drag_start = None
            drag_cur = None
            if abs(x - sx) >= 12 and abs(y - sy) >= 12:
                x1, x2 = min(sx, x), max(sx, x)
                y1, y2 = min(sy, y), max(sy, y)
                annotations[cur_fidx] = {
                    'frame_idx': cur_fidx,
                    't_sec': rel_sec,
                    'gt_present': 1,
                    'bbox': (int(x1), int(y1), int(x2), int(y2)),
                    'corrected': 1,
                    'source': 'manual_box',
                    'helper_track_id': -1,
                }
                # 드래그 박스 중심에 검출이 있으면 보조 트래커도 함께 재잠금
                cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                det = _find_det_near_point(dets, cx, cy, max_dist_px=60.0)
                if det is not None:
                    tracker.select_target_by_id(det.track_id)
            else:
                det = _find_det_near_point(dets, x, y)
                if det is not None:
                    tracker.select_target_by_id(det.track_id)
                    det.is_target = True
                    annotations[cur_fidx] = {
                        'frame_idx': cur_fidx,
                        't_sec': rel_sec,
                        'gt_present': 1,
                        'bbox': tuple(int(v) for v in det.bbox),
                        'corrected': 1,
                        'source': 'click',
                        'helper_track_id': int(det.track_id),
                    }
                    # cache 갱신
                    frame_cache[cur_fidx] = (img, dets, det)
        elif event == cv2.EVENT_RBUTTONDOWN:
            annotations[cur_fidx] = {
                'frame_idx': cur_fidx,
                't_sec': rel_sec,
                'gt_present': 0,
                'bbox': None,
                'corrected': 1,
                'source': 'absent',
                'helper_track_id': -1,
            }

    cv2.setMouseCallback(win_name, _on_mouse)

    frame_interval = 1.0 / max(1.0, float(args.fps))
    last_step_t = time.perf_counter()

    try:
        while True:
            img, dets, tgt = _ensure_frame(cur_fidx)
            rel_sec, _ = image_events[cur_fidx - 1]
            _auto_propagate_from_tracker(cur_fidx, tgt, rel_sec)

            drag_rect = None
            if drag_start is not None and drag_cur is not None:
                drag_rect = (
                    min(drag_start[0], drag_cur[0]),
                    min(drag_start[1], drag_cur[1]),
                    max(drag_start[0], drag_cur[0]),
                    max(drag_start[1], drag_cur[1]),
                )

            corr_cnt = sum(1 for v in annotations.values() if v.get('corrected', 0) == 1)
            vis = _render_ui_frame(
                img,
                cur_fidx,
                len(image_events),
                rel_sec,
                dets,
                tgt,
                annotations.get(cur_fidx),
                paused,
                drag_rect,
                tracker.metrics.adaptive_state,
                corr_cnt,
            )
            cv2.imshow(win_name, vis)

            key = cv2.waitKey(15 if not paused else 30) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord(' '):
                paused = not paused
                last_step_t = time.perf_counter()
            elif key in (ord('d'), 83):  # D or RightArrow
                paused = True
                cur_fidx = min(len(image_events), cur_fidx + 1)
            elif key in (ord('a'), 81):  # A or LeftArrow
                paused = True
                cur_fidx = max(1, cur_fidx - 1)
            elif key == ord('n'):
                annotations[cur_fidx] = {
                    'frame_idx': cur_fidx,
                    't_sec': rel_sec,
                    'gt_present': 0,
                    'bbox': None,
                    'corrected': 1,
                    'source': 'absent',
                    'helper_track_id': -1,
                }
                if paused and cur_fidx < len(image_events):
                    cur_fidx += 1
            elif key == ord('c'):
                tracker.select_target_by_id(None)
            elif key == ord('s'):
                st = save_gt_csv(out_csv, image_events, annotations)
                print(f'[Saved] {out_csv} -> {st}')

            if not paused:
                now = time.perf_counter()
                if now - last_step_t >= frame_interval:
                    last_step_t = now
                    if cur_fidx < len(image_events):
                        cur_fidx += 1
                    else:
                        paused = True
    finally:
        cv2.destroyAllWindows()

    stats = save_gt_csv(out_csv, image_events, annotations)
    print(f'GT saved to {out_csv}: {stats}')


def main():
    parser = argparse.ArgumentParser(description='Human-in-the-Loop 클릭 교정 GT 어노테이터')
    parser.add_argument('--bag', required=True, help='어노테이션할 ROS 2 bag 폴더 경로')
    parser.add_argument('--out', default='', help='저장할 GT CSV 경로 (기본: ~/bench_gt/<bag>_gt.csv)')
    parser.add_argument('--resume', default='', help='이어서 수정할 기존 GT CSV 경로')
    parser.add_argument('--model', default='26n-seg', choices=list(MODEL_SPECS.keys()), help='보조 트래커 모델')
    parser.add_argument('--mode', type=int, default=5, help='보조 트래커 알고리즘 모드 (기본 5:Adaptive)')
    parser.add_argument('--device', default='', help='추론 장치 (cuda:0 / cpu 등)')
    parser.add_argument('--fps', type=float, default=20.0, help='재생 속도 제한 (FPS)')
    parser.add_argument('--max-frames', type=int, default=0, help='앞 N프레임만 로드 (0=전체)')
    parser.add_argument('--stride', type=int, default=1, help='프레임 샘플링 간격')
    parser.add_argument('--image-topic', default='', help='이미지 토픽 수동 지정')
    parser.add_argument('--no-reid-scale', dest='reid_scale', action='store_false', help='REID_SCALE 완화 끄기')
    args = parser.parse_args()
    run_annotator(args)


if __name__ == '__main__':
    main()
