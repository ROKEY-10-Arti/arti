"""로컬 웹캠 열기·크기 맞추기 (config/phone_camera.yaml 의 capture 묶음).

넓은 모드로 받고 바로 줄여 쓴다: 노트북 웹캠은 4:3(640x480) 모드에서 센서 좌우를 잘라 쓰는 경우가 많아
16:9(1280x720)로 받으면 좌우 화각이 넓어질 수 있다. 받자마자 out_width로 줄이므로(비율 유지, 예 640x360)
YOLO·전송 부담은 640x480과 비슷하다 (2026-11-21).
- 720p를 YUYV로 받으면 USB 대역폭 때문에 5~10 fps로 떨어지는 웹캠이 많아 MJPG로 요청한다.
- 모드를 바꾸면 camera_intrinsics.hfov_deg 도 그 모드에서 잰 값으로 바꿔야 한다 (방향 계산이 어긋남).
"""
from typing import Union

import cv2

from person_tracker.core.env_bootstrap import load_phone_camera_config

DEFAULT = {'width': 640, 'height': 480, 'fourcc': '', 'out_width': 640}


def capture_config() -> dict:
    cfg = dict(DEFAULT)
    try:
        cfg.update((load_phone_camera_config() or {}).get('capture') or {})
    except Exception:
        pass
    return cfg


def open_webcam(src: Union[int, str], logger=None):
    """웹캠을 열고 capture 설정(fourcc, width, height)을 적용한다. 장치(int, /dev/video*)는 V4L2 우선.
    돌려줌: (cv2.VideoCapture, cfg)"""
    cfg = capture_config()
    local = isinstance(src, int) or str(src).startswith('/dev/video')
    cap = cv2.VideoCapture(src, cv2.CAP_V4L2) if local else cv2.VideoCapture(src)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(src)
    if cap.isOpened() and local:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if cfg.get('fourcc'):
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*str(cfg['fourcc'])[:4]))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cfg['width']))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cfg['height']))
        if logger is not None:
            got = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            fcc = int(cap.get(cv2.CAP_PROP_FOURCC))
            fcc = ''.join(chr((fcc >> 8 * i) & 0xFF) for i in range(4)) if fcc else '?'
            msg = (f'[웹캠] 요청 {cfg["width"]}x{cfg["height"]} {cfg.get("fourcc") or "기본"} → 실제 {got[0]}x{got[1]} {fcc}, '
                   f'{cap.get(cv2.CAP_PROP_FPS):.0f} fps, 받은 뒤 가로 {cfg["out_width"]}로 줄임')
            if got != (int(cfg['width']), int(cfg['height'])):
                logger.warning(msg + ' (요청한 모드 없음 → hfov_deg가 이 모드 값인지 확인)')
            else:
                logger.info(msg)
    return cap, cfg


def fit_frame(frame, out_width: int):
    """가로를 out_width로 줄인다 (가로세로 비율 유지). 이미 그 폭이면 그대로."""
    h, w = frame.shape[:2]
    out_width = int(out_width)
    if out_width <= 0 or w == out_width:
        return frame
    out_h = int(round(h * out_width / w))
    interp = cv2.INTER_AREA if out_width < w else cv2.INTER_LINEAR
    return cv2.resize(frame, (out_width, out_h), interpolation=interp)
