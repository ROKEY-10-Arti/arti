"""웹캠·DroidCam을 토픽 없이 노드 안에서 직접 읽는다 (multi_tracker_node용, BaseTrackerNode와 같은 방식).

video_source  '0', '2' 등 = 웹캠 번호(V4L2) / 'http://.../video' = DroidCam URL
읽기는 별도 스레드가 계속 하고, take()는 가장 최근 프레임 하나만 꺼낸다 (밀린 프레임은 버림 → 지연 없음).
영상은 토픽으로 내보내지 않는다 (관람객 영상이 네트워크·bag에 남지 않음).
"""
import threading
import time
from typing import Optional, Tuple, Union

import cv2
import numpy as np

FRAME_W, FRAME_H = 640, 480


def parse_source(raw) -> Union[int, str, None]:
    """파라미터 값 → cv2.VideoCapture 인자. 빈 값이면 None (= 직접 읽지 않고 토픽 구독)."""
    s = str(raw).strip()
    if not s:
        return None
    return int(s) if s.isdigit() else s


class LocalCamera:
    def __init__(self, src: Union[int, str], logger):
        self.src, self.log = src, logger
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._stop = threading.Event()
        cap = cv2.VideoCapture(src, cv2.CAP_V4L2) if isinstance(src, int) else cv2.VideoCapture(src)
        if not cap.isOpened():
            cap.release()
            cap = cv2.VideoCapture(src)
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
            self.log.info(f'카메라 직접 읽기: {src}')
        else:
            self.log.error(f'카메라를 열 수 없습니다: {src} (다른 프로세스가 쓰는 중인지 확인: camera_pub 등)')
        self._cap = cap
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            if not self._cap.isOpened():
                time.sleep(0.2)
                continue
            ok, frame = self._cap.read()
            if ok and frame is not None:
                if frame.shape[1] != FRAME_W or frame.shape[0] != FRAME_H:
                    frame = cv2.resize(frame, (FRAME_W, FRAME_H), interpolation=cv2.INTER_AREA)
                with self._lock:
                    self._frame = frame
            else:
                time.sleep(0.02)

    def take(self) -> Optional[np.ndarray]:
        """새 프레임이 있으면 꺼내고(한 번만), 없으면 None."""
        with self._lock:
            frame, self._frame = self._frame, None
        return frame

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._cap.release()
