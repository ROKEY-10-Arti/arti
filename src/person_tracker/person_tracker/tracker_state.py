"""
ROS 2 비의존 선택적 인식 상태 스냅샷 및 이벤트 콜백 저장소 (tracker_state.py)

- 동일 프로세스 내 `import` 연동(주행/키오스크/테스트 모듈) 시 토픽 직렬화 없이
  최신 상태(`TrackerState`) 조회 및 이벤트 콜백(`on_target_registered` 등) 실행을 지원합니다.
- 부하를 피하고 싶은 실전 경량 모드에서는 `enabled=False`로 두면 모든 갱신/콜백이
  즉시 no-op으로 동작해 추가 CPU/메모리 비용이 발생하지 않습니다.
"""

from dataclasses import asdict, dataclass
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple


@dataclass(frozen=True)
class TrackerState:
    """인식·추적 모듈의 불변(읽기 전용) 상태 스냅샷."""

    phase: str = 'IDLE'
    target_id: Optional[int] = None
    target_visible: bool = False
    adaptive_state: str = 'IDLE'
    ema_status: str = 'IDLE'
    similarity: float = 0.0
    visible_parts: Tuple[bool, bool, bool] = (False, False, False)
    base_xyz: Optional[Tuple[float, float, float]] = None
    map_xyz: Optional[Tuple[float, float, float]] = None
    loc_source: str = ''
    sync_dt_ms: Optional[float] = None
    fps: float = 0.0
    updated_at: float = 0.0

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


class TrackerStateStore:
    """스레드 안전한 선택적 상태 저장소 및 이벤트 훅 디스패처."""

    VALID_EVENTS = (
        'on_phase_changed',
        'on_target_registered',
        'on_target_lost',
        'on_target_recovered',
        'on_target_localized',
        'on_tour_reset',
        'on_state_updated',
    )

    def __init__(self, enabled: bool = True):
        self.enabled = bool(enabled)
        self._lock = threading.Lock()
        self._state = TrackerState()
        self._hooks: Dict[str, List[Callable[..., None]]] = {
            ev: [] for ev in self.VALID_EVENTS
        }

    @property
    def state(self) -> TrackerState:
        with self._lock:
            return self._state

    def register_hook(self, event_name: str, callback: Callable[..., None]):
        if event_name not in self._hooks:
            raise ValueError(f'Unknown tracker event: {event_name}')
        with self._lock:
            if callback not in self._hooks[event_name]:
                self._hooks[event_name].append(callback)

    def unregister_hook(self, event_name: str, callback: Callable[..., None]):
        if event_name not in self._hooks:
            return
        with self._lock:
            if callback in self._hooks[event_name]:
                self._hooks[event_name].remove(callback)

    def _get_hooks(self, event_name: str) -> List[Callable[..., None]]:
        with self._lock:
            return list(self._hooks[event_name])

    def update(
        self,
        *,
        phase: str,
        target_id: Optional[int],
        target_visible: bool,
        adaptive_state: str,
        ema_status: str,
        similarity: float,
        visible_parts: Tuple[bool, bool, bool],
        base_xyz: Optional[Tuple[float, float, float]],
        map_xyz: Optional[Tuple[float, float, float]],
        loc_source: str,
        sync_dt_ms: Optional[float],
        fps: float,
        registered_now: bool = False,
    ):
        """새 상태 스냅샷을 갱신하고 전이된 이벤트에 등록된 콜백을 호출합니다."""
        if not self.enabled:
            return

        new_state = TrackerState(
            phase=phase,
            target_id=target_id,
            target_visible=target_visible,
            adaptive_state=adaptive_state,
            ema_status=ema_status,
            similarity=float(similarity),
            visible_parts=visible_parts,
            base_xyz=base_xyz,
            map_xyz=map_xyz,
            loc_source=loc_source,
            sync_dt_ms=sync_dt_ms,
            fps=float(fps),
            updated_at=time.time(),
        )

        with self._lock:
            prev = self._state
            self._state = new_state
            hooks_snapshot = {k: list(v) for k, v in self._hooks.items() if v}

        if not hooks_snapshot:
            return

        if prev.phase != new_state.phase:
            for cb in hooks_snapshot.get('on_phase_changed', ()):
                cb(prev.phase, new_state.phase, new_state)

        if registered_now or (prev.target_id is None and new_state.target_id is not None):
            for cb in hooks_snapshot.get('on_target_registered', ()):
                cb(new_state)

        if prev.target_id is not None and new_state.target_id is not None:
            if prev.target_visible and not new_state.target_visible:
                for cb in hooks_snapshot.get('on_target_lost', ()):
                    cb(new_state)
            elif not prev.target_visible and new_state.target_visible:
                for cb in hooks_snapshot.get('on_target_recovered', ()):
                    cb(new_state)

        if new_state.base_xyz is not None or new_state.map_xyz is not None:
            for cb in hooks_snapshot.get('on_target_localized', ()):
                cb(new_state)

        for cb in hooks_snapshot.get('on_state_updated', ()):
            cb(new_state)

    def reset(self, reason: str = ''):
        """타겟 초기화 시 상태를 `IDLE`로 되돌리고 `on_tour_reset` 콜백을 실행합니다."""
        if not self.enabled:
            return
        new_state = TrackerState(updated_at=time.time())
        with self._lock:
            prev = self._state
            self._state = new_state
            hooks_snapshot = {k: list(v) for k, v in self._hooks.items() if v}

        if prev.phase != new_state.phase:
            for cb in hooks_snapshot.get('on_phase_changed', ()):
                cb(prev.phase, new_state.phase, new_state)
        for cb in hooks_snapshot.get('on_tour_reset', ()):
            cb(reason, new_state)
        for cb in hooks_snapshot.get('on_state_updated', ()):
            cb(new_state)
