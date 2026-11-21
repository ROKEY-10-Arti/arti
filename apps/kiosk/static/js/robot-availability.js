// Flask가 로봇 배터리·도킹 상태로 계산한 호출 가능 여부를 1초마다 전달합니다.
export function subscribeRobotAvailability(onChange, {
  EventSourceImpl = globalThis.EventSource,
  now = Date.now,
  every = globalThis.setInterval,
  cancelEvery = globalThis.clearInterval,
  staleAfter = 3500,
} = {}) {
  let stopped = false;
  let lastFrameAt = now();
  let latest = {status:'waiting', available:null};
  const emit = snapshot => {
    if (stopped) return;
    latest = snapshot;
    onChange(snapshot);
  };
  emit(latest);
  const source = new EventSourceImpl('/api/availability/stream');
  source.addEventListener('availability', event => {
    try {
      const snapshot = JSON.parse(event.data);
      if (!['waiting','available','unavailable','stale','disconnected'].includes(snapshot.status)
          || (snapshot.status === 'available' && snapshot.available !== true)
          || (snapshot.status === 'unavailable' && snapshot.available !== false)
          || (!['available','unavailable'].includes(snapshot.status) && snapshot.available !== null)) {
        throw new Error('Invalid availability event');
      }
      lastFrameAt = now();
      emit({status:snapshot.status, available:snapshot.available});
    } catch {
      emit({status:'disconnected', available:null});
    }
  });
  source.addEventListener('error', () => emit({status:'disconnected', available:null}));
  const watchdog = every(() => {
    if (now() - lastFrameAt > staleAfter && latest.status !== 'disconnected') {
      emit({status:'stale', available:null});
    }
  }, 1000);
  return () => {
    stopped = true;
    source.close();
    cancelEvery(watchdog);
  };
}
