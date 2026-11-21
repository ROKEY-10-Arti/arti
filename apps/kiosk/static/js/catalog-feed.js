// 화면 조작 없이 DB의 최신 카탈로그를 조회합니다. 요청은 겹치지 않습니다.
export function watchCatalog({refresh, onUpdate, onError = () => {}, interval = 3000, immediate = false,
  documentImpl = document, setTimeoutImpl = setTimeout, clearTimeoutImpl = clearTimeout}) {
  let stopped = false;
  let timer = null;
  let controller = null;
  let running = false;
  let retryDelay = interval;
  const schedule = () => {
    if (!stopped && !documentImpl.hidden) timer = setTimeoutImpl(poll, retryDelay);
  };
  async function poll() {
    timer = null;
    if (stopped || running || documentImpl.hidden) return;
    running = true;
    controller = new AbortController();
    const timeout = setTimeoutImpl(() => controller?.abort(), 5000);
    try {
      const changed = await refresh({signal: controller.signal});
      retryDelay = interval;
      if (!stopped && !documentImpl.hidden) onUpdate(changed);
    } catch {
      if (!stopped && !documentImpl.hidden) {
        retryDelay = Math.min(retryDelay * 2, Math.max(interval, 30000));
        onError();
      }
    } finally {
      clearTimeoutImpl(timeout);
      controller = null;
      running = false;
      schedule();
    }
  }
  function visibilityChanged() {
    clearTimeoutImpl(timer);
    timer = null;
    if (documentImpl.hidden) controller?.abort();
    else if (!running && !stopped) void poll();
  }
  documentImpl.addEventListener('visibilitychange', visibilityChanged);
  if (immediate) void poll();
  else schedule();
  return () => {
    stopped = true;
    clearTimeoutImpl(timer);
    controller?.abort();
    documentImpl.removeEventListener('visibilitychange', visibilityChanged);
  };
}
