async function request(path, method, body, options = {}) {
  let response;
  const headers = { 'Content-Type': 'application/json' };
  if (path.startsWith('/api/dev/')) {
    const statusResponse = await fetch('/api/auth/status');
    const auth = statusResponse.ok ? await statusResponse.json() : {};
    if (!auth.authenticated) throw new Error('개발자 제어는 관리자 로그인 후 사용할 수 있어요.');
    headers['X-CSRF-Token'] = auth.csrf_token;
  }
  try {
    response = await fetch(path, {
      ...options,
      method,
      headers,
      body: JSON.stringify(body),
    });
  } catch {
    throw new Error('로컬 서버에 연결할 수 없어요. 서버 실행 상태를 확인해 주세요.');
  }
  const data = await response.json();
  if (!response.ok) throw new Error(data.error?.message || '요청을 처리하지 못했어요.');
  return data;
}

export const createSession = body => request('/api/sessions', 'POST', body);
export const startSession = session => request(`/api/sessions/${session.id}`, 'PATCH', { status: 'active' });
export const cancelSession = (session, visitedIds) => request(`/api/sessions/${session.id}`, 'PATCH', { status: 'cancelled', visitedIds });
export const injectDevPreparationReady = session => request(`/api/dev/preparation/${encodeURIComponent(session.id)}/ready`, 'POST', {});

export const getSession = id => request(`/api/sessions/${encodeURIComponent(id)}`, 'GET');
export const beginPreparation = session => request(`/api/sessions/${session.id}/preparation`, 'POST', {});
export const registerVisitor = session => request(`/api/sessions/${session.id}/preparation/register`, 'POST', {});
export const cancelPreparation = session => request(`/api/sessions/${session.id}/preparation/cancel`, 'POST', {});
