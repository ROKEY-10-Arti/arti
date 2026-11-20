export const escape = value => String(value ?? '').replace(/[&<>"']/g, char => ({
  '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;',
}[char]));
export const formatTime = value => value == null ? '—' : new Date(value).toLocaleString('ko-KR', {timeZone:'Asia/Seoul', hour12:false}) + ' KST';
export function numericValue(value) {
  if (value === '' || value == null) return null;
  const number = Number(value);
  if (!Number.isFinite(number) || Math.abs(number) > 1e12) throw new Error('좌표는 ±1e12 범위의 숫자로 입력해 주세요.');
  return number;
}
export function artworkPayload(values) {
  const payload = {...values};
  for (const axis of ['x','y']) if (axis in payload) payload[axis] = numericValue(payload[axis]);
  return payload;
}
export function posePayload(values, active) {
  const pose = Object.fromEntries(['x','y','theta'].map(key => [key, numericValue(values[key])]));
  const present = Object.values(pose).filter(value => value !== null).length;
  if (present !== 0 && present !== 3) throw new Error('위치는 x·y·theta를 모두 입력하거나 모두 비워 주세요.');
  return {active, frame_id: values.frame_id, pose: present ? pose : null};
}
export function robotRows(robots) {
  return robots.length ? robots.map(robot => `<tr><th scope="row">${escape(robot.robot_id)}</th><td>${robot.pose ? escape(robot.pose.x) : '수신 전'}</td><td>${robot.pose ? escape(robot.pose.y) : '수신 전'}</td><td>${robot.pose ? escape(robot.pose.theta) : '수신 전'}</td><td>${robot.active ? '활성' : '비활성'}</td><td>${escape(robot.frame_id)}</td><td>${escape(formatTime(robot.pose_updated_at))}</td></tr>`).join('') : '<tr><td colspan="7" class="empty">등록된 로봇이 없습니다.</td></tr>';
}
export function logRows(logs, openDetails = new Set()) {
  if (!logs.length) return '<tr><td colspan="4" class="empty">조건에 맞는 수신 로그가 없습니다.</td></tr>';
  return logs.map(entry => {
    const {sender, target, namespace, resource, type, payload, description} = entry.operator;
    const detailsId = `log-detail-${entry.log_id}`;
    const namespaceLabel = namespace ? ` · <code>${escape(namespace)}</code>` : '';
    return `<tr>
      <td>${escape(formatTime(entry.sent_at))}</td><td>${escape(sender)}</td><td>${escape(target)}</td>
      <td>
        <p class="log-description">${escape(description)}</p>
        <details class="log-entry-details" id="${escape(detailsId)}"${openDetails.has(detailsId) ? ' open' : ''}>
          <summary>기술 정보</summary>
          <dl>
            <dt>대상 / 네임스페이스</dt><dd>${escape(target)}${namespaceLabel}</dd>
            <dt>통신 경로 (리소스)</dt><dd><code>${escape(resource || '—')}</code></dd>
            <dt>통신 유형</dt><dd>${escape(type)}</dd>
            <dt>상세 내용 (Payload)</dt><dd><code class="log-payload">${escape(payload)}</code></dd>
            <dt>기록 ID / 로그 ID</dt><dd>${escape(entry.robot_id)} / ${escape(entry.log_id)}</dd>
          </dl>
        </details>
      </td>
    </tr>`;
  }).join('');
}
