"""수집한 로그의 관리자 조회·검색."""
import re
from datetime import datetime, timedelta
from zoneinfo import ZoneInfo
from flask import abort

from ros_events import ACTION_NAMES, ROBOT_NAMES, operator_log


def log_page(db, args, operator_view=False):
    def integer(name, default, minimum, maximum):
        value = args.get(name, str(default))
        if not re.fullmatch(r'[0-9]{1,7}', value) or not minimum <= int(value) <= maximum:
            abort(400, description=f'{name}은 {minimum}~{maximum} 사이 정수여야 합니다.')
        return int(value)
    limit, offset = integer('limit', 100, 1, 500), integer('offset', 0, 0, 1000000)
    clauses, values = [], []
    filters = {name: args.get(name, '').strip() for name in ('q','robot_id','date','from','to','kind')}
    if len(filters['q']) > 200 or len(filters['robot_id']) > 64:
        abort(400, description='검색어는 200자, 로봇 ID는 64자 이내로 입력해 주세요.')
    if filters['robot_id']:
        clauses.append('robot_id=?'); values.append(filters['robot_id'])
    if filters['kind'] == 'action':
        robot_ids = ','.join('?' for _ in ROBOT_NAMES)
        action_paths = ' OR '.join("message GLOB '/' || robot_id || ?" for _ in ACTION_NAMES)
        clauses.append(f'robot_id IN ({robot_ids}) AND ({action_paths})')
        values.extend([*ROBOT_NAMES, *(f'/arti/{action} *' for action in ACTION_NAMES)])
    elif filters['kind'] == 'service':
        clauses.append("robot_id = 'kiosk' AND message GLOB '/arti/description *'")
    elif filters['kind']:
        abort(400, description='통신 유형은 액션 또는 서비스여야 합니다.')
    if filters["date"]:
        filters["from"] = filters["to"] = filters["date"]
    dates = {}
    for key, operator in (('from','>='),('to','<')):
        if not filters[key]: continue
        try:
            if not re.fullmatch(r'\d{4}-\d{2}-\d{2}', filters[key]): raise ValueError()
            date = datetime.strptime(filters[key], '%Y-%m-%d').replace(tzinfo=ZoneInfo('Asia/Seoul'))
            dates[key] = date
            boundary = date + timedelta(days=1) if key == 'to' else date
            value = int(boundary.timestamp() * 1000)
        except (ValueError, OverflowError, OSError):
            abort(400, description='검색 날짜는 YYYY-MM-DD 형식의 올바른 날짜여야 합니다.')
        clauses.append('sent_at'+operator+'?'); values.append(value)
    if dates.get('from') and dates.get('to') and dates['from'] > dates['to']:
        abort(400, description='시작 날짜는 종료 날짜보다 늦을 수 없습니다.')
    if filters['q']:
        escaped = filters['q'].replace('\\','\\\\').replace('%','\\%').replace('_','\\_')
        clauses.append("(CAST(log_id AS TEXT) LIKE ? ESCAPE '\\' OR robot_id LIKE ? ESCAPE '\\' OR message LIKE ? ESCAPE '\\')")
        values.extend(['%'+escaped+'%'] * 3)
    where = ' WHERE '+' AND '.join(clauses) if clauses else ''
    total = db.execute('SELECT COUNT(*) FROM Log'+where, values).fetchone()[0]
    rows = db.execute('SELECT log_id,robot_id,sent_at,message FROM Log'+where+' ORDER BY sent_at DESC,log_id DESC LIMIT ? OFFSET ?', [*values,limit,offset]).fetchall()
    entries = [dict(row) for row in rows]
    if operator_view:
        for entry in entries:
            entry['operator'] = operator_log(entry)
    return {'logs':entries, 'total':total, 'limit':limit, 'offset':offset,
            'has_more':offset+len(rows)<total, 'filters':filters}
