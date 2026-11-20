"""Operator labels for the kiosk's ROS 2 tour and description interfaces."""

import re


FEEDBACK_LABELS = {
    'prepare_tour': {
        'UNDOCKING': '도킹 해제 중',
        'MOVING': '만남 장소로 이동 중',
        'ARRIVED': '만남 장소 도착',
        'REGISTERING': '관람객 등록 중',
        'REGISTERED': '관람객 등록 완료',
    },
    'start_tour': {
        'MOVING': '이동 중',
        'FOLLOW': '자유 관람 중',
        'EXPLAINING': '작품 설명 중',
        'ARTWORK_COMPLETED': '작품 설명 완료',
        'RETURNING': '충전 스테이션으로 복귀 중',
    },
}

ACTION_NAMES = {'prepare_tour': '관람 준비', 'start_tour': '관람'}
ROBOT_NAMES = {'robot1': '로봇 1', 'robot4': '로봇 4'}
ACTION_LINE = re.compile(r'/(?P<robot>robot1|robot4)/arti/(?P<action>prepare_tour|start_tour) (?P<event>.+)', re.DOTALL)
DESCRIPTION_LINE = re.compile(r'/arti/description request artwork_id=(-?\d{1,10}); response success=(True|False) description_chars=\d+')
RESULT_REASONS = {
    'not_requested': '준비를 시작하라는 지시가 없었습니다.',
    'not_docked': '로봇이 도킹 중인지 확인할 수 없었습니다.',
    'no_undock_server': '도킹 해제 기능에 연결하지 못했습니다.',
    'undock_failed': '로봇이 도킹을 해제하지 못했습니다.',
    'no_localization': '로봇이 지도에서 현재 위치를 확인하지 못했습니다.',
    'not_started': '제한 시간 안에 관람이 시작되지 않았습니다.',
    'not_prepared': '관람 준비가 완료되지 않았습니다.',
    'register_failed': '제한 시간 안에 관람객을 등록하지 못했습니다.',
    'lost': '관람객의 위치를 더 이상 확인할 수 없었습니다.',
    'visitor_lost': '관람객의 위치를 더 이상 확인할 수 없었습니다.',
    'shutdown': '로봇 제어 프로그램이 종료됐습니다.',
    'AMCL/map_server/Nav2 lifecycle not active': '로봇의 주행 시스템이 활성화되지 않았습니다.',
    'AMCL map 위치 정보가 신선하지 않음': '로봇의 지도상 위치가 최근에 갱신되지 않았습니다.',
    'LiDAR scan 신호가 없음': '로봇의 거리 센서 신호가 수신되지 않았습니다.',
    'map -> base_link TF 변환 미수신 또는 만료': '지도와 로봇 위치를 연결하는 정보가 수신되지 않았거나 만료됐습니다.',
    'Nav2 navigate_to_pose 액션 미준비': '로봇의 주행 기능을 사용할 준비가 되지 않았습니다.',
    '코스주행 중복 요청': '로봇이 이미 코스 관람을 처리하고 있습니다.',
    '최신 dock_status 없음': '로봇의 최신 도킹 상태를 수신하지 못했습니다.',
    '이미 코스주행 중입니다': '로봇이 이미 코스 관람을 처리하고 있습니다.',
    '언도킹 출발 위치(map 좌표)가 필요합니다': '로봇의 도킹 해제 출발 위치를 확인하지 못했습니다.',
    '목표 지역 내 정지 확인 시간 초과': '제한 시간 안에 목적지에서 정차했는지 확인하지 못했습니다.',
    '해설 중 현재 작품 반원 이탈 또는 위치 정보 소실': '작품 설명 중에 로봇이 지정 구역을 벗어났거나 위치를 확인할 수 없었습니다.',
    '해설 완료 응답 시간 초과': '제한 시간 안에 작품 설명 완료 신호를 받지 못했습니다.',
    '해설 완료 시점에 현재 작품 반원 밖 또는 위치 정보 만료': '작품 설명을 마쳤을 때 로봇이 지정 구역 안에 있는지 확인하지 못했습니다.',
    'Dock 액션 시간 초과': '제한 시간 안에 도킹이 완료되지 않았습니다.',
    'Dock 액션 성공 이후 dock_status.is_docked 확인 실패': '도킹 동작이 끝났지만 도크에 연결됐는지 확인하지 못했습니다.',
    'Dock 액션 실패/거절/취소': '도킹 작업을 완료하지 못했습니다.',
    '언도킹 후 LiDAR/AMCL/TF 미수신 타임아웃': '도킹 해제 후 주행에 필요한 센서나 위치 정보를 받지 못했습니다.',
}
RESULT_PREFIXES = {
    'explain_artwork 액션 서버 미준비': '작품 설명 재생 기능이 준비되지 않았습니다.',
    'Dock 액션 서버 미준비': '도킹 기능이 준비되지 않았습니다.',
    'Dock 액션 서버 없음': '도킹 기능에 연결하지 못했습니다.',
    'Dock 목표 전송 실패': '도킹 요청을 보내지 못했습니다.',
    'Dock 요청 거절/오류': '도킹 요청이 거절됐거나 처리 중 오류가 발생했습니다.',
    'Dock 결과 조회 실패': '도킹 작업의 결과를 받지 못했습니다.',
    'Undock 액션 서버 미준비': '도킹 해제 기능이 준비되지 않았습니다.',
    'Undock 실패': '로봇이 도킹을 해제하지 못했습니다.',
    'Undock 타임아웃': '제한 시간 안에 도킹 해제가 확인되지 않았습니다.',
    'Undock 후 처리 실패': '도킹을 해제한 뒤 주행 준비를 마치지 못했습니다.',
    '배터리 정보 없음': '로봇의 배터리 정보를 확인하지 못했습니다.',
    '배터리 부족:': '로봇 배터리가 부족합니다.',
    'Nav2 전송 실패': '로봇의 이동 요청을 보내지 못했습니다.',
    'Nav2 결과 확인 실패': '로봇의 이동 결과를 확인하지 못했습니다.',
    'Nav2 목표 취소': '로봇의 이동 취소 결과를 확인하지 못했습니다.',
    'Nav2 ': '로봇이 이동 작업을 완료하지 못했습니다.',
    '해설 요청 실패': '작품 설명 요청을 보내지 못했습니다.',
    '해설 결과 실패': '작품 설명 결과를 받지 못했습니다.',
    '잘못된 작품 해설 응답': '요청한 작품과 다른 작품의 설명 응답이 왔습니다.',
    '복귀 실패:': '로봇이 충전 스테이션으로 복귀하지 못했습니다.',
}


def feedback_activity(action, state, artwork_id=0):
    label = FEEDBACK_LABELS[action].get(state)
    if action == 'start_tour' and artwork_id > 0:
        return {
            'MOVING': f'{artwork_id}번 작품으로 이동 중',
            'EXPLAINING': f'{artwork_id}번 작품 설명 중',
            'ARTWORK_COMPLETED': f'{artwork_id}번 작품 설명 완료',
        }.get(state, label)
    return label


def result_description(action, status, success, reason):
    task = ACTION_NAMES[action]
    if status == '5' or reason in {'canceled', 'cancelled'}:
        return f'로봇이 {task} 취소를 확인했습니다.'
    if status == '4' and success == 'True':
        return ('관람객 등록을 마치고 관람을 시작할 준비가 됐습니다.' if action == 'prepare_tour'
                else '로봇이 관람을 정상적으로 마쳤습니다.')
    cause = RESULT_REASONS.get(reason)
    artwork_failure = re.fullmatch(r'작품 (\d{1,10}) 해설 실패', reason)
    if artwork_failure:
        cause = f'{artwork_failure[1]}번 작품의 설명 재생을 완료하지 못했습니다.'
    if not cause:
        cause = next((label for prefix, label in RESULT_PREFIXES.items() if reason.startswith(prefix)), None)
    return f'{task} 작업이 완료되지 않았습니다. {cause}' if cause else f'{task} 작업이 완료되지 않았습니다.'


def describe_action(robot, action, event):
    """Return the direction, subtype and explanation for a recognized log line."""
    task = ACTION_NAMES[action]
    request_name = '관람 시작' if action == 'start_tour' else task
    if event == 'sending goal start=true' and action == 'prepare_tour':
        return True, '요청', '키오스크가 로봇에 관람 준비 요청을 보내려고 했습니다.'
    goal = re.fullmatch(r'sending goal artwork_ids=\[([0-9, ]*)\]', event)
    if goal and action == 'start_tour':
        ids = goal[1]
        if ids:
            description = f'키오스크가 {ids}번 작품의 코스 관람 시작을 요청하려고 했습니다.'
        elif robot == 'robot1':
            description = '키오스크가 자유 관람 시작을 요청하려고 했습니다.'
        else:
            description = '키오스크가 작품 목록 없이 코스 관람 시작을 요청하려고 했습니다.'
        return True, '요청', description
    communication_issues = {
        'goal not sent: cancelled': '요청이 취소돼 로봇에 보내지 않았습니다.',
        'goal not sent: ros initialization failed': '키오스크가 ROS 연결을 준비하지 못해 요청을 보내지 못했습니다.',
        'goal not sent: action client unavailable': '키오스크가 관람 요청을 보낼 통신 채널을 열지 못했습니다.',
        'goal not sent: transmission failed': '키오스크에서 로봇에 요청을 보내는 중 오류가 발생했습니다.',
        'goal not sent: server unavailable': '로봇의 액션 서버에 연결하지 못해 요청을 보내지 못했습니다.',
        'goal not sent: cancelled or server unavailable': '요청이 취소됐거나 로봇의 액션 서버에 연결하지 못해 요청을 보내지 못했습니다.',
        'goal response timeout': '로봇으로부터 요청 수락 여부를 받지 못했습니다. 작업 종료를 확인해야 합니다.',
        'action server unavailable after send': '요청을 보낸 뒤 로봇 액션 서버 연결이 끊겼습니다. 작업 종료를 확인해야 합니다.',
        'action transport stopped': '요청을 보낸 뒤 키오스크의 ROS 연결이 중단됐습니다. 작업 종료를 확인해야 합니다.',
        'action connection check failed': '요청을 보낸 뒤 로봇과의 연결 상태를 확인하지 못했습니다.',
        'action response failed': '로봇 응답에서 작업 종료를 확인하지 못했습니다.',
        'cancel request failed': '로봇에 취소 요청을 보내지 못했습니다. 작업 종료를 확인해야 합니다.',
        'result timeout': '제한 시간 안에 로봇의 작업 결과를 받지 못했습니다.',
    }
    if event in communication_issues:
        return True, '통신 기록', f'{task} 중 {communication_issues[event]}'
    if event in {'goal accepted=True', 'goal accepted=False'}:
        verb = '수락했습니다' if event.endswith('True') else '거절했습니다'
        return False, '요청 응답', f'로봇이 {request_name} 요청을 {verb}.'
    if event == 'cancel requested':
        return True, '취소 요청', f'키오스크가 로봇에 {task} 취소를 요청했습니다.'
    feedback = re.fullmatch(r'feedback state=(\S+)(?: artwork_id=(None|-?\d{1,10}))?', event)
    if feedback:
        state, artwork_id = feedback.groups()
        activity = feedback_activity(action, state, 0 if artwork_id in {None, 'None'} else int(artwork_id))
        description = f'로봇이 보고한 상태 · {activity}' if activity else f'로봇이 {task} 작업 상태를 보고했습니다.'
        return False, '진행 상태', description
    result = re.fullmatch(r'result status=([0-6]) success=(True|False) message=(.*)', event, re.DOTALL)
    if result:
        return False, '결과', result_description(action, *result.groups())
    return None


def operator_log(entry):
    message = entry['message']
    action_line = ACTION_LINE.fullmatch(message)
    if action_line and entry['robot_id'] == action_line['robot']:
        robot, action, event = action_line.group('robot', 'action', 'event')
        details = describe_action(robot, action, event)
        namespace = f'/{robot}'
        resource = f'{namespace}/arti/{action}'
        if details:
            outgoing, kind, description = details
            sender, target = ('키오스크', ROBOT_NAMES[robot]) if outgoing else (ROBOT_NAMES[robot], '키오스크')
            return dict(sender=sender, target=target, namespace=namespace, resource=resource,
                        type=f'ROS 액션 · {kind}', payload=event, description=description)
        return dict(sender='미확인', target='미확인', namespace=namespace, resource=resource,
                    type='ROS 액션', payload=event, description=f'{ACTION_NAMES[action]} 통신 메시지가 기록됐습니다.')

    service = DESCRIPTION_LINE.fullmatch(message)
    if service and entry['robot_id'] == 'kiosk':
        artwork_id, success = service.groups()
        description = (f'키오스크가 요청한 로봇에 {artwork_id}번 작품의 설명을 반환했습니다.' if success == 'True'
                       else f'키오스크가 {artwork_id}번 작품의 설명을 제공하지 못했습니다.')
        return dict(sender='키오스크', target='요청 로봇(미확인)', namespace='/arti', resource='/arti/description',
                    type='ROS 서비스 · 요청 및 응답', payload=message.removeprefix('/arti/description '),
                    description=description)

    return dict(sender='미확인', target='미확인', namespace=None, resource=None, type='미확인',
                payload=message, description='메시지의 통신 유형을 확인할 수 없습니다.')
