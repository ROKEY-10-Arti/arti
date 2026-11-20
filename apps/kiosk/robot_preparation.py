"""Bridge the ROS tour actions to kiosk preparation and session state."""
import atexit
import json
import threading
import time
from dataclasses import dataclass, field

from flask import Response, abort, jsonify, request


class PreparationError(RuntimeError):
    
    pass


@dataclass
class Preparation:
    session: dict
    phase: str = 'calling'
    failure_phase: str = ''
    message: str = '사용 가능한 아티를 배정하고 있어요.'
    robot_id: str = ''
    robot_namespace: str = ''
    visitor_id: str = ''
    progress: float = 0.0
    motion_pending: bool = False
    dev_mode: bool = False
    manual_registration: bool = False
    tour_started: bool = False
    tour_state: str = ""
    artwork_id: int = 0
    cancel: threading.Event = field(default_factory=threading.Event)
    tour_done: threading.Event = field(default_factory=threading.Event)
    lock: threading.RLock = field(default_factory=threading.RLock)

    def snapshot(self):
        with self.lock:
            return dict(session_id=self.session['id'], phase=self.phase, message=self.message,
                        robot_id=self.robot_id, visitor_id=self.visitor_id, progress=self.progress,
                        dev_mode=self.dev_mode, failure_phase=self.failure_phase,
                        manual_registration=self.manual_registration, tour_state=self.tour_state,
                        artwork_id=self.artwork_id)

    def update(self, phase, message, progress=0.0):
        with self.lock:
            if self.cancel.is_set():
                return False
            self.phase, self.message, self.progress = phase, message, progress
            return True


class RosPreparationTransport:
    """All ROS futures run on one executor. Workers only wait; they never spin its node."""
    def __init__(self, description, record_event, discovery_timeout=8, action_timeout=270):
        self.description = description
        self.record_event = record_event
        self.discovery_timeout = discovery_timeout
        self.action_timeout = action_timeout
        self.started = threading.Event()
        self.stopping = threading.Event()
        self.error = None
        self.clients = {}
        self.clients_lock = threading.Lock()
        self.thread = None

    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        context = node = executor = None
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from arti_interfaces.action import PrepareTour, StartTour
            from arti_interfaces.srv import Description
            from rclpy.action import ActionClient
            self.types = {'prepare_tour': PrepareTour, 'start_tour': StartTour}
            self.action_client_type = ActionClient
            context = Context()
            rclpy.init(context=context, args=['--ros-args', '--disable-external-lib-logs'])
            node = rclpy.create_node('arti_kiosk_preparation', namespace='/arti', context=context,
                                    enable_rosout=False, start_parameter_services=False)
            self.node = node
            def describe(request, response):
                description = self.description(request.artwork_id)
                response.success = description is not None
                response.description = description or ''
                self.record_event(
                    'kiosk', f'/arti/description request artwork_id={request.artwork_id}; '
                    f'response success={response.success} description_chars={len(response.description)}',
                )
                return response

            self.service = node.create_service(Description, '/arti/description', describe)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            self.started.set()
            while not self.stopping.is_set() and context.ok():
                executor.spin_once(timeout_sec=0.1)
        except Exception as error:
            self.error = error
            self.started.set()
        finally:
            if executor:
                executor.shutdown(timeout_sec=1)
            for client in self.clients.values():
                client.destroy()
            if node:
                node.destroy_node()
            if context and context.ok():
                context.shutdown()

    def _ensure(self):
        if not self.started.wait(self.discovery_timeout) or self.error or self.stopping.is_set():
            raise PreparationError('ROS 연결을 준비하지 못했어요. 인터페이스 빌드와 서버 실행 상태를 확인해 주세요.')

    def action(self, job, kind, feedback):
        from action_msgs.msg import GoalStatus

        self._ensure()
        action_type = self.types[kind]
        name = job.robot_namespace + '/arti/' + kind
        def log(message):
            self.record_event(job.robot_id, f'{name} {message}')

        with self.clients_lock:
            client = self.clients.get(name)
            if client is None:
                client = self.action_client_type(self.node, action_type, name)
                self.clients[name] = client
        deadline = time.monotonic() + self.discovery_timeout
        while not client.wait_for_server(timeout_sec=0.2):
            if job.cancel.is_set() or time.monotonic() >= deadline:
                log('goal not sent: cancelled or server unavailable')
                raise PreparationError('로봇 액션 서버에 연결하지 못했어요.')
        goal = action_type.Goal()
        if kind == 'prepare_tour':
            goal.start = True
        else:
            goal.artwork_ids = (
                [int(value.removeprefix('art-')) for value in job.session['artworkIds']]
                if job.session['mode'] == 'course' else []
            )
        with job.lock:
            if job.cancel.is_set():
                raise PreparationError('호출을 취소했어요.')
            job.motion_pending = True
        holder = {}
        terminal = threading.Event()
        cancel_lock = threading.Lock()
        last_feedback = None

        def on_feedback(frame):
            nonlocal last_feedback
            message = frame.feedback
            current = (message.state, getattr(message, 'artwork_id', None))
            if current != last_feedback and not job.cancel.is_set():
                last_feedback = current
                log(f'feedback state={current[0]} artwork_id={current[1]}')
            feedback(message)

        def request_cancel(handle):
            with cancel_lock:
                if not handle or not handle.accepted or holder.get('cancel_sent'):
                    return
                handle.cancel_goal_async()
                holder['cancel_sent'] = True
                log('cancel requested')

        def result_done(future):
            try:
                result = holder['result'] = future.result()
                log(f'result status={result.status} success={result.result.success} message={result.result.message}')
            except Exception as error:
                holder['error'] = error
            else:
                with job.lock:
                    job.motion_pending = False
            finally:
                terminal.set()

        def goal_done(future):
            try:
                handle = future.result()
                holder['handle'] = handle
                log(f'goal accepted={handle.accepted}')
                if not handle.accepted:
                    with job.lock:
                        job.motion_pending = False
                    terminal.set()
                else:
                    handle.get_result_async().add_done_callback(result_done)
                    if job.cancel.is_set():
                        request_cancel(handle)
            except Exception as error:
                # A transport error cannot prove the robot stopped. Keep cancellation unconfirmed.
                holder['error'] = error
                terminal.set()

        try:
            payload = 'start=true' if kind == 'prepare_tour' else f'artwork_ids={list(goal.artwork_ids)}'
            log(f'sending goal {payload}')
            future = client.send_goal_async(goal, feedback_callback=on_feedback)
        except Exception:
            with job.lock:
                job.motion_pending = False  # No goal was submitted.
            raise
        future.add_done_callback(goal_done)
        deadline = time.monotonic() + self.action_timeout if kind == 'prepare_tour' else None
        while not terminal.wait(0.05):
            handle = holder.get('handle')
            if job.cancel.is_set():
                request_cancel(handle)
            if deadline is not None and time.monotonic() >= deadline:
                job.cancel.set()  # A late goal response also receives cancellation.
                request_cancel(handle)
                log('result timeout')
                raise PreparationError('로봇 작업 시간이 초과됐어요. 취소 후 정지 확인이 필요해요.')
        if 'error' in holder:
            raise PreparationError('로봇 액션 연결이 끊겼어요. 정지 상태를 확인해 주세요.')
        handle = holder.get('handle')
        if not handle or not handle.accepted:
            raise PreparationError('로봇이 작업 요청을 거절했어요.')
        result = holder['result']
        if result.status == GoalStatus.STATUS_CANCELED and job.cancel.is_set():
            raise PreparationError('로봇 작업을 취소했어요.')
        if result.status != GoalStatus.STATUS_SUCCEEDED or not result.result.success:
            raise PreparationError(result.result.message or '로봇 작업이 완료되지 않았어요.')
        return result.result

    def stop(self):
        self.stopping.set()
        if self.thread:
            self.thread.join(timeout=2)


class PreparationManager:
    def __init__(self, transport, persist, finish, record_artwork):
        self.transport, self.persist = transport, persist
        self.finish, self.record_artwork = finish, record_artwork
        self.jobs = {}
        self.lock = threading.Lock()

    def start(self, session):
        with self.lock:
            if session['id'] in self.jobs:
                return self.jobs[session['id']]
            manual = getattr(self.transport, 'manual', False)
            job = Preparation(session, manual_registration=manual)
            if not manual:
                job.robot_id = {'free': 'robot1', 'course': 'robot4'}[session['mode']]
                job.robot_namespace = '/' + job.robot_id
                for previous in self.jobs.values():
                    with previous.lock:
                        if previous.robot_id == job.robot_id and (
                            previous.motion_pending or previous.phase in {
                                'calling', 'moving', 'arrived', 'registering', 'ready', 'touring', 'cancelling'
                            }
                        ):
                            raise PreparationError('이 관람을 담당하는 아티가 사용 중이에요.')
                self.persist(job)
            self.jobs[session['id']] = job
        if not manual:
            threading.Thread(target=self._prepare, args=(job,), daemon=True).start()
        return job

    def get(self, session_id):
        with self.lock:
            return self.jobs.get(session_id)

    def dev_ready(self, session):
        """완료된 오류 작업을 로컬 화면 테스트용 준비 완료 작업으로 교체합니다."""
        with self.lock:
            previous = self.jobs.get(session['id'])
            if previous:
                with previous.lock:
                    if previous.dev_mode and previous.phase == 'ready':
                        return previous
                    if previous.motion_pending:
                        raise PreparationError('로봇 작업의 종료를 확인한 뒤 dev2를 눌러 주세요.')
                    if previous.phase != 'error':
                        raise PreparationError('오류 화면에서 dev2 준비 완료 신호를 사용할 수 있어요.')
                    previous.cancel.set()
            job = Preparation(session, phase='ready', message='dev2 테스트 신호로 관람 준비를 완료했어요. 관람 시작하기를 눌러 주세요.',
                              robot_id='DEV-ARTI', robot_namespace='/dev_arti',
                              visitor_id='dev-visitor-' + session['id'], progress=1.0,
                              dev_mode=True)
            self.persist(job)
            self.jobs[session['id']] = job
            return job

    def _error(self, job, error):
        with job.lock:
            if job.phase != 'cancelled':
                if job.phase != 'error':
                    job.failure_phase = job.phase
                job.phase, job.message = 'error', str(error)

    def _prepare(self, job):
        phases = {
            'UNDOCKING': ('moving', '아티가 도킹을 해제하고 있어요.', 0.1),
            'MOVING': ('moving', '아티가 만남 장소로 이동하고 있어요.', 0.3),
            'ARRIVED': ('arrived', '아티가 도착했어요. 정면에 한 분만 서 주세요.', 0.6),
            'REGISTERING': ('registering', '정면을 바라보고 잠시 기다려 주세요.', 0.8),
            'REGISTERED': ('registering', '관람객 등록을 확인하고 있어요.', 1.0),
        }

        def feedback(message):
            if message.state in phases:
                job.update(*phases[message.state])

        try:
            result = self.transport.action(job, 'prepare_tour', feedback)
            job.update('ready', result.message or '관람을 시작할 준비가 됐어요.', 1.0)
        except Exception as error:
            if not job.cancel.is_set() or job.motion_pending:
                self._error(job, error)

    def register(self, job):
        with job.lock:
            if not job.manual_registration or job.phase != 'arrived' or job.cancel.is_set():
                raise PreparationError('현재 등록을 수동으로 시작할 수 없어요.')
            job.update('registering', '정면을 바라보고 잠시 기다려 주세요.')

    def start_tour(self, job):
        with job.lock:
            if job.dev_mode or job.manual_registration or job.tour_started:
                return
            job.tour_started = True
            launch = job.update('touring', '아티와 함께 관람 중이에요.')
        if launch:
            threading.Thread(target=self._tour, args=(job,), daemon=True).start()
        else:
            self.finish(job, 'cancelled')
            job.tour_done.set()

    def _tour(self, job):
        def feedback(message):
            with job.lock:
                if job.cancel.is_set():
                    return
                job.tour_state, job.artwork_id = message.state, message.artwork_id
            if message.state == 'ARTWORK_COMPLETED' and message.artwork_id > 0:
                self.record_artwork(job, message.artwork_id)

        try:
            result = self.transport.action(job, 'start_tour', feedback)
        except Exception as error:
            if job.motion_pending:
                self._error(job, error)
                return
            self.finish(job, 'cancelled')
            with job.lock:
                if job.cancel.is_set():
                    job.phase, job.message = 'cancelled', '로봇 작업을 취소했어요.'
                else:
                    self._error(job, error)
        else:
            self.finish(job, 'completed')
            with job.lock:
                job.phase, job.message, job.progress = 'completed', result.message, 1.0
        job.tour_done.set()

    def cancel_job(self, job, timeout=6):
        with job.lock:
            if job.phase == 'completed':
                raise PreparationError('이미 종료된 관람이에요.')
            job.cancel.set()
            job.phase, job.message = 'cancelling', '로봇 작업 취소와 정지 상태를 확인하고 있어요.'
        deadline = time.monotonic() + timeout
        while True:
            with job.lock:
                if job.tour_started and job.tour_done.is_set():
                    if job.phase == 'completed':
                        raise PreparationError('이미 종료된 관람이에요.')
                    job.phase, job.message = 'cancelled', '로봇 작업을 취소했어요.'
                    return
                if not job.tour_started and not job.motion_pending:
                    job.phase, job.message = 'cancelled', '로봇 작업을 취소했어요.'
                    return
            if time.monotonic() >= deadline:
                raise PreparationError('아직 로봇 작업의 종료를 확인하지 못했어요. 잠시 후 다시 취소해 주세요.')
            time.sleep(0.05)

    def stop(self):
        with self.lock:
            jobs = list(self.jobs.values())
        for job in jobs:
            job.cancel.set()
        deadline = time.monotonic() + 2
        while any(job.motion_pending for job in jobs) and time.monotonic() < deadline:
            time.sleep(0.05)
        self.transport.stop()


def register_robot_preparation(app, get_session, serialize_session, persist, finish, record_artwork, description, record_event):
    transport = app.config.get('PREPARATION_TRANSPORT') or RosPreparationTransport(description, record_event)
    manager = PreparationManager(transport, persist, finish, record_artwork)
    app.extensions['robot_preparation'] = manager
    if not app.testing or app.config.get('PREPARATION_TRANSPORT'):
        transport.start()
        atexit.register(manager.stop)

    def job_for(session_id):
        get_session(session_id)
        job = manager.get(session_id)
        if not job:
            abort(409, description='진행 중인 로봇 준비 작업이 없어요. 다시 호출해 주세요.')
        return job

    @app.post('/api/sessions/<session_id>/preparation')
    def begin(session_id):
        session = serialize_session(get_session(session_id))
        if session['status'] != 'calling':
            abort(409, description='현재 관람 상태에서는 호출할 수 없어요.')
        try:
            return jsonify(manager.start(session).snapshot()), 202
        except PreparationError as error:
            abort(409, description=str(error))

    @app.post('/api/sessions/<session_id>/preparation/register')
    def register(session_id):
        job = job_for(session_id)
        try:
            manager.register(job)
        except PreparationError as error:
            abort(409, description=str(error))
        return jsonify(job.snapshot()), 202

    @app.post('/api/sessions/<session_id>/preparation/cancel')
    def cancel(session_id):
        get_session(session_id)
        job = manager.get(session_id)
        if job:
            try:
                manager.cancel_job(job)
            except PreparationError as error:
                abort(409, description=str(error))
        return jsonify(cancelled=True)

    @app.post('/api/dev/preparation/<session_id>/ready')
    def dev_ready(session_id):
        if request.remote_addr not in {'127.0.0.1', '::1'}:
            abort(403, description='dev2는 이 컴퓨터에서만 사용할 수 있어요.')
        session = serialize_session(get_session(session_id))
        if session['status'] != 'calling':
            abort(409, description='호출 준비 중인 관람에서만 dev2를 사용할 수 있어요.')
        try:
            job = manager.dev_ready(session)
        except PreparationError as error:
            abort(409, description=str(error))
        return jsonify(job.snapshot())

    @app.get('/api/sessions/<session_id>/preparation/events')
    def events(session_id):
        job = job_for(session_id)
        def stream():
            while True:
                # dev2로 작업을 교체해도 이미 열린 스트림에서 최신 준비 상태를 전달합니다.
                current = manager.get(session_id) or job
                yield 'event: preparation\ndata: ' + json.dumps(current.snapshot()) + '\n\n'
                time.sleep(0.5)
        return Response(stream(), mimetype='text/event-stream', headers={'X-Accel-Buffering': 'no', 'Cache-Control': 'no-store'})
