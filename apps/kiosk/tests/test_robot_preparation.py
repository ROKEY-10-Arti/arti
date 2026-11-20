import sys
import tempfile
import threading
import time
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace as Msg
from unittest.mock import Mock, patch

from app import create_app
from robot_preparation import Preparation, PreparationError, PreparationManager, RosPreparationTransport
from ros_events import operator_log
from tests.auth_helpers import login_admin


def until(predicate, timeout=2):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError('state did not converge')
        time.sleep(.005)


class FakeTransport:
    def __init__(self):
        self.calls = []
        self.accept = True
        self.prepare_release = threading.Event()
        self.prepare_release.set()
        self.tour_release = threading.Event()
        self.tour_release.set()
        self.explanation_release = threading.Event()
        self.explanation_release.set()
        self.allow_stop = threading.Event()
        self.allow_stop.set()

    def start(self):
        pass

    def action(self, job, kind, feedback):
        self.calls.append(kind)
        with job.lock:
            job.motion_pending = True
        try:
            if not self.accept:
                raise PreparationError('요청 거절')
            if kind == 'prepare_tour':
                feedback(Msg(state='ARRIVED'))
                self.prepare_release.wait(2)
                feedback(Msg(state='REGISTERING'))
                feedback(Msg(state='REGISTERED'))
            else:
                feedback(Msg(state='MOVING', artwork_id=1))
                self.explanation_release.wait(2)
                feedback(Msg(state='EXPLAINING', artwork_id=1))
                self.tour_release.wait(2)
                feedback(Msg(state='ARTWORK_COMPLETED', artwork_id=1))
            self.allow_stop.wait(2)
            return Msg(success=True, message='완료')
        finally:
            with job.lock:
                job.motion_pending = False

    def stop(self):
        self.allow_stop.set()


class PreparationTest(unittest.TestCase):
    def setUp(self):
        self.transport = FakeTransport()
        self.manager = PreparationManager(self.transport, lambda _: None, lambda *_: None, lambda *_: None)
        self.addCleanup(self.manager.stop)

    def test_mode_assignment_and_same_robot_exclusion(self):
        self.transport.prepare_release.clear()
        free = self.manager.start(dict(id='free', mode='free'))
        course = self.manager.start(dict(id='course', mode='course'))
        self.assertEqual((free.robot_namespace, course.robot_namespace), ('/robot1', '/robot4'))
        with self.assertRaises(PreparationError):
            self.manager.start(dict(id='other', mode='free'))
        self.transport.prepare_release.set()
        until(lambda: free.phase == course.phase == 'ready')

    def test_cancel_requires_terminal_action_and_ignores_late_feedback(self):
        self.transport.allow_stop.clear()
        job = self.manager.start(dict(id='cancel', mode='free'))
        until(lambda: job.motion_pending)
        with self.assertRaises(PreparationError):
            self.manager.cancel_job(job, timeout=.02)
        self.assertEqual(job.phase, 'cancelling')
        self.transport.allow_stop.set()
        until(lambda: not job.motion_pending)
        self.manager.cancel_job(job)
        self.assertEqual(job.phase, 'cancelled')

    def test_ros_initialization_failure_is_logged_as_an_unsent_action(self):
        records = []
        transport = RosPreparationTransport(lambda _: None, lambda *event: records.append(event), discovery_timeout=0)
        job = Preparation(session={}, robot_id='robot1', robot_namespace='/robot1')
        with self.assertRaises(PreparationError):
            transport.action(job, 'prepare_tour', lambda _: None)
        self.assertEqual(records, [('robot1', '/robot1/arti/prepare_tour goal not sent: ros initialization failed')])
        self.assertFalse(job.motion_pending)


class ActionResponseTest(unittest.TestCase):
    def setUp(self):
        self.goal_status = Msg(STATUS_SUCCEEDED=4, STATUS_CANCELED=5, STATUS_ABORTED=6)
        self.enterContext(patch.dict(sys.modules, {'action_msgs.msg': Msg(GoalStatus=self.goal_status)}))
        self.result = Future()
        self.goal_response = Future()
        self.handle = Mock(accepted=True)
        self.handle.get_result_async.return_value = self.result
        self.client = Mock()
        self.client.wait_for_server.return_value = True
        self.client.server_is_ready.return_value = True
        self.client.send_goal_async.return_value = self.goal_response
        self.logs = []
        self.transport = RosPreparationTransport(lambda _: None, self.record, discovery_timeout=0.1,
                                                 server_loss_timeout=0.1)
        self.transport.types = {'start_tour': Msg(Goal=Msg)}
        self.transport.clients['/robot1/arti/start_tour'] = self.client
        self.transport.started.set()
        self.job = Preparation(dict(id='free', mode='free', artworkIds=['art-01']), phase='touring',
                               robot_id='robot1', robot_namespace='/robot1', tour_started=True)

    def record(self, robot_id, message):
        self.logs.append({'robot_id': robot_id, 'message': message})

    def test_lost_server_keeps_job_blocked_until_late_terminal_result(self):
        finish = Mock()
        manager = PreparationManager(self.transport, Mock(), finish, Mock())
        manager.jobs[self.job.session['id']] = self.job
        self.goal_response.set_result(self.handle)
        self.client.server_is_ready.return_value = False
        manager._tour(self.job)

        self.assertEqual(self.job.phase, 'error')
        self.assertTrue(self.job.motion_pending)
        self.handle.cancel_goal_async.assert_called_once()
        failure = next(entry for entry in self.logs if entry['message'].endswith('action server unavailable after send'))
        self.assertIn('서버 연결이 끊겼습니다', operator_log(failure)['description'])
        with self.assertRaises(PreparationError):
            manager.start(dict(id='next', mode='free'))
        with self.assertRaises(PreparationError):
            manager.cancel_job(self.job, timeout=0)
        finish.assert_not_called()

        self.result.set_result(Msg(status=self.goal_status.STATUS_CANCELED, result=Msg(success=False, message='canceled')))
        manager.cancel_job(self.job, timeout=0)
        self.assertFalse(self.job.motion_pending)
        self.assertEqual(self.job.phase, 'cancelled')
        finish.assert_called_once_with(self.job, 'cancelled')

    def test_missing_goal_response_times_out_and_late_acceptance_requests_cancel(self):
        with self.assertRaisesRegex(PreparationError, '요청을 받았는지 확인하지 못했어요'):
            self.transport.action(self.job, 'start_tour', Mock())
        self.assertEqual(self.logs[-1]['message'], '/robot1/arti/start_tour goal response timeout')
        self.assertTrue(self.job.motion_pending)
        self.goal_response.set_result(self.handle)
        self.handle.cancel_goal_async.assert_called_once()

    def test_unknown_result_status_keeps_motion_unconfirmed(self):
        self.goal_response.set_result(self.handle)
        self.result.set_result(Msg(status=0, result=Msg(success=False, message='')))
        with self.assertRaisesRegex(PreparationError, '액션 응답을 처리하지 못했어요'):
            self.transport.action(self.job, 'start_tour', Mock())
        self.assertTrue(self.job.motion_pending)
        self.assertTrue(any(entry['message'].endswith('action response failed') for entry in self.logs))


class PreparationApiTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.transport = FakeTransport()
        self.app = create_app(dict(TESTING=True, DATABASE=Path(self.temp.name)/'db.sqlite3', PREPARATION_TRANSPORT=self.transport))
        self.manager = self.app.extensions['robot_preparation']
        self.addCleanup(self.manager.stop)
        self.client = self.app.test_client()
        self.session = self.client.post('/api/sessions', json=dict(mode='course', courseId='A', artworkIds=['art-01','art-02','art-03'])).json
        self.url = '/api/sessions/' + self.session['id']

    def test_start_gate_persistence_stream_and_duplicate_requests(self):
        self.transport.prepare_release.clear()
        self.transport.tour_release.clear()
        self.assertEqual(self.client.patch(self.url, json=dict(status='active')).status_code, 409)
        self.assertEqual(self.client.post(self.url+'/preparation', json={}).status_code, 202)
        job = self.manager.get(self.session['id'])
        until(lambda: job.phase == 'arrived')
        self.client.post(self.url+'/preparation', json={})
        self.assertEqual(self.transport.calls, ['prepare_tour'])
        self.assertEqual(self.client.get(self.url).json['robotId'], 'robot4')
        self.assertEqual(self.client.post(self.url+'/preparation/register', json={}).status_code, 409)
        self.assertEqual(self.client.patch(self.url, json=dict(status='active')).status_code, 409)
        self.transport.prepare_release.set()
        until(lambda: job.phase == 'ready')
        stream = self.client.get(self.url+'/preparation/events', buffered=False)
        self.assertIn(b'"phase": "ready"', next(iter(stream.response)))
        stream.close()
        self.assertEqual(self.client.patch(self.url, json=dict(status='active')).status_code, 200)
        self.assertEqual(self.client.patch(self.url, json=dict(status='active')).status_code, 200)
        self.transport.tour_release.set()
        until(lambda: job.phase == 'completed')
        record = self.client.get(self.url).json
        self.assertEqual(record['status'], 'completed')
        self.assertEqual(record['visitedIds'], ['art-01'])
        self.assertEqual(self.transport.calls, ['prepare_tour', 'start_tour'])

    def test_fleet_activity_follows_action_feedback_and_clears_after_result(self):
        admin = login_admin(self.app.test_client())
        self.transport.explanation_release.clear()
        self.transport.tour_release.clear()
        self.addCleanup(self.transport.explanation_release.set)
        self.addCleanup(self.transport.tour_release.set)
        self.client.post(self.url + '/preparation', json={})
        job = self.manager.get(self.session['id'])
        until(lambda: job.phase == 'ready')
        self.assertEqual(admin.get('/api/fleet').json['tb4']['activity'], '관람 준비 완료 · 관람 시작 대기 중')
        self.client.patch(self.url, json={'status':'active'})
        until(lambda: job.tour_state == 'MOVING')
        self.assertEqual(admin.get('/api/fleet').json['tb4']['activity'], '1번 작품으로 이동 중')
        self.transport.explanation_release.set()
        until(lambda: job.tour_state == 'EXPLAINING')
        self.assertEqual(admin.get('/api/fleet').json['tb4']['activity'], '1번 작품 설명 중')
        self.transport.tour_release.set()
        until(lambda: job.phase == 'completed')
        self.assertEqual(admin.get('/api/fleet').json['tb4']['activity'], '활동 상태 미수신')

    def test_cancel_route_then_record_cancel(self):
        self.client.post(self.url+'/preparation', json={})
        job = self.manager.get(self.session['id'])
        until(lambda: job.phase == 'ready')
        self.assertEqual(self.client.post(self.url+'/preparation/cancel', json={}).status_code, 200)
        self.assertEqual(self.client.patch(self.url, json=dict(status='cancelled')).status_code, 200)
        self.assertEqual(self.client.post(self.url+'/preparation', json={}).status_code, 409)

    def test_dev2_error_to_ready_stream_and_normal_tour_without_ros_commands(self):
        self.transport.accept = False
        self.client.post(self.url+'/preparation', json={})
        old_job = self.manager.get(self.session['id'])
        until(lambda: old_job.phase == 'error')
        stream = self.client.get(self.url+'/preparation/events', buffered=False)
        self.addCleanup(stream.close)
        frames = iter(stream.response)
        self.assertIn(b'"phase": "error"', next(frames))
        login_admin(self.client)
        endpoint = '/api/dev/preparation/' + self.session['id'] + '/ready'
        calls = list(self.transport.calls)
        response = self.client.post(endpoint, json={})
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json['phase'], 'ready')
        self.assertTrue(response.json['dev_mode'])
        self.assertEqual(response.json['robot_id'], 'DEV-ARTI')
        self.assertEqual(self.client.post(endpoint, json={}).json, response.json)
        self.assertTrue(old_job.cancel.is_set())
        self.manager._error(old_job, PreparationError('late response'))
        self.assertIn(b'"phase": "ready"', next(frames))
        stream.close()
        row = self.client.get(self.url).json
        self.assertEqual(row['robotId'], 'DEV-ARTI')
        self.assertTrue(row['visitorId'].startswith('dev-visitor-'))
        self.assertEqual(self.client.patch(self.url, json={'status':'active'}).status_code, 200)
        completed = self.client.patch(self.url, json={'status':'completed', 'visitedIds':['art-01']})
        self.assertEqual(completed.status_code, 200)
        self.assertIsInstance(completed.json['endedAt'], int)
        self.assertEqual(self.transport.calls, calls)
        self.assertEqual(self.client.post(endpoint, json={}).status_code, 409)

    def test_dev2_requires_admin_csrf_and_local_access(self):
        endpoint = '/api/dev/preparation/' + self.session['id'] + '/ready'
        self.assertEqual(self.client.post(endpoint, json={}).status_code, 401)
        login_admin(self.client)
        token = self.client.environ_base.pop('HTTP_X_CSRF_TOKEN')
        self.assertEqual(self.client.post(endpoint, json={}).status_code, 403)
        self.client.environ_base['HTTP_X_CSRF_TOKEN'] = token
        self.assertEqual(self.client.post(endpoint, json={}, environ_overrides={'REMOTE_ADDR':'192.168.1.10'}).status_code, 403)
        self.assertIsNone(self.manager.get(self.session['id']))

    def test_dev2_recovers_preparation_lost_on_restart(self):
        login_admin(self.client)
        endpoint = '/api/dev/preparation/' + self.session['id'] + '/ready'
        self.assertIsNone(self.manager.get(self.session['id']))
        self.assertEqual(self.client.post(endpoint, json={}).status_code, 200)
        self.assertEqual(self.client.patch(self.url, json={'status':'active'}).status_code, 200)
        self.assertEqual(self.transport.calls, [])

    def test_dev2_cannot_override_pending_or_live_robot_work(self):
        login_admin(self.client)
        endpoint = '/api/dev/preparation/' + self.session['id'] + '/ready'
        job = Preparation(self.session, phase='error', motion_pending=True)
        self.manager.jobs[self.session['id']] = job
        self.assertEqual(self.client.post(endpoint, json={}).status_code, 409)
        self.assertIs(self.manager.get(self.session['id']), job)
        self.assertFalse(job.cancel.is_set())
        job.motion_pending = False
        for phase in ('calling','moving','registering','cancelling','cancelled'):
            job.phase = phase
            self.assertEqual(self.client.post(endpoint, json={}).status_code, 409)
        self.assertEqual(self.transport.calls, [])


if __name__ == '__main__':
    unittest.main()
