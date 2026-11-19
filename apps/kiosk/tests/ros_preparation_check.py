"""Test kiosk ROS contracts on an isolated local ROS domain."""
import os
import sqlite3
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


def wait_for(predicate, timeout=10):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise AssertionError('ROS fixture did not reach the expected state')
        time.sleep(0.02)


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '181' or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise SystemExit('Run with ROS_DOMAIN_ID=181 and ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST.')
    if os.environ.get('ROS_DISCOVERY_SERVER'):
        raise SystemExit('Unset ROS_DISCOVERY_SERVER for the local fixture.')

    import rclpy
    from arti_interfaces.action import PrepareTour, StartTour
    from arti_interfaces.srv import Description
    from rclpy.action import ActionServer, CancelResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import MultiThreadedExecutor
    from app import create_app
    from robot_preparation import RosPreparationTransport

    rclpy.init()
    node = rclpy.create_node('arti_test_robot')
    group = ReentrantCallbackGroup()
    goals = {}

    def prepare(handle):
        assert handle.request.start is True
        for state in ('UNDOCKING', 'MOVING', 'ARRIVED', 'REGISTERING', 'REGISTERED'):
            handle.publish_feedback(PrepareTour.Feedback(state=state))
            time.sleep(0.04)
        handle.succeed()
        return PrepareTour.Result(success=True, message='ready')

    def tour(handle, robot):
        goals[robot] = list(handle.request.artwork_ids)
        if robot == 'robot1':
            while not handle.is_cancel_requested:
                handle.publish_feedback(StartTour.Feedback(state='FOLLOW', artwork_id=0))
                time.sleep(0.05)
            handle.canceled()
            return StartTour.Result(success=False, message='canceled')
        for artwork_id in handle.request.artwork_ids:
            handle.publish_feedback(StartTour.Feedback(state='ARTWORK_COMPLETED', artwork_id=artwork_id))
            time.sleep(0.05)
        handle.succeed()
        return StartTour.Result(success=True, message='completed')

    servers = []
    for robot in ('robot1', 'robot4'):
        options = {'callback_group': group, 'cancel_callback': lambda _: CancelResponse.ACCEPT}
        servers.append(ActionServer(node, PrepareTour, f'/{robot}/arti/prepare_tour', prepare, **options))
        servers.append(ActionServer(node, StartTour, f'/{robot}/arti/start_tour',
                                    lambda handle, robot=robot: tour(handle, robot), **options))
    description = node.create_client(Description, '/arti/description', callback_group=group)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    manager = None
    try:
        with tempfile.TemporaryDirectory(prefix='arti-ros-ui-') as directory:
            # TESTING disables production discovery settings, while the explicit transport uses DDS.
            app = create_app({'TESTING': True, 'DATABASE': Path(directory) / 'test.sqlite3',
                              'PREPARATION_TRANSPORT': None})
            manager = app.extensions['robot_preparation']
            assert isinstance(manager.transport, RosPreparationTransport)
            manager.transport.start()
            client = app.test_client()

            def start(mode, artwork_ids):
                session = client.post('/api/sessions', json={'mode': mode, 'artworkIds': artwork_ids}).json
                url = '/api/sessions/' + session['id']
                assert client.post(url + '/preparation', json={}).status_code == 202
                job = manager.get(session['id'])
                wait_for(lambda: job.phase == 'ready')
                assert client.patch(url, json={'status': 'active'}).status_code == 200
                return url, job

            assert description.wait_for_service(timeout_sec=10)
            assert manager.transport.node.get_namespace() == '/arti'
            for artwork_id, exists in ((1, True), (999, False)):
                request = Description.Request(artwork_id=artwork_id)
                for attempt in range(3):
                    future = description.call_async(request)
                    try:
                        wait_for(future.done, timeout=2)
                        break
                    except AssertionError:
                        future.cancel()
                        if attempt == 2:
                            raise
                assert future.result().success is exists
                assert bool(future.result().description) is exists
                if exists:
                    with sqlite3.connect(app.config['DATABASE']) as db:
                        expected, = db.execute(
                            'SELECT description FROM Art WHERE artwork_id=?', (f"{artwork_id:03d}",)
                        ).fetchone()
                    assert future.result().description == expected

            url, job = start('course', ['art-01', 'art-03', 'art-04'])
            wait_for(lambda: job.phase == 'completed')
            record = client.get(url).json
            assert record['status'] == 'completed'
            assert record['visitedIds'] == ['art-01', 'art-03', 'art-04']
            url, job = start('free', ['art-01'])
            wait_for(lambda: goals.get('robot1') == [])
            assert client.post(url + '/preparation/cancel', json={}).status_code == 200
            assert client.get(url).json['status'] == 'cancelled'
            assert goals == {'robot1': [], 'robot4': [1, 3, 4]}
            with sqlite3.connect(app.config['DATABASE']) as db:
                messages = [message for message, in db.execute('SELECT message FROM Log')]
            for fragment in (
                'request artwork_id=1; response success=True',
                'request artwork_id=999; response success=False',
                'prepare_tour sending goal start=true',
                'start_tour feedback state=ARTWORK_COMPLETED artwork_id=3',
                'start_tour result status=4 success=True',
                'start_tour cancel requested',
            ):
                assert any(fragment in message for message in messages), fragment
            print('ROS integration passed: both robots, description lookup, feedback, completion, cancellation.')
    finally:
        if manager:
            manager.stop()
        executor.shutdown(timeout_sec=2)
        thread.join(timeout=2)
        for server in servers:
            server.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
