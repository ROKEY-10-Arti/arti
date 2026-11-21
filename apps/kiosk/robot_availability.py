"""로봇 배터리·도킹 상태 구독과 브라우저용 1Hz 상태 스트림."""

import atexit
import json
import threading
import time

from flask import Response, abort, jsonify

from fleet import Fleet


class RobotAvailability:
    def __init__(self, fleet=None):
        self.fleet = fleet if fleet is not None else Fleet()
        self._lock = threading.Lock()
        self._failed = False
        self._stop = threading.Event()
        self._thread = None

    def snapshot(self):
        with self._lock:
            rows = self.fleet.snapshot().values()
            readings = [row[key] for row in rows for key in ("dock", "battery") if key in row]
            available = None
            if self._failed:
                status = "disconnected"
            elif not readings:
                status = "waiting"
            elif all(value is None for value in readings):
                status = "stale"
            else:
                available = any(row["available"] for row in rows)
                status = "available" if available else "unavailable"
            return {"status": status, "available": available}

    def start(self, logger):
        if self._thread and self._thread.is_alive():
            return
        with self._lock:
            self._failed = False
        self._stop.clear()
        self._thread = threading.Thread(target=self._receive, args=(logger,), daemon=True)
        self._thread.start()

    def _receive(self, logger):
        context = node = executor = None
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data

            context = Context()
            rclpy.init(context=context, args=["--ros-args", "--disable-external-lib-logs"])
            node = rclpy.create_node(
                "arti_kiosk_availability", namespace="/arti", context=context,
                enable_rosout=False, start_parameter_services=False,
            )
            self.fleet.subscribe(node, qos_profile_sensor_data)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            logger.info("로봇 상태 구독 시작: battery_state, dock_status, odom")
            while not self._stop.is_set() and context.ok():
                executor.spin_once(timeout_sec=0.2)
        except Exception:
            with self._lock:
                self._failed = True
            logger.exception("로봇 가용 상태 토픽을 구독하지 못했습니다.")
        finally:
            if executor:
                executor.shutdown(timeout_sec=1)
            if node:
                node.destroy_node()
            if context and context.ok():
                context.shutdown()

    def stop(self):
        self._stop.set()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=2)


def register_robot_availability(app):
    monitor = app.config.get("AVAILABILITY_MONITOR")
    if monitor is None:
        simulated = app.config.get("SIGNAL_SIMULATOR") is not None
        monitor = RobotAvailability(Fleet(max_age=3.5) if simulated else None)
    app.extensions["robot_availability"] = monitor

    @app.get("/api/fleet")
    def fleet_status():
        if not app.extensions["admin_auth"]["current_admin"]():
            abort(401)
        activity_source = app.config.get('SIGNAL_SIMULATOR') or app.extensions['robot_preparation']
        fleet = monitor.fleet.snapshot()
        for name, row in fleet.items():
            moving = row.pop('moving', None)
            activity = activity_source.activity(monitor.fleet.names[name])
            if not activity:
                if moving is True:
                    activity = '이동 중'
                elif row.get('dock') is True:
                    activity = '도크에 정차 중'
                elif moving is False:
                    activity = '정지 중'
                else:
                    activity = '활동 상태 미수신'
            row['activity'] = activity
        return jsonify(fleet)

    @app.get("/api/availability/stream")
    def availability_stream():
        def events():
            while True:
                yield "event: availability\ndata: " + json.dumps(monitor.snapshot()) + "\n\n"
                time.sleep(1)

        return Response(events(), mimetype="text/event-stream", headers={
            "Cache-Control": "no-store", "X-Accel-Buffering": "no",
        })

    if not app.testing and app.config.get("ROS_AVAILABILITY_ENABLED", True):
        monitor.start(app.logger)
        atexit.register(monitor.stop)
