import json
import tempfile
import unittest
from pathlib import Path

from app import create_app
from fleet import Fleet
from robot_availability import RobotAvailability


class AvailabilityTest(unittest.TestCase):
    def test_waiting_threshold_and_expiry(self):
        clock = [0]
        fleet = Fleet(clock=lambda: clock[0])
        monitor = RobotAvailability(fleet)

        self.assertEqual(monitor.snapshot(), {"status": "waiting", "available": None})
        fleet.receive("tb1", moving=False)
        self.assertEqual(monitor.snapshot()["status"], "waiting")
        fleet.receive("tb1", dock=True, battery=30)
        fleet.receive("tb4", dock=False, battery=80)
        self.assertEqual(monitor.snapshot(), {"status": "unavailable", "available": False})
        fleet.receive("tb4", dock=True)
        self.assertEqual(monitor.snapshot(), {"status": "available", "available": True})

        clock[0] = 16
        self.assertEqual(monitor.snapshot(), {"status": "stale", "available": None})
        fleet.receive("tb4", dock=True, battery=80)
        self.assertEqual(monitor.snapshot()["status"], "available")

    def test_stream_sends_fleet_state(self):
        with tempfile.TemporaryDirectory() as folder:
            app = create_app({"TESTING": True, "DATABASE": Path(folder) / "test.sqlite3"})
            monitor = app.extensions["robot_availability"]
            monitor.fleet.receive("tb1", dock=True, battery=80)
            response = app.test_client().get("/api/availability/stream", buffered=False)
            try:
                self.assertEqual(response.mimetype, "text/event-stream")
                frame = next(iter(response.response)).decode()
                self.assertIn("event: availability", frame)
                self.assertEqual(json.loads(frame.split("data: ")[1]),
                                 {"status": "available", "available": True})
            finally:
                response.close()
