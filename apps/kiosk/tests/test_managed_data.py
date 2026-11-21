import tempfile
import unittest

from tests.auth_helpers import login_admin
from pathlib import Path

from app import create_app


class ManagedDataTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.config = {"TESTING": True, "DATABASE": Path(self.temp.name) / "test.sqlite3"}
        self.client = login_admin(create_app(self.config).test_client())

    def test_four_artworks_with_requested_fields(self):
        artworks = self.client.get("/api/artworks").json["artworks"]
        self.assertEqual([art["artwork_id"] for art in artworks], ["001", "002", "003", "004"])
        self.assertEqual(set(artworks[0]), {"artwork_id", "artwork_title", "artist_name", "theme", "description", "image_path", "image_alt",
            "location", "x", "y", "x_min", "x_max", "y_min", "y_max"})
        self.assertEqual(self.client.get("/api/artworks/001").json, artworks[0])
        self.assertEqual(self.client.get("/api/artworks/1").json, artworks[0])
        self.assertEqual(self.client.get("/api/artworks/999").status_code, 404)
        self.assertEqual(self.client.post("/api/artworks", json={}).status_code, 405)

    def test_artwork_edit_survives_restart_and_escapes_html(self):
        body = {"artist_name": "새 작가", "theme": '새 테마 " &', "artwork_title": '<img src=x onerror="alert(1)">',
                "description": "바뀐 설명\n두 번째 줄"}
        original = self.client.get("/api/artworks/002").json
        result = self.client.patch("/api/artworks/002", json=body)
        self.assertEqual(result.status_code, 200)
        self.assertEqual(result.json, {**original, **body})
        restarted = login_admin(create_app(self.config).test_client())
        self.assertEqual(restarted.get("/api/artworks/002").json, result.json)
        self.assertEqual(restarted.get("/api/catalog").json["artworks"][1], result.json)
        page = restarted.get("/data").get_data(as_text=True)
        self.assertNotIn(body["artwork_title"], page)
        self.assertIn("&lt;img", page)

    def test_invalid_artwork_edit_does_not_change_saved_data(self):
        original = self.client.get("/api/artworks/001").json
        for body in ({}, {"artwork_title": "   "}, {"description": None}, {"artwork_index": 2},
                     {"artist_name": "가" * 201}, {"artwork_title": ["제목"]}):
            with self.subTest(body=body):
                self.assertEqual(self.client.patch("/api/artworks/001", json=body).status_code, 400)
                self.assertEqual(self.client.get("/api/artworks/001").json, original)
        long_description = "작품🌱" * 2500
        self.assertEqual(self.client.patch("/api/artworks/001", json={"description": long_description}).status_code, 200)

    def test_robot_pose_api_is_removed(self):
        # 로봇 위치는 키오스크가 ROS /robotN/odom을 직접 구독한다. DB·HTTP로 받지 않는다.
        self.assertEqual(self.client.get("/api/robots").status_code, 404)
        self.assertEqual(self.client.put("/api/robots/robot-01", json={"active": True, "pose": None}).status_code, 404)


if __name__ == "__main__":
    unittest.main()
