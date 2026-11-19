"""요청한 SQL 구조, 기존 DB 이전, 실제 API 쓰기의 통합 검증."""
import json
import sqlite3
import tempfile
import unittest
from pathlib import Path

from app import ROOT, create_app
from database import initialize_database
from tests.auth_helpers import login_admin


class DatabaseSchemaTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / 'test.sqlite3'
        self.config = {'TESTING': True, 'DATABASE': self.path, 'ROS_PREPARATION_REQUIRED': False}

    def test_requested_tables_values_and_unknown_pose(self):
        client = create_app(self.config).test_client()
        with sqlite3.connect(self.path) as db:
            rows = db.execute('SELECT artwork_id, artwork_title, artist_name, location, theme, x, y FROM Art ORDER BY artwork_id').fetchall()
            self.assertEqual(rows, [
                ('001', 'Eaves in the Water', '박찬의', 'location1', 'nature', -2.535, 1.235),
                ('002', 'A Needle for the Sunset', '우민우', 'location2', 'color', 0.495, 3.255),
                ('003', 'A Fold in Silver', '이준혁', 'location3', 'abstract', -1.265, 5.145),
                ('004', 'A Spectrum over the River', '전유한', 'location4', 'color', 0.245, 3.735),
            ])
            self.assertEqual(db.execute('SELECT robot_id, x, y, theta FROM Pose ORDER BY robot_id').fetchall(),
                             [('robot-01', None, None, None), ('robot-02', None, None, None)])
            self.assertEqual(db.execute('SELECT COUNT(*) FROM Log').fetchone()[0], 0)
            self.assertEqual(db.execute('PRAGMA user_version').fetchone()[0], 7)
        self.assertEqual(len(client.get('/api/robots?active=all').json['robots']), 2)

    def test_management_and_tour_write_to_requested_columns(self):
        client = login_admin(create_app(self.config).test_client())
        body = {'artwork_title': 'Edited Title', 'artist_name': 'Edited Artist', 'location': '009', 'x': -2.5, 'y': 8}
        result = client.patch('/api/artworks/001', json=body)
        self.assertEqual(result.status_code, 200)
        self.assertEqual(result.json['artwork_id'], '001')
        pose = {'x': 0.5, 'y': 1, 'theta': -1.2}
        self.assertEqual(client.put('/api/robots/robot-02', json={'active': True, 'pose': pose}).status_code, 200)
        session = client.post('/api/sessions', json={'mode': 'course', 'courseId': 'A', 'artworkIds': ['art-01', 'art-02', 'art-03']}).json
        url = '/api/sessions/' + session['id']
        client.patch(url, json={'status': 'active'})
        completed = client.patch(url, json={'status': 'completed'}).json
        with sqlite3.connect(self.path) as db:
            self.assertEqual(db.execute("SELECT artwork_title, artist_name, location, x, y FROM Art WHERE artwork_id='001'").fetchone(),
                             ('Edited Title', 'Edited Artist', '009', -2.5, 8))
            self.assertEqual(db.execute("SELECT x,y,theta FROM Pose WHERE robot_id='robot-02'").fetchone(), (.5, 1, -1.2))
            self.assertEqual(db.execute('SELECT created_time, mode, route, status, end_time, record_id FROM TourSession').fetchone(),
                             (session['createdAt'], 'course', 'A', 'completed', completed['endedAt'], session['id']))
        restarted = create_app(self.config).test_client()
        self.assertEqual(restarted.get('/api/artworks/001').json, result.json)
        self.assertEqual(restarted.get(url).json, completed)

    def test_invalid_coordinates_or_fixed_id_do_not_overwrite(self):
        client = login_admin(create_app(self.config).test_client())
        original = client.get('/api/artworks/001').json
        for body in ({'x': True}, {'x': '2'}, {'y': float('nan')}, {'y': float('inf')}, {'x': 10**1000},
                     {'x': 1e13}, {'location': ''}, {'location': 'x'*65}, {'artwork_id': '005'}):
            with self.subTest(body=body):
                self.assertEqual(client.patch('/api/artworks/001', json=body).status_code, 400)
                self.assertEqual(client.get('/api/artworks/001').json, original)
        self.assertEqual(client.patch('/api/artworks/001', json={'x': None, 'y': None}).status_code, 200)

    def create_legacy_database(self, bad_pose=False):
        catalog = json.loads((ROOT / 'data/catalog.json').read_text())
        fields = ('artwork_index', 'title', 'title_en', 'artist_name', 'theme', 'description')
        with sqlite3.connect(self.path) as db:
            db.execute('CREATE TABLE artworks (' + ', '.join(key + (' INTEGER' if key in {'artwork_index', 'year'} else ' TEXT') for key in fields) + ')')
            for item in catalog['artworks'][:3]:
                art = dict(item, artwork_index=int(item['artwork_id']), title=item['artwork_title'],
                           title_en=item['artwork_title'])
                if art['artwork_index'] == 1:
                    art.update(title='수정한 작품', description='보존할 설명', title_en='My Edited Title', theme='자연')
                db.execute('INSERT INTO artworks VALUES (' + ', '.join('?' for _ in fields) + ')', [art[key] for key in fields])
            db.execute('CREATE TABLE catalog_migrations (version TEXT PRIMARY KEY)')
            db.execute('INSERT INTO catalog_migrations VALUES (?)', (catalog['artwork_version'],))
            db.execute('INSERT INTO catalog_migrations VALUES (?)', (catalog['artwork_content_version'],))
            db.execute('CREATE TABLE robots (robot_id TEXT PRIMARY KEY, active INTEGER, pose_x REAL, pose_y REAL, pose_theta REAL, frame_id TEXT, pose_updated_at INTEGER)')
            db.execute("INSERT INTO robots VALUES ('robot-01', ?, 2.5, 3, 1.5, 'map', 1234)", (2 if bad_pose else 1,))
            # 로봇 배정 컬럼이 없던 버전에서도 이전할 수 있어야 합니다.
            db.execute('CREATE TABLE sessions (id TEXT PRIMARY KEY, mode TEXT, course_id TEXT, artwork_ids TEXT, visited_ids TEXT, status TEXT, created_at INTEGER, started_at INTEGER, ended_at INTEGER)')
            db.execute("INSERT INTO sessions VALUES ('old-record', 'course', 'A', '[\"art-01\"]', '[\"art-01\"]', 'completed', 1000, 2000, 3000)")

    def test_migration_preserves_edits_pose_logs_and_runs_once(self):
        self.create_legacy_database()
        client = create_app(self.config).test_client()
        art = client.get('/api/artworks/001').json
        self.assertEqual((art['artwork_title'], art['description'], art['theme']),
                         ('My Edited Title', '보존할 설명', 'nature'))
        self.assertEqual(client.get('/api/robots/robot-01').json['pose'], {'x': 2.5, 'y': 3, 'theta': 1.5})
        record = client.get('/api/sessions/old-record').json
        self.assertEqual((record['courseId'], record['createdAt'], record['endedAt']), ('A', 1000, 3000))
        with sqlite3.connect(self.path) as db:
            tables = {row[0] for row in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
            self.assertTrue({'Art', 'Pose', 'Log'}.issubset(tables))
            self.assertTrue({'artworks', 'robots', 'sessions'}.isdisjoint(tables))
            self.assertEqual(db.execute('PRAGMA integrity_check').fetchone()[0], 'ok')
        restarted = create_app(self.config).test_client()
        self.assertEqual(restarted.get('/api/artworks/001').json, art)
        self.assertEqual(restarted.get('/api/sessions/old-record').json, record)

    def test_failed_migration_rolls_back_original_tables_and_data(self):
        self.create_legacy_database(bad_pose=True)
        with sqlite3.connect(self.path) as db:
            db.row_factory = sqlite3.Row
            with self.assertRaises(sqlite3.IntegrityError):
                initialize_database(db, ROOT)
            self.assertEqual(db.execute('SELECT COUNT(*) FROM artworks').fetchone()[0], 3)
            self.assertEqual(db.execute('SELECT active FROM robots').fetchone()[0], 2)
            self.assertEqual(db.execute('SELECT COUNT(*) FROM sessions').fetchone()[0], 1)
            self.assertIsNone(db.execute("SELECT name FROM sqlite_master WHERE name='Art'").fetchone())

    def test_migrating_duplicate_artwork_fields_preserves_coordinates_and_sessions(self):
        client = create_app(self.config).test_client()
        record = client.post('/api/sessions', json={'mode': 'free', 'artworkIds': ['art-01']}).json
        coordinates = (-8.5, None, -9.1, -8.0, 0.5, 1.0)
        for old_version in (4, 6):
            with self.subTest(old_version=old_version):
                with sqlite3.connect(self.path) as db:
                    db.execute('DROP TABLE Art')
                    db.execute("""CREATE TABLE Art (
                        artwork_id TEXT PRIMARY KEY, artwork_index INTEGER UNIQUE,
                        artwork_code TEXT, artwork_title TEXT, title TEXT, artist TEXT,
                        artist_name TEXT, location TEXT, theme TEXT, description TEXT,
                        image_path TEXT, image_alt TEXT, x REAL, y REAL
                    )""")
                    db.execute("""INSERT INTO Art VALUES
                        ('001', 1, 'ART-001', 'Keep this title', 'Other title', 'Other artist',
                         'Keep this artist', '201', 'theme', 'description', '/static/art/art-001.png', '', ?, ?),
                        ('002', 2, 'ART-002', '', 'Fallback title', 'Fallback artist',
                         '', '202', 'theme', 'description', '/static/art/art-002.png', '', NULL, 3)""",
                        coordinates[:2])
                    if old_version == 6:
                        for field in ('x_min', 'x_max', 'y_min', 'y_max'):
                            db.execute(f'ALTER TABLE Art ADD COLUMN {field} REAL')
                        db.execute("UPDATE Art SET x_min=?, x_max=?, y_min=?, y_max=? WHERE artwork_id='001'",
                                   coordinates[2:])
                    db.execute(f'PRAGMA user_version={old_version}')

                restarted = create_app(self.config).test_client()
                artwork = restarted.get('/api/artworks/001').json
                self.assertEqual((artwork['artwork_title'], artwork['artist_name'], artwork['location']),
                                 ('Keep this title', 'Keep this artist', '201'))
                self.assertEqual((artwork['x'], artwork['y']), coordinates[:2])
                if old_version == 6:
                    self.assertEqual(tuple(artwork[key] for key in ('x_min', 'x_max', 'y_min', 'y_max')),
                                     coordinates[2:])
                fallback = restarted.get('/api/artworks/002').json
                self.assertEqual((fallback['artwork_title'], fallback['artist_name']),
                                 ('Fallback title', 'Fallback artist'))
                self.assertEqual(restarted.get('/api/artworks/004').json['artwork_id'], '004')
                self.assertEqual(restarted.get('/api/sessions/' + record['id']).json, record)
                with sqlite3.connect(self.path) as db:
                    self.assertEqual({row[1] for row in db.execute('PRAGMA table_info(Art)')}, set(artwork))
                    self.assertEqual(db.execute('PRAGMA integrity_check').fetchone()[0], 'ok')
                twice = create_app(self.config).test_client()
                self.assertEqual(twice.get('/api/artworks/001').json, artwork)

    def test_seoul_collection_applies_once_and_later_sql_changes_are_live(self):
        app = create_app(self.config)
        client = app.test_client()
        with sqlite3.connect(self.path) as db:
            db.execute("UPDATE catalog_migrations SET version='collection-2026-v1' WHERE version='seoul-collection-v1'")
            db.execute("""UPDATE Art SET artwork_title='old-title', artist_name='old-artist',
                location='old-room', theme='old-theme', description='old-description',
                image_path='/static/art/art-001.png', image_alt='old-image',
                x=12, y=-4, x_min=11, x_max=13, y_min=-5, y_max=-3 WHERE artwork_id='001'""")
        restarted = create_app(self.config).test_client()
        row = restarted.get('/api/artworks/001').json
        seed = json.loads((ROOT / 'data/catalog.json').read_text(encoding='utf-8'))['artworks'][0]
        metadata = ('artwork_title', 'artist_name', 'location', 'theme', 'description', 'image_path', 'image_alt')
        self.assertEqual({key: row[key] for key in metadata}, {key: seed[key] for key in metadata})
        self.assertEqual([row[key] for key in ('x', 'y', 'x_min', 'x_max', 'y_min', 'y_max')],
                         [12, -4, 11, 13, -5, -3])
        with sqlite3.connect(self.path) as db:
            db.execute("UPDATE Art SET artwork_title='Live SQL Title', artist_name='Live SQL Artist', location='New Room', theme='New Theme', description='Live SQL Description' WHERE artwork_id='001'")
        # 같은 앱에서도 새 요청 즉시 최신 DB 값이 반환됩니다.
        live = client.get('/api/catalog').json['artworks'][0]
        self.assertEqual([live[key] for key in ('artwork_title','artist_name','location','theme','description')],
                         ['Live SQL Title','Live SQL Artist','New Room','New Theme','Live SQL Description'])
        self.assertEqual(create_app(self.config).test_client().get('/api/artworks/001').json, live)
