"""Art / Pose / Log 구조 생성과 기존 SQLite 데이터의 원자적 이전."""

import json


ARTWORK_SELECT = """SELECT artwork_id, artwork_title, artist_name, theme, description,
    image_path, image_alt, location, x, y, x_min, x_max, y_min, y_max FROM Art"""
SESSION_SELECT = """SELECT *, record_id AS id, route AS course_id,
    created_time AS created_at, end_time AS ended_at FROM TourSession"""


def artwork_rows(db):
    return [dict(row) for row in db.execute(ARTWORK_SELECT + " ORDER BY artwork_id")]


def insert_artwork(db, art, replace=False):
    if replace:
        updates = {key: value for key, value in art.items()
                   if key not in {'artwork_id', 'x', 'y'}}
        db.execute(
            f"UPDATE Art SET {', '.join(field + '=?' for field in updates)} WHERE artwork_id=?",
            [*updates.values(), art['artwork_id']],
        )
    else:
        db.execute(
            f"INSERT OR IGNORE INTO Art ({', '.join(art)}) VALUES ({', '.join('?' for _ in art)})",
            list(art.values()),
        )


def initialize_database(db, root):
    catalog = json.loads((root / 'data/catalog.json').read_text(encoding='utf-8'))
    db.execute('PRAGMA journal_mode = WAL')
    with db:
        db.execute('BEGIN IMMEDIATE')
        old_tables = {row['name'] for row in db.execute("SELECT name FROM sqlite_master WHERE type='table'")}
        if 'artworks' in old_tables:
            columns = {row['name'] for row in db.execute('PRAGMA table_info(artworks)')}
            if 'data' in columns:
                db.execute('ALTER TABLE artworks RENAME TO artworks_legacy_v1')
                old_tables.remove('artworks')
        if 'Log' in old_tables:
            log_columns = {row['name'] for row in db.execute('PRAGMA table_info(Log)')}
            if 'record_id' in log_columns:
                # 기존 관람 상태는 키오스크 내부 동작을 위해 보존합니다.
                db.execute('ALTER TABLE Log RENAME TO TourSession')
                db.execute('DROP INDEX IF EXISTS log_created_time')
        previous_art_columns = {row['name'] for row in db.execute('PRAGMA table_info(Art)')}
        rebuild_art = 'artwork_index' in previous_art_columns
        if rebuild_art:
            db.execute('ALTER TABLE Art RENAME TO Art_before_v7')
        for statement in (root / 'schema.sql').read_text(encoding='utf-8').split(';'):
            if statement.strip():
                db.execute(statement)

        if rebuild_art:
            bounds = ('x_min', 'x_max', 'y_min', 'y_max')
            old_bounds = ', '.join(field if field in previous_art_columns else 'NULL' for field in bounds)
            db.execute(f"""INSERT INTO Art
                (artwork_id, artwork_title, artist_name, location, theme, description,
                 x, y, image_path, image_alt, x_min, x_max, y_min, y_max)
                SELECT artwork_id,
                       CASE WHEN trim(artwork_title) = '' THEN title ELSE artwork_title END,
                       CASE WHEN trim(artist_name) = '' THEN artist ELSE artist_name END,
                       location, theme, description, x, y, image_path, image_alt, {old_bounds}
                FROM Art_before_v7""")
            db.execute('DROP TABLE Art_before_v7')

        stored_coordinate_ids = {row['artwork_id'] for row in db.execute('SELECT artwork_id FROM Art')}
        # 이전과 테이블 교체를 하나의 트랜잭션으로 처리하여 실패하면 원래 상태로 되돌립니다.
        if 'artworks' in old_tables:
            for row in db.execute('SELECT * FROM artworks').fetchall():
                old = dict(row)
                artwork_id = f"{old['artwork_index']:03d}"
                art = next(dict(item) for item in catalog['artworks']
                           if item['artwork_id'] == artwork_id)
                art.update({key: value for key, value in old.items() if key in art})
                art['artwork_title'] = old.get('title_en') or old.get('title') or art['artwork_title']
                art['artist_name'] = old.get('artist_name') or old.get('artist_name_en') or art['artist_name']
                art['theme'] = {'자연': 'nature', '색채': 'color', '추상': 'abstract'}.get(art['theme'], art['theme'])
                insert_artwork(db, art)
                if {'x', 'y'} & old.keys():
                    stored_coordinate_ids.add(artwork_id)
            db.execute('DROP TABLE artworks')
        if 'robots' in old_tables:
            db.execute('''INSERT INTO Pose (robot_id, x, y, theta, active, frame_id, pose_updated_at)
                SELECT robot_id, pose_x, pose_y, pose_theta, active, frame_id, pose_updated_at FROM robots''')
            db.execute('DROP TABLE robots')
        if 'sessions' in old_tables:
            columns = {row['name'] for row in db.execute('PRAGMA table_info(sessions)')}
            optional = ', '.join(key if key in columns else 'NULL'
                                 for key in ('robot_id', 'robot_namespace', 'visitor_id'))
            db.execute(f'''INSERT INTO TourSession (record_id, mode, route, artwork_ids, visited_ids,
                status, created_time, started_at, end_time, robot_id, robot_namespace, visitor_id)
                SELECT id, mode, course_id, artwork_ids, visited_ids, status, created_at,
                started_at, ended_at, {optional} FROM sessions''')
            db.execute('DROP TABLE sessions')

        for art in catalog['artworks']:
            insert_artwork(db, art)
        # 새 전시 데이터는 스키마 이전 여부와 관계없이 버전당 한 번 반영합니다.
        artwork_version = catalog.get('artwork_version')
        if artwork_version and not db.execute(
            'SELECT 1 FROM catalog_migrations WHERE version=?', (artwork_version,)
        ).fetchone():
            for art in catalog['artworks']:
                insert_artwork(db, art, replace=True)
            db.execute('INSERT INTO catalog_migrations (version) VALUES (?)', (artwork_version,))
        # 요청한 임시 표 내용은 한 번만 적용합니다. 이후 SQL/관리 화면 편집은 보존합니다.
        content_version = catalog.get('artwork_content_version')
        if content_version and not db.execute(
            'SELECT 1 FROM catalog_migrations WHERE version=?', (content_version,)
        ).fetchone():
            if not rebuild_art:
                content_fields = ('artwork_title', 'artist_name', 'location', 'theme', 'description')
                assignments = ', '.join(field + '=?' for field in content_fields)
                for art in catalog['artworks']:
                    db.execute(f'UPDATE Art SET {assignments} WHERE artwork_id=?',
                               [art[field] for field in content_fields] + [art['artwork_id']])
            db.execute('INSERT INTO catalog_migrations (version) VALUES (?)', (content_version,))
        for field, version_key in (('artist_name', 'artwork_artist_version'),
                                   ('location', 'artwork_location_version')):
            field_version = catalog.get(version_key)
            if field_version and not db.execute(
                'SELECT 1 FROM catalog_migrations WHERE version=?', (field_version,)
            ).fetchone():
                db.executemany(f'UPDATE Art SET {field}=? WHERE artwork_id=?',
                               [(art[field], art['artwork_id']) for art in catalog['artworks']])
                db.execute('INSERT INTO catalog_migrations (version) VALUES (?)', (field_version,))
        description_update = catalog.get('artwork_description_update')
        if description_update and not db.execute(
            'SELECT 1 FROM catalog_migrations WHERE version=?', (description_update['version'],)
        ).fetchone():
            previous_descriptions = description_update['previous']
            db.executemany('UPDATE Art SET description=? WHERE artwork_id=? AND description=?',
                           [(art['description'], art['artwork_id'], previous_descriptions[art['artwork_id']])
                            for art in catalog['artworks'] if art['artwork_id'] in previous_descriptions])
            db.execute('INSERT INTO catalog_migrations (version) VALUES (?)', (description_update['version'],))
        for course in catalog['courses']:
            db.execute('INSERT OR IGNORE INTO courses (id, data) VALUES (?, ?)',
                       (course['id'], json.dumps(course, ensure_ascii=False)))
        version = catalog.get('version')
        if version and not db.execute(
            'SELECT 1 FROM catalog_migrations WHERE version=?', (version,)
        ).fetchone():
            for course in catalog['courses']:
                db.execute('UPDATE courses SET data=? WHERE id=?',
                           (json.dumps(course, ensure_ascii=False), course['id']))
            db.execute('INSERT INTO catalog_migrations (version) VALUES (?)', (version,))
        for row in db.execute('SELECT id, data FROM courses').fetchall():
            old = json.loads(row['data'])
            if any(item not in {'art-01', 'art-02', 'art-03'} for item in old['ids']):
                old.update(ids=['art-03', 'art-02', 'art-01'], minutes=15, theme='추상 · 색채 · 자연')
                db.execute('UPDATE courses SET data=? WHERE id=?',
                           (json.dumps(old, ensure_ascii=False), row['id']))

        # 위치 미수신 상태는 NULL입니다. robot-02는 수신/활성화 전까지 비활성입니다.
        db.execute("INSERT OR IGNORE INTO Pose (robot_id, active) VALUES ('robot-01', 1)")
        db.execute("INSERT OR IGNORE INTO Pose (robot_id, active) VALUES ('robot-02', 0)")
        if 'x_min' not in previous_art_columns:
            ranges = [(-2.56,-2.51,1.02,1.45), (-2.52,3.51,2.5,4.01),
                      (-1.51,-1.02,5.14,5.15), (0.06,0.43,3.51,3.96)]
            for index, (left, right, bottom, top) in enumerate(ranges, 1):
                artwork_id = f'{index:03d}'
                db.execute("UPDATE Art SET x_min=?,x_max=?,y_min=?,y_max=? WHERE artwork_id=?",
                           (left, right, bottom, top, artwork_id))
                if artwork_id not in stored_coordinate_ids:
                    db.execute("UPDATE Art SET x=?,y=? WHERE artwork_id=?",
                               (round((left + right) / 2, 4), round((bottom + top) / 2, 4), artwork_id))
        db.execute('PRAGMA user_version = 7')
