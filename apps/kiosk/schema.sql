CREATE TABLE IF NOT EXISTS Art (
    artwork_id TEXT NOT NULL PRIMARY KEY CHECK (artwork_id GLOB '[0-9][0-9][0-9]' AND artwork_id <> '000'),
    artwork_title TEXT NOT NULL DEFAULT '',
    artist_name TEXT NOT NULL,
    location TEXT NOT NULL,
    theme TEXT NOT NULL,
    description TEXT NOT NULL,
    x REAL,
    y REAL,
    image_path TEXT NOT NULL DEFAULT '',
    image_alt TEXT NOT NULL DEFAULT '',
    x_min REAL,
    x_max REAL,
    y_min REAL,
    y_max REAL,
    CHECK (length(trim(location)) BETWEEN 1 AND 64),
    CHECK (x IS NULL OR x BETWEEN -1e12 AND 1e12),
    CHECK (y IS NULL OR y BETWEEN -1e12 AND 1e12)
);

CREATE TABLE IF NOT EXISTS catalog_migrations (
    version TEXT PRIMARY KEY
);

CREATE TABLE IF NOT EXISTS admin_users (
    id INTEGER PRIMARY KEY CHECK (id = 1),
    username TEXT NOT NULL UNIQUE,
    password_hash TEXT NOT NULL,
    auth_version INTEGER NOT NULL DEFAULT 1,
    created_at INTEGER NOT NULL
);

CREATE TABLE IF NOT EXISTS admin_login_attempts (
    client_address TEXT PRIMARY KEY,
    failures INTEGER NOT NULL,
    window_started REAL NOT NULL,
    blocked_until REAL NOT NULL DEFAULT 0
);

CREATE TABLE IF NOT EXISTS admin_sessions (
    token_hash TEXT PRIMARY KEY,
    admin_id INTEGER NOT NULL REFERENCES admin_users(id),
    expires_at REAL NOT NULL
);

CREATE TABLE IF NOT EXISTS TourSession (
    created_time INTEGER NOT NULL,
    mode TEXT NOT NULL CHECK (mode IN ('course', 'free')),
    route TEXT,
    status TEXT NOT NULL CHECK (status IN ('calling', 'active', 'completed', 'cancelled')),
    end_time INTEGER,
    record_id TEXT NOT NULL PRIMARY KEY,
    artwork_ids TEXT NOT NULL,
    visited_ids TEXT NOT NULL DEFAULT '[]',
    started_at INTEGER,
    robot_id TEXT,
    robot_namespace TEXT,
    visitor_id TEXT
);

CREATE INDEX IF NOT EXISTS tour_session_created_time ON TourSession (created_time DESC, record_id DESC);

CREATE TABLE IF NOT EXISTS Log (
    log_id INTEGER PRIMARY KEY AUTOINCREMENT,
    robot_id TEXT NOT NULL CHECK (length(robot_id) BETWEEN 1 AND 64),
    sent_at INTEGER NOT NULL CHECK (sent_at BETWEEN 0 AND 253402300799999),
    message TEXT NOT NULL CHECK (length(trim(message)) BETWEEN 1 AND 10000)
);

CREATE INDEX IF NOT EXISTS robot_log_sent_at ON Log (sent_at DESC, log_id DESC);
