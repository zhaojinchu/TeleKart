"""SQLite storage: connection policy, schema, migrations, queries."""

from __future__ import annotations

import sqlite3
import threading
import time
from contextlib import contextmanager
from pathlib import Path
from typing import Any, Iterator, Sequence

from ..config import paths
from ..core.log import get_logger

_log = get_logger(__name__)

SCHEMA_VERSION = 1

_SCHEMA_V1 = (
    """
    CREATE TABLE IF NOT EXISTS sessions (
        id            INTEGER PRIMARY KEY AUTOINCREMENT,
        started_at    REAL    NOT NULL,
        ended_at      REAL,
        car_id        TEXT    NOT NULL DEFAULT '',
        driver        TEXT    NOT NULL DEFAULT '',
        fw_version    TEXT    NOT NULL DEFAULT '',
        app_version   TEXT    NOT NULL DEFAULT '',
        proto_version INTEGER NOT NULL DEFAULT 0,
        session_id    INTEGER NOT NULL DEFAULT 0,
        notes         TEXT    NOT NULL DEFAULT '',
        samples       INTEGER NOT NULL DEFAULT 0,
        distance_m    REAL    NOT NULL DEFAULT 0.0,
        max_speed_mps REAL    NOT NULL DEFAULT 0.0,
        laps          INTEGER NOT NULL DEFAULT 0,
        best_lap      REAL
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS telemetry (
        session   INTEGER NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
        t         REAL    NOT NULL,
        seq       INTEGER NOT NULL,
        state     INTEGER NOT NULL,
        faults    INTEGER NOT NULL,
        flags     INTEGER NOT NULL,
        speed     REAL    NOT NULL,
        v_max     REAL    NOT NULL,
        rpm_l     REAL    NOT NULL,
        rpm_r     REAL    NOT NULL,
        rpm_tl    REAL    NOT NULL,
        rpm_tr    REAL    NOT NULL,
        duty_l    REAL    NOT NULL,
        duty_r    REAL    NOT NULL,
        steer     REAL    NOT NULL,
        servo_us  INTEGER NOT NULL,
        x         REAL    NOT NULL,
        y         REAL    NOT NULL,
        heading   REAL    NOT NULL,
        distance  REAL    NOT NULL,
        slip      REAL    NOT NULL,
        pack_v    REAL    NOT NULL,
        cpu_c     REAL    NOT NULL,
        rtt       REAL    NOT NULL
    )
    """,
    "CREATE INDEX IF NOT EXISTS telemetry_session_t ON telemetry(session, t)",
    """
    CREATE TABLE IF NOT EXISTS inputs (
        session  INTEGER NOT NULL REFERENCES sessions(id) ON DELETE CASCADE,
        t        REAL    NOT NULL,
        seq      INTEGER NOT NULL,
        steering REAL    NOT NULL,
        throttle REAL    NOT NULL,
        brake    REAL    NOT NULL,
        flags    INTEGER NOT NULL
    )
    """,
    "CREATE INDEX IF NOT EXISTS inputs_session_t ON inputs(session, t)",
    """
    CREATE TABLE IF NOT EXISTS events (
        id      INTEGER PRIMARY KEY AUTOINCREMENT,
        session INTEGER REFERENCES sessions(id) ON DELETE CASCADE,
        t       REAL NOT NULL,
        kind    TEXT NOT NULL,
        detail  TEXT NOT NULL DEFAULT ''
    )
    """,
    "CREATE INDEX IF NOT EXISTS events_session_t ON events(session, t)",
    """
    CREATE TABLE IF NOT EXISTS laps (
        id       INTEGER PRIMARY KEY AUTOINCREMENT,
        session  INTEGER REFERENCES sessions(id) ON DELETE CASCADE,
        idx      INTEGER NOT NULL,
        t_start  REAL    NOT NULL,
        t_end    REAL    NOT NULL,
        duration REAL    NOT NULL,
        valid    INTEGER NOT NULL DEFAULT 1,
        source   TEXT    NOT NULL DEFAULT ''
    )
    """,
    "CREATE INDEX IF NOT EXISTS laps_session ON laps(session, idx)",
)

#: One entry per schema version. Index N-1 takes user_version from N-1 to N.
_MIGRATIONS: tuple[tuple[str, ...], ...] = (_SCHEMA_V1,)


class Database:
    """One connection, one lock, WAL.

    The connection is shared across threads with ``check_same_thread=False``
    and an explicit lock, rather than one connection per thread. The reason is
    the recorder: it batches writes from its own thread while the GUI thread
    runs the occasional query, and two connections to the same file would then
    be two writers -- which under WAL means ``SQLITE_BUSY`` at exactly the
    moment a session is being closed out.
    """

    def __init__(self, path: Path | None = None) -> None:
        self.path = path or paths.database_file()
        self._lock = threading.RLock()
        self._conn: sqlite3.Connection | None = None

    # -- lifecycle ----------------------------------------------------------

    def connect(self) -> sqlite3.Connection:
        with self._lock:
            if self._conn is not None:
                return self._conn
            self.path.parent.mkdir(parents=True, exist_ok=True)
            conn = sqlite3.connect(str(self.path), check_same_thread=False, timeout=5.0)
            conn.row_factory = sqlite3.Row
            # WAL so a reader never blocks the recorder's writes. NORMAL
            # synchronous is the right durability trade here: the worst a power
            # cut can cost is the last half second of a recording, and paying
            # an fsync per commit at 25 Hz would be absurd.
            conn.execute("PRAGMA journal_mode=WAL")
            conn.execute("PRAGMA synchronous=NORMAL")
            conn.execute("PRAGMA foreign_keys=ON")
            conn.execute("PRAGMA busy_timeout=5000")
            self._conn = conn
            self._migrate(conn)
            return conn

    def close(self) -> None:
        with self._lock:
            conn, self._conn = self._conn, None
            if conn is None:
                return
            try:
                conn.execute("PRAGMA optimize")
                conn.close()
            except sqlite3.Error as exc:
                _log.warning("closing the database raised: %s", exc)

    def _migrate(self, conn: sqlite3.Connection) -> None:
        """Apply every migration the file has not seen.

        ``user_version`` is sqlite's own header field: it needs no table, it
        survives a VACUUM, and reading it costs nothing -- which matters
        because this runs on every startup.
        """
        current = int(conn.execute("PRAGMA user_version").fetchone()[0])
        if current > SCHEMA_VERSION:
            raise RuntimeError(
                f"{self.path} was written by a newer build (schema {current} > "
                f"{SCHEMA_VERSION}); refusing to touch it"
            )
        if current == SCHEMA_VERSION:
            return
        for version in range(current, SCHEMA_VERSION):
            _log.info("migrating %s: schema %d -> %d", self.path.name, version, version + 1)
            with conn:
                for statement in _MIGRATIONS[version]:
                    conn.execute(statement)
                # No parameter binding on PRAGMA -- sqlite does not allow it,
                # and the value is a loop counter, not user input.
                conn.execute(f"PRAGMA user_version={version + 1}")

    # -- access -------------------------------------------------------------

    @contextmanager
    def transaction(self) -> Iterator[sqlite3.Connection]:
        conn = self.connect()
        with self._lock:
            with conn:
                yield conn

    def execute(self, sql: str, params: Sequence[Any] = ()) -> sqlite3.Cursor:
        conn = self.connect()
        with self._lock:
            with conn:
                return conn.execute(sql, params)

    def executemany(self, sql: str, rows: Sequence[Sequence[Any]]) -> None:
        if not rows:
            return
        conn = self.connect()
        with self._lock:
            with conn:
                conn.executemany(sql, rows)

    def query(self, sql: str, params: Sequence[Any] = ()) -> list[sqlite3.Row]:
        conn = self.connect()
        with self._lock:
            return list(conn.execute(sql, params))

    def query_one(self, sql: str, params: Sequence[Any] = ()) -> sqlite3.Row | None:
        rows = self.query(sql, params)
        return rows[0] if rows else None

    # -- sessions -----------------------------------------------------------

    def create_session(
        self,
        *,
        car_id: str = "",
        driver: str = "",
        fw_version: str = "",
        app_version: str = "",
        proto_version: int = 0,
        session_id: int = 0,
        started_at: float | None = None,
    ) -> int:
        cursor = self.execute(
            "INSERT INTO sessions (started_at, car_id, driver, fw_version, app_version,"
            " proto_version, session_id) VALUES (?,?,?,?,?,?,?)",
            (
                started_at if started_at is not None else time.time(),
                car_id,
                driver,
                fw_version,
                app_version,
                proto_version,
                session_id,
            ),
        )
        record_id = cursor.lastrowid
        if record_id is None:
            raise sqlite3.DatabaseError("could not create a session row")
        return int(record_id)

    def finish_session(
        self,
        record_id: int,
        *,
        samples: int = 0,
        distance_m: float = 0.0,
        max_speed_mps: float = 0.0,
        laps: int = 0,
        best_lap: float | None = None,
        ended_at: float | None = None,
    ) -> None:
        self.execute(
            "UPDATE sessions SET ended_at=?, samples=?, distance_m=?, max_speed_mps=?,"
            " laps=?, best_lap=? WHERE id=?",
            (
                ended_at if ended_at is not None else time.time(),
                samples,
                distance_m,
                max_speed_mps,
                laps,
                best_lap,
                record_id,
            ),
        )

    def set_notes(self, record_id: int, notes: str) -> None:
        self.execute("UPDATE sessions SET notes=? WHERE id=?", (notes, record_id))

    def list_sessions(self, limit: int = 100) -> list[sqlite3.Row]:
        return self.query(
            "SELECT * FROM sessions ORDER BY started_at DESC LIMIT ?", (limit,)
        )

    def session(self, record_id: int) -> sqlite3.Row | None:
        return self.query_one("SELECT * FROM sessions WHERE id=?", (record_id,))

    def telemetry(self, record_id: int, *, limit: int = 0) -> list[sqlite3.Row]:
        sql = "SELECT * FROM telemetry WHERE session=? ORDER BY t"
        if limit > 0:
            return self.query(sql + " LIMIT ?", (record_id, limit))
        return self.query(sql, (record_id,))

    def laps(self, record_id: int) -> list[sqlite3.Row]:
        return self.query("SELECT * FROM laps WHERE session=? ORDER BY idx", (record_id,))

    def events(self, record_id: int) -> list[sqlite3.Row]:
        return self.query("SELECT * FROM events WHERE session=? ORDER BY t", (record_id,))

    def delete_session(self, record_id: int) -> None:
        # The child rows go with it via ON DELETE CASCADE, which is why
        # foreign_keys=ON is set on every connection.
        self.execute("DELETE FROM sessions WHERE id=?", (record_id,))

    def prune(self, keep_days: int) -> int:
        """Delete sessions older than ``keep_days``. Returns how many went.

        Recording is on by default, so something has to bound the file. A
        session is about 8 MB an hour; ninety days of daily driving is still
        under a gigabyte, which is why the default is generous.
        """
        if keep_days <= 0:
            return 0
        cutoff = time.time() - keep_days * 86400.0
        cursor = self.execute("DELETE FROM sessions WHERE started_at < ?", (cutoff,))
        removed = cursor.rowcount if cursor.rowcount and cursor.rowcount > 0 else 0
        if removed:
            _log.info("pruned %d session(s) older than %d days", removed, keep_days)
            with self._lock:
                conn = self.connect()
                conn.execute("VACUUM")
        return removed

    def vacuum(self) -> None:
        with self._lock:
            self.connect().execute("VACUUM")
