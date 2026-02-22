#!/usr/bin/env python3
"""INAV Flight History Database.

Stores flight analysis results in SQLite for progression tracking.
Each analyzed flight is recorded with scores, per-axis measurements,
motor data, configuration snapshot, and recommended actions.

Usage:
    from inav_flight_db import FlightDB

    db = FlightDB("./inav_flights.db")
    flight_id = db.store_flight(plan, config, data, hover_osc, motor_analysis,
                                pid_results, noise_results, diff_raw=None)

    # Get progression for a craft
    history = db.get_craft_history("NAZGUL 10", limit=10)
"""

import sqlite3
import json
import os
from datetime import datetime

VERSION = "1.0.0"

SCHEMA_VERSION = 1

SCHEMA = """
CREATE TABLE IF NOT EXISTS schema_info (
    key TEXT PRIMARY KEY,
    value TEXT
);

CREATE TABLE IF NOT EXISTS flights (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    timestamp TEXT NOT NULL,
    craft TEXT NOT NULL,
    firmware TEXT,
    board TEXT,
    duration_s REAL,
    sample_rate REAL,
    total_frames INTEGER,
    log_file TEXT,
    -- Scores
    overall_score REAL,
    noise_score REAL,
    pid_score REAL,
    pid_measurable INTEGER,
    motor_score REAL,
    osc_score REAL,
    verdict TEXT,
    verdict_text TEXT,
    -- Raw diff output
    diff_raw TEXT
);

CREATE TABLE IF NOT EXISTS flight_axes (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    flight_id INTEGER NOT NULL,
    axis TEXT NOT NULL,
    -- Hover oscillation
    hover_severity TEXT,
    hover_rms REAL,
    hover_p2p REAL,
    hover_freq_hz REAL,
    hover_cause TEXT,
    hover_seconds REAL,
    -- PID response
    overshoot_pct REAL,
    delay_ms REAL,
    tracking_error REAL,
    n_steps INTEGER,
    -- Config at time of flight
    p_value INTEGER,
    i_value INTEGER,
    d_value INTEGER,
    ff_value INTEGER,
    FOREIGN KEY (flight_id) REFERENCES flights(id) ON DELETE CASCADE
);

CREATE TABLE IF NOT EXISTS flight_motors (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    flight_id INTEGER NOT NULL,
    motor_num INTEGER NOT NULL,
    avg_pct REAL,
    saturation_pct REAL,
    FOREIGN KEY (flight_id) REFERENCES flights(id) ON DELETE CASCADE
);

CREATE TABLE IF NOT EXISTS flight_config (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    flight_id INTEGER NOT NULL,
    param TEXT NOT NULL,
    value TEXT,
    FOREIGN KEY (flight_id) REFERENCES flights(id) ON DELETE CASCADE
);

CREATE TABLE IF NOT EXISTS flight_actions (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    flight_id INTEGER NOT NULL,
    priority INTEGER,
    urgency TEXT,
    category TEXT,
    action TEXT,
    reason TEXT,
    deferred INTEGER DEFAULT 0,
    FOREIGN KEY (flight_id) REFERENCES flights(id) ON DELETE CASCADE
);

CREATE INDEX IF NOT EXISTS idx_flights_craft ON flights(craft);
CREATE INDEX IF NOT EXISTS idx_flights_timestamp ON flights(timestamp);
CREATE INDEX IF NOT EXISTS idx_flight_axes_flight ON flight_axes(flight_id);
CREATE INDEX IF NOT EXISTS idx_flight_motors_flight ON flight_motors(flight_id);
CREATE INDEX IF NOT EXISTS idx_flight_config_flight ON flight_config(flight_id);
CREATE INDEX IF NOT EXISTS idx_flight_actions_flight ON flight_actions(flight_id);
"""


class FlightDB:
    """SQLite database for storing and querying flight analysis history."""

    def __init__(self, db_path="./inav_flights.db"):
        self.db_path = db_path
        self._conn = None

    def _connect(self):
        if self._conn is None:
            self._conn = sqlite3.connect(self.db_path)
            self._conn.row_factory = sqlite3.Row
            self._conn.execute("PRAGMA journal_mode=WAL")
            self._conn.execute("PRAGMA foreign_keys=ON")
            self._init_schema()
        return self._conn

    def _init_schema(self):
        conn = self._conn
        conn.executescript(SCHEMA)
        # Check/set schema version
        cur = conn.execute(
            "SELECT value FROM schema_info WHERE key='schema_version'")
        row = cur.fetchone()
        if row is None:
            conn.execute(
                "INSERT INTO schema_info (key, value) VALUES ('schema_version', ?)",
                (str(SCHEMA_VERSION),))
            conn.commit()

    def close(self):
        if self._conn:
            self._conn.close()
            self._conn = None

    def store_flight(self, plan, config, data, hover_osc=None,
                     motor_analysis=None, pid_results=None,
                     noise_results=None, log_file=None, diff_raw=None):
        """Store a complete flight analysis.

        Args:
            plan: Analysis plan dict (from generate_action_plan)
            config: Parsed config dict
            data: Parsed flight data dict
            hover_osc: Hover oscillation results list
            motor_analysis: Motor analysis dict
            pid_results: PID analysis results list
            noise_results: Noise analysis results list
            log_file: Path to the log file
            diff_raw: Raw CLI diff output string

        Returns:
            flight_id (int), or existing flight_id if duplicate detected
        """
        conn = self._connect()
        scores = plan.get("scores", {})

        # Extract metadata
        craft = config.get("craft_name", "unknown")
        firmware = config.get("firmware_version", "")
        board = config.get("board", "")
        duration = float(data["time_s"][-1]) if "time_s" in data and len(data["time_s"]) > 0 else 0
        sr = data.get("sample_rate", 0)
        total_frames = len(data.get("time_s", []))

        # ── Deduplication ──
        # Content-based: match on craft + total_frames + duration + firmware.
        # This catches re-downloads of the same flash (different filenames)
        # as well as re-analysis of the same file.
        existing = conn.execute("""
            SELECT id FROM flights
            WHERE craft = ? AND total_frames = ?
            AND ABS(duration_s - ?) < 0.1
            AND firmware = ?
        """, (craft, total_frames, duration, firmware)).fetchone()
        if existing:
            return existing[0], False  # (flight_id, is_new=False)

        # Insert flight record
        cur = conn.execute("""
            INSERT INTO flights (timestamp, craft, firmware, board, duration_s,
                sample_rate, total_frames, log_file,
                overall_score, noise_score, pid_score, pid_measurable,
                motor_score, osc_score, verdict, verdict_text, diff_raw)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """, (
            datetime.now().isoformat(),
            craft, firmware, board, duration, sr, total_frames, log_file,
            scores.get("overall"),
            scores.get("noise"),
            scores.get("pid"),
            1 if scores.get("pid_measurable", False) else 0,
            scores.get("motor"),
            scores.get("gyro_oscillation"),
            plan.get("verdict"),
            plan.get("verdict_text"),
            diff_raw,
        ))
        flight_id = cur.lastrowid

        # Store per-axis data
        axis_names = ["Roll", "Pitch", "Yaw"]
        for i, axis in enumerate(axis_names):
            axis_l = axis.lower()
            # Hover oscillation
            h_sev = h_rms = h_p2p = h_freq = h_cause = h_sec = None
            if hover_osc:
                for osc in hover_osc:
                    if osc["axis"] == axis:
                        h_sev = osc["severity"]
                        h_rms = osc["gyro_rms"]
                        h_p2p = osc["gyro_p2p"]
                        h_freq = osc.get("dominant_freq_hz")
                        h_cause = osc.get("cause")
                        h_sec = osc.get("hover_seconds")
                        break

            # PID response
            os_pct = dl_ms = trk_err = n_steps = None
            if pid_results and i < len(pid_results) and pid_results[i]:
                pid = pid_results[i]
                os_pct = pid.get("avg_overshoot_pct")
                dl_ms = pid.get("tracking_delay_ms")
                trk_err = pid.get("rms_error")
                n_steps = pid.get("n_steps")

            # Config values
            p_val = config.get(f"{axis_l}_p")
            i_val = config.get(f"{axis_l}_i")
            d_val = config.get(f"{axis_l}_d")
            ff_val = config.get(f"{axis_l}_ff")

            conn.execute("""
                INSERT INTO flight_axes (flight_id, axis,
                    hover_severity, hover_rms, hover_p2p, hover_freq_hz,
                    hover_cause, hover_seconds,
                    overshoot_pct, delay_ms, tracking_error, n_steps,
                    p_value, i_value, d_value, ff_value)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                flight_id, axis,
                h_sev, h_rms, h_p2p, h_freq, h_cause, h_sec,
                os_pct, dl_ms, trk_err, n_steps,
                p_val, i_val, d_val, ff_val,
            ))

        # Store motor data
        if motor_analysis and "motors" in motor_analysis:
            for m in motor_analysis["motors"]:
                conn.execute("""
                    INSERT INTO flight_motors (flight_id, motor_num, avg_pct, saturation_pct)
                    VALUES (?, ?, ?, ?)
                """, (flight_id, m["motor"], m["avg_pct"], m["saturation_pct"]))

        # Store config snapshot (meaningful params only)
        config_params = [
            "gyro_lpf_hz", "dterm_lpf_hz", "dyn_notch_min_hz", "dyn_notch_q",
            "rpm_filter_enabled", "motor_protocol", "looptime",
            "roll_p", "roll_i", "roll_d", "roll_ff",
            "pitch_p", "pitch_i", "pitch_d", "pitch_ff",
            "yaw_p", "yaw_i", "yaw_d", "yaw_ff",
            "craft_name", "firmware_version", "board",
        ]
        for param in config_params:
            val = config.get(param)
            if val is not None:
                conn.execute("""
                    INSERT INTO flight_config (flight_id, param, value)
                    VALUES (?, ?, ?)
                """, (flight_id, param, str(val)))

        # Store diff config if available
        if diff_raw:
            diff_config = parse_diff_output(diff_raw)
            for param, val in diff_config.items():
                # Avoid duplicates with blackbox config
                if param not in config_params:
                    conn.execute("""
                        INSERT INTO flight_config (flight_id, param, value)
                        VALUES (?, ?, ?)
                    """, (flight_id, param, str(val)))

        # Store actions
        for a in plan.get("actions", []):
            conn.execute("""
                INSERT INTO flight_actions (flight_id, priority, urgency,
                    category, action, reason, deferred)
                VALUES (?, ?, ?, ?, ?, ?, ?)
            """, (
                flight_id,
                a.get("priority"),
                a.get("urgency"),
                a.get("category"),
                a.get("action"),
                a.get("reason"),
                1 if a.get("deferred") else 0,
            ))

        conn.commit()
        return flight_id, True  # (flight_id, is_new=True)

    def get_craft_history(self, craft, limit=20):
        """Get flight history for a specific craft, newest first.

        Returns list of dicts with scores and key metrics.
        """
        conn = self._connect()
        rows = conn.execute("""
            SELECT f.*,
                   GROUP_CONCAT(DISTINCT fa.axis || ':' || COALESCE(fa.hover_severity, 'none'))
                       AS osc_summary
            FROM flights f
            LEFT JOIN flight_axes fa ON fa.flight_id = f.id
            WHERE f.craft = ?
            GROUP BY f.id
            ORDER BY f.timestamp DESC
            LIMIT ?
        """, (craft, limit)).fetchall()

        history = []
        for row in rows:
            flight = dict(row)
            # Get axis details
            axes = conn.execute("""
                SELECT * FROM flight_axes WHERE flight_id = ?
            """, (row["id"],)).fetchall()
            flight["axes"] = [dict(a) for a in axes]

            # Get motor data
            motors = conn.execute("""
                SELECT * FROM flight_motors WHERE flight_id = ?
            """, (row["id"],)).fetchall()
            flight["motors"] = [dict(m) for m in motors]

            history.append(flight)

        return history

    def get_progression(self, craft, limit=10):
        """Get a progression summary for a craft.

        Returns a dict with:
            flights: list of simplified flight records (oldest first)
            trend: "improving", "stable", "degrading", or "insufficient"
            changes: list of notable changes between flights

        Ground-only flights (armed but never flew) are excluded from
        progression since they have no meaningful scores.
        """
        history = self.get_craft_history(craft, limit)
        if not history:
            return {"flights": [], "trend": "insufficient", "changes": []}

        # Reverse to chronological order, filter out ground-only flights
        flights = [f for f in reversed(history)
                   if f.get("verdict") != "GROUND_ONLY"
                   and f.get("overall_score") is not None]

        if not flights:
            return {"flights": [], "trend": "insufficient", "changes": []}

        simplified = []
        for f in flights:
            simplified.append({
                "id": f["id"],
                "timestamp": f["timestamp"],
                "score": f["overall_score"],
                "noise": f["noise_score"],
                "pid": f["pid_score"],
                "motor": f["motor_score"],
                "osc": f["osc_score"],
                "verdict": f["verdict"],
                "duration": f["duration_s"],
                "axes": f["axes"],
            })

        # Determine trend
        changes = []
        if len(simplified) >= 2:
            prev = simplified[-2]
            curr = simplified[-1]
            delta = (curr["score"] or 0) - (prev["score"] or 0)

            if delta > 10:
                trend = "improving"
            elif delta < -10:
                trend = "degrading"
            else:
                trend = "stable"

            # Note specific changes
            if prev["score"] and curr["score"]:
                changes.append(
                    f"Score: {prev['score']:.0f} → {curr['score']:.0f} "
                    f"({'↑' if delta > 0 else '↓'}{abs(delta):.0f})")

            # Compare axes
            for i, axis in enumerate(["Roll", "Pitch", "Yaw"]):
                prev_ax = next((a for a in prev["axes"] if a["axis"] == axis), None)
                curr_ax = next((a for a in curr["axes"] if a["axis"] == axis), None)
                if prev_ax and curr_ax:
                    if prev_ax["hover_rms"] and curr_ax["hover_rms"]:
                        rms_delta = curr_ax["hover_rms"] - prev_ax["hover_rms"]
                        if abs(rms_delta) > 2:
                            changes.append(
                                f"{axis} hover: {prev_ax['hover_rms']:.1f} → "
                                f"{curr_ax['hover_rms']:.1f}°/s "
                                f"({'↑worse' if rms_delta > 0 else '↓better'})")
                    if prev_ax["p_value"] and curr_ax["p_value"]:
                        if prev_ax["p_value"] != curr_ax["p_value"]:
                            changes.append(
                                f"{axis} P: {prev_ax['p_value']} → {curr_ax['p_value']}")
        else:
            trend = "insufficient"

        return {"flights": simplified, "trend": trend, "changes": changes}

    def get_flight_count(self, craft=None):
        """Get total number of stored flights, optionally filtered by craft."""
        conn = self._connect()
        if craft:
            row = conn.execute(
                "SELECT COUNT(*) FROM flights WHERE craft = ?", (craft,)).fetchone()
        else:
            row = conn.execute("SELECT COUNT(*) FROM flights").fetchone()
        return row[0]


def parse_diff_output(diff_text):
    """Parse INAV CLI 'diff all' output into key-value dict.

    Handles lines like:
        set gyro_main_lpf_hz = 40
        set mc_p_roll = 32
        # master
        # profile
    """
    config = {}
    if not diff_text:
        return config
    for line in diff_text.splitlines():
        line = line.strip()
        if line.startswith("set ") and " = " in line:
            # "set param_name = value"
            parts = line[4:].split(" = ", 1)
            if len(parts) == 2:
                param = parts[0].strip()
                value = parts[1].strip()
                config[param] = value
    return config
