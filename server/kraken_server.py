#!/usr/bin/env python3
"""
kraken_server.py — KrakenSDR Triangulator Dev Server
=====================================================
Single mobile KrakenSDR model with playback controls.

Playback API:
  GET  /api/bearings           — current observation history
  GET  /api/playback           — playback state (index, total, speed, paused)
  POST /api/playback           — send playback command (JSON body)
    { "action": "play" | "pause" | "forward" | "rewind" | "reset" | "seek" | "set_speed",
      "value": <int or float>  }   # value used by seek (waypoint index) and set_speed

Usage:
    python server/kraken_server.py
    KRAKEN_API_URL=http://<pi-ip>:8080 python server/kraken_server.py
"""

import json
import math
import os
import sys
import time
import socket
import threading
import logging
from pathlib import Path
from datetime import datetime, timezone

try:
    from flask import Flask, jsonify, request, send_from_directory
    from flask_cors import CORS
except ImportError:
    print("ERROR: Missing dependencies. Run: pip install -r requirements.txt")
    sys.exit(1)

try:
    import requests as req_lib
    REQUESTS_AVAILABLE = True
except ImportError:
    REQUESTS_AVAILABLE = False

try:
    from pymavlink import mavutil
    PYMAVLINK_AVAILABLE = True
except ImportError:
    PYMAVLINK_AVAILABLE = False

# ── Configuration ──────────────────────────────────────────────────────────────
PORT            = int(os.environ.get("PORT", 5050))
KRAKEN_API_URL  = os.environ.get("KRAKEN_API_URL", "")
BASE_ADVANCE_S  = float(os.environ.get("ADVANCE_EVERY_S", "0.5"))

BASE_DIR        = Path(__file__).resolve().parent.parent
MOCK_FILE_NAME  = os.environ.get("BEARINGS_FILE", "bearings_20260313_154333.json")
MOCK_DATA_PATH  = BASE_DIR / "data" / MOCK_FILE_NAME
APP_DIR         = BASE_DIR / "app"
UPLOAD_DIR      = BASE_DIR / "data" / "uploads"
UPLOAD_DIR.mkdir(parents=True, exist_ok=True)

# Valid file extensions for replay data
VALID_REPLAY_EXTS = {".json", ".jsonl"}
MAX_LIVE_HISTORY = int(os.environ.get("MAX_LIVE_HISTORY", "1000"))
DEFAULT_FREQUENCY_HZ = 462637500

# ── App Setup ──────────────────────────────────────────────────────────────────
logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("kraken_server")

app = Flask(__name__, static_folder=str(APP_DIR))
CORS(app)

# ── Playback State (protected by lock) ────────────────────────────────────────
_lock           = threading.Lock()
_mock_data      = None
_waypoints      = []
_obs_index      = 1        # How many waypoints are visible (1-based)
_paused         = False
_speed          = 1.0      # Multiplier: 0.25x, 0.5x, 1x, 2x, 4x
_last_advance_t = 0.0
# Observation receive timestamps (system clock, not mock data)
_obs_timestamps = {}       # { obs_id: ISO timestamp string }

# ── Live UDP State ────────────────────────────────────────────────────────────
_live_mode      = False
UDP_PORT        = int(os.environ.get("UDP_PORT", 5051))
_live_history   = []
_live_current   = None

# ── Persistent MAVLink Upstream Connection ────────────────────────────────────
# Binds to port 14551 where the GCS Laptop MAVProxy router sends telemetry.
# This establishes a bidirectional link: we receive MAVLink data from MAVProxy,
# and can send packets back (e.g. DEBUG_VECT) which MAVProxy forwards to
# COM port (RFD-900x) → Pi 5 → gcs_translator.py.
MAVLINK_UPSTREAM_PORT = int(os.environ.get("MAVLINK_UPSTREAM_PORT", 14551))
_mav_upstream = None        # Set at startup
_mav_upstream_ready = False # True once we've received at least one packet


def _finite_number(value):
    """Return a finite float, or None for missing/invalid input."""
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _clamp01(value):
    value = _finite_number(value)
    if value is None:
        return None
    return max(0.0, min(1.0, value))


def _as_bool(value, default=True):
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        lowered = value.strip().lower()
        if lowered in {"true", "1", "yes", "y", "on"}:
            return True
        if lowered in {"false", "0", "no", "n", "off"}:
            return False
    return bool(value)


def _looks_like_raw_kraken_record(record):
    """Raw Kraken logger rows do not contain the UAV telemetry sequence."""
    return (
        "telemetry_seq" not in record
        and "lat" not in record
        and (
            "source_timestamp_ms" in record
            or "kraken_counter" in record
            or "vfo_index" in record
        )
    )


def _normalize_observation(record, index=0, source_hint="fusion"):
    """Convert supported input records into the browser's canonical shape.

    The new logger calls its raw signal-quality value ``doa_papr_db``. Older
    fusion logs called the same kind of raw value ``confidence_0_1`` even
    though it is not guaranteed to be in the 0..1 range. Native waypoint JSON
    is the only format where ``confidence`` is treated as a normalized weight.
    """
    if not isinstance(record, dict) or _looks_like_raw_kraken_record(record):
        return None

    lat = record.get("lat")
    if lat is None:
        lat = record.get("lat_deg")
    lon = record.get("lon")
    if lon is None:
        lon = record.get("lon_deg")
    bearing = record.get("bearing_deg")
    if bearing is None:
        bearing = record.get("doa_deg")

    lat = _finite_number(lat)
    lon = _finite_number(lon)
    bearing = _finite_number(bearing)
    if lat is None or lon is None or bearing is None:
        return None
    if not -90.0 <= lat <= 90.0 or not -180.0 <= lon <= 180.0:
        return None

    run_id = record.get("run_id")
    kraken_seq = record.get("kraken_seq")
    telemetry_seq = record.get("telemetry_seq")
    record_id = record.get("id")
    if not record_id:
        identity = [str(value) for value in (run_id, kraken_seq, telemetry_seq) if value is not None]
        record_id = ":".join(identity) if identity else f"replay_{index:05d}"

    if record.get("doa_papr_db") is not None or record.get("spectrum_parse_valid") is not None:
        source_schema = "fusion_v2"
    elif "confidence_0_1" in record and (kraken_seq is not None or source_hint != "waypoint"):
        source_schema = "fusion_legacy"
    elif source_hint == "waypoint" or "confidence" in record:
        source_schema = "waypoint"
    else:
        source_schema = source_hint

    papr = _finite_number(record.get("doa_papr_db"))
    legacy_metric = _finite_number(record.get("confidence_0_1"))
    quality_score = _clamp01(record.get("quality_score"))
    quality_kind = record.get("quality_kind")

    if papr is not None:
        quality_kind = quality_kind or "papr_db"
    elif legacy_metric is not None and source_schema != "waypoint":
        # The upstream fusion logger intentionally accepts this legacy field
        # as an alias for the raw PAPR metric.
        papr = legacy_metric
        quality_kind = quality_kind or "legacy_papr_db"
    elif quality_score is None:
        legacy_confidence = _clamp01(record.get("confidence"))
        if legacy_confidence is not None:
            quality_score = legacy_confidence
            quality_kind = quality_kind or "legacy_confidence"

    if quality_score is None:
        # Peakiness is already a bounded 0..1 concentration measure. It is a
        # solver weight only, never a replacement for the raw PAPR metric.
        quality_score = _clamp01(record.get("spectrum_peakiness_0_1"))

    normalized = {
        "id": str(record_id),
        "label": record.get("label") or f"Replay #{index} | seq={kraken_seq if kraken_seq is not None else index}",
        "lat": lat,
        "lon": lon,
        "bearing_deg": bearing % 360.0,
        "source_schema": source_schema,
        "quality_kind": quality_kind or "unknown",
        "usable_for_triangulation": _as_bool(record.get("usable_for_triangulation"), True),
        "received_at": record.get("received_at") or _now_iso(),
    }

    if quality_score is not None:
        normalized["quality_score"] = quality_score
    if papr is not None:
        normalized["doa_papr_db"] = papr
    if legacy_metric is not None:
        normalized["legacy_confidence_0_1"] = legacy_metric
    if "confidence" in record and source_schema == "waypoint":
        normalized["confidence"] = _clamp01(record.get("confidence"))

    numeric_fields = {
        "t_rx_ms", "source_timestamp_ms", "dt_ms", "frequency_hz", "vfo_index",
        "raw_field_count", "rssi_power_db", "kraken_latency_ms",
        "kraken_lat_deg", "kraken_lon_deg", "gps_heading_deg", "compass_heading_deg",
        "spectrum_peak_unit_deg", "spectrum_peak_compass_deg", "peak_width_deg",
        "peak_to_second_peak_db", "spectrum_entropy_0_1", "spectrum_peakiness_0_1",
        "temporal_sample_count", "temporal_stability_0_1", "temporal_jitter_deg",
        "altitude_rel_ft", "yaw_deg", "pitch_deg", "roll_deg", "ground_speed_ft_s",
        "vel_north_m_s", "vel_east_m_s", "vel_down_m_s",
    }
    copied_fields = {
        "run_id", "kraken_seq", "telemetry_seq", "t_rx_ms", "source_timestamp_ms", "dt_ms",
        "frequency_hz", "vfo_index", "raw_field_count", "doa_angle_convention",
        "spectrum_angle_convention",
        "rssi_power_db", "antenna_array_arrangement", "kraken_latency_ms",
        "kraken_station_id", "kraken_lat_deg", "kraken_lon_deg", "gps_heading_deg",
        "compass_heading_deg", "heading_sensor", "spectrum_peak_unit_deg",
        "spectrum_peak_compass_deg", "peak_width_deg", "peak_to_second_peak_db",
        "spectrum_entropy_0_1", "spectrum_peakiness_0_1", "spectrum_parse_valid",
        "temporal_sample_count", "temporal_stability_0_1", "temporal_jitter_deg",
        "altitude_rel_ft", "yaw_deg", "pitch_deg", "roll_deg", "ground_speed_ft_s",
        "vel_north_m_s", "vel_east_m_s", "vel_down_m_s", "heading_used_deg",
    }
    for field in copied_fields:
        if field not in record or record[field] is None:
            continue
        normalized[field] = _finite_number(record[field]) if field in numeric_fields else record[field]

    if "heading_used_deg" not in normalized:
        heading = normalized.get("yaw_deg")
        if heading is None:
            heading = normalized.get("compass_heading_deg")
        if heading is None:
            heading = normalized.get("gps_heading_deg")
        if heading is not None:
            normalized["heading_used_deg"] = heading

    return normalized

def _load_mock():
    _load_mock_from_path(str(MOCK_DATA_PATH))

def _now_iso():
    return datetime.now(timezone.utc).isoformat(timespec='seconds')

def _advance_waypoint():
    """Advance one step if unpaused and enough time has elapsed."""
    global _obs_index, _last_advance_t
    if _paused or not _waypoints:
        return
    effective_interval = BASE_ADVANCE_S / max(_speed, 0.1)
    now = time.time()
    if now - _last_advance_t >= effective_interval:
        if _obs_index < len(_waypoints):
            _obs_index += 1
            wp = _waypoints[_obs_index - 1]
            _obs_timestamps.setdefault(wp['id'], _now_iso())
            logger.info(f"Advanced to waypoint {_obs_index}/{len(_waypoints)}")
        else:
            _obs_index = 1
            _obs_timestamps.clear()
            _obs_timestamps[_waypoints[0]['id']] = _now_iso()
            logger.info("Sequence complete — looping back to start.")
        _last_advance_t = now

def _build_response():
    if _live_mode:
        live_frequency = (_live_current or {}).get("frequency_hz") if _live_current else None
        if live_frequency is None and _mock_data:
            live_frequency = _mock_data.get("frequency_hz")
        return {
            "source":              "udp_stream",
            "frequency_hz":        live_frequency or DEFAULT_FREQUENCY_HZ,
            "mode":                "live_telemetry",
            "doa_method":          _mock_data.get("doa_method", "MUSIC") if _mock_data else "MUSIC",
            "source_schema":       (_live_current or {}).get("source_schema", "unknown") if _live_current else "unknown",
            "current_observation": _live_current,
            "observation_history": _live_history,
            "expected_target":     _mock_data.get("expected_target") if _mock_data else None,
            "playback":            None
        }

    if not _waypoints:
        return {
            "source":              "replay",
            "frequency_hz":        DEFAULT_FREQUENCY_HZ,
            "mode":                "replay",
            "doa_method":          "MUSIC",
            "source_schema":       "unknown",
            "current_observation": None,
            "observation_history": [],
            "expected_target":     None,
            "playback": {"index": 0, "total": 0, "speed": _speed, "paused": _paused},
        }

    visible = _waypoints[:_obs_index]
    # Attach system-clock timestamps to each visible observation
    enriched = []
    for obs in visible:
        enriched.append({
            **obs,
            "received_at": _obs_timestamps.get(obs['id'], _now_iso()),
        })
    current = enriched[-1] if enriched else None
    return {
        "source":              "replay",
        "frequency_hz":        _mock_data.get("frequency_hz", DEFAULT_FREQUENCY_HZ) if _mock_data else DEFAULT_FREQUENCY_HZ,
        "mode":                _mock_data.get("mode", "single_mobile_kraken") if _mock_data else "replay",
        "doa_method":          _mock_data.get("doa_method", "MUSIC") if _mock_data else "MUSIC",
        "source_schema":       _mock_data.get("source_schema", "unknown") if _mock_data else "unknown",
        "current_observation": current,
        "observation_history": enriched,
        "expected_target":     _mock_data.get("expected_target") if _mock_data else None,
        "playback": {
            "index":   _obs_index,
            "total":   len(_waypoints),
            "speed":   _speed,
            "paused":  _paused,
        },
    }

# ── Routes ─────────────────────────────────────────────────────────────────────

@app.route("/")
def index():
    return send_from_directory(str(APP_DIR), "index.html")

@app.route("/<path:filename>")
def static_files(filename):
    return send_from_directory(str(APP_DIR), filename)

@app.route("/api/bearings", methods=["GET"])
def get_bearings():
    if KRAKEN_API_URL and REQUESTS_AVAILABLE:
        try:
            resp = req_lib.get(f"{KRAKEN_API_URL}/DOA_res", timeout=3)
            resp.raise_for_status()
            return jsonify({"source": "live", "raw": resp.json()})
        except Exception as e:
            logger.warning(f"Live fetch failed ({e}), falling back to mock.")

    with _lock:
        _advance_waypoint()
        return jsonify(_build_response())

@app.route("/api/clear_history", methods=["POST"])
def clear_history():
    with _lock:
        _live_history.clear()
        global _live_current
        _live_current = None
        logger.info("Live history cleared via API")
    return jsonify({"status": "cleared"})

@app.route("/api/playback", methods=["GET"])
def get_playback():
    with _lock:
        return jsonify({
            "index":  _obs_index,
            "total":  len(_waypoints),
            "speed":  _speed,
            "paused": _paused,
            "labels": [wp['label'] for wp in _waypoints],
        })

@app.route("/api/playback", methods=["POST"])
def post_playback():
    global _obs_index, _paused, _speed, _last_advance_t
    body = request.get_json(force=True) or {}
    action = body.get("action", "")

    with _lock:
        if action == "play":
            _paused = False
            _last_advance_t = time.time()
            logger.info("Playback: PLAY")
        elif action == "pause":
            _paused = True
            logger.info("Playback: PAUSE")
        elif action == "forward":
            if _obs_index < len(_waypoints):
                _obs_index += 1
                wp = _waypoints[_obs_index - 1]
                _obs_timestamps.setdefault(wp['id'], _now_iso())
            _last_advance_t = time.time()
            logger.info(f"Playback: FORWARD -> {_obs_index}")
        elif action == "rewind":
            if _obs_index > 1:
                _obs_index -= 1
            _last_advance_t = time.time()
            logger.info(f"Playback: REWIND -> {_obs_index}")
        elif action == "reset":
            _obs_index = 1
            _obs_timestamps.clear()
            _obs_timestamps[_waypoints[0]['id']] = _now_iso()
            _last_advance_t = time.time()
            logger.info("Playback: RESET")
        elif action == "seek":
            val = int(body.get("value", 1))
            _obs_index = max(1, min(val, len(_waypoints)))
            for i in range(_obs_index):
                wp = _waypoints[i]
                _obs_timestamps.setdefault(wp['id'], _now_iso())
            _last_advance_t = time.time()
            logger.info(f"Playback: SEEK -> {_obs_index}")
        elif action == "set_speed":
            val = float(body.get("value", 1.0))
            _speed = max(0.25, min(val, 200.0))
            logger.info(f"Playback: SET SPEED {_speed}x")

        return jsonify(_build_response())

@app.route("/api/health", methods=["GET"])
def health():
    mode = "live" if _live_mode else "replay"
    with _lock:
        return jsonify({
            "status":  "ok",
            "mode":    mode,
            "port":    PORT,
            "index":   _obs_index,
            "total":   len(_waypoints),
        })

# ── Replay Mode API ────────────────────────────────────────────────────────────

@app.route("/api/replay/files", methods=["GET"])
def list_replay_files():
    """List all valid replay files from data/, data/uploads/, and GCS fusion logs."""
    files = []
    # Scan data/ for bearings JSON files
    data_dir = BASE_DIR / "data"
    for f in sorted(data_dir.glob("bearings_*.json")):
        files.append({"name": f.name, "path": str(f.relative_to(BASE_DIR)), "size_kb": round(f.stat().st_size / 1024, 1), "type": "bearings_json"})
    for f in sorted(data_dir.glob("mock_bearings.json")):
        files.append({"name": f.name, "path": str(f.relative_to(BASE_DIR)), "size_kb": round(f.stat().st_size / 1024, 1), "type": "bearings_json"})
    # Scan uploads/
    for f in sorted(UPLOAD_DIR.iterdir()):
        if f.suffix in VALID_REPLAY_EXTS:
            files.append({"name": f.name, "path": str(f.relative_to(BASE_DIR)), "size_kb": round(f.stat().st_size / 1024, 1), "type": "upload"})
    # Scan GCS fusion logs (common location)
    gcs_log_dir = BASE_DIR.parent / "ngcp-uav-software" / "src" / "comms" / "logs" / "fusion_gcs"
    if gcs_log_dir.exists():
        for f in sorted(gcs_log_dir.glob("fusion_gcs_*.jsonl")):
            if f.stat().st_size > 0:  # skip empty logs
                files.append({"name": f.name, "path": str(f), "size_kb": round(f.stat().st_size / 1024, 1), "type": "fusion_gcs_log"})
    # Scan Pi-side fusion logs
    pi_log_dir = BASE_DIR.parent / "ngcp-uav-software" / "logs" / "fusion"
    if pi_log_dir.exists():
        for f in sorted(pi_log_dir.glob("fusion_*.jsonl")):
            if f.stat().st_size > 0:
                files.append({"name": f.name, "path": str(f), "size_kb": round(f.stat().st_size / 1024, 1), "type": "pi_fusion_log"})
    return jsonify({"files": files})

@app.route("/api/replay/upload", methods=["POST"])
def upload_replay_file():
    """Upload a bearings JSON or fusion JSONL file for replay."""
    if 'file' not in request.files:
        return jsonify({"error": "No file in request"}), 400
    f = request.files['file']
    if not f.filename:
        return jsonify({"error": "Empty filename"}), 400
    ext = Path(f.filename).suffix.lower()
    if ext not in VALID_REPLAY_EXTS:
        return jsonify({"error": f"Invalid file type '{ext}'. Accepted: {', '.join(VALID_REPLAY_EXTS)}"}), 400
    dest = UPLOAD_DIR / f.filename
    f.save(str(dest))
    logger.info(f"Uploaded replay file: {dest}")
    return jsonify({"status": "ok", "name": f.filename, "size_kb": round(dest.stat().st_size / 1024, 1)})

def _load_replay_jsonl(path):
    """Load a supported JSONL log and normalize records for the web app."""
    global _mock_data, _waypoints, _obs_index, _last_advance_t, _obs_timestamps
    waypoints = []
    schema_counts = {}
    records_read = 0
    records_rejected = 0
    with open(path, encoding='utf-8') as fh:
        for idx, line in enumerate(fh):
            line = line.strip()
            if not line:
                continue
            records_read += 1
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                records_rejected += 1
                continue
            wp = _normalize_observation(rec, idx, source_hint="fusion")
            if wp is None:
                records_rejected += 1
                continue
            schema_counts[wp["source_schema"]] = schema_counts.get(wp["source_schema"], 0) + 1
            waypoints.append(wp)

    if not waypoints:
        raise ValueError(
            "No usable fused observations found. Raw Kraken JSONL must be fused "
            "with UAV telemetry before replay."
        )

    schemas = sorted(schema_counts)
    source_schema = schemas[0] if len(schemas) == 1 else "mixed"
    frequency = next(
        (wp.get("frequency_hz") for wp in waypoints if wp.get("frequency_hz")),
        DEFAULT_FREQUENCY_HZ,
    )
    stats = {
        "records_read": records_read,
        "records_loaded": len(waypoints),
        "records_rejected": records_rejected,
        "usable_records": sum(1 for wp in waypoints if wp.get("usable_for_triangulation", True)),
        "schemas": schema_counts,
        "source_schema": source_schema,
    }
    _mock_data = {
        "frequency_hz": frequency,
        "mode": "replay",
        "doa_method": "MUSIC",
        "source_schema": source_schema,
        "replay_stats": stats,
        "waypoint_sequence": waypoints,
    }
    _waypoints = waypoints
    _obs_index = 1
    _last_advance_t = time.time()
    _obs_timestamps = {}
    if _waypoints:
        _obs_timestamps[_waypoints[0]['id']] = _now_iso()
    logger.info(
        "Loaded %s observations from JSONL replay: %s (schema=%s, rejected=%s)",
        len(_waypoints), Path(path).name, source_schema, records_rejected,
    )
    return stats

@app.route("/api/replay/load", methods=["POST"])
def load_replay():
    """Switch to replay mode and load a specific file."""
    global _live_mode, _mock_data, _waypoints, _obs_index, _last_advance_t, _obs_timestamps, _paused, _speed
    body = request.get_json(force=True) or {}
    file_path = body.get("path", "")
    if not file_path:
        return jsonify({"error": "Missing 'path'"}), 400

    # Resolve path: could be relative to BASE_DIR or absolute
    p = Path(file_path)
    if not p.is_absolute():
        p = BASE_DIR / p
    if not p.exists():
        return jsonify({"error": f"File not found: {p}"}), 404
    if p.suffix.lower() not in VALID_REPLAY_EXTS:
        return jsonify({"error": f"Invalid file type"}), 400

    replay_stats = None
    with _lock:
        _live_mode = False
        _paused = True
        _speed = 1.0
        if p.suffix.lower() == '.jsonl':
            try:
                replay_stats = _load_replay_jsonl(str(p))
            except ValueError as exc:
                _live_mode = True
                _paused = False
                return jsonify({"error": str(exc)}), 400
        else:
            _load_mock_from_path(str(p))
    logger.info(f"REPLAY MODE — Loaded {len(_waypoints)} waypoints from {p.name}")
    return jsonify({
        "status": "ok",
        "total": len(_waypoints),
        "filename": p.name,
        "source_schema": _mock_data.get("source_schema", "waypoint") if _mock_data else "unknown",
        "stats": replay_stats or {},
    })

def _load_mock_from_path(path):
    """Load a bearings JSON file (original format)."""
    global _mock_data, _waypoints, _obs_index, _last_advance_t, _obs_timestamps
    with open(path, encoding='utf-8') as fh:
        _mock_data = json.load(fh)
    normalized = []
    for idx, record in enumerate(_mock_data.get("waypoint_sequence", [])):
        waypoint = _normalize_observation(record, idx, source_hint="waypoint")
        if waypoint is not None:
            normalized.append(waypoint)
    _waypoints = normalized
    _mock_data["waypoint_sequence"] = _waypoints
    _mock_data["source_schema"] = "waypoint"
    _obs_index = 1
    _last_advance_t = time.time()
    _obs_timestamps = {}
    if _waypoints:
        _obs_timestamps[_waypoints[0]['id']] = _now_iso()
    logger.info(f"Loaded {len(_waypoints)} waypoints from JSON: {Path(path).name}")

@app.route("/api/replay/stop", methods=["POST"])
def stop_replay():
    """Switch back to live UDP mode."""
    global _live_mode, _mock_data, _waypoints, _obs_index, _paused
    with _lock:
        _live_mode = True
        _mock_data = None
        _waypoints = []
        _obs_index = 1
        _paused = False
    logger.info("Switched back to LIVE UDP mode.")
    return jsonify({"status": "ok", "mode": "live"})

@app.route("/api/transmit", methods=["POST"])
def transmit():
    """Bridge endpoint from UI to gcs_translator.py"""
    body = request.get_json(force=True) or {}
    lat = body.get("lat")
    lon = body.get("lon")
    spread_m = body.get("spread_m", 0)
    count = body.get("count", 1)
    
    if lat is None or lon is None:
        return jsonify({"error": "Missing lat/lon"}), 400
        
    payload = {
        "lat": lat,
        "lon": lon,
        "spread_m": spread_m,
        "count": count,
        "timestamp": time.time()
    }
    
    # Write to a known file location so gcs_translator.py can read it
    target_file = Path("/tmp/kraken_gcs_target.json") if os.name != 'nt' else Path(os.environ.get("TEMP", "C:/Temp")) / "kraken_gcs_target.json"
    
    try:
        # Create parent directory if needed
        target_file.parent.mkdir(parents=True, exist_ok=True)
        with open(target_file, "w") as f:
            json.dump(payload, f)
            
        # --- UPSTREAM MAVLINK TRANSMISSION ---
        # Send the target coordinates upstream through the persistent MAVLink
        # connection. This flows: kraken_server → MAVProxy (port 14551) →
        # COM13 (RFD-900x) → Pi 5 MAVProxy → gcs_translator.py
        if _mav_upstream and _mav_upstream_ready:
            try:
                _mav_upstream.mav.debug_vect_send(
                    b'KRAKEN_TGT', int(time.time() * 1e6), lat, lon, spread_m
                )
                logger.info(f"Target sent UPSTREAM via MAVLink: {lat}, {lon} (spread={spread_m:.1f}m)")
            except Exception as mav_e:
                logger.error(f"MAVLink upstream transmission failed: {mav_e}")
        elif not PYMAVLINK_AVAILABLE:
            logger.error("pymavlink is not installed. Run: pip install pymavlink")
        elif not _mav_upstream_ready:
            logger.warning("MAVLink upstream not ready — no packets received from MAVProxy yet. "
                           "Is the GCS Laptop MAVProxy Router running?")

        return jsonify({"status": "ok", "message": "Transmitted successfully"})
    except Exception as e:
        logger.error(f"Failed to export target: {e}")
        return jsonify({"error": str(e)}), 500

# ── Entry Point ────────────────────────────────────────────────────────────────

def _translate_fusion_record(payload):
    """Normalize fusion or native waypoint payloads received over UDP."""
    if not isinstance(payload, dict):
        return None
    if "kraken_seq" not in payload and not {"lat", "lon", "bearing_deg"}.issubset(payload):
        return payload
    return _normalize_observation(payload, payload.get("kraken_seq", 0), source_hint="udp_fusion")

def udp_listener_thread():
    global _live_mode, _live_history, _live_current
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", UDP_PORT))
    logger.info(f"UDP Telemetry Listener bound to 0.0.0.0:{UDP_PORT}")
    while True:
        try:
            data, addr = sock.recvfrom(65535)
            payload = json.loads(data.decode("utf-8"))
            
            # Translate MRA fusion records to Kraken schema
            payload = _translate_fusion_record(payload)
            if payload is None:
                continue
                
            # Ensure it has a system clock stamp
            if 'received_at' not in payload:
                payload['received_at'] = _now_iso()
            with _lock:
                _live_mode = True
                _live_history.append(payload)
                if len(_live_history) > MAX_LIVE_HISTORY:
                    del _live_history[:-MAX_LIVE_HISTORY]
                _live_current = payload
        except Exception as e:
            logger.error(f"UDP parse error: {e}")

def _mavlink_drain_thread():
    """Background thread: drain incoming MAVLink packets to keep the connection alive.
    Once the first packet arrives from MAVProxy, the return address is learned
    and _mav_upstream_ready is set to True."""
    global _mav_upstream_ready
    while True:
        try:
            if _mav_upstream is None:
                time.sleep(1)
                continue
            msg = _mav_upstream.recv_match(blocking=True, timeout=5)
            if msg and not _mav_upstream_ready:
                _mav_upstream_ready = True
                logger.info(f"MAVLink upstream link ESTABLISHED — bidirectional path to MAVProxy confirmed.")
        except Exception as e:
            logger.error(f"MAVLink drain error: {e}")
            time.sleep(1)

if __name__ == "__main__":
    _live_mode = True
    logger.info("LIVE UDP MODE FORCED. Mock data disabled.")
    
    # Spin up the background telemetry listener
    threading.Thread(target=udp_listener_thread, daemon=True).start()

    # Establish persistent MAVLink upstream connection for bidirectional
    # communication with the GCS Laptop MAVProxy Router (RFD-900x link).
    if PYMAVLINK_AVAILABLE:
        try:
            _mav_upstream = mavutil.mavlink_connection(f'udpin:0.0.0.0:{MAVLINK_UPSTREAM_PORT}')
            logger.info(f"MAVLink upstream bound to udpin:0.0.0.0:{MAVLINK_UPSTREAM_PORT} "
                        f"— waiting for MAVProxy packets...")
            threading.Thread(target=_mavlink_drain_thread, daemon=True).start()
        except Exception as e:
            logger.error(f"Failed to bind MAVLink upstream on port {MAVLINK_UPSTREAM_PORT}: {e}")
            _mav_upstream = None
    else:
        logger.warning("pymavlink not installed — upstream MAVLink transmission disabled.")

    if KRAKEN_API_URL:
        logger.info(f"LIVE KRAKEN MODE — Proxying KrakenSDR API at {KRAKEN_API_URL}")
    else:
        logger.info(f"LIVE UDP MODE — Ready for UDP stream on port {UDP_PORT}.")
    logger.info(f"Opening at http://localhost:{PORT}")
    app.run(host="0.0.0.0", port=PORT, debug=False)
