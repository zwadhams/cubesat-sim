"""Flight report dashboard: one recording -> one self-contained HTML file.

Reads a flight recorder .db and renders a static report — stat tiles, a
digital state strip (eclipse / contact / safe mode / shedding ...), an
event timeline with severity glyphs, and analog telemetry lanes with a
shared crosshair — into a single HTML file with zero external
dependencies (inline CSS/JS, data embedded as JSON). Open it in any
browser; light/dark follow the OS with a manual toggle.

Usage:
    python -m cubesat_sim.dashboard runs/phase5_hard_failure.db
    python -m cubesat_sim.dashboard runs/mc/*.db        # one report each

or programmatically: `render_flight("runs/foo.db")` -> Path to the HTML.

Chart rules of the house (see the dataviz method this follows): one unit
per lane — never a dual axis; color is assigned by entity in fixed slot
order (truth first, estimate second); status colors are reserved for
event severity and always ride with a glyph and a label; every chart has
a table-view twin.
"""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
from datetime import datetime, timezone
from pathlib import Path

from cubesat_sim.environment.groundstation import gmst_rad
from cubesat_sim.environment.orbit import R_EARTH, CircularOrbit
from cubesat_sim.environment.sun import sun_direction_eci
from cubesat_sim.faults import SAA_LAT_DEG, SAA_LON_DEG
from cubesat_sim.physics.spacecraft import DEFAULT_STATION, DEFAULT_TARGETS

# analog lanes: (title, unit, fixed y-domain or None, series)
# each series: (source, key, label, transform)
_K2C = ("kelvin", lambda v: v - 273.15)
_ID = (None, None)
ANALOG_LANES = [
    ("State of charge", "fraction", (0.0, 1.0), [
        ("physics", "soc_true", "true", _ID),
        ("eps", "soc_est", "EPS estimate", _ID),
    ]),
    ("Electrical power", "W", None, [
        ("physics", "p_gen_w", "generation", _ID),
        ("physics", "p_load_w", "load", _ID),
    ]),
    ("Battery voltage", "V", None, [
        ("physics", "battery_v_true", "true", _ID),
    ]),
    ("Temperatures", "°C", None, [
        ("physics", "t_batt_k", "battery", _K2C),
        ("physics", "t_struct_k", "structure", _K2C),
    ]),
    ("Body rate", "deg/s", None, [
        ("physics", "rate_dps", "true", _ID),
        ("adcs", "rate_dps", "ADCS estimate", _ID),
    ]),
    ("Sun facing", "cosine", (-1.0, 1.0), [
        ("physics", "sun_facing", "panel · sun", _ID),
    ]),
    ("Wheel momentum", "frac of max", (0.0, 1.0), [
        ("physics", "wheel_h_frac", "true", _ID),
        ("adcs", "wheel_frac", "ADCS estimate", _ID),
    ]),
    ("Data", "MB", None, [
        ("comms", "queue_mb", "onboard queue", _ID),
        ("ground", "archive_mb", "ground archive", _ID),
        ("comms", "dropped_mb", "dropped", _ID),
    ]),
    ("Battery capacity", "Wh", None, [
        ("physics", "batt_capacity_wh", "capacity", _ID),
    ]),
    ("Array health", "illumination", None, [
        ("physics", "array_illum", "illumination", _ID),
    ]),
]

DIGITAL_TRACKS = [
    ("physics", "eclipse", "eclipse"),
    ("physics", "gs_contact", "ground contact"),
    ("physics", "target_visible", "target visible"),
    ("payload", "imaging", "imaging"),
    ("obc", "safe_mode", "safe mode"),
    ("eps", "shedding", "load shed"),
    ("physics", "charge_blocked", "charge inhibit"),
    ("thermal", "heater_request", "heater request"),
    ("adcs", "mode_sun_point", "sun-point mode"),
]

# events that the bands/tracks already tell better than markers would
SKIP_EVENT_KINDS = {
    "eclipse_enter", "eclipse_exit", "contact_aos", "contact_los",
    "charge_inhibit_on", "charge_inhibit_off",
}

EVENT_SEVERITY = {
    "inject": "critical",
    "inject_seu": "warning",
    "brownout": "critical",
    "fdir_giveup": "critical",
    "storage_full": "critical",
    "fdir_adcs_power_cycle": "warning",
    "gyro_anomaly": "warning",
    "seu_corruption": "warning",
    "load_shed": "warning",
    "frame_reject": "warning",
    "fdir_adcs_repower": "good",
    "latchup_cleared": "good",
    "load_restore": "good",
}

MAX_BUCKETS = 600


def _severity(kind: str, detail: dict) -> str:
    if kind == "mode_change":
        return "warning" if detail.get("to") == "SAFE" else "good"
    return EVENT_SEVERITY.get(kind, "neutral")


def _round(v: float) -> float:
    return float(f"{v:.6g}")


def _series(db: sqlite3.Connection, source: str, key: str):
    rows = db.execute(
        "SELECT time, value FROM telemetry WHERE source=? AND key=? ORDER BY tick",
        (source, key)).fetchall()
    return [(t, v) for t, v in rows if v is not None]


def _downsample(points, max_buckets=MAX_BUCKETS):
    """Min/max bucketing: keeps every spike a plain stride would erase."""
    if len(points) <= 2 * max_buckets:
        return [[_round(t), _round(v)] for t, v in points]
    per = math.ceil(len(points) / max_buckets)
    out = []
    for i in range(0, len(points), per):
        bucket = points[i:i + per]
        lo = min(bucket, key=lambda p: p[1])
        hi = max(bucket, key=lambda p: p[1])
        pair = sorted({lo, hi}, key=lambda p: p[0])
        out.extend([_round(t), _round(v)] for t, v in pair)
    return out


def _intervals(points, t_end: float):
    """Collapse a sampled 0/1 channel into [start, end] high intervals."""
    out, start = [], None
    for t, v in points:
        if v >= 0.5 and start is None:
            start = t
        elif v < 0.5 and start is not None:
            out.append([_round(start), _round(t)])
            start = None
    if start is not None:
        out.append([_round(start), _round(t_end)])
    return out


def _fmt_detail(detail: dict) -> str:
    parts = []
    for k, v in detail.items():
        if isinstance(v, float):
            parts.append(f"{k}={v:.4g}")
        else:
            parts.append(f"{k}={v}")
    return ", ".join(parts)


def load_flight(db_path: str | Path) -> dict:
    db_path = Path(db_path)
    db = sqlite3.connect(str(db_path))
    try:
        meta = dict(db.execute("SELECT key, value FROM meta").fetchall())
        t_end = db.execute("SELECT MAX(time) FROM telemetry").fetchone()[0] or 0.0
        period = CircularOrbit().period_s

        lanes = []
        for title, unit, domain, series_spec in ANALOG_LANES:
            series = []
            for source, key, label, (_, transform) in series_spec:
                pts = _series(db, source, key)
                if not pts:
                    continue
                if transform is not None:
                    pts = [(t, transform(v)) for t, v in pts]
                series.append({"label": label,
                               "points": _downsample(pts)})
            if series:
                lanes.append({"title": title, "unit": unit,
                              "domain": list(domain) if domain else None,
                              "series": series})

        tracks = []
        for source, key, label in DIGITAL_TRACKS:
            pts = _series(db, source, key)
            if pts:
                iv = _intervals(pts, t_end)
                if iv:
                    tracks.append({"label": label, "intervals": iv})

        events = []
        for tick, t, source, kind, detail_json in db.execute(
                "SELECT tick, time, source, kind, detail FROM events ORDER BY tick"):
            if kind in SKIP_EVENT_KINDS:
                continue
            detail = json.loads(detail_json) if detail_json else {}
            events.append({"t": _round(t), "source": source, "kind": kind,
                           "severity": _severity(kind, detail),
                           "detail": _fmt_detail(detail)})

        soc = _series(db, "physics", "soc_true")
        cap = _series(db, "physics", "batt_capacity_wh")
        illum = _series(db, "physics", "array_illum")
        archive = _series(db, "ground", "archive_mb")
        dropped = _series(db, "comms", "dropped_mb")
        n_brownouts = sum(1 for e in events if e["kind"] == "brownout")
        n_safe = sum(1 for e in events
                     if e["kind"] == "mode_change" and "to=SAFE" in e["detail"])
        n_cycles = sum(1 for e in events if e["kind"] == "fdir_adcs_power_cycle")
        gave_up = any(e["kind"] == "fdir_giveup" for e in events)
        n_faults = sum(1 for e in events if e["kind"] == "inject")
        n_seus = sum(1 for e in events if e["kind"] == "inject_seu")

        tiles = [
            {"label": "Duration", "value": f"{t_end / period:.1f} orbits",
             "note": f"{t_end / 3600.0:.1f} h at dt={meta.get('dt', '?')} s"},
            {"label": "Min SoC (true)",
             "value": f"{min(v for _, v in soc):.2f}" if soc else "—",
             "note": f"ends {soc[-1][1]:.2f}" if soc else ""},
            {"label": "Brownouts", "value": str(n_brownouts),
             "note": "battery hit empty" if n_brownouts else "never browned out"},
            {"label": "Safe-mode entries", "value": str(n_safe), "note": ""},
            {"label": "FDIR power cycles", "value": str(n_cycles),
             "note": "gave up" if gave_up else ""},
            {"label": "Faults injected", "value": str(n_faults),
             "note": f"+ {n_seus} SEUs" if n_seus else ""},
        ]
        if archive:
            tiles.append({"label": "Data archived",
                          "value": f"{archive[-1][1]:.0f} MB",
                          "note": (f"{dropped[-1][1]:.0f} MB dropped"
                                   if dropped and dropped[-1][1] > 0 else "")})
        if cap and illum:
            tiles.append({"label": "Degradation",
                          "value": f"{cap[-1][1]:.2f} Wh",
                          "note": f"array at {illum[-1][1]:.3f}"})

        epoch_iso = json.loads(meta["epoch"]) if "epoch" in meta else None

        return {
            "title": db_path.stem,
            "meta": {"seed": meta.get("seed", "?"), "dt": meta.get("dt", "?"),
                     "epoch": epoch_iso or "?",
                     "duration_s": _round(t_end), "period_s": _round(period)},
            "tiles": tiles,
            "tracks": tracks,
            "events": events,
            "lanes": lanes,
            "orbit3d": _orbit_geometry(epoch_iso),
        }
    finally:
        db.close()


def _orbit_geometry(epoch_iso: str | None) -> dict:
    """Everything the in-page orbit view needs, in ~300 bytes.

    A circular orbit is exactly `a * (e1 cos(nt) + e2 sin(nt))`, so two
    basis vectors and the mean motion reconstruct the whole path in JS.
    Assumes the default mission geometry (orbit, station, targets) — true
    of every flight build_sim produces today.
    """
    orbit = CircularOrbit()
    a = orbit.semi_major_axis_m
    e1 = orbit.position_eci(0.0) / a
    e2 = orbit.position_eci(orbit.period_s / 4.0) / a
    epoch = (datetime.fromisoformat(epoch_iso) if epoch_iso
             else datetime(2026, 1, 1, tzinfo=timezone.utc))
    sun = sun_direction_eci(epoch)
    sites = [{"name": DEFAULT_STATION.name, "lat": DEFAULT_STATION.lat_deg,
              "lon": DEFAULT_STATION.lon_deg, "kind": "station"}]
    sites += [{"name": s.name, "lat": s.lat_deg, "lon": s.lon_deg,
               "kind": "target"} for s in DEFAULT_TARGETS]
    return {
        "r_orbit_re": _round(a / R_EARTH),
        "n_rad_s": orbit.mean_motion_rad_s,
        "e1": [_round(v) for v in e1],
        "e2": [_round(v) for v in e2],
        "sun": [_round(v) for v in sun],
        "gmst0_rad": _round(gmst_rad(epoch)),
        "w_earth_rad_s": math.radians(360.98564736629) / 86400.0,
        "sites": sites,
        "saa": {"lat": list(SAA_LAT_DEG), "lon": list(SAA_LON_DEG)},
    }


def render_flight(db_path: str | Path, out_path: str | Path | None = None) -> Path:
    db_path = Path(db_path)
    out_path = Path(out_path) if out_path else db_path.with_suffix(".html")
    payload = load_flight(db_path)
    blob = json.dumps(payload, separators=(",", ":")).replace("</", "<\\/")
    html = (_TEMPLATE
            .replace("__TITLE__", payload["title"])
            .replace("__FLIGHT_JSON__", blob))
    out_path.write_text(html, encoding="utf-8")
    return out_path


_TEMPLATE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>__TITLE__ — flight report</title>
<style>
:root {
  color-scheme: light;
  --page: #f9f9f7; --surface: #fcfcfb;
  --ink: #0b0b0b; --ink-2: #52514e; --muted: #898781;
  --grid: #e1e0d9; --axis: #c3c2b7;
  --border: rgba(11,11,11,0.10);
  --s1: #2a78d6; --s2: #008300; --s3: #e87ba4; --s4: #eda100;
  --good: #0ca30c; --warning: #fab219; --serious: #ec835a; --critical: #d03b3b;
  --band: rgba(11,11,11,0.045);
}
@media (prefers-color-scheme: dark) {
  :root:where(:not([data-theme="light"])) {
    color-scheme: dark;
    --page: #0d0d0d; --surface: #1a1a19;
    --ink: #ffffff; --ink-2: #c3c2b7; --muted: #898781;
    --grid: #2c2c2a; --axis: #383835;
    --border: rgba(255,255,255,0.10);
    --s1: #3987e5; --s2: #008300; --s3: #d55181; --s4: #c98500;
    --band: rgba(255,255,255,0.055);
  }
}
:root[data-theme="dark"] {
  color-scheme: dark;
  --page: #0d0d0d; --surface: #1a1a19;
  --ink: #ffffff; --ink-2: #c3c2b7; --muted: #898781;
  --grid: #2c2c2a; --axis: #383835;
  --border: rgba(255,255,255,0.10);
  --s1: #3987e5; --s2: #008300; --s3: #d55181; --s4: #c98500;
  --band: rgba(255,255,255,0.055);
}
* { box-sizing: border-box; }
body {
  margin: 0; background: var(--page); color: var(--ink);
  font: 14px/1.45 system-ui, -apple-system, "Segoe UI", sans-serif;
}
.report { max-width: 1080px; margin: 0 auto; padding: 20px 16px 48px; }
header { display: flex; align-items: baseline; gap: 12px; flex-wrap: wrap;
         margin: 4px 2px 14px; }
header h1 { font-size: 19px; font-weight: 650; margin: 0; }
header .chips { color: var(--ink-2); font-size: 12.5px; }
header button {
  margin-left: auto; font: inherit; font-size: 12.5px; color: var(--ink-2);
  background: var(--surface); border: 1px solid var(--border);
  border-radius: 7px; padding: 3px 10px; cursor: pointer;
}
.tiles { display: grid; grid-template-columns: repeat(auto-fill, minmax(150px, 1fr));
         gap: 10px; margin-bottom: 14px; }
.tile { background: var(--surface); border: 1px solid var(--border);
        border-radius: 10px; padding: 10px 12px 9px; }
.tile .lb { font-size: 12px; color: var(--ink-2); }
.tile .vl { font-size: 23px; font-weight: 600; margin-top: 1px; }
.tile .nt { font-size: 11.5px; color: var(--muted); margin-top: 1px; min-height: 15px; }
.card { background: var(--surface); border: 1px solid var(--border);
        border-radius: 10px; padding: 12px 14px 10px; margin-bottom: 12px; }
.card h2 { font-size: 13px; font-weight: 650; margin: 0 0 8px; color: var(--ink); }
.lane { margin-bottom: 2px; }
.lane-head { display: flex; align-items: baseline; gap: 8px; margin: 6px 2px 0; }
.lane-head .t { font-size: 12.5px; font-weight: 650; }
.lane-head .u { font-size: 11.5px; color: var(--muted); }
.legend { margin-left: auto; display: flex; gap: 14px; font-size: 11.5px;
          color: var(--ink-2); }
.legend .key { display: inline-block; width: 14px; height: 0;
               border-top: 2.5px solid; border-radius: 2px;
               vertical-align: middle; margin-right: 5px; }
svg { display: block; }
svg text { font: 10.5px system-ui, -apple-system, "Segoe UI", sans-serif;
           fill: var(--muted); font-variant-numeric: tabular-nums; }
svg .rowlab { font-size: 11px; fill: var(--ink-2); font-variant-numeric: normal; }
#tooltip {
  position: fixed; display: none; pointer-events: none; z-index: 10;
  background: var(--surface); border: 1px solid var(--border);
  border-radius: 8px; padding: 7px 10px; font-size: 12px;
  box-shadow: 0 4px 14px rgba(0,0,0,0.13); max-width: 320px;
}
#tooltip .tt-t { color: var(--muted); font-size: 11px; margin-bottom: 3px; }
#tooltip .row { display: flex; align-items: baseline; gap: 7px; }
#tooltip .key { display: inline-block; width: 12px; height: 0;
                border-top: 2.5px solid; border-radius: 2px; flex: none; }
#tooltip .v { font-weight: 630; font-variant-numeric: tabular-nums; }
#tooltip .l { color: var(--ink-2); }
#orbit canvas { display: block; width: 100%; touch-action: none;
                border-radius: 6px; cursor: grab; }
.orbit-controls { display: flex; align-items: center; gap: 10px;
                  margin-top: 8px; flex-wrap: wrap; }
.orbit-controls button, .orbit-controls select {
  font: inherit; font-size: 12.5px; color: var(--ink-2);
  background: var(--surface); border: 1px solid var(--border);
  border-radius: 7px; padding: 3px 10px; cursor: pointer;
}
.orbit-controls input[type="range"] { flex: 1; min-width: 110px;
                                      accent-color: var(--s1); }
.chip { font-size: 11.5px; color: var(--muted); border: 1px solid var(--border);
        border-radius: 999px; padding: 2px 9px; white-space: nowrap; }
.chip.on { color: var(--ink); border-color: var(--s1); }
details { margin: 10px 2px; color: var(--ink-2); }
summary { cursor: pointer; font-size: 12.5px; }
table { border-collapse: collapse; margin-top: 8px; font-size: 12px; width: 100%; }
th { text-align: left; color: var(--ink-2); font-weight: 600; }
th, td { padding: 3px 10px 3px 0; border-bottom: 1px solid var(--grid); }
td.num { font-variant-numeric: tabular-nums; }
.sev { display: inline-block; width: 8px; height: 8px; border-radius: 50%;
       margin-right: 6px; vertical-align: baseline; }
</style>
</head>
<body>
<div class="report">
  <header>
    <h1>__TITLE__</h1>
    <span class="chips" id="chips"></span>
    <button id="themebtn" type="button">theme: auto</button>
  </header>
  <section class="tiles" id="tiles"></section>
  <div class="card" id="orbitcard"><h2>Orbit</h2><div id="orbit"></div></div>
  <div class="card"><h2>State channels</h2><div id="tracks"></div></div>
  <div class="card"><h2>Events</h2><div id="events"></div></div>
  <div class="card"><h2>Telemetry</h2><div id="lanes"></div><div id="xaxis"></div></div>
  <div class="card"><h2>Event log</h2><div id="evtable"></div></div>
</div>
<div id="tooltip"></div>
<script type="application/json" id="flight-data">__FLIGHT_JSON__</script>
<script>
"use strict";
var DATA = JSON.parse(document.getElementById("flight-data").textContent);
var PERIOD = DATA.meta.period_s, T_END = Math.max(DATA.meta.duration_s, 1);
var PADL = 52, PADR = 14;
var SERIES = ["var(--s1)", "var(--s2)", "var(--s3)", "var(--s4)"];
var SEV = { critical: "var(--critical)", warning: "var(--warning)",
            good: "var(--good)", neutral: "var(--muted)" };
var NS = "http://www.w3.org/2000/svg";
var charts = [];   // {svg, plotW, update(tOrNull)}

function el(name, attrs, parent) {
  var n = document.createElementNS(NS, name);
  for (var k in attrs) n.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(n);
  return n;
}
function div(cls, parent) {
  var n = document.createElement("div");
  if (cls) n.className = cls;
  if (parent) parent.appendChild(n);
  return n;
}
function fmt(v) {
  if (!isFinite(v)) return String(v);
  var a = Math.abs(v);
  if (a >= 1000) return v.toFixed(0);
  if (a >= 10) return v.toFixed(1);
  if (a >= 0.01 || a === 0) return v.toFixed(2);
  return v.toPrecision(2);
}
function orbits(t) { return t / PERIOD; }
function niceStep(span, n) {
  var raw = span / n, mag = Math.pow(10, Math.floor(Math.log10(raw)));
  var r = raw / mag;
  return (r >= 5 ? 10 : r >= 2 ? 5 : r >= 1 ? 2 : 1) * mag;
}
function xOf(t, plotW) { return PADL + (t / T_END) * plotW; }

function laneWidth() {
  return document.getElementById("lanes").clientWidth;
}

/* eclipse shading behind a plot area */
function drawBands(svg, plotW, h) {
  var ecl = null;
  for (var i = 0; i < DATA.tracks.length; i++)
    if (DATA.tracks[i].label === "eclipse") ecl = DATA.tracks[i].intervals;
  if (!ecl) return;
  ecl.forEach(function (iv) {
    el("rect", { x: xOf(iv[0], plotW), y: 0,
                 width: Math.max(1, xOf(iv[1], plotW) - xOf(iv[0], plotW)),
                 height: h, fill: "var(--band)" }, svg);
  });
}

function orbitGrid(svg, plotW, h) {
  var step = T_END / PERIOD > 6 ? 1 : 0.5;
  for (var o = step; o < T_END / PERIOD; o += step) {
    var x = xOf(o * PERIOD, plotW);
    el("line", { x1: x, y1: 0, x2: x, y2: h,
                 stroke: "var(--grid)", "stroke-width": 1 }, svg);
  }
}

/* ---- digital state strip ---- */
function drawTracks() {
  var host = document.getElementById("tracks");
  host.textContent = "";
  if (!DATA.tracks.length) { host.textContent = "no state channels"; return; }
  var W = host.clientWidth, ROW = 17;
  var H = DATA.tracks.length * ROW + 4;
  var plotW = W - PADL - PADR;
  var svg = el("svg", { width: W, height: H }, host);
  drawBands(svg, plotW, H); orbitGrid(svg, plotW, H);
  DATA.tracks.forEach(function (tr, i) {
    var y = i * ROW + 3;
    var t = el("text", { x: PADL - 8, y: y + 10, "text-anchor": "end",
                         "class": "rowlab" }, svg);
    t.textContent = tr.label;
    tr.intervals.forEach(function (iv) {
      el("rect", { x: xOf(iv[0], plotW), y: y,
                   width: Math.max(1.5, xOf(iv[1], plotW) - xOf(iv[0], plotW)),
                   height: ROW - 5, rx: 2, fill: "var(--s1)",
                   opacity: 0.55 }, svg);
    });
  });
  attachCrosshair(svg, plotW, H, function (tt, tHover) {
    tt.push(["", "t", ""]);
    DATA.tracks.forEach(function (tr) {
      var on = tr.intervals.some(function (iv) {
        return tHover >= iv[0] && tHover <= iv[1]; });
      tt.push([on ? "var(--s1)" : "transparent", tr.label, on ? "on" : "·"]);
    });
  });
}

/* ---- event timeline ---- */
function glyph(svg, x, y, sev) {
  var c = SEV[sev];
  if (sev === "critical")
    el("path", { d: "M" + x + " " + (y - 5) + " l5 9 h-10 z", fill: c,
                 stroke: "var(--surface)", "stroke-width": 2,
                 "paint-order": "stroke" }, svg);
  else if (sev === "warning")
    el("rect", { x: x - 4.2, y: y - 4.2, width: 8.4, height: 8.4, rx: 1.5,
                 transform: "rotate(45 " + x + " " + y + ")", fill: c,
                 stroke: "var(--surface)", "stroke-width": 2,
                 "paint-order": "stroke" }, svg);
  else if (sev === "good")
    el("circle", { cx: x, cy: y, r: 4.4, fill: c,
                   stroke: "var(--surface)", "stroke-width": 2,
                   "paint-order": "stroke" }, svg);
  else
    el("circle", { cx: x, cy: y, r: 3.8, fill: "none", stroke: c,
                   "stroke-width": 1.8 }, svg);
}

function drawEvents() {
  var host = document.getElementById("events");
  host.textContent = "";
  if (!DATA.events.length) { host.textContent = "no events"; return; }
  var sources = [];
  DATA.events.forEach(function (e) {
    if (sources.indexOf(e.source) < 0) sources.push(e.source);
  });
  var W = host.clientWidth, ROW = 20;
  var H = sources.length * ROW + 4, plotW = W - PADL - PADR;
  var svg = el("svg", { width: W, height: H }, host);
  drawBands(svg, plotW, H); orbitGrid(svg, plotW, H);
  sources.forEach(function (s, i) {
    var t = el("text", { x: PADL - 8, y: i * ROW + 16, "text-anchor": "end",
                         "class": "rowlab" }, svg);
    t.textContent = s;
  });
  DATA.events.forEach(function (e) {
    glyph(svg, xOf(e.t, plotW), sources.indexOf(e.source) * ROW + 12.5, e.severity);
  });
  attachCrosshair(svg, plotW, H, function (tt, tHover) {
    var win = T_END / 90;
    var near = DATA.events.filter(function (e) {
      return Math.abs(e.t - tHover) < win; }).slice(0, 8);
    near.forEach(function (e) {
      tt.push([SEV[e.severity], e.source + " · " + e.kind +
               (e.detail ? " (" + e.detail + ")" : ""),
               orbits(e.t).toFixed(2) + " orb"]);
    });
  });
}

/* ---- analog lanes ---- */
function drawLane(lane) {
  var host = div("lane", document.getElementById("lanes"));
  var head = div("lane-head", host);
  var ttl = div("t", head); ttl.textContent = lane.title;
  var unit = div("u", head); unit.textContent = lane.unit;
  if (lane.series.length > 1) {
    var lg = div("legend", head);
    lane.series.forEach(function (s, i) {
      var item = document.createElement("span");
      var key = document.createElement("span");
      key.className = "key"; key.style.borderTopColor = SERIES[i];
      item.appendChild(key);
      item.appendChild(document.createTextNode(s.label));
      lg.appendChild(item);
    });
  }
  var W = laneWidth(), H = 96, PT = 6, PB = 6;
  var plotW = W - PADL - PADR, plotH = H - PT - PB;
  var svg = el("svg", { width: W, height: H }, host);
  drawBands(svg, plotW, H); orbitGrid(svg, plotW, H);

  var lo = Infinity, hi = -Infinity;
  lane.series.forEach(function (s) {
    s.points.forEach(function (p) {
      if (p[1] < lo) lo = p[1];
      if (p[1] > hi) hi = p[1];
    });
  });
  if (lane.domain) { lo = lane.domain[0]; hi = lane.domain[1]; }
  if (hi - lo < 1e-9) { hi += 1; lo -= 1; }
  var pad = lane.domain ? 0 : (hi - lo) * 0.08;
  lo -= pad; hi += pad;
  var yOf = function (v) { return PT + (1 - (v - lo) / (hi - lo)) * plotH; };

  var step = niceStep(hi - lo, 3);
  for (var v = Math.ceil(lo / step) * step; v <= hi + 1e-9; v += step) {
    var y = yOf(v);
    el("line", { x1: PADL, y1: y, x2: PADL + plotW, y2: y,
                 stroke: "var(--grid)", "stroke-width": 1 }, svg);
    var t = el("text", { x: PADL - 6, y: y + 3.5, "text-anchor": "end" }, svg);
    t.textContent = fmt(v);
  }
  el("line", { x1: PADL, y1: PT + plotH, x2: PADL + plotW, y2: PT + plotH,
               stroke: "var(--axis)", "stroke-width": 1 }, svg);

  lane.series.forEach(function (s, i) {
    var d = "";
    s.points.forEach(function (p, j) {
      d += (j ? "L" : "M") + xOf(p[0], plotW).toFixed(1) + " " +
           yOf(p[1]).toFixed(1);
    });
    el("path", { d: d, fill: "none", stroke: SERIES[i], "stroke-width": 2,
                 "stroke-linejoin": "round", "stroke-linecap": "round" }, svg);
  });

  attachCrosshair(svg, plotW, H, function (tt, tHover) {
    lane.series.forEach(function (s, i) {
      var p = nearest(s.points, tHover);
      if (p) tt.push([SERIES[i], s.label, fmt(p[1]) + " " + lane.unit]);
    });
  });
}

function nearest(points, t) {
  if (!points.length) return null;
  var a = 0, b = points.length - 1;
  while (b - a > 1) {
    var m = (a + b) >> 1;
    if (points[m][0] < t) a = m; else b = m;
  }
  return (Math.abs(points[a][0] - t) < Math.abs(points[b][0] - t))
    ? points[a] : points[b];
}

/* shared x axis */
function drawXAxis() {
  var host = document.getElementById("xaxis");
  host.textContent = "";
  var W = laneWidth(), H = 24, plotW = W - PADL - PADR;
  var svg = el("svg", { width: W, height: H }, host);
  var step = T_END / PERIOD > 6 ? 1 : 0.5;
  for (var o = 0; o <= T_END / PERIOD + 1e-9; o += step) {
    var x = xOf(o * PERIOD, plotW);
    if (x > PADL + plotW + 1) break;
    el("line", { x1: x, y1: 0, x2: x, y2: 5, stroke: "var(--axis)",
                 "stroke-width": 1 }, svg);
    var t = el("text", { x: x, y: 17, "text-anchor": "middle" }, svg);
    t.textContent = o.toFixed(step < 1 ? 1 : 0);
  }
  var lbl = el("text", { x: PADL + plotW, y: 17, "text-anchor": "end" }, svg);
  lbl.textContent = "orbits";
}

/* ---- orbit view: canvas globe + animated satellite ---- */
function trackOn(label, t) {
  for (var i = 0; i < DATA.tracks.length; i++) {
    if (DATA.tracks[i].label !== label) continue;
    var iv = DATA.tracks[i].intervals;
    for (var j = 0; j < iv.length; j++)
      if (t >= iv[j][0] && t <= iv[j][1]) return true;
    return false;
  }
  return false;
}

function drawOrbitView() {
  if (window.__orbitStop) window.__orbitStop();
  var host = document.getElementById("orbit");
  host.textContent = "";
  var O = DATA.orbit3d;
  if (!O) { document.getElementById("orbitcard").style.display = "none"; return; }

  var W = host.clientWidth;
  var H = Math.max(240, Math.min(420, Math.round(W * 0.52)));
  var dpr = window.devicePixelRatio || 1;
  var canvas = document.createElement("canvas");
  canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
  canvas.style.width = W + "px"; canvas.style.height = H + "px";
  host.appendChild(canvas);
  var ctx = canvas.getContext("2d");
  ctx.scale(dpr, dpr);

  var bar = div("orbit-controls", host);
  var btn = document.createElement("button"); bar.appendChild(btn);
  var speedSel = document.createElement("select"); bar.appendChild(speedSel);
  [60, 300, 1500].forEach(function (v) {
    var o = document.createElement("option");
    o.value = v; o.textContent = v + "×";
    speedSel.appendChild(o);
  });
  speedSel.value = "300";
  var range = document.createElement("input");
  range.type = "range"; range.min = 0; range.max = T_END;
  range.step = Math.max(1, T_END / 2000); bar.appendChild(range);
  var orbChip = div("chip", bar), eclChip = div("chip", bar),
      conChip = div("chip", bar);

  var reduced = window.matchMedia &&
      matchMedia("(prefers-reduced-motion: reduce)").matches;
  var yaw = -0.9, pitch = 0.38, oT = 0, playing = !reduced, last = null,
      raf = null;
  var cx = W / 2, cy = H / 2;
  var s = (Math.min(W, H) / 2 - 8) / 1.32;
  var PERIOD_O = 2 * Math.PI / O.n_rad_s;

  /* view: Rz(yaw) then Rx(pitch); screen x right, ECI north up,
     depth d > 0 faces the viewer */
  function rot(v) {
    var c = Math.cos(yaw), sn = Math.sin(yaw);
    var x = v[0] * c - v[1] * sn, y = v[0] * sn + v[1] * c, z = v[2];
    var cp = Math.cos(pitch), sp = Math.sin(pitch);
    return [x, y * cp - z * sp, y * sp + z * cp];
  }
  function P(v) {
    var r = rot(v);
    return { x: cx + r[0] * s, y: cy - r[2] * s, d: -r[1] };
  }
  function satPos(t) {
    var u = O.n_rad_s * t, cu = Math.cos(u), su = Math.sin(u),
        R = O.r_orbit_re;
    return [R * (cu * O.e1[0] + su * O.e2[0]),
            R * (cu * O.e1[1] + su * O.e2[1]),
            R * (cu * O.e1[2] + su * O.e2[2])];
  }
  function siteEci(lat, lon, t) {
    var la = lat * Math.PI / 180;
    var lo = lon * Math.PI / 180 + O.gmst0_rad + O.w_earth_rad_s * t;
    return [Math.cos(la) * Math.cos(lo), Math.cos(la) * Math.sin(lo),
            Math.sin(la)];
  }
  function latLon(lat, lon) {
    var la = lat * Math.PI / 180, lo = lon * Math.PI / 180;
    return [Math.cos(la) * Math.cos(lo), Math.cos(la) * Math.sin(lo),
            Math.sin(la)];
  }
  function css(name) {
    return getComputedStyle(document.documentElement)
      .getPropertyValue(name).trim();
  }
  function occluded(p) {
    var dx = p.x - cx, dy = p.y - cy;
    return p.d < 0 && (dx * dx + dy * dy) < (s * 0.998) * (s * 0.998);
  }
  function polyline(pts, filter, color, width, alpha) {
    ctx.strokeStyle = color; ctx.lineWidth = width; ctx.globalAlpha = alpha;
    ctx.beginPath();
    var pen = false;
    for (var i = 0; i < pts.length; i++) {
      if (filter(pts[i])) {
        if (pen) ctx.lineTo(pts[i].x, pts[i].y);
        else { ctx.moveTo(pts[i].x, pts[i].y); pen = true; }
      } else pen = false;
    }
    ctx.stroke(); ctx.globalAlpha = 1;
  }
  function render() {
    ctx.clearRect(0, 0, W, H);
    var grid = css("--grid"), axis = css("--axis"), s1 = css("--s1"),
        s2 = css("--s2"), s3 = css("--s3"), s4 = css("--s4"),
        serious = css("--serious"), ink2 = css("--ink-2"),
        surface = css("--surface");
    var gmst = O.gmst0_rad + O.w_earth_rad_s * oT;
    var lonOff = gmst * 180 / Math.PI;
    function earthPt(lat, lon) { return P(latLon(lat, lon + lonOff)); }

    // graticule rotates with the Earth: back half, disk, front half
    var lines = [];
    for (var lon = 0; lon < 180; lon += 30) {
      var pts = [];
      for (var a = 0; a <= 360; a += 5) {
        var la = (a <= 180 ? a - 90 : 270 - a);
        pts.push(earthPt(la, a <= 180 ? lon : lon + 180));
      }
      lines.push(pts);
    }
    [-60, -30, 0, 30, 60].forEach(function (lat) {
      var pts = [];
      for (var b = 0; b <= 360; b += 5) pts.push(earthPt(lat, b));
      lines.push(pts);
    });
    lines.forEach(function (pts) {
      polyline(pts, function (p) { return p.d < 0; }, grid, 1, 0.45);
    });
    ctx.fillStyle = css("--band");
    ctx.beginPath(); ctx.arc(cx, cy, s, 0, 2 * Math.PI); ctx.fill();
    // crude terminator: darken the anti-sun half of the disk
    var sv = rot(O.sun), sl = Math.hypot(sv[0], sv[2]) || 1;
    ctx.save();
    ctx.beginPath(); ctx.arc(cx, cy, s, 0, 2 * Math.PI); ctx.clip();
    ctx.translate(cx, cy);
    ctx.rotate(Math.atan2(-sv[2], sv[0]) + Math.PI);
    ctx.fillStyle = "rgba(0,0,0,0.10)";
    ctx.fillRect(0, -s, s, 2 * s);
    ctx.restore();
    lines.forEach(function (pts) {
      polyline(pts, function (p) { return p.d >= 0; }, grid, 1, 0.9);
    });
    ctx.strokeStyle = axis; ctx.lineWidth = 1; ctx.globalAlpha = 1;
    ctx.beginPath(); ctx.arc(cx, cy, s, 0, 2 * Math.PI); ctx.stroke();

    // SAA box, on the rotating surface
    var saa = [];
    var la0 = O.saa.lat[0], la1 = O.saa.lat[1],
        lo0 = O.saa.lon[0], lo1 = O.saa.lon[1], k;
    for (k = lo0; k <= lo1; k += 5) saa.push(earthPt(la1, k));
    for (k = la1; k >= la0; k -= 5) saa.push(earthPt(k, lo1));
    for (k = lo1; k >= lo0; k -= 5) saa.push(earthPt(la0, k));
    for (k = la0; k <= la1; k += 5) saa.push(earthPt(k, lo0));
    polyline(saa, function (p) { return p.d >= 0; }, serious, 1.4, 0.6);
    var saaC = earthPt((la0 + la1) / 2, (lo0 + lo1) / 2);
    if (saaC.d > 0.15) {
      ctx.fillStyle = serious; ctx.globalAlpha = 0.75;
      ctx.font = "10px system-ui, sans-serif";
      ctx.fillText("SAA", saaC.x - 10, saaC.y + 3);
      ctx.globalAlpha = 1;
    }

    // orbit ring: dim behind the globe, faded in eclipse
    var t0ring = Math.floor(oT / PERIOD_O) * PERIOD_O;
    ctx.lineWidth = 2;
    for (var i = 0; i < 180; i++) {
      var ta = t0ring + (i / 180) * PERIOD_O,
          tb = t0ring + ((i + 1) / 180) * PERIOD_O;
      var pa = P(satPos(ta)), pb = P(satPos(tb));
      var hid = occluded(pa) || occluded(pb);
      var ecl = trackOn("eclipse", (ta + tb) / 2);
      ctx.strokeStyle = s1;
      ctx.globalAlpha = hid ? 0.10 : (ecl ? 0.25 : 0.75);
      ctx.beginPath(); ctx.moveTo(pa.x, pa.y); ctx.lineTo(pb.x, pb.y);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;

    // ground sites ride the rotation
    O.sites.forEach(function (site) {
      var p = P(siteEci(site.lat, site.lon, oT));
      if (p.d <= 0) return;
      var stn = site.kind === "station";
      ctx.beginPath();
      ctx.arc(p.x, p.y, stn ? 4.5 : 3.5, 0, 2 * Math.PI);
      ctx.fillStyle = stn ? s2 : s3; ctx.fill();
      ctx.lineWidth = 2; ctx.strokeStyle = surface; ctx.stroke();
      ctx.fillStyle = ink2; ctx.font = "10px system-ui, sans-serif";
      ctx.fillText(site.name, p.x + 7, p.y + 3);
    });

    // trail, then the satellite itself
    var sp = P(satPos(oT));
    ctx.lineWidth = 2; ctx.strokeStyle = s1;
    var TRAIL = Math.min(oT, PERIOD_O * 0.22);
    for (var j = 0; j < 24; j++) {
      var u0 = oT - TRAIL * (1 - j / 24), u1 = oT - TRAIL * (1 - (j + 1) / 24);
      var qa = P(satPos(u0)), qb = P(satPos(u1));
      if (occluded(qa) || occluded(qb)) continue;
      ctx.globalAlpha = 0.06 + 0.5 * (j / 24);
      ctx.beginPath(); ctx.moveTo(qa.x, qa.y); ctx.lineTo(qb.x, qb.y);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
    if (trackOn("ground contact", oT)) {
      var st = O.sites[0], gp = P(siteEci(st.lat, st.lon, oT));
      ctx.strokeStyle = s2; ctx.lineWidth = 1.5; ctx.globalAlpha = 0.8;
      ctx.beginPath(); ctx.moveTo(gp.x, gp.y); ctx.lineTo(sp.x, sp.y);
      ctx.stroke(); ctx.globalAlpha = 1;
    }
    ctx.globalAlpha = occluded(sp) ? 0.35 : 1;
    ctx.beginPath(); ctx.arc(sp.x, sp.y, 5, 0, 2 * Math.PI);
    ctx.fillStyle = s1; ctx.fill();
    ctx.lineWidth = 2; ctx.strokeStyle = surface; ctx.stroke();
    ctx.globalAlpha = 1;

    // sun direction glyph at the rim
    var gx = cx + (sv[0] / sl) * s * 1.22, gy = cy - (sv[2] / sl) * s * 1.22;
    ctx.globalAlpha = -sv[1] >= 0 ? 0.95 : 0.55;
    ctx.beginPath(); ctx.arc(gx, gy, 6, 0, 2 * Math.PI);
    ctx.fillStyle = s4; ctx.fill();
    for (var r = 0; r < 8; r++) {
      var an = r * Math.PI / 4;
      ctx.beginPath();
      ctx.moveTo(gx + Math.cos(an) * 8, gy + Math.sin(an) * 8);
      ctx.lineTo(gx + Math.cos(an) * 11, gy + Math.sin(an) * 11);
      ctx.strokeStyle = s4; ctx.lineWidth = 1.4; ctx.stroke();
    }
    ctx.globalAlpha = 1;

    orbChip.textContent = "orbit " + (oT / PERIOD_O).toFixed(2);
    eclChip.textContent = trackOn("eclipse", oT) ? "eclipse" : "sunlit";
    eclChip.className = "chip" + (trackOn("eclipse", oT) ? "" : " on");
    conChip.textContent = trackOn("ground contact", oT)
      ? "in contact" : "no contact";
    conChip.className = "chip" + (trackOn("ground contact", oT) ? " on" : "");
  }

  function frame(ts) {
    raf = null;
    if (last === null) last = ts;
    var dt = (ts - last) / 1000; last = ts;
    if (playing) {
      oT += dt * parseFloat(speedSel.value);
      if (oT > T_END) oT = 0;
      range.value = oT;
    }
    render();
    if (playing) raf = requestAnimationFrame(frame);
  }
  function setPlaying(on) {
    playing = on; last = null;
    btn.textContent = on ? "Pause" : "Play";
    if (on && raf === null) raf = requestAnimationFrame(frame);
  }
  btn.addEventListener("click", function () { setPlaying(!playing); });
  range.addEventListener("input", function () {
    oT = parseFloat(range.value);
    if (!playing) render();
  });
  speedSel.addEventListener("change", function () { last = null; });

  var dragging = false, lx = 0, ly = 0;
  canvas.addEventListener("pointerdown", function (ev) {
    dragging = true; lx = ev.clientX; ly = ev.clientY;
    canvas.setPointerCapture(ev.pointerId);
  });
  canvas.addEventListener("pointermove", function (ev) {
    if (!dragging) return;
    yaw += (ev.clientX - lx) * 0.008;
    pitch = Math.max(-1.35, Math.min(1.35, pitch + (ev.clientY - ly) * 0.008));
    lx = ev.clientX; ly = ev.clientY;
    if (!playing) render();
  });
  canvas.addEventListener("pointerup", function () { dragging = false; });

  window.__orbitSeek = function (t) {
    if (!playing) { oT = t; range.value = t; render(); }
  };
  window.__orbitStop = function () {
    if (raf !== null) cancelAnimationFrame(raf);
    raf = null; window.__orbitSeek = null;
  };
  btn.textContent = playing ? "Pause" : "Play";
  render();
  if (playing) raf = requestAnimationFrame(frame);
}

/* ---- crosshair + tooltip, shared across every chart ---- */
var tooltip = document.getElementById("tooltip");
function attachCrosshair(svg, plotW, h, fill) {
  var line = el("line", { y1: 0, y2: h, stroke: "var(--axis)",
                          "stroke-width": 1, visibility: "hidden" }, svg);
  charts.push({ svg: svg, plotW: plotW, line: line });
  svg.addEventListener("pointermove", function (ev) {
    var rect = svg.getBoundingClientRect();
    var t = ((ev.clientX - rect.left) - PADL) / plotW * T_END;
    if (t < 0 || t > T_END) { hideCross(); return; }
    charts.forEach(function (c) {
      var x = PADL + (t / T_END) * c.plotW;
      c.line.setAttribute("x1", x); c.line.setAttribute("x2", x);
      c.line.setAttribute("visibility", "visible");
    });
    var rows = [];
    fill(rows, t);
    showTooltip(ev, t, rows);
    if (window.__orbitSeek) window.__orbitSeek(t);  // paused globe follows
  });
  svg.addEventListener("pointerleave", hideCross);
}
function hideCross() {
  charts.forEach(function (c) { c.line.setAttribute("visibility", "hidden"); });
  tooltip.style.display = "none";
}
function showTooltip(ev, t, rows) {
  tooltip.textContent = "";
  var head = div("tt-t", tooltip);
  head.textContent = "orbit " + orbits(t).toFixed(2) + "  ·  t=" +
                     Math.round(t) + " s";
  rows.forEach(function (r) {
    var row = div("row", tooltip);
    var key = document.createElement("span");
    key.className = "key"; key.style.borderTopColor = r[0];
    row.appendChild(key);
    var v = document.createElement("span");
    v.className = "v"; v.textContent = r[2];
    var l = document.createElement("span");
    l.className = "l"; l.textContent = r[1];
    row.appendChild(v); row.appendChild(l);
  });
  tooltip.style.display = "block";
  var tw = tooltip.offsetWidth, th = tooltip.offsetHeight;
  var x = ev.clientX + 14, y = ev.clientY + 12;
  if (x + tw > window.innerWidth - 8) x = ev.clientX - tw - 14;
  if (y + th > window.innerHeight - 8) y = ev.clientY - th - 12;
  tooltip.style.left = x + "px"; tooltip.style.top = y + "px";
}

/* ---- header, tiles, event table ---- */
function drawChips() {
  document.getElementById("chips").textContent =
    "seed " + DATA.meta.seed + " · dt " + DATA.meta.dt +
    " s · epoch " + DATA.meta.epoch;
}
function drawTiles() {
  var host = document.getElementById("tiles");
  host.textContent = "";
  DATA.tiles.forEach(function (t) {
    var tile = div("tile", host);
    div("lb", tile).textContent = t.label;
    div("vl", tile).textContent = t.value;
    div("nt", tile).textContent = t.note || "";
  });
}
function drawEventTable() {
  var host = document.getElementById("evtable");
  host.textContent = "";
  if (!DATA.events.length) { host.textContent = "no events"; return; }
  var table = document.createElement("table");
  var tr = document.createElement("tr");
  ["orbit", "t (s)", "source", "event", "detail"].forEach(function (h) {
    var th = document.createElement("th"); th.textContent = h;
    tr.appendChild(th);
  });
  table.appendChild(tr);
  DATA.events.forEach(function (e) {
    var row = document.createElement("tr");
    function td(txt, num) {
      var c = document.createElement("td");
      if (num) c.className = "num";
      c.textContent = txt; row.appendChild(c); return c;
    }
    td(orbits(e.t).toFixed(2), true);
    td(String(Math.round(e.t)), true);
    td(e.source);
    var kc = td("", false);
    var dot = document.createElement("span");
    dot.className = "sev"; dot.style.background = SEV[e.severity];
    kc.appendChild(dot);
    kc.appendChild(document.createTextNode(e.kind));
    td(e.detail);
    table.appendChild(row);
  });
  host.appendChild(table);
}

/* ---- theme toggle ---- */
var themebtn = document.getElementById("themebtn");
var themes = ["auto", "light", "dark"], themeIdx = 0;
themebtn.addEventListener("click", function () {
  themeIdx = (themeIdx + 1) % 3;
  var t = themes[themeIdx];
  if (t === "auto") document.documentElement.removeAttribute("data-theme");
  else document.documentElement.setAttribute("data-theme", t);
  themebtn.textContent = "theme: " + t;
});

function buildAll() {
  charts = [];
  document.getElementById("lanes").textContent = "";
  drawChips(); drawTiles(); drawOrbitView(); drawTracks(); drawEvents();
  DATA.lanes.forEach(drawLane);
  drawXAxis(); drawEventTable();
}
var resizeTimer = null;
window.addEventListener("resize", function () {
  clearTimeout(resizeTimer);
  resizeTimer = setTimeout(buildAll, 150);
});
buildAll();
</script>
</body>
</html>
"""


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Render flight recordings to self-contained HTML reports.")
    ap.add_argument("recordings", nargs="+", help="flight recorder .db files")
    ap.add_argument("-o", "--out", help="output path (single recording only)")
    args = ap.parse_args()
    if args.out and len(args.recordings) > 1:
        ap.error("-o only makes sense with a single recording")
    for db in args.recordings:
        out = render_flight(db, args.out)
        print(f"{db} -> {out}")


if __name__ == "__main__":
    main()
