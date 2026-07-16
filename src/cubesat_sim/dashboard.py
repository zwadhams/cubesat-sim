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
from pathlib import Path

from cubesat_sim.environment.orbit import CircularOrbit

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

        return {
            "title": db_path.stem,
            "meta": {"seed": meta.get("seed", "?"), "dt": meta.get("dt", "?"),
                     "epoch": json.loads(meta["epoch"]) if "epoch" in meta else "?",
                     "duration_s": _round(t_end), "period_s": _round(period)},
            "tiles": tiles,
            "tracks": tracks,
            "events": events,
            "lanes": lanes,
        }
    finally:
        db.close()


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
  drawChips(); drawTiles(); drawTracks(); drawEvents();
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
