"""Flight report dashboard: one recording -> one self-contained HTML file.

Reads a flight recorder .db and renders a static report — stat tiles, a
digital state strip (eclipse / contact / safe mode / shedding ...), an
event timeline with severity glyphs, and analog telemetry lanes with a
shared crosshair — into a single HTML file with zero external
dependencies (inline CSS/JS, data embedded as JSON). Open it in any
browser; light/dark follow the OS with a manual toggle.

The report teaches itself: GLOSSARY terms grow dotted-underline hover
tooltips wherever they appear (tiles, hints, legends, row labels), the
event log defines every event kind on hover via EVENT_GLOSS, and the
primer opens with the system in one minute. `_annotations` runs the
emergent-behavior catalog's signatures against the flight and writes a
plain-language "What happened" card: each finding is click-to-zoom onto
its evidence and links to the catalog entry it matched (the real
EMERGENT_BEHAVIORS.md text, embedded via `parse_catalog` so the report
stays self-contained). A flight that goes off-nominal but matches no
known signature is flagged "possibly new" — a prompt to investigate and,
if real, catalog it. The goal is that a reader can learn the whole
spacecraft, and what this particular flight did, without leaving the
page.

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
import re
import sqlite3
from datetime import datetime, timezone
from pathlib import Path

from cubesat_sim.environment.groundstation import gmst_rad
from cubesat_sim.environment.orbit import R_EARTH, CircularOrbit
from cubesat_sim.environment.sun import sun_direction_eci
from cubesat_sim.faults import SAA_LAT_DEG, SAA_LON_DEG
from cubesat_sim.physics.spacecraft import DEFAULT_STATION, DEFAULT_TARGETS

# the emergent-behavior catalog, parsed at render time so findings can
# link to the real entry text and the report stays self-contained
_CATALOG_PATH = Path(__file__).resolve().parents[2] / "EMERGENT_BEHAVIORS.md"

# analog lanes: (title, section, hint, unit, fixed y-domain or None, series)
# each series: (source, key, label, transform)
_K2C = ("kelvin", lambda v: v - 273.15)
_ID = (None, None)
ANALOG_LANES = [
    ("State of charge", "Power & thermal",
     "Truth, the sag-biased onboard estimate, and the ground's snapshot "
     "(which only updates during passes).",
     "fraction", (0.0, 1.0), [
        ("physics", "soc_true", "true", _ID),
        ("eps", "soc_est", "EPS estimate", _ID),
        ("ground", "sat_soc_est", "ground last heard", _ID),
    ]),
    ("Electrical power", "Power & thermal",
     "Generation follows sun-pointing and eclipse; load follows whatever "
     "is switched on (payload, heater, radio TX).",
     "W", None, [
        ("physics", "p_gen_w", "generation", _ID),
        ("physics", "p_load_w", "load", _ID),
    ]),
    ("Battery voltage", "Power & thermal",
     "Sags under discharge, rises under charge — the raw (and misleading) "
     "signal behind the SoC estimate.",
     "V", None, [
        ("physics", "battery_v_true", "true", _ID),
    ]),
    ("Temperatures", "Power & thermal",
     "Li-ion cells must stay above 0 °C to charge; the battery heater "
     "spends watts to keep them there.",
     "°C", None, [
        ("physics", "t_batt_k", "battery", _K2C),
        ("physics", "t_struct_k", "structure", _K2C),
    ]),
    ("Body rate", "Attitude",
     "How fast the satellite is rotating. The ADCS estimate freezes if "
     "its gyro latches up — watch for the two lines splitting.",
     "deg/s", None, [
        ("physics", "rate_dps", "true", _ID),
        ("adcs", "rate_dps", "ADCS estimate", _ID),
    ]),
    ("Sun facing", "Attitude",
     "Cosine of the panel-to-sun angle: 1 is full sun on the array, "
     "0 edge-on, negative is anti-sun. Generation follows this.",
     "cosine", (-1.0, 1.0), [
        ("physics", "sun_facing", "panel · sun", _ID),
    ]),
    ("Wheel momentum", "Attitude",
     "Momentum stored in the reaction wheels; near 1.0 they saturate and "
     "the magnetorquers must dump momentum.",
     "frac of max", (0.0, 1.0), [
        ("physics", "wheel_h_frac", "true", _ID),
        ("adcs", "wheel_frac", "ADCS estimate", _ID),
    ]),
    ("Data", "Data & link",
     "Imaging fills the onboard queue; passes drain it to the ground "
     "archive; overflow when full is dropped forever.",
     "MB", None, [
        ("comms", "queue_mb", "onboard queue", _ID),
        ("ground", "archive_mb", "ground archive", _ID),
        ("comms", "dropped_mb", "dropped", _ID),
    ]),
    ("Link", "Data & link",
     "Cumulative counts: frames the ground rejected (CRC), frame-counter "
     "gaps it noticed, and command retransmissions.",
     "count", None, [
        ("ground", "frames_rejected", "frames rejected", _ID),
        ("ground", "seq_gaps", "sequence gaps", _ID),
        ("ground", "tc_retransmits", "TC retransmits", _ID),
    ]),
    ("Battery capacity", "Degradation",
     "Cycle aging: every watt-hour through the pack shrinks the tank a "
     "little.",
     "Wh", None, [
        ("physics", "batt_capacity_wh", "capacity", _ID),
    ]),
    ("Array health", "Degradation",
     "Radiation darkening in sunlight, plus any debris strikes.",
     "illumination", None, [
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

# One glossary, two uses: rendered as the primer's reference grid, and
# matched against visible text so every acronym / term of art grows a
# hover tooltip. Keys are matched case-sensitively when they look like
# acronyms (any uppercase), case-insensitively otherwise; a trailing
# "s" is tolerated so "SEUs" and "passes" resolve. Grouped for the grid:
# subsystems first, then power, attitude, link, radiation, sim.
GLOSSARY = {
    "physics": "the truth layer — actual orbit, attitude, battery and "
               "temperatures; no subsystem sees it directly, only noisy "
               "sensors",
    "EPS": "electrical power system — battery, solar array, and the load "
           "switches (flown in bare-metal-style C)",
    "OBC": "on-board computer — mode management and FDIR (flown in C)",
    "ADCS": "attitude determination and control system — gyro, "
            "magnetometer, sun sensor, wheels, magnetorquers (flown in "
            "Rust)",
    "comms": "radio and CCSDS framer with the downlink queue (flown in "
             "C++)",
    "payload": "the imaging instrument — fills the data queue when "
               "capturing over a target",
    "thermal": "thermal control — a bang-bang battery heater",
    "ground": "ground station: frame decoder, archive, and the operator "
              "rules that command the payload",
    "faults": "the fault injector — scheduled hardware misfortunes plus "
              "random SEUs, elevated over the SAA",
    "console": "a human at the live console — manual TCs and fault "
               "injections, recorded like everything else",
    "SoC": "state of charge — how full the battery is, 0 to 1",
    "safe mode": "OBC protective mode: payload off to save power until "
                 "the SoC estimate recovers",
    "load shed": "the EPS's own last-ditch veto — non-essential loads "
                 "(ADCS, heater, payload) forced off at very low "
                 "estimated SoC",
    "charge inhibit": "li-ion cells must not charge below 0 °C — in deep "
                      "cold the battery drains even in full sun",
    "brownout": "the battery hit empty; everything shuts down until "
                "sunlight brings it back",
    "heater request": "thermal control asking the EPS for battery-heater "
                      "power",
    "eclipse": "in Earth's shadow — no generation, battery discharging",
    "li-ion": "lithium-ion battery chemistry — hence the 0 °C cold-charge "
              "limit",
    "gyro": "gyroscope — measures body rotation rate; the only one "
            "aboard, so FDIR can't cross-check it",
    "magnetometer": "measures Earth's magnetic field direction; drives "
                    "B-dot detumble",
    "magnetorquers": "electromagnets pushing against Earth's field — "
                     "detumble and wheel-momentum dumping",
    "detumble": "ADCS mode that kills rotation after deployment (B-dot "
                "law); exits below 0.5 deg/s",
    "sun-point mode": "ADCS mode steering the solar panel at the sun — "
                      "where the watts come from",
    "wheel momentum": "spin stored in the reaction wheels; near "
                      "saturation they lose control authority and must "
                      "desaturate",
    "imaging": "the instrument is capturing — data flowing into the "
               "onboard queue",
    "target visible": "an imaging target is in view below the "
                      "spacecraft",
    "pass": "the few minutes the satellite is above the ground station's "
            "horizon — the only time the link exists",
    "beacon": "the periodic housekeeping TM frame — one packet per "
              "subsystem, the ground's whole picture of the spacecraft",
    "TM": "telemetry — downlink transfer frames (sync marker, counters, "
          "CRC)",
    "TC": "telecommand — an uplinked command frame, retransmitted until "
          "the beacon acknowledges it",
    "ARQ": "automatic repeat request — resend until acknowledged",
    "CRC": "cyclic redundancy check — the frame checksum; corruption "
           "fails it and the frame is discarded",
    "VC0": "virtual channel 0 — housekeeping beacons",
    "VC1": "virtual channel 1 — bulk science data",
    "sequence gap": "a jump in the frame counters — the observable proof "
                    "that frames went missing",
    "SEU": "single-event upset — a radiation strike flips one bit in a "
           "sensor word",
    "SAA": "South Atlantic Anomaly — a radiation hotspot where the SEU "
           "rate jumps ~25×",
    "FDIR": "fault detection, isolation, recovery — here, a gyro "
            "watchdog that power-cycles the ADCS, three attempts max",
    "latch-up": "a sensor stuck repeating one output word; soft ones "
                "clear on a power cycle, hard ones are forever",
    "seed": "the mission's random seed — same seed and dt replays the "
            "identical flight",
    "dt": "the simulation timestep in seconds",
    "epoch": "mission start time (UTC) — sets sun geometry and ground "
             "tracks",
}

# Every event kind that can appear in a recording, in plain language.
# The event log shows these on hover; kinds missing here render without
# a tooltip (tests keep this honest for kinds the sample flights emit).
EVENT_GLOSS = {
    "mode_change": "a mode switch — OBC NOMINAL/SAFE or ADCS "
                   "detumble/sun-point (detail says which and why)",
    "load_shed": "EPS forced non-essential loads off — estimated SoC "
                 "fell below the shed threshold",
    "load_restore": "EPS re-enabled shed loads — the estimate recovered "
                    "past the restore threshold",
    "brownout": "battery hit empty; all loads down until sunlight "
                "recharges it",
    "gyro_anomaly": "the OBC watchdog first noticed the gyro misbehaving "
                    "(exact repeats or impossible rates)",
    "fdir_adcs_power_cycle": "FDIR's classic first move: cut the ADCS "
                             "rail to clear a latch-up",
    "fdir_adcs_repower": "power-cycle dwell over — ADCS rail back on",
    "fdir_giveup": "FDIR exhausted its three power cycles and stopped "
                   "trying; the ADCS stays powered, the ground inherits "
                   "the problem",
    "seu_corruption": "a radiation bit flip landed in a sensor word",
    "inject": "a hardware fault was injected — by the campaign script, "
              "or by hand from the console",
    "inject_seu": "a random SEU fired (Poisson process, ~25× over the "
                  "SAA)",
    "latchup_cleared": "a soft sensor latch-up cleared by the ADCS power "
                       "cycle",
    "imaging_start": "payload began capturing over a visible target",
    "imaging_stop": "capture ended — target out of view or instrument "
                    "off",
    "instrument_enable": "the payload instrument switched on (ground "
                         "command took effect)",
    "instrument_disable": "the payload instrument switched off (ground "
                          "command took effect)",
    "operator_enable_payload": "ground operator rule decided to re-enable "
                               "the payload and queued the TC",
    "operator_disable_payload": "ground operator rule vetoed the payload "
                                "(storage or power) and queued the TC",
    "operator_manual_tc": "a human queued a TC from the live console's "
                          "command panel",
    "uplink_dispatch": "the spacecraft's FARM accepted a TC frame and "
                       "executed the command",
    "uplink_acked": "the beacon's acceptance counter advanced — the "
                    "ground stops retransmitting",
    "tc_reject": "a TC frame failed CRC or parsing at the spacecraft and "
                 "was discarded",
    "tc_duplicate": "an already-accepted TC sequence number arrived "
                    "again (a retransmission crossed the ack); ignored",
    "tc_out_of_sequence": "a TC arrived out of order and was refused — "
                          "FARM accepts strictly in sequence",
    "vc0_gap": "housekeeping frame counter jumped — beacons were lost in "
               "the channel",
    "vc1_gap": "science frame counter jumped — data frames were lost in "
               "the channel",
    "frame_reject": "a corrupted or unparseable frame was discarded "
                    "(ground CRC check, or flight software input guard)",
    "storage_full": "the onboard data queue is full — new science is "
                    "being dropped",
    "storage_recovered": "the queue drained below full; capturing "
                         "resumes",
    "desat_start": "wheel momentum near saturation — magnetorquers "
                   "started dumping it",
    "desat_stop": "momentum dump complete",
    "pub_reject": "the bridge quarantined a poison bus message from "
                  "flight software (null / non-finite)",
    "telemetry_reject": "the bridge quarantined a poison telemetry value "
                        "from flight software",
    "uplink": "a console TC queued into the ground station's ARQ",
    "publish": "a raw bus message injected from the console",
    "eclipse_enter": "into Earth's shadow", "eclipse_exit": "back into "
    "sunlight",
    "contact_aos": "acquisition of signal — pass begins",
    "contact_los": "loss of signal — pass ends",
    "charge_inhibit_on": "battery below 0 °C — charging blocked",
    "charge_inhibit_off": "battery warm enough to charge again",
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


def _overlap(iv_a, iv_b):
    """Total seconds where intervals from list A overlap intervals in B."""
    total = 0.0
    for a0, a1 in iv_a:
        for b0, b1 in iv_b:
            total += max(0.0, min(a1, b1) - max(a0, b0))
    return total


def _annotations(evs, tracks_map, bag, t_end, period) -> list[dict]:
    """Auto-detected signatures: run the emergent-behavior catalog's
    known mechanisms against this flight and write a sentence for each
    match, anchored to a time span the reader can zoom to. Detectors are
    deliberately conservative — a finding must be cheap to verify by eye
    in the charts right below it."""
    notes: list[dict] = []

    def note(sev, t0, t1, text):
        notes.append({"sev": sev, "t0": _round(max(0.0, t0)),
                      "t1": _round(min(t_end, max(t0, t1))), "text": text})

    def orb(t):
        return f"{t / period:.2f}"

    # -- FDIR giveup, classified by which side of the mode gate the gyro
    #    froze on (catalog entries 7 and 10) ------------------------------
    giveups = [e for e in evs if e[2] == "fdir_giveup"]
    if giveups:
        tg = giveups[0][0]
        est_tail = [v for t, v in bag.get("rate_est", []) if t > tg]
        true_tail = [v for t, v in bag.get("rate_true", []) if t > tg]
        # "frozen" tolerates cross-language float wobble in norm(gyro):
        # a latched sensor reads a near-constant word, not a bit-exact one
        frozen = (len(est_tail) >= 2
                  and max(est_tail) - min(est_tail) < 0.02)
        est_last = est_tail[-1] if est_tail else 0.0
        true_last = true_tail[-1] if true_tail else 0.0
        if est_tail and max(est_tail) <= 0.5 and true_last > 2.0:
            # belief (calm) and truth (tumbling) diverged: the confident
            # corpse — a gyro stuck below the gate, locked into sun-point
            note("critical", tg, t_end,
                 f"FDIR gave up at orbit {orb(tg)} with the ADCS reporting "
                 f"{est_last:.2f} deg/s — below every mode gate, so it stayed "
                 f"locked in sun-point with a dead damping term and pumped the "
                 f"true rate to {true_last:.1f} deg/s. The watchdog checks the "
                 f"same frozen word, so nothing on board can know (catalog "
                 f"entry 10).")
        elif frozen and est_last > 0.5:
            note("critical", tg, t_end,
                 f"FDIR gave up at orbit {orb(tg)} on a gyro frozen at "
                 f"{est_last:.2f} deg/s — above the detumble-exit gate, so "
                 f"sun-point never engages again and the power budget runs a "
                 f"structural deficit (catalog entry 7).")
        else:
            note("critical", tg, t_end,
                 f"FDIR exhausted its three ADCS power cycles at orbit "
                 f"{orb(tg)} and gave up; the ADCS flies on whatever the "
                 f"gyro reports now.")

    # -- recovery that worked ---------------------------------------------
    cleared = [e for e in evs if e[2] == "latchup_cleared"]
    if cleared:
        note("good", cleared[0][0], cleared[-1][0],
             f"FDIR recovery worked as designed: {len(cleared)} soft "
             f"latch-up(s) cleared by power-cycling the ADCS rail.")

    # -- SEU flips the mode switch (catalog entry 12) ---------------------
    flips = [e for e in evs if e[2] == "mode_change"
             and e[3].get("to") == "NOMINAL"
             and float(e[3].get("soc_est", 0.0)) >= 0.999]
    if flips:
        note("warning", flips[0][0], flips[-1][0],
             f"radiation toggled the mode switch ×{len(flips)}: a SEU "
             f"saturated the SoC estimate to exactly 1.0 for one sample, "
             f"and the un-debounced SAFE-exit gate believed it — payload "
             f"commanded on mid-crisis (catalog entry 12).")

    # -- the shed one-way door (catalog entry 6) --------------------------
    shed_iv = tracks_map.get("load shed", [])
    if shed_iv and shed_iv[-1][1] >= t_end - 60.0 \
            and shed_iv[-1][1] - shed_iv[-1][0] > period:
        a = shed_iv[-1][0]
        facing = [v for t, v in bag.get("sun_facing", []) if t > a]
        if facing and sum(facing) / len(facing) < 0.0:
            note("critical", a, t_end,
                 f"the EPS shed at orbit {orb(a)} latched to the end of the "
                 f"flight: the freewheeling satellite settled anti-sun, "
                 f"generation pinned at the side-panel floor, and the "
                 f"estimate can never climb back over the restore threshold "
                 f"— the one-way door (catalog entry 6).")
        else:
            note("warning", a, t_end,
                 f"the EPS load shed at orbit {orb(a)} never released — the "
                 f"SoC estimate stayed below the restore threshold for the "
                 f"rest of the flight.")

    # -- cold-charge trap (catalog entry 3's ingredient) ------------------
    inhib = tracks_map.get("charge inhibit", [])
    eclipse = tracks_map.get("eclipse", [])
    if inhib:
        sunlit_blocked = sum(b - a for a, b in inhib) - _overlap(inhib, eclipse)
        if sunlit_blocked > 300.0:
            note("warning", inhib[0][0], inhib[-1][1],
                 f"sun on the panel, charging physically blocked: the "
                 f"battery sat below the li-ion 0 °C limit for "
                 f"{sunlit_blocked / 60.0:.0f} sunlit minutes while loads "
                 f"kept draining (catalog entry 3's trap).")

    # -- brownout ---------------------------------------------------------
    brs = [e for e in evs if e[2] == "brownout"]
    if brs:
        note("critical", brs[0][0], brs[-1][0],
             f"the battery hit empty at orbit {orb(brs[0][0])} — every "
             f"protection upstream of this either fired or was defeated.")

    # -- the watchdog's blind spot (catalog entry 8) ----------------------
    mag_stuck = [e for e in evs if e[2] == "inject"
                 and e[3].get("sensor") == "mag" and e[3].get("stuck")]
    fdir_ts = [e[0] for e in evs if e[2] == "fdir_adcs_power_cycle"]
    for m in mag_stuck:
        if not any(abs(ft - m[0]) < 600.0 for ft in fdir_ts):
            note("warning", m[0], m[0],
                 f"a magnetometer latch-up at orbit {orb(m[0])} passed "
                 f"unnoticed — the watchdog only monitors the gyro; a "
                 f"tumbling satellite with a frozen mag cannot detumble and "
                 f"nothing on board would know (catalog entry 8).")
            break

    # -- ground veto latch (catalog entry 11) -----------------------------
    dis = [e for e in evs if e[2] == "operator_disable_payload"]
    ena = [e for e in evs if e[2] == "operator_enable_payload"]
    if dis and (not ena or ena[-1][0] < dis[-1][0]):
        td = dis[-1][0]
        targets = [iv for iv in tracks_map.get("target visible", [])
                   if iv[0] > td]
        imaged = [iv for iv in tracks_map.get("imaging", []) if iv[0] > td]
        if targets and not imaged and t_end - td > period:
            note("warning", td, t_end,
                 f"the ground's payload veto latched at orbit {orb(td)} and "
                 f"never released — {len(targets)} target pass(es) went "
                 f"unimaged. Re-enable requires hearing a healthy estimate "
                 f"during a pass, and a sagging estimator never delivers "
                 f"one (catalog entry 11).")

    # -- eclipse-phase-locked limit cycle (catalog entry 1) ---------------
    safe_iv = tracks_map.get("safe mode", [])
    if len(safe_iv) >= 3:
        starts = [a for a, _ in safe_iv]
        gaps = [y - x for x, y in zip(starts, starts[1:])]
        if gaps and all(0.6 * period < g < 1.4 * period for g in gaps):
            note("warning", safe_iv[0][0], safe_iv[-1][1],
                 f"a SAFE/NOMINAL limit cycle phase-locked to the eclipse "
                 f"cycle (×{len(safe_iv)}, ~once per orbit): voltage sag "
                 f"biases the SoC estimate low in eclipse, turning the "
                 f"hysteresis band into an oscillator (catalog entry 1).")

    # -- bridge quarantine ------------------------------------------------
    rej = [e for e in evs if e[2] in ("pub_reject", "telemetry_reject")]
    if rej:
        note("warning", rej[0][0], rej[-1][0],
             f"the bridge quarantined {len(rej)} poison value(s) from "
             f"flight software — JSON null / non-finite words rejected "
             f"before they could reach the bus (catalog entry 9).")

    # tag every finding that names a catalog entry, so the report can link
    # it and so "explained" below knows which distress the catalog covers
    for n in notes:
        m = re.search(r"entry (\d+)", n["text"])
        if m:
            n["entry"] = int(m.group(1))

    # -- possibly new: distress the catalog didn't explain ----------------
    # if the flight went off-nominal but no known signature fired, flag it
    # loudly — the whole point of the simulator is to surface behaviors
    # that aren't in EMERGENT_BEHAVIORS.md yet
    true_tail = [v for t, v in bag.get("rate_true", []) if t > t_end - period]
    shed_iv = tracks_map.get("load shed", [])
    safe_iv = tracks_map.get("safe mode", [])
    distress = []
    if any(e[2] == "brownout" for e in evs):
        distress.append("browned out")
    if any(e[2] == "fdir_giveup" for e in evs):
        distress.append("FDIR gave up")
    if true_tail and true_tail[-1] > 2.0:
        distress.append(f"ended tumbling at {true_tail[-1]:.1f} deg/s")
    if (shed_iv and shed_iv[-1][1] >= t_end - 60.0
            and shed_iv[-1][1] - shed_iv[-1][0] > period):
        distress.append("load-shed latched to the end")
    if (safe_iv and safe_iv[-1][1] >= t_end - 60.0
            and safe_iv[-1][1] - safe_iv[-1][0] > period):
        distress.append("stuck in safe mode")
    explained = any("entry" in n for n in notes)
    if distress and not explained:
        notes.append({
            "sev": "critical", "new": True,
            "t0": _round(0.0), "t1": _round(t_end),
            "text": "This flight went off-nominal (" + "; ".join(distress) +
            ") but matched no catalogued mechanism — possibly a new "
            "emergent behavior. Worth digging into and, if it's real, "
            "adding to EMERGENT_BEHAVIORS.md as a new entry."})

    return sorted(notes, key=lambda n: (0 if n.get("new") else 1, n["t0"]))


def _fmt_detail(detail: dict) -> str:
    parts = []
    for k, v in detail.items():
        if isinstance(v, float):
            parts.append(f"{k}={v:.4g}")
        else:
            parts.append(f"{k}={v}")
    return ", ".join(parts)


_CATALOG_CACHE: dict[str, dict] | None = None


def parse_catalog(path: Path = _CATALOG_PATH) -> dict[str, dict]:
    """Parse EMERGENT_BEHAVIORS.md into {entry_number: {title, mechanism,
    status}} so findings can carry the real catalog text (kept embedded —
    the report must stay self-contained). Cached; missing file yields an
    empty catalog and findings simply render without the expandable
    entry."""
    global _CATALOG_CACHE
    if _CATALOG_CACHE is not None:
        return _CATALOG_CACHE
    catalog: dict[str, dict] = {}
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        _CATALOG_CACHE = catalog
        return catalog
    # entries are "## N. Title", each with "- mechanism:" and "- status:"
    # bullets whose values can wrap across lines until the next bullet
    for block in re.split(r"\n(?=## \d+\.)", text):
        m = re.match(r"## (\d+)\.\s*(.+)", block)
        if not m:
            continue
        num, title = m.group(1), m.group(2).strip()

        def field(name: str) -> str:
            fm = re.search(rf"- {name}:\s*(.+?)(?=\n- |\n## |\Z)", block, re.S)
            return re.sub(r"\s+", " ", fm.group(1)).strip() if fm else ""

        catalog[num] = {"title": title, "mechanism": field("mechanism"),
                        "status": field("status")}
    _CATALOG_CACHE = catalog
    return catalog


def compute_annotations(db: sqlite3.Connection, t_end: float,
                        period: float) -> list[dict]:
    """Run the catalog-signature detectors against an open recording.
    Shared by the flight report and the live console (which recomputes it
    as the flight grows, treating 'now' as the end)."""
    raw_events = [
        (t, source, kind, json.loads(detail) if detail else {})
        for t, source, kind, detail in db.execute(
            "SELECT time, source, kind, detail FROM events "
            "WHERE time <= ? ORDER BY tick", (t_end,))]
    tracks_map = {}
    for source, key, label in DIGITAL_TRACKS:
        pts = [(t, v) for t, v in db.execute(
            "SELECT time, value FROM telemetry WHERE source=? AND key=? "
            "AND time <= ? ORDER BY tick", (source, key, t_end))
            if v is not None]
        iv = _intervals(pts, t_end)
        if iv:
            tracks_map[label] = iv
    bag = {}
    for name, source, key in (("rate_true", "physics", "rate_dps"),
                              ("rate_est", "adcs", "rate_dps"),
                              ("sun_facing", "physics", "sun_facing")):
        bag[name] = [(t, v) for t, v in db.execute(
            "SELECT time, value FROM telemetry WHERE source=? AND key=? "
            "AND time <= ? ORDER BY tick", (source, key, t_end))
            if v is not None]
    return _annotations(raw_events, tracks_map, bag, t_end, period)


def load_flight(db_path: str | Path) -> dict:
    db_path = Path(db_path)
    db = sqlite3.connect(str(db_path))
    try:
        meta = dict(db.execute("SELECT key, value FROM meta").fetchall())
        t_end = db.execute("SELECT MAX(time) FROM telemetry").fetchone()[0] or 0.0
        period = CircularOrbit().period_s

        lanes = []
        for title, section, hint, unit, domain, series_spec in ANALOG_LANES:
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
                lanes.append({"title": title, "section": section,
                              "hint": hint, "unit": unit,
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
             "note": "battery hit empty" if n_brownouts else "never browned out",
             "warn": n_brownouts > 0},
            {"label": "Safe-mode entries", "value": str(n_safe), "note": ""},
            {"label": "FDIR power cycles", "value": str(n_cycles),
             "note": "gave up" if gave_up else "", "warn": gave_up},
            {"label": "Faults injected", "value": str(n_faults),
             "note": f"+ {n_seus} SEUs" if n_seus else ""},
        ]
        if archive:
            tiles.append({"label": "Data archived",
                          "value": f"{archive[-1][1]:.0f} MB",
                          "note": (f"{dropped[-1][1]:.0f} MB dropped"
                                   if dropped and dropped[-1][1] > 0 else ""),
                          "warn": bool(dropped and dropped[-1][1] > 0)})
        if cap and illum:
            tiles.append({"label": "Degradation",
                          "value": f"{cap[-1][1]:.2f} Wh",
                          "note": f"array at {illum[-1][1]:.3f}"})

        epoch_iso = json.loads(meta["epoch"]) if "epoch" in meta else None
        annotations = compute_annotations(db, t_end, period)

        return {
            "title": db_path.stem,
            "meta": {"seed": meta.get("seed", "?"), "dt": meta.get("dt", "?"),
                     "epoch": epoch_iso or "?",
                     "duration_s": _round(t_end), "period_s": _round(period)},
            "tiles": tiles,
            "tracks": tracks,
            "events": events,
            "lanes": lanes,
            "annotations": annotations,
            "catalog": parse_catalog(),
            "orbit3d": _orbit_geometry(epoch_iso),
            "gloss": GLOSSARY,
            "evgloss": EVENT_GLOSS,
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
  --bandc: rgba(42,120,214,0.07);
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
    --bandc: rgba(57,135,229,0.13);
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
  --bandc: rgba(57,135,229,0.13);
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
.tile .vl.warn { color: var(--serious); }
.tile .nt { font-size: 11.5px; color: var(--muted); margin-top: 1px; min-height: 15px; }
.card { background: var(--surface); border: 1px solid var(--border);
        border-radius: 10px; padding: 12px 14px 10px; margin-bottom: 12px; }
.card h2 { font-size: 13px; font-weight: 650; margin: 0 0 8px; color: var(--ink); }
.card-head { display: flex; align-items: baseline; gap: 10px;
             flex-wrap: wrap; }
.card-head h2 { margin-bottom: 8px; }
.card-head .zinfo { font-size: 11.5px; color: var(--muted); }
.card-head button { margin-left: auto; font: inherit; font-size: 11.5px;
  color: var(--ink-2); background: var(--surface);
  border: 1px solid var(--border); border-radius: 7px; padding: 1px 9px;
  cursor: pointer; }
/* telemetry window readout: prominent, always on */
#zinfo { font-size: 12px; color: var(--ink-2);
         font-variant-numeric: tabular-nums; }
/* zoom / pan control bar */
.zoomctl { margin-left: auto; display: inline-flex; gap: 4px;
           align-self: center; }
.zoomctl button { margin-left: 0; font-size: 12px; min-width: 27px;
                  text-align: center; padding: 2px 8px; }
.zoomctl button:hover { border-color: var(--s1); color: var(--ink); }
.zoomctl button:disabled { opacity: 0.4; cursor: default;
                           border-color: var(--border); color: var(--ink-2); }
svg text.axsub { font-size: 9.5px; fill: var(--muted); }
.intro { background: var(--surface); border: 1px solid var(--border);
         border-radius: 10px; padding: 4px 14px; margin: 0 0 12px;
         color: var(--ink-2); font-size: 13px; }
.intro summary { font-weight: 650; color: var(--ink); font-size: 13px;
                 padding: 8px 0; cursor: pointer; }
.intro ul { margin: 6px 0 10px; padding-left: 20px; }
.intro li { margin: 3px 0; }
.intro .gloss { display: grid; grid-template-columns: max-content 1fr;
                gap: 3px 12px; margin: 6px 0 12px; }
.intro .gloss b { color: var(--ink); font-weight: 600; white-space: nowrap; }
.section-head { font-size: 11px; font-weight: 650; letter-spacing: 0.08em;
                text-transform: uppercase; color: var(--muted);
                margin: 16px 2px 4px; }
.lane .hint { font-size: 11.5px; color: var(--muted); margin: 0 2px 2px; }
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
/* glossary terms: dotted underline, definition on hover/tap */
.term { text-decoration: underline dotted var(--muted);
        text-underline-offset: 2.5px; cursor: help; }
svg text.hasdef { text-decoration: underline dotted; cursor: help; }
#tooltip .tt-d { color: var(--ink-2); max-width: 300px; }
#tooltip .tt-d b { color: var(--ink); }
/* auto-detected findings */
.findings { display: flex; flex-direction: column; gap: 0; }
.finding { display: flex; gap: 10px; align-items: flex-start;
           padding: 9px 6px; border-top: 1px solid var(--grid);
           cursor: pointer; }
.finding:first-child { border-top: none; }
.finding:hover { background: var(--band); }
.finding.isnew { background: var(--bandc);
                 border-left: 3px solid var(--critical); padding-left: 8px; }
.finding .fsev { flex: none; width: 9px; height: 9px; border-radius: 50%;
                 margin-top: 5px; }
.finding .fmain { flex: 1; min-width: 0; }
.finding .ftext { color: var(--ink-2); }
.finding .fwhen { flex: none; color: var(--muted); font-size: 11.5px;
                  font-variant-numeric: tabular-nums; white-space: nowrap;
                  margin-top: 1px; }
.finding .zoomto { color: var(--s1); font-size: 11.5px; white-space: nowrap; }
.findings-none { color: var(--muted); font-size: 12.5px; padding: 2px; }
.newbadge { display: inline-block; font-size: 10.5px; font-weight: 700;
            letter-spacing: 0.05em; text-transform: uppercase;
            color: var(--surface); background: var(--critical);
            border-radius: 5px; padding: 1px 6px; margin-right: 6px;
            vertical-align: 1px; }
.catchip { display: inline-block; margin-top: 5px; font-size: 11.5px;
           color: var(--s1); cursor: pointer; user-select: none; }
.catchip:hover { text-decoration: underline; }
.catpanel { margin: 6px 0 2px; padding: 8px 10px; border-left: 2px solid
            var(--border); background: var(--band); border-radius: 0 6px 6px 0;
            font-size: 12px; color: var(--ink-2); }
.catpanel .catmech { margin-bottom: 6px; }
.catpanel .catstatus { color: var(--muted); font-size: 11.5px; }
/* event-log severity filter chips */
.evchips { margin-left: auto; display: flex; gap: 6px; flex-wrap: wrap; }
.evchip { font-size: 11.5px; color: var(--muted); border: 1px solid
          var(--border); border-radius: 999px; padding: 2px 9px;
          cursor: pointer; white-space: nowrap; user-select: none; }
.evchip.on { color: var(--ink); border-color: var(--s1); }
.evchip .sev { margin-right: 5px; }
</style>
</head>
<body>
<div class="report">
  <header>
    <h1>__TITLE__</h1>
    <span class="chips" id="chips"></span>
    <button id="themebtn" type="button">theme: auto</button>
  </header>
  <details class="intro">
    <summary>How to read this report</summary>
    <p><strong>The spacecraft in one minute.</strong> The physics layer
      holds the truth — orbit, attitude, battery, temperatures. No
      subsystem sees it: they read noisy sensors and one-tick-stale bus
      traffic, and each pursues its own local objective. The EPS
      protects the battery (load shed), the OBC protects the mission
      (safe mode, FDIR), the ADCS points the panel at the sun (that's
      where the watts come from), thermal keeps the battery warm enough
      to charge, the payload fills the data queue, and the ground —
      seeing only what a pass lets through — commands the payload from
      hours-stale telemetry. Every protection is locally sensible;
      everything interesting in this report is what happens when they
      interact.</p>
    <ul>
      <li><strong>What happened</strong> lists the flight's story:
        signatures the report auto-detected by running the
        emergent-behavior catalog's known mechanisms against this
        recording. Click one to zoom every chart to its evidence and
        read it yourself.</li>
      <li><strong>Top to bottom:</strong> headline numbers, the findings,
        the orbit replay, on/off state channels, discrete events, then
        continuous telemetry lanes. Everything shares one time axis,
        measured in orbits (~94 min each).</li>
      <li><strong>Gray bands</strong> are eclipse (no sun, no power
        generation). <strong>Blue bands</strong> are ground-station
        contact — the only minutes when data goes down or commands go
        up. Everything the ground knows and does happens inside the
        blue.</li>
      <li><strong>Hover anywhere</strong> for a crosshair across every
        chart with exact values. If the orbit replay is paused, the
        satellite follows your crosshair.</li>
      <li><strong>Zoom &amp; pan:</strong> drag horizontally on any chart
        to zoom every chart to that window, or use the
        <strong>&minus; + &#9664; &#9654; reset</strong> controls by the
        telemetry heading; <strong>double-click</strong> a chart to zoom
        back out. The readout and the x-axis show the window in both
        orbits and elapsed mission time (h:mm:ss).</li>
      <li>Event glyphs: <strong>▲ critical</strong>, <strong>◆
        warning</strong>, <strong>● recovery/good</strong>, ○ neutral.
        The full list with details is in the event log below.</li>
    </ul>
    <p style="margin:2px 0 6px">Dotted-underlined terms anywhere in the
      report show their definition on hover (tap on a phone); event
      names in the log do the same. The full glossary:</p>
    <div class="gloss" id="gloss"></div>
  </details>
  <section class="tiles" id="tiles"></section>
  <div class="card" id="findingscard">
    <div class="card-head"><h2>What happened</h2>
      <span class="zinfo">auto-detected from the flight — click one to zoom
        the charts to it</span></div>
    <div class="findings" id="findings"></div></div>
  <div class="card" id="orbitcard"><h2>Orbit</h2><div id="orbit"></div></div>
  <div class="card"><h2>State channels</h2><div id="xaxis-top"></div>
    <div id="tracks"></div></div>
  <div class="card">
    <div class="card-head"><h2>Events</h2>
      <span class="zinfo" id="evcount"></span></div>
    <div id="events"></div></div>
  <div class="card">
    <div class="card-head"><h2>Telemetry</h2><span class="zinfo" id="zinfo"></span>
      <span class="zoomctl" id="zoomctl">
        <button data-z="out" type="button" title="zoom out">&minus;</button>
        <button data-z="in" type="button" title="zoom in">+</button>
        <button data-z="panL" type="button" title="pan earlier">&#9664;</button>
        <button data-z="panR" type="button" title="pan later">&#9654;</button>
        <button id="zoomreset" type="button" title="reset to full flight">reset</button>
      </span></div>
    <div id="lanes"></div><div id="xaxis"></div></div>
  <div class="card">
    <div class="card-head"><h2>Event log</h2>
      <span class="zinfo">hover an event name for what it means</span>
      <span class="evchips" id="evchips"></span></div>
    <div id="evtable"></div></div>
</div>
<div id="tooltip"></div>
<script type="application/json" id="flight-data">__FLIGHT_JSON__</script>
<script>
"use strict";
var DATA = JSON.parse(document.getElementById("flight-data").textContent);
var PERIOD = DATA.meta.period_s, T_END = Math.max(DATA.meta.duration_s, 1);
var VIEW = { t0: 0, t1: T_END };  // zoom window; survives rebuilds
var PADL = 112, PADR = 14;        // label gutter; recomputed per build
var SERIES = ["var(--s1)", "var(--s2)", "var(--s3)", "var(--s4)"];
var SEV = { critical: "var(--critical)", warning: "var(--warning)",
            good: "var(--good)", neutral: "var(--muted)" };
var NS = "http://www.w3.org/2000/svg";
var charts = [];   // {svg, plotW, update(tOrNull)}
var orbitUI = null;  // orbit view state; survives chart rebuilds

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
function pad(n) { return (n < 10 ? "0" : "") + n; }
/* elapsed mission time: H:MM:SS, dropping the hours field under an hour */
function hms(t) {
  var s = Math.round(t), h = Math.floor(s / 3600),
      m = Math.floor(s / 60) % 60, ss = s % 60;
  return (h ? h + ":" + pad(m) : m) + ":" + pad(ss);
}
/* coarser H:MM for the window readout */
function hm(t) {
  var s = Math.round(t);
  return Math.floor(s / 3600) + ":" + pad(Math.floor(s / 60) % 60);
}
function niceStep(span, n) {
  var raw = span / n, mag = Math.pow(10, Math.floor(Math.log10(raw)));
  var r = raw / mag;
  return (r >= 5 ? 10 : r >= 2 ? 5 : r >= 1 ? 2 : 1) * mag;
}
function viewSpan() { return VIEW.t1 - VIEW.t0; }
function xOf(t, plotW) {
  return PADL + ((t - VIEW.t0) / viewSpan()) * plotW;
}
function orbitStep() {
  var span = viewSpan() / PERIOD;
  return span > 6 ? 1 : span > 2.5 ? 0.5 : span > 1.2 ? 0.25 : 0.1;
}

function laneWidth() {
  return document.getElementById("lanes").clientWidth;
}

/* truncate an SVG text node to a pixel budget, with an ellipsis; the
 * tooltip always carries the full name */
function fitText(node, maxW) {
  var s = node.textContent;
  while (s.length > 3 && node.getComputedTextLength() > maxW) {
    s = s.slice(0, -1);
    node.textContent = s + "…";
  }
}

/* ---- glossary: every term of art teaches itself on hover ----
   One dictionary (DATA.gloss) feeds the primer grid, inline .term spans
   wrapped around matches in visible text, and the row labels of the
   state strip / event timeline. Event kinds get their own dictionary
   (DATA.evgloss) in the event log. */
var GLOSS = DATA.gloss || {}, EVGLOSS = DATA.evgloss || {};
var ALIAS = { "ground contact": "pass" };
var GLOSS_LC = {};
function normTerm(s) { return String(s).toLowerCase().replace(/-/g, " "); }
Object.keys(GLOSS).forEach(function (k) { GLOSS_LC[normTerm(k)] = k; });
function defFor(name) {
  var lc = normTerm(name);
  if (ALIAS[lc]) lc = normTerm(ALIAS[lc]);
  var k = GLOSS_LC[lc];
  return k ? { name: k, def: GLOSS[k] } : null;
}
function termRegex(keys, flags) {
  keys = keys.slice().sort(function (a, b) { return b.length - a.length; });
  var alts = keys.map(function (k) {
    return k.replace(/[.*+?^${}()|[\]\\]/g, "\\$&").replace(/[ -]/g, "[ -]");
  });
  // leading boundary is a captured group (no lookbehind for reach);
  // an optional trailing "s"/"es" lets "SEUs" and "passes" resolve
  return new RegExp("(^|[^A-Za-z0-9_-])(" + alts.join("|") +
                    ")((?:es|s)?)(?![A-Za-z0-9_-])", flags);
}
var _acr = [], _phr = [];
Object.keys(GLOSS).forEach(function (k) {
  (/[A-Z]/.test(k) ? _acr : _phr).push(k);
});
var reACR = _acr.length ? termRegex(_acr, "") : null;      // case matters
var rePHR = _phr.length ? termRegex(_phr, "i") : null;     // it doesn't

function glossifyNode(textNode) {
  var s = textNode.nodeValue, out = [], pos = 0, hits = 0;
  while (pos < s.length) {
    var rest = s.slice(pos), bm = null, bat = Infinity;
    [reACR, rePHR].forEach(function (re) {
      if (!re) return;
      var m = rest.match(re);
      if (m && m.index + m[1].length < bat) {
        bat = m.index + m[1].length; bm = m;
      }
    });
    if (!bm) break;
    var start = pos + bat, len = bm[2].length + bm[3].length;
    var d = defFor(bm[2]);
    if (d) {
      out.push(s.slice(pos, start));
      out.push({ text: s.slice(start, start + len), d: d });
      hits++;
    } else {
      out.push(s.slice(pos, start + len));
    }
    pos = start + len;
  }
  if (!hits) return;
  out.push(s.slice(pos));
  var frag = document.createDocumentFragment();
  out.forEach(function (o) {
    if (typeof o === "string") {
      if (o) frag.appendChild(document.createTextNode(o));
      return;
    }
    var sp = document.createElement("span");
    sp.className = "term"; sp.textContent = o.text;
    sp.dataset.name = o.d.name; sp.dataset.def = o.d.def;
    frag.appendChild(sp);
  });
  textNode.parentNode.replaceChild(frag, textNode);
}
function glossify(root) {
  if (!root) return;
  var walker = document.createTreeWalker(root, NodeFilter.SHOW_TEXT, null);
  var nodes = [];
  while (walker.nextNode()) {
    var p = walker.currentNode.parentNode;
    if (p.closest && p.closest(".term,.gloss,.evkind,.catchip,script,style")) continue;
    nodes.push(walker.currentNode);
  }
  nodes.forEach(glossifyNode);
}
var termActive = false;
function showTermTip(ev, name, def) {
  tooltip.textContent = "";
  div("tt-t", tooltip).textContent = name;
  div("tt-d", tooltip).textContent = def;
  tooltip.style.display = "block";
  placeTip(ev);
}
document.addEventListener("mouseover", function (ev) {
  var t = ev.target.closest && ev.target.closest(".term");
  if (t) { termActive = true; showTermTip(ev, t.dataset.name, t.dataset.def); }
});
document.addEventListener("mouseout", function (ev) {
  var t = ev.target.closest && ev.target.closest(".term");
  if (t) { termActive = false; tooltip.style.display = "none"; }
});
document.addEventListener("click", function (ev) {  // touch: tap toggles
  var t = ev.target.closest && ev.target.closest(".term");
  if (t) { termActive = true; showTermTip(ev, t.dataset.name, t.dataset.def); }
  else if (termActive) { termActive = false; tooltip.style.display = "none"; }
});
/* SVG row labels can't hold .term spans; give the whole label a hover */
function svgDef(node, label) {
  var d = defFor(label);
  if (!d) return;
  node.classList.add("hasdef");
  node.addEventListener("pointerenter", function (ev) {
    termActive = true; showTermTip(ev, d.name, d.def);
  });
  node.addEventListener("pointerleave", function () {
    termActive = false; tooltip.style.display = "none";
  });
}

/* context shading behind every plot area: gray = eclipse, blue = the
   minutes of ground-station contact — the only time the link exists */
function drawBands(svg, plotW, h) {
  [["eclipse", "var(--band)"],
   ["ground contact", "var(--bandc)"]].forEach(function (spec) {
    DATA.tracks.forEach(function (tr) {
      if (tr.label !== spec[0]) return;
      tr.intervals.forEach(function (iv) {
        if (iv[1] < VIEW.t0 || iv[0] > VIEW.t1) return;
        var x0 = Math.max(PADL, xOf(iv[0], plotW));
        var x1 = Math.min(PADL + plotW, xOf(iv[1], plotW));
        el("rect", { x: x0, y: 0, width: Math.max(1, x1 - x0),
                     height: h, fill: spec[1] }, svg);
      });
    });
  });
}

function orbitGrid(svg, plotW, h) {
  var step = orbitStep();
  var o0 = Math.ceil(VIEW.t0 / PERIOD / step) * step;
  for (var o = o0; o * PERIOD <= VIEW.t1 + 1e-9; o += step) {
    if (o <= 0) continue;
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
    t.textContent = tr.label + " ×" + tr.intervals.length;
    fitText(t, PADL - 12);
    svgDef(t, tr.label);
    tr.intervals.forEach(function (iv) {
      if (iv[1] < VIEW.t0 || iv[0] > VIEW.t1) return;
      var x0 = Math.max(PADL, xOf(iv[0], plotW));
      var x1 = Math.min(PADL + plotW, xOf(iv[1], plotW));
      el("rect", { x: x0, y: y, width: Math.max(1.5, x1 - x0),
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
    fitText(t, PADL - 12);
    svgDef(t, s);
  });
  DATA.events.forEach(function (e) {
    if (e.t < VIEW.t0 || e.t > VIEW.t1) return;
    glyph(svg, xOf(e.t, plotW), sources.indexOf(e.source) * ROW + 12.5, e.severity);
  });
  attachCrosshair(svg, plotW, H, function (tt, tHover) {
    var win = viewSpan() / 90;
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
  if (lane.hint) {
    var hint = div("hint", host);
    hint.textContent = lane.hint;
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

/* shared x axis, drawn above the state strip and below the lanes */
function drawXAxis(hostId) {
  var host = document.getElementById(hostId);
  host.textContent = "";
  var W = laneWidth(), H = 33, plotW = W - PADL - PADR;
  var svg = el("svg", { width: W, height: H }, host);
  var step = orbitStep();
  var decimals = step >= 1 ? 0 : step >= 0.5 ? 1 : 2;
  var o0 = Math.ceil(VIEW.t0 / PERIOD / step) * step;
  var lastSub = -1e9;  // elapsed labels are wider — thin them when crowded
  for (var o = o0; o * PERIOD <= VIEW.t1 + 1e-9; o += step) {
    var x = xOf(o * PERIOD, plotW);
    if (x > PADL + plotW + 1) break;
    el("line", { x1: x, y1: 0, x2: x, y2: 5, stroke: "var(--axis)",
                 "stroke-width": 1 }, svg);
    var t = el("text", { x: x, y: 16, "text-anchor": "middle" }, svg);
    t.textContent = o.toFixed(decimals);
    if (x - lastSub >= 46) {
      var sub = el("text", { x: x, y: 29, "text-anchor": "middle",
                             "class": "axsub" }, svg);
      sub.textContent = hms(o * PERIOD);
      lastSub = x;
    }
  }
  el("text", { x: PADL + plotW, y: 16, "text-anchor": "end" }, svg)
    .textContent = "orbits";
  el("text", { x: PADL + plotW, y: 29, "text-anchor": "end", "class": "axsub" },
     svg).textContent = "elapsed";
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
  var range = document.createElement("input");
  range.type = "range"; range.min = 0; range.max = T_END;
  range.step = Math.max(1, T_END / 2000); bar.appendChild(range);
  var orbChip = div("chip", bar), eclChip = div("chip", bar),
      conChip = div("chip", bar);

  var reduced = window.matchMedia &&
      matchMedia("(prefers-reduced-motion: reduce)").matches;
  // view state lives outside this function so a rebuild (resize, mobile
  // URL-bar show/hide) never snaps the globe back to its default pose
  // or restarts the clock
  if (!orbitUI) {
    orbitUI = { yaw: -0.9, pitch: 0.38, oT: 0, playing: !reduced,
                speed: "300" };
  }
  var vs = orbitUI;
  var last = null, raf = null;
  var cx = W / 2, cy = H / 2;
  var s = (Math.min(W, H) / 2 - 8) / 1.32;
  var PERIOD_O = 2 * Math.PI / O.n_rad_s;

  speedSel.value = vs.speed;

  /* view: Rz(yaw) then Rx(pitch); screen x right, ECI north up,
     depth d > 0 faces the viewer */
  function rot(v) {
    var c = Math.cos(vs.yaw), sn = Math.sin(vs.yaw);
    var x = v[0] * c - v[1] * sn, y = v[0] * sn + v[1] * c, z = v[2];
    var cp = Math.cos(vs.pitch), sp = Math.sin(vs.pitch);
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
    var gmst = O.gmst0_rad + O.w_earth_rad_s * vs.oT;
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
    var t0ring = Math.floor(vs.oT / PERIOD_O) * PERIOD_O;
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
      var p = P(siteEci(site.lat, site.lon, vs.oT));
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
    var sp = P(satPos(vs.oT));
    ctx.lineWidth = 2; ctx.strokeStyle = s1;
    var TRAIL = Math.min(vs.oT, PERIOD_O * 0.22);
    for (var j = 0; j < 24; j++) {
      var u0 = vs.oT - TRAIL * (1 - j / 24),
          u1 = vs.oT - TRAIL * (1 - (j + 1) / 24);
      var qa = P(satPos(u0)), qb = P(satPos(u1));
      if (occluded(qa) || occluded(qb)) continue;
      ctx.globalAlpha = 0.06 + 0.5 * (j / 24);
      ctx.beginPath(); ctx.moveTo(qa.x, qa.y); ctx.lineTo(qb.x, qb.y);
      ctx.stroke();
    }
    ctx.globalAlpha = 1;
    if (trackOn("ground contact", vs.oT)) {
      var stn = O.sites[0], gp = P(siteEci(stn.lat, stn.lon, vs.oT));
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

    orbChip.textContent = "orbit " + (vs.oT / PERIOD_O).toFixed(2);
    eclChip.textContent = trackOn("eclipse", vs.oT) ? "eclipse" : "sunlit";
    eclChip.className = "chip" + (trackOn("eclipse", vs.oT) ? "" : " on");
    conChip.textContent = trackOn("ground contact", vs.oT)
      ? "in contact" : "no contact";
    conChip.className = "chip" + (trackOn("ground contact", vs.oT) ? " on" : "");
  }

  function frame(ts) {
    raf = null;
    if (last === null) last = ts;
    var dt = (ts - last) / 1000; last = ts;
    if (vs.playing) {
      vs.oT += dt * parseFloat(speedSel.value);
      if (vs.oT > T_END) vs.oT = 0;
      range.value = vs.oT;
    }
    render();
    if (vs.playing) raf = requestAnimationFrame(frame);
  }
  function setPlaying(on) {
    vs.playing = on; last = null;
    btn.textContent = on ? "Pause" : "Play";
    if (on && raf === null) raf = requestAnimationFrame(frame);
  }
  btn.addEventListener("click", function () { setPlaying(!vs.playing); });
  range.addEventListener("input", function () {
    vs.oT = parseFloat(range.value);
    if (!vs.playing) render();
  });
  speedSel.addEventListener("change", function () {
    vs.speed = speedSel.value; last = null;
  });

  var dragging = false, lx = 0, ly = 0;
  canvas.addEventListener("pointerdown", function (ev) {
    dragging = true; lx = ev.clientX; ly = ev.clientY;
    canvas.setPointerCapture(ev.pointerId);
  });
  canvas.addEventListener("pointermove", function (ev) {
    if (!dragging) return;
    vs.yaw += (ev.clientX - lx) * 0.008;
    vs.pitch = Math.max(-1.35,
                        Math.min(1.35, vs.pitch + (ev.clientY - ly) * 0.008));
    lx = ev.clientX; ly = ev.clientY;
    if (!vs.playing) render();
  });
  // a drag ends however the browser says it ends — release, or the
  // pointer being taken away (scroll gesture, leaving the viewport);
  // the pose simply stays where the finger left it
  canvas.addEventListener("pointerup", function () { dragging = false; });
  canvas.addEventListener("pointercancel", function () { dragging = false; });

  window.__orbitSeek = function (t) {
    if (!vs.playing) { vs.oT = t; range.value = t; render(); }
  };
  window.__orbitStop = function () {
    if (raf !== null) cancelAnimationFrame(raf);
    raf = null; window.__orbitSeek = null;
  };
  btn.textContent = vs.playing ? "Pause" : "Play";
  range.value = vs.oT;
  render();
  if (vs.playing) raf = requestAnimationFrame(frame);
}

/* ---- crosshair + tooltip + drag-to-zoom, shared across every chart ---- */
var tooltip = document.getElementById("tooltip");
function setView(t0, t1) {
  t0 = Math.max(0, t0);
  t1 = Math.min(T_END, t1);
  if (t1 - t0 < 30) return;  // don't zoom below half a beacon period
  VIEW = { t0: t0, t1: t1 };
  buildAll();
}
function resetView() {
  if (VIEW.t0 === 0 && VIEW.t1 === T_END) return;
  VIEW = { t0: 0, t1: T_END };
  buildAll();
}
function attachCrosshair(svg, plotW, h, fill) {
  var line = el("line", { y1: 0, y2: h, stroke: "var(--axis)",
                          "stroke-width": 1, visibility: "hidden" }, svg);
  charts.push({ svg: svg, plotW: plotW, line: line });
  var selX0 = null, selRect = null;
  function pxToT(clientX) {
    var rect = svg.getBoundingClientRect();
    return VIEW.t0 + ((clientX - rect.left) - PADL) / plotW * viewSpan();
  }
  svg.addEventListener("pointerdown", function (ev) {
    if (ev.button !== 0) return;
    selX0 = ev.clientX;
    svg.setPointerCapture(ev.pointerId);
  });
  svg.addEventListener("pointermove", function (ev) {
    var t = pxToT(ev.clientX);
    if (selX0 !== null && Math.abs(ev.clientX - selX0) > 6) {
      if (!selRect) {
        selRect = el("rect", { y: 0, height: h, fill: "var(--s1)",
                               opacity: 0.13 }, svg);
      }
      var rect = svg.getBoundingClientRect();
      var a = Math.min(selX0, ev.clientX) - rect.left;
      selRect.setAttribute("x", a);
      selRect.setAttribute("width", Math.abs(ev.clientX - selX0));
    }
    if (t < VIEW.t0 || t > VIEW.t1) { hideCross(); return; }
    charts.forEach(function (c) {
      var x = PADL + ((t - VIEW.t0) / viewSpan()) * c.plotW;
      c.line.setAttribute("x1", x); c.line.setAttribute("x2", x);
      c.line.setAttribute("visibility", "visible");
    });
    var rows = [];
    fill(rows, t);
    showTooltip(ev, t, rows);
    if (window.__orbitSeek) window.__orbitSeek(t);  // paused globe follows
  });
  svg.addEventListener("pointerup", function (ev) {
    var x0 = selX0;
    selX0 = null;
    if (selRect) { selRect.remove(); selRect = null; }
    if (x0 !== null && Math.abs(ev.clientX - x0) > 12) {
      setView(pxToT(Math.min(x0, ev.clientX)),
              pxToT(Math.max(x0, ev.clientX)));  // rebuilds every chart
    }
  });
  svg.addEventListener("pointercancel", function () {
    selX0 = null;
    if (selRect) { selRect.remove(); selRect = null; }
  });
  svg.addEventListener("dblclick", resetView);
  svg.addEventListener("pointerleave", hideCross);
}
function hideCross() {
  charts.forEach(function (c) { c.line.setAttribute("visibility", "hidden"); });
  if (!termActive) tooltip.style.display = "none";
}
function placeTip(ev) {
  var tw = tooltip.offsetWidth, th = tooltip.offsetHeight;
  var x = ev.clientX + 14, y = ev.clientY + 12;
  if (x + tw > window.innerWidth - 8) x = ev.clientX - tw - 14;
  if (y + th > window.innerHeight - 8) y = ev.clientY - th - 12;
  tooltip.style.left = x + "px"; tooltip.style.top = y + "px";
}
function showTooltip(ev, t, rows) {
  tooltip.textContent = "";
  var head = div("tt-t", tooltip);
  head.textContent = "orbit " + orbits(t).toFixed(2) + " · t=" +
                     Math.round(t) + " s · " +
                     (trackOn("eclipse", t) ? "eclipse" : "sunlit") +
                     (trackOn("ground contact", t) ? " · in contact" : "");
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
  placeTip(ev);
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
    div(t.warn ? "vl warn" : "vl", tile).textContent = t.value;
    div("nt", tile).textContent = t.note || "";
  });
  glossify(host);
}
/* auto-detected findings: the flight's story, each row a click-to-zoom
   span. Rendered once (content is view-independent); the click drives
   the same setView() the drag-zoom uses. */
function drawFindings() {
  var host = document.getElementById("findings");
  host.textContent = "";
  var notes = DATA.annotations || [];
  if (!notes.length) {
    var none = div("findings-none", host);
    none.textContent = "No known signatures fired — a clean flight, or a "
      + "new behavior the detectors don't recognize yet.";
    document.getElementById("findingscard")
      .querySelector(".zinfo").textContent = "auto-detected from the flight";
    return;
  }
  var CAT = DATA.catalog || {};
  notes.forEach(function (n) {
    var row = div("finding" + (n.new ? " isnew" : ""), host);
    var dot = div("fsev", row);
    dot.style.background = SEV[n.sev] || SEV.neutral;
    var main = div("fmain", row);
    var body = div("ftext", main);
    if (n.new) {
      var badge = document.createElement("span");
      badge.className = "newbadge"; badge.textContent = "possibly new";
      body.appendChild(badge);
    }
    body.appendChild(document.createTextNode(n.text));
    // link to the catalog entry this finding matched — the real entry
    // text, embedded, expands inline (keeps the report self-contained)
    var cat = n.entry && CAT[n.entry];
    if (cat) {
      var label = "catalog entry " + n.entry + ": " + cat.title;
      var chip = div("catchip nogloss", main);
      chip.textContent = "▸ " + label;
      var panel = div("catpanel", main);
      panel.style.display = "none";
      div("catmech", panel).textContent = cat.mechanism;
      if (cat.status) div("catstatus", panel).textContent = "status: " + cat.status;
      chip.addEventListener("click", function (ev) {
        ev.stopPropagation();
        var open = panel.style.display === "none";
        panel.style.display = open ? "" : "none";
        chip.textContent = (open ? "▾ " : "▸ ") + label;
      });
      panel.addEventListener("click", function (ev) { ev.stopPropagation(); });
    }
    var when = div("fwhen", row);
    var span = n.t1 - n.t0;
    when.textContent = span > PERIOD * 0.05
      ? "orbit " + orbits(n.t0).toFixed(2) + "–" + orbits(n.t1).toFixed(2)
      : "orbit " + orbits(n.t0).toFixed(2);
    div("zoomto", row).textContent = "zoom ›";
    row.addEventListener("click", function () {
      var pad = Math.max(span * 0.15, PERIOD * 0.5);
      setView(n.t0 - pad, n.t1 + pad);
      document.getElementById("lanes").scrollIntoView(
        { behavior: "smooth", block: "start" });
    });
  });
  glossify(host);
}
var EVFILTER = { critical: true, warning: true, good: true, neutral: true };
function drawEventTable() {
  var host = document.getElementById("evtable");
  host.textContent = "";
  if (!DATA.events.length) { host.textContent = "no events"; return; }
  var shown = DATA.events.filter(function (e) {
    return EVFILTER[e.severity];
  });
  if (!shown.length) {
    host.textContent = "no events match the severity filter";
    return;
  }
  var table = document.createElement("table");
  var tr = document.createElement("tr");
  ["orbit", "t (s)", "source", "event", "detail"].forEach(function (h) {
    var th = document.createElement("th"); th.textContent = h;
    tr.appendChild(th);
  });
  table.appendChild(tr);
  shown.forEach(function (e) {
    var row = document.createElement("tr");
    function td(txt, num) {
      var c = document.createElement("td");
      if (num) c.className = "num";
      c.textContent = txt; row.appendChild(c); return c;
    }
    td(orbits(e.t).toFixed(2), true);
    td(String(Math.round(e.t)), true);
    var sc = td("", false);
    var sd = defFor(e.source);
    if (sd) {
      var ss = document.createElement("span");
      ss.className = "term"; ss.textContent = e.source;
      ss.dataset.name = sd.name; ss.dataset.def = sd.def;
      sc.appendChild(ss);
    } else sc.textContent = e.source;
    var kc = td("", false);
    kc.className = "evkind";
    var dot = document.createElement("span");
    dot.className = "sev"; dot.style.background = SEV[e.severity];
    kc.appendChild(dot);
    var kn = document.createElement("span");
    kn.textContent = e.kind;
    if (EVGLOSS[e.kind]) {
      kn.className = "term";
      kn.dataset.name = e.kind; kn.dataset.def = EVGLOSS[e.kind];
    }
    kc.appendChild(kn);
    td(e.detail);
    table.appendChild(row);
  });
  host.appendChild(table);
  glossify(table);
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

var lastBuildWidth = 0;
function updateZoomUI() {
  var zoomed = VIEW.t0 > 0 || VIEW.t1 < T_END;
  document.getElementById("zoomreset").disabled = !zoomed;
  var span = VIEW.t1 - VIEW.t0;
  var spanTxt = span < 5400 ? Math.round(span / 60) + " min"
                            : (span / 3600).toFixed(1) + " h";
  var txt = "orbit " + orbits(VIEW.t0).toFixed(2) + "–"
          + orbits(VIEW.t1).toFixed(2) + " · elapsed "
          + hm(VIEW.t0) + "–" + hm(VIEW.t1) + " · " + spanTxt;
  document.getElementById("zinfo").textContent =
    txt + (zoomed ? "" : " · full flight");
}
function buildAll() {
  charts = [];
  lastBuildWidth = document.getElementById("lanes").clientWidth;
  // the label gutter breathes with the page: room for "ground contact"
  // on desktop, graceful truncation on a phone
  PADL = Math.min(112, Math.max(64, Math.round(lastBuildWidth * 0.15)));
  document.getElementById("lanes").textContent = "";
  drawChips(); drawTiles(); drawOrbitView(); drawTracks(); drawEvents();
  var lastSection = null;
  DATA.lanes.forEach(function (lane) {
    if (lane.section && lane.section !== lastSection) {
      lastSection = lane.section;
      div("section-head",
          document.getElementById("lanes")).textContent = lane.section;
    }
    drawLane(lane);
  });
  drawXAxis("xaxis-top"); drawXAxis("xaxis");
  drawEventTable(); updateZoomUI();
  glossify(document.getElementById("lanes"));
  glossify(document.getElementById("chips"));
}
/* one-time teaching chrome: the glossary grid, the primer text, the
   event count and severity-filter chips */
(function () {
  drawFindings();
  var grid = document.getElementById("gloss");
  Object.keys(GLOSS).forEach(function (k) {
    var b = document.createElement("b"); b.textContent = k;
    var sp = document.createElement("span"); sp.textContent = GLOSS[k];
    grid.appendChild(b); grid.appendChild(sp);
  });
  glossify(document.querySelector(".intro"));
  var counts = { critical: 0, warning: 0, good: 0, neutral: 0 };
  DATA.events.forEach(function (e) { counts[e.severity] += 1; });
  document.getElementById("evcount").textContent =
    DATA.events.length + " events" +
    (counts.critical ? " · " + counts.critical + " critical" : "");
  var chips = document.getElementById("evchips");
  ["critical", "warning", "good", "neutral"].forEach(function (sev) {
    if (!counts[sev]) return;
    var c = document.createElement("span");
    c.className = "evchip on";
    var dot = document.createElement("span");
    dot.className = "sev"; dot.style.background = SEV[sev];
    c.appendChild(dot);
    c.appendChild(document.createTextNode(sev + " " + counts[sev]));
    c.addEventListener("click", function () {
      EVFILTER[sev] = !EVFILTER[sev];
      c.className = "evchip" + (EVFILTER[sev] ? " on" : "");
      drawEventTable();
    });
    chips.appendChild(c);
  });
})();
document.getElementById("zoomreset").addEventListener("click", resetView);
/* zoom / pan buttons: zoom about the window center, pan by 30% of span.
   setView clamps to [0, T_END] and rebuilds every chart. */
function panBy(frac) {
  var span = VIEW.t1 - VIEW.t0, d = span * frac;
  if (VIEW.t0 + d < 0) d = -VIEW.t0;
  if (VIEW.t1 + d > T_END) d = T_END - VIEW.t1;
  if (d !== 0) setView(VIEW.t0 + d, VIEW.t1 + d);
}
document.getElementById("zoomctl").addEventListener("click", function (ev) {
  var z = ev.target.getAttribute("data-z");
  if (!z) return;  // the reset button (no data-z) has its own handler
  var c = (VIEW.t0 + VIEW.t1) / 2, span = VIEW.t1 - VIEW.t0;
  if (z === "in") setView(c - span / 4, c + span / 4);
  else if (z === "out") setView(c - span, c + span);
  else if (z === "panL") panBy(-0.3);
  else if (z === "panR") panBy(0.3);
});
var resizeTimer = null;
window.addEventListener("resize", function () {
  clearTimeout(resizeTimer);
  resizeTimer = setTimeout(function () {
    // mobile browsers fire resize when the URL bar shows/hides (height
    // only); rebuilding then would interrupt every drag and animation.
    // Only a real width change invalidates the layout.
    if (document.getElementById("lanes").clientWidth !== lastBuildWidth) {
      buildAll();
    }
  }, 150);
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
