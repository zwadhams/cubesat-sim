"""Flight report dashboard: recordings render to self-contained HTML."""

import json
import re

from cubesat_sim.frontend.dashboard import load_flight, render_flight, render_html
from cubesat_sim.faults import sensor_stuck
from cubesat_sim.mission import build_sim


def make_recording(tmp_path, **cfg):
    db = tmp_path / "flight.db"
    sim = build_sim(dt=5.0, seed=3, recorder_path=db, adcs_impl="none",
                    comms_impl="none", faults=[sensor_stuck(300.0, "gyro")],
                    **cfg)
    sim.run(duration=0.5 * sim.components[0].orbit.period_s)
    sim.close()
    return db


def test_load_flight_extracts_everything(tmp_path):
    data = load_flight(make_recording(tmp_path))
    lane_titles = [ln["title"] for ln in data["lanes"]]
    assert "State of charge" in lane_titles
    assert "Electrical power" in lane_titles
    for lane in data["lanes"]:  # every lane teaches: section + one-liner
        assert lane["section"] and lane["hint"]
    track_labels = [t["label"] for t in data["tracks"]]
    assert "eclipse" in track_labels
    kinds = [e["kind"] for e in data["events"]]
    assert "inject" in kinds and "fdir_adcs_power_cycle" in kinds
    # bands/tracks own these; they must not double as markers
    assert "eclipse_enter" not in kinds and "charge_inhibit_on" not in kinds
    assert any(t["label"] == "Min SoC (true)" for t in data["tiles"])
    # orbit view geometry: orthonormal in-plane basis, sane radius
    orb = data["orbit3d"]
    e1, e2 = orb["e1"], orb["e2"]
    assert abs(sum(a * b for a, b in zip(e1, e2))) < 1e-6
    assert abs(sum(a * a for a in e1) - 1.0) < 1e-6
    assert 1.05 < orb["r_orbit_re"] < 1.10  # 500 km over a 6371 km Earth
    assert orb["sites"][0]["kind"] == "station"
    assert len(orb["sites"]) == 4
    # every series point is finite JSON (json.dumps would throw on NaN
    # later; check the contract at the source)
    for lane in data["lanes"]:
        for s in lane["series"]:
            assert all(isinstance(v, float) for _, v in s["points"])


def test_render_is_self_contained_html(tmp_path):
    out = render_flight(make_recording(tmp_path))
    html = out.read_text()
    assert out.suffix == ".html"
    assert html.startswith("<!doctype html>")
    # embedded data parses back out
    m = re.search(
        r'<script type="application/json" id="flight-data">(.*?)</script>',
        html, re.S)
    payload = json.loads(m.group(1).replace("<\\/", "</"))
    assert payload["title"] == "flight"
    assert payload["lanes"]
    # self-contained: no external fetches of any kind (the SVG namespace
    # identifier is the one legitimate URL-shaped string)
    stripped = html.replace("http://www.w3.org/2000/svg", "")
    assert "http://" not in stripped and "https://" not in stripped
    assert 'src="' not in html and "@import" not in html


def test_shared_js_modules_inlined_once(tmp_path):
    """The frontend/js modules are spliced into the report exactly once
    each, and no __JS_ marker survives to the page."""
    html = render_flight(make_recording(tmp_path)).read_text()
    for name in ("glossary", "theme", "glyph"):
        assert html.count(f"cubesat shared js module: {name}") == 1
    assert "__JS_" not in html


def test_js_modules_parse(tmp_path):
    """node --check the shared modules and the whole assembled report
    script — a syntax error in a module would break both viewers."""
    import re
    import shutil
    import subprocess
    from pathlib import Path

    import pytest
    if shutil.which("node") is None:
        pytest.skip("node not installed")
    from cubesat_sim.frontend import dashboard
    js_dir = Path(dashboard.__file__).parent / "js"
    files = sorted(js_dir.glob("*.js"))
    assert len(files) == 3
    html = render_flight(make_recording(tmp_path)).read_text()
    body = re.search(r"<script>(.*)</script>", html, re.S).group(1)
    script = tmp_path / "report_script.js"
    script.write_text(body, encoding="utf-8")
    for f in files + [script]:
        subprocess.run(["node", "--check", str(f)], check=True)


def test_render_html_is_the_shared_renderer(tmp_path):
    """render_flight is a thin file-writer over render_html — the live
    console's GET /report calls render_html directly, so the two paths
    must produce the identical page."""
    db = make_recording(tmp_path)
    out = render_flight(db, tmp_path / "twin.html")
    assert out.read_text(encoding="utf-8") == render_html(load_flight(db))


def test_glossary_covers_what_renders(tmp_path):
    """The teaching layer must not have holes: every event kind, state
    channel, and event source that can appear in a report has a
    plain-language definition riding along in the payload."""
    from cubesat_sim.frontend.dashboard import EVENT_GLOSS, EVENT_SEVERITY
    # every severity-classified kind is explained
    assert set(EVENT_SEVERITY) <= set(EVENT_GLOSS)

    data = load_flight(make_recording(tmp_path))
    assert data["gloss"] and data["evgloss"]
    kinds = {e["kind"] for e in data["events"]}
    assert not kinds - set(data["evgloss"]), kinds - set(data["evgloss"])
    gloss_lc = {k.lower() for k in data["gloss"]}
    for tr in data["tracks"]:  # "ground contact" resolves via its alias
        assert tr["label"].lower() in gloss_lc | {"ground contact"}, tr["label"]
    for s in {e["source"] for e in data["events"]}:
        assert s in gloss_lc, s

    # static: every curated track label teaches itself, including the
    # ones this sample flight can't exercise (beliefs need a decoded
    # beacon, the SAA band a nonzero SEU rate)
    from cubesat_sim.frontend.dashboard import DIGITAL_TRACKS, GLOSSARY
    all_gloss_lc = {k.lower() for k in GLOSSARY}
    for _src, _key, label in DIGITAL_TRACKS:
        assert label.lower() in all_gloss_lc | {"ground contact"}, label
    assert "contact_handover" in EVENT_GLOSS  # multi-station flights emit it


def test_phase_a_curated_channels():
    """The truth-vs-measured and truth-vs-belief stories are curated:
    the noisy battery temp the controller acts on rides beside the truth,
    the ADCS's own pointing error gets a lane, generated science joins
    the Data conservation story, and the ground's stale beliefs sit as
    tracks directly beside the truths they shadow."""
    from cubesat_sim.frontend.dashboard import ANALOG_LANES, DIGITAL_TRACKS
    series = {(src, key) for *_, spec in ANALOG_LANES
              for src, key, _label, _tf in spec}
    assert ("thermal", "battery_temp_k") in series
    assert ("adcs", "sun_err_deg") in series
    assert ("payload", "generated_mb") in series
    labels = [label for _src, _key, label in DIGITAL_TRACKS]
    assert labels.index("believed SAFE") == labels.index("safe mode") + 1
    assert labels.index("believed shed") == labels.index("load shed") + 1
    assert "in SAA" in labels


def test_lane_csv_and_theme_persist(tmp_path):
    """Phase-A affordances ride in the template: every lane header
    carries a CSV download of its embedded series, and the theme choice
    survives a reload."""
    html = render_flight(make_recording(tmp_path)).read_text()
    assert "laneCsv" in html and "csvbtn" in html
    assert "cubesat-report-theme" in html


def test_channel_browser_embeds_every_channel(tmp_path):
    """B1: the report is no longer curated-only — every recorded
    (source, key) rides in the payload downsampled, so any channel can
    be charted ad hoc without going back to the .db."""
    import sqlite3
    from cubesat_sim.frontend.dashboard import CHANNEL_META, MAX_BUCKETS
    db_path = make_recording(tmp_path)
    data = load_flight(db_path)
    con = sqlite3.connect(db_path)
    recorded = set(con.execute(
        "SELECT DISTINCT source, key FROM telemetry WHERE value IS NOT NULL"))
    con.close()
    embedded = {(c["source"], c["key"]) for c in data["channels"]}
    assert embedded == recorded
    for c in data["channels"]:
        assert len(c["points"]) <= 2 * MAX_BUCKETS
        assert all(v is not None for _, v in c["points"])
        # known channels carry their unit and teaching hint
        if f"{c['source']}/{c['key']}" in CHANNEL_META:
            assert c["hint"]


def test_channel_meta_covers_the_curated_channels():
    """Every channel a curated lane or track charts must also be
    explained in the browser; meta entries are well-formed (a hint is
    mandatory, a unit may be blank — quaternions are unitless)."""
    from cubesat_sim.frontend.dashboard import (
        ANALOG_LANES, CHANNEL_META, DIGITAL_TRACKS)
    for *_, spec in ANALOG_LANES:
        for src, key, _label, _tf in spec:
            assert f"{src}/{key}" in CHANNEL_META, (src, key)
    for src, key, _label in DIGITAL_TRACKS:
        assert f"{src}/{key}" in CHANNEL_META, (src, key)
    for cid, (unit, hint) in CHANNEL_META.items():
        assert "/" in cid and hint, cid


def test_channel_browser_in_template(tmp_path):
    """The browser card ships in the rendered page: chips, ad-hoc lane
    plumbing, and the export-all button."""
    html = render_flight(make_recording(tmp_path)).read_text()
    assert "All channels" in html and "chanchip" in html
    assert "drawAdhoc" in html and "allChannelsCsv" in html


def test_lane_table_twin_in_template(tmp_path):
    """B2: every lane — curated and ad-hoc alike, both render through
    drawLane — can twin as a table of the current zoom window, capped
    with a zoom-to-narrow hint; the toggle survives rebuilds."""
    html = render_flight(make_recording(tmp_path)).read_text()
    assert "laneTable" in html and "TABLED" in html
    assert "TABLE_MAX_ROWS" in html and "zoom to narrow" in html


def test_annotations_detect_the_confident_corpse():
    """The entry-10 signature: FDIR gives up, and afterward the ADCS
    believes it is calm (rate estimate below the mode gate) while the
    truth tumbles past 2 deg/s. The detector must classify it as entry
    10, not the generic giveup."""
    from cubesat_sim.frontend.dashboard import _annotations
    period = 5560.0
    tg = 5.35 * period
    # frozen estimate at 0.02 (with cross-language float wobble), truth
    # spun up to ~7.8 by the undamped controller
    est = [(tg + i, 0.0231 + (i % 3) * 1e-7) for i in range(0, 2000, 50)]
    true = [(tg + i, 7.8 * i / 1950.0) for i in range(0, 2000, 50)]
    evs = [(tg, "obc", "fdir_giveup", {"cycles_used": 3})]
    notes = _annotations(evs, {}, {"rate_est": est, "rate_true": true},
                         tg + 2000, period)
    corpse = [n for n in notes if "entry 10" in n["text"]]
    assert len(corpse) == 1
    assert corpse[0]["sev"] == "critical"
    assert corpse[0]["entry"] == 10          # tagged for the catalog link
    assert abs(corpse[0]["t0"] - tg) < 1.0  # anchored at the giveup
    assert corpse[0]["t1"] >= tg + 1900     # spans to (near) the end
    assert "7.8 deg/s" in corpse[0]["text"]  # the pumped truth is reported
    # the catalog explains this flight, so it is NOT flagged as new
    assert not any(n.get("new") for n in notes)


def test_annotations_flag_uncatalogued_distress():
    """A flight that goes off-nominal but matches no catalogued mechanism
    is flagged 'possibly new' and sorted to the top; one the catalog
    explains is not."""
    from cubesat_sim.frontend.dashboard import _annotations
    period = 5560.0
    # a brownout with nothing the detectors recognize -> possibly new
    notes = _annotations([(3.0 * period, "physics", "brownout", {})],
                         {}, {}, 5 * period, period)
    new = [n for n in notes if n.get("new")]
    assert len(new) == 1
    assert new[0] is notes[0]                  # surfaced at the top
    assert "browned out" in new[0]["text"]
    assert "EMERGENT_BEHAVIORS" in new[0]["text"]

    # the same brownout, now explained by the entry-6 one-way door
    # (shed latched to the end while the panel faces anti-sun): not new
    shed = {"load shed": [[3.0 * period, 5 * period]]}
    facing = {"sun_facing": [(3.2 * period + i, -0.5) for i in range(0, 5000, 500)]}
    notes2 = _annotations([(3.0 * period, "physics", "brownout", {})],
                          shed, facing, 5 * period, period)
    assert any(n.get("entry") == 6 for n in notes2)
    assert not any(n.get("new") for n in notes2)


def test_parse_catalog_reads_the_markdown():
    """The findings' catalog links carry the real EMERGENT_BEHAVIORS.md
    text, embedded so the report stays self-contained."""
    from cubesat_sim.frontend.dashboard import parse_catalog
    cat = parse_catalog()
    assert "10" in cat and "6" in cat
    assert "confident corpse" in cat["10"]["title"]
    assert cat["10"]["mechanism"] and cat["10"]["status"]
    # every entry a detector can emit must resolve to real catalog text
    for entry in ("1", "3", "6", "7", "8", "9", "10", "11", "12"):
        assert cat.get(entry, {}).get("title"), entry


def test_annotations_ground_veto_and_clean_flight():
    """Entry 11 (the ground veto that starves the mission) fires when a
    payload disable latches over unimaged target passes; a flight with
    no signatures yields an empty list."""
    from cubesat_sim.frontend.dashboard import _annotations
    period = 5560.0
    td = 4.25 * period
    tracks = {"target visible": [[td + 500, td + 700], [td + period, td + period + 200]],
              "imaging": []}
    evs = [(td, "ground", "operator_disable_payload", {"reason": "power"})]
    notes = _annotations(evs, tracks, {}, td + 3 * period, period)
    veto = [n for n in notes if "entry 11" in n["text"]]
    assert len(veto) == 1 and veto[0]["sev"] == "warning"
    assert "2 target pass" in veto[0]["text"]

    assert _annotations([], {}, {}, 10 * period, period) == []


def test_downsampling_keeps_spikes():
    from cubesat_sim.frontend.dashboard import _downsample
    points = [(float(i), 0.0) for i in range(10000)]
    points[7777] = (7777.0, 99.0)  # one SEU-like spike
    out = _downsample(points, max_buckets=300)
    assert len(out) <= 610
    assert any(v == 99.0 for _, v in out)  # min/max bucketing kept it
