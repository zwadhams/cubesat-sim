# Project review — 2026-07-19

A full-project review with a deep focus on the GUI (both viewers), done at the
owner's request. Part 1 is the general health check; Part 2 is the GUI
improvement roadmap — the owner's stated priorities: **readability first**, but
with ways to get *all* the data shown should we want it. Roadmap items are
independently landable across future sessions.

Reviewed at HEAD `1ff3939` (multi-ground-station support). Uncommitted at review
time: the tests for the two newest features (`tests/test_live.py` attitude
quaternion, `tests/test_phase4_integration.py` second station) — worth committing
alongside this doc.

---

## Part 1 — whole-project health

### Strengths (genuinely unusual for a project this size)

- **Documentation discipline.** Every module opens with a *why* docstring;
  inline comments cite catalog entries; README / CLAUDE.md / EMERGENT_BEHAVIORS.md
  / examples are all accurate against the code post-refactor (zero stale paths).
  Zero TODO/FIXME markers anywhere.
- **Port-equivalence testing is the crown jewel.** Bit-identical full-log `==`
  across four languages, including the `1.0`-vs-`1` formatting trap, with
  graceful skips when toolchains are absent.
- **Uniform defensive posture.** "Never leave the bus hanging" holds in all four
  languages; output saturation everywhere; the bridge quarantines garbage.
- **Determinism touches**: `stream()` name-hashing, recording unlink on reuse,
  `allow_nan=False`, clamp-before-round in `_sat16`.
- **Hygiene**: clean src-layout, tight `.gitignore`, no binaries/dbs/HTML in git.

### Risks / future improvements (non-GUI, roughly ordered)

1. **No CI.** Port parity is load-bearing but only checked when someone runs
   pytest locally. A GitHub Actions workflow (apt gcc/g++ + rustup + make ×3 +
   cargo build + pytest) would lock it in. The suite is ~4 min plus builds —
   well inside free-tier limits.
2. **`.claude/` is untracked but NOT gitignored**, and it contains a full locked
   worktree copy of the repo (`closeup-attitude-view` pinned to `ccd7b76`).
   A `git add -A` would stage a second complete repo. Add `.claude/` to
   `.gitignore`; consider `git worktree remove` on the stale worktree.
3. **Frontend JS is untested beyond string checks.** ~4.7k lines of embedded
   JS across the two templates with no syntax gate. Roadmap item B4 addresses
   this (shared `.js` files + `node --check` in pytest, skipped when node is
   absent — `apt install nodejs` recommended).
4. **Hand-rolled C/C++ JSON scanners** (`strstr`/`strtod`, exact-substring
   keyed) are wire-format-fragile; the equivalence suite is the only net.
   Accepted trade-off (idiomatic bare-metal style) — but any wire-format change
   must run the full suite, never a subset.
5. **`spacecraft.py step()`** is a ~250-line high-fan-in hotspot. Natural for
   the truth authority, but worth splitting into named phases (orbit / RF /
   thermal / power / sensors) next time it grows.
6. **Naming collision**: `environment/groundstation.py` (`GroundSite`, geometry)
   vs `ground/station.py` (`GroundStation`, ops component). A rename or module
   docstring cross-reference would spare future confusion.
7. **Packaging**: no `[project.scripts]` entry points (everything is
   `python -m` — coherent, but a `cubesat-sim` command would be friendlier);
   lower-bound-only pins (consider `numpy<3`).
8. Small fixes: README.md:190 "works whichever" → "works with whichever";
   `runs/` is 3.3 GB local (the lean recording mode already on the CLAUDE.md
   backlog is the real fix).

---

## Part 2 — GUI roadmap

### Where the two viewers stand

The shared design language, teaching layer (81-term glossary, primer, lane
captions, narrated findings with click-to-zoom evidence), synchronized
crosshair, and spike-preserving min/max downsampling are the project's GUI
strengths — everything below builds on them rather than replacing them.

The structural problems, verified in exploration:

- **Data-access asymmetry.** The static report is curated-only: 11 lanes,
  9 tracks, tiles — no way to see any other channel, no export, no table view
  for lanes (the docstring's "every chart has a table-view twin" is currently
  false — only the event log has one). The live console exposes *everything*
  (all-telemetry table, bus monitor with click-to-tail, link decode) but with
  no history: lanes roll over a 1.5-orbit window, everything else is
  latest-value-only.
- **14 recorded telemetry channels are charted by NEITHER viewer** (list at
  the end), plus the noisy sensor words (`sensors/adcs/gyro|mag|sun`…) that
  catalog findings #8/#9/#10 hinge on exist only in the messages table.
- **Template duplication is diverging.** Glossary engine, findings renderer,
  theme, severity maps, and the whole orbit globe are copy-pasted between the
  two ~1500–1900-line raw-string templates; live's globe has coastlines /
  gradient terminator / geometric shadow / subsolar point / SAA chip while the
  report's still darkens the anti-sun half.
- **Campaign results have no visualization at all** — 17 metrics per flight,
  ASCII table only.
- Live-console readability: 9 stacked panels with weak grouping; ticker
  severity is a color-only dot (report uses CVD-safe glyphs ▲◆●○); findings
  card layout-shifts in; the ticker doesn't apply `SKIP_EVENT_KINDS`.

### Measured facts that shape the design

- Embedding **every** channel of a 19 h / 183 MB / 43-channel recording,
  downsampled through the existing `_downsample` (MAX_BUCKETS=600), costs
  **0.54 MB** of JSON. Reports are 0.3–0.45 MB today → a complete channel
  browser takes them to ~1 MB. Downsampling caps this regardless of flight
  length. So: embed everything.
- Live late-joiners backfill only one lane window, so any client-side "since
  launch" accumulator would be silently wrong — history must come from the
  server. The server is `ThreadingHTTPServer` and live recordings run WAL, so
  a slow read-only report render cannot block SSE.

### Phase A — quick wins (each S-sized, independent, viewer-side only)

> **Landed 2026-07-19** (same session as this review; owner commits). Two
> deltas from the spec below: `contact_handover` went into `EVENT_GLOSS`
> only (`EVENT_SEVERITY` lists only non-neutral kinds, and a handover is
> correctly neutral), and the belief-track labels shipped as "believed
> SAFE" / "believed shed" — the long forms truncate in the SVG track
> gutter; the glossary tooltips name the ground as the believer. New nit
> spotted during verification, for a future pass: near-flat lanes repeat
> the same y-tick label (e.g. "20.0" three times on Battery capacity) —
> the JS `fmt()` needs adaptive precision on narrow domains.

- **A1. Correctness & teaching patches** ✅ *landed 2026-07-19* (`dashboard.py`):
  `contact_handover` added to `EVENT_SEVERITY` + `EVENT_GLOSS` (currently
  renders neutral, no tooltip); `n_safe` tile counts via the parsed detail
  (`detail.get("to") == "SAFE"`, same predicate montecarlo uses) instead of
  substring-matching the formatted string; theme toggle persisted to
  localStorage (copy live's pattern).
- **A2. Live ticker readability** ✅ *landed 2026-07-19* (`live.py`): apply `SKIP_EVENT_KINDS` with a
  "show routine" toggle; replace the color-only severity dot with the report's
  shape glyphs; render the findings card always (quiet "no findings yet" line)
  so it stops layout-shifting; one caption line under the attitude postcard
  explaining that lit faces are real sun geometry.
- **A3. Per-lane CSV export** ✅ *landed 2026-07-19* (`dashboard.py` template only): a "CSV" button
  per lane header — Blob + `a[download]` from the already-embedded series,
  labeled `_downsampled`, with a pointer at the .db for full resolution.
- **A4. Curate the best of the uncharted channels** ✅ *landed 2026-07-19* (all additive; absent keys
  in old recordings degrade gracefully):
  - Temperatures lane += `thermal/battery_temp_k` "measured" — the untold
    truth-vs-measured story, same °C unit.
  - New "Sun pointing error" lane (Attitude): `adcs/sun_err_deg` — central to
    findings #7/#10.
  - Data lane += `payload/generated_mb` — completes the conservation story
    (generated ≈ queue + archive + dropped).
  - New digital tracks: `ground/sat_safe_mode` "ground believes SAFE" and
    `ground/sat_shedding` "ground believes shedding", placed adjacent to the
    truth tracks so the stale-belief offset is visible; `faults/in_saa`.
  - Everything else (tx_w, link_ber, sent_mb, tc_ack, carrier, frames_ok,
    telemetry_frames, fdir_cycles, q0..q3) belongs in the B1 browser, not in
    curated lanes — one-unit-per-lane and clutter rules.

### Phase B — the data-access and dedup moves

- **B1. Channel browser in the report (M).** ✅ *landed 2026-07-19* — `load_flight` embeds every
  distinct `(source,key)` downsampled (~0.5 MB). A collapsed "All channels"
  card at the page bottom: source-grouped key chips; clicking one spawns an
  ad-hoc single-series lane reusing `drawLane`/`attachCrosshair`, joining the
  shared zoom; removable; "export all CSV" button. Optional `CHANNEL_META`
  dict gives known keys units/hints; unknown keys fall back to raw names —
  future additive keys (the next q0..q3) appear with zero code changes.
- **B2. Per-lane table-view twin (S/M).** ✅ *landed 2026-07-19* — A table toggle per lane rendering
  the current zoom window (rows = t, columns = series), capped with "zoom to
  narrow". Makes the docstring true — tooltips enhance, never gate.
- **B3. `GET /report` on the live server (S/M).** ✅ *landed 2026-07-19* — Refactor `render_flight`
  into a `render_html(payload) -> str` both callers share; the live server
  renders the recording-so-far on demand; one header button "Flight report so
  far" opens it in a new tab. This is the right-sized history answer:
  late-joiner-safe, full scrub/zoom/tables for free, zero duplicated UI.
  Defer any in-page history strip unless this proves insufficient.
- **B4. JS dedup step 1 (M): glossary → theme → glyph.** ✅ *landed 2026-07-19* — Extract to
  `frontend/js/*.js`, inlined into both templates at render time via marker
  substitution (pages stay self-contained; no build step). Tests: rendered
  pages contain each module sentinel exactly once and no unreplaced markers;
  `node --check` on the js files and the extracted script body, skipif node
  missing.
- **B5. JS dedup step 2 (M/L): the globe — which is also a feature.**
  Extracting `js/globe.js` means upgrading the report to live's better globe
  (coastlines, gradient terminator, geometric shadow, subsolar point, SAA
  chip) and deleting the report's old one. Report payload gains the COASTLINE
  blob (already proven fine in the live boot). Land after B4 so the module
  infra and tests exist.
- Deliberately NOT dedup'd: the two lane renderers (SVG-zoomable vs
  canvas-rolling were never copies) and the tile mechanisms (whole-flight
  aggregates vs latest-value snapshots answer different questions — align
  labels, don't share code).

### Phase C — new surfaces

- **C1. `frontend/campaign.py` — campaign report (M).** Input: a directory of
  `flight_*.db`. A pure `summarize_db(db) -> FlightSummary` recomputes
  per-flight summaries from recordings (don't touch `run_flight` —
  determinism surface stays untouched). Output: one self-contained
  `campaign.html` in house style. First cut (the 24×12-sized version):
  - KPI tile row: flights, % nominal, worst outcome + seed, faults/SEUs,
    floor min_soc.
  - Sortable seed × metric table: outcome as shape glyph + word (CVD-safe),
    inline thin bars for counts, notes, each seed linking to its flight
    report when present.
  - SoC dumbbell per seed (min_soc → end_soc, sorted by min_soc).
  - Small-multiple SoC sparklines (~150 buckets each, ≈100 KB for 24): the
    emphasis form — off-nominal in accent, nominal in gray, click-through.
  - Second cut (defer): fault-injection timeline strip, outcome-vs-fault-type
    matrix.
- **C2. Live console information architecture (M).** No tabs — an ops console
  must surface anomalies peripherally. Four labeled zones, single scroll:
  *Mission* (clock/controls, tiles, always-rendered findings), *Spacecraft*
  (globe+attitude duo, state pills in their own card — split the current
  double-h2 card; events move out), *Telemetry* (lanes, all-telemetry table,
  the B3 report button), *Comms & commanding* (bus monitor + link feed
  collapsed-by-default with persisted open state; ticker and command panel
  stay visible). Land after A2.
- **C3. Deferred**: attitude scrub in the static report (q0..q3 is recorded;
  only meaningful for recordings that have it); `/history` endpoint +
  since-launch strip; campaign second-cut charts.

### Landing order

A1–A4 in any order → B1 → B2 → B3 → B4 → B5 → C1 → C2. Real dependencies
only: B5 and C1 want B4's module infrastructure; C2 wants A2. Everything else
is independent across sessions.

**Status (2026-07-19): A1–A4 and B1–B4 landed. Next up: B5 — the globe
extraction/upgrade — then the full test suite to wrap Phase B.**

### Appendix — recorded-but-never-charted channels and their disposition

| Channel | Disposition |
|---|---|
| `thermal/battery_temp_k` | A4: Temperatures lane, "measured" series |
| `adcs/sun_err_deg` | A4: new Sun-pointing-error lane |
| `payload/generated_mb` | A4: Data lane |
| `ground/sat_safe_mode`, `ground/sat_shedding` | A4: belief tracks beside truth tracks |
| `faults/in_saa` | A4: digital track |
| `physics/tx_w`, `physics/link_ber` | B1 browser only |
| `comms/sent_mb`, `comms/tc_ack`, `comms/carrier` | B1 browser only |
| `ground/telemetry_frames`, `ground/frames_ok` | B1 browser only |
| `obc/fdir_cycles` | B1 browser only (tile already exists) |
| `physics/q0..q3` | C3 attitude scrub (browser lines are meaningless) |
| noisy sensor bus words (`sensors/*`) | messages table only; live bus monitor reaches them — a report-side surface would need recording changes, not proposed |
