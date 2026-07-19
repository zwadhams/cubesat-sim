# cubesat-sim

A software-only cyber-physical system simulator of a CubeSat, built to study
**emergent behavior**: coupled resource loops (power, heat, momentum, data),
subsystems with local and conflicting objectives, delays, degradation, and
seeded randomness — with enough observability to catch and replay whatever
falls out.

## Architecture

- **Sim kernel** (`cubesat_sim.kernel`) — fixed-timestep clock, pub/sub message
  bus with one-tick delivery latency, per-component deterministic RNG streams,
  and a SQLite flight recorder that logs every message, telemetry sample, and
  event.
- **Environment & physics** (later phases) — orbit propagation, sun/eclipse,
  magnetic field, and physical truth models (battery, thermal nodes, attitude).
  Only this layer holds ground truth.
- **Subsystems** (later phases) — EPS, ADCS, thermal, OBC, comms, payload.
  They perceive the world only through sensors, act only through actuators,
  and talk only over the bus.
- **Polyglot flight software** — subsystems are written in the language
  their real-world counterpart would be: OBC in C (cFS heritage), EPS in
  bare-metal-style C (static buffers, no heap — PDU firmware), ADCS in
  Rust, comms in C++, payload controller and ground segment in Python.
  Compiled subsystems run as separate OS processes behind
  `RemoteComponent`, a lockstep NDJSON stdin/stdout bridge that preserves
  determinism — equivalence tests require the C OBC and C EPS to fly
  bit-identically to their Python reference implementations.

- **Faults & degradation** (`cubesat_sim.faults`) — a `FaultInjector`
  publishes `fault/*` messages that physics honors: latched (stuck)
  sensors, SEU bit flips (Poisson, rate multiplied over the South Atlantic
  Anomaly), wheel-bearing friction steps, solar-array strikes. Soft
  latch-ups clear when the ADCS rail power-cycles; hard faults are
  forever. Battery fade, array darkening, and bearing wear run
  continuously at realistic rates. The OBC carries FDIR: a gyro health
  watchdog (exact-repeat and out-of-range detection) that responds with
  ADCS power cycles on a three-attempt budget, then gives up loudly.
- **Monte Carlo** (`cubesat_sim.montecarlo`) — `sweep(seeds)` flies
  parallel campaigns with seed-deterministic random misfortunes, keeps
  every flight recording for replay, and classifies outcomes
  (NOMINAL/SAFE/SHED/FDIR_GIVEUP/BROWNED_OUT/DEAD/CRASHED).
- **Flight reports** (`cubesat_sim.frontend.dashboard`) — renders any flight
  recording into a single self-contained HTML file (no server, no
  dependencies): an animated orbit view, stat tiles, a digital state
  strip (eclipse, contact, safe mode, shedding, ...), an event timeline
  with severity glyphs, and telemetry lanes with a shared crosshair,
  eclipse shading, and table views. It teaches itself: every acronym
  and term of art grows a hover definition, the event log explains each
  event kind, and a **What happened** card runs the emergent-behavior
  catalog's signatures against the flight and writes a plain-language
  finding for each — click one to zoom every chart to its evidence, or
  expand the catalog entry it matched (the real EMERGENT_BEHAVIORS.md
  text, embedded). A flight that goes off-nominal but matches no known
  signature is flagged **possibly new** — a prompt to investigate a
  behavior the catalog hasn't seen yet. Its first render caught a wrong
  mechanism claim in catalog entry 7.
- **The space link is a real protocol** (`cubesat_sim.ccsds` +
  `cubesat_sim.linkdump`) — housekeeping telemetry crosses the channel
  as byte-true CCSDS-style TM transfer frames (sync marker, frame
  counters, space packets, CRC-16 FECF) built by the C++ flight framer
  and decoded by an independent Python ground implementation; bulk
  science moves on virtual channel 1 as frame-accounted bursts. The
  channel applies an elevation-dependent bit error rate (pass edges are
  lossy; scintillation faults crank it), the ground rejects corrupted
  frames by CRC and *sees* the losses as frame-counter gaps, and
  uplinked commands run a FARM-style ARQ loop: sequence-numbered TC
  frames retransmitted until the beacon's acceptance counter advances.
  Every beacon multiplexes one space packet per subsystem (comms 0x020,
  EPS power health 0x040, OBC mode/FDIR 0x060), so the ground's picture
  of the spacecraft — exposed as `sat_*` telemetry, frozen between
  passes — comes through the real protocol, and its operator runs a
  power-protection veto beside the storage rule.
  `python -m cubesat_sim.linkdump <recording.db>` plays protocol
  analyzer over any flight.
- **Live mission console** (`cubesat_sim.frontend.live`) — flies a mission paced
  to wall clock and serves an ops-room page in the browser: mission
  clock, the orbit globe with the sat moving in real time, live stat
  tiles and rolling telemetry lanes, a spacecraft-bus monitor (every
  topic with live payloads and rates — click one to tail it), the
  decoded space link, and an event ticker. Pause and 1–120×
  time-acceleration controls, plus a commanding panel on live flights:
  queue a real TC through the ground station's ARQ, inject faults
  mid-flight, or publish raw bus messages — everything is recorded, so
  the flight stays replayable (though `(seed, dt)` alone no longer
  reproduces a manually-commanded one). It shares the flight report's
  teaching layer — the same term tooltips, event definitions, and a
  **What happened** card the server recomputes as the flight unfolds, so
  a signature (an FDIR giveup, a shed one-way door) surfaces live the
  moment it fires. Because the server just tails the flight recording
  over SSE, `--replay` re-flies any finished recording the same way —
  view-only.

Rules of the house:

1. Ground truth lives in the physics layer; everything else sees noisy sensors.
2. Components never call each other — all coordination goes over the bus.
3. Every run is reproducible from `(seed, dt)`.

## Build phases

- [x] Phase 0 — kernel: clock, bus, components, RNG, flight recorder
- [x] Phase 1 — orbit + power (eclipse cycles breathing through the EPS)
- [x] Phase 2 — thermal (heater/battery coupling, cold-charge inhibit, death spiral)
- [x] Phase 2.5 — process bridge + OBC ported to C (bit-identical to reference)
- [x] Phase 3 — ADCS in Rust (B-dot detumble, sun pointing, momentum dump;
      attitude ↔ solar generation coupling; dipole magnetic field)
- [x] Phase 4 — data economy: payload imaging over targets, C++ comms with
      bounded downlink queue, RF channel with contact windows and packet
      loss, ground station whose operator rule closes a control loop
      through space (telemetry down -> decision -> command up)
- [x] Phase 5 — FDIR (gyro watchdog + ADCS power-cycle recovery, in both
      OBC implementations), fault injection engine (stuck sensors, SEU
      bit flips peaking over the South Atlantic Anomaly, bearing wear,
      debris strikes), continuous degradation (battery fade, array
      darkening), and a Monte Carlo campaign harness
- [x] Phase 6a — flight report dashboard (recording -> self-contained HTML,
      including the animated orbit view)
- [x] Phase 6b — the link as a real protocol: CCSDS-style TM/TC framing
      with CRC-16, frame counters and sequence gaps, elevation-dependent
      bit error rate, FARM-style command ARQ, and the linkdump analyzer
      (MQTT-style live transport deliberately avoided to keep
      byte-identical replay)
- [x] Phase 6b' — EPS ported to bare-metal-style C (bit-identical to the
      Python reference; the polyglot flight-computer map is complete)
- [ ] Phase 6c — second satellite (deferred until the single-sat setup is
      thoroughly explored)

## Getting started

**Prerequisites:** Python 3.11+, plus the toolchains for the compiled
flight software — [Rust](https://rustup.rs) (`cargo`) and a C/C++
compiler with `make` (on Debian/Ubuntu, `sudo apt install build-essential`).

**1. Install.** [uv](https://docs.astral.sh/uv/) is the easy path — it
creates the virtualenv and installs everything, no system `pip` required:

```bash
uv venv
uv pip install -e ".[dev]"
```

Prefer the standard library? `python3 -m venv .venv && .venv/bin/pip install
-e ".[dev]"` works whenever your Python ships `pip`.

**2. Build the flight software** (ADCS in Rust, comms in C++, plus C builds
of the OBC and EPS):

```bash
make -C backend/obc-c && make -C backend/eps-c && make -C backend/comms-cpp
cargo build --release --manifest-path backend/adcs-rust/Cargo.toml
```

**3. Run the tests, fly a demo, and open the report:**

```bash
.venv/bin/pytest                                      # ~4 min, all green
.venv/bin/python examples/phase6_link_demo.py         # -> runs/phase6_link.db
.venv/bin/python -m cubesat_sim.frontend.dashboard runs/phase6_link.db  # -> runs/phase6_link.html
```

Open `runs/phase6_link.html` in any browser — a self-contained flight
report. Hover any acronym for a definition, and read the **What happened**
card for auto-detected findings, each linked to the behavior catalog.
Decode the space link with
`.venv/bin/python -m cubesat_sim.linkdump runs/phase6_link.db`.

**4. Fly one live** and watch it in the browser at http://localhost:8765:

```bash
.venv/bin/python -m cubesat_sim.frontend.live --seed 19 --orbits 4 --seu-rate 6 \
    --campaign --speed 60
```

The live console is interactive: pause and change time-acceleration, and use
the **Commanding** panel to uplink real telecommands, inject faults, or
publish raw bus messages mid-flight — then watch the findings react. Re-fly
any finished recording the same way with `--replay runs/phase6_link.db`.

To fly with the C flight software instead of the Python references:
`build_sim(obc_impl="c", eps_impl="c")`. Observed emergent behaviors are cataloged in
[EMERGENT_BEHAVIORS.md](EMERGENT_BEHAVIORS.md) — the policy is to keep and
document them unless they break simulation integrity.

## Configuring the ground network

The mission flies a single ground station by default (Bozeman, 45.7°N
111°W). A station is just a named site — `GroundSite(name, lat, lon)` —
and `build_sim` takes a list of them, so adding coverage is a one-liner.
The satellite works whichever visible station has the best elevation
(several antennas, one ops center); a change of active station mid-pass
is logged as a `contact_handover` event.

```python
from cubesat_sim.environment.groundstation import GroundSite
from cubesat_sim.environment.orbit import CircularOrbit
from cubesat_sim.physics.spacecraft import DEFAULT_STATION, TOKYO_STATION
from cubesat_sim.mission import build_sim

sao_paulo_gs = GroundSite("sao_paulo_gs", -23.55, -46.63)   # name, lat, lon

sim = build_sim(
    seed=1,
    recorder_path="runs/threestation.db",
    stations=[DEFAULT_STATION, TOKYO_STATION, sao_paulo_gs],
)
sim.run(duration=12 * CircularOrbit().period_s)   # 12 orbits
sim.close()
```

Nothing else needs wiring: `build_sim` records the station geometry into
the flight recording, so the report and live console render every
station straight from it — its marker, its green downlink beam, the
`ground contact ×N` count, and the map legend:

```bash
.venv/bin/python -m cubesat_sim.frontend.dashboard runs/threestation.db
.venv/bin/python -m cubesat_sim.frontend.live --replay runs/threestation.db
```

**Where to put one.** The default orbit is 51.6° inclined, so the ground
track only reaches ±51.6° latitude:

- Keep a station's latitude inside that band — a higher-latitude site
  only ever sees the satellite low on the horizon, so a polar station
  does *not* help this orbit.
- Spread stations in longitude, ideally near an imaging target, to catch
  the passes a lone station misses as its ground track drifts west — and
  to downlink science in the same pass it was captured. (One station near
  Tokyo took stranded science from 79% to 27% on a 12-orbit flight.)
- A pass opens above `CONTACT_MIN_ELEV_DEG` (10° elevation); raise it in
  `cubesat_sim.physics.spacecraft` to model terrain masking.

Passing `stations=` per run is deterministic and leaves the
single-station default — and every existing recording — untouched. To
make a network the default for *every* flight, edit `DEFAULT_STATIONS` in
`cubesat_sim.physics.spacecraft`; that is a deliberate change, since it
moves all default recordings. The live console builds its own sim and has
no `--stations` flag yet, so a live (non-`--replay`) multi-station flight
currently means a short `build_sim` script.
