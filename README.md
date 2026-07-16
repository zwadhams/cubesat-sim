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
- **Flight reports** (`cubesat_sim.dashboard`) — renders any flight
  recording into a single self-contained HTML file (no server, no
  dependencies): an animated orbit view, stat tiles, a digital state
  strip (eclipse, contact, safe mode, shedding, ...), an event timeline
  with severity glyphs, and telemetry lanes with a shared crosshair,
  eclipse shading, and table views. Its first render caught a wrong
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

```bash
python3 -m venv .venv
.venv/bin/pip install -e ".[dev]"
make -C c/obc                                  # C flight software (OBC)
make -C c/eps                                  # C flight software (EPS)
make -C cpp/comms                              # C++ flight software (comms)
cargo build --release --manifest-path rust/adcs/Cargo.toml   # Rust ADCS
.venv/bin/pytest
.venv/bin/python examples/phase6_link_demo.py
.venv/bin/python -m cubesat_sim.dashboard runs/phase6_link.db   # -> .html report
.venv/bin/python -m cubesat_sim.linkdump runs/phase6_link.db    # decode the link
```

To fly with the C flight software instead of the Python references:
`build_sim(obc_impl="c", eps_impl="c")`. Observed emergent behaviors are cataloged in
[EMERGENT_BEHAVIORS.md](EMERGENT_BEHAVIORS.md) — the policy is to keep and
document them unless they break simulation integrity.
