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
  their real-world counterpart would be: OBC in C (cFS heritage), ADCS in
  Rust (planned, Phase 3), EPS bare-metal-style C (planned), comms C++
  (planned), payload controller and ground segment in Python. Compiled
  subsystems run as separate OS processes behind `RemoteComponent`, a
  lockstep NDJSON stdin/stdout bridge that preserves determinism — the
  equivalence test requires the C OBC to fly bit-identically to the Python
  reference implementation.

Rules of the house:

1. Ground truth lives in the physics layer; everything else sees noisy sensors.
2. Components never call each other — all coordination goes over the bus.
3. Every run is reproducible from `(seed, dt)`.

## Build phases

- [x] Phase 0 — kernel: clock, bus, components, RNG, flight recorder
- [x] Phase 1 — orbit + power (eclipse cycles breathing through the EPS)
- [x] Phase 2 — thermal (heater/battery coupling, cold-charge inhibit, death spiral)
- [x] Phase 2.5 — process bridge + OBC ported to C (bit-identical to reference)
- [ ] Phase 3 — ADCS (attitude ↔ solar generation coupling)
- [ ] Phase 4 — data economy (payload, storage, comms windows)
- [ ] Phase 5 — FDIR, fault injection, degradation, Monte Carlo harness

## Getting started

```bash
python3 -m venv .venv
.venv/bin/pip install -e ".[dev]"
make -C c/obc          # build the C flight software
.venv/bin/pytest
.venv/bin/python examples/phase2_demo.py
```

To fly with the C OBC instead of the Python reference:
`build_sim(obc_impl="c")`.
