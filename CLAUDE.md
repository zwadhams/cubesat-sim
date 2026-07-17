# CLAUDE.md — working notes for agents on this repo

A CubeSat cyber-physical-system simulator. The owner's core interest is
**emergent behavior** from coupled subsystems — not scripted scenarios.
Polyglot flight software, realistic to what each component would fly:
OBC in C (cFS style), EPS in bare-metal C (no heap), ADCS in Rust,
comms in C++, payload/ground/physics in Python, all speaking over a
lockstep NDJSON bridge (`kernel/remote.py`).

## House rules (do not relitigate)

1. **Keep emergent behaviors; document them in EMERGENT_BEHAVIORS.md.**
   Fix a behavior only when simulation integrity breaks (NaN, crash,
   deadlock). This is the project's standing policy from the owner.
2. **Determinism is sacred.** Same (seed, dt) → bit-identical recording.
   No wall clock or unseeded RNG anywhere, including flight binaries.
   Bus delivery has a deliberate one-tick latency. Per-component RNG is
   `stream(seed, name)`.
3. **Ports must be bit-identical to their Python reference** and prove
   it with an equivalence test (see `tests/test_remote_*.py`). Gotcha
   that bites: C `%.17g` prints `1.0` as `1`, breaking JSON float parity
   — both C ports use `fmt_num`/`emit_num` for this.
4. **The bridge quarantines garbage** — JSON null / non-finite values
   from flight software are rejected loudly (`pub_reject` /
   `telemetry_reject`), never delivered (catalog entry #9 is why).
5. Charts follow the dataviz house rules (one unit per lane, never dual
   axes, fixed series-color order: truth first, estimate second).
6. Ground truth lives in physics; every other component sees only noisy
   sensors and bus traffic. Components never call each other.

## Build & test

```bash
.venv/bin/pip install -e ".[dev]"      # venv via uv; system python lacks ensurepip
make -C c/obc && make -C c/eps && make -C cpp/comms
cargo build --release --manifest-path rust/adcs/Cargo.toml   # rustup in ~/.cargo
.venv/bin/pytest                        # ~4 min, all of it must stay green
```

Don't pipe `make` through `tail`/`head` — a hidden compile error once
shipped a stale binary. Recordings land in `runs/` (gitignored).

## Owner's working style

- Right-size everything: they rejected a 192-seed campaign for 24×12;
  propose the small version first.
- They push to GitHub themselves (SSH). Commit locally; don't push.
- Infrastructure realism is valued ("components built out like real
  life") — prefer real protocols over shortcuts.
- Learning pace matters: they deferred the second satellite twice to
  master the single-sat system first.

## Backlog (agreed next steps, in rough priority)

- **FDIR experiments from campaign 1** — three findings point the same
  direction: cross-sensor consistency checks (entry #10: one frozen
  gyro fools every gate) and debounced mode transitions (entry #12:
  one SEU-corrupted sample exits SAFE). A/B them with the Monte Carlo
  harness against the campaign-1 seeds.
- **Catalog question #6** — should FDIR ever re-power a load-shed ADCS
  (the "gamble" case)? Explicitly deferred by the owner; revisit when
  FDIR work resumes.
- **Mag watchdog blind spot** (entry #8) — the gyro watchdog has no
  magnetometer sibling.
- **Larger / mission-length campaigns** — need a lean recording mode
  first (messages off, thinned telemetry); 24×12 already produces
  1.6 GB. Design sketched in conversation, not built.
- **Phase 6c: second satellite** — deferred until the owner is fluent
  with the single-sat system. When it lands: inter-sat link, shared
  ground station contention.
- **Live console on the phone** — `--host 0.0.0.0` plus a Windows
  portproxy (WSL2 NAT); small quality-of-life follow-up.

## Context worth knowing

- EMERGENT_BEHAVIORS.md holds 12 cataloged findings; read it before
  touching FDIR, EPS, or mode logic — several "bugs" are documented
  keepers.
- `runs/campaign1/` (local only) holds 24 flight recordings; seeds 0
  and 19 are the star flights (entries #10–#12).
- The flight-report dashboard (`cubesat_sim.dashboard`) renders any
  recording to self-contained HTML; the live console (`cubesat_sim.live`)
  tails a recording over SSE — headless-browser screenshots cannot
  exercise SSE, drive real CDP if you need to verify the live page.
- The live console's command panel (live flights only) posts
  `tc`/`inject` actions to `POST /control`; injections are queued and
  applied on the runner thread between ticks, and ground TCs ride
  `ops/tc` into the station's ARQ. The `_payload_ok` poison door is
  **deliberately disarmed** (owner's call, 2026-07-17): null and
  non-finite injections go through so their failure modes — bridge
  allow_nan refusal, frame_reject, Python-component crashes — stay
  reachable by hand. The armed check sits commented in
  `_inject_request`; do not restore it uninvited. This is a console-
  injection exception; house rule 4 (the bridge quarantines flight-
  software output) still stands.
