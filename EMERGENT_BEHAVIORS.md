# Emergent behavior catalog

Policy: emergent behaviors are findings, not bugs. When one appears, it is
**kept and documented here** — we only intervene if simulation integrity
breaks (numerical blow-up, crash, NaN). Each entry records how to reproduce
the behavior and the mechanism behind it, verified against the flight
recorder.

Template:

```
## <name>
- first observed: <phase / commit>
- reproduce: build_sim(<args>), run <N> orbits
- mechanism: <causal chain>
- status: kept / superseded by <entry>
```

---

## 1. Eclipse-phase-locked mode limit cycle
- first observed: Phase 1 (commit 9aa3216, pre-thermal)
- reproduce: at 9aa3216, `build_sim(dt=1.0, seed=42, illumination=0.45)`,
  12 orbits (see `examples/phase1_demo.py` of that era)
- mechanism: with a ~35% energy deficit, the OBC's NOMINAL<->SAFE hysteresis
  band (soc_est 0.25/0.45) turns into a stable oscillator with period ~1
  orbit. The lock to the eclipse cycle comes from *estimator bias*, not
  energy: the EPS derives SoC from bus voltage, which sags under eclipse
  discharge and rises under charge current. The satellite entered SAFE in
  eclipse and exited in sunlight at nearly the same true SoC (~0.30). An
  unplanned payload duty cycle, phase-locked by a sensor artifact.
- status: superseded by entry 2 once thermal (Phase 2) added the heater to
  the energy budget.

## 2. EPS-hysteresis bang-bang equilibrium
- first observed: Phase 2 (commit a01c658)
- reproduce: `build_sim(dt=5.0, seed=2, illumination=0.45)`, 12 orbits
  (asserted by `test_degraded_array_finds_protected_equilibrium`)
- mechanism: with the battery heater in the budget, SAFE-mode margin is
  ~zero, so the OBC parks in SAFE permanently. Instead of mode cycling, the
  EPS's own shed/restore band (soc_est 0.15/0.30) starts flapping once per
  orbit — shedding ADCS + heater near eclipse end, restoring in sunlight.
  Two protection layers wedge the battery at ~0.25 SoC indefinitely: the
  shed hysteresis has become an accidental bang-bang charge regulator.
- status: superseded by entry 6 once Phase 3 coupled attitude into
  generation — the bang-bang loop no longer closes, because shedding the
  ADCS now collapses generation instead of merely saving load.

## 3. Cold-shed death spiral
- first observed: Phase 2 (commit a01c658)
- reproduce: `build_sim(dt=5.0, seed=4, illumination=0.45,
  thermal_sun_w=26.0)`, 20 orbits (asserted by
  `test_cold_degraded_flight_death_spirals`; `examples/phase2_demo.py`)
- mechanism: energy deficit -> SAFE (orbit ~2.2) -> EPS hard-shed kills the
  battery heater (orbit ~4.2) -> battery falls below the li-ion cold-charge
  limit (0 C) and keeps dropping to ~-17 C -> every sun pass, charging is
  physically inhibited while loads keep draining -> brownout (~orbit 17).
  Each protection did its local job; jointly they killed the satellite. The
  EPS treating the heater as an ordinary sheddable load is the trap.
- status: kept — this is the Phase 2 signature failure mode.

## 4. Startup-race tumble pumping (fixed — broke sim integrity)
- first observed: Phase 3 development
- reproduce: revert the `has_gyro` guard in `rust/adcs/src/main.rs`,
  `build_sim(dt=1.0, seed=21)`, ~1 orbit
- mechanism: on its first control frame the ADCS had received no sensor
  data yet, read a zeroed gyro as "rate below detumble-exit threshold," and
  switched to SUN_POINT — while the vehicle was genuinely tumbling at
  4 deg/s, *below* the 5 deg/s detumble re-entry threshold, so the mistake
  latched. The PD controller then chased a spinning sun vector through the
  bus's one-tick sensor latency, pumping energy into the tumble (4 -> 7+
  deg/s) instead of damping it; at coarse timesteps the growing rates
  eventually blew up the explicit-Euler attitude integrator into NaN.
- status: fixed (violates the "sim must not blow up" line): flight software
  now refuses to make mode decisions before sensor data arrives, and the
  attitude integrator substeps at <= 1 s internally. Preserved here because
  the *mechanism* — mode logic latching a wrong state via a threshold gap
  plus sensor latency — is a real class of flight software bug worth
  keeping in mind for Phase 5 fault injection.

## 5. Under-sampled control loop divergence + NaN bus deadlock (fixed —
##    broke sim integrity)
- first observed: Phase 3 development
- reproduce: revert the gain-scheduling block in `rust/adcs/src/main.rs`,
  `build_sim(dt=5.0, seed=1)`, ~350 ticks
- mechanism: two-stage failure. (1) The sun-point PD gains were designed
  for a 1 Hz control rate; run at 0.2 Hz (sim dt=5) the damping term alone
  sits past the discrete stability boundary (kd*dt/I = 2.0), and the bus's
  one-tick actuation delay pushes it firmly unstable — the controller
  pumped the smallest-inertia axis exponentially to ~1e210 and NaN.
  (2) The NaN then propagated into a sensor frame: Python's json emits
  non-standard `NaN`, the Rust side rejected the unparseable frame and
  skipped it *without replying*, deadlocking the lockstep bus. A corrupted
  packet quietly hanging the whole spacecraft bus is a very real CPS
  failure shape.
- status: fixed (both stages violate sim integrity): the ADCS now
  gain-schedules by measured sample time, flight software always answers a
  step frame even if unparseable (and logs a `frame_reject` event), the
  bridge refuses to serialize non-finite values, and physics raises loudly
  if attitude state goes non-finite. (Two further integrity fixes fell out
  of the same investigation: a wheel-torque sign-convention mismatch
  between controller and plant that turned the rate damper into a rate
  pump, and explicit-Euler attitude integration slowly pumping energy into
  *uncontrolled* tumbling until overflow after ~18 simulated hours — the
  integrator is now RK4 with substeps.)

## 6. Gravity-gradient anti-sun capture: the ADCS shed is a one-way door
- first observed: Phase 3 (attitude ↔ power coupling)
- reproduce: `build_sim(dt=5.0, seed=2, illumination=0.45)`, 12 orbits
  (asserted by `test_degraded_array_adcs_shed_is_a_one_way_door`)
- mechanism: energy deficit -> SAFE -> EPS hard-shed cuts the ADCS load
  (orbit ~7.4). The now-freewheeling satellite is captured by
  gravity-gradient torque into a *calmer* attitude than active control
  gave it (rates fall to ~0.4 deg/s) — but with the solar panel averaging
  anti-sun (facing ~ -0.5). Generation pins at the side-panel floor
  (~0.7 W), below even the essential OBC+radio load, so the SoC estimate
  can never climb back over the EPS restore threshold: the shed latches
  permanently and the satellite coasts down toward brownout over many
  orbits. Under Phase 2 physics this same scenario self-stabilized
  (entry 2); adding one coupling turned the EPS's protective reflex into a
  terminal decision. Passivity is not safety.
- status: kept — flagship Phase 3 finding. A future FDIR rule
  might notice "shed ADCS + facing < 0 + SoC falling" and gamble on
  re-powering the ADCS; whether that rule makes things better or worse is
  exactly the kind of question this simulator is for. (Deliberately
  deferred out of Phase 5 scope, 2026-07-16 — to be revisited.)

## 7. FDIR giveup cascade: an informational fault becomes an electrical death
- first observed: Phase 5 (FDIR + fault injection)
- reproduce: `build_sim(dt=1.0, seed=42,
  faults=[sensor_stuck(2500.0, "gyro", hard=True)])`, 4+ orbits
  (`examples/phase5_demo.py` hard_failure scenario)
- mechanism: a *hard* gyro latch-up defeats the OBC's power-cycle recovery:
  FDIR correctly detects the frozen output word, burns its three-cycle
  budget in ~90 s, and gives up (by design — do no harm, leave the ADCS
  powered). The lethal detail is *what value* the sensor froze at: the
  fault landed mid-detumble, latching the rate word at 0.599 deg/s —
  0.1 deg/s **above** the ADCS's 0.5 deg/s detumble-exit gate. The mode
  logic compares that threshold against a dead sensor forever, so
  SUN_POINT never engages for the rest of the mission (the recording
  shows `mode_sun_point` flat at 0; the soft-fault twin flight crosses
  the gate at t=2688 after recovery and averages 5.4 W — the hard-fault
  flight averages 2.1 W on incidental attitude alone). Nothing is
  "failing" — every subsystem is green except one sensor — but the
  energy budget now runs a structural deficit: the OBC oscillates
  SAFE/NOMINAL from orbit ~1.8, and at orbit ~3.3 the EPS hard-shed
  fires and latches (entry 6's one-way door, entered through a *sensor*
  fault instead of a degraded array). True SoC flatlines at ~0.22 with
  the payload and ADCS dead. The failure was informational; the death
  was electrical; the coupling was a mode gate.
- status: kept — flagship Phase 5 finding, and a direct sequel to entry
  4's mechanism class (mode logic latching a wrong state off bad sensor
  data — which entry 4 explicitly flagged as one to watch for under
  fault injection). First written up with the wrong mechanism ("PD flying
  with a dead damping term"); the Phase 6 dashboard exposed the truth
  within minutes of first rendering this flight — the missing sun-point
  track was visible at a glance. No rule on board connects "gyro
  unrecoverable" to "expect a power deficit"; a mag-based rate-estimate
  fallback (or entry 6's deferred re-power gamble) is what a fix would
  look like.

## 8. The watchdog's blind spot: stuck mag is invisible by design
- first observed: Phase 5 Monte Carlo sweep (16 seeds x 8 orbits,
  `examples/phase5_demo.py`)
- reproduce: any campaign flight whose stuck sensor is the magnetometer
  (e.g. `run_flight(6)`: two mag latch-ups, `fdir_cycles == 0`)
- mechanism: the OBC watchdog monitors only the gyro, so a latched
  magnetometer sails through untouched — visible in the Monte Carlo table
  as `sensor_stuck` faults with zero FDIR cycles. Consequences are mode-
  dependent: in sun-point the mag only steers desaturation (mild); in
  detumble B-dot differentiates the mag signal, and a frozen B reads as
  "field derivative zero," silently disabling detumble authority. A
  satellite that happens to be tumbling when the mag latches cannot
  detumble and nothing on board notices.
- status: kept — an honest reflection of the FDIR coverage actually built
  (real FDIR suites have exactly these coverage gaps). The Monte Carlo
  table makes the gap measurable rather than hypothetical; extending the
  watchdog pattern to the mag is straightforward if a future phase wants
  the comparison flight.
