//! ADCS flight software: detumble, sun acquisition, momentum management.
//!
//! Runs as a separate process behind the simulator's lockstep NDJSON
//! bridge (see kernel/remote.py for the protocol). Deterministic by
//! construction: no wall clock, no randomness — state depends only on the
//! frames received.
//!
//! Control modes:
//!   DETUMBLE  — B-dot law on magnetorquers until body rates are low
//!   SUN_POINT — PD control on reaction wheels to put body +Z on the sun
//!               (rate damping only while in eclipse), with a concurrent
//!               magnetorquer momentum-dump when the wheels fill up

use serde::Deserialize;
use serde_json::{json, Value};
use std::io::{self, BufRead, Write};

const H_MAX: f64 = 0.010; // wheel momentum capacity per axis, N m s
const TAU_MAX: f64 = 1.0e-3; // wheel torque limit per axis, N m
const M_MAX: f64 = 0.2; // magnetorquer moment limit per axis, A m^2
const KP: f64 = 4.0e-4; // sun-pointing proportional gain (at 1 Hz design rate)
const KD: f64 = 1.0e-2; // rate damping gain (at 1 Hz design rate)
const I_MIN: f64 = 0.025; // smallest principal inertia, for gain scheduling
const K_BDOT: f64 = 1.0e5; // detumble gain
const DETUMBLE_EXIT_DPS: f64 = 0.5;
const DETUMBLE_ENTER_DPS: f64 = 5.0;
const DESAT_START_FRAC: f64 = 0.7;
const DESAT_STOP_FRAC: f64 = 0.3;

#[derive(Deserialize)]
struct InMsg {
    topic: String,
    data: Value,
}

#[derive(Deserialize)]
struct Frame {
    #[serde(rename = "type")]
    kind: String,
    t: Option<f64>,
    msgs: Option<Vec<InMsg>>,
}

#[derive(Clone, Copy, PartialEq)]
enum Mode {
    Detumble,
    SunPoint,
}

impl Mode {
    fn name(self) -> &'static str {
        match self {
            Mode::Detumble => "DETUMBLE",
            Mode::SunPoint => "SUN_POINT",
        }
    }
}

fn vec3(data: &Value) -> [f64; 3] {
    [
        data["x"].as_f64().unwrap_or(0.0),
        data["y"].as_f64().unwrap_or(0.0),
        data["z"].as_f64().unwrap_or(0.0),
    ]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn norm(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

fn clamp3(a: [f64; 3], limit: f64) -> [f64; 3] {
    [
        a[0].clamp(-limit, limit),
        a[1].clamp(-limit, limit),
        a[2].clamp(-limit, limit),
    ]
}

struct Adcs {
    mode: Mode,
    desat: bool,
    has_gyro: bool,
    gyro: [f64; 3],
    mag: [f64; 3],
    sun: [f64; 3],
    sun_valid: bool,
    wheel_h: [f64; 3],
    prev_mag: Option<([f64; 3], f64)>, // (field, time) for B-dot
    prev_t: Option<f64>,               // for sample-time measurement
}

impl Adcs {
    fn new() -> Self {
        Adcs {
            mode: Mode::Detumble,
            desat: false,
            has_gyro: false,
            gyro: [0.0; 3],
            mag: [0.0; 3],
            sun: [0.0; 3],
            sun_valid: false,
            wheel_h: [0.0; 3],
            prev_mag: None,
            prev_t: None,
        }
    }

    fn ingest(&mut self, msgs: &[InMsg]) {
        for m in msgs {
            match m.topic.as_str() {
                "sensors/adcs/gyro" => {
                    self.gyro = vec3(&m.data);
                    self.has_gyro = true;
                }
                "sensors/adcs/mag" => self.mag = vec3(&m.data),
                "sensors/adcs/wheel_momentum" => self.wheel_h = vec3(&m.data),
                "sensors/adcs/sun" => {
                    self.sun_valid = m.data["valid"].as_bool().unwrap_or(false);
                    self.sun = vec3(&m.data);
                }
                _ => {}
            }
        }
    }

    fn control(&mut self, t: f64) -> (Value, Vec<Value>) {
        // no sensor data yet: command nothing, decide nothing. Making mode
        // decisions from a zeroed gyro on the first frame put the controller
        // into SUN_POINT while genuinely tumbling — see EMERGENT_BEHAVIORS.md
        // entry 4 for what that did.
        if !self.has_gyro {
            return (
                json!({"type": "out", "pub": [], "telemetry": {}, "events": []}),
                Vec::new(),
            );
        }

        // Sample-time-aware gain scheduling: the PD gains are designed for
        // the 1 Hz control rate; if frames arrive slower (coarser sim step),
        // scale gains down to stay inside the discrete stability envelope
        // (a delayed damping loop needs kd*dt/I well below 1).
        let dt = self
            .prev_t
            .map(|p| t - p)
            .filter(|d| *d > 0.0)
            .unwrap_or(5.0); // conservative until measured
        self.prev_t = Some(t);
        let kd = KD.min(0.4 * I_MIN / dt);
        let kp = KP.min(0.1 * I_MIN / (dt * dt));

        let rate_dps = norm(self.gyro).to_degrees();
        let mut events: Vec<Value> = Vec::new();

        let new_mode = match self.mode {
            Mode::Detumble if rate_dps < DETUMBLE_EXIT_DPS => Mode::SunPoint,
            Mode::SunPoint if rate_dps > DETUMBLE_ENTER_DPS => Mode::Detumble,
            m => m,
        };
        if new_mode != self.mode {
            self.mode = new_mode;
            events.push(json!({
                "kind": "mode_change",
                "detail": {"to": self.mode.name(), "rate_dps": rate_dps}
            }));
        }

        let wheel_frac = self
            .wheel_h
            .iter()
            .fold(0.0f64, |acc, h| acc.max(h.abs() / H_MAX));
        if !self.desat && wheel_frac > DESAT_START_FRAC {
            self.desat = true;
            events.push(json!({
                "kind": "desat_start", "detail": {"wheel_frac": wheel_frac}
            }));
        } else if self.desat && wheel_frac < DESAT_STOP_FRAC {
            self.desat = false;
            events.push(json!({
                "kind": "desat_stop", "detail": {"wheel_frac": wheel_frac}
            }));
        }

        let mut wheel_tau = [0.0f64; 3];
        let mut mtq_m = [0.0f64; 3];
        let mut sun_err_deg = -1.0f64; // sentinel: unknown (eclipse)

        match self.mode {
            Mode::Detumble => {
                // B-dot: m = -k dB/dt, pure magnetorquer detumble
                if let Some((b_prev, t_prev)) = self.prev_mag {
                    let dt = t - t_prev;
                    if dt > 0.0 {
                        for i in 0..3 {
                            mtq_m[i] = -K_BDOT * (self.mag[i] - b_prev[i]) / dt;
                        }
                    }
                }
            }
            Mode::SunPoint => {
                if self.sun_valid {
                    let s = self.sun;
                    let z = [0.0, 0.0, 1.0];
                    let axis = cross(z, s); // rotate +Z toward the sun
                    for i in 0..3 {
                        wheel_tau[i] = kp * axis[i] - kd * self.gyro[i];
                    }
                    let cos_err = s[2] / norm(s).max(1e-12);
                    sun_err_deg = cos_err.clamp(-1.0, 1.0).acos().to_degrees();
                } else {
                    // eclipse: hold rates, drift in attitude
                    for i in 0..3 {
                        wheel_tau[i] = -kd * self.gyro[i];
                    }
                }
                if self.desat {
                    // momentum dump: m = c (h x b_hat) gives torque ~ -h_perp
                    let bn = norm(self.mag).max(1e-12);
                    let b_hat = [self.mag[0] / bn, self.mag[1] / bn, self.mag[2] / bn];
                    let hxb = cross(self.wheel_h, b_hat);
                    for i in 0..3 {
                        mtq_m[i] = M_MAX * hxb[i] / H_MAX;
                    }
                }
            }
        }

        self.prev_mag = Some((self.mag, t));
        let wheel_tau = clamp3(wheel_tau, TAU_MAX);
        let mtq_m = clamp3(mtq_m, M_MAX);

        let out = json!({
            "type": "out",
            "pub": [
                {"topic": "cmd/adcs/wheel_torque",
                 "data": {"x": wheel_tau[0], "y": wheel_tau[1], "z": wheel_tau[2]}},
                {"topic": "cmd/adcs/mtq",
                 "data": {"x": mtq_m[0], "y": mtq_m[1], "z": mtq_m[2]}},
                {"topic": "adcs/status",
                 "data": {"mode": self.mode.name(), "rate_dps": rate_dps,
                           "wheel_frac": wheel_frac, "desat": self.desat}}
            ],
            "telemetry": {
                "rate_dps": rate_dps,
                "wheel_frac": wheel_frac,
                "sun_err_deg": sun_err_deg,
                "mode_sun_point": if self.mode == Mode::SunPoint {1.0} else {0.0}
            },
            "events": events
        });
        (out, Vec::new())
    }
}

fn main() {
    let stdin = io::stdin();
    let stdout = io::stdout();
    let mut out = stdout.lock();
    let mut adcs = Adcs::new();

    for line in stdin.lock().lines() {
        let line = match line {
            Ok(l) => l,
            Err(_) => break,
        };
        let frame: Frame = match serde_json::from_str(&line) {
            Ok(f) => f,
            Err(_) => {
                // Never leave the bus hanging: a step frame we cannot parse
                // (corrupt packet) still gets an (empty) reply, plus an
                // event so the rejection is visible in the flight recorder.
                if line.contains("\"type\":\"step\"") {
                    writeln!(
                        out,
                        "{}",
                        json!({"type": "out", "pub": [], "telemetry": {},
                               "events": [{"kind": "frame_reject", "detail": {}}]})
                    )
                    .unwrap();
                    out.flush().unwrap();
                }
                continue;
            }
        };
        match frame.kind.as_str() {
            "init" => {
                writeln!(
                    out,
                    "{}",
                    json!({"type": "ready", "subscribe": ["sensors/adcs/*"]})
                )
                .unwrap();
                out.flush().unwrap();
            }
            "shutdown" => break,
            "step" => {
                if let Some(msgs) = &frame.msgs {
                    adcs.ingest(msgs);
                }
                let (reply, _) = adcs.control(frame.t.unwrap_or(0.0));
                writeln!(out, "{}", reply).unwrap();
                out.flush().unwrap();
            }
            _ => {}
        }
    }
}
