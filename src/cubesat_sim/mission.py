"""Convenience builder for the current mission configuration."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

from cubesat_sim.environment.orbit import CircularOrbit
from cubesat_sim.faults import FaultInjector, ScheduledFault
from cubesat_sim.kernel.remote import RemoteComponent
from cubesat_sim.kernel.simulation import Simulation
from cubesat_sim.physics.attitude import AttitudeDynamics, AttitudeParams
from cubesat_sim.physics.power import Battery, SolarArray
from cubesat_sim.physics.spacecraft import SpacecraftPhysics
from cubesat_sim.physics.thermal import ThermalModel
from cubesat_sim.subsystems.eps import EPS
from cubesat_sim.subsystems.obc import OBC
from cubesat_sim.ground.station import GroundStation
from cubesat_sim.subsystems.payload import PayloadController
from cubesat_sim.subsystems.thermal_ctrl import ThermalControl

REPO_ROOT = Path(__file__).resolve().parents[2]
C_OBC_BIN = REPO_ROOT / "c" / "obc" / "obc"
C_EPS_BIN = REPO_ROOT / "c" / "eps" / "eps"
RUST_ADCS_BIN = REPO_ROOT / "rust" / "adcs" / "target" / "release" / "adcs"
CPP_COMMS_BIN = REPO_ROOT / "cpp" / "comms" / "comms"


def build_sim(
    *,
    dt: float = 1.0,
    seed: int = 0,
    recorder_path: str | Path | None = None,
    epoch: datetime | None = None,
    illumination: float = 0.8,
    initial_soc: float = 0.85,
    thermal_sun_w: float = 36.0,
    initial_tumble_dps: float = 4.0,
    payload_rate_mb_s: float = 0.25,
    battery_fade_per_wh: float = 2e-4,
    array_decay_per_year: float = 0.025,
    wheel_friction_nm_per_nms: float = 0.0,
    wheel_friction_growth_per_s: float = 0.0,
    faults: tuple[ScheduledFault, ...] | list[ScheduledFault] = (),
    seu_rate_per_day: float = 0.0,
    obc_impl: str = "python",
    eps_impl: str = "python",
    adcs_impl: str = "rust",
    comms_impl: str = "cpp",
) -> Simulation:
    """One CubeSat in a 500 km / 51.6 deg orbit: physics + EPS + OBC + thermal.

    `illumination` scales electrical generation (array health / pointing);
    `thermal_sun_w` scales absorbed solar heat (a cold case models e.g. a
    high-beta winter season or degraded coatings). Degradation defaults are
    realistic (i.e. tiny over a few orbits): battery fades with throughput,
    the array darkens under radiation, wheel bearings can be born worn or
    wear during flight. `faults` is a ScheduledFault campaign script;
    `seu_rate_per_day` adds random bit flips, elevated over the South
    Atlantic Anomaly. `obc_impl` selects the Python reference OBC or the C
    flight build ("python" | "c"); the two are bit-identical in behavior.
    """
    sim = Simulation(dt=dt, seed=seed, recorder_path=recorder_path, epoch=epoch)
    orbit = CircularOrbit()
    sim.add(SpacecraftPhysics(
        orbit=orbit,
        array=SolarArray(illumination=illumination,
                         decay_per_year=array_decay_per_year),
        battery=Battery(soc=initial_soc, fade_per_wh=battery_fade_per_wh),
        thermal=ThermalModel(sun_absorbed_w=thermal_sun_w),
        attitude=AttitudeDynamics(AttitudeParams(
            wheel_friction_nm_per_nms=wheel_friction_nm_per_nms,
            wheel_friction_growth_per_s=wheel_friction_growth_per_s,
        )),
        initial_tumble_dps=initial_tumble_dps,
    ))
    if eps_impl == "python":
        sim.add(EPS())
    elif eps_impl == "c":
        sim.add(RemoteComponent("eps", period=1.0, argv=[str(C_EPS_BIN)]))
    else:
        raise ValueError(f"unknown eps_impl: {eps_impl!r}")
    if obc_impl == "python":
        sim.add(OBC())
    elif obc_impl == "c":
        sim.add(RemoteComponent("obc", period=1.0, argv=[str(C_OBC_BIN)]))
    else:
        raise ValueError(f"unknown obc_impl: {obc_impl!r}")
    sim.add(ThermalControl())
    if adcs_impl == "rust":
        sim.add(RemoteComponent("adcs", period=1.0, argv=[str(RUST_ADCS_BIN)]))
    elif adcs_impl != "none":
        raise ValueError(f"unknown adcs_impl: {adcs_impl!r}")
    sim.add(PayloadController(data_rate_mb_s=payload_rate_mb_s))
    if comms_impl == "cpp":
        sim.add(RemoteComponent("comms", period=1.0, argv=[str(CPP_COMMS_BIN)]))
    elif comms_impl != "none":
        raise ValueError(f"unknown comms_impl: {comms_impl!r}")
    sim.add(GroundStation())
    if faults or seu_rate_per_day > 0.0:
        sim.add(FaultInjector(schedule=faults, seu_rate_per_day=seu_rate_per_day,
                              orbit=orbit))
    return sim
