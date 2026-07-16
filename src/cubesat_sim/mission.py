"""Convenience builder for the current mission configuration."""

from __future__ import annotations

from datetime import datetime
from pathlib import Path

from cubesat_sim.kernel.simulation import Simulation
from cubesat_sim.physics.power import Battery, SolarArray
from cubesat_sim.physics.spacecraft import SpacecraftPhysics
from cubesat_sim.physics.thermal import ThermalModel
from cubesat_sim.subsystems.eps import EPS
from cubesat_sim.subsystems.obc import OBC
from cubesat_sim.subsystems.thermal_ctrl import ThermalControl


def build_sim(
    *,
    dt: float = 1.0,
    seed: int = 0,
    recorder_path: str | Path | None = None,
    epoch: datetime | None = None,
    illumination: float = 0.8,
    initial_soc: float = 0.85,
    thermal_sun_w: float = 36.0,
) -> Simulation:
    """One CubeSat in a 500 km / 51.6 deg orbit: physics + EPS + OBC + thermal.

    `illumination` scales electrical generation (array health / pointing);
    `thermal_sun_w` scales absorbed solar heat (a cold case models e.g. a
    high-beta winter season or degraded coatings).
    """
    sim = Simulation(dt=dt, seed=seed, recorder_path=recorder_path, epoch=epoch)
    sim.add(SpacecraftPhysics(
        array=SolarArray(illumination=illumination),
        battery=Battery(soc=initial_soc),
        thermal=ThermalModel(sun_absorbed_w=thermal_sun_w),
    ))
    sim.add(EPS())
    sim.add(OBC())
    sim.add(ThermalControl())
    return sim
