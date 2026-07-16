from cubesat_sim.physics.thermal import ThermalModel


def run_model(model, seconds, in_sun, struct_w=0.0, batt_w=0.0, dt=10.0):
    for _ in range(int(seconds / dt)):
        model.step(dt, in_sun=in_sun, struct_dissipation_w=struct_w,
                   batt_dissipation_w=batt_w)


def test_cools_in_eclipse():
    model = ThermalModel(initial_temp_k=283.0)
    run_model(model, 2000, in_sun=False)
    assert model.structure.temp_k < 278.0


def test_sun_equilibrium_matches_radiation_balance():
    model = ThermalModel(initial_temp_k=283.0)
    run_model(model, 80000, in_sun=True, struct_w=3.0)
    q_total = model.sun_absorbed_w + model.earth_ir_w + 3.0
    t_eq = (q_total / model.rad_coeff_w_k4) ** 0.25
    assert abs(model.structure.temp_k - t_eq) < 1.0


def test_heater_holds_battery_above_structure():
    model = ThermalModel(initial_temp_k=283.0)
    run_model(model, 80000, in_sun=True, struct_w=3.0, batt_w=1.5)
    delta = model.battery.temp_k - model.structure.temp_k
    expected = 1.5 / model.g_batt_struct_w_k  # 10 K at defaults
    assert abs(delta - expected) < 0.5


def test_battery_lags_structure():
    model = ThermalModel(initial_temp_k=290.0)
    run_model(model, 1500, in_sun=False)
    struct_drop = 290.0 - model.structure.temp_k
    batt_drop = 290.0 - model.battery.temp_k
    assert batt_drop < struct_drop * 0.5  # battery is the slow node
