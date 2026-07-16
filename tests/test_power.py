from cubesat_sim.physics.power import Battery, SolarArray


def test_array_dark_in_eclipse():
    array = SolarArray(p_max_w=10.0, illumination=0.8)
    assert array.generation_w(in_eclipse=False) == 8.0
    assert array.generation_w(in_eclipse=True) == 0.0


def test_battery_integrates_energy():
    battery = Battery(capacity_wh=20.0, soc=0.4)
    battery.integrate(p_net_w=10.0, dt_s=3600.0)  # +10 Wh on 20 Wh pack
    assert abs(battery.soc - 0.9) < 1e-9


def test_battery_clamps():
    battery = Battery(capacity_wh=20.0, soc=0.95)
    battery.integrate(p_net_w=100.0, dt_s=3600.0)
    assert battery.soc == 1.0
    battery.integrate(p_net_w=-1000.0, dt_s=3600.0)
    assert battery.soc == 0.0


def test_voltage_curve_and_sag():
    battery = Battery(soc=0.5)
    v_rest = battery.voltage(0.0)
    assert abs(v_rest - 7.2) < 1e-9
    assert battery.voltage(-7.2) < v_rest  # discharge sags
    assert battery.voltage(+7.2) > v_rest  # charge rises
    assert Battery(soc=0.9).voltage(0.0) > Battery(soc=0.2).voltage(0.0)
