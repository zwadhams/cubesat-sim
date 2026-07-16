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


def test_battery_fades_with_throughput():
    battery = Battery(capacity_wh=20.0, soc=0.5, fade_per_wh=2e-4)
    for _ in range(100):  # 100 full cycles: 40 Wh throughput each
        battery.integrate(p_net_w=-20.0, dt_s=3600.0)
        battery.integrate(p_net_w=20.0, dt_s=3600.0)
    # 4000 Wh through a 2e-4/Wh fade -> 0.8 Wh gone (20%/500 cycles pace)
    assert abs(battery.capacity_wh - 19.2) < 0.01
    floor = Battery(capacity_wh=20.0, soc=0.5, fade_per_wh=1.0)
    for _ in range(100):
        floor.integrate(p_net_w=-20.0, dt_s=3600.0)
    assert floor.capacity_wh == floor.capacity_floor_wh  # bounded below


def test_array_darkens_only_in_sunlight():
    array = SolarArray(illumination=0.8, decay_per_year=0.5)
    year_s = 365.25 * 86400.0
    array.age(dt_s=year_s / 2, in_sun=False)
    assert array.illumination == 0.8  # eclipse: no radiation aging modeled
    array.age(dt_s=year_s / 2, in_sun=True)
    assert 0.55 < array.illumination < 0.65  # half a year at 50%/yr
    for _ in range(100):
        array.age(dt_s=year_s, in_sun=True)
    assert array.illumination == array.illum_floor  # bounded below


def test_voltage_curve_and_sag():
    battery = Battery(soc=0.5)
    v_rest = battery.voltage(0.0)
    assert abs(v_rest - 7.2) < 1e-9
    assert battery.voltage(-7.2) < v_rest  # discharge sags
    assert battery.voltage(+7.2) > v_rest  # charge rises
    assert Battery(soc=0.9).voltage(0.0) > Battery(soc=0.2).voltage(0.0)
