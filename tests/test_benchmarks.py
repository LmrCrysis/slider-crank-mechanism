"""
Validation against Theory of Machines literature benchmarks
(e.g., Norton "Design of Machinery", Khurmi, S.S. Rattan).
"""

import pytest
import numpy as np
from engine.mechanism import SliderCrankParameters
from engine.kinematics import KinematicSolver
from engine.dynamics import DynamicSolver


def test_standard_textbook_engine_benchmark():
    """
    Standard textbook benchmark problem:
    Crank r = 60 mm, Con-rod l = 240 mm (n = 4.0), RPM = 1800.
    Reciprocating mass = 1.2 kg, Bore = 80 mm.
    At theta = 45 degrees, gas pressure = 1.5 MPa.
    """
    r = 0.06
    l = 0.24
    rpm = 1800.0
    theta_rad = np.radians(45.0)

    params = SliderCrankParameters(
        crank_length=r,
        con_rod_length=l,
        offset=0.0,
        rpm=rpm,
        piston_mass=1.2,
        con_rod_mass=0.0,  # Pure piston mass for exact textbook match
        bore_diameter=0.08,
    )

    kin_solver = KinematicSolver(params)
    kin = kin_solver.solve(theta_rad)

    # 1. Kinematic checks
    # Expected con-rod angle: sin(phi) = sin(45)/4 = 0.176777 -> phi = 10.182 deg
    assert np.isclose(kin.con_rod_angle_deg, 10.182, atol=0.01)

    # Expected velocity: around -9.43 m/s
    omega = 2.0 * np.pi * 1800.0 / 60.0
    approx_v = -r * omega * (np.sin(theta_rad) + np.sin(2 * theta_rad) / (2 * 4.0))
    assert np.isclose(kin.piston_velocity, approx_v, rtol=0.02)

    # 2. Dynamic checks with 1.5 MPa gas pressure
    p_gas = 1.5e6  # 1.5 MPa
    f_gas = p_gas * params.piston_area  # ~7539.82 N

    dyn_solver = DynamicSolver(params)
    dyn = dyn_solver.solve(theta_rad, kinematics=kin, custom_gas_force=f_gas)

    assert np.isclose(dyn.gas_force, f_gas, rtol=1e-5)
    # Piston effort = F_gas + F_I
    # F_I = m * a where a is ~ -1507 m/s^2, so F_I is ~ -1809 N
    assert dyn.inertia_force < 0  # Opposing gas force at 45 deg
    assert np.isclose(dyn.piston_effort, dyn.gas_force + dyn.inertia_force, rtol=1e-5)

    # Crank effort Ft and Torque T
    expected_ft = dyn.con_rod_force * np.sin(theta_rad + kin.con_rod_angle_rad)
    assert np.isclose(dyn.crank_effort, expected_ft, rtol=1e-5)
    assert np.isclose(dyn.crank_torque, expected_ft * r, rtol=1e-5)


def test_adams_matlab_inline_slider_benchmark():
    """
    ADAMS / MATLAB Simscape benchmark case:
    r = 0.1 m, l = 0.4 m, RPM = 600 (omega = 20*pi rad/s ~ 62.83 rad/s).
    Max piston stroke must equal 0.2 m exactly.
    """
    params = SliderCrankParameters(crank_length=0.1, con_rod_length=0.4, offset=0.0, rpm=600.0)
    assert np.isclose(params.stroke, 0.2, atol=1e-12)

    kin_solver = KinematicSolver(params)
    state_cycle = kin_solver.solve_full_cycle(num_points=1000)

    # Max displacement should equal stroke
    assert np.isclose(np.max(state_cycle.piston_displacement), 0.2, atol=1e-5)
    # Min displacement should equal 0
    assert np.isclose(np.min(state_cycle.piston_displacement), 0.0, atol=1e-5)
