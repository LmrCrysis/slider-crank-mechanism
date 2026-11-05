"""
Unit tests and analytical verification for Slider-Crank Kinematics.
"""

import pytest
import numpy as np
from engine.mechanism import SliderCrankParameters
from engine.kinematics import KinematicSolver


def test_tdc_and_bdc_positions():
    """Verify exact analytical values at Dead Centers for inline slider-crank."""
    r = 0.06
    l = 0.24
    rpm = 1200.0
    params = SliderCrankParameters(crank_length=r, con_rod_length=l, offset=0.0, rpm=rpm)
    solver = KinematicSolver(params)

    # 1. At TDC (theta = 0)
    tdc_state = solver.solve(0.0)
    assert np.isclose(tdc_state.piston_position, r + l, atol=1e-12)
    assert np.isclose(tdc_state.piston_displacement, 0.0, atol=1e-12)
    assert np.isclose(tdc_state.piston_velocity, 0.0, atol=1e-10)
    assert np.isclose(tdc_state.con_rod_angle_rad, 0.0, atol=1e-12)

    # Con-rod angular velocity at TDC: omega_c = omega / n
    omega = params.omega
    n = l / r
    expected_omega_c_tdc = omega / n
    assert np.isclose(tdc_state.con_rod_ang_velocity, expected_omega_c_tdc, rtol=1e-5)
    assert np.isclose(tdc_state.con_rod_ang_accel, 0.0, atol=1e-10)

    # Theoretical acceleration at TDC: a = -r * omega^2 * (1 + 1/n)
    omega = params.omega
    n = l / r
    expected_a_tdc = -r * (omega**2) * (1.0 + 1.0 / n)
    assert np.isclose(tdc_state.piston_acceleration, expected_a_tdc, rtol=1e-5)

    # 2. At BDC (theta = pi)
    bdc_state = solver.solve(np.pi)
    assert np.isclose(bdc_state.piston_position, l - r, atol=1e-12)
    assert np.isclose(bdc_state.piston_displacement, 2.0 * r, atol=1e-12)
    assert np.isclose(bdc_state.piston_velocity, 0.0, atol=1e-10)
    assert np.isclose(bdc_state.con_rod_angle_rad, 0.0, atol=1e-12)

    # Theoretical acceleration at BDC: a = +r * omega^2 * (1 - 1/n)
    expected_a_bdc = r * (omega**2) * (1.0 - 1.0 / n)
    assert np.isclose(bdc_state.piston_acceleration, expected_a_bdc, rtol=1e-5)


def test_finite_difference_derivatives():
    """Verify that analytical formulas match numerical finite difference derivatives."""
    params = SliderCrankParameters(crank_length=0.05, con_rod_length=0.20, offset=0.01, rpm=2400.0)
    solver = KinematicSolver(params)

    verification = solver.numerical_verification(num_points=360, delta_theta=1e-6)
    # Velocity error between analytical & central finite difference should be < 0.001 m/s
    assert verification["max_velocity_error"] < 1e-3, f"Vel error: {verification['max_velocity_error']}"
    # Acceleration error should be < 0.1 m/s^2 (for 2400 RPM where peak acc is > 3000 m/s^2)
    assert verification["max_acceleration_error"] < 0.5, f"Acc error: {verification['max_acceleration_error']}"


def test_stroke_length():
    """Verify stroke equals 2*r for inline, and correct geometry for offset."""
    params_inline = SliderCrankParameters(crank_length=0.075, con_rod_length=0.30, offset=0.0)
    assert np.isclose(params_inline.stroke, 2.0 * 0.075, atol=1e-12)

    # For offset
    params_offset = SliderCrankParameters(crank_length=0.05, con_rod_length=0.20, offset=0.02)
    expected_stroke = np.sqrt(0.25**2 - 0.02**2) - np.sqrt(0.15**2 - 0.02**2)
    assert np.isclose(params_offset.stroke, expected_stroke, atol=1e-10)


def test_speed_range_0_to_8000_rpm():
    """Verify mechanism operation at boundaries: 0 RPM (static) and 8000 RPM (max speed)."""
    # 1. Static at 0 RPM
    p_zero = SliderCrankParameters(crank_length=0.05, con_rod_length=0.20, rpm=0.0)
    solver_zero = KinematicSolver(p_zero)
    state_zero = solver_zero.solve(np.pi / 4.0)
    assert np.isclose(state_zero.piston_velocity, 0.0, atol=1e-12)
    assert np.isclose(state_zero.piston_acceleration, 0.0, atol=1e-12)
    assert state_zero.piston_displacement > 0

    # 2. Maximum speed at 8000 RPM
    p_max = SliderCrankParameters(crank_length=0.05, con_rod_length=0.20, rpm=8000.0)
    assert np.isclose(p_max.rpm, 8000.0)
    solver_max = KinematicSolver(p_max)
    state_max = solver_max.solve(0.0)
    assert state_max.piston_acceleration < -10000.0  # High acceleration at 8000 RPM

    # 3. Validation errors outside 0-8000 range
    with pytest.raises(ValueError):
        SliderCrankParameters(rpm=-10.0)
    with pytest.raises(ValueError):
        SliderCrankParameters(rpm=8500.0)
