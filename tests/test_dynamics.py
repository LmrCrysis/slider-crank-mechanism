"""
Unit tests and energy conservation verification for Slider-Crank Dynamics.
"""

import pytest
import numpy as np
from engine.mechanism import SliderCrankParameters
from engine.kinematics import KinematicSolver
from engine.dynamics import DynamicSolver
from engine.gas_pressure import GasPressureModel, EngineCycleType


def test_pure_inertia_energy_conservation():
    """
    Fundamental Law of Machine Dynamics:
    In the absence of external gas pressure (pure reciprocating inertia),
    the net work done by reciprocating inertia over one complete cycle must equal ZERO.
    """
    params = SliderCrankParameters(
        crank_length=0.06,
        con_rod_length=0.24,
        offset=0.0,
        rpm=3000.0,
        piston_mass=1.5,
        con_rod_mass=1.8,
    )
    gas_model = GasPressureModel(params, cycle_type=EngineCycleType.CONSTANT_PRESSURE, peak_pressure_bar=1.01325)
    dyn_solver = DynamicSolver(params, gas_model=gas_model)

    thetas = np.linspace(0, 2.0 * np.pi, 720, endpoint=False)
    # Custom zero net gas force
    dyn_state = dyn_solver.solve(thetas, custom_gas_force=np.zeros_like(thetas))

    # Net work = Integral of T * dtheta
    trapz_func = getattr(np, "trapezoid", getattr(np, "trapz", None))
    net_work = trapz_func(dyn_state.crank_torque, thetas)

    # Net work should be zero within numerical integration tolerance (< 0.05 Joules for ~1000 J peak swings)
    assert np.isclose(net_work, 0.0, atol=1e-1), f"Net inertia work should be zero, got {net_work} J"


def test_torque_at_dead_centers():
    """At Dead Centers (0 and 180 deg for inline), the turning moment must be ZERO."""
    params = SliderCrankParameters(crank_length=0.05, con_rod_length=0.20, offset=0.0, rpm=1800.0)
    dyn_solver = DynamicSolver(params)

    # At TDC
    state_tdc = dyn_solver.solve(0.0)
    assert np.isclose(state_tdc.crank_torque, 0.0, atol=1e-9)
    assert np.isclose(state_tdc.cylinder_side_thrust, 0.0, atol=1e-9)

    # At BDC
    state_bdc = dyn_solver.solve(np.pi)
    assert np.isclose(state_bdc.crank_torque, 0.0, atol=1e-9)
    assert np.isclose(state_bdc.cylinder_side_thrust, 0.0, atol=1e-9)


def test_force_balance_along_connecting_rod():
    """Verify equilibrium: F_Q * cos(phi) == F_P, and F_N == F_P * tan(phi)."""
    params = SliderCrankParameters(crank_length=0.05, con_rod_length=0.15, offset=0.0, rpm=2000.0)
    kin_solver = KinematicSolver(params)
    dyn_solver = DynamicSolver(params)

    thetas = np.linspace(0.1, np.pi - 0.1, 50)
    kin = kin_solver.solve(thetas)
    dyn = dyn_solver.solve(thetas, kinematics=kin)

    # Check equilibrium relations
    phi = kin.con_rod_angle_rad
    assert np.allclose(dyn.con_rod_force * np.cos(phi), dyn.piston_effort, rtol=1e-5)
    assert np.allclose(dyn.cylinder_side_thrust, dyn.piston_effort * np.tan(phi), rtol=1e-5)
