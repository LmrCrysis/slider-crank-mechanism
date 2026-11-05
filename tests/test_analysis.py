"""
Tests for full cycle analysis, flywheel sizing, and harmonic decomposition.
"""

import pytest
import numpy as np
from engine.mechanism import SliderCrankParameters
from engine.analysis import CycleAnalyzer
from engine.gas_pressure import GasPressureModel, EngineCycleType


def test_flywheel_sizing_and_harmonics():
    """Verify cycle analysis calculations and harmonic decomposition."""
    params = SliderCrankParameters(
        crank_length=0.045,
        con_rod_length=0.180,
        offset=0.0,
        rpm=3600.0,
        piston_mass=0.45,
        con_rod_mass=0.65,
        bore_diameter=0.075,
    )
    gas_model = GasPressureModel(params, cycle_type=EngineCycleType.POLYTROPIC_EXPANSION, peak_pressure_bar=50.0)
    analyzer = CycleAnalyzer(params, gas_model=gas_model)
    results = analyzer.analyze(num_points=720)

    # 1. Kinematics assertions
    assert results["mean_piston_speed"] > 0
    assert results["max_piston_velocity"] > results["mean_piston_speed"]
    assert results["max_piston_acceleration"] > 0

    # 2. Dynamics assertions
    assert results["max_gas_force"] > 0
    assert results["max_torque"] > results["mean_torque"]
    assert results["power_kw"] > 0

    # 3. Flywheel assertions
    flywheel = results["flywheel"]
    assert flywheel.required_inertia_kgm2 > 0
    assert flywheel.flywheel_mass_kg > 0
    assert flywheel.max_fluctuation_energy_j > 0

    # 4. Harmonics assertions
    assert len(results["harmonic_orders"]) == 8
    assert len(results["harmonic_amplitudes"]) == 8
    # Primary harmonic should be non-zero
    assert results["harmonic_amplitudes"][0] > 0


def test_approximation_convergence_with_obliquity_ratio():
    """
    Theoretical principle:
    As n = l / r increases (e.g. from n=2.5 to n=6.0),
    the error of the classical 2nd order series approximation decreases significantly!
    """
    from engine.kinematics import KinematicSolver

    errors = []
    ratios = [2.5, 3.5, 5.0, 7.0]
    for n in ratios:
        r = 0.05
        l = n * r
        params = SliderCrankParameters(crank_length=r, con_rod_length=l, rpm=1500.0)
        solver = KinematicSolver(params)
        state = solver.solve_full_cycle(360)
        max_acc_rel_error = np.max(np.abs(state.piston_acceleration - state.approx_acceleration)) / np.max(np.abs(state.piston_acceleration))
        errors.append(max_acc_rel_error)

    # Error must strictly monotonically decrease as n increases
    for i in range(len(errors) - 1):
        assert errors[i] > errors[i + 1], f"Error did not decrease: {errors[i]} vs {errors[i+1]}"
