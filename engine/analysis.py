"""
Comprehensive cycle analysis, Turning Moment Diagram (TMD) integration,
flywheel sizing, and harmonic Fourier analysis.
"""

from dataclasses import dataclass
from typing import Dict, Any
import numpy as np

from .mechanism import SliderCrankParameters
from .kinematics import KinematicSolver, KinematicState
from .dynamics import DynamicSolver, DynamicState
from .gas_pressure import GasPressureModel


@dataclass
class FlywheelDesign:
    """Calculated flywheel specifications to smooth torque fluctuations."""
    mean_torque_nm: float
    work_per_cycle_j: float
    max_fluctuation_energy_j: float      # Delta E
    coeff_fluctuation_energy: float      # C_e = Delta E / Work
    coeff_fluctuation_speed: float       # C_s (e.g. 0.02 for 2% speed variation)
    required_inertia_kgm2: float         # I = Delta E / (C_s * omega^2)
    flywheel_rim_radius_m: float
    flywheel_mass_kg: float


class CycleAnalyzer:
    """Performs deep thermodynamic, mechanical, and vibration analysis of the cycle."""

    def __init__(self, params: SliderCrankParameters, gas_model: GasPressureModel = None):
        self.params = params
        self.kin_solver = KinematicSolver(params)
        self.dyn_solver = DynamicSolver(params, gas_model=gas_model)

    def analyze(self, num_points: int = 720) -> Dict[str, Any]:
        """
        Runs full 360-degree simulation and computes performance metrics,
        energy balance, flywheel requirements, and harmonic components.
        """
        thetas = np.linspace(0, 2.0 * np.pi, num_points, endpoint=False)
        d_theta = thetas[1] - thetas[0]

        kin = self.kin_solver.solve(thetas)
        dyn = self.dyn_solver.solve(thetas, kinematics=kin)

        # 1. Piston Speeds
        stroke = self.params.stroke
        rpm = self.params.rpm
        mean_piston_speed = 2.0 * stroke * (rpm / 60.0)
        max_piston_velocity = float(np.max(np.abs(kin.piston_velocity)))
        max_piston_acceleration = float(np.max(np.abs(kin.piston_acceleration)))
        theta_max_vel_deg = float(kin.theta_deg[np.argmax(np.abs(kin.piston_velocity))])
        theta_max_acc_deg = float(kin.theta_deg[np.argmax(np.abs(kin.piston_acceleration))])

        # 2. Turning Moment Diagram & Work
        torques = dyn.crank_torque
        trapz_func = getattr(np, "trapezoid", getattr(np, "trapz", None))
        mean_torque = float(trapz_func(torques, thetas) / (2.0 * np.pi))
        work_per_cycle = float(2.0 * np.pi * mean_torque)
        power_kw = float((mean_torque * self.params.omega) / 1000.0)

        # Fluctuation of energy Delta E
        # Integrate (T - T_mean) dtheta
        delta_t = torques - mean_torque
        energy_integral = np.cumsum(delta_t) * d_theta
        # Center around zero
        energy_integral -= np.mean(energy_integral)
        max_e = float(np.max(energy_integral))
        min_e = float(np.min(energy_integral))
        delta_e = max_e - min_e

        # Coefficient of fluctuation of energy
        c_e = delta_e / abs(work_per_cycle) if abs(work_per_cycle) > 1e-3 else 0.0

        # Flywheel design for C_s = 0.02 (2% fluctuation)
        c_s = 0.02
        omega = self.params.omega
        i_req = delta_e / (c_s * (omega**2)) if (omega > 0 and delta_e > 0) else 0.0
        rim_radius = self.params.r * 2.5  # practical default
        flywheel_mass = i_req / (rim_radius**2) if rim_radius > 0 else 0.0

        flywheel = FlywheelDesign(
            mean_torque_nm=mean_torque,
            work_per_cycle_j=work_per_cycle,
            max_fluctuation_energy_j=delta_e,
            coeff_fluctuation_energy=c_e,
            coeff_fluctuation_speed=c_s,
            required_inertia_kgm2=i_req,
            flywheel_rim_radius_m=rim_radius,
            flywheel_mass_kg=flywheel_mass,
        )

        # 3. Peak Forces
        max_gas_force = float(np.max(dyn.gas_force))
        max_inertia_force = float(np.max(np.abs(dyn.inertia_force)))
        max_side_thrust = float(np.max(np.abs(dyn.cylinder_side_thrust)))
        max_con_rod_force = float(np.max(np.abs(dyn.con_rod_force)))
        max_torque = float(np.max(torques))
        min_torque = float(np.min(torques))

        # 4. Harmonic Fourier Analysis of Crank Torque
        # First 8 harmonics
        fft_vals = np.fft.rfft(torques) / len(torques)
        harmonic_orders = np.arange(1, min(9, len(fft_vals)))
        harmonic_amplitudes = [float(2.0 * np.abs(fft_vals[k])) for k in harmonic_orders]

        return {
            "parameters": self.params,
            "kinematics": kin,
            "dynamics": dyn,
            "thetas": thetas,
            "theta_deg": kin.theta_deg,
            "mean_piston_speed": mean_piston_speed,
            "max_piston_velocity": max_piston_velocity,
            "max_piston_acceleration": max_piston_acceleration,
            "theta_max_vel_deg": theta_max_vel_deg,
            "theta_max_acc_deg": theta_max_acc_deg,
            "mean_torque": mean_torque,
            "max_torque": max_torque,
            "min_torque": min_torque,
            "power_kw": power_kw,
            "work_per_cycle": work_per_cycle,
            "delta_e": delta_e,
            "max_gas_force": max_gas_force,
            "max_inertia_force": max_inertia_force,
            "max_side_thrust": max_side_thrust,
            "max_con_rod_force": max_con_rod_force,
            "flywheel": flywheel,
            "energy_diagram": energy_integral,
            "harmonic_orders": harmonic_orders.tolist(),
            "harmonic_amplitudes": harmonic_amplitudes,
        }
