"""
Dynamic force and torque analysis for Slider-Crank Mechanism.
Includes reciprocating inertia forces, gas forces, connecting rod forces,
cylinder side-thrust, crank effort, and turning moment (torque) diagrams.
"""

from dataclasses import dataclass
from typing import Union
import numpy as np

from .mechanism import SliderCrankParameters
from .kinematics import KinematicSolver, KinematicState
from .gas_pressure import GasPressureModel, EngineCycleType


@dataclass
class DynamicState:
    """Dynamic forces and torques at single crank angle or across an array of angles."""
    theta_rad: Union[float, np.ndarray]
    theta_deg: Union[float, np.ndarray]

    # Forces (Newtons)
    gas_force: Union[float, np.ndarray]              # F_g: Cylinder gas pressure force on piston
    inertia_force: Union[float, np.ndarray]          # F_I: Reciprocating inertia force (-m_rec * a)
    piston_effort: Union[float, np.ndarray]          # F_P: Net force driving the piston
    con_rod_force: Union[float, np.ndarray]          # F_Q: Force along connecting rod
    cylinder_side_thrust: Union[float, np.ndarray]   # F_N: Normal force on cylinder wall
    crank_effort: Union[float, np.ndarray]           # F_t: Tangential force at crank pin
    crank_radial_force: Union[float, np.ndarray]     # F_r: Radial force along crank arm

    # Torques (N*m)
    crank_torque: Union[float, np.ndarray]           # T: Turning moment on crankshaft

    # Shaking / Unbalance Forces (Newtons)
    primary_shaking_force: Union[float, np.ndarray]
    secondary_shaking_force: Union[float, np.ndarray]
    total_shaking_force: Union[float, np.ndarray]


class DynamicSolver:
    """Computes dynamic force equilibrium and turning moment for slider-crank."""

    def __init__(self, params: SliderCrankParameters, gas_model: GasPressureModel = None):
        self.params = params
        self.kinematic_solver = KinematicSolver(params)
        self.gas_model = gas_model or GasPressureModel(params, EngineCycleType.POLYTROPIC_EXPANSION)

    def solve(
        self,
        theta: Union[float, np.ndarray],
        kinematics: KinematicState = None,
        custom_gas_force: Union[float, np.ndarray] = None,
    ) -> DynamicState:
        """
        Evaluate dynamic forces and turning moment at given crank angle(s) theta (radians).
        """
        if kinematics is None:
            kinematics = self.kinematic_solver.solve(theta)

        thetas = np.asarray(theta, dtype=float)
        is_scalar = thetas.ndim == 0
        thetas_1d = np.atleast_1d(thetas)

        m_rec = self.params.equivalent_reciprocating_mass
        r = self.params.r
        omega = self.params.omega
        n = self.params.n

        # Gas Force F_g = P_net * Area
        if custom_gas_force is not None:
            F_g = np.atleast_1d(np.asarray(custom_gas_force, dtype=float))
        else:
            p_net = self.gas_model.get_pressure(thetas_1d)
            F_g = np.atleast_1d(p_net * self.params.piston_area)

        # Piston acceleration a_B
        a_B = np.atleast_1d(np.asarray(kinematics.piston_acceleration, dtype=float))

        # Inertia force of reciprocating parts: F_I = -m_rec * a_B
        # When a_B is negative (towards crank), F_I is positive (opposes acceleration)
        # In IC engine convention, net driving piston effort:
        # F_P = F_g - m_rec * a_reciprocating_towards_crank = F_g + F_I_forward
        # Since x_B decreases towards crank, a_towards_crank = -a_B.
        # F_I_towards_crank = -m_rec * (-a_B) = m_rec * a_B.
        # Thus F_P = F_g + m_rec * a_B
        F_I = m_rec * a_B
        F_P = F_g + F_I

        # Connecting rod angle phi
        phi = np.atleast_1d(np.asarray(kinematics.con_rod_angle_rad, dtype=float))
        cos_phi = np.cos(phi)
        # Avoid division by zero
        cos_phi = np.where(np.abs(cos_phi) < 1e-6, 1e-6, cos_phi)

        # Force along connecting rod: F_Q = F_P / cos(phi)
        F_Q = F_P / cos_phi

        # Normal thrust on cylinder wall: F_N = F_P * tan(phi)
        F_N = F_P * np.tan(phi)

        # Tangential crank effort: F_t = F_Q * sin(theta + phi)
        # = F_P * sin(theta + phi) / cos(phi)
        F_t = F_P * np.sin(thetas_1d + phi) / cos_phi

        # Radial bearing load on crank arm: F_r = F_Q * cos(theta + phi)
        F_r = F_P * np.cos(thetas_1d + phi) / cos_phi

        # Crankshaft turning moment / torque: T = F_t * r
        T = F_t * r

        # Shaking forces (unbalanced reciprocating forces along line of stroke)
        F_shake_primary = m_rec * r * (omega**2) * np.cos(thetas_1d)
        F_shake_secondary = m_rec * r * (omega**2) * np.cos(2.0 * thetas_1d) / n
        F_shake_total = F_shake_primary + F_shake_secondary

        if is_scalar:
            return DynamicState(
                theta_rad=float(thetas_1d[0]),
                theta_deg=float(np.degrees(thetas_1d[0])),
                gas_force=float(F_g[0]),
                inertia_force=float(F_I[0]),
                piston_effort=float(F_P[0]),
                con_rod_force=float(F_Q[0]),
                cylinder_side_thrust=float(F_N[0]),
                crank_effort=float(F_t[0]),
                crank_radial_force=float(F_r[0]),
                crank_torque=float(T[0]),
                primary_shaking_force=float(F_shake_primary[0]),
                secondary_shaking_force=float(F_shake_secondary[0]),
                total_shaking_force=float(F_shake_total[0]),
            )
        else:
            return DynamicState(
                theta_rad=thetas,
                theta_deg=np.degrees(thetas),
                gas_force=F_g,
                inertia_force=F_I,
                piston_effort=F_P,
                con_rod_force=F_Q,
                cylinder_side_thrust=F_N,
                crank_effort=F_t,
                crank_radial_force=F_r,
                crank_torque=T,
                primary_shaking_force=F_shake_primary,
                secondary_shaking_force=F_shake_secondary,
                total_shaking_force=F_shake_total,
            )

    def solve_full_cycle(self, num_points: int = 360) -> DynamicState:
        """Solves dynamics across one full 360-degree rotation (0 to 2*pi)."""
        theta_arr = np.linspace(0, 2.0 * np.pi, num_points, endpoint=False)
        kinematics = self.kinematic_solver.solve(theta_arr)
        return self.solve(theta_arr, kinematics=kinematics)
