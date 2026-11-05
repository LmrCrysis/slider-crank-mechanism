"""
Kinematic solver for Slider-Crank Mechanism.
Provides exact closed-form equations, series approximations,
and numerical finite-difference verification.
"""

from dataclasses import dataclass
from typing import Union
import numpy as np

from .mechanism import SliderCrankParameters


@dataclass
class KinematicState:
    """Kinematic results at a single crank angle or across an array of angles."""
    theta_rad: Union[float, np.ndarray]
    theta_deg: Union[float, np.ndarray]
    time_s: Union[float, np.ndarray]

    # Piston linear kinematics
    piston_position: Union[float, np.ndarray]       # x_B from crank axis O (m)
    piston_displacement: Union[float, np.ndarray]   # s from TDC (m)
    piston_velocity: Union[float, np.ndarray]       # v_B (m/s)
    piston_acceleration: Union[float, np.ndarray]   # a_B (m/s^2)

    # Connecting rod angular kinematics
    con_rod_angle_rad: Union[float, np.ndarray]     # phi (rad)
    con_rod_angle_deg: Union[float, np.ndarray]     # phi (deg)
    con_rod_ang_velocity: Union[float, np.ndarray]  # omega_c (rad/s)
    con_rod_ang_accel: Union[float, np.ndarray]     # alpha_c (rad/s^2)

    # Crank-pin kinematics
    crank_pin_x: Union[float, np.ndarray]
    crank_pin_y: Union[float, np.ndarray]
    crank_pin_vx: Union[float, np.ndarray]
    crank_pin_vy: Union[float, np.ndarray]
    crank_pin_ax: Union[float, np.ndarray]
    crank_pin_ay: Union[float, np.ndarray]

    # Classical approximations (for comparison / validation)
    approx_displacement: Union[float, np.ndarray]
    approx_velocity: Union[float, np.ndarray]
    approx_acceleration: Union[float, np.ndarray]


class KinematicSolver:
    """Computes closed-form exact and approximate kinematics for slider-crank."""

    def __init__(self, params: SliderCrankParameters):
        self.params = params

    def solve(self, theta: Union[float, np.ndarray]) -> KinematicState:
        """
        Evaluate full kinematic state at given crank angle(s) theta (in radians).
        Supports scalar float or NumPy array.
        """
        theta_arr = np.asarray(theta, dtype=float)
        is_scalar = theta_arr.ndim == 0
        if is_scalar:
            theta_arr = np.array([theta_arr])

        r = self.params.r
        l = self.params.l
        e = self.params.e
        omega = self.params.omega
        n = self.params.n

        # Crank pin Cartesian coordinates and derivatives
        cos_t = np.cos(theta_arr)
        sin_t = np.sin(theta_arr)

        x_A = r * cos_t
        y_A = r * sin_t
        vx_A = -r * omega * sin_t
        vy_A = r * omega * cos_t
        ax_A = -r * (omega**2) * cos_t
        ay_A = -r * (omega**2) * sin_t

        # Connecting rod angle phi
        # l * sin(phi) = r * sin(theta) - e
        sin_phi = (r * sin_t - e) / l
        # Clamp within [-1, 1] for safety
        sin_phi = np.clip(sin_phi, -1.0, 1.0)
        phi = np.arcsin(sin_phi)
        cos_phi = np.cos(phi)

        # Angular rate of change of phi w.r.t theta:
        # cos(phi) * (dphi/dtheta) = (r / l) * cos(theta)
        dphi_dtheta = (r * cos_t) / (l * cos_phi)
        omega_c = omega * dphi_dtheta

        # Angular acceleration of connecting rod
        # d^2phi/dtheta^2 = (r/l) * [ -sin(theta)*cos(phi) - cos(theta)*(-sin(phi)*dphi_dtheta) ] / cos^2(phi)
        d2phi_dtheta2 = (r / l) * (
            -sin_t * cos_phi + cos_t * sin_phi * dphi_dtheta
        ) / (cos_phi**2)
        alpha_c = (omega**2) * d2phi_dtheta2

        # Piston position x_B
        # x_B = r * cos(theta) + l * cos(phi)
        x_B = r * cos_t + l * cos_phi

        # Displacement from TDC
        x_tdc = self.params.x_tdc
        s_B = x_tdc - x_B

        # Exact piston velocity v_B = dx_B/dt = omega * dx_B/dtheta
        # dx_B/dtheta = -r * sin(theta) - l * sin(phi) * dphi_dtheta
        # note: l * sin(phi) * dphi_dtheta = l * sin(phi) * (r * cos(theta) / (l * cos(phi))) = r * tan(phi) * cos(theta)
        dx_dtheta = -r * sin_t - r * np.tan(phi) * cos_t
        v_B = omega * dx_dtheta

        # Exact piston acceleration a_B = dv_B/dt = omega^2 * d^2x_B/dtheta^2
        # d/dtheta[-r*sin(theta) - r*tan(phi)*cos(theta)]
        # = -r*cos(theta) - r [ sec^2(phi)*dphi_dtheta * cos(theta) - tan(phi)*sin(theta) ]
        d2x_dtheta2 = -r * cos_t - r * (
            (1.0 / (cos_phi**2)) * dphi_dtheta * cos_t - np.tan(phi) * sin_t
        )
        a_B = (omega**2) * d2x_dtheta2

        # Classical textbook approximations (for inline slider-crank)
        # s_approx = r * ( (1 - cos(theta)) + (sin(theta)^2)/(2*n) )
        # v_approx = -r * omega * ( sin(theta) + sin(2*theta)/(2*n) )
        # a_approx = -r * omega^2 * ( cos(theta) + cos(2*theta)/n )
        sin_2t = np.sin(2.0 * theta_arr)
        cos_2t = np.cos(2.0 * theta_arr)
        approx_s = r * ((1.0 - cos_t) + (sin_t**2) / (2.0 * n))
        approx_v = -r * omega * (sin_t + sin_2t / (2.0 * n))
        approx_a = -r * (omega**2) * (cos_t + cos_2t / n)

        # Time corresponding to theta
        time_arr = (theta_arr / omega) if omega > 0 else np.zeros_like(theta_arr)

        if is_scalar:
            return KinematicState(
                theta_rad=float(theta_arr[0]),
                theta_deg=float(np.degrees(theta_arr[0])),
                time_s=float(time_arr[0]),
                piston_position=float(x_B[0]),
                piston_displacement=float(s_B[0]),
                piston_velocity=float(v_B[0]),
                piston_acceleration=float(a_B[0]),
                con_rod_angle_rad=float(phi[0]),
                con_rod_angle_deg=float(np.degrees(phi[0])),
                con_rod_ang_velocity=float(omega_c[0]),
                con_rod_ang_accel=float(alpha_c[0]),
                crank_pin_x=float(x_A[0]),
                crank_pin_y=float(y_A[0]),
                crank_pin_vx=float(vx_A[0]),
                crank_pin_vy=float(vy_A[0]),
                crank_pin_ax=float(ax_A[0]),
                crank_pin_ay=float(ay_A[0]),
                approx_displacement=float(approx_s[0]),
                approx_velocity=float(approx_v[0]),
                approx_acceleration=float(approx_a[0]),
            )
        else:
            return KinematicState(
                theta_rad=theta_arr,
                theta_deg=np.degrees(theta_arr),
                time_s=time_arr,
                piston_position=x_B,
                piston_displacement=s_B,
                piston_velocity=v_B,
                piston_acceleration=a_B,
                con_rod_angle_rad=phi,
                con_rod_angle_deg=np.degrees(phi),
                con_rod_ang_velocity=omega_c,
                con_rod_ang_accel=alpha_c,
                crank_pin_x=x_A,
                crank_pin_y=y_A,
                crank_pin_vx=vx_A,
                crank_pin_vy=vy_A,
                crank_pin_ax=ax_A,
                crank_pin_ay=ay_A,
                approx_displacement=approx_s,
                approx_velocity=approx_v,
                approx_acceleration=approx_a,
            )

    def solve_full_cycle(self, num_points: int = 360) -> KinematicState:
        """Solves kinematics across one full 360-degree rotation (0 to 2*pi)."""
        theta_arr = np.linspace(0, 2.0 * np.pi, num_points, endpoint=False)
        return self.solve(theta_arr)

    def numerical_verification(self, num_points: int = 360, delta_theta: float = 1e-6):
        """
        Calculates numerical derivatives using high-accuracy central finite differences
        to validate the closed-form analytical equations.
        """
        thetas = np.linspace(0, 2.0 * np.pi, num_points, endpoint=False)
        exact_state = self.solve(thetas)

        # Central difference for velocity: dx/dt = omega * (x(theta + h) - x(theta - h)) / (2h)
        omega = self.params.omega
        h = delta_theta
        x_plus = self.solve(thetas + h).piston_position
        x_minus = self.solve(thetas - h).piston_position
        v_num = omega * (x_plus - x_minus) / (2.0 * h)

        # Central difference for acceleration: a = omega * (v(theta + h) - v(theta - h)) / (2h)
        v_plus = self.solve(thetas + h).piston_velocity
        v_minus = self.solve(thetas - h).piston_velocity
        a_num = omega * (v_plus - v_minus) / (2.0 * h)

        # Max errors
        max_vel_err = float(np.max(np.abs(exact_state.piston_velocity - v_num)))
        max_acc_err = float(np.max(np.abs(exact_state.piston_acceleration - a_num)))

        return {
            "thetas": thetas,
            "v_analytical": exact_state.piston_velocity,
            "v_numerical": v_num,
            "a_analytical": exact_state.piston_acceleration,
            "a_numerical": a_num,
            "max_velocity_error": max_vel_err,
            "max_acceleration_error": max_acc_err,
        }
