"""
Matplotlib-based 2D visual schematic renderer for Slider-Crank animation.
Renders links, joints, piston, cylinder walls, traces, and vector overlays.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon
from engine.mechanism import SliderCrankParameters, MechanismGeometry
from engine.kinematics import KinematicSolver, KinematicState
from engine.dynamics import DynamicSolver, DynamicState


class MechanismCanvas:
    """Draws a mechanical engineering 2D schematic of the slider-crank."""

    def __init__(self, fig, ax, params: SliderCrankParameters):
        self.fig = fig
        self.ax = ax
        self.params = params
        self.geom = MechanismGeometry(params)
        self.kin_solver = KinematicSolver(params)
        self.dyn_solver = DynamicSolver(params)

        # Plot elements
        self.crank_line = None
        self.rod_line = None
        self.piston_patch = None
        self.crank_circle = None
        self.cg_trace = None
        self.vel_vector = None
        self.acc_vector = None
        self.status_text = None

        self._setup_canvas()

    def update_params(self, params: SliderCrankParameters):
        self.params = params
        self.geom = MechanismGeometry(params)
        self.kin_solver = KinematicSolver(params)
        self.dyn_solver = DynamicSolver(params)
        self._setup_canvas()

    def _setup_canvas(self):
        self.ax.clear()
        self.ax.set_aspect("equal", adjustable="datalim")
        self.ax.grid(True, linestyle="--", alpha=0.4)

        r = self.params.r
        l = self.params.l
        e = self.params.e
        piston_w = max(self.params.slider_width, r * 0.8)
        piston_h = max(self.params.slider_height, r * 0.6)

        # Set plot limits with padding
        x_min = -r * 1.5
        x_max = (r + l) * 1.25 + piston_w
        y_max = max(r * 1.6, abs(e) + piston_h * 1.5)
        y_min = min(-r * 1.6, e - piston_h * 1.5)

        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)
        self.ax.set_xlabel("X - Coordinate (meters)", fontsize=10, fontweight="bold")
        self.ax.set_ylabel("Y - Coordinate (meters)", fontsize=10, fontweight="bold")
        self.ax.set_title("Slider-Crank Mechanism Kinematics & Dynamic Motion", fontsize=11, fontweight="bold", pad=10)

        # Static elements:
        # Crank path circle (trajectory of crank pin A)
        circle_theta = np.linspace(0, 2.0 * np.pi, 100)
        self.ax.plot(r * np.cos(circle_theta), r * np.sin(circle_theta), "k--", alpha=0.3, label="Crank Orbit")

        # Coupler CG path trace
        thetas_trace = np.linspace(0, 2.0 * np.pi, 120)
        cg_x = []
        cg_y = []
        for t in thetas_trace:
            pos = self.geom.get_joint_positions(t)
            cg_x.append(pos["G"][0])
            cg_y.append(pos["G"][1])
        self.ax.plot(cg_x, cg_y, color="#9C27B0", linestyle=":", alpha=0.6, label="Con-Rod CG Path")

        # Line of stroke (cylinder axis)
        self.ax.axhline(e, color="#607D8B", linestyle="-.", alpha=0.5, label="Stroke Axis")

        # Cylinder walls
        x_tdc = self.params.x_tdc
        x_bdc = self.params.x_bdc
        cyl_x_start = x_bdc - piston_w * 0.5
        cyl_x_end = x_tdc + piston_w * 1.2
        cyl_y_top = e + piston_h * 0.58
        cyl_y_bot = e - piston_h * 0.58

        self.ax.plot([cyl_x_start, cyl_x_end], [cyl_y_top, cyl_y_top], color="#37474F", linewidth=3)
        self.ax.plot([cyl_x_start, cyl_x_end], [cyl_y_bot, cyl_y_bot], color="#37474F", linewidth=3)

        # Cylinder crosshatch
        hatch_count = 12
        hatch_xs = np.linspace(cyl_x_start, cyl_x_end, hatch_count)
        for hx in hatch_xs:
            self.ax.plot([hx, hx + 0.015], [cyl_y_top, cyl_y_top + 0.015], color="#546E7A", linewidth=1.2)
            self.ax.plot([hx, hx + 0.015], [cyl_y_bot, cyl_y_bot - 0.015], color="#546E7A", linewidth=1.2)

        # Ground mount at O (0, 0)
        self.ax.plot([0, -0.015, 0.015, 0], [0, -0.02, -0.02, 0], color="#263238", linewidth=2)
        self.ax.plot([-0.025, 0.025], [-0.02, -0.02], color="#263238", linewidth=2)

        # Dynamic lines / patches
        (self.crank_line,) = self.ax.plot([], [], color="#D32F2F", linewidth=5, solid_capstyle="round", label="Crank (Link 2)")
        (self.rod_line,) = self.ax.plot([], [], color="#1976D2", linewidth=4, solid_capstyle="round", label="Con-Rod (Link 3)")
        (self.joint_o,) = self.ax.plot([0], [0], "o", color="#212121", markersize=9, zorder=5)
        (self.joint_a,) = self.ax.plot([], [], "o", color="#FF5722", markersize=8, zorder=6)
        (self.joint_b,) = self.ax.plot([], [], "o", color="#009688", markersize=8, zorder=6)
        (self.joint_g,) = self.ax.plot([], [], "x", color="#E91E63", markersize=8, markeredgewidth=2, zorder=6, label="Rod CG (G)")

        # Piston rectangle patch
        self.piston_patch = Rectangle(
            (0, 0), piston_w, piston_h,
            facecolor="#90A4AE", edgecolor="#263238", linewidth=2, zorder=4
        )
        self.ax.add_patch(self.piston_patch)

        # Vector annotations
        self.vel_quiver = None
        self.acc_quiver = None

        # Text readouts
        self.status_text = self.ax.text(
            0.02, 0.95, "", transform=self.ax.transAxes,
            fontsize=9, verticalalignment="top",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="#ECEFF1", edgecolor="#90A4AE", alpha=0.9)
        )

        self.ax.legend(loc="lower left", fontsize=8, framealpha=0.85)

    def draw_state(self, theta_rad: float):
        """Updates canvas to current crank angle (radians)."""
        pos = self.geom.get_joint_positions(theta_rad)
        O = pos["O"]
        A = pos["A"]
        B = pos["B"]
        G = pos["G"]

        # Update link coordinates
        self.crank_line.set_data([O[0], A[0]], [O[1], A[1]])
        self.rod_line.set_data([A[0], B[0]], [A[1], B[1]])

        self.joint_a.set_data([A[0]], [A[1]])
        self.joint_b.set_data([B[0]], [B[1]])
        self.joint_g.set_data([G[0]], [G[1]])

        # Update piston position
        piston_w = max(self.params.slider_width, self.params.r * 0.8)
        piston_h = max(self.params.slider_height, self.params.r * 0.6)
        # Center wrist pin B inside the piston block
        self.piston_patch.set_xy((B[0] - piston_w * 0.5, B[1] - piston_h * 0.5))

        # Kinematic and dynamic values
        kin = self.kin_solver.solve(theta_rad)
        dyn = self.dyn_solver.solve(theta_rad, kinematics=kin)

        # Update status text
        theta_deg = np.degrees(theta_rad) % 360.0
        disp_mm = kin.piston_displacement * 1000.0
        vel_ms = kin.piston_velocity
        acc_ms2 = kin.piston_acceleration
        torque_nm = dyn.crank_torque
        thrust_n = dyn.cylinder_side_thrust

        info_str = (
            f"Crank Angle θ: {theta_deg:6.1f}°\n"
            f"Piston Disp s: {disp_mm:6.1f} mm\n"
            f"Velocity v:    {vel_ms:6.2f} m/s\n"
            f"Accel a:       {acc_ms2:7.1f} m/s²\n"
            f"Torque T:      {torque_nm:6.1f} N·m\n"
            f"Side Thrust:   {thrust_n:6.1f} N"
        )
        self.status_text.set_text(info_str)
