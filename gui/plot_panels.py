"""
Multi-tab Matplotlib performance plot panels for slider-crank analysis.
Includes Kinematics, Dynamics, Turning Moment Diagrams, and Harmonic Spectrum.
"""

import numpy as np
import matplotlib.pyplot as plt
from engine.mechanism import SliderCrankParameters
from engine.analysis import CycleAnalyzer
from engine.gas_pressure import GasPressureModel, EngineCycleType


class PlotPanels:
    """Manages multi-view graphical analysis plots."""

    def __init__(self, fig, params: SliderCrankParameters, gas_model: GasPressureModel = None):
        self.fig = fig
        self.params = params
        self.gas_model = gas_model
        self.analyzer = CycleAnalyzer(params, gas_model=gas_model)
        self.current_mode = "kinematics"
        self.indicator_lines = []

    def update_params(self, params: SliderCrankParameters, gas_model: GasPressureModel = None):
        self.params = params
        if gas_model is not None:
            self.gas_model = gas_model
        self.analyzer = CycleAnalyzer(self.params, gas_model=self.gas_model)

    def plot_kinematics(self, current_theta_deg: float = 0.0):
        """Generates 3-row kinematics subplot (displacement, velocity, acceleration)."""
        self.fig.clf()
        self.current_mode = "kinematics"
        self.indicator_lines.clear()

        results = self.analyzer.analyze(num_points=360)
        deg = results["theta_deg"]
        kin = results["kinematics"]

        axs = self.fig.subplots(3, 1, sharex=True)
        self.fig.subplots_adjust(hspace=0.25, left=0.12, right=0.96, top=0.93, bottom=0.10)

        # 1. Displacement
        ax1 = axs[0]
        ax1.plot(deg, kin.piston_displacement * 1000.0, "b-", linewidth=2, label="Exact Closed-Form")
        ax1.plot(deg, kin.approx_displacement * 1000.0, "r--", linewidth=1.5, alpha=0.8, label="Series Approx")
        ax1.set_ylabel("Disp. (mm)", fontsize=9, fontweight="bold")
        ax1.grid(True, linestyle="--", alpha=0.4)
        ax1.legend(loc="upper right", fontsize=8)
        ax1.set_title("Kinematic Analysis: Displacement, Velocity & Acceleration vs Crank Angle", fontsize=10, fontweight="bold")
        l1 = ax1.axvline(current_theta_deg, color="#E91E63", linestyle=":", linewidth=2)
        self.indicator_lines.append(l1)

        # 2. Velocity
        ax2 = axs[1]
        ax2.plot(deg, kin.piston_velocity, "g-", linewidth=2, label="Exact Velocity")
        ax2.plot(deg, kin.approx_velocity, "r--", linewidth=1.5, alpha=0.8, label="Approx Velocity")
        ax2.axhline(0, color="k", linewidth=0.8, linestyle="-")
        ax2.set_ylabel("Vel. (m/s)", fontsize=9, fontweight="bold")
        ax2.grid(True, linestyle="--", alpha=0.4)
        l2 = ax2.axvline(current_theta_deg, color="#E91E63", linestyle=":", linewidth=2)
        self.indicator_lines.append(l2)

        # 3. Acceleration
        ax3 = axs[2]
        ax3.plot(deg, kin.piston_acceleration, "m-", linewidth=2, label="Exact Accel.")
        ax3.plot(deg, kin.approx_acceleration, "r--", linewidth=1.5, alpha=0.8, label="Approx Accel.")
        ax3.axhline(0, color="k", linewidth=0.8, linestyle="-")
        ax3.set_xlabel("Crank Angle θ (degrees)", fontsize=10, fontweight="bold")
        ax3.set_ylabel("Accel. (m/s²)", fontsize=9, fontweight="bold")
        ax3.set_xlim(0, 360)
        ax3.set_xticks(np.arange(0, 361, 45))
        ax3.grid(True, linestyle="--", alpha=0.4)
        l3 = ax3.axvline(current_theta_deg, color="#E91E63", linestyle=":", linewidth=2)
        self.indicator_lines.append(l3)

    def plot_dynamics(self, current_theta_deg: float = 0.0):
        """Generates dynamic force variation subplots."""
        self.fig.clf()
        self.current_mode = "dynamics"
        self.indicator_lines.clear()

        results = self.analyzer.analyze(num_points=360)
        deg = results["theta_deg"]
        dyn = results["dynamics"]

        axs = self.fig.subplots(2, 1, sharex=True)
        self.fig.subplots_adjust(hspace=0.25, left=0.12, right=0.96, top=0.93, bottom=0.10)

        # 1. Piston Driving Forces
        ax1 = axs[0]
        ax1.plot(deg, dyn.gas_force / 1000.0, color="#E65100", linewidth=2, label="Gas Force Fg")
        ax1.plot(deg, dyn.inertia_force / 1000.0, color="#0288D1", linewidth=2, label="Recip. Inertia FI")
        ax1.plot(deg, dyn.piston_effort / 1000.0, color="#2E7D32", linewidth=2.2, linestyle="-", label="Piston Effort FP")
        ax1.axhline(0, color="k", linewidth=0.8)
        ax1.set_ylabel("Force (kN)", fontsize=9, fontweight="bold")
        ax1.set_title("Dynamic Force Variation: Piston Effort & Cylinder Side Thrust", fontsize=10, fontweight="bold")
        ax1.grid(True, linestyle="--", alpha=0.4)
        ax1.legend(loc="upper right", fontsize=8)
        l1 = ax1.axvline(current_theta_deg, color="#E91E63", linestyle=":", linewidth=2)
        self.indicator_lines.append(l1)

        # 2. Side Thrust & Con-Rod Force
        ax2 = axs[1]
        ax2.plot(deg, dyn.cylinder_side_thrust / 1000.0, color="#D81B60", linewidth=2, label="Side Thrust FN (kN)")
        ax2.plot(deg, dyn.con_rod_force / 1000.0, color="#5E35B1", linewidth=1.8, linestyle="--", label="Con-Rod Force FQ (kN)")
        ax2.axhline(0, color="k", linewidth=0.8)
        ax2.set_xlabel("Crank Angle θ (degrees)", fontsize=10, fontweight="bold")
        ax2.set_ylabel("Force (kN)", fontsize=9, fontweight="bold")
        ax2.set_xlim(0, 360)
        ax2.set_xticks(np.arange(0, 361, 45))
        ax2.grid(True, linestyle="--", alpha=0.4)
        ax2.legend(loc="upper right", fontsize=8)
        l2 = ax2.axvline(current_theta_deg, color="#E91E63", linestyle=":", linewidth=2)
        self.indicator_lines.append(l2)

    def plot_tmd(self, current_theta_deg: float = 0.0):
        """Generates Turning Moment Diagram (TMD) and energy fluctuations."""
        self.fig.clf()
        self.current_mode = "tmd"
        self.indicator_lines.clear()

        results = self.analyzer.analyze(num_points=360)
        deg = results["theta_deg"]
        dyn = results["dynamics"]
        t_mean = results["mean_torque"]
        power_kw = results["power_kw"]
        delta_e = results["delta_e"]

        axs = self.fig.subplots(2, 1, sharex=True)
        self.fig.subplots_adjust(hspace=0.25, left=0.12, right=0.96, top=0.93, bottom=0.10)

        # 1. Crank Torque & Mean Torque
        ax1 = axs[0]
        ax1.plot(deg, dyn.crank_torque, color="#C2185B", linewidth=2.2, label="Crankshaft Torque T(θ)")
        ax1.axhline(t_mean, color="#1976D2", linewidth=2, linestyle="--", label=f"Mean Torque T_mean ({t_mean:.1f} N·m)")
        ax1.fill_between(deg, dyn.crank_torque, t_mean, where=(dyn.crank_torque >= t_mean), color="#4CAF50", alpha=0.3, label="Excess Energy (+)")
        ax1.fill_between(deg, dyn.crank_torque, t_mean, where=(dyn.crank_torque < t_mean), color="#F44336", alpha=0.3, label="Deficit Energy (-)")
        ax1.set_ylabel("Torque (N·m)", fontsize=9, fontweight="bold")
        ax1.set_title(f"Turning Moment Diagram (TMD) — Power Output: {power_kw:.2f} kW | ΔE: {delta_e:.1f} J", fontsize=10, fontweight="bold")
        ax1.grid(True, linestyle="--", alpha=0.4)
        ax1.legend(loc="upper right", fontsize=8)
        l1 = ax1.axvline(current_theta_deg, color="#E91E63", linestyle=":", linewidth=2)
        self.indicator_lines.append(l1)

        # 2. Cumulative Fluctuation of Energy
        ax2 = axs[1]
        ax2.plot(deg, results["energy_diagram"], color="#303F9F", linewidth=2, label="Energy Variation ∫(T - T_mean)dθ")
        ax2.axhline(0, color="k", linewidth=0.8)
        ax2.set_xlabel("Crank Angle θ (degrees)", fontsize=10, fontweight="bold")
        ax2.set_ylabel("Energy (Joules)", fontsize=9, fontweight="bold")
        ax2.set_xlim(0, 360)
        ax2.set_xticks(np.arange(0, 361, 45))
        ax2.grid(True, linestyle="--", alpha=0.4)
        ax2.legend(loc="upper right", fontsize=8)
        l2 = ax2.axvline(current_theta_deg, color="#E91E63", linestyle=":", linewidth=2)
        self.indicator_lines.append(l2)

    def plot_harmonics(self):
        """Generates Fourier harmonic orders and polar torque diagram."""
        self.fig.clf()
        self.current_mode = "harmonics"
        self.indicator_lines.clear()

        results = self.analyzer.analyze(num_points=360)

        # 2 Subplots: left bar chart, right polar
        ax1 = self.fig.add_subplot(1, 2, 1)
        ax2 = self.fig.add_subplot(1, 2, 2, projection="polar")
        self.fig.subplots_adjust(left=0.08, right=0.95, top=0.90, bottom=0.15, wspace=0.3)

        # Harmonic Bar Chart
        orders = results["harmonic_orders"]
        amps = results["harmonic_amplitudes"]
        bars = ax1.bar([f"{k}θ" for k in orders], amps, color="#3F51B5", edgecolor="#1A237E", alpha=0.85)
        ax1.set_xlabel("Harmonic Order (Engine Vibration)", fontsize=9, fontweight="bold")
        ax1.set_ylabel("Torque Amplitude (N·m)", fontsize=9, fontweight="bold")
        ax1.set_title("Harmonic Fourier Decomposition of Crank Torque", fontsize=10, fontweight="bold")
        ax1.grid(True, axis="y", linestyle="--", alpha=0.5)

        for bar in bars:
            height = bar.get_height()
            if height > 0.5:
                ax1.annotate(f"{height:.1f}",
                    xy=(bar.get_x() + bar.get_width() / 2, height),
                    xytext=(0, 3), textcoords="offset points",
                    ha='center', va='bottom', fontsize=8)

        # Polar Torque Diagram
        thetas_rad = results["thetas"]
        torques = results["dynamics"].crank_torque
        # Offset negative torques for polar visualization
        r_vals = np.maximum(torques, 0)
        ax2.plot(thetas_rad, r_vals, color="#E91E63", linewidth=2.2)
        ax2.fill(thetas_rad, r_vals, color="#F8BBD0", alpha=0.5)
        ax2.set_theta_zero_location("E")
        ax2.set_title("Polar Torque Diagram (T vs θ)", fontsize=10, fontweight="bold", pad=15)

    def update_cursor(self, current_theta_deg: float):
        """Moves cursor lines smoothly during live animation without redrawing entire plot."""
        for line in self.indicator_lines:
            try:
                line.set_xdata([current_theta_deg, current_theta_deg])
            except Exception:
                pass
