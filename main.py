"""
Main Application and Command-Line Interface (CLI) for Slider-Crank Simulation.
Author: Dhruv Chauhan (24JE0497)
"""

import os
import sys
import argparse
import tempfile

# Ensure stdout handles Unicode or fall back cleanly
if sys.platform.startswith("win"):
    try:
        sys.stdout.reconfigure(encoding="utf-8")
    except Exception:
        pass

# Ensure Matplotlib uses a writable config dir
os.environ.setdefault("MPLCONFIGDIR", os.path.join(tempfile.gettempdir(), "matplotlib_cache"))

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from engine.mechanism import SliderCrankParameters, MechanismGeometry
from engine.kinematics import KinematicSolver
from engine.dynamics import DynamicSolver
from engine.gas_pressure import GasPressureModel, EngineCycleType
from engine.analysis import CycleAnalyzer


def save_all_plots(params: SliderCrankParameters, gas_model: GasPressureModel, output_dir: str = "reports"):
    """Generates and saves publication-quality 300 DPI figures."""
    os.makedirs(output_dir, exist_ok=True)
    analyzer = CycleAnalyzer(params, gas_model=gas_model)
    results = analyzer.analyze(num_points=720)
    deg = results["theta_deg"]
    kin = results["kinematics"]
    dyn = results["dynamics"]

    # 1. Kinematics Plot
    fig1, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig1.subplots_adjust(hspace=0.25)
    axs[0].plot(deg, kin.piston_displacement * 1000.0, "b-", linewidth=2, label="Exact Closed-Form")
    axs[0].plot(deg, kin.approx_displacement * 1000.0, "r--", linewidth=1.5, label="Classical Approx")
    axs[0].set_ylabel("Displacement (mm)", fontweight="bold")
    axs[0].grid(True, linestyle="--", alpha=0.5)
    axs[0].legend(loc="upper right")
    axs[0].set_title(f"Kinematic Analysis — Crank r={params.r*1000:.1f} mm, Rod l={params.l*1000:.1f} mm, {params.rpm:.0f} RPM", fontweight="bold")

    axs[1].plot(deg, kin.piston_velocity, "g-", linewidth=2, label="Exact Velocity")
    axs[1].plot(deg, kin.approx_velocity, "r--", linewidth=1.5, label="Approx Velocity")
    axs[1].axhline(0, color="k", linewidth=0.8)
    axs[1].set_ylabel("Velocity (m/s)", fontweight="bold")
    axs[1].grid(True, linestyle="--", alpha=0.5)
    axs[1].legend(loc="upper right")

    axs[2].plot(deg, kin.piston_acceleration, "m-", linewidth=2, label="Exact Acceleration")
    axs[2].plot(deg, kin.approx_acceleration, "r--", linewidth=1.5, label="Approx Acceleration")
    axs[2].axhline(0, color="k", linewidth=0.8)
    axs[2].set_xlabel("Crank Angle θ (degrees)", fontweight="bold")
    axs[2].set_ylabel("Acceleration (m/s²)", fontweight="bold")
    axs[2].set_xlim(0, 360)
    axs[2].set_xticks(np.arange(0, 361, 45))
    axs[2].grid(True, linestyle="--", alpha=0.5)
    axs[2].legend(loc="upper right")

    kin_path = os.path.join(output_dir, "kinematics_analysis.png")
    fig1.savefig(kin_path, dpi=300, bbox_inches="tight")
    plt.close(fig1)

    # 2. Dynamics Forces Plot
    fig2, (ax_f1, ax_f2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    fig2.subplots_adjust(hspace=0.25)
    ax_f1.plot(deg, dyn.gas_force / 1000.0, color="#E65100", linewidth=2, label="Gas Force Fg")
    ax_f1.plot(deg, dyn.inertia_force / 1000.0, color="#0288D1", linewidth=2, label="Inertia Force FI")
    ax_f1.plot(deg, dyn.piston_effort / 1000.0, color="#2E7D32", linewidth=2.5, label="Piston Effort FP")
    ax_f1.axhline(0, color="k", linewidth=0.8)
    ax_f1.set_ylabel("Force (kN)", fontweight="bold")
    ax_f1.set_title("Dynamic Forces on Reciprocating Components", fontweight="bold")
    ax_f1.grid(True, linestyle="--", alpha=0.5)
    ax_f1.legend(loc="upper right")

    ax_f2.plot(deg, dyn.cylinder_side_thrust / 1000.0, color="#D81B60", linewidth=2, label="Cylinder Side Thrust FN")
    ax_f2.plot(deg, dyn.con_rod_force / 1000.0, color="#5E35B1", linewidth=2, linestyle="--", label="Connecting Rod Force FQ")
    ax_f2.axhline(0, color="k", linewidth=0.8)
    ax_f2.set_xlabel("Crank Angle θ (degrees)", fontweight="bold")
    ax_f2.set_ylabel("Force (kN)", fontweight="bold")
    ax_f2.set_xlim(0, 360)
    ax_f2.set_xticks(np.arange(0, 361, 45))
    ax_f2.grid(True, linestyle="--", alpha=0.5)
    ax_f2.legend(loc="upper right")

    dyn_path = os.path.join(output_dir, "dynamics_forces.png")
    fig2.savefig(dyn_path, dpi=300, bbox_inches="tight")
    plt.close(fig2)

    # 3. Turning Moment Diagram (TMD) Plot
    fig3, (ax_t1, ax_t2) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    fig3.subplots_adjust(hspace=0.25)
    t_mean = results["mean_torque"]
    ax_t1.plot(deg, dyn.crank_torque, color="#C2185B", linewidth=2.2, label="Turning Moment T(θ)")
    ax_t1.axhline(t_mean, color="#1976D2", linewidth=2, linestyle="--", label=f"Mean Torque = {t_mean:.1f} N·m")
    ax_t1.fill_between(deg, dyn.crank_torque, t_mean, where=(dyn.crank_torque >= t_mean), color="#4CAF50", alpha=0.35, label="Excess Work")
    ax_t1.fill_between(deg, dyn.crank_torque, t_mean, where=(dyn.crank_torque < t_mean), color="#F44336", alpha=0.35, label="Deficit Work")
    ax_t1.set_ylabel("Torque (N·m)", fontweight="bold")
    ax_t1.set_title(f"Turning Moment Diagram & Energy Fluctuation — Power: {results['power_kw']:.2f} kW", fontweight="bold")
    ax_t1.grid(True, linestyle="--", alpha=0.5)
    ax_t1.legend(loc="upper right")

    ax_t2.plot(deg, results["energy_diagram"], color="#303F9F", linewidth=2, label="Cumulative Energy Variation ∫(T - T_mean)dθ")
    ax_t2.axhline(0, color="k", linewidth=0.8)
    ax_t2.set_xlabel("Crank Angle θ (degrees)", fontweight="bold")
    ax_t2.set_ylabel("Energy (Joules)", fontweight="bold")
    ax_t2.set_xlim(0, 360)
    ax_t2.set_xticks(np.arange(0, 361, 45))
    ax_t2.grid(True, linestyle="--", alpha=0.5)
    ax_t2.legend(loc="upper right")

    tmd_path = os.path.join(output_dir, "turning_moment_diagram.png")
    fig3.savefig(tmd_path, dpi=300, bbox_inches="tight")
    plt.close(fig3)

    # 4. Harmonics & Polar Plot
    fig4 = plt.figure(figsize=(11, 5))
    ax_h1 = fig4.add_subplot(1, 2, 1)
    ax_h2 = fig4.add_subplot(1, 2, 2, projection="polar")
    fig4.subplots_adjust(wspace=0.3)

    orders = results["harmonic_orders"]
    amps = results["harmonic_amplitudes"]
    ax_h1.bar([f"{k}θ" for k in orders], amps, color="#3F51B5", edgecolor="#1A237E", alpha=0.85)
    ax_h1.set_xlabel("Harmonic Order (Vibration Analysis)", fontweight="bold")
    ax_h1.set_ylabel("Torque Amplitude (N·m)", fontweight="bold")
    ax_h1.set_title("Harmonic Fourier Spectrum", fontweight="bold")
    ax_h1.grid(True, axis="y", linestyle="--", alpha=0.5)

    r_vals = np.maximum(dyn.crank_torque, 0)
    ax_h2.plot(results["thetas"], r_vals, color="#E91E63", linewidth=2.2)
    ax_h2.fill(results["thetas"], r_vals, color="#F8BBD0", alpha=0.5)
    ax_h2.set_title("Polar Torque Diagram (T vs θ)", fontweight="bold", pad=15)

    harm_path = os.path.join(output_dir, "harmonics_polar.png")
    fig4.savefig(harm_path, dpi=300, bbox_inches="tight")
    plt.close(fig4)

    return {
        "kinematics": kin_path,
        "dynamics": dyn_path,
        "tmd": tmd_path,
        "harmonics": harm_path,
    }


def generate_html_report(params: SliderCrankParameters, gas_model: GasPressureModel, report_path: str = "reports/engineering_report.html"):
    """Generates an executive, beautifully formatted standalone HTML engineering report."""
    os.makedirs(os.path.dirname(os.path.abspath(report_path)), exist_ok=True)
    plots = save_all_plots(params, gas_model, output_dir=os.path.dirname(os.path.abspath(report_path)))
    analyzer = CycleAnalyzer(params, gas_model=gas_model)
    res = analyzer.analyze(num_points=720)
    fw = res["flywheel"]

    html_content = f"""<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>Design & Simulation of Slider-Crank Mechanism | Analysis Report</title>
    <style>
        body {{
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif;
            line-height: 1.6;
            color: #212529;
            background: #f8f9fa;
            margin: 0;
            padding: 30px;
        }}
        .container {{
            max-width: 1000px;
            margin: 0 auto;
            background: white;
            border-radius: 12px;
            box-shadow: 0 4px 20px rgba(0,0,0,0.08);
            padding: 40px;
        }}
        h1 {{
            color: #0d47a1;
            margin-bottom: 5px;
            border-bottom: 3px solid #1976d2;
            padding-bottom: 12px;
        }}
        .author {{
            color: #546e7a;
            font-size: 1.1em;
            margin-bottom: 25px;
        }}
        .grid-2 {{
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            margin-bottom: 30px;
        }}
        table {{
            width: 100%;
            border-collapse: collapse;
            margin-bottom: 25px;
        }}
        th, td {{
            padding: 10px 14px;
            text-align: left;
            border-bottom: 1px solid #e0e0e0;
        }}
        th {{
            background: #f1f5f9;
            color: #334155;
            font-weight: 600;
        }}
        .val {{
            font-weight: bold;
            color: #1565c0;
        }}
        .img-card {{
            background: #ffffff;
            border: 1px solid #e2e8f0;
            border-radius: 8px;
            padding: 15px;
            margin-bottom: 30px;
            text-align: center;
        }}
        .img-card img {{
            max-width: 100%;
            height: auto;
            border-radius: 4px;
        }}
        .badge {{
            display: inline-block;
            padding: 4px 10px;
            border-radius: 20px;
            background: #e3f2fd;
            color: #0d47a1;
            font-size: 0.85em;
            font-weight: bold;
        }}
    </style>
</head>
<body>
<div class="container">
    <h1>DESIGN AND SIMULATION OF A SLIDER-CRANK MECHANISM</h1>
    <div class="author">Kinematic and Dynamic Analysis Report | <strong>Author: Dhruv Chauhan (24JE0497)</strong></div>

    <div class="grid-2">
        <div>
            <h3>Mechanism Parameters</h3>
            <table>
                <tr><td>Crank Radius (r):</td><td class="val">{params.r*1000.0:.1f} mm</td></tr>
                <tr><td>Connecting Rod Length (l):</td><td class="val">{params.l*1000.0:.1f} mm</td></tr>
                <tr><td>Obliquity Ratio (n = l/r):</td><td class="val">{params.n:.2f}</td></tr>
                <tr><td>Offset (e):</td><td class="val">{params.e*1000.0:.1f} mm</td></tr>
                <tr><td>Stroke Length:</td><td class="val">{params.stroke*1000.0:.1f} mm</td></tr>
                <tr><td>Cylinder Bore:</td><td class="val">{params.bore_diameter*1000.0:.1f} mm</td></tr>
                <tr><td>Swept Displacement Volume:</td><td class="val">{params.swept_volume*1e6:.1f} cc</td></tr>
                <tr><td>Crankshaft Speed:</td><td class="val">{params.rpm:.0f} RPM (ω = {params.omega:.1f} rad/s) [Speed Range: 0 - 8000 RPM]</td></tr>
                <tr><td>Reciprocating Mass (mp + rod):</td><td class="val">{params.equivalent_reciprocating_mass:.2f} kg</td></tr>
            </table>
        </div>
        <div>
            <h3>Dynamic & Performance Results</h3>
            <table>
                <tr><td>Mean Piston Speed:</td><td class="val">{res['mean_piston_speed']:.2f} m/s</td></tr>
                <tr><td>Maximum Piston Speed:</td><td class="val">{res['max_piston_velocity']:.2f} m/s (@ {res['theta_max_vel_deg']:.0f}°)</td></tr>
                <tr><td>Maximum Acceleration:</td><td class="val">{res['max_piston_acceleration']:.1f} m/s² (@ {res['theta_max_acc_deg']:.0f}°)</td></tr>
                <tr><td>Mean Indicated Torque:</td><td class="val">{res['mean_torque']:.1f} N·m</td></tr>
                <tr><td>Peak Indicated Torque:</td><td class="val">{res['max_torque']:.1f} N·m</td></tr>
                <tr><td>Engine Power Output:</td><td class="val">{res['power_kw']:.2f} kW</td></tr>
                <tr><td>Energy Fluctuation (ΔE):</td><td class="val">{res['delta_e']:.1f} Joules</td></tr>
                <tr><td>Coeff. of Fluctuation (Ce):</td><td class="val">{fw.coeff_fluctuation_energy:.3f}</td></tr>
                <tr><td>Required Flywheel Inertia:</td><td class="val">{fw.required_inertia_kgm2:.4f} kg·m²</td></tr>
                <tr><td>Flywheel Mass (at R={fw.flywheel_rim_radius_m*1000:.0f}mm):</td><td class="val">{fw.flywheel_mass_kg:.2f} kg</td></tr>
            </table>
        </div>
    </div>

    <h3>1. Kinematic Analysis (Displacement, Velocity & Acceleration)</h3>
    <div class="img-card">
        <img src="{os.path.basename(plots['kinematics'])}" alt="Kinematics Plot">
    </div>

    <h3>2. Dynamic Force Analysis (Gas, Inertia, Piston Effort, Side Thrust)</h3>
    <div class="img-card">
        <img src="{os.path.basename(plots['dynamics'])}" alt="Dynamics Plot">
    </div>

    <h3>3. Turning Moment Diagram (TMD) & Flywheel Design</h3>
    <div class="img-card">
        <img src="{os.path.basename(plots['tmd'])}" alt="TMD Plot">
    </div>

    <h3>4. Harmonic Fourier Analysis & Polar Torque Diagram</h3>
    <div class="img-card">
        <img src="{os.path.basename(plots['harmonics'])}" alt="Harmonics Plot">
    </div>

    <div style="text-align: center; color: #78909c; font-size: 0.9em; margin-top: 30px;">
        Generated automatically by Python-Based Slider-Crank Kinematic and Dynamic Analysis Tool.
    </div>
</div>
</body>
</html>"""

    with open(report_path, "w", encoding="utf-8") as f:
        f.write(html_content)

    return report_path


def main():
    parser = argparse.ArgumentParser(
        description="Design and Simulation of Slider-Crank Mechanism | Dhruv Chauhan (24JE0497)"
    )
    parser.add_argument("--gui", action="store_true", help="Launch interactive Tkinter desktop GUI")
    parser.add_argument("--crank", type=float, default=50.0, help="Crank radius in mm (default: 50 mm)")
    parser.add_argument("--conrod", type=float, default=200.0, help="Connecting rod length in mm (default: 200 mm)")
    parser.add_argument("--offset", type=float, default=0.0, help="Stroke line offset in mm (default: 0 mm)")
    parser.add_argument("--bore", type=float, default=80.0, help="Cylinder bore in mm (default: 80 mm)")
    parser.add_argument("--rpm", type=float, default=1800.0, help="Rotational speed in RPM (range: 0 to 8000 RPM, default: 1800)")
    parser.add_argument("--mp", type=float, default=0.85, help="Piston mass in kg (default: 0.85)")
    parser.add_argument("--mrod", type=float, default=1.20, help="Connecting rod mass in kg (default: 1.20)")
    parser.add_argument("--ppeak", type=float, default=45.0, help="Peak gas pressure in bar (default: 45 bar)")
    parser.add_argument("--plots", action="store_true", help="Generate and save high-resolution plots to reports/")
    parser.add_argument("--report", action="store_true", help="Generate full HTML engineering report to reports/")
    parser.add_argument("--csv", type=str, default="", help="Export 360-degree simulation data to specified CSV path")
    parser.add_argument("--validate", action="store_true", help="Run analytical validation and test suite")

    args = parser.parse_args()

    # If --validate is requested
    if args.validate:
        print("\n" + "=" * 65)
        print("RUNNING SLIDER-CRANK VALIDATION AND BENCHMARK TEST SUITE")
        print("=" * 65)
        import pytest
        exit_code = pytest.main(["tests", "-v"])
        sys.exit(exit_code)

    params = SliderCrankParameters(
        crank_length=args.crank / 1000.0,
        con_rod_length=args.conrod / 1000.0,
        offset=args.offset / 1000.0,
        bore_diameter=args.bore / 1000.0,
        rpm=args.rpm,
        piston_mass=args.mp,
        con_rod_mass=args.mrod,
    )
    gas_model = GasPressureModel(params, peak_pressure_bar=args.ppeak)

    # Launch GUI if explicitly asked or no action arguments provided
    if args.gui or (len(sys.argv) == 1):
        try:
            from gui.app import launch_gui
            launch_gui()
            return
        except Exception as e:
            print(f"[Notice] GUI could not be opened in current terminal environment: {e}")
            print("[Info] Defaulting to CLI analysis and report generation...\n")

    # CLI Output
    analyzer = CycleAnalyzer(params, gas_model=gas_model)
    res = analyzer.analyze(num_points=720)
    fw = res["flywheel"]

    print("=" * 65)
    print("  SLIDER-CRANK KINEMATIC & DYNAMIC ANALYSIS TOOL")
    print("  Author: Dhruv Chauhan (24JE0497)")
    print("=" * 65)
    print(f"Crank Radius (r):            {params.r * 1000.0:8.2f} mm")
    print(f"Connecting Rod Length (l):   {params.l * 1000.0:8.2f} mm")
    print(f"Obliquity Ratio (n = l/r):   {params.n:8.2f}")
    print(f"Crank Offset (e):            {params.e * 1000.0:8.2f} mm")
    print(f"Total Stroke Length:         {params.stroke * 1000.0:8.2f} mm")
    print(f"Cylinder Bore:               {params.bore_diameter * 1000.0:8.2f} mm")
    print(f"Swept Volume:                {params.swept_volume * 1e6:8.1f} cc")
    print(f"Rotational Speed:            {params.rpm:8.0f} RPM (omega = {params.omega:.1f} rad/s)")
    print("-" * 65)
    print(f"Mean Piston Speed:           {res['mean_piston_speed']:8.2f} m/s")
    print(f"Peak Piston Velocity:        {res['max_piston_velocity']:8.2f} m/s  (@ {res['theta_max_vel_deg']:.0f} deg)")
    print(f"Peak Acceleration:           {res['max_piston_acceleration']:8.1f} m/s^2 (@ {res['theta_max_acc_deg']:.0f} deg)")
    print("-" * 65)
    print(f"Mean Indicated Torque:       {res['mean_torque']:8.1f} N*m")
    print(f"Peak Engine Torque:          {res['max_torque']:8.1f} N*m")
    print(f"Indicated Power:             {res['power_kw']:8.2f} kW")
    print(f"Fluctuation of Energy (dE):  {res['delta_e']:8.1f} Joules")
    print(f"Flywheel Required Inertia:   {fw.required_inertia_kgm2:8.4f} kg*m^2 (for Cs=0.02)")
    print(f"Flywheel Mass (R={fw.flywheel_rim_radius_m*1000:.0f} mm):     {fw.flywheel_mass_kg:8.2f} kg")
    print("=" * 65)

    # Save plots
    if args.plots or args.report or len(sys.argv) == 1:
        plots = save_all_plots(params, gas_model, output_dir="reports")
        print("\n[+] Publication quality plots saved:")
        for k, v in plots.items():
            print(f"    - {k.capitalize()}: {v}")

    # Generate Report
    if args.report or len(sys.argv) == 1:
        report_file = generate_html_report(params, gas_model, "reports/engineering_report.html")
        print(f"[+] Full HTML Engineering Report generated: {report_file}")

    # Export CSV
    csv_file = args.csv if args.csv else "export/slider_crank_data.csv"
    os.makedirs(os.path.dirname(os.path.abspath(csv_file)), exist_ok=True)
    deg = res["theta_deg"]
    kin = res["kinematics"]
    dyn = res["dynamics"]
    with open(csv_file, "w", encoding="utf-8") as f:
        f.write("theta_deg,time_s,piston_disp_m,piston_vel_m_s,piston_acc_m_s2,con_rod_angle_deg,gas_force_N,inertia_force_N,piston_effort_N,side_thrust_N,crank_torque_Nm\n")
        for i in range(len(deg)):
            f.write(f"{deg[i]:.2f},{kin.time_s[i]:.6f},{kin.piston_displacement[i]:.6f},{kin.piston_velocity[i]:.6f},{kin.piston_acceleration[i]:.6f},{kin.con_rod_angle_deg[i]:.4f},{dyn.gas_force[i]:.2f},{dyn.inertia_force[i]:.2f},{dyn.piston_effort[i]:.2f},{dyn.cylinder_side_thrust[i]:.2f},{dyn.crank_torque[i]:.2f}\n")
    print(f"[+] Simulation data exported: {csv_file}\n")


if __name__ == "__main__":
    main()
