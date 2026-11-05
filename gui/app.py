"""
Main Desktop GUI Application for Slider-Crank Simulation and Analysis.
Built with Tkinter and Matplotlib.
Author: Dhruv Chauhan (24JE0497)
"""

import os
import sys
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

from engine.mechanism import SliderCrankParameters
from engine.kinematics import KinematicSolver
from engine.dynamics import DynamicSolver
from engine.gas_pressure import GasPressureModel, EngineCycleType
from engine.analysis import CycleAnalyzer
from gui.mechanism_canvas import MechanismCanvas
from gui.plot_panels import PlotPanels


PRESETS = {
    "Single-Cylinder Petrol Engine (Standard)": {
        "r": 50.0, "l": 200.0, "e": 0.0, "rpm": 1800.0,
        "m_p": 0.85, "m_rod": 1.2, "bore": 80.0, "p_peak": 45.0,
        "cycle": EngineCycleType.POLYTROPIC_EXPANSION
    },
    "High-Speed Motorcycle Engine (Short Stroke)": {
        "r": 27.0, "l": 105.0, "e": 0.0, "rpm": 7500.0,
        "m_p": 0.22, "m_rod": 0.35, "bore": 67.0, "p_peak": 60.0,
        "cycle": EngineCycleType.POLYTROPIC_EXPANSION
    },
    "Heavy-Duty Diesel Engine (Long Stroke)": {
        "r": 75.0, "l": 300.0, "e": 0.0, "rpm": 1500.0,
        "m_p": 2.40, "m_rod": 3.80, "bore": 120.0, "p_peak": 85.0,
        "cycle": EngineCycleType.POLYTROPIC_EXPANSION
    },
    "Offset Slider-Crank (e = 15 mm)": {
        "r": 50.0, "l": 200.0, "e": 15.0, "rpm": 1200.0,
        "m_p": 0.90, "m_rod": 1.30, "bore": 85.0, "p_peak": 40.0,
        "cycle": EngineCycleType.POLYTROPIC_EXPANSION
    },
    "Reciprocating Air Compressor": {
        "r": 35.0, "l": 130.0, "e": 0.0, "rpm": 1450.0,
        "m_p": 0.60, "m_rod": 0.85, "bore": 70.0, "p_peak": 12.0,
        "cycle": EngineCycleType.RECIPROCATING_COMPRESSOR
    },
}


class SliderCrankApp:
    """Complete Interactive Desktop GUI for Slider-Crank Analysis."""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("Design & Simulation of Slider-Crank Mechanism | Dhruv Chauhan (24JE0497)")
        self.root.geometry("1400x900")
        self.root.minsize(1100, 750)

        # Initialize default parameters
        self.params = SliderCrankParameters(
            crank_length=0.05,
            con_rod_length=0.20,
            offset=0.0,
            rpm=1800.0,
            piston_mass=0.85,
            con_rod_mass=1.20,
            bore_diameter=0.080,
        )
        self.gas_model = GasPressureModel(
            self.params,
            cycle_type=EngineCycleType.POLYTROPIC_EXPANSION,
            peak_pressure_bar=45.0,
        )

        # Animation state
        self.current_theta_deg = 0.0
        self.is_playing = True
        self.anim_speed_multiplier = 0.5
        self.after_id = None

        self._init_theme()
        self._build_layout()
        self._load_preset("Single-Cylinder Petrol Engine (Standard)")
        self._start_animation_loop()

    def _init_theme(self):
        self.style = ttk.Style()
        self.style.theme_use("clam")
        self.style.configure(".", font=("Segoe UI", 9))
        self.style.configure("Header.TLabel", font=("Segoe UI", 12, "bold"), foreground="#0D47A1")
        self.style.configure("Subheader.TLabel", font=("Segoe UI", 10, "bold"), foreground="#1565C0")
        self.style.configure("Metric.TLabel", font=("Segoe UI", 9, "bold"), foreground="#2E7D32")
        self.style.configure("Action.TButton", font=("Segoe UI", 9, "bold"))

    def _build_layout(self):
        # 1. Top Header Banner
        header_frame = tk.Frame(self.root, bg="#1565C0", height=48)
        header_frame.pack(fill=tk.X, side=tk.TOP)

        title_lbl = tk.Label(
            header_frame,
            text="SLIDER-CRANK KINEMATIC & DYNAMIC ANALYSIS TOOL",
            font=("Segoe UI", 13, "bold"),
            fg="white",
            bg="#1565C0",
            padx=12,
            pady=8,
        )
        title_lbl.pack(side=tk.LEFT)

        author_lbl = tk.Label(
            header_frame,
            text="Dhruv Chauhan (24JE0497) | Theory of Machines",
            font=("Segoe UI", 10, "italic"),
            fg="#BBDEFB",
            bg="#1565C0",
            padx=12,
        )
        author_lbl.pack(side=tk.RIGHT)

        # 2. Main content paned window
        main_paned = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_paned.pack(fill=tk.BOTH, expand=True)

        # Left Sidebar (Inputs & Controls)
        sidebar_frame = ttk.Frame(main_paned, width=340, padding=10)
        main_paned.add(sidebar_frame, weight=0)

        # Right Display (Top: Mechanism Canvas, Bottom: Plot Panels)
        right_frame = ttk.Frame(main_paned, padding=5)
        main_paned.add(right_frame, weight=1)

        self._build_sidebar(sidebar_frame)
        self._build_visualization_area(right_frame)

    def _build_sidebar(self, parent):
        canvas = tk.Canvas(parent, borderwidth=0, highlightthickness=0, width=320)
        scrollbar = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        scroll_frame = ttk.Frame(canvas)

        scroll_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=scroll_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Preset Selector
        preset_grp = ttk.LabelFrame(scroll_frame, text="Mechanism Presets", padding=8)
        preset_grp.pack(fill=tk.X, pady=(0, 10))

        self.preset_var = tk.StringVar(value="Single-Cylinder Petrol Engine (Standard)")
        preset_cb = ttk.Combobox(
            preset_grp,
            textvariable=self.preset_var,
            values=list(PRESETS.keys()),
            state="readonly",
            font=("Segoe UI", 8),
        )
        preset_cb.pack(fill=tk.X, pady=2)
        preset_cb.bind("<<ComboboxSelected>>", lambda e: self._load_preset(self.preset_var.get()))

        # Geometry Group
        geom_grp = ttk.LabelFrame(scroll_frame, text="Mechanism Geometry", padding=8)
        geom_grp.pack(fill=tk.X, pady=(0, 10))

        self.crank_entry = self._create_input_row(geom_grp, "Crank Radius r (mm):", "50.0")
        self.con_rod_entry = self._create_input_row(geom_grp, "Con-Rod Length l (mm):", "200.0")
        self.offset_entry = self._create_input_row(geom_grp, "Stroke Offset e (mm):", "0.0")
        self.bore_entry = self._create_input_row(geom_grp, "Cylinder Bore D (mm):", "80.0")

        # Inertia & Dynamics Group
        dyn_grp = ttk.LabelFrame(scroll_frame, text="Operating & Dynamic Parameters", padding=8)
        dyn_grp.pack(fill=tk.X, pady=(0, 10))

        self.rpm_entry = self._create_input_row(dyn_grp, "Speed (RPM) [0-8000]:", "1800.0")
        self.rpm_scale = ttk.Scale(
            dyn_grp, from_=0.0, to=8000.0, orient=tk.HORIZONTAL,
            command=self._on_rpm_scale_move
        )
        self.rpm_scale.set(1800.0)
        self.rpm_scale.pack(fill=tk.X, pady=(0, 6))

        self.piston_mass_entry = self._create_input_row(dyn_grp, "Piston Mass mp (kg):", "0.85")
        self.con_rod_mass_entry = self._create_input_row(dyn_grp, "Con-Rod Mass (kg):", "1.20")
        self.p_peak_entry = self._create_input_row(dyn_grp, "Peak Gas Pressure (bar):", "45.0")

        ttk.Label(dyn_grp, text="Indicator Cycle:").pack(anchor="w", pady=(4, 2))
        self.cycle_var = tk.StringVar(value=EngineCycleType.POLYTROPIC_EXPANSION.value)
        cycle_cb = ttk.Combobox(
            dyn_grp,
            textvariable=self.cycle_var,
            values=[c.value for c in EngineCycleType],
            state="readonly",
            font=("Segoe UI", 8),
        )
        cycle_cb.pack(fill=tk.X, pady=(0, 6))

        # Update button
        update_btn = ttk.Button(
            scroll_frame,
            text="⚡ Update Parameters & Recalculate",
            style="Action.TButton",
            command=self._apply_inputs,
        )
        update_btn.pack(fill=tk.X, pady=(0, 10))

        # Calculated Metrics Box
        self.metrics_grp = ttk.LabelFrame(scroll_frame, text="Cycle Summary & Design Metrics", padding=8)
        self.metrics_grp.pack(fill=tk.X, pady=(0, 10))

        self.metric_labels = {}
        metrics = [
            ("Stroke Length:", "stroke"),
            ("Obliquity Ratio (l/r):", "ratio_n"),
            ("Swept Volume:", "swept_vol"),
            ("Mean Piston Speed:", "mean_speed"),
            ("Peak Piston Velocity:", "max_speed"),
            ("Peak Acceleration:", "max_acc"),
            ("Mean Engine Torque:", "mean_torque"),
            ("Power Output:", "power"),
            ("Fluctuation Energy ΔE:", "delta_e"),
            ("Flywheel Mass (Cs=0.02):", "flywheel_m"),
        ]
        for title, key in metrics:
            row = ttk.Frame(self.metrics_grp)
            row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=title, font=("Segoe UI", 8)).pack(side=tk.LEFT)
            val_lbl = ttk.Label(row, text="--", style="Metric.TLabel", font=("Segoe UI", 8, "bold"))
            val_lbl.pack(side=tk.RIGHT)
            self.metric_labels[key] = val_lbl

        # Export Actions
        export_grp = ttk.LabelFrame(scroll_frame, text="Data Export & Reports", padding=8)
        export_grp.pack(fill=tk.X, pady=(0, 10))

        btn_csv = ttk.Button(export_grp, text="📁 Export CSV Data", command=self._export_csv)
        btn_csv.pack(fill=tk.X, pady=2)

        btn_plots = ttk.Button(export_grp, text="🖼 Export High-Res Plots", command=self._export_plots)
        btn_plots.pack(fill=tk.X, pady=2)

        btn_report = ttk.Button(export_grp, text="📄 Generate Analysis Report", command=self._generate_report)
        btn_report.pack(fill=tk.X, pady=2)

    def _create_input_row(self, parent, label_text, default_val):
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text=label_text, width=22).pack(side=tk.LEFT)
        entry = ttk.Entry(row, width=10)
        entry.insert(0, default_val)
        entry.pack(side=tk.RIGHT)
        return entry

    def _build_visualization_area(self, parent):
        # Split into upper (mechanism canvas) and lower (notebook plots)
        vis_paned = ttk.PanedWindow(parent, orient=tk.VERTICAL)
        vis_paned.pack(fill=tk.BOTH, expand=True)

        # 1. Mechanism Animation Canvas Frame
        top_frame = ttk.Frame(vis_paned)
        vis_paned.add(top_frame, weight=3)

        self.fig_canvas = Figure(figsize=(8, 3.6), dpi=100)
        self.ax_canvas = self.fig_canvas.add_subplot(111)
        self.mech_canvas_obj = MechanismCanvas(self.fig_canvas, self.ax_canvas, self.params)

        self.tk_canvas = FigureCanvasTkAgg(self.fig_canvas, master=top_frame)
        self.tk_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Animation playback control bar
        anim_bar = ttk.Frame(top_frame, padding=4)
        anim_bar.pack(fill=tk.X)

        self.play_btn = ttk.Button(anim_bar, text="⏸ Pause", width=9, command=self._toggle_playback)
        self.play_btn.pack(side=tk.LEFT, padx=3)

        ttk.Button(anim_bar, text="⏮ Step -5°", width=10, command=lambda: self._step_angle(-5.0)).pack(side=tk.LEFT, padx=2)
        ttk.Button(anim_bar, text="Step +5° ⏭", width=10, command=lambda: self._step_angle(5.0)).pack(side=tk.LEFT, padx=2)

        ttk.Label(anim_bar, text="Speed:").pack(side=tk.LEFT, padx=(10, 2))
        self.speed_var = tk.StringVar(value="0.5x")
        speed_cb = ttk.Combobox(
            anim_bar,
            textvariable=self.speed_var,
            values=["0.1x", "0.25x", "0.5x", "1.0x", "2.0x"],
            width=6,
            state="readonly",
        )
        speed_cb.pack(side=tk.LEFT, padx=2)
        speed_cb.bind("<<ComboboxSelected>>", self._change_speed)

        ttk.Label(anim_bar, text="Scrub θ:").pack(side=tk.LEFT, padx=(15, 2))
        self.scrub_scale = ttk.Scale(
            anim_bar, from_=0.0, to=360.0, orient=tk.HORIZONTAL,
            command=self._on_scrub
        )
        self.scrub_scale.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)

        self.angle_readout = ttk.Label(anim_bar, text="  0.0°", font=("Segoe UI", 9, "bold"), width=7)
        self.angle_readout.pack(side=tk.RIGHT, padx=4)

        # 2. Lower Notebook for Analysis Plots
        bottom_frame = ttk.Frame(vis_paned)
        vis_paned.add(bottom_frame, weight=4)

        self.notebook = ttk.Notebook(bottom_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        self.fig_plots = Figure(figsize=(8, 3.8), dpi=100)
        self.plot_panels_obj = PlotPanels(self.fig_plots, self.params, self.gas_model)

        self.plot_canvas = FigureCanvasTkAgg(self.fig_plots, master=bottom_frame)
        self.plot_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        toolbar_frame = ttk.Frame(bottom_frame)
        toolbar_frame.pack(fill=tk.X)
        self.toolbar = NavigationToolbar2Tk(self.plot_canvas, toolbar_frame)
        self.toolbar.update()

        # Tabs in notebook act as views
        tab_names = [
            "Kinematics (x, v, a)",
            "Dynamics & Forces",
            "Turning Moment (TMD)",
            "Harmonics & Polar Diagram",
        ]
        self.tab_frames = []
        for name in tab_names:
            f = ttk.Frame(self.notebook)
            self.notebook.add(f, text=f"  {name}  ")
            self.tab_frames.append(f)

        self.notebook.bind("<<NotebookTabChanged>>", self._on_tab_changed)

    def _on_rpm_scale_move(self, val):
        rpm_val = float(val)
        self.rpm_entry.delete(0, tk.END)
        self.rpm_entry.insert(0, f"{rpm_val:.0f}")

    def _load_preset(self, preset_name: str):
        if preset_name not in PRESETS:
            return
        p = PRESETS[preset_name]
        self.crank_entry.delete(0, tk.END)
        self.crank_entry.insert(0, str(p["r"]))

        self.con_rod_entry.delete(0, tk.END)
        self.con_rod_entry.insert(0, str(p["l"]))

        self.offset_entry.delete(0, tk.END)
        self.offset_entry.insert(0, str(p["e"]))

        self.bore_entry.delete(0, tk.END)
        self.bore_entry.insert(0, str(p["bore"]))

        self.rpm_entry.delete(0, tk.END)
        self.rpm_entry.insert(0, str(p["rpm"]))
        if hasattr(self, "rpm_scale"):
            self.rpm_scale.set(float(p["rpm"]))

        self.piston_mass_entry.delete(0, tk.END)
        self.piston_mass_entry.insert(0, str(p["m_p"]))

        self.con_rod_mass_entry.delete(0, tk.END)
        self.con_rod_mass_entry.insert(0, str(p["m_rod"]))

        self.p_peak_entry.delete(0, tk.END)
        self.p_peak_entry.insert(0, str(p["p_peak"]))

        self.cycle_var.set(p["cycle"].value)
        self._apply_inputs()

    def _apply_inputs(self):
        try:
            r = float(self.crank_entry.get()) / 1000.0
            l = float(self.con_rod_length_entry() if hasattr(self, "con_rod_length_entry") else self.con_rod_entry.get()) / 1000.0
            e = float(self.offset_entry.get()) / 1000.0
            bore = float(self.bore_entry.get()) / 1000.0
            rpm = float(self.rpm_entry.get())
            if rpm < 0 or rpm > 8000:
                raise ValueError(f"Speed ({rpm:.1f} RPM) must be in range 0 to 8000 RPM.")
            if hasattr(self, "rpm_scale"):
                self.rpm_scale.set(rpm)

            m_p = float(self.piston_mass_entry.get())
            m_rod = float(self.con_rod_mass_entry.get())
            p_peak = float(self.p_peak_entry.get())

            cycle_val = self.cycle_var.get()
            selected_cycle = EngineCycleType.POLYTROPIC_EXPANSION
            for c in EngineCycleType:
                if c.value == cycle_val:
                    selected_cycle = c
                    break

            new_params = SliderCrankParameters(
                crank_length=r,
                con_rod_length=l,
                offset=e,
                bore_diameter=bore,
                rpm=rpm,
                piston_mass=m_p,
                con_rod_mass=m_rod,
            )

            new_gas = GasPressureModel(
                new_params,
                cycle_type=selected_cycle,
                peak_pressure_bar=p_peak,
            )

            self.params = new_params
            self.gas_model = new_gas

            # Update sub-objects
            self.mech_canvas_obj.update_params(self.params)
            self.plot_panels_obj.update_params(self.params, self.gas_model)

            # Update metrics readout
            self._update_metrics()
            # Redraw current tab plot
            self._render_current_plot()
            # Redraw canvas
            self.mech_canvas_obj.draw_state(np.radians(self.current_theta_deg))
            self.tk_canvas.draw_idle()

        except Exception as exc:
            messagebox.showerror("Parameter Error", f"Invalid parameters:\n{exc}")

    def _update_metrics(self):
        results = self.plot_panels_obj.analyzer.analyze(num_points=360)
        p = self.params

        self.metric_labels["stroke"].config(text=f"{p.stroke * 1000.0:.1f} mm")
        self.metric_labels["ratio_n"].config(text=f"{p.n:.2f}")
        self.metric_labels["swept_vol"].config(text=f"{p.swept_volume * 1e6:.1f} cc")
        self.metric_labels["mean_speed"].config(text=f"{results['mean_piston_speed']:.2f} m/s")
        self.metric_labels["max_speed"].config(text=f"{results['max_piston_velocity']:.2f} m/s")
        self.metric_labels["max_acc"].config(text=f"{results['max_piston_acceleration']:.0f} m/s²")
        self.metric_labels["mean_torque"].config(text=f"{results['mean_torque']:.1f} N·m")
        self.metric_labels["power"].config(text=f"{results['power_kw']:.2f} kW")
        self.metric_labels["delta_e"].config(text=f"{results['delta_e']:.1f} J")
        self.metric_labels["flywheel_m"].config(text=f"{results['flywheel'].flywheel_mass_kg:.1f} kg")

    def _render_current_plot(self):
        tab_idx = self.notebook.index(self.notebook.select())
        if tab_idx == 0:
            self.plot_panels_obj.plot_kinematics(self.current_theta_deg)
        elif tab_idx == 1:
            self.plot_panels_obj.plot_dynamics(self.current_theta_deg)
        elif tab_idx == 2:
            self.plot_panels_obj.plot_tmd(self.current_theta_deg)
        elif tab_idx == 3:
            self.plot_panels_obj.plot_harmonics()

        self.plot_canvas.draw_idle()

    def _on_tab_changed(self, event):
        self._render_current_plot()

    def _toggle_playback(self):
        self.is_playing = not self.is_playing
        self.play_btn.config(text="▶ Play" if not self.is_playing else "⏸ Pause")

    def _change_speed(self, event=None):
        val_map = {"0.1x": 0.1, "0.25x": 0.25, "0.5x": 0.5, "1.0x": 1.0, "2.0x": 2.0}
        self.anim_speed_multiplier = val_map.get(self.speed_var.get(), 0.5)

    def _step_angle(self, delta_deg: float):
        self.is_playing = False
        self.play_btn.config(text="▶ Play")
        self.current_theta_deg = (self.current_theta_deg + delta_deg) % 360.0
        self._update_animation_frame()

    def _on_scrub(self, val):
        self.current_theta_deg = float(val) % 360.0
        self._update_animation_frame()

    def _update_animation_frame(self):
        theta_rad = np.radians(self.current_theta_deg)
        self.mech_canvas_obj.draw_state(theta_rad)
        self.tk_canvas.draw_idle()

        # Update cursor on plots
        self.plot_panels_obj.update_cursor(self.current_theta_deg)
        self.plot_canvas.draw_idle()

        # Update scrub and label
        self.scrub_scale.set(self.current_theta_deg)
        self.angle_readout.config(text=f"{self.current_theta_deg:5.1f}°")

    def _start_animation_loop(self):
        frame_interval_ms = 33  # ~30 FPS

        def loop():
            if self.is_playing:
                # Delta angle per frame based on RPM and speed multiplier
                # delta_deg = (RPM * 360 / 60) * dt * speed_mult
                dt = frame_interval_ms / 1000.0
                deg_per_sec = (self.params.rpm * 6.0) * self.anim_speed_multiplier
                delta_deg = deg_per_sec * dt
                self.current_theta_deg = (self.current_theta_deg + delta_deg) % 360.0
                self._update_animation_frame()

            self.after_id = self.root.after(frame_interval_ms, loop)

        self.after_id = self.root.after(frame_interval_ms, loop)

    def _export_csv(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV Spreadsheet", "*.csv")],
            initialfile="slider_crank_simulation_data.csv",
        )
        if not path:
            return
        results = self.plot_panels_obj.analyzer.analyze(num_points=360)
        deg = results["theta_deg"]
        kin = results["kinematics"]
        dyn = results["dynamics"]

        header = (
            "theta_deg,time_s,piston_disp_m,piston_vel_m_s,piston_acc_m_s2,"
            "con_rod_angle_deg,con_rod_omega_rad_s,gas_force_N,inertia_force_N,"
            "piston_effort_N,con_rod_force_N,side_thrust_N,crank_effort_N,crank_torque_Nm\n"
        )
        with open(path, "w", encoding="utf-8") as f:
            f.write(header)
            for i in range(len(deg)):
                f.write(
                    f"{deg[i]:.2f},{kin.time_s[i]:.6f},{kin.piston_displacement[i]:.6f},"
                    f"{kin.piston_velocity[i]:.6f},{kin.piston_acceleration[i]:.6f},"
                    f"{kin.con_rod_angle_deg[i]:.4f},{kin.con_rod_ang_velocity[i]:.4f},"
                    f"{dyn.gas_force[i]:.2f},{dyn.inertia_force[i]:.2f},{dyn.piston_effort[i]:.2f},"
                    f"{dyn.con_rod_force[i]:.2f},{dyn.cylinder_side_thrust[i]:.2f},"
                    f"{dyn.crank_effort[i]:.2f},{dyn.crank_torque[i]:.2f}\n"
                )
        messagebox.showinfo("Export Successful", f"Full cycle simulation data exported to:\n{path}")

    def _export_plots(self):
        folder = filedialog.askdirectory(title="Select Folder to Save High-Resolution Plots")
        if not folder:
            return
        from main import save_all_plots
        save_all_plots(self.params, self.gas_model, output_dir=folder)
        messagebox.showinfo("Export Successful", f"Publication quality plots saved to:\n{folder}")

    def _generate_report(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".html",
            filetypes=[("HTML Report", "*.html"), ("Markdown Report", "*.md")],
            initialfile="slider_crank_engineering_report.html",
        )
        if not path:
            return
        from main import generate_html_report
        generate_html_report(self.params, self.gas_model, path)
        messagebox.showinfo("Report Generated", f"Engineering analysis report generated at:\n{path}")


def launch_gui():
    root = tk.Tk()
    app = SliderCrankApp(root)
    root.mainloop()


if __name__ == "__main__":
    launch_gui()
