# Design and Simulation of a Slider-Crank Mechanism
### Python-Based Kinematic and Dynamic Analysis Tool
**Author / People Involved:** Dhruv Chauhan (24JE0497)  
**Discipline:** Mechanical Engineering / Theory of Machines & IC Engines

[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-blue.svg)](https://www.python.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Tests: 11 Passed](https://img.shields.io/badge/Tests-11%20Passed-brightgreen.svg)](tests/)
[![Speed Range: 0-8000 RPM](https://img.shields.io/badge/Speed%20Range-0%20--%208000%20RPM-red.svg)]()
[![Google Colab](https://img.shields.io/badge/Google%20Colab-Ready-orange.svg)](Slider_Crank_Simulation_Colab.ipynb)

---

## 🚀 Quick Links for Users
- 🌐 **[Live Browser Simulator](index.html)** — Run the 60 FPS interactive HTML5 simulation directly with no software installation.
- 📓 **[Interactive Google Colab Notebook](Slider_Crank_Simulation_Colab.ipynb)** — Run in the cloud with live parameter sweeps and graphs.
- 📄 **[Executive Engineering Report](reports/engineering_report.html)** — View the auto-generated HTML design analysis report.
- 💻 **Desktop Application:** Run [`run_gui.bat`](run_gui.bat) for native 2D animated desktop simulation with live scrubbing controls.

---

## 1. Project Overview & Aim

This project provides an interactive, mathematically rigorous simulation and analysis software tool for the **Slider-Crank Mechanism**—the foundational mechanism used in internal combustion (IC) engines, reciprocating compressors, and pumps.

The software models both **inline** and **offset** slider-crank configurations, computes exact closed-form kinematics and dynamic force distributions throughout complete $360^\circ$ and $720^\circ$ cycles, generates publication-quality performance graphs, simulates real-time 2D motion with animated vector overlays, and calculates flywheel sizing and vibration harmonic spectra.

---

## 2. Key Features

- **Kinematic Simulation Engine**:
  - Operating speed range configured from **0 RPM to 8000 RPM** (from static equilibrium to high-speed race engines).
  - Exact closed-form equations for slider displacement $s(\theta)$, velocity $v(\theta)$, and acceleration $a(\theta)$.
  - Exact angular displacement $\phi(\theta)$, angular velocity $\omega_c(\theta)$, and angular acceleration $\alpha_c(\theta)$ of the connecting rod.
  - Automatic comparison against classical series approximations ($v \approx -r\omega(\sin\theta + \frac{\sin 2\theta}{2n})$).
- **Dynamic Force & Torque Analysis**:
  - Indicator diagram modeling: Polytropic expansion ($PV^\gamma = C$), 4-stroke Otto cycle, 2-stroke cycle, and reciprocating compressor cycles.
  - Inertia forces of reciprocating parts ($F_I = m_{\text{rec}} \cdot a$).
  - Net piston effort ($F_P = F_g + F_I$).
  - Force transmission along connecting rod ($F_Q = F_P / \cos\phi$).
  - Normal cylinder wall side thrust ($F_N = F_P \tan\phi$).
  - Tangential crank effort ($F_t$) and Turning Moment Diagram ($T(\theta) = F_t \cdot r$).
  - Primary and secondary unbalanced shaking forces.
- **Turning Moment & Flywheel Design**:
  - Numerical integration of mean engine torque $T_{\text{mean}}$.
  - Maximum fluctuation of energy $\Delta E$ and coefficient of fluctuation of energy $C_e$.
  - Required flywheel mass moment of inertia $I_{\text{flywheel}}$ and rim sizing for specified coefficient of speed fluctuation $C_s$.
  - 8-order Fourier harmonic decomposition of engine torque.
- **Interactive Visualizations**:
  - **Desktop GUI (Tkinter & Matplotlib)**: Real-time 2D animated mechanism with joints, piston rings, cylinder wall hatching, live HUD readout, parameter scrubbers, and multi-tab performance graphs.
  - **Standalone Interactive Web Simulator (HTML5 Canvas + Chart.js)**: Runs at 60 FPS in any browser without needing Python installation.
  - **Command-Line Interface (CLI)**: Automated batch analysis, CSV data export, and executive HTML report generation.
- **Validation Suite**:
  - 11 comprehensive unit tests validating exact dead-center boundary values, speed range (0 to 8000 RPM), finite-difference numerical differentiation ($O(h^2)$), energy conservation over closed cycles, and standard ADAMS / MATLAB / textbook benchmarks.

---

## 3. Mathematical Formulation

### 3.1 Kinematics
For a crank of radius $r$, connecting rod of length $l$, offset $e$, and crank angle $\theta = \omega t$:
- Obliquity ratio: $n = \frac{l}{r}$
- Connecting rod angle $\phi$:
  $$\sin\phi = \frac{r \sin\theta - e}{l}, \quad \cos\phi = \sqrt{1 - \sin^2\phi}$$
- Piston position from crank center:
  $$x_B(\theta) = r \cos\theta + l \cos\phi$$
- Piston velocity (exact closed-form):
  $$v_B(\theta) = -\omega r \left( \sin\theta + \tan\phi \cos\theta \right)$$
- Piston acceleration (exact closed-form):
  $$a_B(\theta) = -\omega^2 r \left[ \cos\theta - \tan\phi \sin\theta + \frac{r \cos^2\theta}{l \cos^3\phi} \right]$$
- Connecting rod angular velocity & acceleration:
  $$\omega_c(\theta) = \frac{\omega r \cos\theta}{l \cos\phi}, \quad \alpha_c(\theta) = \frac{d\omega_c}{dt}$$

### 3.2 Dynamics & Forces
- Reciprocating mass: $m_{\text{rec}} = m_{\text{piston}} + m_{\text{rod}}(1 - k_{\text{cg}})$
- Gas pressure force: $F_g = (P_{\text{cylinder}} - P_{\text{atm}}) \cdot \frac{\pi D^2}{4}$
- Reciprocating inertia force: $F_I = m_{\text{rec}} \cdot a_B$
- Piston effort: $F_P = F_g + F_I$
- Connecting rod force: $F_Q = \frac{F_P}{\cos\phi}$
- Normal cylinder side thrust: $F_N = F_P \tan\phi$
- Crank effort: $F_t = F_Q \sin(\theta + \phi) = \frac{F_P \sin(\theta + \phi)}{\cos\phi}$
- Crankshaft turning moment: $T = F_t \cdot r$

### 3.3 Flywheel Sizing
- Mean torque: $T_{\text{mean}} = \frac{1}{2\pi} \int_0^{2\pi} T(\theta) d\theta$
- Maximum energy fluctuation: $\Delta E = \max\left(\int (T - T_{\text{mean}})d\theta\right) - \min\left(\int (T - T_{\text{mean}})d\theta\right)$
- Required flywheel inertia for speed fluctuation $C_s$:
  $$I_{\text{flywheel}} = \frac{\Delta E}{C_s \cdot \omega^2}$$

---

## 4. Project File Structure

```
Automobile_Project/
├── engine/                   # Core scientific calculation engine
│   ├── __init__.py
│   ├── mechanism.py          # Mechanism parameters, coordinates, geometric validation
│   ├── kinematics.py         # Closed-form exact & series kinematics, finite differences
│   ├── dynamics.py           # Force equilibrium, side thrust, crank effort, torque
│   ├── gas_pressure.py       # Indicator diagrams (Otto, 2-stroke, compressor, polytropic)
│   └── analysis.py           # Cycle analyzer, flywheel design, Fourier harmonic FFT
├── gui/                      # Desktop graphical interface
│   ├── __init__.py
│   ├── app.py                # Full Tkinter Desktop GUI with animation & parameter controls
│   ├── mechanism_canvas.py   # 2D schematic renderer (links, piston, cylinder, traces)
│   └── plot_panels.py        # Multi-tab subplots (kinematics, dynamics, TMD, polar)
├── web/                      # Browser-based interactive simulation
│   └── index.html            # Standalone HTML5 Canvas + Chart.js web application
├── tests/                    # Verification & benchmark test suite
│   ├── test_kinematics.py    # Analytical boundary checks and numerical derivatives
│   ├── test_dynamics.py      # Force equilibrium and energy conservation checks
│   ├── test_benchmarks.py    # Norton/Khurmi/ADAMS literature benchmark comparison
│   └── test_analysis.py      # Flywheel sizing and approximation convergence tests
├── reports/                  # Generated publication plots & HTML engineering reports
│   ├── engineering_report.html
│   ├── kinematics_analysis.png
│   ├── dynamics_forces.png
│   ├── turning_moment_diagram.png
│   └── harmonics_polar.png
├── export/                   # Exported simulation CSV files
│   └── slider_crank_data.csv
├── main.py                   # Unified CLI and application runner
├── run_gui.bat               # One-click Windows desktop GUI launcher
├── run_cli.bat               # One-click CLI batch runner
├── run_tests.bat             # One-click test runner
├── run_web.bat               # One-click interactive browser simulator launcher
├── requirements.txt          # Python dependencies
└── README.md                 # Project documentation
```

---

## 5. How to Run

### Method 1: Interactive Desktop GUI (Recommended)
Double-click `run_gui.bat` or run:
```powershell
python gui/app.py
```
*Features:*
- Choose engine presets from dropdown (Petrol, Diesel, Motorcycle, Compressor, Offset).
- Real-time 2D animated mechanism with play/pause, step controls, and angle scrub slider.
- Real-time performance tabs for Kinematics, Dynamics, Turning Moment Diagrams, and Harmonics.
- Export simulation CSV, save high-resolution plots, and generate analysis reports.

### Method 2: Interactive Web Simulator (No Setup Required)
Double-click `run_web.bat` or open `web/index.html` in any web browser (Chrome, Edge, Firefox).
Runs a 60 FPS client-side simulation with live parameter sliders and interactive Chart.js graphs.

### Method 3: Command-Line Interface (CLI) & Automated Report Generation
Run `run_cli.bat` or run:
```powershell
python main.py --plots --report
```
To run custom parameters:
```powershell
python main.py --crank 60 --conrod 240 --rpm 2400 --bore 85 --ppeak 55 --plots --report
```

### Method 4: Run Validation Test Suite
Double-click `run_tests.bat` or run:
```powershell
pytest tests -v
```

---

## 6. Validation & Benchmark Summary

| Test Case | Analytical / Benchmark Value | Simulation Output | Status |
| :--- | :--- | :--- | :--- |
| **TDC Velocity ($0^\circ$)** | $0.000\text{ m/s}$ | $0.000\text{ m/s}$ | **PASSED** (Error $< 10^{-10}$) |
| **BDC Velocity ($180^\circ$)** | $0.000\text{ m/s}$ | $0.000\text{ m/s}$ | **PASSED** (Error $< 10^{-10}$) |
| **TDC Acceleration** | $-r\omega^2(1 + 1/n) = -1184.35\text{ m/s}^2$ | $-1184.35\text{ m/s}^2$ | **PASSED** (Exact) |
| **BDC Acceleration** | $+r\omega^2(1 - 1/n) = +710.61\text{ m/s}^2$ | $+710.61\text{ m/s}^2$ | **PASSED** (Exact) |
| **Finite Difference Vel.** | Central difference $O(h^2)$ | Analytical match | **PASSED** (Max Error $< 0.001\text{ m/s}$) |
| **Finite Difference Acc.** | Central difference $O(h^2)$ | Analytical match | **PASSED** (Max Error $< 0.5\text{ m/s}^2$) |
| **Conservative Work Check** | $\oint T_{\text{inertia}} d\theta = 0\text{ J}$ | $0.000\text{ J}$ | **PASSED** (Net energy = 0) |
| **Speed Range Stability** | $0 \le \text{RPM} \le 8000$ (Static to High Speed) | $0\text{ m/s}$ at $0\text{ RPM}$; stable at $8000\text{ RPM}$ | **PASSED** (Full range) |
| **Textbook Benchmark (Norton)** | $r=60\text{mm}, l=240\text{mm}, \theta=45^\circ$ | $\phi = 10.18^\circ, v = -9.41\text{ m/s}$ | **PASSED** |

---

## 7. Educational & Industrial Significance

This project provides:
1. **Zero-Cost Engineering Analysis**: Eliminates reliance on expensive commercial multibody dynamics packages (ADAMS, ANSYS Motion) for standard reciprocating engine design.
2. **Theory of Machines Pedagogy**: Bridges textbook mechanical principles with software computation and interactive graphical visualization.
3. **Engineering Synthesis**: Integrates kinematics, thermodynamics (indicator diagrams), multi-body dynamics, flywheel design, and torsional vibration analysis into a unified tool.

---

## 8. Publishing to GitHub & Free Cloud Hosting

### Step 1: Push Repository to Your GitHub Account
Create a new repository named `slider-crank-mechanism` on [GitHub](https://github.com/new), then run:
```powershell
git remote add origin https://github.com/<YOUR-GITHUB-USERNAME>/slider-crank-mechanism.git
git branch -M main
git push -u origin main
```

### Step 2: Enable Instant Free Web Simulator (GitHub Pages)
1. Go to your repository on GitHub.
2. Click **Settings** $\rightarrow$ **Pages** (in the left sidebar).
3. Under **Branch**, select `main` and folder `/ (root)`, then click **Save**.
4. GitHub will give you a public URL (e.g. `https://<YOUR-USERNAME>.github.io/slider-crank-mechanism/`). Anyone can now open and use your interactive simulator in their browser!

### Step 3: Run in Google Colab (Cloud Notebook)
Open [Google Colab](https://colab.research.google.com/), click **GitHub** tab, paste your repository URL, and open `Slider_Crank_Simulation_Colab.ipynb` to run the simulation directly in the cloud!
