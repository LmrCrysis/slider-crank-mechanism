# DESIGN AND SIMULATION OF A SLIDER-CRANK MECHANISM
## Python-Based Kinematic and Dynamic Analysis Tool

---

### **A Project Report Submitted in Partial Fulfillment of the Requirements for Theory of Machines & Machine Dynamics**

**Project Title:** Design and Simulation of a Slider-Crank Mechanism using a Python-Based Kinematic and Dynamic Analysis Tool  
**Candidate Name:** Dhruv Chauhan  
**Registration / Roll Number:** 24JE0497  
**Department:** Mechanical / Automobile Engineering  
**Academic Year:** 2026–2027  

---

## CERTIFICATE OF AUTHENTICITY

This is to certify that the project entitled **"Design and Simulation of a Slider-Crank Mechanism using a Python-Based Kinematic and Dynamic Analysis Tool"** submitted by **Dhruv Chauhan (24JE0497)** is a bonafide record of original computational and analytical engineering work developed and completed under the curriculum requirements of Theory of Machines, Dynamics of Machinery, and Computational Mechanical Engineering.

---

## TABLE OF CONTENTS

1. [Abstract & Executive Summary](#1-abstract--executive-summary)
2. [Chapter 1: Introduction & Problem Statement](#chapter-1-introduction--problem-statement)
   - 1.1 Background of the Slider-Crank Mechanism
   - 1.2 Motivation and Industry Relevance
   - 1.3 Project Aim and Specific Objectives
   - 1.4 Operating Scope & Specifications
3. [Chapter 2: Mathematical Formulation & Theoretical Derivations](#chapter-2-mathematical-formulation--theoretical-derivations)
   - 2.1 Geometric Model & Kinematic Configuration
   - 2.2 Exact Closed-Form Kinematics (Displacement, Velocity, Acceleration)
   - 2.3 Classical Series Approximations & Truncation Errors
   - 2.4 Connecting Rod Angular Motion ($\phi, \omega_c, \alpha_c$)
   - 2.5 Thermodynamic Gas Pressure & Indicator Diagram Modeling
   - 2.6 Dynamic Force Analysis & Equilibrium
   - 2.7 Crankshaft Turning Moment Diagram (TMD)
   - 2.8 Engine Cycle Energy Fluctuation & Flywheel Sizing
   - 2.9 Fourier Harmonic Torsional Vibration Analysis
4. [Chapter 3: Software Architecture & Implementation](#chapter-3-software-architecture--implementation)
   - 3.1 Modular System Design
   - 3.2 Scientific Computation Engine (`engine/`)
   - 3.3 Interactive Desktop Application (`gui/`)
   - 3.4 Standalone Cross-Platform Web Simulator (`web/`)
   - 3.5 Command-Line Interface & Automated Reporting (`main.py`)
5. [Chapter 4: Results, Discussion & Visual Analysis](#chapter-4-results-discussion--visual-analysis)
   - 4.1 Kinematic Response Across Crank Rotation
   - 4.2 Dynamic Force Variations & Cylinder Wall Side Thrust
   - 4.3 Turning Moment Characteristics & Power Calculation
   - 4.4 Flywheel Sizing Specifications
   - 4.5 Harmonic Order Decomposition Spectrum
   - 4.6 Parametric Case Studies (High-Speed Engine, Offset Cylinder, Compressor)
6. [Chapter 5: Verification, Validation & Benchmark Comparisons](#chapter-5-verification-validation--benchmark-comparisons)
   - 5.1 Analytical Boundary Checks at Dead Centers (TDC & BDC)
   - 5.2 Finite-Difference Numerical Differentiation Verification
   - 5.3 First-Law Cyclic Energy Conservation Check
   - 5.4 Literature Benchmark Validation (Norton, Khurmi, MATLAB/ADAMS)
   - 5.5 Summary of Automated Unit Test Suite (11/11 Passed)
7. [Chapter 6: Conclusions & Future Scope](#chapter-6-conclusions--future-scope)
8. [References & Bibliography](#references--bibliography)

---

## 1. ABSTRACT & EXECUTIVE SUMMARY

The slider-crank mechanism is the primary kinematic and dynamic linkage utilized in internal combustion (IC) engines, reciprocating air compressors, and positive displacement pumps. While commercial multibody dynamics packages (such as MSC ADAMS and ANSYS Motion) are standard in large automotive corporations, their high licensing costs, substantial computational overhead, and steep learning curves hinder rapid preliminary design and educational exploration.

This project presents the complete mathematical derivation, computational engine development, and interactive simulation of an advanced **Python-Based Slider-Crank Kinematic and Dynamic Analysis Tool**. The system incorporates both **inline** and **offset** mechanism geometries, operating continuously across a user-defined speed range of **$0\text{ to }8000\text{ RPM}$**.

Key capabilities developed in this project include:
1. Exact closed-form analytical kinematic solutions compared against classical second-order binomial series approximations.
2. Full dynamic force resolution comprising reciprocating inertia forces ($F_I = m_{\text{rec}} a$), polytropic gas indicator diagrams ($PV^\gamma = C$), net driving piston effort ($F_P$), connecting rod axial thrust ($F_Q$), and normal cylinder wall side thrust ($F_N$).
3. Turning Moment Diagram (TMD) numerical integration for mean indicated torque ($T_{\text{mean}}$), power output (kW), maximum fluctuation of energy ($\Delta E$), and precision flywheel mass moment of inertia sizing ($I_{\text{flywheel}}$).
4. Fast Fourier Transform (FFT) harmonic decomposition (orders $1\theta$ through $8\theta$) to quantify torsional excitation harmonics.
5. Dual-interface deployment: an interactive native Desktop GUI (Tkinter + Matplotlib) and a client-side 60 FPS Web Simulator (HTML5 Canvas + Chart.js).
6. Complete validation using an 11-test automated suite verifying dead-center boundary conditions, $O(h^2)$ central finite-difference derivatives, cyclic energy conservation, and published benchmarks from Norton and ADAMS.

---

## CHAPTER 1: INTRODUCTION & PROBLEM STATEMENT

### 1.1 Background of the Slider-Crank Mechanism
The slider-crank is a four-bar kinematic chain comprising one sliding pair and three revolute pairs:
* **Link 1 (Frame / Cylinder Block):** Fixed reference ground.
* **Link 2 (Crank):** Driver link undergoing pure continuous rotation about the main bearing axis $O$.
* **Link 3 (Connecting Rod):** Floating link undergoing complex planar motion (translation and rotation).
* **Link 4 (Slider / Piston):** Driven link undergoing pure linear reciprocating motion inside the cylinder guide.

In IC engines, high-pressure gas combustion exerts a driving thrust on the piston crown, translating reciprocating linear motion into continuous rotational torque at the crankshaft. In compressors, external electric or mechanical torque drives the crankshaft to compress gases.

### 1.2 Motivation and Industry Relevance
In engine research and development (R&D), selecting the optimal crank radius ($r$), connecting rod length ($l$), cylinder offset ($e$), and reciprocating mass is critical. A connecting rod that is too short increases cylinder wall side-thrust, causing premature liner ovality, friction loss, and piston slap. Conversely, an excessively long connecting rod increases engine height and weight. Furthermore, at high engine speeds (e.g. up to 8,000 RPM), reciprocating inertia forces scale quadratically ($\propto \text{RPM}^2$), creating intense structural stresses on rod bolts and wrist pins.

Automotive engineers need rapid 1D computational simulation tools during preliminary architecture sizing to evaluate hundreds of geometric iterations before committing to 3D CAD modeling and finite element analysis (FEA).

### 1.3 Project Aim and Specific Objectives
**Aim:** To design, build, and validate an interactive, lightweight computational software tool in Python that rigorously models the kinematics and dynamics of a slider-crank mechanism throughout complete crank cycles.

**Specific Objectives:**
1. Derive exact closed-form kinematic equations for displacement, velocity, and acceleration of the piston, as well as the angular position, angular velocity, and angular acceleration of the connecting rod.
2. Formulate and resolve dynamic equilibrium equations including gas pressure forces, reciprocating inertia, net driving piston effort, con-rod axial thrust, and cylinder side thrust.
3. Integrate the Turning Moment Diagram (TMD) to evaluate mean indicated torque, indicated engine power, and energy fluctuation ($\Delta E$) for flywheel sizing.
4. Implement Fast Fourier Transform (FFT) harmonic order analysis to assess torsional vibration excitation frequencies.
5. Provide real-time 2D vector animation and multi-tab performance plotting via an interactive Desktop GUI and a cross-platform Web Simulator.
6. Validate all computational outputs against analytical dead-center values, central finite differences, and standard literature benchmarks.

### 1.4 Operating Scope & Specifications
* **Rotational Speed Range:** $0\text{ to }8000\text{ RPM}$ (supporting static equilibrium to high-speed motorsport applications).
* **Geometry Modes:** Inline slider-crank ($e = 0$) and Desaxé / Offset slider-crank ($e \neq 0$).
* **Cycle Types:** 4-Stroke IC Engine, 2-Stroke Engine, Reciprocating Compressor, and Polytropic Expansion models.

---

## CHAPTER 2: MATHEMATICAL FORMULATION & THEORETICAL DERIVATIONS

### 2.1 Geometric Model & Kinematic Configuration
Let:
* $r$ = Crank radius (m)
* $l$ = Connecting rod length (m)
* $e$ = Offset distance of slider stroke axis relative to crank center (m)
* $n = \frac{l}{r}$ = Obliquity ratio / connecting rod ratio
* $\theta = \omega t$ = Crank angle measured from Inner/Top Dead Center (rad)
* $\phi$ = Angle of connecting rod relative to the cylinder stroke axis (rad)
* $\omega = \frac{2\pi \cdot \text{RPM}}{60}$ = Crankshaft angular velocity (rad/s)

```
        Y
        ^           A (Crank pin: xA = r*cosθ, yA = r*sinθ)
        |          / \
        |      r  /   \  l (Connecting rod)
        |        /  φ  \
        +-------O-------B========[ PISTON (xB, e) ]-----> X (Line of stroke)
             (Crank axis)
```

From planar vector loop closure along the transverse $Y$-axis:
$$r \sin\theta = l \sin\phi + e$$

Rearranging for the connecting rod angle $\phi$:
$$\sin\phi = \frac{r \sin\theta - e}{l}$$
$$\cos\phi = \sqrt{1 - \sin^2\phi} = \sqrt{1 - \left(\frac{r \sin\theta - e}{l}\right)^2}$$

For an inline slider-crank ($e = 0$):
$$\sin\phi = \frac{\sin\theta}{n}, \quad \cos\phi = \sqrt{1 - \frac{\sin^2\theta}{n^2}}$$

### 2.2 Exact Closed-Form Kinematics

#### 1. Piston Position & Displacement ($s$)
The instantaneous coordinate of the piston wrist pin $B$ from the crankshaft rotation axis $O$ along the line of stroke is:
$$x_B(\theta) = r \cos\theta + l \cos\phi = r \cos\theta + \sqrt{l^2 - (r \sin\theta - e)^2}$$

At Top Dead Center (TDC, $\theta = 0$ for inline):
$$x_{\text{TDC}} = \sqrt{(l + r)^2 - e^2}$$

At Bottom Dead Center (BDC, $\theta = \pi$ for inline):
$$x_{\text{BDC}} = \sqrt{(l - r)^2 - e^2}$$

Total piston stroke length $S$:
$$S = x_{\text{TDC}} - x_{\text{BDC}}$$

The displacement of the piston from TDC as a function of crank angle is:
$$s(\theta) = x_{\text{TDC}} - x_B(\theta)$$

#### 2. Piston Velocity ($v$)
Differentiating position $x_B$ with respect to time $t$:
$$v_B = \frac{dx_B}{dt} = \omega \frac{dx_B}{d\theta}$$

Evaluating $\frac{dx_B}{d\theta}$:
$$\frac{dx_B}{d\theta} = -r \sin\theta - l \sin\phi \frac{d\phi}{d\theta}$$

Differentiating $\sin\phi = \frac{r \sin\theta - e}{l}$ with respect to $\theta$:
$$\cos\phi \frac{d\phi}{d\theta} = \frac{r \cos\theta}{l} \implies \frac{d\phi}{d\theta} = \frac{r \cos\theta}{l \cos\phi}$$

Substituting $\frac{d\phi}{d\theta}$ into the velocity equation:
$$l \sin\phi \frac{d\phi}{d\theta} = l \sin\phi \left(\frac{r \cos\theta}{l \cos\phi}\right) = r \tan\phi \cos\theta$$

Thus, the **exact closed-form piston velocity** is:
$$v_B(\theta) = -\omega r \left( \sin\theta + \tan\phi \cos\theta \right)$$

*(The negative sign signifies velocity directed toward the crank center $O$ during the expansion stroke from $\theta = 0$ to $\theta = \pi$).*

#### 3. Piston Acceleration ($a$)
Differentiating velocity with respect to time $t$:
$$a_B = \frac{dv_B}{dt} = \omega \frac{dv_B}{d\theta} = \omega^2 \frac{d^2x_B}{d\theta^2}$$

Differentiating $\frac{dx_B}{d\theta} = -r \sin\theta - r \tan\phi \cos\theta$:
$$\frac{d^2x_B}{d\theta^2} = -r \cos\theta - r \left[ \sec^2\phi \frac{d\phi}{d\theta} \cos\theta - \tan\phi \sin\theta \right]$$

Substituting $\frac{d\phi}{d\theta} = \frac{r \cos\theta}{l \cos\phi}$:
$$\frac{d^2x_B}{d\theta^2} = -r \cos\theta - r \left[ \frac{r \cos^2\theta}{l \cos^3\phi} - \tan\phi \sin\theta \right]$$

Therefore, the **exact closed-form piston acceleration** is:
$$a_B(\theta) = -\omega^2 r \left[ \cos\theta - \tan\phi \sin\theta + \frac{r \cos^2\theta}{l \cos^3\phi} \right]$$

### 2.3 Classical Series Approximations & Truncation Errors
In classical Theory of Machines textbooks (e.g., Khurmi, Norton, Rattan), binomial expansion is applied assuming $n \ge 3$:
$$\cos\phi = \left(1 - \frac{\sin^2\theta}{n^2}\right)^{1/2} \approx 1 - \frac{\sin^2\theta}{2n^2}$$

This yields the widely used classical approximations for inline mechanisms ($e = 0$):
* **Approximate Displacement:**
  $$s_{\text{approx}}(\theta) \approx r \left[ (1 - \cos\theta) + \frac{\sin^2\theta}{2n} \right]$$
* **Approximate Velocity:**
  $$v_{\text{approx}}(\theta) \approx -\omega r \left( \sin\theta + \frac{\sin 2\theta}{2n} \right)$$
* **Approximate Acceleration:**
  $$a_{\text{approx}}(\theta) \approx -\omega^2 r \left( \cos\theta + \frac{\cos 2\theta}{n} \right)$$

**Key Theoretical Boundary Observations:**
1. At **TDC ($\theta = 0^\circ$):** $\cos 0 = 1$, $\cos 2(0) = 1$. The exact and approximate accelerations converge to:
   $$a_{\text{TDC}} = -\omega^2 r \left(1 + \frac{1}{n}\right)$$
2. At **BDC ($\theta = 180^\circ$):** $\cos \pi = -1$, $\cos 2\pi = 1$:
   $$a_{\text{BDC}} = +\omega^2 r \left(1 - \frac{1}{n}\right)$$
3. The secondary acceleration term $\frac{\cos 2\theta}{n}$ occurs at twice crank frequency ($2\omega$). As obliquity ratio $n$ increases, the truncation error of the classical approximation decreases monotonically.

### 2.4 Connecting Rod Angular Motion ($\phi, \omega_c, \alpha_c$)
The angular velocity of the connecting rod $\omega_c = \frac{d\phi}{dt}$ is:
$$\omega_c(\theta) = \omega \frac{d\phi}{d\theta} = \frac{\omega r \cos\theta}{l \cos\phi}$$

Differentiating with respect to time yields the connecting rod angular acceleration $\alpha_c$:
$$\alpha_c(\theta) = -\frac{\omega^2 r \left[ \sin\theta \cos^2\phi - (r/l)\cos^2\theta \sin\phi \right]}{l \cos^3\phi}$$

For an inline mechanism ($e = 0$):
$$\omega_c(\theta) = \frac{\omega \cos\theta}{\sqrt{n^2 - \sin^2\theta}}, \quad \alpha_c(\theta) = -\frac{\omega^2 (n^2 - 1)\sin\theta}{(n^2 - \sin^2\theta)^{3/2}}$$

### 2.5 Thermodynamic Gas Pressure & Indicator Diagram Modeling
Cylinder pressure varies throughout the 4 strokes or 2 strokes according to polytropic process thermodynamics:
$$P(\theta) \cdot V(\theta)^\gamma = \text{Constant}$$

Where:
* $V(\theta) = V_c + A_p \cdot s(\theta)$ (Instantaneous cylinder volume)
* $V_c = \frac{V_s}{r_c - 1}$ (Clearance volume, with compression ratio $r_c$)
* $V_s = A_p \cdot S$ (Swept volume)
* $A_p = \frac{\pi D^2}{4}$ (Bore cross-sectional area)
* $\gamma \approx 1.30\text{ to }1.35$ (Polytropic expansion index)

The net combustion gas force driving the piston is:
$$F_g(\theta) = \left[ P(\theta) - P_{\text{atm}} \right] \cdot A_p$$

### 2.6 Dynamic Force Analysis & Equilibrium

```
                  F_N (Side Thrust on Cylinder Wall)
                        ^
                        |
  F_g (Gas) ---> [ PISTON ] ---> F_P (Net Effort)
  F_I (Inertia)->       \
                         \  F_Q (Along Con-Rod)
                          \
                           v
                           (A) Crank Pin
```

#### 1. Equivalent Dynamically Balanced Two-Mass System
The connecting rod (mass $m_{\text{rod}}$) undergoes combined translation and rotation. It is modeled as a dynamically equivalent two-mass system:
* Reciprocating mass concentrated at the piston wrist pin $B$:
  $$m_{\text{rec}} = m_{\text{piston}} + m_{\text{rod}} \left(1 - k_{\text{cg}}\right)$$
* Rotating mass concentrated at the crank pin $A$:
  $$m_{\text{rot}} = m_{\text{rod}} \cdot k_{\text{cg}}$$
  *(where $k_{\text{cg}} = \frac{l_{\text{cg}}}{l} \approx 0.35$ is the location of rod CG from crank end).*

#### 2. Reciprocating Inertia Force ($F_I$)
According to D'Alembert's principle, the inertia force opposes piston acceleration:
$$F_I(\theta) = -m_{\text{rec}} \cdot a_B(\theta)$$

#### 3. Net Piston Driving Effort ($F_P$)
The net resultant driving force transmitted down the cylinder axis:
$$F_P(\theta) = F_g(\theta) + F_I(\theta)$$

#### 4. Connecting Rod Force ($F_Q$)
Resolving along the inclined axis of the connecting rod:
$$F_Q(\theta) = \frac{F_P(\theta)}{\cos\phi}$$

#### 5. Normal Cylinder Side Thrust ($F_N$)
The lateral reaction force exerted by the piston skirt on the cylinder liner:
$$F_N(\theta) = F_P(\theta) \tan\phi$$

#### 6. Tangential Crank Effort ($F_t$)
The force component acting perpendicular to the crank arm at crank pin $A$:
$$F_t(\theta) = F_Q \sin(\theta + \phi) = \frac{F_P \sin(\theta + \phi)}{\cos\phi}$$

For an inline mechanism:
$$F_t(\theta) = F_P \left( \sin\theta + \frac{\sin 2\theta}{2\sqrt{n^2 - \sin^2\theta}} \right)$$

### 2.7 Crankshaft Turning Moment Diagram (TMD)
The instantaneous torsional turning moment (torque) transmitted to the crankshaft is:
$$T(\theta) = F_t(\theta) \cdot r$$

The mean indicated torque over a complete $2\pi$ cycle is evaluated by numerical integration:
$$T_{\text{mean}} = \frac{1}{2\pi} \int_0^{2\pi} T(\theta) \, d\theta$$

The indicated power output ($P$) in kilowatts is:
$$P = \frac{T_{\text{mean}} \cdot \omega}{1000}$$

### 2.8 Engine Cycle Energy Fluctuation & Flywheel Sizing
Torque output is highly non-uniform over the cycle. The difference between instantaneous torque and mean resisting torque causes angular speed fluctuations.
The cumulative excess energy curve is:
$$\Delta E(\theta) = \int_0^\theta \left[ T(\theta') - T_{\text{mean}} \right] d\theta'$$

The **Maximum Fluctuation of Energy ($\Delta E$)** is:
$$\Delta E = \max[\Delta E(\theta)] - \min[\Delta E(\theta)]$$

The **Coefficient of Fluctuation of Energy ($C_e$)**:
$$C_e = \frac{\Delta E}{\text{Work per cycle}} = \frac{\Delta E}{2\pi T_{\text{mean}}}$$

For an allowable **Coefficient of Fluctuation of Speed ($C_s = \frac{\omega_{\text{max}} - \omega_{\text{min}}}{\omega_{\text{mean}}} \approx 0.02$)**, the required flywheel mass moment of inertia is:
$$I_{\text{flywheel}} = \frac{\Delta E}{C_s \cdot \omega_{\text{mean}}^2}$$

Given a flywheel rim radius $R_{\text{rim}} \approx 2.5 r$, the required rim mass is:
$$M_{\text{flywheel}} = \frac{I_{\text{flywheel}}}{R_{\text{rim}}^2}$$

### 2.9 Fourier Harmonic Torsional Vibration Analysis
The periodic torque function $T(\theta)$ is expanded into its discrete Fourier harmonic series:
$$T(\theta) = T_0 + \sum_{k=1}^N \left[ A_k \cos(k\theta) + B_k \sin(k\theta) \right]$$

The torque amplitude for each harmonic order $k$ (where $k = 1, 2, 3, \dots, 8$) is computed via Fast Fourier Transform (FFT):
$$C_k = \sqrt{A_k^2 + B_k^2}$$

Harmonic analysis enables NVH engineers to prevent torsional resonance where an engine order matches the natural frequency of the crankshaft.

---

## CHAPTER 3: SOFTWARE ARCHITECTURE & IMPLEMENTATION

### 3.1 Modular System Design
The project follows a clean decoupled architecture separating mathematical computation, user interface, automated testing, and report generation:

```
Automobile_Project/
├── engine/                       # Pure scientific computation (zero GUI dependencies)
│   ├── mechanism.py              # Parameter structures, geometry & validation
│   ├── kinematics.py             # Exact & series kinematics, finite differences
│   ├── dynamics.py               # Force equilibrium, side thrust, crank effort, torque
│   ├── gas_pressure.py           # Polytropic & indicator cycle thermodynamics
│   └── analysis.py               # TMD integration, flywheel sizing & FFT harmonics
├── gui/                          # Desktop application (Tkinter + Matplotlib)
│   ├── app.py                    # Main GUI loop, preset manager, parameter scrubbers
│   ├── mechanism_canvas.py       # 2D schematic renderer with live vector overlays
│   └── plot_panels.py            # Real-time multi-tab performance plotting panels
├── web/                          # Standalone web simulator (HTML5 + Chart.js)
│   └── index.html                # 60 FPS client-side interactive simulator
├── tests/                        # Automated unit tests & benchmark suite
│   ├── test_kinematics.py        # Boundary tests & numerical differentiation checks
│   ├── test_dynamics.py          # Dynamic force equilibrium & energy conservation
│   ├── test_benchmarks.py        # Comparison against Norton & ADAMS benchmarks
│   └── test_analysis.py          # Flywheel sizing and convergence rate tests
├── reports/                      # Auto-generated publication-grade assets
├── export/                       # Raw 360° CSV simulation data
├── main.py                       # Unified command-line interface (CLI)
├── index.html                    # Root GitHub Pages deployment file
├── Slider_Crank_Simulation_Colab.ipynb # Interactive Google Colab cloud notebook
└── README.md                     # Comprehensive technical documentation
```

### 3.2 Scientific Computation Engine (`engine/`)
The computational backend is implemented with vectorized NumPy routines:
* [`engine/mechanism.py`](file:///c:/Users/lenovo/Music/Automobile_Project/engine/mechanism.py): Implements `SliderCrankParameters` with boundary checks ($0 \le \text{RPM} \le 8000$, $l > r + |e|$). Safely evaluates zero-speed limits where $\omega = 0$ and period $\to \infty$.
* [`engine/kinematics.py`](file:///c:/Users/lenovo/Music/Automobile_Project/engine/kinematics.py): Implements exact and approximate linear/angular motion solvers. Includes an automated numerical differentiation routine using central finite differences ($O(h^2)$ with $h = 10^{-6}\text{ rad}$).
* [`engine/dynamics.py`](file:///c:/Users/lenovo/Music/Automobile_Project/engine/dynamics.py): Evaluates forces $F_g, F_I, F_P, F_Q, F_N, F_t$ and turning moment $T$.
* [`engine/gas_pressure.py`](file:///c:/Users/lenovo/Music/Automobile_Project/engine/gas_pressure.py): Generates indicator diagrams for 4-stroke Otto, 2-stroke, compressor, and polytropic cycles.
* [`engine/analysis.py`](file:///c:/Users/lenovo/Music/Automobile_Project/engine/analysis.py): Numerically integrates torque using trapezoidal quadrature, determines $\Delta E$, sizes the flywheel, and performs 8-order Fourier FFT decomposition.

### 3.3 Interactive Desktop Application (`gui/`)
Built using Python's native `tkinter` framework with embedded `matplotlib.backends.backend_tkagg`:
* **Parameter Sidebar:** Sliders and entries for crank length, con-rod length, offset, bore, RPM (0–8000), piston mass, and gas pressure.
* **Mechanism Presets:** 1-click loading for Petrol Engine, High-Speed Motorcycle, Heavy Diesel, Offset Slider-Crank, and Air Compressor.
* **2D Animated Canvas:** Displays the rotating crank arm (counterweighted), connecting rod with center-of-gravity ($G$) trajectory, wrist pin, piston body with piston rings, and cross-hatched cylinder guide walls.
* **Live Plot Panels:** 4 tabs for Kinematics, Dynamics, Turning Moment Diagrams, and Harmonics/Polar Torque, equipped with a live synchronized cursor.

### 3.4 Standalone Cross-Platform Web Simulator (`web/index.html` & `index.html`)
To enable universal access without local Python installation:
* Engineered with client-side HTML5 Canvas and Chart.js.
* Operates at 60 FPS in any browser (Chrome, Edge, Safari, mobile devices).
* Placed at repository root for instant 1-click deployment on **GitHub Pages**.

### 3.5 Command-Line Interface (`main.py`)
Provides automated headless simulation, batch parameter sweeps, 300 DPI figure exporting, and executive HTML report generation:
```powershell
python main.py --crank 50 --conrod 200 --rpm 100 --plots --report
```

---

## CHAPTER 4: RESULTS, DISCUSSION & VISUAL ANALYSIS

### 4.1 Kinematic Response Across Crank Rotation
Running the standard engine configuration ($r = 50\text{ mm}$, $l = 200\text{ mm}$, $n = 4.0$, $\text{RPM} = 100$):
* **Stroke Length ($S$):** $100.0\text{ mm}$ ($0.100\text{ m}$).
* **Mean Piston Speed ($v_{\text{mean}}$):** $0.33\text{ m/s}$.
* **Peak Piston Velocity ($v_{\text{max}}$):** $0.54\text{ m/s}$ occurring at $\theta = 76^\circ$ (due to connecting rod obliquity, peak velocity occurs before $90^\circ$).
* **Peak Acceleration ($a_{\text{max}}$):** $6.9\text{ m/s}^2$ at TDC ($\theta = 0^\circ$).
* **At 8000 RPM:** Peak piston velocity reaches $43.18\text{ m/s}$ and peak acceleration reaches $43,865\text{ m/s}^2$ ($4,470\text{ G's}$).

```
[ FIGURE 1: Kinematic Analysis — Displacement, Velocity, and Acceleration vs. Crank Angle ]
Path: reports/kinematics_analysis.png
```

### 4.2 Dynamic Force Variations & Cylinder Wall Side Thrust
* **Gas Force ($F_g$):** Peaks immediately after ignition at Top Dead Center ($F_{g,\text{peak}} \approx 22.6\text{ kN}$ at $45\text{ bar}$).
* **Inertia Force ($F_I$):** Opposes acceleration. At low speeds (100 RPM), gas pressure dominates. At high speeds (8000 RPM), inertia forces exceed $37\text{ kN}$, significantly offsetting combustion pressure near TDC and loading the rod bolts in tension during the exhaust stroke.
* **Side Thrust ($F_N$):** Reaches its maximum during the power stroke around $\theta = 30^\circ\text{ to }40^\circ$. Reducing obliquity ratio $n$ increases peak side thrust, directly correlating to cylinder liner wear.

```
[ FIGURE 2: Dynamic Force Variation — Gas Force, Inertia Force, Piston Effort, and Side Thrust ]
Path: reports/dynamics_forces.png
```

### 4.3 Turning Moment Characteristics & Power Output
* **Mean Indicated Torque ($T_{\text{mean}}$):** $37.0\text{ N}\cdot\text{m}$.
* **Peak Engine Torque ($T_{\text{max}}$):** $330.8\text{ N}\cdot\text{m}$.
* **Indicated Engine Power:** $0.39\text{ kW}$ at $100\text{ RPM}$; scales to $31.01\text{ kW}$ at $8000\text{ RPM}$.
* **Energy Fluctuation ($\Delta E$):** $278.8\text{ Joules}$.

```
[ FIGURE 3: Turning Moment Diagram (TMD) & Energy Fluctuation Curve ]
Path: reports/turning_moment_diagram.png
```

### 4.4 Flywheel Sizing Specifications
For a speed fluctuation coefficient $C_s = 0.02$ ($\pm 1\%$ speed regulation):
* At $100\text{ RPM}$ ($\omega = 10.47\text{ rad/s}$):
  $$I_{\text{req}} = \frac{278.8}{0.02 \times (10.47)^2} \approx 127.1\text{ kg}\cdot\text{m}^2$$
* At $1800\text{ RPM}$ ($\omega = 188.5\text{ rad/s}$):
  $$I_{\text{req}} = \frac{262.6}{0.02 \times (188.5)^2} \approx 0.3695\text{ kg}\cdot\text{m}^2 \implies M_{\text{rim}} \approx 23.65\text{ kg}$$

*(This demonstrates the inverse-square dependency of flywheel inertia on rotational speed: high-speed engines require significantly smaller and lighter flywheels to maintain identical speed stability).*

### 4.5 Harmonic Order Decomposition Spectrum
Fourier analysis decomposes the torque signal into harmonic engine orders:
* **Order $1\theta$ (Fundamental):** Dominant component corresponding to firing frequency.
* **Order $2\theta$ (Secondary):** Generated by connecting rod obliquity and reciprocating inertia.
* **Orders $3\theta$ to $8\theta$:** Higher-order harmonics critical for tuning torsional crankshaft dampers.

```
[ FIGURE 4: Harmonic Fourier Spectrum & Polar Torque Diagram ]
Path: reports/harmonics_polar.png
```

---

## CHAPTER 5: VERIFICATION, VALIDATION & BENCHMARK COMPARISONS

To guarantee academic and industrial rigor, the software was subjected to an **11-test automated verification suite** implemented in Pytest.

### 5.1 Analytical Boundary Checks at Dead Centers
At Dead Centers ($\theta = 0^\circ$ and $\theta = 180^\circ$):
* Velocity must equal zero identically:
  $$v(0) = 0.0000000000\text{ m/s}, \quad v(\pi) = 0.0000000000\text{ m/s}$$
* Acceleration must match theoretical limits:
  $$a_{\text{TDC}} = -\omega^2 r \left(1 + \frac{1}{n}\right), \quad a_{\text{BDC}} = +\omega^2 r \left(1 - \frac{1}{n}\right)$$
* Turning moment must equal zero:
  $$T(0) = 0.000\text{ N}\cdot\text{m}, \quad T(\pi) = 0.000\text{ N}\cdot\text{m}$$

### 5.2 Finite-Difference Numerical Differentiation Verification
Central finite-difference approximations ($h = 10^{-6}\text{ rad}$) verified exact closed-form derivative equations:
* Maximum velocity error across complete cycle: **$< 0.0001\text{ m/s}$**.
* Maximum acceleration error across complete cycle: **$< 0.01\text{ m/s}^2$**.

### 5.3 First-Law Cyclic Energy Conservation Check
In the absence of external gas pressure (pure reciprocating inertia), the First Law of Thermodynamics mandates that net work done over one complete $360^\circ$ cycle must equal zero:
$$\oint T_{\text{inertia}} \, d\theta = 0.000\text{ Joules}$$
The simulation yielded $\int_0^{2\pi} T_{\text{inertia}} \, d\theta = 0.0000\text{ J}$ within numerical integration tolerance ($< 0.01\text{ J}$ on $1000\text{ J}$ swings).

### 5.4 Literature Benchmark Validation (Norton "Design of Machinery")
* **Input Parameters:** $r = 60\text{ mm}$, $l = 240\text{ mm}$ ($n = 4.0$), $\text{RPM} = 1800$, $\theta = 45^\circ$, $m_p = 1.2\text{ kg}$, $P_{\text{gas}} = 1.5\text{ MPa}$.
* **Con-Rod Angle:** Analytical $\phi = 10.182^\circ$; Simulation $\phi = 10.182^\circ$ (**Exact match**).
* **Piston Velocity:** Analytical $v = -9.41\text{ m/s}$; Simulation $v = -9.41\text{ m/s}$ (**Exact match**).
* **Piston Effort & Torque:** Matched theoretical equilibrium within $0.001\%$.

### 5.5 Summary of Automated Unit Test Suite

| Test Identifier | Validation Target | Benchmark Standard | Test Result |
| :--- | :--- | :--- | :--- |
| `test_tdc_and_bdc_positions` | Dead-center velocity & acceleration | Analytical theoretical limit | **PASSED** |
| `test_finite_difference_derivatives` | Closed-form equations vs numerical | Central difference $O(h^2)$ | **PASSED** |
| `test_stroke_length` | Stroke length calculation (inline/offset) | Geometric theorem | **PASSED** |
| `test_speed_range_0_to_8000_rpm` | Stability at $0\text{ RPM}$ and $8000\text{ RPM}$ | Boundary edge case handling | **PASSED** |
| `test_torque_at_dead_centers` | Zero torque at dead centers | Equilibrium condition | **PASSED** |
| `test_force_balance_along_connecting_rod` | $F_Q \cos\phi = F_P$ and $F_N = F_P \tan\phi$ | Static & dynamic force equilibrium | **PASSED** |
| `test_pure_inertia_energy_conservation` | $\oint T_{\text{inertia}} d\theta = 0$ | First Law of Thermodynamics | **PASSED** |
| `test_standard_textbook_engine_benchmark`| Literature test problem (Norton) | Published benchmark values | **PASSED** |
| `test_adams_matlab_inline_slider_benchmark`| Multibody simulation comparison | MATLAB / MSC ADAMS benchmark | **PASSED** |
| `test_flywheel_sizing_and_harmonics` | TMD integration, $\Delta E$, FFT orders | Sizing equations & Fourier theorem | **PASSED** |
| `test_approximation_convergence_with_n`| Error decay as $n \to \infty$ | Series convergence theorem | **PASSED** |

**Execution Result:** `11 passed in 0.50s (100% Success Rate)`.

---

## CHAPTER 6: CONCLUSIONS & FUTURE SCOPE

### 6.1 Conclusions
1. A complete, mathematically rigorous Python simulation tool for slider-crank mechanisms was derived, developed, and validated.
2. The software models both inline and offset configurations across the complete operating speed range of **$0\text{ to }8000\text{ RPM}$**.
3. Closed-form exact kinematic and dynamic equations were implemented, eliminating truncation errors associated with classical second-order textbook approximations.
4. Real-time 2D animated visualizations, Turning Moment Diagrams, flywheel sizing routines, and FFT harmonic order spectra were successfully integrated into both a Desktop GUI and a cross-platform Web Simulator.
5. All mathematical outputs were rigorously validated through 11 automated unit tests matching published literature benchmarks.

### 6.2 Future Scope & Research Extensions
* **Multi-Cylinder Balancing:** Expanding the dynamic solver to multi-cylinder engines (Inline-4, V6, Crossplane V8) to compute secondary rocking couples and firing order balancing.
* **Elastohydrodynamic (EHD) Lubrication:** Coupling the cylinder side-thrust force ($F_N$) with the 2D Reynolds equation to simulate instantaneous oil-film thickness and friction loss.
* **Direct 3D CAD/FEA Integration:** Exporting instantaneous dynamic joint loads directly into SolidWorks Motion and ANSYS Mechanical for automated structural stress and fatigue analysis.

---

## REFERENCES & BIBLIOGRAPHY

1. **Norton, Robert L.** (2012). *Design of Machinery: An Introduction to the Synthesis and Analysis of Mechanisms and Machines*. 5th Edition, McGraw-Hill Education.
2. **Shigley, Joseph E., Uicker, John J., & Pennock, Gordon R.** (2016). *Theory of Machines and Mechanisms*. 5th Edition, Oxford University Press.
3. **Khurmi, R.S., & Gupta, J.K.** (2005). *Theory of Machines*. 14th Edition, S. Chand Publishing.
4. **Rattan, S.S.** (2014). *Theory of Machines*. 4th Edition, McGraw-Hill Education India.
5. **Heywood, John B.** (2018). *Internal Combustion Engine Fundamentals*. 2nd Edition, McGraw-Hill Education.
6. **Harris, C.R., et al.** (2020). *Array programming with NumPy*. Nature, 585(7825), 357–362.
7. **Hunter, J.D.** (2007). *Matplotlib: A 2D Graphics Environment*. Computing in Science & Engineering, 9(3), 90–95.
