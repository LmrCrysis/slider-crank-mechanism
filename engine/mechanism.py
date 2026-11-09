"""
Mechanism parameters and geometric definitions for Slider-Crank Analysis.
"""

from dataclasses import dataclass, field
import numpy as np


@dataclass
class SliderCrankParameters:
    """
    Physical and geometric parameters of a slider-crank mechanism.
    All inputs in SI units (meters, kg, seconds, radians, rpm).
    """
    crank_length: float = 0.05       # r: crank radius in meters (e.g. 50 mm)
    con_rod_length: float = 0.20     # l: connecting rod length in meters (e.g. 200 mm)
    offset: float = 0.0              # e: offset of slider stroke axis in meters (0 for inline)
    rpm: float = 100.0               # Crankshaft rotational speed in RPM (default: 100 RPM)
    piston_mass: float = 0.85        # m_p: mass of piston assembly in kg
    con_rod_mass: float = 1.20       # m_rod: mass of connecting rod in kg
    con_rod_cg_ratio: float = 0.35   # Distance of CG from crank-pin end / con_rod_length
    con_rod_inertia: float = 0.008   # I_G: moment of inertia about CG in kg*m^2 (0 for auto-estimate)
    bore_diameter: float = 0.080     # Cylinder bore diameter in meters (80 mm)
    crank_pin_radius: float = 0.015  # For realistic visual rendering
    slider_width: float = 0.085      # Visual piston width in meters
    slider_height: float = 0.065     # Visual piston height in meters
    name: str = "Standard Mechanism"

    def __post_init__(self):
        self.validate()
        if self.con_rod_inertia <= 0:
            # Slender rod approximation about CG: I_G = (1/12) * m * l^2
            self.con_rod_inertia = (1.0 / 12.0) * self.con_rod_mass * (self.con_rod_length ** 2)

    def validate(self):
        """Validate mechanism geometric assembly criteria."""
        if self.crank_length <= 0:
            raise ValueError("Crank length must be positive.")
        if self.con_rod_length <= 0:
            raise ValueError("Connecting rod length must be positive.")
        if self.con_rod_length <= self.crank_length + abs(self.offset):
            raise ValueError(
                f"Invalid mechanism geometry: connecting rod length ({self.con_rod_length} m) "
                f"must be greater than crank length + offset ({self.crank_length + abs(self.offset)} m) "
                f"to allow complete 360-degree rotation."
            )
        if self.rpm < 0 or self.rpm > 8000:
            raise ValueError(f"RPM ({self.rpm}) must be in the range 0 to 8000 RPM.")

    @property
    def r(self) -> float:
        return self.crank_length

    @property
    def l(self) -> float:
        return self.con_rod_length

    @property
    def e(self) -> float:
        return self.offset

    @property
    def n(self) -> float:
        """Obliquity ratio (connecting rod length / crank length)."""
        return self.con_rod_length / self.crank_length

    @property
    def omega(self) -> float:
        """Angular velocity of crank in rad/s."""
        return 2.0 * np.pi * self.rpm / 60.0

    @property
    def period(self) -> float:
        """Time for one complete revolution in seconds."""
        return 60.0 / self.rpm if self.rpm > 0 else float("inf")

    @property
    def x_tdc(self) -> float:
        """Piston coordinate at Top Dead Center / Inner Dead Center (m)."""
        # At TDC, crank and connecting rod extend to maximum distance along line of stroke
        hyp = self.l + self.r
        return float(np.sqrt(hyp**2 - self.e**2))

    @property
    def x_bdc(self) -> float:
        """Piston coordinate at Bottom Dead Center / Outer Dead Center (m)."""
        hyp = self.l - self.r
        return float(np.sqrt(hyp**2 - self.e**2))

    @property
    def stroke(self) -> float:
        """Total piston stroke length in meters."""
        return self.x_tdc - self.x_bdc

    @property
    def piston_area(self) -> float:
        """Cross-sectional area of cylinder bore in m^2."""
        return np.pi * (self.bore_diameter / 2.0) ** 2

    @property
    def swept_volume(self) -> float:
        """Swept volume / displacement volume in m^3."""
        return self.piston_area * self.stroke

    @property
    def equivalent_reciprocating_mass(self) -> float:
        """
        Dynamically equivalent reciprocating mass at piston pin (kg).
        Uses two-mass dynamically equivalent system for connecting rod:
        m_rec = m_piston + m_rod * (1 - con_rod_cg_ratio)
        """
        m_rod_rec = self.con_rod_mass * (1.0 - self.con_rod_cg_ratio)
        return self.piston_mass + m_rod_rec

    @property
    def equivalent_rotating_mass(self) -> float:
        """
        Dynamically equivalent rotating mass at crank pin (kg).
        m_rot = m_rod * con_rod_cg_ratio
        """
        return self.con_rod_mass * self.con_rod_cg_ratio


class MechanismGeometry:
    """Computes instantaneous joint positions for 2D animation and rendering."""

    def __init__(self, params: SliderCrankParameters):
        self.params = params

    def get_joint_positions(self, theta: float):
        """
        Returns Cartesian coordinates (x, y) of:
        - Crank center O: (0, 0)
        - Crank pin A: (r*cos(theta), r*sin(theta))
        - Piston pin B: (x_piston, offset)
        - Con-rod CG G
        """
        r = self.params.r
        l = self.params.l
        e = self.params.e

        xA = r * np.cos(theta)
        yA = r * np.sin(theta)

        # Solve for xB where (xB - xA)^2 + (e - yA)^2 = l^2
        dy = e - yA
        discriminant = l**2 - dy**2
        if discriminant < 0:
            raise ValueError(f"Geometry error at theta={np.degrees(theta)}: dy ({dy}) exceeds con-rod length {l}")
        xB = xA + np.sqrt(discriminant)
        yB = e

        # Center of gravity of connecting rod
        # cg_ratio measured from crank pin A towards B
        ratio = self.params.con_rod_cg_ratio
        xG = xA + ratio * (xB - xA)
        yG = yA + ratio * (yB - yA)

        return {
            "O": (0.0, 0.0),
            "A": (float(xA), float(yA)),
            "B": (float(xB), float(yB)),
            "G": (float(xG), float(yG)),
        }
