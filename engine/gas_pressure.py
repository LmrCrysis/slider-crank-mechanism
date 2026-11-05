"""
Gas pressure and indicator diagram models for IC engines and compressors.
"""

from enum import Enum
import numpy as np
from .mechanism import SliderCrankParameters


class EngineCycleType(str, Enum):
    TWO_STROKE_IC = "2-Stroke IC Engine (360°)"
    FOUR_STROKE_IC = "4-Stroke IC Engine (720°)"
    RECIPROCATING_COMPRESSOR = "Reciprocating Compressor"
    CONSTANT_PRESSURE = "Constant Pressure / Pure Inertia"
    POLYTROPIC_EXPANSION = "Polytropic Gas Expansion"


class GasPressureModel:
    """Computes instantaneous cylinder gas pressure (Pa) across the engine cycle."""

    def __init__(
        self,
        params: SliderCrankParameters,
        cycle_type: EngineCycleType = EngineCycleType.POLYTROPIC_EXPANSION,
        peak_pressure_bar: float = 45.0,
        compression_ratio: float = 9.5,
        polytropic_index: float = 1.33,
        intake_pressure_bar: float = 1.013,
    ):
        self.params = params
        self.cycle_type = cycle_type
        self.peak_pressure_pa = peak_pressure_bar * 1e5
        self.compression_ratio = compression_ratio
        self.polytropic_index = polytropic_index
        self.intake_pressure_pa = intake_pressure_bar * 1e5
        self.patm = 1.01325e5

    def get_pressure(self, theta: np.ndarray, cycle_720: bool = False) -> np.ndarray:
        """
        Calculates cylinder pressure in Pascals for crank angles theta (radians).
        Returns net pressure (P_cylinder - P_atm).
        """
        thetas = np.asarray(theta, dtype=float)
        p_cyl = np.zeros_like(thetas)

        # Modulo to cycle length
        cycle_len = 4.0 * np.pi if (cycle_720 and self.cycle_type == EngineCycleType.FOUR_STROKE_IC) else 2.0 * np.pi
        t_mod = np.mod(thetas, cycle_len)

        if self.cycle_type == EngineCycleType.CONSTANT_PRESSURE:
            p_cyl = np.full_like(thetas, self.peak_pressure_pa)

        elif self.cycle_type == EngineCycleType.POLYTROPIC_EXPANSION:
            # Polytropic model based on instantaneous cylinder volume
            # V(theta) = V_c + A * s(theta)
            r = self.params.r
            l = self.params.l
            s = (r + l) - (r * np.cos(t_mod) + np.sqrt(l**2 - (r * np.sin(t_mod))**2))
            # Clearance volume
            v_s = self.params.swept_volume
            v_c = v_s / (self.compression_ratio - 1.0)
            v_inst = v_c + self.params.piston_area * s

            # Polytropic expansion from TDC (high pressure) to BDC
            # and compression back
            # Power stroke (0 to pi)
            power_mask = t_mod <= np.pi
            comp_mask = ~power_mask

            # Expansion: P * V^gamma = P_peak * V_c^gamma
            p_cyl[power_mask] = self.peak_pressure_pa * (v_c / v_inst[power_mask]) ** self.polytropic_index
            # Compression: P = P_intake * (V_max / V)^gamma
            v_max = v_c + v_s
            p_cyl[comp_mask] = self.intake_pressure_pa * (v_max / v_inst[comp_mask]) ** self.polytropic_index

        elif self.cycle_type == EngineCycleType.RECIPROCATING_COMPRESSOR:
            # Compressor: 0 to pi is compression & discharge, pi to 2pi is expansion & suction
            comp_mask = t_mod <= np.pi
            p_cyl[comp_mask] = self.peak_pressure_pa * (0.5 * (1.0 - np.cos(t_mod[comp_mask])))
            p_cyl[~comp_mask] = self.intake_pressure_pa

        elif self.cycle_type == EngineCycleType.FOUR_STROKE_IC:
            # 720 degree 4-stroke cycle:
            # 0-180: Power / Expansion
            # 180-360: Exhaust (approx 1.1 bar)
            # 360-540: Suction (approx 0.95 bar)
            # 540-720: Compression (rising to ignition at 720/0)
            for i, t in enumerate(t_mod):
                deg = np.degrees(t)
                if deg < 180.0:  # Power
                    progress = deg / 180.0
                    p_cyl[i] = self.intake_pressure_pa + (self.peak_pressure_pa - self.intake_pressure_pa) * np.exp(-3.5 * progress)
                elif deg < 360.0:  # Exhaust
                    p_cyl[i] = 1.1e5
                elif deg < 540.0:  # Suction
                    p_cyl[i] = 0.95e5
                else:  # Compression
                    comp_prog = (deg - 540.0) / 180.0
                    p_cyl[i] = self.intake_pressure_pa * (1.0 + (self.compression_ratio**1.35 - 1.0) * (comp_prog**2.5))
        else:
            # 2-Stroke
            power_mask = t_mod <= np.pi
            p_cyl[power_mask] = self.peak_pressure_pa * np.exp(-2.5 * (t_mod[power_mask] / np.pi))
            p_cyl[~power_mask] = self.intake_pressure_pa * (1.0 + 8.0 * ((t_mod[~power_mask] - np.pi) / np.pi)**2)

        # Net pressure over atmospheric
        p_net = p_cyl - self.patm
        return p_net
