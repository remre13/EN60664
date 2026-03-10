#!/usr/bin/env python3
"""
IEC 60664-1:2020+A1:2025 Clearance and Creepage Distance Calculator

This tool calculates minimum clearance and creepage distances according to
BS EN IEC 60664-1:2020+A1:2025 "Insulation coordination for equipment
within low-voltage supply systems".

It implements the key normative tables:
  - Table F.1: Rated impulse withstand voltage
  - Table F.2: Clearances to withstand transient overvoltages
  - Table F.3: Single-phase AC or DC voltage rationalization
  - Table F.4: Three-phase AC voltage rationalization
  - Table F.5: Creepage distances to avoid failure due to tracking
  - Table F.8: Clearances for steady-state/temporary/recurring peak voltages
  - Table F.10: Altitude correction factors
"""

import bisect
import math
import sys
from dataclasses import dataclass
from enum import Enum
from typing import Optional


# =============================================================================
# Enumerations
# =============================================================================

class OvervoltageCategory(Enum):
    OVC_I = 1
    OVC_II = 2
    OVC_III = 3
    OVC_IV = 4


class PollutionDegree(Enum):
    PD1 = 1
    PD2 = 2
    PD3 = 3
    PD4 = 4


class FieldCondition(Enum):
    INHOMOGENEOUS = "A"   # Case A
    HOMOGENEOUS = "B"     # Case B


class InsulationType(Enum):
    FUNCTIONAL = "functional"
    BASIC = "basic"
    SUPPLEMENTARY = "supplementary"
    REINFORCED = "reinforced"


class MaterialGroup(Enum):
    I = 1        # 600 <= CTI
    II = 2       # 400 <= CTI < 600
    IIIa = 3     # 175 <= CTI < 400
    IIIb = 4     # 100 <= CTI < 175


class SystemType(Enum):
    SINGLE_PHASE = "single_phase"          # Single-phase or DC
    THREE_PHASE_4WIRE = "three_phase_4w"   # Three-phase four-wire, neutral-earthed
    THREE_PHASE_3WIRE = "three_phase_3w"   # Three-phase three-wire, unearthed/corner-earthed


class InsulationPosition(Enum):
    LINE_TO_LINE = "line_to_line"
    LINE_TO_EARTH = "line_to_earth"


# =============================================================================
# Table F.1 – Rated impulse withstand voltage
# =============================================================================

# (voltage_line_to_neutral_max, {OVC: rated_impulse_voltage})
TABLE_F1 = [
    (50,   {1: 330,  2: 500,  3: 800,   4: 1500}),
    (100,  {1: 500,  2: 800,  3: 1500,  4: 2500}),
    (150,  {1: 800,  2: 1500, 3: 2500,  4: 4000}),
    (300,  {1: 1500, 2: 2500, 3: 4000,  4: 6000}),
    (600,  {1: 2500, 2: 4000, 3: 6000,  4: 8000}),
    (1000, {1: 4000, 2: 6000, 3: 8000,  4: 12000}),
    (1250, {1: 4000, 2: 6000, 3: 8000,  4: 12000}),
    (1500, {1: 6000, 2: 8000, 3: 10000, 4: 15000}),
]


def get_rated_impulse_voltage(voltage_ln: float, ovc: OvervoltageCategory) -> int:
    """
    Look up rated impulse withstand voltage from Table F.1.

    Args:
        voltage_ln: Voltage line-to-neutral (or DC voltage) in V
        ovc: Overvoltage category (I to IV)

    Returns:
        Rated impulse withstand voltage in V
    """
    for v_max, ovc_map in TABLE_F1:
        if voltage_ln <= v_max:
            return ovc_map[ovc.value]
    raise ValueError(
        f"Voltage {voltage_ln} V exceeds Table F.1 range (max 1500 V DC)"
    )


# =============================================================================
# Table F.2 – Clearances to withstand transient overvoltages
# =============================================================================

# (impulse_voltage_kV, {(field_condition, pollution_degree): clearance_mm})
# Pollution degree 4 uses PD3 values with 1.6 mm minimum
TABLE_F2_DATA = [
    (0.33,  {"A1": 0.01,  "B1": 0.01}),
    (0.40,  {"A1": 0.02,  "B1": 0.02}),
    (0.50,  {"A1": 0.04,  "A2": 0.2,   "B1": 0.04, "B2": 0.2}),
    (0.60,  {"A1": 0.06,  "A3": 0.8,   "B1": 0.06}),
    (0.80,  {"A1": 0.10,  "B1": 0.10,  "B3": 0.8}),
    (1.0,   {"A1": 0.15,  "B1": 0.15}),
    (1.2,   {"A1": 0.25,  "A2": 0.25,  "B1": 0.2}),
    (1.5,   {"A1": 0.5,   "A2": 0.5,   "B1": 0.3,  "B2": 0.3}),
    (2.0,   {"A1": 1.0,   "A2": 1.0,   "A3": 1.0,  "B1": 0.45, "B2": 0.45}),
    (2.5,   {"A1": 1.5,   "A2": 1.5,   "A3": 1.5,  "B1": 0.60, "B2": 0.60}),
    (3.0,   {"A1": 2.0,   "A2": 2.0,   "A3": 2.0,  "B1": 0.80, "B2": 0.80}),
    (4.0,   {"A1": 3.0,   "A2": 3.0,   "A3": 3.0,  "B1": 1.2,  "B2": 1.2,  "B3": 1.2}),
    (5.0,   {"A1": 4.0,   "A2": 4.0,   "A3": 4.0,  "B1": 1.5,  "B2": 1.5,  "B3": 1.5}),
    (6.0,   {"A1": 5.5,   "A2": 5.5,   "A3": 5.5,  "B1": 2.0,  "B2": 2.0,  "B3": 2.0}),
    (8.0,   {"A1": 8.0,   "A2": 8.0,   "A3": 8.0,  "B1": 3.0,  "B2": 3.0,  "B3": 3.0}),
    (10.0,  {"A1": 11.0,  "A2": 11.0,  "A3": 11.0, "B1": 3.5,  "B2": 3.5,  "B3": 3.5}),
    (12.0,  {"A1": 14.0,  "A2": 14.0,  "A3": 14.0, "B1": 4.5,  "B2": 4.5,  "B3": 4.5}),
    (15.0,  {"A1": 18.0,  "A2": 18.0,  "A3": 18.0, "B1": 5.5,  "B2": 5.5,  "B3": 5.5}),
    (20.0,  {"A1": 25.0,  "A2": 25.0,  "A3": 25.0, "B1": 8.0,  "B2": 8.0,  "B3": 8.0}),
    (25.0,  {"A1": 33.0,  "A2": 33.0,  "A3": 33.0, "B1": 10.0, "B2": 10.0, "B3": 10.0}),
    (30.0,  {"A1": 40.0,  "A2": 40.0,  "A3": 40.0, "B1": 12.5, "B2": 12.5, "B3": 12.5}),
    (40.0,  {"A1": 60.0,  "A2": 60.0,  "A3": 60.0, "B1": 17.0, "B2": 17.0, "B3": 17.0}),
    (50.0,  {"A1": 75.0,  "A2": 75.0,  "A3": 75.0, "B1": 22.0, "B2": 22.0, "B3": 22.0}),
    (60.0,  {"A1": 90.0,  "A2": 90.0,  "A3": 90.0, "B1": 27.0, "B2": 27.0, "B3": 27.0}),
    (80.0,  {"A1": 130.0, "A2": 130.0, "A3": 130.0,"B1": 35.0, "B2": 35.0, "B3": 35.0}),
    (100.0, {"A1": 170.0, "A2": 170.0, "A3": 170.0,"B1": 45.0, "B2": 45.0, "B3": 45.0}),
]


def _interpolate(x: float, x0: float, y0: float, x1: float, y1: float) -> float:
    """Linear interpolation between two points."""
    if x1 == x0:
        return y0
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def get_clearance_transient(
    impulse_voltage_kv: float,
    field_condition: FieldCondition,
    pollution_degree: PollutionDegree,
) -> float:
    """
    Get minimum clearance from Table F.2 for transient overvoltages.

    Args:
        impulse_voltage_kv: Required impulse withstand voltage in kV
        field_condition: INHOMOGENEOUS (case A) or HOMOGENEOUS (case B)
        pollution_degree: Pollution degree 1-4

    Returns:
        Minimum clearance in mm
    """
    pd = pollution_degree.value
    if pd == 4:
        pd = 3  # PD4 uses PD3 values with 1.6 mm minimum

    key = f"{field_condition.value}{pd}"

    voltages = [row[0] for row in TABLE_F2_DATA]
    values = []
    for row in TABLE_F2_DATA:
        # Fall back to lower pollution degree if key not present
        v = row[1].get(key)
        if v is None:
            # Try falling back
            for fallback_pd in range(pd - 1, 0, -1):
                fallback_key = f"{field_condition.value}{fallback_pd}"
                v = row[1].get(fallback_key)
                if v is not None:
                    break
        values.append(v)

    # Find bounding entries for interpolation
    if impulse_voltage_kv <= voltages[0]:
        result = values[0] if values[0] is not None else 0.01
    elif impulse_voltage_kv >= voltages[-1]:
        result = values[-1] if values[-1] is not None else 170.0
    else:
        # Find interval
        idx = bisect.bisect_right(voltages, impulse_voltage_kv) - 1
        # Find valid lower bound
        while idx >= 0 and values[idx] is None:
            idx -= 1
        idx_upper = idx + 1
        while idx_upper < len(voltages) and values[idx_upper] is None:
            idx_upper += 1

        if idx < 0 or idx_upper >= len(voltages):
            raise ValueError(f"Cannot interpolate for {impulse_voltage_kv} kV")

        result = _interpolate(
            impulse_voltage_kv,
            voltages[idx], values[idx],
            voltages[idx_upper], values[idx_upper],
        )

    # PD4 minimum is 1.6 mm
    if pollution_degree == PollutionDegree.PD4:
        result = max(result, 1.6)

    return round(result, 3)


# =============================================================================
# Table F.3 – Single-phase / DC voltage rationalization
# =============================================================================

# (nominal_voltage, rationalized_line_to_line, rationalized_line_to_earth_3wire)
TABLE_F3 = [
    (12.5,  12.5,  None),
    (24,    25,    None),
    (25,    25,    None),
    (30,    32,    None),
    (42,    50,    None),
    (48,    50,    None),
    (50,    50,    None),
    (60,    63,    32),
    (100,   100,   None),
    (110,   125,   None),
    (120,   125,   None),
    (150,   160,   None),
    (200,   200,   100),
    (220,   250,   None),
    (240,   250,   125),
    (300,   320,   None),
    (440,   500,   250),
    (600,   630,   None),
    (960,   1000,  500),
    (1000,  1000,  None),
    (1500,  1500,  None),
]


def rationalize_voltage_single_phase(
    nominal_voltage: float,
    insulation_position: InsulationPosition = InsulationPosition.LINE_TO_LINE,
) -> float:
    """
    Rationalize voltage for Table F.5 lookup using Table F.3.

    Args:
        nominal_voltage: Nominal voltage of the mains supply in V
        insulation_position: LINE_TO_LINE or LINE_TO_EARTH

    Returns:
        Rationalized voltage in V for Table F.5 lookup
    """
    # Find the matching or next higher entry
    for nom, v_ll, v_le in TABLE_F3:
        if nominal_voltage <= nom:
            if insulation_position == InsulationPosition.LINE_TO_EARTH and v_le is not None:
                return v_le
            return v_ll

    return TABLE_F3[-1][1]


# =============================================================================
# Table F.4 – Three-phase voltage rationalization
# =============================================================================

# (nominal_voltage, rationalized_ll, rationalized_le_4wire, rationalized_le_3wire)
TABLE_F4 = [
    (60,   63,   32,   63),
    (110,  125,  80,   125),
    (120,  125,  80,   125),
    (127,  125,  80,   125),
    (150,  160,  None, 160),
    (200,  200,  None, 200),
    (208,  200,  125,  200),
    (220,  250,  160,  250),
    (230,  250,  160,  250),
    (240,  250,  160,  250),
    (300,  320,  None, 320),
    (380,  400,  250,  400),
    (400,  400,  250,  400),
    (415,  400,  250,  400),
    (440,  500,  250,  500),
    (480,  500,  320,  500),
    (500,  500,  320,  500),
    (575,  630,  400,  630),
    (600,  630,  None, 630),
    (660,  630,  400,  630),
    (690,  630,  400,  630),
    (720,  800,  500,  800),
    (830,  800,  500,  800),
    (960,  1000, 630,  1000),
    (1000, 1000, None, 1000),
]


def rationalize_voltage_three_phase(
    nominal_voltage: float,
    system_type: SystemType,
    insulation_position: InsulationPosition = InsulationPosition.LINE_TO_LINE,
) -> float:
    """
    Rationalize voltage for Table F.5 lookup using Table F.4.

    Args:
        nominal_voltage: Nominal voltage of the mains supply in V
        system_type: THREE_PHASE_4WIRE or THREE_PHASE_3WIRE
        insulation_position: LINE_TO_LINE or LINE_TO_EARTH

    Returns:
        Rationalized voltage in V for Table F.5 lookup
    """
    for nom, v_ll, v_le_4w, v_le_3w in TABLE_F4:
        if nominal_voltage <= nom:
            if insulation_position == InsulationPosition.LINE_TO_LINE:
                return v_ll
            if system_type == SystemType.THREE_PHASE_4WIRE:
                return v_le_4w if v_le_4w is not None else v_ll
            return v_le_3w
    return TABLE_F4[-1][1]


# =============================================================================
# Table F.5 – Creepage distances
# =============================================================================

# Columns: voltage_rms, PWB_PD1, PWB_PD2, PD1_all, PD2_I, PD2_II, PD2_III,
#           PD3_I, PD3_II, PD3_III
TABLE_F5 = [
    (10,    0.025, 0.040, 0.080, 0.400, 0.400, 0.400, 1.000, 1.000, 1.000),
    (12.5,  0.025, 0.040, 0.090, 0.420, 0.420, 0.420, 1.050, 1.050, 1.050),
    (16,    0.025, 0.040, 0.100, 0.450, 0.450, 0.450, 1.100, 1.100, 1.100),
    (20,    0.025, 0.040, 0.110, 0.480, 0.480, 0.480, 1.200, 1.200, 1.200),
    (25,    0.025, 0.040, 0.125, 0.500, 0.500, 0.500, 1.250, 1.250, 1.250),
    (32,    0.025, 0.040, 0.14,  0.53,  0.53,  0.53,  1.30,  1.30,  1.30),
    (40,    0.025, 0.040, 0.16,  0.56,  0.80,  1.10,  1.40,  1.60,  1.80),
    (50,    0.025, 0.040, 0.18,  0.60,  0.85,  1.20,  1.50,  1.70,  1.90),
    (63,    0.040, 0.063, 0.20,  0.63,  0.90,  1.25,  1.60,  1.80,  2.00),
    (80,    0.063, 0.100, 0.22,  0.67,  0.95,  1.30,  1.70,  1.90,  2.10),
    (100,   0.100, 0.160, 0.25,  0.71,  1.00,  1.40,  1.80,  2.00,  2.20),
    (125,   0.160, 0.250, 0.28,  0.75,  1.05,  1.50,  1.90,  2.10,  2.40),
    (160,   0.250, 0.400, 0.32,  0.80,  1.10,  1.60,  2.00,  2.20,  2.50),
    (200,   0.400, 0.630, 0.42,  1.00,  1.40,  2.00,  2.50,  2.80,  3.20),
    (250,   0.560, 1.000, 0.56,  1.25,  1.80,  2.50,  3.20,  3.60,  4.00),
    (320,   0.75,  1.60,  0.75,  1.60,  2.20,  3.20,  4.00,  4.50,  5.00),
    (400,   1.0,   2.0,   1.0,   2.0,   2.8,   4.0,   5.0,   5.6,   6.3),
    (500,   1.3,   2.5,   1.3,   2.5,   3.6,   5.0,   6.3,   7.1,   8.0),
    (630,   1.8,   3.2,   1.8,   3.2,   4.5,   6.3,   8.0,   9.0,   10.0),
    (800,   2.4,   4.0,   2.4,   4.0,   5.6,   8.0,   10.0,  11.0,  12.5),
    (1000,  3.2,   5.0,   3.2,   5.0,   7.1,   10.0,  12.5,  14.0,  16.0),
    (1250,  None,  None,  4.2,   6.3,   9.0,   12.5,  16.0,  18.0,  20.0),
    (1600,  None,  None,  5.6,   8.0,   11.0,  16.0,  20.0,  22.0,  25.0),
    (2000,  None,  None,  7.5,   10.0,  14.0,  20.0,  25.0,  28.0,  32.0),
    (2500,  None,  None,  10.0,  12.5,  18.0,  25.0,  32.0,  36.0,  40.0),
    (3200,  None,  None,  12.5,  16.0,  22.0,  32.0,  40.0,  45.0,  50.0),
]

# Column index mapping for Table F.5
_F5_COL = {
    # (pollution_degree, material_group_or_special): column_index
    "PWB_PD1": 1,
    "PWB_PD2": 2,
    "PD1": 3,
    "PD2_I": 4,
    "PD2_II": 5,
    "PD2_III": 6,
    "PD3_I": 7,
    "PD3_II": 8,
    "PD3_III": 9,
}


def _get_f5_column_index(
    pollution_degree: PollutionDegree,
    material_group: MaterialGroup,
    is_printed_wiring: bool = False,
) -> int:
    """Determine the column index in TABLE_F5 for given parameters."""
    pd = pollution_degree.value

    if is_printed_wiring:
        if pd <= 1:
            return _F5_COL["PWB_PD1"]
        return _F5_COL["PWB_PD2"]

    if pd == 1:
        return _F5_COL["PD1"]

    mg = material_group.value
    if pd == 2:
        if mg == 1:
            return _F5_COL["PD2_I"]
        elif mg == 2:
            return _F5_COL["PD2_II"]
        else:
            return _F5_COL["PD2_III"]
    else:  # PD3 or PD4
        if mg == 1:
            return _F5_COL["PD3_I"]
        elif mg == 2:
            return _F5_COL["PD3_II"]
        else:
            return _F5_COL["PD3_III"]


def get_creepage_distance(
    voltage_rms: float,
    pollution_degree: PollutionDegree,
    material_group: MaterialGroup,
    is_printed_wiring: bool = False,
) -> float:
    """
    Get minimum creepage distance from Table F.5.

    Args:
        voltage_rms: RMS voltage in V
        pollution_degree: Pollution degree 1-4
        material_group: Material group (I, II, IIIa, IIIb)
        is_printed_wiring: True for printed wiring board material

    Returns:
        Minimum creepage distance in mm
    """
    col_idx = _get_f5_column_index(pollution_degree, material_group, is_printed_wiring)

    voltages = [row[0] for row in TABLE_F5]
    values = [row[col_idx] for row in TABLE_F5]

    if voltage_rms <= voltages[0]:
        return values[0] if values[0] is not None else 0.025

    if voltage_rms >= voltages[-1]:
        v = values[-1]
        if v is not None:
            return v
        raise ValueError(f"Voltage {voltage_rms} V exceeds Table F.5 range for this configuration")

    # Linear interpolation
    idx = bisect.bisect_right(voltages, voltage_rms) - 1
    while idx >= 0 and values[idx] is None:
        idx -= 1
    idx_upper = idx + 1
    while idx_upper < len(voltages) and values[idx_upper] is None:
        idx_upper += 1

    if idx < 0 or idx_upper >= len(voltages) or values[idx] is None or values[idx_upper] is None:
        raise ValueError(f"Cannot interpolate for {voltage_rms} V in selected column")

    result = _interpolate(
        voltage_rms,
        voltages[idx], values[idx],
        voltages[idx_upper], values[idx_upper],
    )
    return round(result, 3)


# =============================================================================
# Table F.8 – Clearances for steady-state peak / temporary / recurring voltages
# =============================================================================

# (voltage_peak_kV, clearance_case_A_mm, clearance_case_B_mm)
TABLE_F8 = [
    (0.04,  0.001,  0.001),
    (0.06,  0.002,  0.002),
    (0.1,   0.003,  0.003),
    (0.12,  0.004,  0.004),
    (0.15,  0.005,  0.005),
    (0.20,  0.006,  0.006),
    (0.25,  0.008,  0.008),
    (0.33,  0.01,   0.01),
    (0.4,   0.02,   0.02),
    (0.5,   0.04,   0.04),
    (0.6,   0.06,   0.06),
    (0.8,   0.13,   0.1),
    (1.0,   0.26,   0.15),
    (1.2,   0.42,   0.2),
    (1.5,   0.76,   0.3),
    (2.0,   1.27,   0.45),
    (2.5,   1.8,    0.6),
    (3.0,   2.4,    0.8),
    (4.0,   3.8,    1.2),
    (5.0,   5.7,    1.5),
    (6.0,   7.9,    2.0),
    (8.0,   11.0,   3.0),
    (10.0,  15.2,   3.5),
    (12.0,  19.0,   4.5),
    (15.0,  25.0,   5.5),
    (20.0,  34.0,   8.0),
    (25.0,  44.0,   10.0),
    (30.0,  55.0,   12.5),
    (40.0,  77.0,   17.0),
    (50.0,  100.0,  22.0),
    (60.0,  None,   27.0),
    (80.0,  None,   35.0),
    (100.0, None,   45.0),
]


def get_clearance_steady_state(
    voltage_peak_kv: float,
    field_condition: FieldCondition,
) -> float:
    """
    Get minimum clearance from Table F.8 for steady-state peak voltages,
    temporary overvoltages, or recurring peak voltages.

    Args:
        voltage_peak_kv: Peak voltage in kV
        field_condition: INHOMOGENEOUS (case A) or HOMOGENEOUS (case B)

    Returns:
        Minimum clearance in mm
    """
    col = 1 if field_condition == FieldCondition.INHOMOGENEOUS else 2

    voltages = [row[0] for row in TABLE_F8]
    values = [row[col] for row in TABLE_F8]

    if voltage_peak_kv <= voltages[0]:
        return values[0]

    if voltage_peak_kv >= voltages[-1]:
        if values[-1] is not None:
            return values[-1]
        raise ValueError(
            f"Voltage {voltage_peak_kv} kV exceeds Table F.8 range for case {field_condition.value}"
        )

    idx = bisect.bisect_right(voltages, voltage_peak_kv) - 1
    while idx >= 0 and values[idx] is None:
        idx -= 1
    idx_upper = idx + 1
    while idx_upper < len(voltages) and values[idx_upper] is None:
        idx_upper += 1

    if idx < 0 or idx_upper >= len(voltages) or values[idx] is None or values[idx_upper] is None:
        raise ValueError(f"Cannot interpolate for {voltage_peak_kv} kV")

    result = _interpolate(
        voltage_peak_kv,
        voltages[idx], values[idx],
        voltages[idx_upper], values[idx_upper],
    )
    return round(result, 3)


# =============================================================================
# Table F.10 – Altitude correction factors
# =============================================================================

TABLE_F10 = [
    (0,    0.784),
    (200,  0.803),
    (500,  0.833),
    (1000, 0.884),
    (2000, 1.000),
]


def get_altitude_correction_factor(altitude_m: float) -> float:
    """
    Get altitude correction factor from Table F.10.

    The clearance values in the tables are specified for 2000 m altitude.
    For other altitudes, multiply clearance by this factor.

    For altitudes above 2000 m, extrapolation is used based on atmospheric
    pressure ratio per IEC 60664-1 clause 4.7.2.

    Args:
        altitude_m: Altitude in meters above sea level

    Returns:
        Correction factor (multiply clearance by this)
    """
    altitudes = [row[0] for row in TABLE_F10]
    factors = [row[1] for row in TABLE_F10]

    if altitude_m <= altitudes[0]:
        return factors[0]
    if altitude_m >= altitudes[-1]:
        # For altitudes above 2000m, use barometric formula
        # p/p0 = exp(-altitude/8500), referenced to 2000m
        if altitude_m > 2000:
            ratio = math.exp(-(altitude_m - 2000) / 8500)
            return 1.0 / ratio  # Factor increases with altitude
        return factors[-1]

    idx = bisect.bisect_right(altitudes, altitude_m) - 1
    return round(
        _interpolate(
            altitude_m,
            altitudes[idx], factors[idx],
            altitudes[idx + 1], factors[idx + 1],
        ),
        4,
    )


# =============================================================================
# High-level calculation functions
# =============================================================================

@dataclass
class ClearanceResult:
    """Result of a clearance calculation."""
    clearance_transient_mm: float
    clearance_steady_state_mm: Optional[float]
    clearance_required_mm: float
    impulse_withstand_voltage_v: int
    altitude_correction_factor: float
    field_condition: str
    pollution_degree: int
    notes: list


@dataclass
class CreepageResult:
    """Result of a creepage calculation."""
    creepage_mm: float
    rationalized_voltage_v: float
    pollution_degree: int
    material_group: str
    notes: list


def calculate_clearance(
    nominal_voltage: float,
    overvoltage_category: OvervoltageCategory,
    pollution_degree: PollutionDegree,
    field_condition: FieldCondition,
    insulation_type: InsulationType = InsulationType.BASIC,
    altitude_m: float = 2000,
    steady_state_peak_kv: Optional[float] = None,
    system_type: SystemType = SystemType.SINGLE_PHASE,
) -> ClearanceResult:
    """
    Calculate minimum clearance distance per IEC 60664-1.

    Args:
        nominal_voltage: Nominal system voltage in V (line-to-neutral for mains)
        overvoltage_category: OVC I to IV
        pollution_degree: PD1 to PD4
        field_condition: INHOMOGENEOUS or HOMOGENEOUS
        insulation_type: Type of insulation
        altitude_m: Installation altitude in meters
        steady_state_peak_kv: Steady-state peak voltage in kV (if applicable)
        system_type: System type for voltage derivation

    Returns:
        ClearanceResult with all details
    """
    notes = []

    # Step 1: Get rated impulse withstand voltage from Table F.1
    impulse_v = get_rated_impulse_voltage(nominal_voltage, overvoltage_category)
    impulse_kv = impulse_v / 1000.0

    # Step 2: For reinforced insulation, use 160% of basic insulation voltage
    if insulation_type == InsulationType.REINFORCED:
        impulse_kv *= 1.6
        notes.append(
            f"Reinforced insulation: impulse voltage increased to 160% = {impulse_kv:.1f} kV"
        )

    # Step 3: Get clearance from Table F.2
    clearance_transient = get_clearance_transient(
        impulse_kv, field_condition, pollution_degree
    )

    # Step 4: Get clearance from Table F.8 if steady-state voltage provided
    clearance_ss = None
    if steady_state_peak_kv is not None:
        if insulation_type == InsulationType.REINFORCED:
            ss_kv = steady_state_peak_kv * 1.6
            notes.append(
                f"Reinforced insulation: steady-state voltage increased to 160% = {ss_kv:.1f} kV"
            )
        else:
            ss_kv = steady_state_peak_kv
        clearance_ss = get_clearance_steady_state(ss_kv, field_condition)

    # Step 5: Apply altitude correction
    alt_factor = get_altitude_correction_factor(altitude_m)
    clearance_transient_corrected = round(clearance_transient * alt_factor, 3)
    if alt_factor != 1.0:
        notes.append(
            f"Altitude correction factor (Table F.10): {alt_factor} "
            f"(at {altitude_m} m)"
        )

    clearance_ss_corrected = None
    if clearance_ss is not None:
        clearance_ss_corrected = round(clearance_ss * alt_factor, 3)

    # Step 6: Required clearance is the greater of transient and steady-state
    required = clearance_transient_corrected
    if clearance_ss_corrected is not None:
        required = max(required, clearance_ss_corrected)
        notes.append(
            f"Required clearance = max(transient={clearance_transient_corrected} mm, "
            f"steady-state={clearance_ss_corrected} mm)"
        )

    return ClearanceResult(
        clearance_transient_mm=clearance_transient_corrected,
        clearance_steady_state_mm=clearance_ss_corrected,
        clearance_required_mm=required,
        impulse_withstand_voltage_v=impulse_v,
        altitude_correction_factor=alt_factor,
        field_condition=field_condition.value,
        pollution_degree=pollution_degree.value,
        notes=notes,
    )


def calculate_creepage(
    nominal_voltage: float,
    pollution_degree: PollutionDegree,
    material_group: MaterialGroup,
    insulation_type: InsulationType = InsulationType.BASIC,
    system_type: SystemType = SystemType.SINGLE_PHASE,
    insulation_position: InsulationPosition = InsulationPosition.LINE_TO_LINE,
    is_printed_wiring: bool = False,
    overvoltage_category: Optional[OvervoltageCategory] = None,
    field_condition: Optional[FieldCondition] = None,
) -> CreepageResult:
    """
    Calculate minimum creepage distance per IEC 60664-1.

    Args:
        nominal_voltage: Nominal system voltage in V
        pollution_degree: PD1 to PD4
        material_group: Material group I, II, IIIa, or IIIb
        insulation_type: Type of insulation
        system_type: System type for voltage rationalization
        insulation_position: LINE_TO_LINE or LINE_TO_EARTH
        is_printed_wiring: True for printed wiring board
        overvoltage_category: Required for checking clearance minimum
        field_condition: Required for checking clearance minimum

    Returns:
        CreepageResult with all details
    """
    notes = []

    # Step 1: Rationalize voltage
    if system_type == SystemType.SINGLE_PHASE:
        rat_voltage = rationalize_voltage_single_phase(nominal_voltage, insulation_position)
    else:
        rat_voltage = rationalize_voltage_three_phase(
            nominal_voltage, system_type, insulation_position
        )

    notes.append(f"Rationalized voltage: {rat_voltage} V")

    # Step 2: For functional insulation, use actual working voltage
    if insulation_type == InsulationType.FUNCTIONAL:
        rat_voltage = nominal_voltage
        notes.append("Functional insulation: using actual working voltage")

    # Step 3: Get creepage from Table F.5
    creepage = get_creepage_distance(
        rat_voltage, pollution_degree, material_group, is_printed_wiring
    )

    # Step 4: For reinforced insulation, creepage shall not be less than that
    # for basic insulation at the next higher voltage step
    if insulation_type == InsulationType.REINFORCED:
        # Per 5.3.5: reinforced = basic value * 2 (in terms of insulation level)
        # The standard says to use the values for basic insulation but cannot be
        # less than the clearance required
        notes.append(
            "Reinforced insulation: creepage distance shall not be less than "
            "the clearance required for reinforced insulation"
        )

    # Step 5: Creepage shall not be less than the associated clearance (case A)
    if overvoltage_category is not None and field_condition is not None:
        # Get the clearance for case A to compare
        impulse_v = get_rated_impulse_voltage(
            nominal_voltage if system_type == SystemType.SINGLE_PHASE else nominal_voltage,
            overvoltage_category,
        )
        impulse_kv = impulse_v / 1000.0
        if insulation_type == InsulationType.REINFORCED:
            impulse_kv *= 1.6

        min_clearance = get_clearance_transient(
            impulse_kv, FieldCondition.INHOMOGENEOUS, pollution_degree
        )

        if creepage < min_clearance:
            notes.append(
                f"Creepage increased from {creepage} mm to {min_clearance} mm "
                f"(cannot be less than clearance for case A per 5.3.2.1)"
            )
            creepage = min_clearance

    mg_names = {
        MaterialGroup.I: "I (CTI >= 600)",
        MaterialGroup.II: "II (400 <= CTI < 600)",
        MaterialGroup.IIIa: "IIIa (175 <= CTI < 400)",
        MaterialGroup.IIIb: "IIIb (100 <= CTI < 175)",
    }

    return CreepageResult(
        creepage_mm=creepage,
        rationalized_voltage_v=rat_voltage,
        pollution_degree=pollution_degree.value,
        material_group=mg_names[material_group],
        notes=notes,
    )


# =============================================================================
# Interactive CLI
# =============================================================================

def _select_enum(prompt_text: str, enum_class, descriptions=None):
    """Helper for CLI enum selection."""
    members = list(enum_class)
    print(f"\n{prompt_text}")
    for i, member in enumerate(members, 1):
        desc = f" - {descriptions[member]}" if descriptions and member in descriptions else ""
        print(f"  {i}. {member.name}{desc}")
    while True:
        try:
            choice = int(input("  Select [number]: "))
            if 1 <= choice <= len(members):
                return members[choice - 1]
        except (ValueError, EOFError):
            pass
        print("  Invalid selection, try again.")


def _get_float(prompt_text: str, default=None) -> float:
    """Helper for CLI float input."""
    while True:
        try:
            default_str = f" [{default}]" if default is not None else ""
            val = input(f"{prompt_text}{default_str}: ").strip()
            if not val and default is not None:
                return float(default)
            return float(val)
        except (ValueError, EOFError):
            print("  Invalid number, try again.")


def print_clearance_result(result: ClearanceResult):
    """Pretty-print clearance result."""
    print("\n" + "=" * 60)
    print("CLEARANCE CALCULATION RESULT (IEC 60664-1)")
    print("=" * 60)
    print(f"  Rated impulse withstand voltage:  {result.impulse_withstand_voltage_v} V")
    print(f"  Field condition:                  Case {result.field_condition}")
    print(f"  Pollution degree:                 {result.pollution_degree}")
    print(f"  Altitude correction factor:       {result.altitude_correction_factor}")
    print(f"  Clearance (transient):            {result.clearance_transient_mm} mm")
    if result.clearance_steady_state_mm is not None:
        print(f"  Clearance (steady-state):         {result.clearance_steady_state_mm} mm")
    print("-" * 60)
    print(f"  >>> REQUIRED CLEARANCE:           {result.clearance_required_mm} mm <<<")
    print("-" * 60)
    if result.notes:
        print("  Notes:")
        for note in result.notes:
            print(f"    - {note}")
    print()


def print_creepage_result(result: CreepageResult):
    """Pretty-print creepage result."""
    print("\n" + "=" * 60)
    print("CREEPAGE DISTANCE RESULT (IEC 60664-1)")
    print("=" * 60)
    print(f"  Rationalized voltage:             {result.rationalized_voltage_v} V")
    print(f"  Pollution degree:                 {result.pollution_degree}")
    print(f"  Material group:                   {result.material_group}")
    print("-" * 60)
    print(f"  >>> REQUIRED CREEPAGE:            {result.creepage_mm} mm <<<")
    print("-" * 60)
    if result.notes:
        print("  Notes:")
        for note in result.notes:
            print(f"    - {note}")
    print()


def interactive_mode():
    """Run the tool in interactive CLI mode."""
    print("=" * 60)
    print("IEC 60664-1:2020+A1:2025")
    print("Clearance & Creepage Distance Calculator")
    print("=" * 60)

    ovc_desc = {
        OvervoltageCategory.OVC_I: "Special protection (well-controlled voltage)",
        OvervoltageCategory.OVC_II: "Appliances, portable equipment",
        OvervoltageCategory.OVC_III: "Fixed installation equipment",
        OvervoltageCategory.OVC_IV: "Equipment at origin of installation",
    }

    pd_desc = {
        PollutionDegree.PD1: "No pollution or dry non-conductive",
        PollutionDegree.PD2: "Non-conductive pollution, temporary condensation",
        PollutionDegree.PD3: "Conductive pollution or dry non-conductive + condensation",
        PollutionDegree.PD4: "Persistent conductivity (rain, snow)",
    }

    field_desc = {
        FieldCondition.INHOMOGENEOUS: "Case A - general, sharp edges/points",
        FieldCondition.HOMOGENEOUS: "Case B - smooth, rounded conductors",
    }

    ins_desc = {
        InsulationType.FUNCTIONAL: "For circuit operation only",
        InsulationType.BASIC: "Basic protection against electric shock",
        InsulationType.SUPPLEMENTARY: "Additional to basic insulation",
        InsulationType.REINFORCED: "Single insulation equivalent to double",
    }

    mg_desc = {
        MaterialGroup.I: "CTI >= 600",
        MaterialGroup.II: "400 <= CTI < 600",
        MaterialGroup.IIIa: "175 <= CTI < 400",
        MaterialGroup.IIIb: "100 <= CTI < 175",
    }

    sys_desc = {
        SystemType.SINGLE_PHASE: "Single-phase or DC",
        SystemType.THREE_PHASE_4WIRE: "3-phase 4-wire, neutral-earthed",
        SystemType.THREE_PHASE_3WIRE: "3-phase 3-wire, unearthed/corner-earthed",
    }

    while True:
        print("\n--- What would you like to calculate? ---")
        print("  1. Clearance distance")
        print("  2. Creepage distance")
        print("  3. Both clearance and creepage")
        print("  4. Quick lookup (Table F.1 - Impulse withstand voltage)")
        print("  5. Quick lookup (Table F.2 - Clearance for transient)")
        print("  6. Quick lookup (Table F.5 - Creepage distance)")
        print("  7. Quick lookup (Table F.8 - Clearance for steady-state)")
        print("  0. Exit")

        try:
            choice = input("\nSelect [0-7]: ").strip()
        except (EOFError, KeyboardInterrupt):
            break

        if choice == "0":
            print("Goodbye!")
            break

        elif choice in ("1", "3"):
            # Clearance calculation
            voltage = _get_float("Nominal voltage line-to-neutral (V)")
            ovc = _select_enum("Overvoltage category:", OvervoltageCategory, ovc_desc)
            pd = _select_enum("Pollution degree:", PollutionDegree, pd_desc)
            fc = _select_enum("Field condition:", FieldCondition, field_desc)
            ins = _select_enum("Insulation type:", InsulationType, ins_desc)
            alt = _get_float("Installation altitude (m)", 2000)

            ss_input = input("Steady-state peak voltage (kV) [skip]: ").strip()
            ss_kv = float(ss_input) if ss_input else None

            sys_type = _select_enum("System type:", SystemType, sys_desc)

            result = calculate_clearance(
                voltage, ovc, pd, fc, ins, alt, ss_kv, sys_type
            )
            print_clearance_result(result)

            if choice == "3":
                # Also calculate creepage
                mg = _select_enum("Material group:", MaterialGroup, mg_desc)
                ip = _select_enum("Insulation position:", InsulationPosition)
                pwb = input("Printed wiring board? [y/N]: ").strip().lower() == "y"

                creepage_result = calculate_creepage(
                    voltage, pd, mg, ins, sys_type, ip, pwb, ovc, fc
                )
                print_creepage_result(creepage_result)

        elif choice == "2":
            # Creepage calculation
            voltage = _get_float("Nominal voltage (V)")
            pd = _select_enum("Pollution degree:", PollutionDegree, pd_desc)
            mg = _select_enum("Material group:", MaterialGroup, mg_desc)
            ins = _select_enum("Insulation type:", InsulationType, ins_desc)
            sys_type = _select_enum("System type:", SystemType, sys_desc)
            ip = _select_enum("Insulation position:", InsulationPosition)
            pwb = input("Printed wiring board? [y/N]: ").strip().lower() == "y"

            ovc_input = input("Check clearance minimum? [y/N]: ").strip().lower()
            ovc = None
            fc = None
            if ovc_input == "y":
                ovc = _select_enum("Overvoltage category:", OvervoltageCategory, ovc_desc)
                fc = _select_enum("Field condition:", FieldCondition, field_desc)

            result = calculate_creepage(
                voltage, pd, mg, ins, sys_type, ip, pwb, ovc, fc
            )
            print_creepage_result(result)

        elif choice == "4":
            voltage = _get_float("Voltage line-to-neutral (V)")
            ovc = _select_enum("Overvoltage category:", OvervoltageCategory, ovc_desc)
            v = get_rated_impulse_voltage(voltage, ovc)
            print(f"\n  Rated impulse withstand voltage: {v} V ({v/1000:.1f} kV)")

        elif choice == "5":
            imp_kv = _get_float("Impulse withstand voltage (kV)")
            fc = _select_enum("Field condition:", FieldCondition, field_desc)
            pd = _select_enum("Pollution degree:", PollutionDegree, pd_desc)
            c = get_clearance_transient(imp_kv, fc, pd)
            print(f"\n  Minimum clearance (Table F.2): {c} mm")

        elif choice == "6":
            v_rms = _get_float("Voltage RMS (V)")
            pd = _select_enum("Pollution degree:", PollutionDegree, pd_desc)
            mg = _select_enum("Material group:", MaterialGroup, mg_desc)
            pwb = input("Printed wiring board? [y/N]: ").strip().lower() == "y"
            c = get_creepage_distance(v_rms, pd, mg, pwb)
            print(f"\n  Minimum creepage distance (Table F.5): {c} mm")

        elif choice == "7":
            v_peak = _get_float("Peak voltage (kV)")
            fc = _select_enum("Field condition:", FieldCondition, field_desc)
            c = get_clearance_steady_state(v_peak, fc)
            print(f"\n  Minimum clearance (Table F.8): {c} mm")

        else:
            print("Invalid selection.")


# =============================================================================
# Example usage and self-test
# =============================================================================

def run_examples():
    """Run example calculations to demonstrate the tool."""
    print("=" * 60)
    print("IEC 60664-1 Calculator - Example Calculations")
    print("=" * 60)

    # Example 1: 230V mains, OVC III, PD2, Case A
    print("\n--- Example 1: 230V mains equipment, OVC III, PD2, Inhomogeneous ---")
    clearance = calculate_clearance(
        nominal_voltage=230,
        overvoltage_category=OvervoltageCategory.OVC_III,
        pollution_degree=PollutionDegree.PD2,
        field_condition=FieldCondition.INHOMOGENEOUS,
        insulation_type=InsulationType.BASIC,
        altitude_m=2000,
    )
    print_clearance_result(clearance)

    creepage = calculate_creepage(
        nominal_voltage=230,
        pollution_degree=PollutionDegree.PD2,
        material_group=MaterialGroup.IIIa,
        insulation_type=InsulationType.BASIC,
        system_type=SystemType.SINGLE_PHASE,
        insulation_position=InsulationPosition.LINE_TO_LINE,
        overvoltage_category=OvervoltageCategory.OVC_III,
        field_condition=FieldCondition.INHOMOGENEOUS,
    )
    print_creepage_result(creepage)

    # Example 2: 400V three-phase, OVC II, PD2, reinforced
    print("\n--- Example 2: 400V 3-phase, OVC II, PD2, Reinforced insulation ---")
    clearance2 = calculate_clearance(
        nominal_voltage=230,  # 400V 3-phase = 230V line-to-neutral
        overvoltage_category=OvervoltageCategory.OVC_II,
        pollution_degree=PollutionDegree.PD2,
        field_condition=FieldCondition.INHOMOGENEOUS,
        insulation_type=InsulationType.REINFORCED,
        altitude_m=2000,
    )
    print_clearance_result(clearance2)

    # Example 3: Sea level installation
    print("\n--- Example 3: 230V at sea level (altitude correction) ---")
    clearance3 = calculate_clearance(
        nominal_voltage=230,
        overvoltage_category=OvervoltageCategory.OVC_III,
        pollution_degree=PollutionDegree.PD2,
        field_condition=FieldCondition.INHOMOGENEOUS,
        insulation_type=InsulationType.BASIC,
        altitude_m=0,
    )
    print_clearance_result(clearance3)

    # Example 4: Table lookups
    print("\n--- Quick Lookups ---")
    print(f"Table F.1: 230V, OVC II -> {get_rated_impulse_voltage(230, OvervoltageCategory.OVC_II)} V")
    print(f"Table F.1: 230V, OVC III -> {get_rated_impulse_voltage(230, OvervoltageCategory.OVC_III)} V")
    print(f"Table F.2: 4kV, Case A, PD2 -> {get_clearance_transient(4.0, FieldCondition.INHOMOGENEOUS, PollutionDegree.PD2)} mm")
    print(f"Table F.5: 250V, PD2, Mat.Grp.II -> {get_creepage_distance(250, PollutionDegree.PD2, MaterialGroup.II)} mm")
    print(f"Table F.8: 2kV peak, Case A -> {get_clearance_steady_state(2.0, FieldCondition.INHOMOGENEOUS)} mm")
    print(f"Table F.10: Sea level factor -> {get_altitude_correction_factor(0)}")
    print(f"Table F.10: 2000m factor -> {get_altitude_correction_factor(2000)}")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--examples":
        run_examples()
    elif len(sys.argv) > 1 and sys.argv[1] == "--interactive":
        interactive_mode()
    else:
        print("IEC 60664-1:2020+A1:2025 Clearance & Creepage Calculator")
        print()
        print("Usage:")
        print(f"  python {sys.argv[0]} --interactive    Run interactive calculator")
        print(f"  python {sys.argv[0]} --examples       Run example calculations")
        print()
        print("Or import as a library:")
        print("  from iec60664_tool import calculate_clearance, calculate_creepage")
        print()
        # Default: run examples
        run_examples()
