#!/usr/bin/env python3
"""
INAV Blackbox Analyzer — Multirotor Tuning Tool v2.0
=====================================================
Analyzes INAV blackbox logs and tells you EXACTLY what to change.

Output: specific "change X from A to B" instructions, not vague advice.
Supports iterative tuning: run → adjust → fly → run again → repeat.

Usage:
    python inav_blackbox_analyzer.py <logfile.bbl|logfile.csv> [options]
    python inav_blackbox_analyzer.py new_log.csv --previous old_log.csv  # compare iterations
"""

import sys
import os
import csv
import json
import re
import argparse
import warnings
from datetime import datetime
from io import BytesIO
import base64

import numpy as np
from scipy import signal
from scipy.fft import rfft, rfftfreq

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

warnings.filterwarnings("ignore", category=RuntimeWarning)

# ─── Constants ────────────────────────────────────────────────────────────────

AXIS_NAMES = ["Roll", "Pitch", "Yaw"]
AXIS_COLORS = ["#FF6B6B", "#4ECDC4", "#FFD93D"]
MOTOR_COLORS = ["#FF6B6B", "#4ECDC4", "#FFD93D", "#A78BFA"]
REPORT_VERSION = "2.11.0"

# ─── Frame and Prop Profiles ─────────────────────────────────────────────────
# Two separate concerns:
#
# FRAME SIZE determines response characteristics:
#   - Moment of inertia → how fast the quad CAN rotate
#   - Arm resonance → where structural vibration lives
#   - PID thresholds → what's "good" overshoot/delay for this airframe
#   - Adjustment factor → how conservative each tuning step should be
#
# PROP CONFIGURATION determines noise characteristics:
#   - Diameter + blade count + RPM → where vibration harmonics land
#   - Larger/more blades at lower RPM → harmonics closer to signal band
#   - Filter ranges must match where noise actually lives
#
# A 5" frame with triblade 5" props has SAME response thresholds but
# DIFFERENT noise frequencies vs a 5" frame with bullnose 6" biblades.

FRAME_PROFILES = {
    # Keyed by frame size (inches). Drives PID response thresholds.
    3: {
        "name": "3-inch", "class": "micro",
        "good_overshoot": 8, "ok_overshoot": 15, "bad_overshoot": 30,
        "good_delay_ms": 6, "ok_delay_ms": 12, "bad_delay_ms": 25,
        "good_noise_db": -45, "ok_noise_db": -35, "bad_noise_db": -25,
        "pid_adjust_factor": 0.35,
        "motor_sat_ok": 3, "motor_sat_warn": 10,
        "motor_imbal_ok": 4, "motor_imbal_warn": 10,
        "notes": "Fast response, high RPM. Noise well separated from signal. Easy to tune.",
    },
    4: {
        "name": "4-inch", "class": "micro",
        "good_overshoot": 8, "ok_overshoot": 15, "bad_overshoot": 30,
        "good_delay_ms": 7, "ok_delay_ms": 13, "bad_delay_ms": 28,
        "good_noise_db": -45, "ok_noise_db": -35, "bad_noise_db": -25,
        "pid_adjust_factor": 0.32,
        "motor_sat_ok": 3, "motor_sat_warn": 10,
        "motor_imbal_ok": 3.5, "motor_imbal_warn": 9,
        "notes": "Similar to 5-inch but slightly faster motor response.",
    },
    5: {
        "name": "5-inch", "class": "standard",
        "good_overshoot": 8, "ok_overshoot": 15, "bad_overshoot": 30,
        "good_delay_ms": 8, "ok_delay_ms": 15, "bad_delay_ms": 30,
        "good_noise_db": -45, "ok_noise_db": -35, "bad_noise_db": -25,
        "pid_adjust_factor": 0.30,
        "motor_sat_ok": 2, "motor_sat_warn": 8,
        "motor_imbal_ok": 3, "motor_imbal_warn": 8,
        "notes": "The reference class. Most defaults and community tunes are calibrated for 5-inch.",
    },
    6: {
        "name": "6-inch", "class": "standard",
        "good_overshoot": 10, "ok_overshoot": 18, "bad_overshoot": 35,
        "good_delay_ms": 10, "ok_delay_ms": 18, "bad_delay_ms": 35,
        "good_noise_db": -42, "ok_noise_db": -32, "bad_noise_db": -22,
        "pid_adjust_factor": 0.28,
        "motor_sat_ok": 3, "motor_sat_warn": 10,
        "motor_imbal_ok": 3.5, "motor_imbal_warn": 9,
        "notes": "Transition zone. Still manageable but noise starts creeping into signal band.",
    },
    7: {
        "name": "7-inch", "class": "mid",
        "good_overshoot": 12, "ok_overshoot": 20, "bad_overshoot": 38,
        "good_delay_ms": 12, "ok_delay_ms": 22, "bad_delay_ms": 40,
        "good_noise_db": -40, "ok_noise_db": -30, "bad_noise_db": -20,
        "pid_adjust_factor": 0.25,
        "motor_sat_ok": 3, "motor_sat_warn": 10,
        "motor_imbal_ok": 4, "motor_imbal_warn": 10,
        "notes": "Long-range territory. Lower KV, noise-signal overlap begins. More filtering needed.",
    },
    8: {
        "name": "8-inch", "class": "mid",
        "good_overshoot": 14, "ok_overshoot": 22, "bad_overshoot": 40,
        "good_delay_ms": 15, "ok_delay_ms": 25, "bad_delay_ms": 45,
        "good_noise_db": -38, "ok_noise_db": -28, "bad_noise_db": -18,
        "pid_adjust_factor": 0.22,
        "motor_sat_ok": 4, "motor_sat_warn": 12,
        "motor_imbal_ok": 4, "motor_imbal_warn": 10,
        "notes": "Significant prop inertia. Motor response is noticeably slower. Conservative tuning essential.",
    },
    9: {
        "name": "9-inch", "class": "large",
        "good_overshoot": 15, "ok_overshoot": 25, "bad_overshoot": 42,
        "good_delay_ms": 18, "ok_delay_ms": 30, "bad_delay_ms": 50,
        "good_noise_db": -36, "ok_noise_db": -26, "bad_noise_db": -16,
        "pid_adjust_factor": 0.20,
        "motor_sat_ok": 4, "motor_sat_warn": 12,
        "motor_imbal_ok": 5, "motor_imbal_warn": 12,
        "notes": "Large quad territory. Signal and noise overlap significantly. Phase lag management critical.",
    },
    10: {
        "name": "10-inch", "class": "large",
        "good_overshoot": 18, "ok_overshoot": 28, "bad_overshoot": 45,
        "good_delay_ms": 20, "ok_delay_ms": 35, "bad_delay_ms": 55,
        "good_noise_db": -34, "ok_noise_db": -24, "bad_noise_db": -14,
        "pid_adjust_factor": 0.20,
        "motor_sat_ok": 5, "motor_sat_warn": 14,
        "motor_imbal_ok": 5, "motor_imbal_warn": 12,
        "notes": "Noise lives in signal band. Aggressive filtering adds dangerous phase lag. Every change must be small.",
    },
    12: {
        "name": "12-inch+", "class": "heavy",
        "good_overshoot": 20, "ok_overshoot": 32, "bad_overshoot": 50,
        "good_delay_ms": 25, "ok_delay_ms": 40, "bad_delay_ms": 65,
        "good_noise_db": -30, "ok_noise_db": -20, "bad_noise_db": -12,
        "pid_adjust_factor": 0.15,
        "motor_sat_ok": 6, "motor_sat_warn": 16,
        "motor_imbal_ok": 6, "motor_imbal_warn": 14,
        "notes": "Heavy lift / cinema. Extremely conservative tuning. Phase lag is the main enemy. "
                 "Consider reducing PID loop rate if not already at 500Hz or lower.",
    },
}

# Prop configuration drives noise prediction and filter ranges.
# Filter ranges depend on PROP diameter (not frame), because that's what
# determines where vibration energy lands in the spectrum.
# Blade count is handled separately in the harmonic prediction.
PROP_NOISE_PROFILES = {
    # prop_diameter_inches: { filter ranges, noise bands, safety margins }
    3:  {"gyro_lpf_range": (80, 350), "dterm_lpf_range": (60, 200), "filter_safety": 0.80,
         "noise_band_mid": (120, 400), "noise_band_high": 400},
    4:  {"gyro_lpf_range": (70, 320), "dterm_lpf_range": (50, 180), "filter_safety": 0.80,
         "noise_band_mid": (110, 380), "noise_band_high": 380},
    5:  {"gyro_lpf_range": (60, 300), "dterm_lpf_range": (40, 150), "filter_safety": 0.80,
         "noise_band_mid": (100, 350), "noise_band_high": 350},
    6:  {"gyro_lpf_range": (50, 250), "dterm_lpf_range": (35, 130), "filter_safety": 0.78,
         "noise_band_mid": (80, 300), "noise_band_high": 300},
    7:  {"gyro_lpf_range": (40, 200), "dterm_lpf_range": (30, 110), "filter_safety": 0.75,
         "noise_band_mid": (60, 250), "noise_band_high": 250},
    8:  {"gyro_lpf_range": (35, 180), "dterm_lpf_range": (25, 100), "filter_safety": 0.72,
         "noise_band_mid": (50, 220), "noise_band_high": 220},
    9:  {"gyro_lpf_range": (30, 150), "dterm_lpf_range": (20, 85),  "filter_safety": 0.70,
         "noise_band_mid": (40, 180), "noise_band_high": 180},
    10: {"gyro_lpf_range": (25, 120), "dterm_lpf_range": (18, 75),  "filter_safety": 0.68,
         "noise_band_mid": (30, 150), "noise_band_high": 150},
    12: {"gyro_lpf_range": (20, 100), "dterm_lpf_range": (15, 60),  "filter_safety": 0.65,
         "noise_band_mid": (20, 120), "noise_band_high": 120},
}

# Map sizes to nearest profile key
FRAME_SIZE_MAP = {3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8, 9: 9, 10: 10, 11: 12, 12: 12, 13: 12, 14: 12, 15: 12}
PROP_SIZE_MAP = {3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8, 9: 9, 10: 10, 11: 12, 12: 12, 13: 12, 14: 12, 15: 12}


def _nearest_key(size_map, inches):
    """Find nearest key in a size map."""
    key = size_map.get(inches)
    if key is None:
        key = min(size_map.keys(), key=lambda k: abs(k - inches))
        key = size_map[key]
    return key


def get_frame_profile(frame_inches=None, prop_inches=None, n_blades=3):
    """Build a complete profile by merging frame response thresholds with prop noise config.

    Args:
        frame_inches: Frame size (determines PID thresholds). Default: 5.
        prop_inches: Prop diameter (determines filter ranges). Default: same as frame.
        n_blades: Number of prop blades (2, 3, 4+). Default: 3 (triblade).
                  Affects harmonic prediction and filter range adjustment.

    Returns:
        Merged profile dict with all thresholds, filter ranges, and prop config.
    """
    if frame_inches is None:
        frame_inches = 5
    if prop_inches is None:
        prop_inches = frame_inches  # most common: prop matches frame

    frame_key = _nearest_key(FRAME_SIZE_MAP, frame_inches)
    prop_key = _nearest_key(PROP_SIZE_MAP, prop_inches)

    frame = FRAME_PROFILES[frame_key]
    prop_noise = PROP_NOISE_PROFILES[prop_key]

    # Blade count affects filter ranges — more blades push harmonics higher,
    # which actually gives more room between signal and noise.
    # 2-blade = baseline. 3-blade = harmonics 1.5× higher. Adjust ranges up.
    blade_factor = n_blades / 2.0  # 1.0 for biblade, 1.5 for triblade
    adjusted_noise = dict(prop_noise)
    if blade_factor > 1.0:
        # More blades = harmonics shift up = can afford higher filter cutoffs
        low, high = prop_noise["gyro_lpf_range"]
        adjusted_noise["gyro_lpf_range"] = (
            int(low * min(blade_factor, 1.3)),   # don't go crazy, cap at 30% boost
            int(high * min(blade_factor, 1.3)),
        )
        low, high = prop_noise["dterm_lpf_range"]
        adjusted_noise["dterm_lpf_range"] = (
            int(low * min(blade_factor, 1.3)),
            int(high * min(blade_factor, 1.3)),
        )
        mid_lo, mid_hi = prop_noise["noise_band_mid"]
        adjusted_noise["noise_band_mid"] = (
            int(mid_lo * blade_factor),
            int(mid_hi * blade_factor),
        )
        adjusted_noise["noise_band_high"] = int(prop_noise["noise_band_high"] * blade_factor)

    # Merge: frame thresholds + adjusted prop noise config + metadata
    profile = {**frame, **adjusted_noise}
    profile["frame_inches"] = frame_inches
    profile["prop_inches"] = prop_inches
    profile["n_blades"] = n_blades
    profile["name"] = f"{frame_inches}\"-frame / {prop_inches}\"×{n_blades}-blade"
    if frame_inches == prop_inches and n_blades == 3:
        profile["name"] = frame["name"]  # clean default: "5-inch"
    elif frame_inches == prop_inches and n_blades == 2:
        profile["name"] = f"{frame['name']} (biblade)"

    return profile


# ─── RPM and Phase Lag Estimation ─────────────────────────────────────────────

def estimate_rpm_range(motor_kv=None, cell_count=None, vbat_avg=None):
    """Estimate motor RPM range from KV and voltage.
    Returns (idle_rpm, max_rpm) or None if insufficient data."""
    voltage = None
    if vbat_avg and vbat_avg > 5:
        voltage = vbat_avg  # use measured
    elif cell_count:
        voltage = cell_count * 3.7  # nominal cell voltage

    if motor_kv and voltage:
        max_rpm = motor_kv * voltage
        idle_rpm = int(max_rpm * 0.15)  # ~15% throttle idle
        return (idle_rpm, int(max_rpm))
    return None


def estimate_prop_harmonics(rpm_range, n_blades=2):
    """Estimate expected vibration frequencies from RPM and blade count.

    The fundamental vibration frequency of a prop is:
        freq = (RPM × n_blades) / 60

    A 2-blade prop at 25000 RPM: 833 Hz fundamental
    A 3-blade prop at 25000 RPM: 1250 Hz fundamental (1.5× higher)
    A 2-blade prop at 10000 RPM: 333 Hz fundamental

    Returns list of harmonics with frequency ranges (min=idle, max=full throttle).
    """
    if rpm_range is None:
        return []
    idle_rpm, max_rpm = rpm_range
    fundamental_min = (idle_rpm * n_blades) / 60.0
    fundamental_max = (max_rpm * n_blades) / 60.0
    harmonics = []
    for h in range(1, 4):  # fundamental + 2 harmonics
        harmonics.append({
            "harmonic": h,
            "min_hz": fundamental_min * h,
            "max_hz": fundamental_max * h,
            "label": ["fundamental", "2nd harmonic", "3rd harmonic"][h - 1],
        })
    return harmonics


def _phase_shift(filter_type, freq_hz, cutoff_hz, q=None):
    """
    Compute phase lag (degrees) for a given filter at a specific frequency.

    Parameters
    ----------
    filter_type : str
        One of 'PT1', 'PT2', 'PT3', 'BIQUAD'.
    freq_hz : float
        Signal frequency of interest (Hz).
    cutoff_hz : float
        Filter cutoff frequency (Hz).
    q : float, optional
        Q factor (only used for BIQUAD). Default 0.5 (Butterworth).

    Returns
    -------
    float
        Phase lag in degrees (negative, indicating lag).
    """
    if cutoff_hz <= 0 or freq_hz <= 0:
        return 0.0

    filter_type = str(filter_type).upper()

    # PT1: first-order low-pass
    if filter_type == "PT1":
        return -np.degrees(np.arctan2(freq_hz, cutoff_hz))

    # PT2: two PT1 in series
    elif filter_type == "PT2":
        return -2 * np.degrees(np.arctan2(freq_hz, cutoff_hz))

    # PT3: three PT1 in series
    elif filter_type == "PT3":
        return -3 * np.degrees(np.arctan2(freq_hz, cutoff_hz))

    # BIQUAD (2nd order low-pass)
    elif "BIQUAD" in filter_type:
        if q is None:
            q = 0.5  # default Butterworth
        w = 2 * np.pi * freq_hz
        wc = 2 * np.pi * cutoff_hz
        # Transfer function: H(s) = wc² / (s² + (wc/q)s + wc²)
        # Phase = -atan2( (w/q)*wc , wc² - w² )
        imag = (w / q) * wc
        real = wc * wc - w * w
        return -np.degrees(np.arctan2(imag, real))

    # Fallback for unknown types
    else:
        order = {"PT1": 1, "PT2": 2, "PT3": 3, "BIQUAD": 2}.get(filter_type, 1)
        return -order * np.degrees(np.arctan2(freq_hz, cutoff_hz))


def estimate_filter_phase_lag(filter_hz, signal_freq, filter_type="PT1", q=None):
    """
    Estimate phase lag in degrees and milliseconds for a single filter.

    Parameters
    ----------
    filter_hz : float
        Filter cutoff frequency (Hz).
    signal_freq : float
        Signal frequency of interest (Hz).
    filter_type : str
        Filter type (e.g., 'PT1', 'BIQUAD').
    q : float, optional
        Q factor for BIQUAD (default None → 0.5).

    Returns
    -------
    dict
        {'degrees': phase_lag_deg, 'ms': delay_ms}
    """
    if filter_hz <= 0 or signal_freq <= 0:
        return {"degrees": 0.0, "ms": 0.0}

    phase_deg = _phase_shift(filter_type, signal_freq, filter_hz, q)

    # Convert phase at this frequency to a time delay
    delay_ms = (abs(phase_deg) / 360.0) / signal_freq * 1000.0

    return {"degrees": phase_deg, "ms": delay_ms}


def estimate_total_phase_lag(config, profile, signal_freq=50.0):
    """Estimate total phase lag through the entire filter chain at a given frequency.
    This is the key metric for large quads — too much lag and the PID loop fights itself."""

    total_deg = 0.0
    total_ms = 0.0
    chain = []

    # Gyro lowpass 1
    glp = config.get("gyro_lowpass_hz")
    if glp and isinstance(glp, (int, float)) and glp > 0:
        glp_type = config.get("gyro_lowpass_type", "PT1")
        lag = estimate_filter_phase_lag(glp, signal_freq, glp_type)
        total_deg += lag["degrees"]
        total_ms += lag["ms"]
        chain.append({"name": f"Gyro LPF ({glp}Hz, {glp_type})", **lag})

    # Gyro lowpass 2
    glp2 = config.get("gyro_lowpass2_hz")
    if glp2 and isinstance(glp2, (int, float)) and glp2 > 0:
        glp2_type = config.get("gyro_lowpass2_type", "PT1")
        lag = estimate_filter_phase_lag(glp2, signal_freq, glp2_type)
        total_deg += lag["degrees"]
        total_ms += lag["ms"]
        chain.append({"name": f"Gyro LPF2 ({glp2}Hz, {glp2_type})", **lag})

    # D-term lowpass (affects D only, but contributes to PID output timing)
    dlp = config.get("dterm_lpf_hz")
    if dlp and isinstance(dlp, (int, float)) and dlp > 0:
        dlp_type = config.get("dterm_lpf_type", "PT1")
        lag = estimate_filter_phase_lag(dlp, signal_freq, dlp_type)
        chain.append({"name": f"D-term LPF ({dlp}Hz, {dlp_type})", **lag,
                       "note": "affects D-term only"})

    return {
        "total_degrees": total_deg,
        "total_ms": total_ms,
        "signal_freq": signal_freq,
        "chain": chain,
    }


def analyze_motor_response(data, sr):
    """Measure how fast motors respond to PID demands.
    Cross-correlates PID output sum with motor output to estimate motor lag."""
    # Sum P+I+D for roll as proxy for total PID command
    pid_sum = None
    for axis in ["roll"]:
        components = []
        for term in ["P", "I", "D"]:
            key = f"axis{term}_{axis}"
            if key in data:
                components.append(data[key])
        if components:
            pid_sum = np.sum(components, axis=0)
            break

    if pid_sum is None or "motor0" not in data:
        return None

    # Use motor 0 as reference
    motor = data["motor0"]
    mask = ~(np.isnan(pid_sum) | np.isnan(motor))
    pid_clean = pid_sum[mask]
    motor_clean = motor[mask]

    if len(pid_clean) < 500:
        return None

    # Cross-correlate to find motor response delay
    n_corr = min(10000, len(pid_clean))
    pid_seg = pid_clean[:n_corr] - np.mean(pid_clean[:n_corr])
    mot_seg = motor_clean[:n_corr] - np.mean(motor_clean[:n_corr])

    corr = np.correlate(mot_seg, pid_seg, mode="full")
    mid = len(corr) // 2
    max_lag = int(sr * 0.05)  # search up to 50ms
    search = corr[mid:mid + max(1, max_lag)]

    if len(search) > 0 and np.max(search) > 0:
        delay_samples = np.argmax(search)
        delay_ms = float(delay_samples / sr * 1000.0)
    else:
        delay_ms = 0.0

    return {"motor_response_ms": delay_ms}


# Legacy constants for backward compat (overridden by profile when available)
GOOD_OVERSHOOT = 8.0
OK_OVERSHOOT = 15.0
BAD_OVERSHOOT = 30.0
GOOD_DELAY_MS = 8.0
OK_DELAY_MS = 15.0
BAD_DELAY_MS = 30.0
GOOD_NOISE_HIGH_DB = -45
OK_NOISE_HIGH_DB = -35
BAD_NOISE_HIGH_DB = -25
MOTOR_SAT_OK = 2.0
MOTOR_SAT_WARN = 8.0
MOTOR_IMBAL_OK = 3.0
MOTOR_IMBAL_WARN = 8.0
PID_ADJUST_FACTOR = 0.25
FILTER_ADJUST_FACTOR = 0.3


# ─── INAV Parameter Extraction ───────────────────────────────────────────────

INAV_PARAM_MAP = {
    "rollPID": "pid_roll", "pitchPID": "pid_pitch", "yawPID": "pid_yaw",
    "roll_p": "roll_p", "roll_i": "roll_i", "roll_d": "roll_d",
    "pitch_p": "pitch_p", "pitch_i": "pitch_i", "pitch_d": "pitch_d",
    "yaw_p": "yaw_p", "yaw_i": "yaw_i", "yaw_d": "yaw_d",
    # Filter settings (underscore format — Betaflight-style headers)
    "gyro_lpf": "gyro_lpf_type", "gyro_hardware_lpf": "gyro_hw_lpf",
    "gyro_lowpass_hz": "gyro_lowpass_hz", "gyro_lowpass_type": "gyro_lowpass_type",
    "gyro_lowpass2_hz": "gyro_lowpass2_hz", "gyro_lowpass2_type": "gyro_lowpass2_type",
    "dterm_lpf_hz": "dterm_lpf_hz", "dterm_lpf_type": "dterm_lpf_type",
    "dterm_lpf2_hz": "dterm_lpf2_hz", "dterm_lpf2_type": "dterm_lpf2_type",
    "yaw_lpf_hz": "yaw_lpf_hz",
    # Filter settings (camelCase — INAV 9.x blackbox header format)
    "gyro_lpf_hz": "gyro_lowpass_hz",
    # Dynamic notch (underscore — Betaflight-style)
    "dyn_notch_width_percent": "dyn_notch_width", "dyn_notch_q": "dyn_notch_q",
    "dyn_notch_min_hz": "dyn_notch_min_hz", "dyn_notch_max_hz": "dyn_notch_max_hz",
    "dynamic_gyro_notch_enabled": "dyn_notch_enabled",
    "dynamic_gyro_notch_q": "dyn_notch_q",
    "dynamic_gyro_notch_min_hz": "dyn_notch_min_hz",
    # Dynamic notch (camelCase — INAV blackbox headers)
    "dynamicGyroNotchQ": "dyn_notch_q",
    "dynamicGyroNotchMinHz": "dyn_notch_min_hz",
    "dynamicGyroNotchEnabled": "dyn_notch_enabled",
    "dynamicGyroNotchMode": "dyn_notch_mode",
    # RPM filter
    "rpm_gyro_filter_enabled": "rpm_filter_enabled",
    "rpm_gyro_harmonics": "rpm_filter_harmonics",
    "rpm_gyro_min_hz": "rpm_filter_min_hz",
    "rpm_gyro_q": "rpm_filter_q",
    # PID headers (camelCase INAV format — parsed separately below)
    "rollPID": "_rollPID", "pitchPID": "_pitchPID", "yawPID": "_yawPID",
    # Timing
    "looptime": "looptime", "gyro_sync_denom": "gyro_sync_denom",
    "pid_process_denom": "pid_denom",
    # Motor/protocol
    "motor_pwm_protocol": "motor_protocol", "motor_pwm_rate": "motor_pwm_rate",
    # Identity
    "firmwareVersion": "firmware_version", "firmwareType": "firmware_type",
    "Firmware revision": "firmware_revision",
    "Firmware type": "firmware_type", "Firmware date": "firmware_date",
    "craftName": "craft_name", "Craft name": "craft_name",
    "motorPwmProtocol": "motor_protocol", "features": "features",
}


def parse_headers_from_bbl(filepath):
    raw_params = {}
    try:
        with open(filepath, "rb") as f:
            for line in f:
                try:
                    text = line.decode("utf-8", errors="ignore").strip()
                except:
                    continue
                if text.startswith("H "):
                    parts = text[2:].split(":", 1)
                    if len(parts) == 2:
                        raw_params[parts[0].strip()] = parts[1].strip()
                elif text and not text.startswith("H"):
                    break
    except:
        pass
    return raw_params


def extract_fc_config(raw_params):
    config = {"raw": raw_params}
    for raw_key, our_key in INAV_PARAM_MAP.items():
        if raw_key in raw_params:
            config[our_key] = raw_params[raw_key]

    # Parse PID values from "rollPID:26,75,35,100" format (INAV header style)
    for axis in ["roll", "pitch", "yaw"]:
        pid_raw_key = f"_{axis}PID"
        if pid_raw_key in config:
            try:
                parts = config[pid_raw_key].split(",")
                if len(parts) >= 3:
                    config[f"{axis}_p"] = int(parts[0].strip())
                    config[f"{axis}_i"] = int(parts[1].strip())
                    config[f"{axis}_d"] = int(parts[2].strip())
                if len(parts) >= 4:
                    config[f"{axis}_ff"] = int(parts[3].strip())
            except (ValueError, IndexError):
                pass

    # Also parse "pid_roll" style keys (Betaflight/older format)
    for axis in ["roll", "pitch", "yaw"]:
        pid_key = f"pid_{axis}"
        if pid_key in config and f"{axis}_p" not in config:
            try:
                parts = config[pid_key].split(",")
                if len(parts) >= 3:
                    config[f"{axis}_p"] = int(parts[0].strip())
                    config[f"{axis}_i"] = int(parts[1].strip())
                    config[f"{axis}_d"] = int(parts[2].strip())
            except (ValueError, IndexError):
                pass

    # Convert numeric fields
    for key in ["gyro_lowpass_hz", "gyro_lowpass2_hz", "dterm_lpf_hz", "dterm_lpf2_hz",
                "yaw_lpf_hz", "dterm_lpf_type",
                "dyn_notch_min_hz", "dyn_notch_max_hz", "dyn_notch_q", "dyn_notch_width",
                "rpm_filter_enabled", "rpm_filter_harmonics", "rpm_filter_min_hz", "rpm_filter_q",
                "looptime", "gyro_sync_denom", "pid_denom", "motor_pwm_rate"]:
        if key in config:
            try:
                config[key] = int(config[key])
            except:
                try:
                    config[key] = float(config[key])
                except:
                    pass

    # Infer dynamic notch enabled state if not explicitly logged
    # INAV blackbox headers don't always include the enabled flag,
    # but Q > 0 means the filter is configured and active
    if "dyn_notch_enabled" not in config or config["dyn_notch_enabled"] is None:
        q = config.get("dyn_notch_q")
        if isinstance(q, (int, float)) and q > 0:
            config["dyn_notch_enabled"] = 1
        elif q is not None:
            config["dyn_notch_enabled"] = 0

    # Normalize enabled flags to int
    for flag_key in ["dyn_notch_enabled", "rpm_filter_enabled"]:
        val = config.get(flag_key)
        if isinstance(val, str):
            config[flag_key] = 1 if val.upper() in ("ON", "1", "TRUE", "YES") else 0

    for axis in ["roll", "pitch", "yaw"]:
        for term in ["p", "i", "d", "ff"]:
            key = f"{axis}_{term}"
            if key in config and isinstance(config[key], str):
                try:
                    config[key] = int(config[key])
                except:
                    pass

    # Map numeric filter types to names (INAV logs numeric codes)
    # INAV: 0=PT1, 1=PT1(alt), 2=BIQUAD, 3=PT2, 4=PT3
    FILTER_TYPE_NAMES = {0: "PT1", 1: "PT1", 2: "BIQUAD", 3: "PT2", 4: "PT3"}
    for ftype_key in ["dterm_lpf_type", "dterm_lpf2_type", "gyro_lowpass_type", "gyro_lowpass2_type"]:
        val = config.get(ftype_key)
        if isinstance(val, (int, float)):
            config[ftype_key] = FILTER_TYPE_NAMES.get(int(val), f"UNKNOWN({val})")

    # Infer gyro_lowpass_type from gyro_lpf_type if not explicitly set
    # INAV 9 logs "gyro_lpf" (hardware LPF enum) but not the software filter type
    if config.get("gyro_lowpass_type") is None or config.get("gyro_lowpass_type") == 0:
        config.setdefault("gyro_lowpass_type", "PT1")  # safe default

    # Map numeric motor protocol to name for DSHOT detection
    # INAV: 0=PWM, 1=ONESHOT125, 2=ONESHOT42, 3=MULTISHOT, 4=BRUSHED,
    #        5=DSHOT150, 6=DSHOT300, 7=DSHOT600, 8=DSHOT1200
    MOTOR_PROTOCOL_NAMES = {
        0: "PWM", 1: "ONESHOT125", 2: "ONESHOT42", 3: "MULTISHOT",
        4: "BRUSHED", 5: "DSHOT150", 6: "DSHOT300", 7: "DSHOT600", 8: "DSHOT1200",
    }
    mp = config.get("motor_protocol")
    if isinstance(mp, (int, float)):
        config["motor_protocol"] = MOTOR_PROTOCOL_NAMES.get(int(mp), str(mp))
    elif isinstance(mp, str):
        try:
            mp_int = int(mp)
            config["motor_protocol"] = MOTOR_PROTOCOL_NAMES.get(mp_int, mp)
        except ValueError:
            pass  # already a string like "DSHOT300"

    return config


def config_has_pid(config):
    return isinstance(config.get("roll_p"), (int, float))

def config_has_filters(config):
    return isinstance(config.get("gyro_lowpass_hz"), (int, float))


# ─── Blackbox Decode ──────────────────────────────────────────────────────────

# ─── Native Blackbox Binary Decoder ─────────────────────────────────────────
# Decodes .bbl/.bfl/.bbs binary logs directly — no external tools needed.
# Implements the INAV/Betaflight blackbox encoding: variable-byte integers,
# ZigZag signed encoding, grouped tag encodings (TAG2_3S32, TAG8_4S16,
# TAG8_8SVB), I-frame/P-frame predictors, and frame type dispatching.

class BlackboxDecoder:
    """Native Python decoder for INAV/Betaflight blackbox binary logs."""

    # Encoding types (from blackbox.c)
    ENC_SIGNED_VB = 0
    ENC_UNSIGNED_VB = 1
    ENC_NEG_14BIT = 3
    ENC_TAG8_8SVB = 6
    ENC_TAG2_3S32 = 7
    ENC_TAG8_4S16 = 8
    ENC_NULL = 9

    # Predictor types
    PRED_ZERO = 0
    PRED_PREVIOUS = 1
    PRED_STRAIGHT_LINE = 2
    PRED_AVERAGE_2 = 3
    PRED_MINTHROTTLE = 4
    PRED_MOTOR_0 = 5
    PRED_INC = 6
    PRED_1500 = 8
    PRED_VBATREF = 9

    FRAME_I, FRAME_P, FRAME_E = ord('I'), ord('P'), ord('E')
    FRAME_S, FRAME_G, FRAME_H = ord('S'), ord('G'), ord('H')
    VALID_FRAMES = {FRAME_I, FRAME_P, FRAME_E, FRAME_S, FRAME_G, FRAME_H}

    def __init__(self, raw_params):
        self.params = raw_params
        self.i_def = self._parse_field_def('I')
        self.p_def = self._parse_field_def('P', fallback_names=self.i_def)
        self.s_def = self._parse_field_def('S')
        self.g_def = self._parse_field_def('G')

        self.minthrottle = self._int_param('minthrottle', 1050)
        self.vbatref = self._int_param('vbatref', 0)
        mo = raw_params.get('motorOutput', '')
        self.motor_output_low = int(mo.split(',')[0]) if ',' in mo else self.minthrottle

        self._motor0_idx = None
        if self.i_def:
            try:
                self._motor0_idx = self.i_def['names'].index('motor[0]')
            except ValueError:
                pass

        self.stats = {'i_frames': 0, 'p_frames': 0, 'errors': 0,
                      'skipped_events': 0, 'skipped_slow': 0, 'skipped_gps': 0}

    def _int_param(self, key, default=0):
        try:
            return int(self.params.get(key, default))
        except (ValueError, TypeError):
            return default

    def _parse_field_def(self, frame_type, fallback_names=None):
        names_str = self.params.get(f'Field {frame_type} name', '')
        if names_str:
            names = [n.strip() for n in names_str.split(',')]
        elif fallback_names:
            # P-frames reuse I-frame field names
            names = fallback_names['names']
        else:
            return None
        n = len(names)
        pred_str = self.params.get(f'Field {frame_type} predictor', '')
        enc_str = self.params.get(f'Field {frame_type} encoding', '')
        if not pred_str and not enc_str:
            return None  # No predictor/encoding = no frame definition
        return {
            'names': names,
            'signed': self._int_list(f'Field {frame_type} signed', n),
            'predictor': self._int_list(f'Field {frame_type} predictor', n),
            'encoding': self._int_list(f'Field {frame_type} encoding', n),
            'count': n,
        }

    def _int_list(self, key, pad_to=0):
        val = self.params.get(key, '')
        if not val:
            return [0] * pad_to
        result = []
        for x in val.split(','):
            x = x.strip()
            if x:
                try:
                    result.append(int(x))
                except ValueError:
                    result.append(0)
        return (result + [0] * pad_to)[:pad_to]

    # ─── Binary Primitives ───────────────────────────────────────────────────

    def _read_byte(self):
        if self.pos >= self.end:
            raise IndexError("EOF")
        b = self.buf[self.pos]
        self.pos += 1
        return b

    def _read_unsigned_vb(self):
        result, shift = 0, 0
        for _ in range(5):
            b = self._read_byte()
            result |= (b & 0x7F) << shift
            if not (b & 0x80):
                return result
            shift += 7
        return result

    def _read_signed_vb(self):
        u = self._read_unsigned_vb()
        return (u >> 1) ^ (-(u & 1))  # ZigZag decode

    def _read_neg_14bit(self):
        # NEG_14BIT: value is negated then encoded as unsigned variable-byte.
        # Decode: read unsigned VB, then negate.
        return -self._read_unsigned_vb()

    def _read_tag2_3s32(self):
        """TAG2_3S32: top 2 bits of lead byte select field size for all 3 values.
        Reference: betaflight/blackbox-tools/src/decoders.c streamReadTag2_3S32"""
        lead = self._read_byte()
        selector = lead >> 6
        if selector == 0:
            # 2-bit fields, all packed in the remaining 6 bits of lead byte
            v0 = (lead >> 4) & 0x03; v0 = v0 - 4 if v0 >= 2 else v0
            v1 = (lead >> 2) & 0x03; v1 = v1 - 4 if v1 >= 2 else v1
            v2 = lead & 0x03;        v2 = v2 - 4 if v2 >= 2 else v2
            return [v0, v1, v2]
        elif selector == 1:
            # 4-bit fields: first in low nibble of lead, next 2 in second byte
            v0 = lead & 0x0F; v0 = v0 - 16 if v0 >= 8 else v0
            b = self._read_byte()
            v1 = b >> 4;     v1 = v1 - 16 if v1 >= 8 else v1
            v2 = b & 0x0F;   v2 = v2 - 16 if v2 >= 8 else v2
            return [v0, v1, v2]
        elif selector == 2:
            # 6-bit fields: first in low 6 bits of lead, next 2 each in 1 byte
            v0 = lead & 0x3F; v0 = v0 - 64 if v0 >= 32 else v0
            b1 = self._read_byte()
            v1 = b1 & 0x3F;  v1 = v1 - 64 if v1 >= 32 else v1
            b2 = self._read_byte()
            v2 = b2 & 0x3F;  v2 = v2 - 64 if v2 >= 32 else v2
            return [v0, v1, v2]
        else:
            # selector == 3: per-field variable size (0/8/16/24 bit)
            values = [0, 0, 0]
            for i in range(3):
                tag = (lead >> (i * 2)) & 0x03
                if tag == 0:
                    values[i] = 0
                elif tag == 1:
                    v = self._read_byte()
                    values[i] = v - 256 if v >= 128 else v
                elif tag == 2:
                    b1, b2 = self._read_byte(), self._read_byte()
                    v = (b1 << 8) | b2
                    values[i] = v - 65536 if v >= 32768 else v
                elif tag == 3:
                    b1, b2, b3 = self._read_byte(), self._read_byte(), self._read_byte()
                    v = (b1 << 16) | (b2 << 8) | b3
                    values[i] = v - 0x1000000 if v >= 0x800000 else v
            return values

    def _read_tag8_4s16(self):
        """TAG8_4S16 v2: nibble-buffered, 2-bit selectors for 4 fields.
        Reference: atomgomba/orangebox/decoders.py _tag8_4s16_v2"""
        selector = self._read_byte()
        values = [0, 0, 0, 0]
        nibble_idx = 0  # 0 = aligned, 1 = have pending low nibble in buf
        buf = 0
        for i in range(4):
            ft = selector & 0x03
            if ft == 0:  # zero
                values[i] = 0
            elif ft == 1:  # 4-bit nibble
                if nibble_idx == 0:
                    buf = self._read_byte()
                    v = buf >> 4
                    nibble_idx = 1
                else:
                    v = buf & 0x0F
                    nibble_idx = 0
                values[i] = v - 16 if v >= 8 else v
            elif ft == 2:  # 8-bit
                if nibble_idx == 0:
                    v = self._read_byte()
                else:
                    v = (buf & 0x0F) << 4
                    buf = self._read_byte()
                    v |= buf >> 4
                    # nibble_idx stays 1
                values[i] = v - 256 if v >= 128 else v
            elif ft == 3:  # 16-bit
                if nibble_idx == 0:
                    b1, b2 = self._read_byte(), self._read_byte()
                    v = (b1 << 8) | b2
                else:
                    b1 = (buf & 0x0F) << 4
                    buf = self._read_byte()
                    b1 |= buf >> 4
                    b2 = (buf & 0x0F) << 4
                    buf = self._read_byte()
                    b2 |= buf >> 4
                    v = (b1 << 8) | b2
                    # nibble_idx stays 1
                values[i] = v - 65536 if v >= 32768 else v
            selector >>= 2
        return values

    def _read_tag8_8svb(self, count=8):
        header = self._read_byte()
        values = [0] * count
        for i in range(min(8, count)):
            if header & (1 << i):
                values[i] = self._read_signed_vb()
        return values

    # ─── Frame Decoding ──────────────────────────────────────────────────────

    def _decode_raw_values(self, field_def):
        count = field_def['count']
        enc = field_def['encoding']
        values = [0] * count
        i = 0
        while i < count:
            e = enc[i]
            if e == self.ENC_NULL:
                values[i] = 0; i += 1
            elif e == self.ENC_SIGNED_VB:
                values[i] = self._read_signed_vb(); i += 1
            elif e == self.ENC_UNSIGNED_VB:
                values[i] = self._read_unsigned_vb(); i += 1
            elif e == self.ENC_NEG_14BIT:
                values[i] = self._read_neg_14bit(); i += 1
            elif e == self.ENC_TAG2_3S32:
                grp = self._read_tag2_3s32()
                for j, v in enumerate(grp):
                    if i + j < count: values[i + j] = v
                i += 3
            elif e == self.ENC_TAG8_4S16:
                grp = self._read_tag8_4s16()
                for j, v in enumerate(grp):
                    if i + j < count: values[i + j] = v
                i += 4
            elif e == self.ENC_TAG8_8SVB:
                # Count consecutive TAG8_8SVB fields (up to 8)
                run = 0
                while i + run < count and enc[i + run] == self.ENC_TAG8_8SVB and run < 8:
                    run += 1
                grp = self._read_tag8_8svb(run)
                for j, v in enumerate(grp):
                    if i + j < count: values[i + j] = v
                i += run
            else:
                values[i] = self._read_signed_vb(); i += 1
        return values

    def _apply_i_predictors(self, raw):
        values = list(raw)
        for i in range(self.i_def['count']):
            pred = self.i_def['predictor'][i]
            if pred == self.PRED_MINTHROTTLE:
                values[i] = raw[i] + self.minthrottle
            elif pred == self.PRED_MOTOR_0:
                if self._motor0_idx is not None and self._motor0_idx < i:
                    values[i] = raw[i] + values[self._motor0_idx]
            elif pred == self.PRED_VBATREF:
                values[i] = raw[i] + self.vbatref
            elif pred == self.PRED_1500:
                values[i] = raw[i] + 1500
        return values

    def _apply_p_predictors(self, raw, prev, prev_prev):
        values = list(raw)
        CLAMP = 2**31  # prevent overflow snowball from straight-line predictor
        for i in range(self.p_def['count']):
            pred = self.p_def['predictor'][i]
            if pred == self.PRED_ZERO:
                pass
            elif pred == self.PRED_PREVIOUS:
                values[i] = raw[i] + prev[i]
            elif pred == self.PRED_STRAIGHT_LINE:
                predicted = (2 * prev[i] - prev_prev[i]) if prev_prev else prev[i]
                values[i] = raw[i] + predicted
            elif pred == self.PRED_AVERAGE_2:
                predicted = ((prev[i] + prev_prev[i]) // 2) if prev_prev else prev[i]
                values[i] = raw[i] + predicted
            elif pred == self.PRED_MINTHROTTLE:
                values[i] = raw[i] + self.minthrottle
            elif pred == self.PRED_MOTOR_0:
                if self._motor0_idx is not None:
                    values[i] = raw[i] + values[self._motor0_idx]
                else:
                    values[i] = raw[i] + prev[i]
            elif pred == self.PRED_INC:
                values[i] = prev[i] + 1
            elif pred == self.PRED_1500:
                values[i] = raw[i] + 1500
            elif pred == self.PRED_VBATREF:
                values[i] = raw[i] + self.vbatref
            else:
                values[i] = raw[i] + prev[i]
            # Clamp to prevent overflow compounding across frames
            if values[i] > CLAMP or values[i] < -CLAMP:
                values[i] = prev[i] if prev else 0
        return values

    def _resync(self):
        while self.pos < self.end:
            if self.buf[self.pos] in self.VALID_FRAMES:
                return True
            self.pos += 1
        return False

    def _skip_frame(self, field_def):
        if field_def is None:
            self._resync()
            return
        try:
            self._decode_raw_values(field_def)
        except (IndexError, ValueError):
            self._resync()

    def _find_binary_start(self):
        """Find byte offset where headers end and binary data begins."""
        pos = 0
        last_header_end = 0
        while pos < min(self.end, 65536):  # headers are always in first 64KB
            nl = -1
            for i in range(pos, min(pos + 2000, self.end)):
                if self.buf[i] == ord('\n'):
                    nl = i
                    break
            if nl == -1:
                break
            if nl - pos >= 2 and self.buf[pos] == ord('H') and self.buf[pos + 1] == ord(' '):
                last_header_end = nl + 1
                pos = nl + 1
                continue
            if self.buf[pos] in self.VALID_FRAMES:
                return pos
            pos = nl + 1
        return last_header_end

    # ─── Event Frame Parsing ────────────────────────────────────────────────

    # INAV/BF event types and their data sizes
    EVT_SYNC_BEEP = 0           # data: 1 unsigned VB (time)
    EVT_INFLIGHT_ADJUSTMENT = 13 # data: 1 byte (func) + 1 signed VB or 4 bytes
    EVT_LOGGING_RESUME = 14      # data: unsigned VB (iter) + unsigned VB (time)
    EVT_FLIGHT_MODE = 15         # data: unsigned VB (flags) + unsigned VB (last)
    EVT_LOG_END = 255            # end of this log

    def _skip_event_frame(self):
        """Parse and skip an event frame by reading its type-specific data."""
        try:
            evt_type = self._read_byte()
            if evt_type == self.EVT_SYNC_BEEP:
                self._read_unsigned_vb()  # time
            elif evt_type == self.EVT_INFLIGHT_ADJUSTMENT:
                func = self._read_byte()
                if func < 128:
                    self._read_signed_vb()  # int value
                else:
                    for _ in range(4): self._read_byte()  # float value
            elif evt_type == self.EVT_LOGGING_RESUME:
                self._read_unsigned_vb()  # iteration
                self._read_unsigned_vb()  # currentTime
            elif evt_type == self.EVT_FLIGHT_MODE:
                self._read_unsigned_vb()  # flags
                self._read_unsigned_vb()  # lastFlags
            elif evt_type == self.EVT_LOG_END:
                # End of this log — scan for next set of headers or EOF
                self._skip_to_next_log()
                return 'log_end'
            else:
                # Unknown event type — try to resync
                self._resync()
            return 'ok'
        except (IndexError, ValueError):
            self._resync()
            return 'error'

    def _skip_to_next_log(self):
        """Skip forward past any new header block to the next binary data."""
        while self.pos < self.end:
            # Look for 'H ' (start of new headers)
            if self.pos + 1 < self.end and self.buf[self.pos] == ord('H') and self.buf[self.pos + 1] == ord(' '):
                # Skip header lines until we hit binary data
                while self.pos < self.end:
                    # Find newline
                    nl = -1
                    for i in range(self.pos, min(self.pos + 2000, self.end)):
                        if self.buf[i] == ord('\n'):
                            nl = i
                            break
                    if nl == -1:
                        self.pos = self.end
                        return
                    self.pos = nl + 1
                    # Check if next line is still a header
                    if self.pos + 1 < self.end and self.buf[self.pos] == ord('H') and self.buf[self.pos + 1] == ord(' '):
                        continue
                    else:
                        return  # Now at binary data start
            elif self.buf[self.pos] in self.VALID_FRAMES:
                return  # Found frame data
            self.pos += 1

    def _validate_i_frame(self, values):
        """Sanity check I-frame values. Returns False if clearly garbage."""
        n = len(values)
        # Check motor[0] — should be near minthrottle..maxthrottle range
        if self._motor0_idx is not None and self._motor0_idx < n:
            m0 = values[self._motor0_idx]
            max_motor = self._int_param('maxthrottle', 2000)
            if m0 < self.minthrottle - 200 or m0 > max_motor + 500:
                return False
        # Check time (field 1) — should be non-negative and < 1 hour in us
        if n > 1:
            t = values[1]
            if t < 0 or t > 3600_000_000:
                return False
        return True

    # ─── Main Decode ─────────────────────────────────────────────────────────

    def decode_file(self, filepath):
        """Decode a blackbox binary log. Returns (frames, field_names)."""
        with open(filepath, 'rb') as f:
            self.buf = f.read()
        self.end = len(self.buf)

        if self.i_def is None:
            print("  ERROR: No I-frame field definitions in headers")
            return [], []

        self.pos = self._find_binary_start()
        all_frames = []
        prev, prev_prev = None, None
        consec_errors = 0

        # Progress tracking
        last_pct = -1
        show_progress = self.end > 500_000  # only for files > 500KB

        while self.pos < self.end:
            # Progress indicator
            if show_progress:
                pct = (self.pos * 100) // self.end
                if pct != last_pct and pct % 5 == 0:
                    frames_so_far = self.stats['i_frames'] + self.stats['p_frames']
                    print(f"\r  Decoding: {pct}% ({frames_so_far:,} frames)", end="", flush=True)
                    last_pct = pct

            byte = self.buf[self.pos]

            # Detect new log headers mid-stream (multi-log files)
            if byte == ord('H') and self.pos + 1 < self.end and self.buf[self.pos + 1] == ord(' '):
                self._skip_to_next_log()
                prev, prev_prev = None, None  # reset predictor state
                self.stats['skipped_events'] += 1
                continue

            if byte == self.FRAME_I:
                self.pos += 1
                saved = self.pos
                try:
                    raw = self._decode_raw_values(self.i_def)
                    values = self._apply_i_predictors(raw)
                    if self._validate_i_frame(values):
                        all_frames.append(values)
                        prev_prev, prev = prev, values
                        self.stats['i_frames'] += 1
                        consec_errors = 0
                    else:
                        # Invalid I-frame — likely false positive from resync
                        self.stats['errors'] += 1
                        self.pos = saved
                        if not self._resync(): break
                except (IndexError, ValueError):
                    consec_errors += 1
                    self.pos = saved
                    if not self._resync(): break
                    if consec_errors > 500: break

            elif byte == self.FRAME_P:
                self.pos += 1
                if prev is None:
                    if not self._resync(): break
                    continue
                saved = self.pos
                try:
                    raw = self._decode_raw_values(self.p_def)
                    values = self._apply_p_predictors(raw, prev, prev_prev)
                    all_frames.append(values)
                    prev_prev, prev = prev, values
                    self.stats['p_frames'] += 1
                    consec_errors = 0
                except (IndexError, ValueError):
                    consec_errors += 1
                    self.pos = saved
                    if not self._resync(): break
                    if consec_errors > 500: break

            elif byte == self.FRAME_E:
                self.pos += 1
                result = self._skip_event_frame()
                self.stats['skipped_events'] += 1
                if result == 'log_end':
                    prev, prev_prev = None, None  # reset for new log

            elif byte == self.FRAME_S:
                self.pos += 1
                self._skip_frame(self.s_def)
                self.stats['skipped_slow'] += 1

            elif byte == self.FRAME_G:
                self.pos += 1
                self._skip_frame(self.g_def)
                self.stats['skipped_gps'] += 1

            elif byte == self.FRAME_H:
                self.pos += 1
                # GPS Home frame — read its field definitions
                if self.g_def:
                    # H-frame has its own field definition
                    h_def = self._parse_field_def('H')
                    if h_def:
                        self._skip_frame(h_def)
                    else:
                        self._resync()
                else:
                    self._resync()

            else:
                self.pos += 1
                consec_errors += 1
                if consec_errors > 500: break

        if show_progress:
            print("\r  Decoding: done                              ", flush=True)
        return all_frames, self.i_def['names']

    def frames_to_data_dict(self, frames, field_names):
        """Convert decoded frames to analysis data dict (same format as parse_csv_log)."""
        if not frames:
            return None

        n_rows = len(frames)
        n_fields = len(field_names)

        # Clamp limit: any value beyond this is a decode error
        CLAMP = 2**31

        # Build arrays per field
        raw_arrays = {}
        for fi, name in enumerate(field_names):
            arr = np.empty(n_rows, dtype=np.float64)
            for ri in range(n_rows):
                f = frames[ri]
                if fi < len(f):
                    v = f[fi]
                    if -CLAMP <= v <= CLAMP:
                        arr[ri] = float(v)
                    else:
                        arr[ri] = np.nan
                else:
                    arr[ri] = np.nan
            raw_arrays[name] = arr

        # Map to standard analysis keys
        col_map = {
            "time": ["time"],
            "loopiter": ["loopIteration"],
            "gyro_roll": ["gyroADC[0]", "gyroData[0]", "gyro[0]"],
            "gyro_pitch": ["gyroADC[1]", "gyroData[1]", "gyro[1]"],
            "gyro_yaw": ["gyroADC[2]", "gyroData[2]", "gyro[2]"],
            "setpoint_roll": ["rcCommand[0]", "setpoint[0]"],
            "setpoint_pitch": ["rcCommand[1]", "setpoint[1]"],
            "setpoint_yaw": ["rcCommand[2]", "setpoint[2]"],
            "axisP_roll": ["axisP[0]"], "axisP_pitch": ["axisP[1]"], "axisP_yaw": ["axisP[2]"],
            "axisI_roll": ["axisI[0]"], "axisI_pitch": ["axisI[1]"], "axisI_yaw": ["axisI[2]"],
            "axisD_roll": ["axisD[0]"], "axisD_pitch": ["axisD[1]"], "axisD_yaw": ["axisD[2]"],
            "motor0": ["motor[0]"], "motor1": ["motor[1]"],
            "motor2": ["motor[2]"], "motor3": ["motor[3]"],
        }

        data = {}
        for our_key, candidates in col_map.items():
            for fn in candidates:
                if fn in raw_arrays:
                    data[our_key] = raw_arrays[fn]
                    break

        # Time and sample rate
        # Prefer header-derived sample rate (reliable) over decoded time diffs (noisy)
        header_rate = None
        looptime = self._int_param('looptime', 0)
        p_interval_str = self.params.get('P interval', '')
        if looptime > 0 and '/' in p_interval_str:
            try:
                p_num, p_denom = p_interval_str.split('/')
                p_ratio = int(p_denom) / max(int(p_num), 1)  # e.g. "1/2" → every 2nd loop
                sample_period_us = looptime * p_ratio
                if sample_period_us > 0:
                    header_rate = 1e6 / sample_period_us
            except (ValueError, ZeroDivisionError):
                pass

        if header_rate and header_rate > 10:
            data["sample_rate"] = header_rate
            data["time_s"] = np.arange(n_rows) / header_rate
        elif "time" in data:
            time_us = data["time"]
            valid_time = time_us[~np.isnan(time_us)]
            if len(valid_time) > 10:
                diffs = np.diff(valid_time)
                good_diffs = diffs[(diffs > 0) & (diffs < 1e7)]
                if len(good_diffs) > 10:
                    data["sample_rate"] = float(1e6 / np.median(good_diffs))
                    data["time_s"] = np.arange(n_rows) / data["sample_rate"]
                else:
                    data["sample_rate"] = 1000.0
                    data["time_s"] = np.arange(n_rows) / 1000.0
            else:
                data["sample_rate"] = 1000.0
                data["time_s"] = np.arange(n_rows) / 1000.0
        else:
            data["sample_rate"] = 1000.0
            data["time_s"] = np.arange(n_rows) / 1000.0

        data["headers"] = field_names
        data["n_rows"] = n_rows
        data["found_columns"] = [k for k in col_map if k in data]
        return data


def decode_blackbox_native(filepath, raw_params):
    """Decode a blackbox binary log file using the native decoder.
    Returns a data dict in the same format as parse_csv_log."""
    decoder = BlackboxDecoder(raw_params)
    frames, field_names = decoder.decode_file(filepath)

    total = decoder.stats['i_frames'] + decoder.stats['p_frames']
    if total == 0:
        print("  ERROR: No frames decoded from blackbox log.")
        print(f"    Stats: {decoder.stats}")
        sys.exit(1)

    print(f"  Decoded: {total:,} frames "
          f"({decoder.stats['i_frames']} I + {decoder.stats['p_frames']} P)")
    if decoder.stats['skipped_events'] or decoder.stats['skipped_gps']:
        print(f"    Skipped: {decoder.stats['skipped_events']} event, "
              f"{decoder.stats['skipped_slow']} slow, {decoder.stats['skipped_gps']} GPS")

    data = decoder.frames_to_data_dict(frames, field_names)
    if data is None:
        print("  ERROR: Failed to convert decoded frames to analysis data.")
        sys.exit(1)

    return data


# ─── CSV Parsing ──────────────────────────────────────────────────────────────

def flexible_col_match(headers, patterns):
    for i, h in enumerate(headers):
        h_clean = h.strip().lower()
        for pat in patterns:
            if pat.lower() in h_clean:
                return i
    return None


def parse_csv_log(csv_path):
    with open(csv_path, "r", errors="ignore") as f:
        lines = [l.strip() for l in f if l.strip() and not l.strip().startswith(("#", "H "))]
    if len(lines) < 2:
        print("ERROR: CSV has insufficient data.")
        sys.exit(1)

    reader = csv.reader(lines)
    headers = [h.strip() for h in next(reader)]
    rows = list(reader)
    data = {}
    col_map = {
        "time": ["time (us)", "time(us)", "time_us", "time"],
        "loopiter": ["loopiteration", "loop_iteration"],
        "gyro_roll": ["gyroadc[0]", "gyrodata[0]", "gyro[0]", "gyro_roll"],
        "gyro_pitch": ["gyroadc[1]", "gyrodata[1]", "gyro[1]", "gyro_pitch"],
        "gyro_yaw": ["gyroadc[2]", "gyrodata[2]", "gyro[2]", "gyro_yaw"],
        "setpoint_roll": ["setpoint[0]", "rccommand[0]"],
        "setpoint_pitch": ["setpoint[1]", "rccommand[1]"],
        "setpoint_yaw": ["setpoint[2]", "rccommand[2]"],
        "axisP_roll": ["axisp[0]", "pidp[0]", "p[0]"],
        "axisP_pitch": ["axisp[1]", "pidp[1]", "p[1]"],
        "axisP_yaw": ["axisp[2]", "pidp[2]", "p[2]"],
        "axisI_roll": ["axisi[0]", "pidi[0]", "i[0]"],
        "axisI_pitch": ["axisi[1]", "pidi[1]", "i[1]"],
        "axisI_yaw": ["axisi[2]", "pidi[2]", "i[2]"],
        "axisD_roll": ["axisd[0]", "pidd[0]", "d[0]"],
        "axisD_pitch": ["axisd[1]", "pidd[1]", "d[1]"],
        "axisD_yaw": ["axisd[2]", "pidd[2]", "d[2]"],
        "motor0": ["motor[0]", "motor_0"], "motor1": ["motor[1]", "motor_1"],
        "motor2": ["motor[2]", "motor_2"], "motor3": ["motor[3]", "motor_3"],
    }
    found = {}
    for key, patterns in col_map.items():
        idx = flexible_col_match(headers, patterns)
        if idx is not None:
            found[key] = idx

    n_rows = len(rows)
    for key, idx in found.items():
        arr = np.zeros(n_rows, dtype=np.float64)
        for i, row in enumerate(rows):
            if idx < len(row):
                try:
                    arr[i] = float(row[idx])
                except:
                    arr[i] = np.nan
        data[key] = arr

    if "time" in data:
        time_us = data["time"]
        valid = np.diff(time_us)
        valid = valid[valid > 0]
        if len(valid) > 10:
            data["sample_rate"] = 1e6 / np.median(valid)
            data["time_s"] = (time_us - time_us[0]) / 1e6
        else:
            data["sample_rate"] = 1000.0
            data["time_s"] = np.arange(n_rows) / 1000.0
    else:
        data["sample_rate"] = 1000.0
        data["time_s"] = np.arange(n_rows) / 1000.0

    data["headers"] = headers
    data["n_rows"] = n_rows
    data["found_columns"] = list(found.keys())
    return data


# ─── Signal Analysis ──────────────────────────────────────────────────────────

def compute_psd(arr, sr, nperseg=None):
    clean = arr[~np.isnan(arr)]
    if nperseg is None:
        nperseg = min(4096, len(clean) // 4)
    nperseg = max(256, min(nperseg, len(clean) // 2))
    freqs, psd = signal.welch(clean, fs=sr, nperseg=nperseg, window="hann", scaling="density")
    return freqs, 10 * np.log10(psd + 1e-20)


def find_noise_peaks(freqs, psd_db, n_peaks=5, min_height_db=-30, min_prominence=6):
    peaks, props = signal.find_peaks(psd_db, height=min_height_db, prominence=min_prominence, distance=10)
    if len(peaks) == 0:
        return []
    prominences = props.get("prominences", np.zeros(len(peaks)))
    order = np.argsort(prominences)[::-1][:n_peaks]
    results = [{"freq_hz": float(freqs[peaks[i]]), "power_db": float(psd_db[peaks[i]]),
                "prominence": float(prominences[i])} for i in order]
    results.sort(key=lambda x: x["freq_hz"])
    return results


def analyze_noise(data, axis_name, gyro_key, sr):
    if gyro_key not in data:
        return None
    freqs, psd_db = compute_psd(data[gyro_key], sr)
    peaks = find_noise_peaks(freqs, psd_db)
    low_band = psd_db[(freqs >= 10) & (freqs < 100)]
    mid_band = psd_db[(freqs >= 100) & (freqs < 300)]
    high_band = psd_db[(freqs >= 300)]
    window = min(20, len(psd_db) // 10)
    smoothed = np.convolve(psd_db, np.ones(max(1,window))/max(1,window), mode="same") if window > 1 else psd_db
    noise_mask = smoothed > -35
    noise_start_freq = float(freqs[np.argmax(noise_mask)]) if np.any(noise_mask) else float(sr / 2)
    return {
        "axis": axis_name, "freqs": freqs, "psd_db": psd_db, "peaks": peaks,
        "noise_start_freq": noise_start_freq,
        "rms_low": float(np.mean(low_band)) if len(low_band) > 0 else -80,
        "rms_mid": float(np.mean(mid_band)) if len(mid_band) > 0 else -80,
        "rms_high": float(np.mean(high_band)) if len(high_band) > 0 else -80,
    }


def measure_tracking_delay_xcorr(sp, gy, sr, max_delay_ms=200):
    """Measure average tracking delay using cross-correlation.
    This is robust against residual tracking error, overshoot, and noise.
    Returns delay in ms, or None if insufficient signal."""
    if len(sp) < 500:
        return None

    # High-pass filter to remove DC/drift — we only care about dynamics
    # Simple differencing acts as a high-pass filter
    sp_hp = np.diff(sp.astype(np.float64))
    gy_hp = np.diff(gy.astype(np.float64))

    # Only analyze segments with actual stick activity
    # (ignore hover/cruise where setpoint is constant)
    activity = np.abs(sp_hp)
    threshold = np.percentile(activity[activity > 0], 50) if np.any(activity > 0) else 0
    if threshold == 0:
        return None

    active_mask = activity > threshold
    if np.sum(active_mask) < 200:
        return None

    # Normalize for correlation
    sp_sig = sp_hp.copy()
    gy_sig = gy_hp.copy()
    sp_sig -= np.mean(sp_sig)
    gy_sig -= np.mean(gy_sig)
    sp_std = np.std(sp_sig)
    gy_std = np.std(gy_sig)
    if sp_std < 1e-6 or gy_std < 1e-6:
        return None

    # Cross-correlation: find lag where gyro best matches setpoint
    max_lag = int(sr * max_delay_ms / 1000)
    max_lag = min(max_lag, len(sp_sig) // 4)

    # Use segment-based approach: split into overlapping windows,
    # compute cross-correlation for each, take median
    win_size = min(int(sr * 0.5), len(sp_sig) // 4)  # 500ms windows
    if win_size < 100:
        win_size = len(sp_sig)

    step = win_size // 2
    delays = []

    for start in range(0, len(sp_sig) - win_size, step):
        sp_win = sp_sig[start:start + win_size]
        gy_win = gy_sig[start:start + win_size]

        # Skip quiet windows
        if np.std(sp_win) < sp_std * 0.3:
            continue

        # Cross-correlate: positive lag = gyro lags behind setpoint
        corr = np.correlate(gy_win, sp_win, mode='full')
        mid = len(sp_win) - 1
        # Only look at positive lags (gyro should lag behind setpoint)
        search_region = corr[mid:mid + max_lag]
        if len(search_region) < 2:
            continue

        peak_idx = np.argmax(search_region)
        delay_ms = float(peak_idx / sr * 1000)

        # Sanity: delay should be positive and reasonable
        if 0 < delay_ms < max_delay_ms:
            delays.append(delay_ms)

    if len(delays) < 3:
        return None

    return float(np.median(delays))


def analyze_pid_response(data, axis_idx, sr):
    axis = AXIS_NAMES[axis_idx]
    sp_key, gyro_key = f"setpoint_{axis.lower()}", f"gyro_{axis.lower()}"
    if sp_key not in data or gyro_key not in data:
        return None
    sp, gy = data[sp_key].copy(), data[gyro_key].copy()
    mask = ~(np.isnan(sp) | np.isnan(gy))
    sp, gy = sp[mask], gy[mask]
    if len(sp) < 100:
        return None

    error = sp - gy
    rms_error = float(np.sqrt(np.mean(error**2)))

    # ── Delay: cross-correlation (robust, uses entire signal) ──
    avg_delay = measure_tracking_delay_xcorr(sp, gy, sr)

    # ── Overshoot: step detection ──
    sp_diff = np.abs(np.diff(sp))
    threshold = np.percentile(sp_diff[sp_diff > 0], 90) if np.any(sp_diff > 0) else 1
    steps = np.where(sp_diff > threshold)[0]
    window = int(sr * 0.1)  # 100ms window
    settle_window = int(sr * 0.01)  # 10ms pre-step settling check

    overshoots = []
    n_steps_analyzed = 0
    for idx in steps[::max(1, len(steps)//30)]:
        end = min(idx + window, len(gy) - 1)
        if end - idx < 10:
            continue
        sp_target, sp_start = sp[end], sp[idx]
        delta = sp_target - sp_start
        if abs(delta) < 1:
            continue

        # Pre-step sanity: reject if gyro is already way past setpoint in the step direction
        # (residual overshoot from prior step would contaminate this measurement)
        if idx >= settle_window:
            pre_gyro = np.mean(gy[idx-settle_window:idx])
            pre_offset = pre_gyro - sp[idx]
            # If offset is >50% of delta AND in the same direction as the step,
            # the gyro is already flying past where we need to measure
            if abs(pre_offset) > abs(delta) * 0.5 and np.sign(pre_offset) == np.sign(delta):
                continue  # skip: gyro already headed past target

        n_steps_analyzed += 1
        response = gy[idx:end]

        # ── Overshoot: peak beyond target ──
        peak = np.max(response) if delta > 0 else np.min(response)
        os_pct = ((peak - sp_target) / delta * 100) if delta > 0 else ((sp_target - peak) / (-delta) * 100)
        if 0 < os_pct < 200:
            overshoots.append(os_pct)

    avg_overshoot = float(np.median(overshoots)) if overshoots else None

    pid_stats = {}
    for name, suffix in [("P", "P"), ("I", "I"), ("D", "D")]:
        key = f"axis{suffix}_{axis.lower()}"
        if key in data:
            arr = data[key][mask]
            arr = arr[~np.isnan(arr)]
            if len(arr) > 0:
                pid_stats[name] = {"rms": float(np.sqrt(np.mean(arr**2))), "max": float(np.max(np.abs(arr)))}

    return {"axis": axis, "rms_error": rms_error, "tracking_delay_ms": avg_delay,
            "avg_overshoot_pct": avg_overshoot, "n_steps": n_steps_analyzed,
            "setpoint": sp, "gyro": gy, "pid_stats": pid_stats}


def analyze_motors(data, sr):
    motors = []
    for i in range(4):
        key = f"motor{i}"
        if key in data:
            motors.append(data[key])
    if not motors:
        return None

    global_max = np.nanmax(np.array(motors))
    motor_min, motor_max = (1000, 2000) if global_max > 1500 else (0, int(global_max * 1.05)) if global_max > 100 else (0, 100)
    span = max(1, motor_max - motor_min)

    results = []
    for i, m in enumerate(motors):
        clean = m[~np.isnan(m)]
        if len(clean) == 0:
            continue
        pct = (clean - motor_min) / span * 100
        results.append({
            "motor": i + 1, "avg_pct": float(np.mean(pct)), "std_pct": float(np.std(pct)),
            "saturation_pct": float(np.sum(pct > 95) / len(pct) * 100),
            "min_pct": float(np.min(pct)), "normalized": pct, "raw": clean,
        })

    avgs = [r["avg_pct"] for r in results]
    spread = max(avgs) - min(avgs) if len(avgs) >= 2 else 0
    worst_motor, worst_direction = None, None
    if spread > MOTOR_IMBAL_OK and len(avgs) >= 4:
        mean_avg = np.mean(avgs)
        deviations = [abs(a - mean_avg) for a in avgs]
        worst_motor = int(np.argmax(deviations)) + 1
        worst_direction = "high" if avgs[worst_motor - 1] > mean_avg else "low"

    return {"n_motors": len(results), "motors": results, "balance_spread_pct": spread,
            "motor_range": (motor_min, motor_max),
            "worst_motor": worst_motor, "worst_direction": worst_direction}


def analyze_dterm_noise(data, sr):
    results = []
    for axis in AXIS_NAMES:
        d_key = f"axisD_{axis.lower()}"
        if d_key not in data:
            continue
        clean = data[d_key][~np.isnan(data[d_key])]
        if len(clean) < 256:
            continue
        freqs, psd_db = compute_psd(clean, sr)
        peaks = find_noise_peaks(freqs, psd_db, min_height_db=-40)
        results.append({"axis": axis, "freqs": freqs, "psd_db": psd_db,
                         "peaks": peaks, "rms": float(np.sqrt(np.mean(clean**2)))})
    return results


# ─── Prescriptive Recommendation Engine ──────────────────────────────────────

def compute_recommended_filter(noise_results, current_hz, filter_type="gyro", profile=None):
    noise_starts = [nr["noise_start_freq"] for nr in noise_results if nr is not None]
    if not noise_starts:
        return None
    if profile is None:
        profile = get_frame_profile(5)
    if filter_type == "gyro":
        min_cutoff, max_cutoff = profile["gyro_lpf_range"]
    else:
        min_cutoff, max_cutoff = profile["dterm_lpf_range"]
    ideal = min(noise_starts) * profile["filter_safety"]
    return round(int(np.clip(ideal, min_cutoff, max_cutoff)) / 5) * 5


def compute_recommended_pid(pid_result, current_p, current_i, current_d, profile=None):
    if pid_result is None:
        return None
    if profile is None:
        profile = get_frame_profile(5)
    axis = pid_result["axis"]
    changes, reasons = {}, []
    max_change = profile["pid_adjust_factor"]  # max fraction change per iteration
    ok_os = profile["ok_overshoot"]
    bad_os = profile["bad_overshoot"]
    ok_dl = profile["ok_delay_ms"]
    bad_dl = profile["bad_delay_ms"]

    os_pct = pid_result.get("avg_overshoot_pct") or 0.0
    delay = pid_result.get("tracking_delay_ms") or 0.0
    os_high = os_pct > ok_os
    os_bad = os_pct > bad_os
    dl_high = delay > ok_dl
    dl_bad = delay > bad_dl

    # ── Severity-proportional P adjustment ──
    # How far off from target determines size of correction.
    # Profile's max_change caps the adjustment for safety.
    if current_p is not None:
        new_p = current_p

        if os_high:
            # severity: 1.0 = at ok threshold, 2.0 = 2× the ok threshold, etc.
            severity = os_pct / ok_os
            if severity > 3.0:
                raw_cut = 0.35
            elif severity > 2.0:
                raw_cut = 0.25
            elif severity > 1.5:
                raw_cut = 0.18
            else:
                raw_cut = 0.10

            has_d = current_d is not None and current_d > 0
            d_hint = " and raise D" if has_d else ""
            if os_high and dl_high:
                # Conflict: overshoot says lower P, delay says raise P.
                # Overshoot wins, but be a bit less aggressive since delay is also a concern.
                raw_cut *= 0.7
                settle = f", higher D helps it settle" if has_d else ""
                reasons.append(
                    f"Overshoot is {os_pct:.0f}% (target: <{ok_os}%) and delay is {delay:.0f}ms. "
                    f"Fixing overshoot first — lower P reduces the bounce{settle}.")
            elif os_bad:
                reasons.append(
                    f"Overshoot is {os_pct:.0f}% (target: <{ok_os}%). "
                    f"The quad swings past the target angle. Reduce P{d_hint} to fix this.")
            else:
                reasons.append(
                    f"Overshoot is {os_pct:.0f}% (target: <{ok_os}%). "
                    f"A small P reduction{d_hint} should tighten this up.")

            # max_change limits P increases (delay fix), but overshoot reduction
            # can exceed it — the quad is ALREADY oscillating, we need meaningful correction.
            p_cut = min(raw_cut, max(max_change, 0.20))  # at least 20% reduction allowed
            new_p = int(current_p * (1 - p_cut))

        elif dl_high:
            severity = delay / ok_dl
            if severity > 2.0:
                raw_bump = 0.25
            elif severity > 1.5:
                raw_bump = 0.15
            else:
                raw_bump = 0.08

            if dl_bad:
                reasons.append(f"Response is sluggish ({delay:.0f}ms delay, target: <{ok_dl}ms). Needs more P.")
            else:
                reasons.append(f"Delay is {delay:.0f}ms (target: <{ok_dl}ms). A small P bump will help.")

            p_bump = min(raw_bump, max_change)
            new_p = int(current_p * (1 + p_bump))

        new_p = max(15, min(200, new_p))
        if new_p != current_p:
            changes["P"] = {"current": current_p, "new": new_p}

    # ── D adjustment: proportional to overshoot severity ──
    if current_d is not None and current_d > 0:
        new_d = current_d
        if os_high:
            severity = os_pct / ok_os
            if severity > 2.5:
                raw_bump = 0.30
            elif severity > 1.5:
                raw_bump = 0.20
            else:
                raw_bump = 0.12
            d_bump = min(raw_bump, max_change * 1.5)  # D can move a bit more than P
            new_d = int(current_d * (1 + d_bump))
        new_d = max(0, min(150, new_d))
        if new_d != current_d:
            changes["D"] = {"current": current_d, "new": new_d}

    # ── I adjustment ──
    if current_i is not None:
        new_i = current_i
        if "I" in pid_result["pid_stats"] and pid_result["pid_stats"]["I"]["rms"] > 80:
            new_i = int(current_i * 0.85)
            reasons.append(f"I-term working very hard (possible CG offset)")
        new_i = max(5, min(200, new_i))
        if new_i != current_i:
            changes["I"] = {"current": current_i, "new": new_i}

    return {"axis": axis, "changes": changes, "reasons": reasons}


def generate_action_plan(noise_results, pid_results, motor_analysis, dterm_results,
                          config, data, profile=None, phase_lag=None, motor_response=None,
                          rpm_range=None, prop_harmonics=None):
    actions = []
    if profile is None:
        profile = get_frame_profile(5)
    has_config = config_has_pid(config)
    has_filters = config_has_filters(config)
    ok_os = profile["ok_overshoot"]
    bad_os = profile["bad_overshoot"]
    ok_dl = profile["ok_delay_ms"]
    bad_dl = profile["bad_delay_ms"]
    ok_noise = profile["ok_noise_db"]
    bad_noise = profile["bad_noise_db"]

    # ═══ 0. PHASE LAG WARNING (large quad critical) ═══
    if phase_lag and phase_lag["total_degrees"] > 45:
        severity = "CRITICAL" if phase_lag["total_degrees"] > 70 else "IMPORTANT"
        prio = 1 if severity == "CRITICAL" else 2
        chain_desc = ", ".join(f'{c["name"]}={c["degrees"]:.0f}°' for c in phase_lag["chain"])
        actions.append({"priority": prio, "urgency": severity, "category": "Phase Lag",
            "action": f"Total filter phase lag: {phase_lag['total_degrees']:.0f}° ({phase_lag['total_ms']:.1f}ms) at {phase_lag['signal_freq']:.0f}Hz",
            "param": "filter_chain", "current": f"{phase_lag['total_degrees']:.0f}°", "new": "<45°",
            "reason": f"Filter chain: {chain_desc}. "
                       f"Excessive phase lag makes the PID controller fight yesterday's problem. "
                       f"Consider raising lowpass cutoffs or switching BIQUAD filters to PT1."})

    # ═══ 0b. MOTOR RESPONSE INFO (not an action — informational only) ═══
    info_items = []
    if motor_response and motor_response["motor_response_ms"] > profile["ok_delay_ms"]:
        mr_ms = motor_response["motor_response_ms"]
        info_items.append({
            "text": f"Motor response time: {mr_ms:.0f}ms",
            "detail": f"Motors physically cannot respond faster than {mr_ms:.0f}ms with these props. "
                      f"This sets a hard floor — tuning for tighter delay than this is futile."})

    # ═══ 1. FILTERS ═══
    current_gyro_lp = config.get("gyro_lowpass_hz") if has_filters else None
    rec_gyro_lp = compute_recommended_filter(noise_results, current_gyro_lp, "gyro", profile)

    if rec_gyro_lp is not None:
        worst_noise = max((nr["rms_high"] for nr in noise_results if nr), default=-80)
        if worst_noise > bad_noise:
            prio, urg = 1, "CRITICAL"
        elif worst_noise > ok_noise:
            prio, urg = 2, "IMPORTANT"
        else:
            prio, urg = 5, None

        if current_gyro_lp is not None and abs(rec_gyro_lp - current_gyro_lp) > 10:
            direction = "Reduce" if rec_gyro_lp < current_gyro_lp else "Increase"
            actions.append({"priority": prio, "urgency": urg, "category": "Filter",
                "action": f"Gyro lowpass filter: {direction} from {current_gyro_lp}Hz to {rec_gyro_lp}Hz",
                "param": "gyro_lowpass_hz", "current": current_gyro_lp, "new": rec_gyro_lp,
                "reason": f"High-freq noise at {worst_noise:.0f} dB avg — lower cutoff will reduce noise reaching the PID controller"})
        elif current_gyro_lp is None and worst_noise > ok_noise:
            actions.append({"priority": prio, "urgency": urg, "category": "Filter",
                "action": f"Set gyro_lowpass_hz to {rec_gyro_lp}",
                "param": "gyro_lowpass_hz", "current": "unknown", "new": rec_gyro_lp,
                "reason": f"Significant noise — use .bbl file for current value reading"})

    # D-term lowpass
    current_dterm_lp = config.get("dterm_lpf_hz") if has_filters else None
    if dterm_results:
        max_dterm_rms = max(d["rms"] for d in dterm_results)
        dterm_noise_starts = []
        for dt in dterm_results:
            nm = dt["psd_db"] > -35
            if np.any(nm):
                dterm_noise_starts.append(float(dt["freqs"][np.argmax(nm)]))
        if dterm_noise_starts and max_dterm_rms > 30:
            dt_min, dt_max = profile["dterm_lpf_range"]
            dt_safety = profile["filter_safety"] * 0.875  # D-term uses tighter margin
            rec_dterm_lp = round(int(np.clip(min(dterm_noise_starts) * dt_safety, dt_min, dt_max)) / 5) * 5
            if current_dterm_lp is not None and abs(rec_dterm_lp - current_dterm_lp) > 10:
                actions.append({"priority": 2, "urgency": "IMPORTANT", "category": "Filter",
                    "action": f"D-term lowpass filter: Reduce from {current_dterm_lp}Hz to {rec_dterm_lp}Hz",
                    "param": "dterm_lpf_hz", "current": current_dterm_lp, "new": rec_dterm_lp,
                    "reason": f"D-term amplifying noise (RMS={max_dterm_rms:.0f})"})
            elif current_dterm_lp is None:
                actions.append({"priority": 2, "urgency": "IMPORTANT", "category": "Filter",
                    "action": f"Set dterm_lpf_hz to {rec_dterm_lp}",
                    "param": "dterm_lpf_hz", "current": "unknown", "new": rec_dterm_lp,
                    "reason": f"D-term noise high — use .bbl for current value"})

    # Dynamic notch / RPM filter recommendations
    all_peaks = []
    for nr in noise_results:
        if nr:
            for p in nr["peaks"]:
                if p["power_db"] > -20 and 50 < p["freq_hz"] < 500:
                    all_peaks.append(p)
    if all_peaks:
        dom_freqs = sorted(set(int(round(p["freq_hz"]/10)*10) for p in all_peaks))
        lowest_peak = min(p["freq_hz"] for p in all_peaks)

        dyn_en = config.get("dyn_notch_enabled")
        dyn_q = config.get("dyn_notch_q")
        dyn_min_hz = config.get("dyn_notch_min_hz")
        rpm_en = config.get("rpm_filter_enabled")

        if dyn_en in (None, "0", 0, "OFF"):
            # Dynamic notch is truly off — recommend enabling
            rec_min_hz = max(30, int(round(lowest_peak * 0.7 / 10) * 10))
            actions.append({"priority": 2, "urgency": "IMPORTANT", "category": "Filter",
                "action": f"Enable dynamic notch filter (set dynamic_gyro_notch_enabled = ON, min_hz = {rec_min_hz})",
                "param": "dynamic_gyro_notch_enabled", "current": "OFF", "new": "ON",
                "reason": f"Noise peaks at: {', '.join(f'{f}Hz' for f in dom_freqs[:4])} — dynamic notch will track and attenuate these"})
        elif isinstance(dyn_min_hz, (int, float)) and dyn_min_hz > 0:
            # Dynamic notch is on — check if min_hz is too high to catch the lowest peaks
            if lowest_peak < dyn_min_hz * 1.1:
                rec_min_hz = max(30, int(round(lowest_peak * 0.7 / 10) * 10))
                actions.append({"priority": 3, "urgency": "RECOMMENDED", "category": "Filter",
                    "action": f"Lower dynamic_gyro_notch_min_hz: {int(dyn_min_hz)} → {rec_min_hz}",
                    "param": "dyn_notch_min_hz", "current": int(dyn_min_hz), "new": rec_min_hz,
                    "reason": f"Noise peak at {int(lowest_peak)}Hz is near/below current min_hz ({int(dyn_min_hz)}Hz) — notch can't track it"})

        # RPM filter recommendation (more effective than dynamic notch for motor noise)
        if rpm_en in (None, "0", 0, "OFF"):
            actions.append({"priority": 4, "urgency": "OPTIONAL", "category": "Filter",
                    "action": "Consider enabling RPM filter (set rpm_gyro_filter_enabled = ON)",
                    "param": "rpm_filter_enabled", "current": "OFF", "new": "ON",
                    "reason": "RPM filter tracks motor noise precisely — requires ESC telemetry wire connected to a UART"})

    # ═══ PID CHANGES (merged per axis) ═══
    for i, axis in enumerate(AXIS_NAMES):
        pid = pid_results[i] if i < len(pid_results) else None
        if pid is None:
            continue
        cur_p = config.get(f"{axis.lower()}_p")
        cur_i = config.get(f"{axis.lower()}_i")
        cur_d = config.get(f"{axis.lower()}_d")

        if has_config:
            rec = compute_recommended_pid(pid, cur_p, cur_i, cur_d, profile)
            if rec and rec["changes"]:
                _os = pid["avg_overshoot_pct"] or 0
                _dl = pid["tracking_delay_ms"] or 0
                prio = 3 if (_os > bad_os or _dl > bad_dl) else 4
                urg = "IMPORTANT" if prio == 3 else None

                # Build a single merged description for all PID changes on this axis
                change_parts = []
                sub_actions = []  # for CLI generation
                for term in ["P", "D", "I"]:
                    if term in rec["changes"]:
                        ch = rec["changes"][term]
                        direction = "Reduce" if ch["new"] < ch["current"] else "Increase"
                        change_parts.append(f"{direction} {term} from {ch['current']} to {ch['new']}")
                        sub_actions.append({"param": f"{axis.lower()}_{term.lower()}",
                                           "current": ch["current"], "new": ch["new"]})

                action_text = f"{axis}: {', '.join(change_parts)}"

                # Use reasons from compute_recommended_pid (already has good descriptions)
                actions.append({"priority": prio, "urgency": urg, "category": "PID",
                    "action": action_text,
                    "param": sub_actions[0]["param"] if len(sub_actions) == 1 else f"{axis.lower()}_pid",
                    "current": sub_actions[0]["current"] if len(sub_actions) == 1 else "multiple",
                    "new": sub_actions[0]["new"] if len(sub_actions) == 1 else "multiple",
                    "sub_actions": sub_actions,
                    "reason": " ".join(rec["reasons"])})
        else:
            _os = pid["avg_overshoot_pct"] or 0
            _dl = pid["tracking_delay_ms"] or 0
            if _os > ok_os:
                actions.append({"priority": 3, "urgency": "IMPORTANT" if _os > bad_os else None,
                    "category": "PID",
                    "action": f"Reduce {axis} P by ~{int(_os/3)}% and increase D by ~10%",
                    "param": f"{axis.lower()}_p", "current": "unknown", "new": "see action",
                    "reason": f"Overshoot {_os:.0f}% — use .bbl file for exact values"})
            if _dl > ok_dl:
                actions.append({"priority": 3, "urgency": "IMPORTANT" if _dl > bad_dl else None,
                    "category": "PID",
                    "action": f"Increase {axis} P by ~{int(_dl/2)}%",
                    "param": f"{axis.lower()}_p", "current": "unknown", "new": "see action",
                    "reason": f"Delay {_dl:.0f}ms — use .bbl for exact values"})

    # ═══ MOTOR / MECHANICAL ═══
    if motor_analysis:
        for m in motor_analysis["motors"]:
            if m["saturation_pct"] > profile["motor_sat_warn"]:
                actions.append({"priority": 1 if m["saturation_pct"] > 15 else 3,
                    "urgency": "CRITICAL" if m["saturation_pct"] > 15 else "IMPORTANT",
                    "category": "Motor",
                    "action": f"Motor {m['motor']} saturating {m['saturation_pct']:.1f}% — reduce overall PID gains by 15%",
                    "param": "motor_saturation", "current": f"{m['saturation_pct']:.1f}%", "new": f"<{profile['motor_sat_ok']}%",
                    "reason": "Motor at 100% can't correct further — PID demands exceed physics"})

        if motor_analysis["worst_motor"] and motor_analysis["balance_spread_pct"] > profile["motor_imbal_warn"]:
            wm, wd = motor_analysis["worst_motor"], motor_analysis["worst_direction"]
            sp = motor_analysis["balance_spread_pct"]
            actions.append({"priority": 2, "urgency": "IMPORTANT", "category": "Mechanical",
                "action": f"Check Motor {wm} — running {wd} ({sp:.1f}% imbalance)",
                "param": "motor_balance", "current": f"{sp:.1f}%", "new": f"<{profile['motor_imbal_ok']}%",
                "reason": "Bent prop, bad motor, loose mount, or CG offset. Fix mechanics before tuning PIDs."})
        elif motor_analysis["worst_motor"] and motor_analysis["balance_spread_pct"] > profile["motor_imbal_ok"]:
            wm = motor_analysis["worst_motor"]
            sp = motor_analysis["balance_spread_pct"]
            actions.append({"priority": 5, "urgency": None, "category": "Mechanical",
                "action": f"Minor imbalance ({sp:.1f}%) — Motor {wm} is the outlier",
                "param": "motor_balance", "current": f"{sp:.1f}%", "new": f"<{profile['motor_imbal_ok']}%",
                "reason": "Check prop balance and CG"})

    actions.sort(key=lambda a: a["priority"])

    # ═══ FILTER-FIRST ENFORCEMENT ═══
    # When filter changes are recommended, PID measurements are unreliable because
    # noise bleeds into the PID loop and inflates overshoot/delay readings.
    # Raising D with a wide-open dterm LPF amplifies noise and makes things worse.
    # Strategy: only output filter changes, tell user to re-fly for PID tuning.
    filter_actions = [a for a in actions if a.get("category") == "Filter"
                      and a.get("urgency") in ("CRITICAL", "IMPORTANT")]
    pid_actions = [a for a in actions if a.get("category") == "PID"]

    if filter_actions and pid_actions:
        # Mark PID actions as deferred — they'll show as informational, not actionable
        for a in pid_actions:
            a["deferred"] = True
            a["urgency"] = None
            a["original_action"] = a["action"]
            a["action"] = f"[DEFERRED] {a['action']}"
            a["reason"] = (
                "PID changes deferred until filters are fixed. Current overshoot/delay readings "
                "are inflated by noise passing through the filter stack. Fix filters first, "
                "re-fly, then the analyzer will give accurate PID recommendations.")

        # Add explicit guidance
        actions.append({
            "priority": 2, "urgency": "IMPORTANT", "category": "Workflow",
            "action": "Fix filters first, then re-fly for PID tuning",
            "param": "workflow", "current": "N/A", "new": "N/A",
            "reason": "Noise is corrupting PID measurements. Apply the filter changes above, "
                      "fly one pack, then run the analyzer again for accurate PID recommendations. "
                      "Do NOT change PIDs and filters at the same time."})
        actions.sort(key=lambda a: a["priority"])

    # ═══ QUALITY SCORE ═══
    good_noise = profile["good_noise_db"]
    good_os = profile["good_overshoot"]
    good_dl = profile["good_delay_ms"]

    noise_scores = []
    for nr in noise_results:
        if nr:
            s = np.clip((nr["rms_high"] - bad_noise) / (good_noise - bad_noise) * 100, 0, 100)
            noise_scores.append(s)
    pid_scores = []
    for pid in pid_results:
        if pid:
            _os = pid["avg_overshoot_pct"]
            _dl = pid["tracking_delay_ms"]
            components = []
            if _os is not None:
                os_s = 100 - np.clip((_os - good_os) / (bad_os - good_os) * 100, 0, 100)
                components.append(os_s)
            if _dl is not None:
                dl_s = 100 - np.clip((_dl - good_dl) / (bad_dl - good_dl) * 100, 0, 100)
                components.append(dl_s)
            if components:
                pid_scores.append(np.mean(components))
    motor_score = 100
    if motor_analysis:
        sat_s = [max(0, 100 - m["saturation_pct"] * 10) for m in motor_analysis["motors"]]
        bal_s = max(0, 100 - motor_analysis["balance_spread_pct"] * 8)
        motor_score = (np.mean(sat_s) + bal_s) / 2

    overall = float(np.mean([np.mean(noise_scores) if noise_scores else 50,
                              np.mean(pid_scores) if pid_scores else 50, motor_score]))
    scores = {"overall": overall,
              "noise": float(np.mean(noise_scores)) if noise_scores else None,
              "pid": float(np.mean(pid_scores)) if pid_scores else None,
              "motor": float(motor_score)}

    critical_actions = [a for a in actions if a["urgency"] in ("CRITICAL", "IMPORTANT")]
    if overall >= 85 and len(critical_actions) == 0:
        verdict, vtext = "DIALED_IN", "This tune is dialed in. You're done — go fly!"
    elif overall >= 75 and len(critical_actions) == 0:
        verdict, vtext = "NEARLY_THERE", "Almost there — just minor tweaks left."
    elif overall >= 60:
        verdict, vtext = "GETTING_BETTER", "Making progress. Apply the changes below and fly again."
    elif overall >= 40:
        verdict, vtext = "NEEDS_WORK", "Significant tuning needed. Start with the top priorities."
    else:
        verdict, vtext = "ROUGH", "Needs attention. Focus on filters first, then PIDs."

    return {"actions": actions, "info": info_items, "scores": scores, "verdict": verdict, "verdict_text": vtext}


# ─── Chart Generation ─────────────────────────────────────────────────────────

def fig_to_base64(fig, dpi=120):
    buf = BytesIO()
    fig.savefig(buf, format="png", dpi=dpi, bbox_inches="tight", facecolor=fig.get_facecolor(), edgecolor="none")
    plt.close(fig)
    buf.seek(0)
    return base64.b64encode(buf.read()).decode("ascii")

def setup_dark_style():
    plt.rcParams.update({
        "figure.facecolor": "#1a1b26", "axes.facecolor": "#1a1b26",
        "axes.edgecolor": "#565f89", "axes.labelcolor": "#c0caf5",
        "text.color": "#c0caf5", "xtick.color": "#565f89", "ytick.color": "#565f89",
        "grid.color": "#24283b", "grid.alpha": 0.6, "font.size": 10,
        "axes.titlesize": 13, "axes.grid": True,
    })

def create_noise_chart(noise_results):
    setup_dark_style()
    fig, axes = plt.subplots(1, 3, figsize=(16, 4.5), sharey=True)
    fig.suptitle("Gyro Noise Spectrum", fontsize=14, color="#c0caf5", fontweight="bold", y=1.02)
    for i, nr in enumerate(noise_results):
        if nr is None: continue
        ax = axes[i]
        ax.plot(nr["freqs"], nr["psd_db"], color=AXIS_COLORS[i], linewidth=1.2, alpha=0.9)
        ax.fill_between(nr["freqs"], nr["psd_db"], -80, alpha=0.15, color=AXIS_COLORS[i])
        for peak in nr["peaks"][:3]:
            ax.axvline(peak["freq_hz"], color="#ff9e64", alpha=0.6, linestyle="--", linewidth=0.8)
            ax.annotate(f'{peak["freq_hz"]:.0f}Hz', xy=(peak["freq_hz"], peak["power_db"]),
                        fontsize=8, color="#ff9e64", ha="center", va="bottom", xytext=(0,8), textcoords="offset points")
        ax.set_title(nr["axis"], color=AXIS_COLORS[i], fontweight="bold")
        ax.set_xlabel("Frequency (Hz)")
        if i == 0: ax.set_ylabel("Power (dB)")
        ax.set_xlim(0, min(1000, nr["freqs"][-1]))
    fig.tight_layout()
    return fig_to_base64(fig)

def create_pid_response_chart(pid_results, sr):
    setup_dark_style()
    valid = [p for p in pid_results if p is not None]
    if not valid: return None
    fig, axes = plt.subplots(len(valid), 1, figsize=(16, 3.5*len(valid)), sharex=True)
    if len(valid) == 1: axes = [axes]
    fig.suptitle("PID Response — Setpoint vs Gyro", fontsize=14, color="#c0caf5", fontweight="bold", y=1.01)
    ws = int(sr * 2)
    for i, pid in enumerate(valid):
        ax = axes[i]
        ai = AXIS_NAMES.index(pid["axis"])
        sp, gy = pid["setpoint"], pid["gyro"]
        bs, bv = 0, 0
        for s in range(0, len(sp)-ws, ws//4):
            v = np.var(sp[s:s+ws])
            if v > bv: bv, bs = v, s
        sl = slice(bs, bs+ws)
        t = np.arange(ws)/sr*1000
        sps = sp[sl] if sl.stop <= len(sp) else sp[:ws]
        gys = gy[sl] if sl.stop <= len(gy) else gy[:ws]
        n = min(len(sps), len(gys), len(t))
        ax.plot(t[:n], sps[:n], color="#565f89", linewidth=1.5, alpha=0.8, label="Setpoint")
        ax.plot(t[:n], gys[:n], color=AXIS_COLORS[ai], linewidth=1.2, alpha=0.9, label="Gyro")
        _os = pid["avg_overshoot_pct"]
        _dl = pid["tracking_delay_ms"]
        dl_str = f"{_dl:.1f}ms" if _dl is not None else "N/A"
        os_str = f"{_os:.1f}%" if _os is not None else "N/A"
        ax.set_title(f'{pid["axis"]} — Delay:{dl_str} | OS:{os_str}',
                      color=AXIS_COLORS[ai], fontweight="bold", fontsize=11)
        ax.set_ylabel("deg/s")
        ax.legend(loc="upper right", fontsize=8, facecolor="#1a1b26", edgecolor="#565f89")
    axes[-1].set_xlabel("Time (ms)")
    fig.tight_layout()
    return fig_to_base64(fig)

def create_motor_chart(motor_analysis, time_s):
    setup_dark_style()
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 6), gridspec_kw={"height_ratios": [2,1]})
    fig.suptitle("Motor Output", fontsize=14, color="#c0caf5", fontweight="bold", y=1.01)
    ns = len(motor_analysis["motors"][0]["raw"])
    w = min(ns, 5000)
    bs, bv = 0, 0
    for m in motor_analysis["motors"]:
        for s in range(0, len(m["raw"])-w, w//4):
            v = np.var(m["normalized"][s:s+w])
            if v > bv: bv, bs = v, s
    for i, m in enumerate(motor_analysis["motors"]):
        sl = slice(bs, bs+w)
        t = time_s[sl] if sl.stop <= len(time_s) else time_s[:w]
        vals = m["normalized"][sl] if sl.stop <= len(m["normalized"]) else m["normalized"][:w]
        ax1.plot(t[:len(vals)], vals[:len(t)], color=MOTOR_COLORS[i], linewidth=0.8, alpha=0.8, label=f'M{m["motor"]}')
    ax1.axhline(95, color="#f7768e", linestyle="--", alpha=0.5)
    ax1.set_ylabel("Output (%)"); ax1.legend(loc="upper right", fontsize=8, ncol=5, facecolor="#1a1b26", edgecolor="#565f89")
    ax1.set_ylim(0, 105)
    names = [f'M{m["motor"]}' for m in motor_analysis["motors"]]
    avgs = [m["avg_pct"] for m in motor_analysis["motors"]]
    bars = ax2.bar(names, avgs, color=MOTOR_COLORS[:len(names)], alpha=0.8, width=0.5)
    ax2.set_ylabel("Avg (%)"); ax2.set_title(f'Spread: {motor_analysis["balance_spread_pct"]:.1f}%', fontsize=10, color="#7aa2f7")
    for b, v in zip(bars, avgs): ax2.text(b.get_x()+b.get_width()/2, b.get_height()+1, f'{v:.1f}%', ha="center", fontsize=9, color="#c0caf5")
    fig.tight_layout()
    return fig_to_base64(fig)

def create_dterm_chart(dterm_results):
    setup_dark_style()
    if not dterm_results: return None
    fig, axes = plt.subplots(1, len(dterm_results), figsize=(16, 4), sharey=True)
    if len(dterm_results) == 1: axes = [axes]
    fig.suptitle("D-Term Noise", fontsize=14, color="#c0caf5", fontweight="bold", y=1.02)
    for i, dt in enumerate(dterm_results):
        ax = axes[i]
        ai = AXIS_NAMES.index(dt["axis"])
        ax.plot(dt["freqs"], dt["psd_db"], color=AXIS_COLORS[ai], linewidth=1.2, alpha=0.9)
        ax.fill_between(dt["freqs"], dt["psd_db"], -80, alpha=0.12, color=AXIS_COLORS[ai])
        ax.set_title(f'{dt["axis"]} (RMS:{dt["rms"]:.1f})', color=AXIS_COLORS[ai], fontweight="bold")
        ax.set_xlabel("Hz"); ax.set_xlim(0, min(500, dt["freqs"][-1]))
        if i == 0: ax.set_ylabel("dB")
    fig.tight_layout()
    return fig_to_base64(fig)


# ─── Narrative & CLI Commands ────────────────────────────────────────────────

# Map from action plan param names to INAV CLI setting names
INAV_CLI_MAP = {
    "roll_p": "mc_p_roll", "roll_i": "mc_i_roll", "roll_d": "mc_d_roll",
    "pitch_p": "mc_p_pitch", "pitch_i": "mc_i_pitch", "pitch_d": "mc_d_pitch",
    "yaw_p": "mc_p_yaw", "yaw_i": "mc_i_yaw", "yaw_d": "mc_d_yaw",
    "gyro_lowpass_hz": "gyro_main_lpf_hz", "gyro_lowpass2_hz": "gyro_main_lpf2_hz",
    "dterm_lpf_hz": "dterm_lpf_hz", "dterm_lpf2_hz": "dterm_lpf2_hz",
    "dyn_notch_min_hz": "dynamic_gyro_notch_min_hz",
    "dyn_notch_q": "dynamic_gyro_notch_q",
}

# Map for GUI (INAV Configurator) tab references
INAV_GUI_MAP = {
    "roll_p": ("PID Tuning", "Roll → P"),
    "roll_i": ("PID Tuning", "Roll → I"),
    "roll_d": ("PID Tuning", "Roll → D"),
    "pitch_p": ("PID Tuning", "Pitch → P"),
    "pitch_i": ("PID Tuning", "Pitch → I"),
    "pitch_d": ("PID Tuning", "Pitch → D"),
    "yaw_p": ("PID Tuning", "Yaw → P"),
    "yaw_i": ("PID Tuning", "Yaw → I"),
    "yaw_d": ("PID Tuning", "Yaw → D"),
    "gyro_lowpass_hz": ("Filtering", "Gyro LPF Hz"),
    "dterm_lpf_hz": ("Filtering", "D-term LPF Hz"),
}


def generate_cli_commands(actions):
    """Generate INAV CLI commands from action plan."""
    cmds = []

    for a in actions:
        # Skip deferred actions (e.g., PID changes when filters need fixing first)
        if a.get("deferred"):
            continue
        # Handle merged PID actions with sub_actions
        if "sub_actions" in a:
            for sa in a["sub_actions"]:
                param = sa.get("param", "")
                new_val = sa.get("new")
                if param in INAV_CLI_MAP and new_val is not None:
                    try:
                        cli_name = INAV_CLI_MAP[param]
                        cmds.append(f"set {cli_name} = {int(new_val)}")
                    except (ValueError, TypeError):
                        pass
        else:
            param = a.get("param", "")
            new_val = a.get("new")
            if param in INAV_CLI_MAP and new_val is not None and new_val not in ("see action",):
                try:
                    cli_name = INAV_CLI_MAP[param]
                    cmds.append(f"set {cli_name} = {int(new_val)}")
                except (ValueError, TypeError):
                    pass

    if cmds:
        cmds.append("save")
    return cmds


def generate_narrative(plan, pid_results, motor_analysis, noise_results, config, data, profile):
    """Generate a human-readable narrative about the quad's current state."""
    craft = config.get("craft_name", "your quad")
    fw = config.get("firmware_version", "")
    duration = data["time_s"][-1]
    sr = data["sample_rate"]
    overall = plan["scores"]["overall"]
    actions = plan["actions"]

    parts = []

    # Opening
    if overall >= 85:
        parts.append(f"{craft} is flying well. The tune is close to dialed in with a quality score of {overall:.0f}/100.")
    elif overall >= 60:
        parts.append(f"{craft} is flyable but has room for improvement (quality score: {overall:.0f}/100).")
    else:
        parts.append(f"{craft} needs significant tuning work (quality score: {overall:.0f}/100).")

    parts.append(f"This analysis is based on {duration:.0f} seconds of flight data at {sr:.0f}Hz.")

    # PID behavior per axis
    ok_os = profile["ok_overshoot"]
    bad_os = profile["bad_overshoot"]
    ok_dl = profile["ok_delay_ms"]
    bad_dl = profile["bad_delay_ms"]

    for pid in pid_results:
        if pid is None:
            continue
        axis = pid["axis"]
        os_pct = pid["avg_overshoot_pct"]
        delay = pid["tracking_delay_ms"]
        n_steps = pid.get("n_steps", 0)

        if os_pct is None and delay is None:
            parts.append(f"{axis} had insufficient stick activity to measure response ({n_steps} steps detected).")
            continue

        _os = os_pct or 0
        _dl = delay or 0

        if _os > bad_os and _dl > ok_dl:
            parts.append(
                f"{axis} has high overshoot ({_os:.0f}%) AND slow response ({_dl:.0f}ms). "
                f"This means the quad overshoots its target angle then takes too long to settle. "
                f"Lowering P and raising D will reduce the overshoot; the delay should improve once oscillation stops fighting itself.")
        elif _os > bad_os:
            parts.append(
                f"{axis} overshoot is very high at {_os:.0f}%. When you move the stick, "
                f"the quad swings past the target and bounces back. This is typically too much P gain. "
                f"Reducing P and increasing D will help it settle faster.")
        elif _os > ok_os:
            parts.append(
                f"{axis} has moderate overshoot ({_os:.0f}%). A small P reduction and D increase should tighten this up.")
        elif _dl > bad_dl:
            parts.append(
                f"{axis} response is sluggish ({_dl:.0f}ms delay). The quad takes too long to reach the commanded angle. "
                f"Increasing P gain will make it more responsive, but watch for overshoot increasing.")
        else:
            detail_parts = []
            if os_pct is not None: detail_parts.append(f"overshoot {_os:.0f}%")
            if delay is not None: detail_parts.append(f"delay {_dl:.0f}ms")
            parts.append(f"{axis} is tracking well ({', '.join(detail_parts)}).")

    # Noise
    noise_score = plan["scores"].get("noise")
    if noise_score is not None:
        if noise_score >= 90:
            parts.append("Gyro noise levels are clean — no major vibration issues detected.")
        elif noise_score >= 60:
            parts.append("There is moderate noise in the gyro signal. Lowering the gyro lowpass filter or enabling RPM filtering would help.")
        else:
            parts.append("The gyro signal is noisy — this often comes from propeller vibrations, loose mounting, or motor issues. "
                        "Addressing noise should be the top priority before fine-tuning PIDs.")

    # Motors
    if motor_analysis:
        spread = motor_analysis.get("balance_spread_pct", 0)
        if spread > profile["motor_imbal_warn"]:
            wm = motor_analysis["worst_motor"]
            parts.append(
                f"Motor {wm} is working significantly harder than the others ({spread:.0f}% imbalance). "
                f"This usually indicates a bent prop, bad motor bearing, loose mount, or CG offset. "
                f"Fix the mechanical issue before adjusting PIDs — no amount of software tuning can compensate for hardware problems.")
        elif spread > profile["motor_imbal_ok"]:
            wm = motor_analysis["worst_motor"]
            parts.append(f"There's a minor motor imbalance ({spread:.0f}%) on Motor {wm}. Worth checking prop balance and CG position.")

    return " ".join(parts)


# ─── Terminal Output ──────────────────────────────────────────────────────────

def print_terminal_report(plan, noise_results, pid_results, motor_analysis, config, data, show_narrative=True, profile=None):
    R, B, C, G, Y, RED, DIM = "\033[0m", "\033[1m", "\033[96m", "\033[92m", "\033[93m", "\033[91m", "\033[2m"
    scores = plan["scores"]
    overall = scores["overall"]

    print(f"\n{B}{C}{'═'*70}{R}")
    print(f"  {DIM}{datetime.now().strftime('%Y-%m-%d %H:%M')} | {data['time_s'][-1]:.1f}s | {data['sample_rate']:.0f}Hz | {data['n_rows']:,} rows{R}")
    if config.get("craft_name"): print(f"  {DIM}Craft: {config['craft_name']}{R}")

    sc = G if overall >= 85 else Y if overall >= 60 else RED
    print(f"\n  {B}TUNE QUALITY: {sc}{'█'*int(overall/5)}{'░'*(20-int(overall/5))} {overall:.0f}/100{R}")
    parts = []
    if scores["noise"] is not None: parts.append(f"Noise:{scores['noise']:.0f}")
    if scores["pid"] is not None: parts.append(f"PID:{scores['pid']:.0f}")
    parts.append(f"Motors:{scores['motor']:.0f}")
    print(f"  {DIM}  {' | '.join(parts)}{R}")

    vc = {"DIALED_IN": G, "NEARLY_THERE": G, "GETTING_BETTER": Y, "NEEDS_WORK": Y, "ROUGH": RED}
    print(f"\n  {B}{vc.get(plan['verdict'],C)}▸ {plan['verdict_text']}{R}")

    # Narrative (on by default)
    if show_narrative and profile:
        narrative = generate_narrative(plan, pid_results, motor_analysis, noise_results, config, data, profile)
        print(f"\n  {DIM}{'─'*68}{R}")
        import textwrap
        for line in textwrap.wrap(narrative, width=66):
            print(f"  {DIM}{line}{R}")

    if config_has_pid(config):
        print(f"\n  {B}CURRENT CONFIG:{R}")
        for ax in ["roll","pitch","yaw"]:
            print(f"    {ax.capitalize():6s} P={config.get(f'{ax}_p','?'):>3}  I={config.get(f'{ax}_i','?'):>3}  D={config.get(f'{ax}_d','?'):>3}")
        if config_has_filters(config):
            print(f"    Gyro LPF: {config.get('gyro_lowpass_hz','?')}Hz  D-term LPF: {config.get('dterm_lpf_hz','?')}Hz")

    # Informational items (not actions)
    info_items = plan.get("info", [])
    if info_items:
        print(f"\n{B}{C}{'─'*70}{R}")
        print(f"  {B}NOTE:{R}")
        for item in info_items:
            print(f"  {DIM}ℹ {item['text']}{R}")
            print(f"    {DIM}{item['detail']}{R}")

    actions = plan["actions"]
    active_actions = [a for a in actions if not a.get("deferred")]
    deferred_actions = [a for a in actions if a.get("deferred")]

    if active_actions:
        print(f"\n{B}{C}{'─'*70}{R}")
        print(f"  {B}DO THIS — {len(active_actions)} change{'s' if len(active_actions)!=1 else ''}:{R}")
        print(f"{B}{C}{'─'*70}{R}")
        for i, a in enumerate(active_actions, 1):
            us = ""
            if a["urgency"] == "CRITICAL": us = f" {RED}{B}[!!]{R}"
            elif a["urgency"] == "IMPORTANT": us = f" {Y}{B}[!]{R}"
            print(f"\n  {B}{C}{i}.{R} {B}{a['action']}{R}{us}")
            print(f"     {DIM}{a['reason']}{R}")

        # CLI commands (only from active actions)
        cli_cmds = generate_cli_commands(active_actions)
        if cli_cmds:
            print(f"\n{B}{C}{'─'*70}{R}")
            print(f"  {B}INAV CLI — paste into Configurator CLI tab:{R}")
            print()
            for cmd in cli_cmds:
                print(f"    {G}{cmd}{R}")
            print()
            # GUI hints from active actions (including sub_actions)
            gui_hints = []
            for a in active_actions:
                if "sub_actions" in a:
                    for sa in a["sub_actions"]:
                        param = sa.get("param", "")
                        if param in INAV_GUI_MAP:
                            tab, field = INAV_GUI_MAP[param]
                            gui_hints.append(f"• {tab} tab → {field} → {sa['new']}")
                else:
                    param = a.get("param", "")
                    if param in INAV_GUI_MAP:
                        tab, field = INAV_GUI_MAP[param]
                        new_val = a.get("new")
                        if new_val is not None and new_val not in ("see action",):
                            gui_hints.append(f"• {tab} tab → {field} → {new_val}")
            if gui_hints:
                print(f"  {DIM}Or apply manually in Configurator:{R}")
                for h in gui_hints:
                    print(f"    {DIM}{h}{R}")

    if deferred_actions:
        print(f"\n{B}{C}{'─'*70}{R}")
        print(f"  {B}{Y}DEFERRED (apply after filter changes + re-fly):{R}")
        print(f"{B}{C}{'─'*70}{R}")
        for a in deferred_actions:
            orig = a.get("original_action", a["action"])
            print(f"    {DIM}⏸ {orig}{R}")
        print(f"\n  {DIM}These PID changes are on hold because noise is distorting the")
        print(f"  measurements. Fix filters, fly one pack, then re-analyze.{R}")

    if not active_actions and not deferred_actions:
        print(f"\n  {G}{B}  ✓ No changes needed — go fly!{R}")

    print(f"\n{B}{C}{'─'*70}{R}")
    print(f"  {B}MEASUREMENTS:{R}")
    for pid in pid_results:
        if pid is None: continue
        _os = pid["avg_overshoot_pct"]
        _dl = pid["tracking_delay_ms"]
        n_steps = pid.get("n_steps", 0)
        if _os is not None:
            oc = RED if _os>BAD_OVERSHOOT else Y if _os>OK_OVERSHOOT else G
            os_str = f"{oc}{_os:5.1f}%{R}"
        else:
            os_str = f"{DIM}  N/A{R}"
        if _dl is not None:
            dc = RED if _dl>BAD_DELAY_MS else Y if _dl>OK_DELAY_MS else G
            dl_str = f"{dc}{_dl:5.1f}ms{R}"
        else:
            dl_str = f"{DIM}  N/A{R}"
        step_hint = f"  {DIM}({n_steps} steps){R}" if n_steps < 5 and (_os is None or _dl is None) else ""
        print(f"    {pid['axis']:6s}  OS:{os_str}  Delay:{dl_str}  Err:{pid['rms_error']:.1f}{step_hint}")
    if motor_analysis:
        for m in motor_analysis["motors"]:
            sc2 = RED if m["saturation_pct"]>MOTOR_SAT_WARN else G
            print(f"    Motor {m['motor']}  Avg:{m['avg_pct']:5.1f}%  Sat:{sc2}{m['saturation_pct']:4.1f}%{R}")
    print(f"\n{B}{C}{'═'*70}{R}\n")


# ─── HTML Report ──────────────────────────────────────────────────────────────

def generate_html_report(plan, noise_results, pid_results, motor_analysis, dterm_results, config, data, charts):
    scores = plan["scores"]
    overall = scores["overall"]
    sg = "linear-gradient(90deg, #9ece6a, #73daca)" if overall >= 85 else "linear-gradient(90deg, #e0af68, #ff9e64)" if overall >= 60 else "linear-gradient(90deg, #f7768e, #ff6b6b)"

    ah = ""
    active_actions = [a for a in plan["actions"] if not a.get("deferred")]
    deferred_actions = [a for a in plan["actions"] if a.get("deferred")]

    for i, a in enumerate(active_actions, 1):
        uc = (a["urgency"] or "normal").lower()
        ub = f'<span class="ub {uc}">{a["urgency"]}</span>' if a["urgency"] else ""
        ah += f'<div class="ac {uc}"><div class="an">{i}</div><div class="ab"><div class="am">{a["action"]} {ub}</div><div class="ar">{a["reason"]}</div></div></div>'
    if deferred_actions:
        ah += '<div class="ac deferred"><div class="ab"><div class="am" style="color:#565f89">⏸ Deferred PID changes (fix filters first, then re-fly)</div>'
        for a in deferred_actions:
            orig = a.get("original_action", a["action"].replace("[DEFERRED] ", ""))
            ah += f'<div class="ar" style="color:#565f89">{orig}</div>'
        ah += '</div></div>'
    if not active_actions and not deferred_actions:
        ah = '<div class="ac ok"><div class="ab"><div class="am">✓ No changes needed, go fly!</div></div></div>'

    ch = ""
    if config_has_pid(config) or config_has_filters(config):
        cr = ""
        for ax in ["roll","pitch","yaw"]:
            cr += f'<tr><td class="ax-{ax}">{ax.capitalize()}</td><td>{config.get(f"{ax}_p","—")}</td><td>{config.get(f"{ax}_i","—")}</td><td>{config.get(f"{ax}_d","—")}</td></tr>'
        fi = ""
        for k, l in [("gyro_lowpass_hz","Gyro LPF"),("dterm_lpf_hz","D-term LPF"),("yaw_lpf_hz","Yaw LPF"),("gyro_lowpass2_hz","Gyro LPF2")]:
            v = config.get(k)
            if v is not None and v != 0: fi += f'<span class="fc">{l}: {v}Hz</span>'
        # Dynamic notch status
        dyn_en = config.get("dyn_notch_enabled")
        dyn_q = config.get("dyn_notch_q")
        dyn_min = config.get("dyn_notch_min_hz")
        if dyn_en and dyn_en not in (0, "0", "OFF"):
            dyn_detail = f"ON (Q={dyn_q}, min={dyn_min}Hz)" if dyn_q else "ON"
            fi += f'<span class="fc">Dyn Notch: {dyn_detail}</span>'
        elif dyn_en is not None:
            fi += f'<span class="fc">Dyn Notch: OFF</span>'
        # RPM filter status
        rpm_en = config.get("rpm_filter_enabled")
        if rpm_en is not None:
            rpm_str = "ON" if rpm_en and rpm_en not in (0, "0", "OFF") else "OFF"
            fi += f'<span class="fc">RPM Filter: {rpm_str}</span>'
        ch = f'<section><h2>Current Config</h2><div class="cg"><div class="cc"><h3>PID</h3><table><tr><th>Axis</th><th>P</th><th>I</th><th>D</th></tr>{cr}</table></div><div class="cc"><h3>Filters</h3><div class="fcs">{fi or "<span class=fc>No filter values in headers</span>"}</div></div></div></section>'

    pt = ""
    for pid in pid_results:
        if pid is None: continue
        _os = pid["avg_overshoot_pct"]
        _dl = pid["tracking_delay_ms"]
        oc = ("bad" if _os>BAD_OVERSHOOT else "warn" if _os>OK_OVERSHOOT else "good") if _os is not None else "good"
        dc = ("bad" if _dl>BAD_DELAY_MS else "warn" if _dl>OK_DELAY_MS else "good") if _dl is not None else "good"
        dl_str = f"{_dl:.1f}ms" if _dl is not None else "N/A"
        os_str = f"{_os:.1f}%" if _os is not None else "N/A"
        pt += f'<tr><td class="ax-{pid["axis"].lower()}">{pid["axis"]}</td><td class="{dc}">{dl_str}</td><td class="{oc}">{os_str}</td><td>{pid["rms_error"]:.1f}</td></tr>'

    mt = ""
    if motor_analysis:
        for m in motor_analysis["motors"]:
            sc = "bad" if m["saturation_pct"]>MOTOR_SAT_WARN else "good"
            mt += f'<tr><td>M{m["motor"]}</td><td>{m["avg_pct"]:.1f}%</td><td>{m["std_pct"]:.1f}%</td><td class="{sc}">{m["saturation_pct"]:.1f}%</td></tr>'

    ci = lambda k: f'<img src="data:image/png;base64,{charts[k]}">' if charts.get(k) else ""
    vc = plan["verdict"].lower().replace("_","-")

    return f"""<!DOCTYPE html><html><head><meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>INAV Tuning Report</title><style>
:root{{--bg:#0f1117;--cd:#1a1b26;--ca:#1e2030;--bd:#2a2d3e;--tx:#c0caf5;--dm:#7982a9;--bl:#7aa2f7;--gn:#9ece6a;--rd:#f7768e;--yl:#e0af68;--tl:#4ecdc4;--pp:#bb9af7;--or:#ff9e64}}
*{{box-sizing:border-box;margin:0;padding:0}}body{{font-family:'SF Mono','Cascadia Code','JetBrains Mono',monospace;background:var(--bg);color:var(--tx);line-height:1.6}}
.ct{{max-width:1200px;margin:0 auto;padding:24px}}header{{background:linear-gradient(135deg,#1a1b26,#24283b);border-bottom:2px solid var(--bl);padding:24px 0;text-align:center}}
header h1{{font-size:1.5rem;letter-spacing:3px;text-transform:uppercase;color:var(--bl)}}.mt{{color:var(--dm);font-size:.8rem;margin-top:8px}}
.ss{{text-align:center;padding:32px 0}}.sb{{width:300px;height:24px;background:var(--cd);border-radius:12px;margin:12px auto;overflow:hidden;border:1px solid var(--bd)}}
.sf{{height:100%;background:{sg};border-radius:12px;width:{overall:.0f}%}}.sv{{font-size:2.5rem;font-weight:700;color:var(--bl)}}.sl{{font-size:.75rem;color:var(--dm);text-transform:uppercase;letter-spacing:2px}}
.sd{{display:flex;justify-content:center;gap:32px;margin-top:12px;font-size:.8rem;color:var(--dm)}}
.vd{{padding:16px 24px;border-radius:8px;font-weight:600;text-align:center;margin:16px auto;max-width:600px}}
.vd.dialed-in{{background:rgba(158,206,106,.12);color:var(--gn);border:1px solid rgba(158,206,106,.3)}}
.vd.nearly-there{{background:rgba(158,206,106,.08);color:var(--gn);border:1px solid rgba(158,206,106,.2)}}
.vd.getting-better{{background:rgba(224,175,104,.1);color:var(--yl);border:1px solid rgba(224,175,104,.2)}}
.vd.needs-work{{background:rgba(224,175,104,.1);color:var(--or);border:1px solid rgba(224,175,104,.2)}}
.vd.rough{{background:rgba(247,118,142,.1);color:var(--rd);border:1px solid rgba(247,118,142,.2)}}
section{{margin:32px 0}}h2{{font-size:1rem;color:var(--bl);letter-spacing:2px;text-transform:uppercase;border-bottom:1px solid var(--bd);padding-bottom:8px;margin-bottom:16px}}
h3{{font-size:.85rem;color:var(--pp);margin-bottom:8px}}.cc{{background:var(--cd);border:1px solid var(--bd);border-radius:8px;padding:16px;margin:12px 0;overflow-x:auto}}
.cc img{{width:100%;height:auto;display:block;border-radius:4px}}
.ac{{display:flex;gap:16px;background:var(--cd);border:1px solid var(--bd);border-left:4px solid var(--bd);border-radius:6px;padding:16px 20px;margin:10px 0;align-items:flex-start}}
.ac.critical{{border-left-color:var(--rd);background:rgba(247,118,142,.04)}}.ac.important{{border-left-color:var(--yl)}}.ac.normal{{border-left-color:var(--bl)}}
.ac.ok{{border-left-color:var(--gn);background:rgba(158,206,106,.06)}}.an{{font-size:1.4rem;font-weight:700;color:var(--bl);min-width:28px}}
.am{{font-weight:600;font-size:.95rem}}.ar{{color:var(--dm);font-size:.8rem;margin-top:4px}}
.ub{{font-size:.65rem;padding:1px 6px;border-radius:3px;font-weight:700;letter-spacing:1px;vertical-align:middle;margin-left:8px}}
.ub.critical{{background:rgba(247,118,142,.2);color:var(--rd)}}.ub.important{{background:rgba(224,175,104,.2);color:var(--yl)}}
table{{width:100%;border-collapse:collapse;margin:8px 0}}th{{background:var(--ca);color:var(--bl);font-size:.7rem;letter-spacing:1px;text-transform:uppercase;padding:8px 14px;text-align:left;border-bottom:1px solid var(--bd)}}
td{{padding:7px 14px;border-bottom:1px solid var(--bd);font-size:.85rem}}.good{{color:var(--gn);font-weight:600}}.warn{{color:var(--yl);font-weight:600}}.bad{{color:var(--rd);font-weight:600}}
.ax-roll{{color:#ff6b6b;font-weight:700}}.ax-pitch{{color:#4ecdc4;font-weight:700}}.ax-yaw{{color:#ffd93d;font-weight:700}}
.cg{{display:grid;grid-template-columns:1fr 1fr;gap:12px}}.fcs{{display:flex;flex-wrap:wrap;gap:8px}}.fc{{background:var(--ca);border:1px solid var(--bd);border-radius:4px;padding:4px 10px;font-size:.8rem}}
footer{{text-align:center;color:var(--dm);font-size:.7rem;padding:24px 0;border-top:1px solid var(--bd);margin-top:40px}}
</style></head><body>
<header><h1>▲ INAV Tuning Report</h1><div class="mt">{config.get('craft_name','')} {'| ' if config.get('craft_name') else ''}{config.get('firmware_type','INAV')} {config.get('firmware_version','')} | {data['time_s'][-1]:.1f}s | {data['sample_rate']:.0f}Hz | {datetime.now().strftime('%Y-%m-%d %H:%M')}</div></header>
<div class="ct">
<div class="ss"><div class="sl">Tune Quality</div><div class="sv">{overall:.0f}</div><div class="sb"><div class="sf"></div></div>
<div class="sd">{'<span>Noise:'+f'{scores["noise"]:.0f}'+'</span>' if scores["noise"] is not None else ''}{'<span>PID:'+f'{scores["pid"]:.0f}'+'</span>' if scores["pid"] is not None else ''}<span>Motors:{scores['motor']:.0f}</span></div></div>
<div class="vd {vc}">▸ {plan['verdict_text']}</div>
<section><h2>Do This</h2>{ah}</section>
{ch}
<section><h2>PID Response</h2><div class="cc"><table><tr><th>Axis</th><th>Delay</th><th>Overshoot</th><th>RMS Error</th></tr>{pt}</table></div><div class="cc">{ci("pid")}</div></section>
<section><h2>Noise</h2><div class="cc">{ci("noise")}</div></section>
{'<section><h2>D-Term Noise</h2><div class="cc">'+ci("dterm")+'</div></section>' if charts.get("dterm") else ''}
{'<section><h2>Motors</h2><div class="cc"><table><tr><th>Motor</th><th>Avg</th><th>StdDev</th><th>Saturation</th></tr>'+mt+'</table></div><div class="cc">'+ci("motor")+'</div></section>' if motor_analysis else ''}
</div><footer>INAV Blackbox Analyzer v{REPORT_VERSION} — Test changes with props off first</footer></body></html>"""


# ─── State file ───────────────────────────────────────────────────────────────

def save_state(filepath, plan, config, data):
    active = [a for a in plan["actions"] if not a.get("deferred")]
    deferred = [a for a in plan["actions"] if a.get("deferred")]
    state = {"version": REPORT_VERSION, "timestamp": datetime.now().isoformat(),
             "scores": plan["scores"], "verdict": plan["verdict"],
             "actions": [{"action": a["action"], "param": a["param"],
                           "current": str(a["current"]), "new": str(a["new"])} for a in active],
             "deferred_actions": [{"action": a.get("original_action", a["action"]),
                                    "param": a["param"],
                                    "current": str(a["current"]), "new": str(a["new"]),
                                    "reason": "Fix filters first, then re-fly"} for a in deferred],
             "config": {k: v for k, v in config.items() if k != "raw" and not isinstance(v, np.ndarray)}}
    sp = filepath.rsplit(".", 1)[0] + "_state.json"
    with open(sp, "w") as f:
        json.dump(state, f, indent=2, default=str)
    return sp


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="INAV Blackbox Analyzer v2.1 — Prescriptive Tuning",
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("logfile", nargs="?", default=None,
                        help="Blackbox log (.bbl/.bfl/.bbs/.txt or decoded .csv). "
                             "Optional when --device is used.")
    parser.add_argument("-o", "--output", help="HTML report filename")
    parser.add_argument("--no-html", action="store_true")
    parser.add_argument("--no-terminal", action="store_true")
    parser.add_argument("--previous", help="Previous state .json for comparison")
    # Device communication
    parser.add_argument("--device", metavar="PORT",
                        help="Download blackbox from FC via serial. Use 'auto' to scan "
                             "or specify port (e.g., /dev/ttyACM0).")
    parser.add_argument("--erase", action="store_true",
                        help="Erase dataflash after successful download.")
    parser.add_argument("--download-only", action="store_true",
                        help="Download blackbox from device but don't analyze.")
    parser.add_argument("--blackbox-dir", default="./blackbox",
                        help="Directory to save downloaded logs (default: ./blackbox).")
    # Frame and prop profile arguments
    parser.add_argument("--frame", type=int, metavar="INCHES",
                        help="Frame size in inches (3-15). Determines PID response thresholds. "
                             "Default: same as --props, or 5 if neither specified.")
    parser.add_argument("--props", type=int, metavar="INCHES",
                        help="Prop diameter in inches (3-15). Determines filter ranges and "
                             "noise frequency predictions. Default: same as --frame, or 5.")
    parser.add_argument("--blades", type=int, metavar="N", default=3,
                        help="Number of prop blades (2, 3, 4). Affects harmonic frequency "
                             "prediction. Default: 3 (triblade, most common for racing/freestyle).")
    parser.add_argument("--cells", type=int, metavar="S",
                        help="Battery cell count (e.g., 4 for 4S, 6 for 6S). "
                             "Used with --kv to predict noise frequencies.")
    parser.add_argument("--kv", type=int, metavar="KV",
                        help="Motor KV rating (e.g., 2400, 1750, 900). "
                             "Used with --cells to predict where prop noise will be.")
    parser.add_argument("--no-narrative", action="store_true",
                        help="Omit the plain-English description of the quad's behavior.")
    args = parser.parse_args()

    # ── Device mode: download blackbox from FC ──
    logfile = args.logfile
    if args.device:
        try:
            from inav_msp import INAVDevice, auto_detect_fc, find_serial_ports
        except ImportError:
            print("  ERROR: inav_msp.py module not found.")
            print("    Make sure inav_msp.py is in the same directory as the analyzer.")
            sys.exit(1)

        print(f"\n  ▲ INAV Blackbox Analyzer v{REPORT_VERSION}")

        if args.device == "auto":
            print("  Scanning for INAV flight controller...")
            port, info = auto_detect_fc()
            if not port:
                print("  ERROR: No INAV flight controller found.")
                ports = find_serial_ports()
                if ports:
                    print(f"    Ports found but none responded as INAV: {', '.join(ports)}")
                    print("    Make sure the FC is powered and not in DFU mode.")
                else:
                    print("    No serial ports detected. Is the FC connected via USB?")
                sys.exit(1)
            print(f"  Found: {port}")
        else:
            port = args.device
            if not os.path.exists(port):
                print(f"  ERROR: Port not found: {port}")
                sys.exit(1)
            print(f"  Connecting: {port}")

        try:
            with INAVDevice(port) as fc:
                info = fc.get_info()
                if not info:
                    print("  ERROR: No response from FC. Check connection and baud rate.")
                    sys.exit(1)

                if info.get("fc_variant") != "INAV":
                    print(f"  ERROR: Not an INAV FC (got: {info.get('fc_variant', '?')})")
                    sys.exit(1)

                print(f"  Connected: {info['craft_name'] or '(unnamed)'} — {info['firmware']}")

                summary = fc.get_dataflash_summary()
                if not summary or not summary["supported"]:
                    print("  ERROR: Dataflash not available on this board.")
                    sys.exit(1)

                used_kb = summary['used_size'] / 1024
                total_kb = summary['total_size'] / 1024
                pct = summary['used_size'] * 100 // summary['total_size'] if summary['total_size'] > 0 else 0
                print(f"  Dataflash: {used_kb:.0f}KB / {total_kb:.0f}KB ({pct}% used)")

                if summary['used_size'] == 0:
                    print("  No blackbox data to download.")
                    sys.exit(0)

                print()
                filepath = fc.download_blackbox(
                    output_dir=args.blackbox_dir,
                    erase_after=args.erase,
                )

                if not filepath:
                    print("  ERROR: Download failed.")
                    sys.exit(1)

                if args.download_only:
                    print(f"\n  To analyze:\n    python3 {sys.argv[0]} {filepath}")
                    sys.exit(0)

                logfile = filepath
                print()  # visual separator before analysis

        except KeyboardInterrupt:
            print("\n  Interrupted.")
            sys.exit(1)
        except Exception as e:
            print(f"  ERROR: {e}")
            sys.exit(1)

    if not logfile:
        parser.error("logfile is required when --device is not specified")

    if not os.path.isfile(logfile):
        print(f"ERROR: File not found: {logfile}"); sys.exit(1)

    # ── Parse headers FIRST (needed for auto-detection) ──
    raw_params = {}
    ext = os.path.splitext(logfile)[1].lower()

    is_blackbox = ext in (".bbl", ".bfl", ".bbs")
    if ext in (".txt", ".TXT") and not is_blackbox:
        try:
            with open(logfile, "rb") as f:
                first_line = f.readline().decode("utf-8", errors="ignore").strip()
                if first_line.startswith("H Product:Blackbox") or first_line.startswith("H Field I name:"):
                    is_blackbox = True
        except:
            pass

    if is_blackbox:
        raw_params = parse_headers_from_bbl(logfile)
    else:
        try:
            with open(logfile, "r", errors="ignore") as f:
                for line in f:
                    if line.strip().startswith("H "):
                        raw_params = parse_headers_from_bbl(logfile)
                        break
                    elif line.strip() and not line.strip().startswith("#"):
                        break
        except:
            pass

    config = extract_fc_config(raw_params)

    # ── Auto-detect platform from field names ──
    field_names_str = raw_params.get("Field I name", "")
    if field_names_str:
        all_fields = [f.strip().lower() for f in field_names_str.split(",")]
        motor_fields = [f for f in all_fields if re.match(r"motor\[\d+\]", f)]
        servo_fields = [f for f in all_fields if re.match(r"servo\[\d+\]", f)]
        n_motors = len(motor_fields)
        n_servos = len(servo_fields)
    else:
        n_motors = 4  # assume quad
        n_servos = 0

    # Determine platform type
    if n_motors == 3 or (n_motors == 4 and n_servos >= 1):
        platform_type = "Tricopter"
    elif n_motors == 4 and n_servos == 0:
        platform_type = "Quadcopter"
    elif n_motors == 6:
        platform_type = "Hexacopter"
    elif n_motors == 8:
        platform_type = "Octocopter"
    else:
        platform_type = f"{n_motors}-motor"

    # ── Auto-detect frame size from craft_name ──
    craft = config.get("craft_name", "")
    detected_frame = None
    if craft:
        # Look for number that could be frame/prop size (e.g., "NAZGUL 10", "Mark4 7", "Source One V5")
        # Match patterns like "10", "7", "5" that are likely inches, not version numbers
        m = re.search(r'\b(\d{1,2})(?:\s*(?:inch|in|"|' + r"'" + r'))?(?:\s|$)', craft, re.I)
        if m:
            candidate = int(m.group(1))
            if 3 <= candidate <= 15:  # plausible frame size range
                detected_frame = candidate

    frame_inches = args.frame
    prop_inches = args.props
    n_blades = args.blades

    # If only one of frame/props specified, use it for both
    if frame_inches and not prop_inches:
        prop_inches = frame_inches
    elif prop_inches and not frame_inches:
        frame_inches = prop_inches

    # Auto-detect logic
    frame_source = "user"
    if frame_inches is None and detected_frame is not None:
        frame_inches = detected_frame
        prop_inches = prop_inches or detected_frame
        frame_source = "auto"
    elif frame_inches is not None and detected_frame is not None and frame_inches != detected_frame:
        frame_source = "conflict"

    profile = get_frame_profile(frame_inches, prop_inches, n_blades)

    # ── Estimate battery from vbatref ──
    vbatref = raw_params.get("vbatref", "")
    detected_cells = args.cells
    if not detected_cells and vbatref:
        try:
            vref_v = int(vbatref) / 100.0
            if vref_v > 0:
                detected_cells = round(vref_v / 4.2)
                if detected_cells < 1 or detected_cells > 14:
                    detected_cells = None
        except:
            pass

    # ── Show comprehensive banner ──
    print(f"\n  ▲ INAV Blackbox Analyzer v{REPORT_VERSION}")
    print(f"  Loading: {logfile}")

    fw_rev = config.get("firmware_revision", "")
    fw_date = config.get("firmware_date", "")

    # Aircraft identification banner
    print(f"\n  {'─'*66}")
    if craft:
        print(f"  Aircraft:  {craft}")
    if fw_rev:
        fw_str = fw_rev
        if fw_date:
            fw_str += f" ({fw_date})"
        print(f"  Firmware:  {fw_str}")
    print(f"  Platform:  {platform_type} ({n_motors} motors{f', {n_servos} servos' if n_servos else ''})")

    frame_str = f"{frame_inches}\"" if frame_inches else "5\" (default)"
    prop_str = f"{prop_inches}\"×{n_blades}-blade" if prop_inches else f"5\"×{n_blades}-blade (default)"
    if frame_source == "auto":
        frame_str += f" (detected from craft name)"
    elif frame_source == "conflict":
        frame_str += f" (user override)"
    print(f"  Frame:     {frame_str}")
    print(f"  Props:     {prop_str}")

    if detected_cells:
        cell_str = f"{detected_cells}S"
        if not args.cells and vbatref:
            cell_str += f" (detected from vbatref={int(vbatref)/100:.1f}V)"
        print(f"  Battery:   {cell_str}")

    if args.kv:
        print(f"  Motors:    {args.kv}KV")

    # Profile and thresholds
    print(f"  Profile:   {profile['name']} ({profile['class']} class)")
    if (frame_inches or 5) >= 8:
        print(f"    Delay: <{profile['ok_delay_ms']}ms | OS: <{profile['ok_overshoot']}% | "
              f"Filters: {profile['gyro_lpf_range'][0]}-{profile['gyro_lpf_range'][1]}Hz")

    # Warnings
    if frame_source == "conflict":
        print(f"\n  ⚠ FRAME SIZE CONFLICT: Craft name \"{craft}\" suggests {detected_frame}\" "
              f"but --frame {args.frame} was specified.")
        print(f"    Using {args.frame}\" as requested. If this is wrong, the analyzer will use "
              f"incorrect thresholds.")
    elif frame_source != "auto" and frame_inches is None:
        if detected_frame is None and craft:
            print(f"\n  ⚠ Could not detect frame size from craft name \"{craft}\".")
            print(f"    Using 5\" defaults. Specify --frame N for accurate thresholds.")
        elif not craft:
            print(f"\n  ⚠ No craft name in log headers. Using 5\" defaults.")
            print(f"    Specify --frame N for accurate thresholds.")

    print(f"  {'─'*66}")

    # ── Decode data ──
    if is_blackbox:
        print(f"\n  Parsing {len(raw_params)} header parameters...")
        print("  Decoding binary log (native)...")
        data = decode_blackbox_native(logfile, raw_params)
    else:
        print("\n  Parsing CSV...")
        data = parse_csv_log(logfile)

    sr = data["sample_rate"]
    print(f"  {data['n_rows']:,} rows | {sr:.0f}Hz | {data['time_s'][-1]:.1f}s")

    if config_has_pid(config):
        for ax in ["roll","pitch","yaw"]:
            ff = config.get(f'{ax}_ff', '')
            ff_str = f" FF={ff}" if ff else ""
            print(f"  {ax.capitalize()} PID: P={config.get(f'{ax}_p','?')} I={config.get(f'{ax}_i','?')} D={config.get(f'{ax}_d','?')}{ff_str}")
    else:
        print("  ⚠ No PID values in headers — use .bbl for exact recommendations")

    if config_has_filters(config):
        filt_parts = []
        for k, l in [("gyro_lowpass_hz","Gyro LPF"),("dterm_lpf_hz","D-term LPF"),("yaw_lpf_hz","Yaw LPF")]:
            v = config.get(k)
            if v is not None and v != 0:
                filt_parts.append(f"{l}={v}Hz")
        dyn_en = config.get("dyn_notch_enabled")
        if dyn_en and dyn_en not in (0, "0", "OFF"):
            dyn_min = config.get("dyn_notch_min_hz", "?")
            dyn_q = config.get("dyn_notch_q", "?")
            filt_parts.append(f"DynNotch=ON(min={dyn_min}Hz,Q={dyn_q})")
        rpm_en = config.get("rpm_filter_enabled")
        if rpm_en and rpm_en not in (0, "0", "OFF"):
            filt_parts.append("RPM=ON")
        elif rpm_en is not None:
            filt_parts.append("RPM=OFF")
        if filt_parts:
            print(f"  Filters: {', '.join(filt_parts)}")

    # ── RPM prediction ──
    rpm_range = estimate_rpm_range(args.kv, detected_cells or args.cells)
    prop_harmonics = None
    if rpm_range:
        cells_used = detected_cells or args.cells
        print(f"  RPM estimate: {rpm_range[0]:,}–{rpm_range[1]:,} ({args.kv}KV × {cells_used}S)")
        prop_harmonics = estimate_prop_harmonics(rpm_range, n_blades)
        for h in prop_harmonics:
            print(f"    {h['label']}: {h['min_hz']:.0f}–{h['max_hz']:.0f} Hz ({n_blades}-blade)")

    # ── Phase lag estimation ──
    phase_lag = None
    if config_has_filters(config):
        # Estimate at a frequency relevant to this frame class
        sig_freq = (profile["noise_band_mid"][0] + profile["noise_band_mid"][1]) / 4
        phase_lag = estimate_total_phase_lag(config, profile, sig_freq)
        if phase_lag["total_degrees"] > 20:
            print(f"  Filter phase lag: {phase_lag['total_degrees']:.0f}° ({phase_lag['total_ms']:.1f}ms) at {sig_freq:.0f}Hz")

    print("  Analyzing...")
    noise_results = [analyze_noise(data, ax, f"gyro_{ax.lower()}", sr) for ax in AXIS_NAMES]
    pid_results = [analyze_pid_response(data, i, sr) for i in range(3)]
    motor_analysis = analyze_motors(data, sr)
    dterm_results = analyze_dterm_noise(data, sr)

    # ── Motor response time ──
    motor_response = analyze_motor_response(data, sr)
    if motor_response and motor_response["motor_response_ms"] > 1:
        print(f"  Motor response: {motor_response['motor_response_ms']:.1f}ms")

    plan = generate_action_plan(noise_results, pid_results, motor_analysis, dterm_results,
                                 config, data, profile, phase_lag, motor_response,
                                 rpm_range, prop_harmonics)

    if not args.no_terminal:
        print_terminal_report(plan, noise_results, pid_results, motor_analysis, config, data,
                              show_narrative=not args.no_narrative, profile=profile)

    if not args.no_html:
        print("  Generating report...")
        charts = {}
        vn = [n for n in noise_results if n]
        if vn: charts["noise"] = create_noise_chart(vn)
        charts["pid"] = create_pid_response_chart(pid_results, sr)
        if motor_analysis: charts["motor"] = create_motor_chart(motor_analysis, data["time_s"])
        if dterm_results: charts["dterm"] = create_dterm_chart(dterm_results)

        html = generate_html_report(plan, noise_results, pid_results, motor_analysis, dterm_results, config, data, charts)
        on = args.output or os.path.splitext(os.path.basename(logfile))[0] + "_report.html"
        op = os.path.join(os.path.dirname(logfile) or ".", on)
        with open(op, "w", encoding="utf-8") as f: f.write(html)
        print(f"\n  ✓ Report: {op}")

    # ── Save state with profile ──
    config["_profile_name"] = profile["name"]
    config["_profile_class"] = profile["class"]
    if frame_inches: config["_frame_inches"] = frame_inches
    if prop_inches: config["_prop_inches"] = prop_inches
    config["_n_blades"] = n_blades
    config["_cell_count"] = detected_cells or args.cells
    if args.kv: config["_motor_kv"] = args.kv
    config["_n_motors"] = n_motors
    config["_platform_type"] = platform_type

    sp = save_state(logfile, plan, config, data)
    print(f"  ✓ State: {sp} (use --previous on next run)")
    print()

if __name__ == "__main__":
    main()
