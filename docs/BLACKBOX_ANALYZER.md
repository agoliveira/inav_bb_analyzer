# Blackbox Analyzer - Detailed Documentation

## Overview

The INAV Blackbox Analyzer decodes binary blackbox logs natively in Python and performs comprehensive PID tuning analysis. It produces an HTML report with interactive plots and a `state.json` file for cross-referencing with the parameter analyzer.

## Usage

```bash
# Basic analysis
python3 inav_blackbox_analyzer.py flight.bbl

# With frame size (improves filter recommendations)
python3 inav_blackbox_analyzer.py flight.bbl --frame 10

# JSON output only (no HTML report)
python3 inav_blackbox_analyzer.py flight.bbl --json

# Specify output directory
python3 inav_blackbox_analyzer.py flight.bbl -o /path/to/output/
```

## Binary Decoder

The analyzer includes a native Python decoder for the INAV/Cleanflight blackbox binary format. No external `blackbox_decode` tool is needed.

**Supported encoding:**
- Variable-byte signed/unsigned integers
- Tag2-3S32, Tag8-4S16, Tag8-8SVB encodings
- I-frame and P-frame (delta) decoding
- Automatic field definition parsing from log headers

**Decoded fields include:** gyro (raw/filtered), setpoint, motor outputs, PID terms (P/I/D per axis), RC commands, battery voltage/current, GPS position, nav state, and more - depending on what's enabled in blackbox settings.

## Analysis Modules

### PID Step Response

Detects discrete stick inputs (steps) in setpoint data and measures the gyro response:

- **Overshoot:** How far the actual rate exceeds the target (percentage). >15% suggests P too high or D too low.
- **Settling time:** Time to reach ±5% of the target. Slow settling may indicate insufficient I-term.
- **Delay:** Time from setpoint change to first gyro response. Dominated by filter phase lag.
- **Rise time:** 10% to 90% of target value. Slow rise = P too low.

Step detection requires clean, discrete stick movements. Continuous stick input (freestyle) may not yield measurable steps - the analyzer reports N/A in this case.

### Noise Spectral Analysis

Performs FFT on gyro data to identify noise frequency peaks:

- **Per-axis spectrum:** Roll, pitch, yaw noise profiles
- **Peak identification:** Dominant noise frequencies with amplitude
- **Motor noise correlation:** Peaks that align with motor RPM harmonics
- **Filter effectiveness:** Compares pre/post filter noise (when GYRO_RAW is logged)

### Motor Balance

Compares average output and variance across all motors:

- **Thrust asymmetry:** One motor consistently working harder indicates CG offset, damaged prop, or motor issue
- **Saturation:** Motors hitting max output = not enough headroom for corrections

### Filter Phase Lag

Calculates total delay through the configured filter chain:

- Supports PT1, PT2, PT3, and BIQUAD filter types
- BIQUAD uses proper transfer function with Q factor (not cascaded PT1 approximation)
- Reports per-filter and total lag in degrees and milliseconds
- Key metric for large quads where excessive filtering causes PID instability

### Cross-Correlation Delay

Uses cross-correlation between setpoint and gyro signals to measure the true system delay including all sources: filtering, motor response, and mechanical inertia.

## Filter Phase Lag Math

The v2.9.0 update improved filter phase calculations:

**PT1 (first-order):**
```
phase = -arctan(f / fc)
```

**PT2 (two cascaded PT1):**
```
phase = -2 × arctan(f / fc)
```

**PT3 (three cascaded PT1):**
```
phase = -3 × arctan(f / fc)
```

**BIQUAD (second-order with Q factor):**
```
H(s) = ωc² / (s² + (ωc/Q)s + ωc²)
phase = -arctan2((ω/Q) × ωc, ωc² - ω²)
```

The BIQUAD calculation matters because INAV's dynamic notch uses Q=250, which behaves very differently from a Butterworth (Q=0.5) at frequencies near the cutoff.

## Output Files

### HTML Report

Interactive report with:
- Summary card with key metrics
- PID response plots per axis
- Noise spectrum plots
- Motor balance chart
- Filter chain analysis
- Recommended CLI commands

### state.json

Machine-readable analysis results for use with `inav_param_analyzer.py --blackbox state.json`:

```json
{
  "version": "2.9.0",
  "timestamp": "2025-02-20T14:30:00",
  "tune_quality": 72,
  "axes": {
    "roll": {
      "overshoot_pct": 12.5,
      "delay_ms": 44.0,
      "rms_error": 53.9,
      "noise_floor": 2.1
    }
  },
  "config": { ... },
  "actions": [ ... ]
}
```

## Blackbox Configuration

For best analysis results, enable these blackbox fields:

```
blackbox GYRO_RAW
blackbox RC_COMMAND
blackbox MOTORS
blackbox ACC
blackbox ATTI
blackbox MAG
blackbox NAV_PID
blackbox NAV_POS
blackbox NAV_ACC
blackbox PEAKS_R
blackbox PEAKS_P
blackbox PEAKS_Y
save
```

**Logging rate:** `blackbox_rate_denom = 2` (half rate) works well for both PID and nav analysis. Full rate (`= 1`) gives marginal improvement for PID tuning at the cost of 2× flash usage.

## Recommended Flight Pattern

For PID analysis:
1. **Manual/Acro mode**, 2 minutes
2. Quick roll snaps left/right (3-4 each)
3. Pitch pumps up/down (3-4 each)
4. Yaw snaps left/right (3-4 each)
5. Throttle punch-outs and chops (2-3 each)

For navigation analysis (future):
1. Altitude hold hover, 30 seconds
2. Position hold, 30 seconds
3. Fly out 100-200m, trigger RTH
4. Let it complete the full return and landing
