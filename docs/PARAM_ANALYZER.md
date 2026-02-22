# Parameter Analyzer - Detailed Documentation

## Overview

The INAV Parameter Analyzer validates `diff all` configuration exports for safety issues, performance problems, and configuration inconsistencies. It also generates conservative starting configurations for new builds.

## Usage

### Analysis Mode (default)

```bash
# Basic analysis
python3 inav_param_analyzer.py my_diff.txt

# With frame size for tailored filter/PID recommendations
python3 inav_param_analyzer.py my_diff.txt --frame 10

# Cross-reference with blackbox analyzer results
python3 inav_param_analyzer.py my_diff.txt --blackbox state.json

# JSON output for automation
python3 inav_param_analyzer.py my_diff.txt --json
```

### Setup Mode

```bash
# Generate starting config for a new build
python3 inav_param_analyzer.py --setup 10 --voltage 6S

# Compare with existing config
python3 inav_param_analyzer.py --setup 10 --voltage 6S my_diff.txt

# JSON output
python3 inav_param_analyzer.py --setup 10 --voltage 6S --json
```

## Getting a `diff all` File

In INAV Configurator:
1. Go to the **CLI** tab
2. Type `diff all` and press Enter
3. Click **Save to file**

Or via command line if you have serial access:
```bash
# The analyzer can also read from stdin
echo "diff all" | picocom /dev/ttyACM0 -b 115200 | python3 inav_param_analyzer.py -
```

## Severity Levels

| Level | Icon | Meaning |
|-------|------|---------|
| **CRITICAL** | ✗ | Safety risk - fix before flying |
| **WARNING** | ⚠ | Performance issue - should address |
| **INFO** | ℹ | Suggestion - consider addressing |
| **OK** | ✓ | Check passed |

## Check Categories

### Safety

- **Beeper configuration:** Alerts if critical beepers (BAT_CRIT_LOW, BAT_LOW, RX_LOST, HW_FAILURE) are disabled. These are your only warning for low battery and signal loss.
- **DSHOT beeper:** Last-resort quad finder using motor beeps. Important if physical buzzer is also disabled.
- **Failsafe procedure:** Validates RTH vs DROP. Checks `failsafe_min_distance` for close-range landing.

### Motors

- **Protocol:** Validates DSHOT300/600 usage.
- **RPM filter:** If `rpm_gyro_filter_enabled` is ON, checks that ESC telemetry is configured on a serial port (function 4096). INAV uses the ESC telemetry wire - not bidirectional DSHOT.
- **Motor stop:** Warns if motor stop on low is enabled (dangerous for multirotor).
- **Throttle idle:** Validates reasonable idle percentage.

### Filters

- **Gyro LPF:** Checks if cutoff frequency is appropriate for frame size. 10" props at 4000 RPM have fundamental noise ~67Hz, so a 65Hz LPF is appropriate. A 110Hz LPF (5" default) would pass too much noise.
- **Dynamic notch:** Validates mode, Q factor, and min_hz for frame size. Large props produce lower-frequency noise that needs a lower min_hz.
- **Kalman Q:** Checks setpoint smoothing aggressiveness.

### PIDs

- **Cross-profile consistency:** Compares settings across control profiles. Catches cases where one profile has iterm_relax RPY, D-boost, or antigravity but another doesn't - usually from incomplete configuration.
- **EZ Tune detection:** Correctly detects whether EZ Tune is active (checks `ez_enabled`, not just presence of `ez_*` params in diff). Warns that manual PID changes will be overwritten if EZ Tune is ON.
- **TPA:** Validates breakpoint isn't too low for the quad's hover throttle.

### Navigation

- RTH altitude sanity (15-100m)
- Hover throttle range
- Position P gain range
- Safehome configuration

### GPS

- Feature enabled vs failsafe RTH consistency
- Multi-constellation (Galileo, GLONASS, BeiDou)
- Compass hardware and alignment
- **Compass calibration quality:** Analyzes `maggain_x/y/z` spread. >40% spread between axes indicates poor calibration, which causes toilet-bowling in GPS modes.

### Battery

- **Li-ion detection:** Uses capacity heuristic (≥7000mAh + low min voltage = likely Li-ion). Adjusts severity from WARNING to INFO for appropriate Li-ion voltages.
- Capacity monitoring and warning thresholds.

### Blackbox

- Logging rate assessment
- Essential fields for PID analysis (GYRO_RAW, MOTORS, RC_COMMAND)
- Nav fields for navigation analysis (NAV_ACC, NAV_POS)

## Setup Mode - Frame Profiles

Conservative starting values designed for safe first hover:

### 7-inch (long-range)
- Typical: 2807-3007 motors, 800-1400g AUW
- Close to INAV defaults, slightly reduced

### 10-inch (cruiser)
- Typical: 3507-4010 motors, 1500-2800g AUW
- PIDs ~60% of 7" values due to ~4× prop inertia

### 12-inch (heavy lift)
- Typical: 4510-5015 motors, 2500-5000g AUW
- Very low P essential to prevent oscillation

### 15-inch (cine lifter)
- Typical: 5515-7015 motors, 4000-10000g AUW
- Most conservative PIDs - overcorrection is the main risk

### Voltage Scaling

| Voltage | PID Scale | Notes |
|---------|-----------|-------|
| 4S | 1.00 | Baseline |
| 6S | 0.85 | Faster motor response, -15% PIDs |
| 8S | 0.75 | -25% PIDs |
| 12S | 0.65 | Very conservative |

I-term scales less aggressively (it's about steady-state holding, not reaction speed).

## Parser

The `diff all` parser extracts:
- Firmware version, board, git hash
- Master (global) settings
- Control profiles (1-3) with per-profile settings
- Mixer profiles with motor/servo mix
- Battery profiles
- Features (enabled and disabled)
- Beepers (enabled and disabled)
- Blackbox fields
- Serial port configurations
- Aux mode assignments

Active profile detection uses the `profile <n>` commands at the end of the diff to determine which profiles are currently selected.
