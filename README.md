# INAV Flight Analyzer Toolkit

A suite of Python tools for analyzing, validating, and tuning INAV flight controller configurations. Built for the INAV long-range community focusing on 7" to 15" multirotors with GPS navigation.

## Tools

| Tool | Version | Purpose |
|------|---------|---------|
| **Blackbox Analyzer** | v2.9.0 | Decode blackbox logs, analyze PID performance, recommend tuning changes |
| **Parameter Analyzer** | v1.1.0 | Validate `diff all` configs for safety, filter, PID, and navigation issues |
| **VTOL Configurator** | v1.0.1 | Validate VTOL mixer profiles, motor/servo mixing, and transition setup |

## Quick Start

### Requirements

```bash
pip install numpy scipy --break-system-packages  # only dependency
```

Python 3.8+ required. No other dependencies, all tools are standalone single-file scripts.

### Blackbox Analyzer

Analyzes binary blackbox logs (`.bbl` / `.bfl`) from INAV. Decodes natively in Python, no `blackbox_decode` needed.

```bash
# Full analysis with HTML report
python3 inav_blackbox_analyzer.py flight.bbl

# Specify frame size for tailored recommendations
python3 inav_blackbox_analyzer.py flight.bbl --frame 10

# JSON output for automation
python3 inav_blackbox_analyzer.py flight.bbl --json
```

**What it measures:**
- PID step response: overshoot percentage, settling time, delay
- Noise spectrum: identifies peak frequencies per axis
- Motor balance: detects thrust asymmetry
- Filter phase lag: total delay through the filter chain
- Tracking error: RMS deviation between setpoint and gyro

**Output:** HTML report with plots + `state.json` for cross-referencing with the parameter analyzer.

### Parameter Analyzer

Validates an INAV `diff all` export for configuration issues.

```bash
# Analyze existing config
python3 inav_param_analyzer.py my_diff.txt --frame 10

# Generate starting PIDs for a new build
python3 inav_param_analyzer.py --setup 10 --voltage 6S

# Compare starting PIDs with current config
python3 inav_param_analyzer.py --setup 10 --voltage 6S my_diff.txt

# Cross-reference with blackbox results
python3 inav_param_analyzer.py my_diff.txt --blackbox state.json
```

**What it checks:**
- **Safety:** Beeper configuration, failsafe procedure, battery limits
- **Motors:** Protocol validation, RPM filter + ESC telemetry consistency
- **Filters:** Frame-appropriate gyro LPF, dynamic notch configuration
- **PIDs:** Cross-profile consistency, iterm_relax, D-boost, antigravity
- **Navigation:** RTH altitude, hover throttle, position PIDs, safehome
- **GPS:** Multi-constellation, compass calibration quality (mag gain spread)
- **Blackbox:** Logging rate, essential fields for analysis
- **Battery:** Li-ion vs LiPo detection, voltage limits

**Setup mode** generates conservative starting PIDs for 7/10/12/15" frames at 4S/6S/8S/12S.

### VTOL Configurator

Validates INAV VTOL configurations using mixer_profile switching.

```bash
# Validate VTOL config
python3 inav_vtol_configurator.py vtol_diff.txt

# JSON output
python3 inav_vtol_configurator.py vtol_diff.txt --json
```

**What it checks:**
- MC and FW mixer profiles present with correct `platform_type`
- Motor role inference: tilt / lift-only / pusher by cross-referencing profiles
- Tilt servo rules (smix source 38) for transition
- Yaw authority: motor yaw mixing or tilt servo yaw
- FW control surfaces: roll + pitch servo mixing
- Mode assignments: MIXER PROFILE 2 (mode 62) and MIXER TRANSITION (mode 63)
- Automated RTH transition: `mixer_automated_switch` in both profiles
- Safety: `airmode_type` conflict with transition motors, compass for MC nav
- Control profile linking between mixer profiles

## Tuning Workflow

The tools are designed to work together in a tuning pipeline:

```
1. NEW BUILD
   └─ param_analyzer --setup 10 --voltage 6S
      → Conservative starting PIDs, filters, settings
      → Paste CLI commands into INAV Configurator

2. FIRST FLIGHTS
   └─ param_analyzer my_diff.txt --frame 10
      → Catches safety issues, missing settings, profile inconsistencies
      → Fix before flying

3. BLACKBOX LOGGING
   └─ Fly with blackbox enabled (GYRO_RAW, MOTORS, RC_COMMAND)
      → Manual/acro segment for PID data
      → Nav segment for position hold / RTH data

4. PID TUNING
   └─ blackbox_analyzer flight.bbl --frame 10
      → Overshoot, delay, noise analysis
      → Specific PID change recommendations with CLI commands

5. ITERATE
   └─ Apply recommendations → fly → analyze → repeat
      → param_analyzer --blackbox state.json to cross-reference
```

## Frame Size Profiles

The `--setup` mode provides conservative starting configurations:

| Frame | P Roll | P Pitch | D | Gyro LPF | Dyn Notch Min |
|-------|--------|---------|---|----------|---------------|
| 7" 4S  | 35 | 38 | 23 | 90Hz | 60Hz |
| 10" 4S | 25 | 28 | 18 | 65Hz | 50Hz |
| 12" 4S | 20 | 22 | 14 | 50Hz | 35Hz |
| 15" 4S | 15 | 16 | 10 | 40Hz | 25Hz |

Higher voltage (6S/8S/12S) scales P and D down proportionally.

## INAV Version Support

Developed and tested against **INAV 9.0.x**. The blackbox binary decoder handles the shared Cleanflight/INAV encoding format. Parameter names are INAV-specific, this toolkit does not support Betaflight (yet).

## Project Structure

```
inav_bb_analyzer/
├── README.md                        # This file
├── CHANGELOG.md                     # Version history
├── requirements.txt                 # Python dependencies
├── inav_blackbox_analyzer.py        # Blackbox log analyzer
├── inav_param_analyzer.py           # Config validator + setup generator
├── inav_vtol_configurator.py        # VTOL mixer profile validator
├── docs/
│   ├── BLACKBOX_ANALYZER.md         # Detailed blackbox analyzer docs
│   ├── PARAM_ANALYZER.md            # Detailed parameter analyzer docs
│   ├── VTOL_CONFIGURATOR.md         # Detailed VTOL configurator docs
│   └── TUNING_WORKFLOW.md           # Step-by-step tuning guide
└── tests/                           # Test diff files and logs
```

## Contributing

This is an active project. The navigation analyzer module is planned for future development, along with potential Raspberry Pi-based autonomous tuning.

## License

MIT License. See [LICENSE](LICENSE).

## Acknowledgments

- The INAV development team and community
- QuadMeUp (Paweł Spychalski) for filter and RPM analysis research
- The INAV Fixed Wing Group for modes documentation
- UAV Tech for the spark that gave me the idea to create this
