# INAV Toolkit

A suite of Python tools for analyzing, validating, and tuning INAV flight controller configurations. Built for the INAV long-range community - 7" to 15" multirotors with GPS navigation.

## Tools

| Tool | Purpose |
|------|---------|
| **Blackbox Analyzer** | Decode blackbox logs, analyze PID performance, detect oscillation, recommend tuning changes |
| **Parameter Analyzer** | Validate `diff all` configs for safety, filter, PID, and navigation issues |
| **VTOL Configurator** | Validate VTOL mixer profiles, motor/servo mixing, and transition setup |
| **Flight Database** | SQLite storage for flight history, progression tracking across tuning sessions |
| **MSP Communication** | Direct FC communication - download blackbox, pull config, identify hardware |

## Quick Start

### Requirements

```bash
pip install numpy scipy --break-system-packages  # core dependency
pip install pyserial --break-system-packages      # optional: for --device mode
```

Python 3.8+ required. All tools are standalone scripts with no external dependencies beyond numpy/scipy. `pyserial` is only needed when downloading blackbox logs directly from the flight controller. The flight database uses Python's built-in sqlite3.

### Blackbox Analyzer

Analyzes binary blackbox logs (`.bbl` / `.bfl`) from INAV. Decodes natively in Python - no `blackbox_decode` needed.

```bash
# Full pipeline: connect to FC, pull config diff, download blackbox, analyze
python3 inav_blackbox_analyzer.py --device auto --diff

# Download, analyze, and erase flash for next session
python3 inav_blackbox_analyzer.py --device auto --diff --erase

# Download only (saves to ./blackbox/, no analysis)
python3 inav_blackbox_analyzer.py --device auto --download-only

# Analyze from file (auto-detects frame size, cells, platform)
python3 inav_blackbox_analyzer.py flight.bbl

# Multi-log files are automatically split and analyzed individually
python3 inav_blackbox_analyzer.py dataflash_dump.bbl

# Show flight history and progression for a craft
python3 inav_blackbox_analyzer.py flight.bbl --history

# Override frame size if auto-detection doesn't apply
python3 inav_blackbox_analyzer.py flight.bbl --frame 10

# Add motor KV for RPM noise prediction (cells auto-detected from vbatref)
python3 inav_blackbox_analyzer.py flight.bbl --kv 980

# Skip database storage for one-off analysis
python3 inav_blackbox_analyzer.py flight.bbl --no-db

# Navigation health analysis (compass, GPS, baro, estimator)
python3 inav_blackbox_analyzer.py flight.bbl --nav

# Custom database path
python3 inav_blackbox_analyzer.py flight.bbl --db-path ~/my_flights.db
```

**What it measures:**
- **Hover oscillation:** Detects oscillation during hover from gyro variance, classifies severity, identifies dominant frequency, diagnoses cause (P too high / PD interaction / D-term noise / filter gap)
- **PID step response:** Overshoot percentage, settling time, delay
- **Noise spectrum:** Identifies peak frequencies per axis
- **Motor balance:** Detects thrust asymmetry
- **Filter phase lag:** Total delay through the filter chain
- **Tracking error:** RMS deviation between setpoint and gyro

**Navigation health analysis** (`--nav`): Separate analysis mode that checks sensor health from any flight - no poshold/RTH required. Runs independently from PID tuning with its own terminal output and HTML report.
- **Compass:** Heading jitter, motor EMI correlation (throttle vs heading), drift rate
- **GPS:** Satellite count, EPH/EPV, position jumps
- **Barometer:** Noise RMS, propwash correlation, altitude spikes
- **Estimator:** navPos vs BaroAlt divergence detection
- **Toilet bowl detector:** Phase-gated position drift analysis for poshold flights
- **Tuning prerequisite check:** Warns when PID oscillation is polluting nav sensor readings

**Multi-log splitting:** Dataflash dumps containing multiple arm/disarm cycles are automatically detected and split. Each flight is analyzed individually with per-flight progression tracking.

**CLI diff merge:** When using `--diff` with `--device`, the analyzer pulls the full `diff all` from the FC. Settings not present in blackbox headers (motor_poles, nav PIDs, rates, level mode, antigravity) are enriched from the diff. Mismatches between what was flying and the current FC config are detected and displayed.

**Flight database:** Every analysis is stored in a SQLite database (`inav_flights.db`). Scores, per-axis oscillation data, PID values, filter config, motor balance, and recommended actions are tracked per flight. The `--history` flag shows a progression table.

**Auto-detection:** Frame size from craft name, battery cells from vbatref, platform type (quad/hex/tri) from motor count. Warns on conflicts between detected and user-specified values.

**Oscillation-first enforcement:** When hover oscillation is detected, the analyzer generates aggressive P/D reductions as top-priority actions and defers regular PID recommendations on oscillating axes - you can't tune what you can't stabilize.

**Filter-first enforcement:** When filter changes are needed, PID recommendations are deferred until after filters are fixed and a re-fly, preventing the common death spiral of raising D-term gains into wide-open filters.

**Output:** Terminal report with actionable CLI commands, HTML report with plots, state JSON for cross-referencing, and SQLite database for history.

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

### MSP Communication

Direct serial communication with INAV flight controllers over USB.

```bash
# Identify connected FC
python3 inav_msp.py --info-only

# Download blackbox data
python3 inav_msp.py --device auto

# Specify serial port
python3 inav_msp.py --device /dev/ttyACM0
```

**Capabilities:**
- MSP v2 protocol with CRC-8 validation
- FC identification (craft name, firmware, board, build date)
- Dataflash summary, read, and erase
- CLI mode for `diff all` config retrieval
- Auto-detection of serial ports (Linux/macOS)
- Binary blackbox download with progress bar and retry logic

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
      → Hover segment for oscillation detection
      → Manual/acro segment for PID step response data
      → Nav segment for position hold / RTH data

4. PID TUNING (one-step with FC connected)
   └─ blackbox_analyzer --device auto --diff --erase
      → Downloads blackbox, pulls current config
      → Splits multi-log files automatically
      → Detects hover oscillation first
      → Recommends filter fixes before PID changes
      → Stores results in flight database
      → Shows progression from previous flights

5. ITERATE
   └─ Apply CLI commands → fly → analyze → repeat
      → Database tracks score progression across sessions
      → --history shows the full tuning journey
      → Mismatch detection catches config drift
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

Developed and tested against **INAV 9.0.x**. The blackbox binary decoder handles the shared Cleanflight/INAV encoding format. MSP v2 protocol for FC communication. Parameter names are INAV-specific - this toolkit does not support Betaflight.

## Project Structure

```
inav-toolkit/
├── README.md                        # This file
├── CHANGELOG.md                     # Version history
├── requirements.txt                 # Python dependencies
├── inav_blackbox_analyzer.py        # Blackbox log analyzer (v2.13.0)
├── inav_msp.py                      # MSP v2 serial communication with FC
├── inav_flight_db.py                # SQLite flight history database
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

This is an active project. Planned next: per-phase nav analysis for poshold/RTH/waypoint flights, and potential Raspberry Pi-based autonomous tuning.

## License

MIT License. See [LICENSE](LICENSE).

## Acknowledgments

- The INAV development team and community
- QuadMeUp (Paweł Spychalski) for filter and RPM analysis research
- The INAV Fixed Wing Group for modes documentation
