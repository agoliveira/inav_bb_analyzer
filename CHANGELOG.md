# Changelog

All notable changes to INAV Toolkit.

## [2026-02-21] — Blackbox Analyzer v2.11.0

### Fixed
- **Removed `profile 1` CLI command**: INAV 9 CLI does not support profile switching. The v2.10.0 fix incorrectly emitted `profile 1` before profile-scoped parameters, causing CLI errors.

### Added
- **Direct FC communication via MSP**: New `--device` flag downloads blackbox data directly from the flight controller over USB serial. Auto-detects INAV FCs, saves logs with sensible names (craft_timestamp.bbl), and optionally erases dataflash after download. Feeds directly into the analysis pipeline — no more switching to INAV Configurator to download logs.
- **New module `inav_msp.py`**: Standalone MSP v2 protocol implementation for INAV. Handles FC identification, dataflash summary/read/erase. Can also be used independently for scripting.
- **Auto-detect frame size from craft name**: Parses the craft name header (e.g., "NAZGUL 10") to automatically determine frame size. No more silent 5" defaults when the log clearly says otherwise.
- **Frame size conflict warning**: When `--frame` is specified but contradicts the craft name (e.g., `--frame 5` on a "NAZGUL 10" log), the analyzer prints a clear ⚠ warning explaining the mismatch and which value is being used.
- **Auto-detect battery cells from vbatref**: If `--cells` is not specified, battery cell count is inferred from the blackbox `vbatref` header voltage.
- **Platform detection from field names**: Motor count and servo count are read from the `Field I name` header to determine platform type (Quadcopter, Hexacopter, Tricopter, etc.).
- **Comprehensive pre-analysis banner**: Before decoding begins, the analyzer now displays a structured identification block showing: aircraft name, firmware version and build date, platform type (quad/hex/tri + motor/servo count), frame size with source (user/auto-detected/conflict), prop configuration, battery cell count, motor KV, and the analysis profile with thresholds.

### Changed
- Headers are now parsed before building the frame profile, enabling auto-detection to inform profile selection.
- RPM prediction now uses auto-detected cell count when `--cells` is not explicitly provided.

## [2025-02-21] — Blackbox Analyzer v2.10.0

### Fixed
- **Filter-first enforcement**: When filter changes are needed, PID recommendations are now deferred until after filters are fixed and a re-fly. Previously, the analyzer would recommend both filter and PID changes simultaneously, leading users to raise D-term gains into wide-open filters, amplifying noise and creating a downward tuning spiral.
- **CLI parameter naming**: `gyro_lpf_hz` corrected to `gyro_main_lpf_hz` (renamed in INAV 3.0+).
- **Profile-scoped CLI commands**: Parameters like `dterm_lpf_hz`, `mc_p_roll`, etc. now correctly emit a `profile 1` line before profile-scoped settings. Previously, pasting CLI output at the master level would fail with "invalid setting".

### Changed
- CLI output now separates global parameters (gyro filters, dynamic notch) from profile-scoped parameters (PIDs, dterm filters) with proper context switching.
- State JSON now includes a `deferred_actions` array alongside `actions`, making it clear which recommendations require a re-fly first.
- HTML report shows deferred PID changes in a distinct visual section rather than mixing them with actionable filter fixes.

## [2025-02-20] — Project Restructure

### Added
- Project documentation: README, detailed docs per tool, tuning workflow guide
- VTOL Configurator v1.0.1 — new tool for validating VTOL mixer profiles
- Parameter Analyzer setup mode (`--setup`) for generating starting configs

### Blackbox Analyzer v2.9.0
- **Improved:** Filter phase lag calculation with proper transfer functions
  - PT1, PT2, PT3: correct cascaded first-order phase
  - BIQUAD: proper H(s) = ωc² / (s² + (ωc/Q)s + ωc²) with Q factor support
  - Previous version approximated BIQUAD as cascaded PT1 which was inaccurate at high Q
- **Fixed:** RPM filter recommendation — correctly states INAV requires ESC telemetry wire, not bidirectional DSHOT (which is Betaflight-only)

### Parameter Analyzer v1.1.0
- **Added:** `--setup` mode for generating starting PIDs by frame size (7/10/12/15") and voltage (4S/6S/8S/12S)
- **Added:** Li-ion battery detection — uses capacity heuristic (≥7000mAh + low min voltage) to adjust severity
- **Added:** ESC telemetry serial port check for RPM filter validation
- **Fixed:** Removed all bidirectional DSHOT references (not supported in INAV)
- **Fixed:** EZ Tune detection — checks `ez_enabled` flag, not presence of `ez_*` parameters
- **Fixed:** Compass calibration quality check — mag gain spread analysis

### VTOL Configurator v1.0.1
- **Fixed:** Mode IDs — MIXER PROFILE 2 is mode 62, MIXER TRANSITION is mode 63 (was using incorrect Betaflight-derived values)
- **Added:** Control profile linking validation (`mixer_control_profile_linking`)
- **Added:** INAV mode name lookup table for readable output

## [2025-02-17] — Blackbox Analyzer v2.8.0

### Added
- Native binary blackbox decoder (no external tools needed)
- Cross-correlation delay measurement between setpoint and gyro
- Noise spectral analysis with peak identification
- Motor balance analysis
- PID step response (overshoot, settling time, delay)
- Filter chain phase lag estimation
- HTML report with embedded plots
- JSON state output for automation
- Frame-size-aware recommendations

## [2025-02-17] — Parameter Analyzer v1.0.0

### Added
- `diff all` parser with profile support
- Safety checks (beepers, failsafe, battery)
- Motor protocol and RPM filter validation
- Filter checks with frame-size awareness
- PID cross-profile consistency
- Navigation parameter validation
- GPS and compass checks
- Blackbox configuration audit
- CLI fix generation
- JSON output mode
- Cross-reference with blackbox state
