# Changelog

All notable changes to INAV Toolkit.

## [2026-02-22] - Blackbox Analyzer v2.13.0

### Added
- **Navigation health analysis** (`--nav` flag): Separate analysis mode for compass, GPS, barometer, and position estimator health. Runs independently from tuning - no PID/noise/filter output. Detects motor EMI on compass (throttle-heading correlation), GPS position jumps, baro noise, and estimator divergence. Weighted nav score (compass 30%, GPS 25%, baro 25%, estimator 20%).
- **Compass EMI detector**: Correlates heading jitter with throttle changes to detect motor electromagnetic interference. Downsamples heading to compass update rate (~50Hz) to avoid quantization artifacts from 1000Hz logging.
- **Toilet bowl / salad bowl detector**: For poshold flights only (phase-gated), detects circular drift from compass miscalibration. Uses angular rotation rate and spectral analysis of position error.
- **GPS quality analyzer**: Tracks EPH, satellite count from decoded GPS frames, and position jumps.
- **Barometer quality analyzer**: Measures baro noise RMS, propwash correlation, and altitude spikes.
- **Position estimator health check**: Compares navPos altitude with barometer to detect estimator divergence.
- **Altitude and position hold analysis**: Phase-gated - only runs during detected nav-controlled phases (navState > 1 for >5s), not during manual flight.
- **Flight phase segmentation**: Segments flight using navState from I-frame data.
- **Standalone nav HTML report**: `--nav` generates a dedicated `_nav_report.html` with dark theme, score card, and findings.

### Changed
- **`--nav` is a separate mode**: No tuning output (PIDs, filters, noise, action plan, CLI commands) when `--nav` is used. Clean separation between tuning and navigation analysis.
- **Multi-flight handling in nav mode**: Skips the tuning summary scan table, analyzes the last flight directly.
- **Decoder: navigation field mapping**: Extended col_map in both binary and CSV decoders to extract 30+ navigation fields from I/P frames (navPos, navVel, navTgtPos, navTgtVel, navAcc, attitude, BaroAlt, navState, navFlags, navEPH/EPV, accSmooth, rcData, vbat, amperage, rssi).
- **Decoder: S-frame and G-frame decoding**: Slow frames (flight mode flags) and GPS frames (satellite data) are now decoded and stored instead of skipped. Raw values returned (no I-frame predictor applied).

## [2026-02-21] - Blackbox Analyzer v2.12.0

### Added
- **Flight history database** (`inav_flight_db.py`): SQLite database stores analysis results from every flight - scores, per-axis oscillation data, PID values, filter config, motor balance, and actions. Enables progression tracking across tuning sessions. Zero external dependencies (Python stdlib sqlite3).
- **Multi-log splitter**: Dataflash dumps containing multiple arm/disarm cycles are automatically detected and split into individual flights. Each is analyzed separately with per-flight progression tracking.
- **Flight progression tracking**: After each analysis, the database compares with the previous flight for the same craft and shows score deltas, oscillation changes, and PID value changes. Trend detection (improving/stable/degrading).
- **MSP CLI diff merge**: New `--diff` flag pulls the full `diff all` configuration from the FC before downloading blackbox data. Settings missing from blackbox headers (motor_poles, nav PIDs, rates, level mode, antigravity) are enriched from the diff. Mismatches between blackbox and current FC config are detected and displayed.
- **`--history` flag**: Shows a tabular flight history for the craft with scores, verdicts, and progression summary.
- **`--db-path` / `--no-db` flags**: Custom database path or skip storage entirely.

## [2026-02-21] - Blackbox Analyzer v2.11.0

### Fixed
- **Removed `profile 1` CLI command**: INAV 9 CLI does not support profile switching. The v2.10.0 fix incorrectly emitted `profile 1` before profile-scoped parameters, causing CLI errors.
- **Scoring bug: inflated scores on hover-only logs**: When no stick inputs were detected (0 steps), PID score silently defaulted to 50/100 instead of being flagged as unmeasurable. A pure hover log could score 83/100 with "No changes needed" even on a wobbling quad. Now: PID shows as N/A, overall score is capped at 65, and the verdict tells the pilot to fly with stick inputs.
- **Duplicate CLI commands**: When both oscillation and PID actions targeted the same axis, conflicting `set` commands were emitted. CLI generation now deduplicates by parameter name.
- **MSP dataflash detection**: INAV uses the flags byte as a simple ready boolean (0x01), not a bitfield like Betaflight. The `supported` bit (0x02) is never set by INAV, causing "Dataflash not available" errors on working hardware. Now inferred from `total_size > 0`.
- **MSP dataflash read format**: INAV's MSP_DATAFLASH_READ request takes `address(u32) + size(u16)` without a compression flag byte, and the response is `address(u32) + data` without dataSize/compressedSize headers. The Betaflight-style parsing was corrupting reads after the first chunk.
- **MSP serial port reuse**: `auto_detect_fc()` left the serial port open, then the analyzer opened a second connection to the same port, causing communication failures. Now returns the open device object for reuse.
- **MSP frame decoder robustness**: `$X` bytes appearing inside binary blackbox payload data could cause false frame matches. The decoder now skips false matches using CRC validation.

### Added
- **Hover oscillation detection**: Detects oscillation during hover (no stick input) by analyzing gyro variance in centered-stick segments. Classifies severity (mild/moderate/severe), identifies the dominant frequency, and diagnoses the cause: low-frequency (~2-10Hz) = P too high, mid-frequency (~10-25Hz) = P/D interaction, high-frequency (~25-50Hz) = D-term noise, very high (~50Hz+) = filter gap. Generates CRITICAL/IMPORTANT actions with aggressive P/D reductions. Oscillation actions take priority over regular PID recommendations using oscillation-first enforcement (similar to filter-first).
- **Direct FC communication via MSP**: New `--device` flag downloads blackbox data directly from the flight controller over USB serial. Auto-detects INAV FCs, saves logs with sensible names (craft_timestamp.bbl), and optionally erases dataflash after download. Feeds directly into the analysis pipeline - no more switching to INAV Configurator to download logs.
- **New module `inav_msp.py`**: Standalone MSP v2 protocol implementation for INAV. Handles FC identification, dataflash summary/read/erase. Can also be used independently for scripting.
- **Gyro oscillation detection**: Even without stick inputs, the analyzer now measures gyro variance to detect wobble/oscillation during hover. This feeds into the quality score as a PID proxy when step response data is unavailable.
- **Auto-detect frame size from craft name**: Parses the craft name header (e.g., "NAZGUL 10") to automatically determine frame size. No more silent 5" defaults when the log clearly says otherwise.
- **Frame size conflict warning**: When `--frame` is specified but contradicts the craft name (e.g., `--frame 5` on a "NAZGUL 10" log), the analyzer prints a clear ⚠ warning explaining the mismatch and which value is being used.
- **Auto-detect battery cells from vbatref**: If `--cells` is not specified, battery cell count is inferred from the blackbox `vbatref` header voltage.
- **Platform detection from field names**: Motor count and servo count are read from the `Field I name` header to determine platform type (Quadcopter, Hexacopter, Tricopter, etc.).
- **Comprehensive pre-analysis banner**: Before decoding begins, the analyzer now displays a structured identification block showing: aircraft name, firmware version and build date, platform type (quad/hex/tri + motor/servo count), frame size with source (user/auto-detected/conflict), prop configuration, battery cell count, motor KV, and the analysis profile with thresholds.

### Changed
- Headers are now parsed before building the frame profile, enabling auto-detection to inform profile selection.
- RPM prediction now uses auto-detected cell count when `--cells` is not explicitly provided.

## [2025-02-21] - Blackbox Analyzer v2.10.0

### Fixed
- **Filter-first enforcement**: When filter changes are needed, PID recommendations are now deferred until after filters are fixed and a re-fly. Previously, the analyzer would recommend both filter and PID changes simultaneously, leading users to raise D-term gains into wide-open filters, amplifying noise and creating a downward tuning spiral.
- **CLI parameter naming**: `gyro_lpf_hz` corrected to `gyro_main_lpf_hz` (renamed in INAV 3.0+).
- **Profile-scoped CLI commands**: Parameters like `dterm_lpf_hz`, `mc_p_roll`, etc. now correctly emit a `profile 1` line before profile-scoped settings. Previously, pasting CLI output at the master level would fail with "invalid setting".

### Changed
- CLI output now separates global parameters (gyro filters, dynamic notch) from profile-scoped parameters (PIDs, dterm filters) with proper context switching.
- State JSON now includes a `deferred_actions` array alongside `actions`, making it clear which recommendations require a re-fly first.
- HTML report shows deferred PID changes in a distinct visual section rather than mixing them with actionable filter fixes.

## [2025-02-20] - Project Restructure

### Added
- Project documentation: README, detailed docs per tool, tuning workflow guide
- VTOL Configurator v1.0.1 - new tool for validating VTOL mixer profiles
- Parameter Analyzer setup mode (`--setup`) for generating starting configs

### Blackbox Analyzer v2.9.0
- **Improved:** Filter phase lag calculation with proper transfer functions
  - PT1, PT2, PT3: correct cascaded first-order phase
  - BIQUAD: proper H(s) = ωc² / (s² + (ωc/Q)s + ωc²) with Q factor support
  - Previous version approximated BIQUAD as cascaded PT1 which was inaccurate at high Q
- **Fixed:** RPM filter recommendation - correctly states INAV requires ESC telemetry wire, not bidirectional DSHOT (which is Betaflight-only)

### Parameter Analyzer v1.1.0
- **Added:** `--setup` mode for generating starting PIDs by frame size (7/10/12/15") and voltage (4S/6S/8S/12S)
- **Added:** Li-ion battery detection - uses capacity heuristic (≥7000mAh + low min voltage) to adjust severity
- **Added:** ESC telemetry serial port check for RPM filter validation
- **Fixed:** Removed all bidirectional DSHOT references (not supported in INAV)
- **Fixed:** EZ Tune detection - checks `ez_enabled` flag, not presence of `ez_*` parameters
- **Fixed:** Compass calibration quality check - mag gain spread analysis

### VTOL Configurator v1.0.1
- **Fixed:** Mode IDs - MIXER PROFILE 2 is mode 62, MIXER TRANSITION is mode 63 (was using incorrect Betaflight-derived values)
- **Added:** Control profile linking validation (`mixer_control_profile_linking`)
- **Added:** INAV mode name lookup table for readable output

## [2025-02-17] - Blackbox Analyzer v2.8.0

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

## [2025-02-17] - Parameter Analyzer v1.0.0

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
