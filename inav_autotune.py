#!/usr/bin/env python3
"""
INAV Autotune Orchestrator
───────────────────────────
Automated PID tuning loop for INAV multirotors.

Connects to the FC via USB/UART, captures blackbox data, runs the
analyzer, computes new parameters, and optionally pushes them back.

Same code runs on:
  - Your laptop at the bench (USB cable to FC)
  - A Pi Zero 2W strapped to the quad (USB OTG to FC)

Usage:
    # Advisory mode - show recommendations, don't apply
    python inav_autotune.py /dev/ttyACM0 --frame 5 --blades 3

    # Auto mode - apply changes automatically between captures
    python inav_autotune.py /dev/ttyACM0 --frame 10 --blades 2 --auto

    # Single analysis of an existing blackbox file (no FC connection)
    python inav_autotune.py --file flight.bbl --frame 7

Requires: inav_msp.py, inav_blackbox_analyzer.py
"""

import argparse
import json
import os
import sys
import time
import copy
import logging

log = logging.getLogger("autotune")

# ─── Import our modules ──────────────────────────────────────────────────────

try:
    from inav_msp import INAVLink
except ImportError:
    INAVLink = None

from inav_blackbox_analyzer import (
    get_frame_profile, parse_csv_log, analyze_noise, analyze_pid_response,
    analyze_motors, analyze_dterm_noise, generate_action_plan,
    analyze_motor_response, estimate_rpm_range, estimate_prop_harmonics,
    estimate_total_phase_lag, config_has_pid, config_has_filters,
    AXIS_NAMES, REPORT_VERSION,
)


# ─── Safety Clamps ───────────────────────────────────────────────────────────
# These are absolute limits that can NEVER be exceeded, regardless of what
# the analyzer recommends. They protect against bugs, bad data, and edge cases.
# The frame profile provides tighter, class-specific limits on top of these.

ABSOLUTE_CLAMPS = {
    "roll_p":  (10, 200), "roll_i":  (5, 200), "roll_d":  (0, 150),
    "pitch_p": (10, 200), "pitch_i": (5, 200), "pitch_d": (0, 150),
    "yaw_p":   (10, 200), "yaw_i":   (5, 200), "yaw_d":   (0, 100),
    "gyro_lowpass_hz":  (10, 500),
    "dterm_lpf_hz":     (10, 300),
}

# Maximum change per parameter per iteration (as fraction of current value)
MAX_CHANGE_PER_STEP = 0.20  # never change more than 20% in one step


def clamp_value(param, value, profile=None):
    """Apply safety clamps to a parameter value."""
    value = int(round(value))
    if param in ABSOLUTE_CLAMPS:
        lo, hi = ABSOLUTE_CLAMPS[param]
        value = max(lo, min(hi, value))
    return value


def limit_change(param, old_value, new_value):
    """Limit the magnitude of change per step."""
    if old_value == 0:
        return new_value
    max_delta = abs(old_value * MAX_CHANGE_PER_STEP)
    delta = new_value - old_value
    if abs(delta) > max_delta:
        clamped = old_value + (max_delta if delta > 0 else -max_delta)
        log.info(f"  Rate-limited {param}: wanted {old_value}→{new_value}, "
                 f"capped to {old_value}→{int(clamped)}")
        return int(clamped)
    return new_value


# ─── Tuning Session ──────────────────────────────────────────────────────────

class TuningSession:
    """Manages an iterative tuning session.

    Tracks the history of parameters and scores across iterations,
    enforces safety limits, and can revert to any previous state.
    """

    def __init__(self, profile, motor_kv=None, cell_count=None):
        self.profile = profile
        self.motor_kv = motor_kv
        self.cell_count = cell_count
        self.iterations = []       # list of {params, plan, score, timestamp}
        self.initial_params = None # snapshot before any changes
        self.best_score = 0
        self.best_iteration = 0

    def record_iteration(self, params, plan):
        """Record the results of one analyze-recommend cycle."""
        score = plan["scores"]["overall"]
        entry = {
            "iteration": len(self.iterations) + 1,
            "params": copy.deepcopy(params),
            "score": score,
            "verdict": plan["verdict"],
            "n_actions": len(plan["actions"]),
            "timestamp": time.time(),
        }
        self.iterations.append(entry)

        if score > self.best_score:
            self.best_score = score
            self.best_iteration = entry["iteration"]

        return entry

    def get_safe_changes(self, plan, current_params):
        """Extract parameter changes from the action plan, apply safety clamps
        and rate limiting. Returns dict of {param: new_value} ready to apply."""
        changes = {}

        for action in plan["actions"]:
            param = action.get("param")
            new_val = action.get("new")

            if param is None or new_val is None:
                continue
            if isinstance(new_val, str):
                continue  # skip non-numeric recommendations like "see action"
            if param in ("motor_saturation", "motor_balance", "motor_response",
                         "filter_chain", "dynamic_notch"):
                continue  # skip diagnostic-only actions

            # Get current value
            old_val = current_params.get(param)
            if old_val is None or isinstance(old_val, str):
                continue

            new_val = int(new_val)
            old_val = int(old_val)

            # Apply rate limiting
            new_val = limit_change(param, old_val, new_val)

            # Apply absolute clamps
            new_val = clamp_value(param, new_val, self.profile)

            # Only include if actually changed
            if new_val != old_val:
                changes[param] = new_val

        return changes

    def is_converged(self, min_score=70, min_iterations=2):
        """Check if tuning has converged (good enough to stop)."""
        if len(self.iterations) < min_iterations:
            return False

        latest = self.iterations[-1]
        if latest["score"] >= min_score and latest["n_actions"] <= 2:
            return True

        # Check if score stopped improving
        if len(self.iterations) >= 3:
            recent_scores = [it["score"] for it in self.iterations[-3:]]
            if max(recent_scores) - min(recent_scores) < 3:
                return True  # plateau

        return False

    def summary(self):
        """Print a summary of the tuning session."""
        lines = []
        lines.append(f"\n  ═══ Tuning Session Summary ═══")
        lines.append(f"  Profile: {self.profile['name']} ({self.profile['class']})")
        lines.append(f"  Iterations: {len(self.iterations)}")

        if self.iterations:
            lines.append(f"  Best score: {self.best_score:.0f} (iteration {self.best_iteration})")
            lines.append(f"  Latest: score={self.iterations[-1]['score']:.0f}, "
                        f"verdict={self.iterations[-1]['verdict']}")
            lines.append(f"")
            lines.append(f"  {'#':>3s}  {'Score':>6s}  {'Actions':>7s}  Verdict")
            lines.append(f"  {'─'*3}  {'─'*6}  {'─'*7}  {'─'*20}")
            for it in self.iterations:
                marker = " ◄" if it["iteration"] == self.best_iteration else ""
                lines.append(f"  {it['iteration']:3d}  {it['score']:6.0f}  {it['n_actions']:7d}  "
                           f"{it['verdict']}{marker}")

        return "\n".join(lines)

    def save(self, path):
        """Save session state to JSON file."""
        data = {
            "profile": self.profile["name"],
            "iterations": self.iterations,
            "best_score": self.best_score,
            "best_iteration": self.best_iteration,
        }
        with open(path, "w") as f:
            json.dump(data, f, indent=2, default=str)
        log.info(f"Session saved to {path}")


# ─── Analyze from FC parameters ──────────────────────────────────────────────

def analyze_from_csv(csv_path, config, profile, motor_kv=None, cell_count=None):
    """Run the full analysis pipeline on a CSV blackbox file.
    Returns (plan, noise_results, pid_results, motor_analysis)."""

    data = parse_csv_log(csv_path)
    sr = data["sample_rate"]

    noise_results = [analyze_noise(data, ax, f"gyro_{ax.lower()}", sr) for ax in AXIS_NAMES]
    pid_results = [analyze_pid_response(data, i, sr) for i in range(3)]
    motor_analysis = analyze_motors(data, sr)
    dterm_results = analyze_dterm_noise(data, sr)
    motor_response = analyze_motor_response(data, sr)

    # RPM prediction
    rpm_range = estimate_rpm_range(motor_kv, cell_count)
    prop_harmonics = estimate_prop_harmonics(rpm_range, profile.get("n_blades", 3)) if rpm_range else None

    # Phase lag
    phase_lag = None
    if config_has_filters(config):
        sig_freq = (profile["noise_band_mid"][0] + profile["noise_band_mid"][1]) / 4
        phase_lag = estimate_total_phase_lag(config, profile, sig_freq)

    plan = generate_action_plan(
        noise_results, pid_results, motor_analysis, dterm_results,
        config, data, profile, phase_lag, motor_response, rpm_range, prop_harmonics
    )

    return plan, data, noise_results, pid_results, motor_analysis


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="INAV Autotune - Iterative PID Tuning",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Bench mode: FC connected via USB, analyze a blackbox file
  python inav_autotune.py /dev/ttyACM0 --file flight.bbl --frame 5

  # Read current PIDs from FC and show status
  python inav_autotune.py /dev/ttyACM0 --frame 5 --status

  # Analyze a blackbox file with FC params, show recommendations
  python inav_autotune.py /dev/ttyACM0 --file flight.bbl --frame 10 --blades 2

  # Auto-apply recommendations to FC (bench tuning with USB)
  python inav_autotune.py /dev/ttyACM0 --file flight.bbl --frame 5 --apply

  # Offline mode (no FC): analyze a file only
  python inav_autotune.py --file flight.bbl --frame 7 --blades 2
        """)

    parser.add_argument("port", nargs="?", help="Serial port to FC (e.g., /dev/ttyACM0)")
    parser.add_argument("--file", help="Blackbox log file (.bbl or .csv) to analyze")
    parser.add_argument("--decoder", help="Path to blackbox_decode binary")

    # Frame profile
    parser.add_argument("--frame", type=int, metavar="INCHES",
                        help="Frame size in inches (determines PID thresholds)")
    parser.add_argument("--props", type=int, metavar="INCHES",
                        help="Prop diameter in inches (determines filter ranges)")
    parser.add_argument("--blades", type=int, default=3, metavar="N",
                        help="Prop blade count (default: 3)")
    parser.add_argument("--cells", type=int, metavar="S", help="Battery cell count")
    parser.add_argument("--kv", type=int, metavar="KV", help="Motor KV rating")

    # Operation modes
    parser.add_argument("--status", action="store_true",
                        help="Read and display FC status and current PIDs")
    parser.add_argument("--apply", action="store_true",
                        help="Apply recommended changes to FC (with confirmation)")
    parser.add_argument("--auto", action="store_true",
                        help="Auto-apply without confirmation (use with caution)")
    parser.add_argument("--dry-run", action="store_true",
                        help="Show what would change without applying")
    parser.add_argument("--save", action="store_true",
                        help="Save applied changes to EEPROM (persistent)")
    parser.add_argument("--revert", action="store_true",
                        help="Revert FC to initial params from session start")
    parser.add_argument("--session", help="Session state file for tracking iterations")

    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="  %(message)s")

    # ── Build profile ──
    frame_inches = args.frame
    prop_inches = args.props
    if frame_inches and not prop_inches:
        prop_inches = frame_inches
    elif prop_inches and not frame_inches:
        frame_inches = prop_inches
    profile = get_frame_profile(frame_inches, prop_inches, args.blades)

    print(f"\n  ▲ INAV Autotune v{REPORT_VERSION}")
    print(f"  Profile: {profile['name']} ({profile['class']})")

    # ── Connect to FC if port provided ──
    fc = None
    fc_params = {}

    if args.port:
        if INAVLink is None:
            print("  ERROR: inav_msp.py not found or pyserial not installed")
            print("         pip install pyserial")
            sys.exit(1)

        fc = INAVLink(args.port)
        try:
            info = fc.connect()
            print(f"  FC: {info['fc_variant']} {'.'.join(str(v) for v in info['fc_version'])}")
            print(f"  Board: {info['board_name']} | Craft: {info.get('craft_name', '?')}")
            print(f"  MSP: {'V2' if info['msp_v2'] else 'V1'}")

            fc_params = fc.read_tuning_params()

            if args.status:
                status = fc.get_status()
                if status:
                    armed = "ARMED" if status["is_armed"] else "DISARMED"
                    print(f"\n  Status: {armed}")

                analog = fc.get_analog()
                if analog:
                    print(f"  Battery: {analog['vbat']:.1f}V | {analog['amps']:.1f}A")

                print(f"\n  Current PID Values:")
                for axis in ["roll", "pitch", "yaw"]:
                    p = fc_params.get(f"{axis}_p", "?")
                    i = fc_params.get(f"{axis}_i", "?")
                    d = fc_params.get(f"{axis}_d", "?")
                    print(f"    {axis.capitalize():6s}  P={p:>3s}  I={i:>3s}  D={d:>3s}" if isinstance(p, str)
                          else f"    {axis.capitalize():6s}  P={p:3d}  I={i:3d}  D={d:3d}")

                filters = fc_params.get("_raw_filters", {})
                if filters:
                    print(f"\n  Current Filters:")
                    for k, v in filters.items():
                        if v and v > 0:
                            print(f"    {k}: {v}")

                if not args.file:
                    print()
                    fc.close()
                    return

        except Exception as e:
            print(f"  ERROR connecting to FC: {e}")
            sys.exit(1)

    elif not args.file:
        parser.print_help()
        print("\n  ERROR: provide either a serial port or --file (or both)")
        sys.exit(1)

    # ── Analyze blackbox file ──
    if args.file:
        from inav_blackbox_analyzer import parse_headers_from_bbl, decode_blackbox, extract_fc_config
        import shutil

        logfile = args.file
        if not os.path.isfile(logfile):
            print(f"  ERROR: File not found: {logfile}")
            sys.exit(1)

        print(f"\n  Analyzing: {logfile}")

        tmpdir = None
        ext = os.path.splitext(logfile)[1].lower()
        if ext in (".bbl", ".bfl", ".bbs"):
            raw_params = parse_headers_from_bbl(logfile)
            csv_path, tmpdir = decode_blackbox(logfile, args.decoder)
            file_config = extract_fc_config(raw_params)
        else:
            csv_path = logfile
            file_config = {}

        # Merge: FC live params take priority over file headers
        config = {**file_config, **{k: v for k, v in fc_params.items() if not k.startswith("_")}}

        plan, data, noise_results, pid_results, motor_analysis = analyze_from_csv(
            csv_path, config, profile, args.kv, args.cells
        )

        if tmpdir:
            shutil.rmtree(tmpdir, ignore_errors=True)

        # ── Display results ──
        score = plan["scores"]["overall"]
        print(f"\n  Quality Score: {score:.0f}/100 - {plan['verdict']}")
        print(f"  Actions: {len(plan['actions'])}")

        if plan["actions"]:
            print(f"\n  Recommended Changes:")
            print(f"  {'#':>3s}  {'Priority':>8s}  Action")
            print(f"  {'─'*3}  {'─'*8}  {'─'*50}")
            for i, action in enumerate(sorted(plan["actions"], key=lambda a: a["priority"])):
                urg = f"[{action['urgency']}]" if action.get("urgency") else ""
                print(f"  {i+1:3d}  P{action['priority']:d} {urg:>10s}  {action['action']}")

        # ── Session tracking ──
        session = TuningSession(profile, args.kv, args.cells)
        if session.initial_params is None:
            session.initial_params = copy.deepcopy(config)
        session.record_iteration(config, plan)

        # ── Apply changes ──
        if (args.apply or args.auto or args.dry_run) and fc:
            changes = session.get_safe_changes(plan, config)

            if not changes:
                print(f"\n  No applicable parameter changes.")
            else:
                print(f"\n  Proposed Changes (safety-clamped):")
                for param, new_val in changes.items():
                    old_val = config.get(param, "?")
                    print(f"    {param}: {old_val} → {new_val}")

                if args.dry_run:
                    print(f"\n  DRY RUN - no changes applied")
                elif args.auto:
                    applied = fc.apply_tuning_params(changes)
                    print(f"\n  ✓ Applied {len(applied)} changes to FC (RAM)")
                    if args.save:
                        fc.save_to_eeprom()
                        print(f"  ✓ Saved to EEPROM")
                elif args.apply:
                    confirm = input(f"\n  Apply {len(changes)} changes to FC? [y/N] ").strip().lower()
                    if confirm == 'y':
                        applied = fc.apply_tuning_params(changes)
                        print(f"\n  ✓ Applied {len(applied)} changes to FC (RAM)")
                        if args.save:
                            fc.save_to_eeprom()
                            print(f"  ✓ Saved to EEPROM")
                        else:
                            print(f"  ⚠ Changes in RAM only - use --save to persist, or reboot to discard")
                    else:
                        print(f"  Cancelled.")

        # ── Revert ──
        if args.revert and fc and session.initial_params:
            print(f"\n  Reverting to initial parameters...")
            revert_changes = {}
            for param in ABSOLUTE_CLAMPS.keys():
                if param in session.initial_params:
                    revert_changes[param] = session.initial_params[param]
            if revert_changes:
                fc.apply_tuning_params(revert_changes)
                if args.save:
                    fc.save_to_eeprom()
                print(f"  ✓ Reverted {len(revert_changes)} parameters")

        # ── Save session ──
        if args.session:
            session.save(args.session)

        print(session.summary())

    # ── Cleanup ──
    if fc:
        fc.close()
    print()


if __name__ == "__main__":
    main()
