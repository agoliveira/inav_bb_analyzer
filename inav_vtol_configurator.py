#!/usr/bin/env python3
"""
INAV VTOL Configurator - Validate and configure VTOL mixer profiles.

Reads an INAV `diff all`, validates the two mixer profiles (MC + FW),
checks for common VTOL configuration mistakes, and optionally runs
an interactive wizard to fix or complete the configuration.

INAV VTOL works by switching between two mixer_profiles:
  - Profile 1 (or 2): Multirotor/Tricopter mode (hover, takeoff, landing)
  - Profile 2 (or 1): Airplane mode (forward flight)
  Switching is via the MIXER PROFILE 2 RC mode.
  Transition mixing (smix source 38) handles tilt servos.

Usage:
    python3 inav_vtol_configurator.py diff_all.txt              # Validate
    python3 inav_vtol_configurator.py diff_all.txt --wizard     # Interactive setup
    python3 inav_vtol_configurator.py diff_all.txt --json       # Machine output
"""

import argparse
import json
import os
import re
import sys
import textwrap

VERSION = "1.0.1"


def _enable_ansi_colors():
    """Enable ANSI color support. Returns True if colors are available."""
    if os.environ.get("NO_COLOR") is not None:
        return False
    if not hasattr(sys.stdout, "isatty") or not sys.stdout.isatty():
        return False
    if sys.platform == "win32":
        try:
            import ctypes
            kernel32 = ctypes.windll.kernel32
            handle = kernel32.GetStdHandle(-11)
            mode = ctypes.c_ulong()
            kernel32.GetConsoleMode(handle, ctypes.byref(mode))
            kernel32.SetConsoleMode(handle, mode.value | 0x0004)
            return True
        except Exception:
            return False
    return True

_ANSI_ENABLED = _enable_ansi_colors()

def _colors():
    """Return (R, B, C, G, Y, RED, DIM) color codes."""
    if _ANSI_ENABLED:
        return ("\033[0m", "\033[1m", "\033[96m", "\033[92m",
                "\033[93m", "\033[91m", "\033[2m")
    return ("", "", "", "", "", "", "")

def _disable_colors():
    global _ANSI_ENABLED
    _ANSI_ENABLED = False

# ─── Severity Levels ─────────────────────────────────────────────────────────

CRITICAL = "CRITICAL"
WARNING = "WARNING"
INFO = "INFO"
OK = "OK"

SEVERITY_ORDER = {CRITICAL: 0, WARNING: 1, INFO: 2, OK: 3}

# ─── INAV Constants ──────────────────────────────────────────────────────────

# Platform types in INAV
PLATFORM_MULTIROTOR = "MULTIROTOR"
PLATFORM_TRICOPTER = "TRICOPTER"
PLATFORM_AIRPLANE = "AIRPLANE"
PLATFORM_TAILSITTER = "TAILSITTER"

MC_PLATFORMS = {PLATFORM_MULTIROTOR, PLATFORM_TRICOPTER, PLATFORM_TAILSITTER}
FW_PLATFORMS = {PLATFORM_AIRPLANE}

# INAV smix input sources (the important ones for VTOL)
SMIX_SOURCES = {
    0: "Stabilised ROLL",
    1: "Stabilised PITCH",
    2: "Stabilised YAW",
    3: "Stabilised THROTTLE",
    4: "RC ROLL",
    5: "RC PITCH",
    6: "RC YAW",
    7: "RC THROTTLE",
    8: "RC AUX 1",
    9: "RC AUX 2",
    10: "RC AUX 3",
    11: "RC AUX 4",
    12: "MAX (always 100%)",
    38: "Mixer Transition",
}

# INAV aux mode IDs (from src/main/fc/rc_modes.h)
# These are the mode_id values used in `aux` commands
INAV_MODE_NAMES = {
    0: "ARM",
    1: "ANGLE",
    2: "HORIZON",
    3: "NAV ALTHOLD",
    5: "HEADING HOLD",
    10: "NAV POSHOLD",
    11: "NAV RTH",
    12: "MANUAL",
    13: "NAV WP",
    28: "NAV LAUNCH",
    45: "TURTLE",
    47: "OSD ALT",
    48: "NAV COURSE HOLD",
    53: "MULTI FUNCTION",
    62: "MIXER PROFILE 2",
    63: "MIXER TRANSITION",
}

MODE_MIXER_PROFILE_2 = 62   # BOXMIXERPROFILE
MODE_MIXER_TRANSITION = 63  # BOXMIXERTRANSITION


# ─── Parser ──────────────────────────────────────────────────────────────────

class Finding:
    """A single validation finding."""
    def __init__(self, severity, category, title, detail, cli_fix=None):
        self.severity = severity
        self.category = category
        self.title = title
        self.detail = detail
        self.cli_fix = cli_fix

    def __repr__(self):
        return f"<{self.severity} {self.category}: {self.title}>"


def parse_diff_all(text):
    """Parse INAV diff all into structured data with full VTOL support."""
    result = {
        "version": None,
        "board": None,
        "master": {},
        "control_profiles": {},
        "mixer_profiles": {},
        "battery_profiles": {},
        "active_control_profile": 1,
        "active_mixer_profile": 1,
        "active_battery_profile": 1,
        "features": [],
        "features_disabled": [],
        "serial_ports": {},
        "aux_modes": [],
        "mmix_by_profile": {},    # {profile_num: [{'index':0, 'throttle':1.0, ...}, ...]}
        "smix_by_profile": {},    # {profile_num: [{'index':0, 'servo':0, 'source':0, ...}, ...]}
        "servo_config": {},       # {servo_index: {'min':..., 'max':..., 'middle':..., 'rate':...}}
        "resources": {},          # {'MOTOR': {1: 'PA0', ...}, 'SERVO': {1: 'PB1', ...}}
    }

    current_section = "master"
    current_profile_type = None
    current_profile_num = 1

    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            m = re.match(r"#\s*INAV/(\S+)\s+([\d.]+)", line)
            if m:
                result["board"] = m.group(1)
                result["version"] = m.group(2)
            continue

        # Features
        if line.startswith("feature "):
            feat = line[8:].strip()
            if feat.startswith("-"):
                result["features_disabled"].append(feat[1:])
            else:
                result["features"].append(feat)
            continue

        # Serial ports
        m = re.match(r"serial\s+(\d+)\s+(.*)", line)
        if m:
            result["serial_ports"][int(m.group(1))] = m.group(2).strip()
            continue

        # Aux modes
        m = re.match(r"aux\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)", line)
        if m:
            result["aux_modes"].append({
                "index": int(m.group(1)),
                "mode_id": int(m.group(2)),
                "channel": int(m.group(3)),
                "range_low": int(m.group(4)),
                "range_high": int(m.group(5)),
            })
            continue

        # Profile switches
        pm = re.match(r"(control_profile|mixer_profile|battery_profile)\s+(\d+)", line)
        if pm:
            current_profile_type = pm.group(1)
            current_profile_num = int(pm.group(2))
            section_map = {
                "control_profile": "control_profiles",
                "mixer_profile": "mixer_profiles",
                "battery_profile": "battery_profiles",
            }
            current_section = section_map[current_profile_type]
            if current_profile_num not in result[current_section]:
                result[current_section][current_profile_num] = {}
            continue

        # Motor mix: mmix <index> <throttle> <roll> <pitch> <yaw>
        mm = re.match(r"mmix\s+(\d+)\s+([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)\s+([\d.-]+)", line)
        if mm:
            entry = {
                "index": int(mm.group(1)),
                "throttle": float(mm.group(2)),
                "roll": float(mm.group(3)),
                "pitch": float(mm.group(4)),
                "yaw": float(mm.group(5)),
            }
            profile = current_profile_num if current_section == "mixer_profiles" else 0
            if profile not in result["mmix_by_profile"]:
                result["mmix_by_profile"][profile] = []
            result["mmix_by_profile"][profile].append(entry)
            continue

        # Servo mix: smix <index> <servo> <source> [<rate> <speed> <min> <max> <box>]
        sm = re.match(r"smix\s+(\d+)\s+(\d+)\s+(\d+)\s*(.*)", line)
        if sm:
            rest = sm.group(4).split()
            entry = {
                "index": int(sm.group(1)),
                "servo": int(sm.group(2)),
                "source": int(sm.group(3)),
                "rate": int(rest[0]) if len(rest) > 0 else 100,
                "speed": int(rest[1]) if len(rest) > 1 else 0,
                "min": int(rest[2]) if len(rest) > 2 else 0,
                "max": int(rest[3]) if len(rest) > 3 else 0,
                "box": int(rest[4]) if len(rest) > 4 else 0,
            }
            profile = current_profile_num if current_section == "mixer_profiles" else 0
            if profile not in result["smix_by_profile"]:
                result["smix_by_profile"][profile] = []
            result["smix_by_profile"][profile].append(entry)
            continue

        # mmix reset / smix reset
        if line in ("mmix reset", "smix reset"):
            continue

        # Servo config: servo <index> <min> <max> <middle> <rate>
        sv = re.match(r"servo\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(-?\d+)", line)
        if sv:
            result["servo_config"][int(sv.group(1))] = {
                "min": int(sv.group(2)),
                "max": int(sv.group(3)),
                "middle": int(sv.group(4)),
                "rate": int(sv.group(5)),
            }
            continue

        # Resources: resource <TYPE> <index> <pin>
        rm = re.match(r"resource\s+(\w+)\s+(\d+)\s+(\S+)", line)
        if rm:
            res_type = rm.group(1)
            if res_type not in result["resources"]:
                result["resources"][res_type] = {}
            result["resources"][res_type][int(rm.group(2))] = rm.group(3)
            continue

        # Channel map
        cm = re.match(r"map\s+(\w+)", line)
        if cm:
            result["master"]["channel_map"] = cm.group(1)
            continue

        # Set commands
        sm2 = re.match(r"set\s+(\S+)\s*=\s*(.*)", line)
        if sm2:
            key = sm2.group(1).strip()
            val = _parse_value(sm2.group(2).strip())
            if current_section == "master":
                result["master"][key] = val
            else:
                result[current_section][current_profile_num][key] = val
            continue

    return result


def _parse_value(val):
    """Parse a string value into appropriate Python type."""
    if val.upper() in ("ON", "TRUE", "YES"):
        return True
    if val.upper() in ("OFF", "FALSE", "NO"):
        return False
    try:
        return int(val)
    except ValueError:
        pass
    try:
        return float(val)
    except ValueError:
        pass
    return val


# ─── VTOL Profile Detection ─────────────────────────────────────────────────

def detect_vtol_profiles(parsed):
    """Detect which mixer profiles are MC and FW."""
    mc_profile = None
    fw_profile = None
    profiles_info = {}

    for num, settings in parsed["mixer_profiles"].items():
        ptype = str(settings.get("platform_type", "")).upper()
        mmix = parsed["mmix_by_profile"].get(num, [])
        smix = parsed["smix_by_profile"].get(num, [])
        active_motors = [m for m in mmix if m["throttle"] != 0 and m["throttle"] != -1]

        profiles_info[num] = {
            "platform_type": ptype or "(not set)",
            "motor_count": len(mmix),
            "active_motors": len(active_motors),
            "servo_rules": len(smix),
            "mmix": mmix,
            "smix": smix,
        }

        if ptype in MC_PLATFORMS:
            mc_profile = num
        elif ptype in FW_PLATFORMS:
            fw_profile = num

    return mc_profile, fw_profile, profiles_info


def detect_motor_roles(parsed, mc_profile, fw_profile):
    """Infer motor roles from mixer profile cross-reference."""
    mc_motors = set()
    fw_motors = set()

    if mc_profile:
        for m in parsed["mmix_by_profile"].get(mc_profile, []):
            if m["throttle"] > 0:
                mc_motors.add(m["index"] + 1)  # 1-based

    if fw_profile:
        for m in parsed["mmix_by_profile"].get(fw_profile, []):
            if m["throttle"] > 0:
                fw_motors.add(m["index"] + 1)

    tilt_motors = mc_motors & fw_motors  # in both = tilt
    lift_only = mc_motors - fw_motors     # MC only = lift
    push_only = fw_motors - mc_motors     # FW only = pusher

    return {
        "mc_motors": mc_motors,
        "fw_motors": fw_motors,
        "tilt_motors": tilt_motors,
        "lift_only": lift_only,
        "push_only": push_only,
    }


def detect_tilt_servos(parsed, mc_profile):
    """Find servo mix rules using Mixer Transition (source 38)."""
    tilt_rules = []
    if mc_profile:
        for rule in parsed["smix_by_profile"].get(mc_profile, []):
            if rule["source"] == 38:  # Mixer Transition
                tilt_rules.append(rule)
    return tilt_rules


# ─── Validation Checks ──────────────────────────────────────────────────────

def run_vtol_checks(parsed):
    """Run all VTOL-specific validation checks."""
    findings = []
    mc_profile, fw_profile, profiles_info = detect_vtol_profiles(parsed)

    # ── Profile existence ────────────────────────────────────────────────
    if mc_profile is None and fw_profile is None:
        findings.append(Finding(
            INFO, "Profiles", "No VTOL profiles detected",
            "Neither mixer profile has platform_type set to a multirotor or airplane type. "
            "This doesn't appear to be a VTOL configuration. If it should be, set "
            "platform_type in each mixer_profile."))
        return findings

    if mc_profile is None:
        findings.append(Finding(
            CRITICAL, "Profiles", "No multirotor mixer profile found",
            "VTOL requires one mixer_profile with platform_type set to MULTIROTOR, "
            "TRICOPTER, or TAILSITTER for hover/takeoff/landing.",
            cli_fix=f"mixer_profile 1\nset platform_type = TRICOPTER"))
    else:
        ptype = profiles_info[mc_profile]["platform_type"]
        n_motors = profiles_info[mc_profile]["active_motors"]
        n_smix = profiles_info[mc_profile]["servo_rules"]
        findings.append(Finding(
            OK, "Profiles", f"MC profile: mixer_profile {mc_profile} ({ptype})",
            f"{n_motors} active motors, {n_smix} servo mix rules."))

    if fw_profile is None:
        findings.append(Finding(
            CRITICAL, "Profiles", "No airplane mixer profile found",
            "VTOL requires one mixer_profile with platform_type = AIRPLANE for forward flight.",
            cli_fix=f"mixer_profile 2\nset platform_type = AIRPLANE"))
    else:
        n_motors = profiles_info[fw_profile]["active_motors"]
        n_smix = profiles_info[fw_profile]["servo_rules"]
        findings.append(Finding(
            OK, "Profiles", f"FW profile: mixer_profile {fw_profile} (AIRPLANE)",
            f"{n_motors} active motors, {n_smix} servo mix rules."))

    if mc_profile is None or fw_profile is None:
        return findings

    # ── Motor mix validation ─────────────────────────────────────────────
    roles = detect_motor_roles(parsed, mc_profile, fw_profile)

    if not roles["mc_motors"]:
        findings.append(Finding(
            CRITICAL, "Motor Mix", "MC profile has no active motors",
            "No motors with throttle > 0 in the multirotor mixer profile."))

    if not roles["fw_motors"]:
        findings.append(Finding(
            CRITICAL, "Motor Mix", "FW profile has no active motors",
            "No motors with throttle > 0 in the airplane mixer profile."))

    mc_mmix = parsed["mmix_by_profile"].get(mc_profile, [])
    fw_mmix = parsed["mmix_by_profile"].get(fw_profile, [])

    # Tricopter-specific: should have exactly 3 motors
    mc_ptype = str(parsed["mixer_profiles"][mc_profile].get("platform_type", "")).upper()
    if mc_ptype == PLATFORM_TRICOPTER:
        active = [m for m in mc_mmix if m["throttle"] > 0]
        if len(active) != 3:
            findings.append(Finding(
                WARNING, "Motor Mix",
                f"Tricopter profile has {len(active)} active motors (expected 3)",
                "A tricopter should have exactly 3 motors with throttle > 0."))

    # Check yaw authority in MC profile
    yaw_motors = [m for m in mc_mmix if m["yaw"] != 0 and m["throttle"] > 0]
    if mc_ptype == PLATFORM_TRICOPTER:
        # Tricopter uses tilt servo for yaw, motor yaw mix is optional
        pass
    elif not yaw_motors:
        findings.append(Finding(
            WARNING, "Motor Mix", "MC profile: no motor yaw mixing",
            "No active motors have a yaw factor. Yaw authority in hover may be insufficient."))

    # Check FW motor mix
    fw_active = [m for m in fw_mmix if m["throttle"] > 0]
    if len(fw_active) == 1:
        # Single pusher - normal
        findings.append(Finding(
            OK, "Motor Mix", f"FW profile: single motor (pusher)",
            f"Motor {fw_active[0]['index']+1} is the forward-flight motor."))
    elif len(fw_active) >= 2:
        # Multi-motor FW - check for differential thrust
        yaw_fw = [m for m in fw_active if m["yaw"] != 0]
        if not yaw_fw:
            findings.append(Finding(
                INFO, "Motor Mix", "FW profile: multiple motors but no yaw mixing",
                "With multiple motors in airplane mode, consider adding yaw factors "
                "for differential thrust (helps with rudder-less turns)."))

    # Motor roles summary
    if roles["tilt_motors"]:
        findings.append(Finding(
            OK, "Motor Roles",
            f"Tilt motors: {sorted(roles['tilt_motors'])} (used in both MC and FW profiles)",
            "These motors are active in both profiles - they tilt between hover and forward flight."))
    if roles["lift_only"]:
        findings.append(Finding(
            OK, "Motor Roles",
            f"Lift-only motors: {sorted(roles['lift_only'])} (MC profile only)",
            "These motors are only active in hover mode."))
    if roles["push_only"]:
        findings.append(Finding(
            OK, "Motor Roles",
            f"Pusher motors: {sorted(roles['push_only'])} (FW profile only)",
            "These motors are only active in forward flight."))

    # Placeholder motors in FW profile (throttle = -1)
    placeholders = [m for m in fw_mmix if m["throttle"] == -1]
    if placeholders:
        findings.append(Finding(
            OK, "Motor Mix",
            f"FW profile: {len(placeholders)} placeholder motor(s) (throttle=-1)",
            "Placeholder entries keep motor indices aligned between profiles. Good."))

    # Transition motors (throttle between -2 and -1)
    transition_motors = [m for m in mc_mmix if -2.0 < m["throttle"] < -1.0]
    if transition_motors:
        for tm in transition_motors:
            speed_pct = (abs(tm["throttle"]) - 1.0) * 100
            findings.append(Finding(
                OK, "Motor Mix",
                f"Transition motor {tm['index']+1}: spins at {speed_pct:.0f}% during transition",
                "This motor activates only when MIXER TRANSITION is engaged (for gaining airspeed)."))

    # ── Servo mix validation ─────────────────────────────────────────────
    tilt_rules = detect_tilt_servos(parsed, mc_profile)

    if mc_ptype in (PLATFORM_TRICOPTER,) and not tilt_rules:
        findings.append(Finding(
            WARNING, "Servo Mix", "No tilt servo rules found (source 38) in MC profile",
            "Tilt-rotor VTOLs need servo mix rules with source 38 (Mixer Transition) "
            "to control the tilt angle during transition. Without this, motors won't "
            "tilt for forward flight.",
            cli_fix=f"mixer_profile {mc_profile}\n"
                     f"# Example: tilt servo 3 by +45° during transition\n"
                     f"smix 0 3 38 45 150 -1"))
    elif tilt_rules:
        servos_used = sorted(set(r["servo"] for r in tilt_rules))
        for rule in tilt_rules:
            src_name = SMIX_SOURCES.get(rule["source"], f"source {rule['source']}")
            findings.append(Finding(
                OK, "Servo Mix",
                f"Tilt rule: servo {rule['servo']} ← {src_name} "
                f"(rate={rule['rate']}, speed={rule['speed']})",
                f"Servo {rule['servo']} tilts during transition."))

        # Check for yaw mixing on tilt servos (tricopter yaw via tilt)
        if mc_ptype == PLATFORM_TRICOPTER:
            mc_smix = parsed["smix_by_profile"].get(mc_profile, [])
            yaw_on_tilt = [r for r in mc_smix
                           if r["servo"] in servos_used and r["source"] == 2]
            if not yaw_on_tilt:
                findings.append(Finding(
                    WARNING, "Servo Mix",
                    "No yaw (source 2) mixed onto tilt servos",
                    "Tricopters typically use tilt servos for yaw control in hover. "
                    "Without yaw mixing on the tilt servos, you may have no yaw authority "
                    "in multirotor mode.",
                    cli_fix=f"mixer_profile {mc_profile}\n"
                             f"smix <next_index> {servos_used.pop()} 2 100 0 -1"))
            else:
                findings.append(Finding(
                    OK, "Servo Mix", "Yaw mixed onto tilt servos",
                    "Tilt servos provide yaw authority in hover mode. Good."))

    # Check FW control surfaces
    fw_smix = parsed["smix_by_profile"].get(fw_profile, [])
    fw_has_roll = any(r["source"] == 0 for r in fw_smix)   # stabilised roll
    fw_has_pitch = any(r["source"] == 1 for r in fw_smix)  # stabilised pitch
    fw_has_yaw = any(r["source"] == 2 for r in fw_smix)    # stabilised yaw

    if not fw_has_roll:
        findings.append(Finding(
            WARNING, "Servo Mix", "FW profile: no roll control surface",
            "Airplane profile has no servo mixed with Stabilised Roll (source 0). "
            "Without ailerons or elevons, the aircraft can't bank in forward flight."))
    if not fw_has_pitch:
        findings.append(Finding(
            WARNING, "Servo Mix", "FW profile: no pitch control surface",
            "Airplane profile has no servo mixed with Stabilised Pitch (source 1). "
            "Without elevator, the aircraft can't control pitch in forward flight."))
    if fw_has_roll and fw_has_pitch:
        findings.append(Finding(
            OK, "Servo Mix", "FW profile has roll and pitch control surfaces",
            "Airplane mode has both roll and pitch authority."))

    # ── Mode switch validation ───────────────────────────────────────────
    has_profile_switch = any(
        m["mode_id"] == MODE_MIXER_PROFILE_2 for m in parsed["aux_modes"])
    has_transition_switch = any(
        m["mode_id"] == MODE_MIXER_TRANSITION for m in parsed["aux_modes"])

    if not has_profile_switch:
        findings.append(Finding(
            CRITICAL, "Modes", "MIXER PROFILE 2 mode not assigned to any switch",
            "Without this mode, you cannot switch between MC and FW profiles in flight. "
            "Assign MIXER PROFILE 2 (mode 62) to an aux channel in the Modes tab.",
            cli_fix="# Example: aux channel 4, high position\n"
                     "aux <next_index> 62 4 1700 2100"))
    else:
        mode = next(m for m in parsed["aux_modes"] if m["mode_id"] == MODE_MIXER_PROFILE_2)
        findings.append(Finding(
            OK, "Modes",
            f"MIXER PROFILE 2 on channel {mode['channel']} "
            f"({mode['range_low']}-{mode['range_high']})",
            "Profile switching is configured."))

    if not has_transition_switch:
        findings.append(Finding(
            WARNING, "Modes", "MIXER TRANSITION mode not assigned",
            "MIXER TRANSITION activates tilt servos and transition motors while in MC mode. "
            "Without it, tilt-rotor transition won't work. Usually mapped to a 3-position "
            "switch: low=MC, mid=transition, high=FW profile.",
            cli_fix="# Example: aux channel 4, mid position\n"
                     "aux <next_index> 63 4 1300 1700"))
    else:
        mode = next(m for m in parsed["aux_modes"] if m["mode_id"] == MODE_MIXER_TRANSITION)
        findings.append(Finding(
            OK, "Modes",
            f"MIXER TRANSITION on channel {mode['channel']} "
            f"({mode['range_low']}-{mode['range_high']})",
            "Transition mixing is configured."))

    # ── Automated transition (RTH) ───────────────────────────────────────
    mc_settings = parsed["mixer_profiles"].get(mc_profile, {})
    fw_settings = parsed["mixer_profiles"].get(fw_profile, {})

    mc_auto = mc_settings.get("mixer_automated_switch", False)
    fw_auto = fw_settings.get("mixer_automated_switch", False)
    mc_timer = mc_settings.get("mixer_switch_trans_timer", 0)

    if mc_auto and fw_auto:
        findings.append(Finding(
            OK, "Transition",
            f"Automated RTH transition enabled (timer: {mc_timer/10:.1f}s)",
            "On RTH, the quad will gain airspeed in MC mode, transition to FW for "
            "efficient cruise, then switch back to MC for landing."))
    elif mc_auto and not fw_auto:
        findings.append(Finding(
            WARNING, "Transition",
            "Automated switch ON in MC profile but OFF in FW profile",
            "For full automated RTH transition, both profiles need mixer_automated_switch = ON. "
            "MC profile switches to FW for cruise, FW profile switches back to MC for landing.",
            cli_fix=f"mixer_profile {fw_profile}\nset mixer_automated_switch = ON"))
    elif not mc_auto and not fw_auto:
        findings.append(Finding(
            INFO, "Transition", "No automated transition configured",
            "Automated switching handles MC↔FW transitions during RTH automatically. "
            "Consider enabling it for hands-off return-to-home behavior.",
            cli_fix=f"mixer_profile {mc_profile}\n"
                     f"set mixer_automated_switch = ON\n"
                     f"set mixer_switch_trans_timer = 30\n"
                     f"mixer_profile {fw_profile}\n"
                     f"set mixer_automated_switch = ON"))

    # ── Control profile linking ──────────────────────────────────────────
    mc_linking = mc_settings.get("mixer_control_profile_linking", False)
    fw_linking = fw_settings.get("mixer_control_profile_linking", False)

    if mc_linking and fw_linking:
        findings.append(Finding(
            OK, "Profiles", "Control profile linking enabled in both mixer profiles",
            "PIDs and rates will automatically switch when changing mixer profiles. "
            "MC PIDs for hover, FW PIDs for forward flight."))
    elif not mc_linking or not fw_linking:
        missing = []
        if not mc_linking:
            missing.append(f"MC (profile {mc_profile})")
        if not fw_linking:
            missing.append(f"FW (profile {fw_profile})")
        findings.append(Finding(
            WARNING, "Profiles",
            f"Control profile linking not enabled in: {', '.join(missing)}",
            "Without mixer_control_profile_linking = ON, PIDs and rates won't "
            "automatically switch when changing between MC and FW modes. You may end up "
            "flying a multirotor with airplane PIDs or vice versa.",
            cli_fix="\n".join(
                [f"mixer_profile {p}\nset mixer_control_profile_linking = ON"
                 for p in ([mc_profile] if not mc_linking else []) +
                          ([fw_profile] if not fw_linking else [])])))

    # ── Safety checks ────────────────────────────────────────────────────
    # Airmode type for transition motors
    airmode = parsed["master"].get("airmode_type", "STICK_CENTER")
    transition_motors = [m for m in mc_mmix if -2.0 < m["throttle"] < -1.0]
    if transition_motors and str(airmode).upper() == "THROTTLE_THRESHOLD":
        findings.append(Finding(
            CRITICAL, "Safety",
            "airmode_type is THROTTLE_THRESHOLD with transition motors",
            "Transition motors (throttle between -2.0 and -1.0) require "
            "airmode_type = STICK_CENTER. With THROTTLE_THRESHOLD, transition motors "
            "will spin at the wrong times.",
            cli_fix="set airmode_type = STICK_CENTER"))

    # Compass required for MC nav modes
    mag_hw = parsed["master"].get("mag_hardware", None)
    if mag_hw is None or str(mag_hw).upper() in ("NONE", "0", "FALSE"):
        findings.append(Finding(
            WARNING, "Safety", "No compass configured",
            "INAV requires a compass for navigation modes in multirotor profile. "
            "Without it, GPS position hold and RTH won't work in hover mode."))

    # MC profile should have some control surface mixing for high-speed recovery
    mc_smix = parsed["smix_by_profile"].get(mc_profile, [])
    mc_has_surfaces = any(r["source"] in (0, 1) for r in mc_smix)
    if not mc_has_surfaces and fw_has_roll:
        findings.append(Finding(
            INFO, "Safety",
            "Consider adding control surface mixing to MC profile",
            "Having some aileron/elevator authority in multirotor mode helps recovery "
            "if the aircraft enters a high-speed dive during transition. The INAV docs "
            "recommend mapping control surfaces in both profiles."))

    # Sort by severity
    findings.sort(key=lambda f: SEVERITY_ORDER.get(f.severity, 99))
    return findings


# ─── Terminal Output ─────────────────────────────────────────────────────────

def print_report(parsed, findings):
    """Print validation report to terminal."""
    R, B, C, G, Y, RED, DIM = _colors()
    sev_color = {CRITICAL: RED, WARNING: Y, INFO: C, OK: G}
    sev_icon = {CRITICAL: "✗", WARNING: "⚠", INFO: "ℹ", OK: "✓"}

    print(f"\n{B}{C}{'═'*70}{R}")
    print(f"{B}{C}  INAV VTOL Configurator v{VERSION}{R}")
    print(f"{B}{C}{'═'*70}{R}")

    if parsed["version"]:
        print(f"  {DIM}Firmware: INAV {parsed['version']} | Board: {parsed['board']}{R}")

    # Profile summary
    mc_profile, fw_profile, profiles_info = detect_vtol_profiles(parsed)
    for num, info in sorted(profiles_info.items()):
        print(f"  {DIM}mixer_profile {num}: {info['platform_type']} "
              f"({info['active_motors']} motors, {info['servo_rules']} servo rules){R}")

    if mc_profile and fw_profile:
        roles = detect_motor_roles(parsed, mc_profile, fw_profile)
        if roles["tilt_motors"]:
            print(f"  {DIM}VTOL type: Tilt-rotor{R}")
        elif roles["push_only"]:
            print(f"  {DIM}VTOL type: Separate lift + pusher{R}")

    # Summary counts
    counts = {CRITICAL: 0, WARNING: 0, INFO: 0, OK: 0}
    for f in findings:
        counts[f.severity] = counts.get(f.severity, 0) + 1

    print(f"\n  {B}SUMMARY:{R}")
    if counts[CRITICAL]:
        print(f"    {RED}{B}{counts[CRITICAL]} CRITICAL{R} - fix before flying")
    if counts[WARNING]:
        print(f"    {Y}{B}{counts[WARNING]} WARNING{R} - should address")
    if counts[INFO]:
        print(f"    {C}{counts[INFO]} suggestions{R}")
    if counts[OK]:
        print(f"    {G}{counts[OK]} checks passed{R}")

    # Non-OK findings
    has_issues = any(f.severity != OK for f in findings)
    if has_issues:
        print(f"\n{B}{C}{'─'*70}{R}")
        print(f"  {B}FINDINGS:{R}")
        print(f"{B}{C}{'─'*70}{R}")

        categories = {}
        for f in findings:
            if f.severity == OK:
                continue
            if f.category not in categories:
                categories[f.category] = []
            categories[f.category].append(f)

        for cat, cat_findings in categories.items():
            print(f"\n  {B}{cat}{R}")
            for f in cat_findings:
                sc = sev_color[f.severity]
                icon = sev_icon[f.severity]
                print(f"    {sc}{B}{icon}{R} {B}{f.title}{R}")
                for line in textwrap.wrap(f.detail, width=62):
                    print(f"      {DIM}{line}{R}")

    # CLI fixes
    fixes = [f for f in findings if f.cli_fix and f.severity in (CRITICAL, WARNING)]
    if fixes:
        print(f"\n{B}{C}{'─'*70}{R}")
        print(f"  {B}SUGGESTED CLI FIXES:{R}")
        print(f"{B}{C}{'─'*70}{R}")
        print()
        for f in fixes:
            print(f"  {DIM}# {f.title}{R}")
            for cmd in f.cli_fix.split("\n"):
                print(f"    {G}{cmd}{R}")
            print()
        print(f"    {G}save{R}")

    # Passed
    ok_items = [f for f in findings if f.severity == OK]
    if ok_items:
        print(f"\n{B}{C}{'─'*70}{R}")
        print(f"  {B}PASSED:{R}")
        for f in ok_items:
            print(f"    {G}✓{R} {DIM}{f.title}{R}")

    print(f"\n{B}{C}{'═'*70}{R}\n")


# ─── Main ────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description=f"INAV VTOL Configurator v{VERSION} - Validate VTOL mixer profiles",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=textwrap.dedent("""\
            Validates INAV VTOL configuration from a diff all file.
            Checks mixer profiles, motor/servo mixing, mode assignments,
            and transition settings for common mistakes.
        """))
    parser.add_argument("difffile", help="INAV `diff all` output file")
    parser.add_argument("--json", action="store_true",
                        help="Output findings as JSON")
    parser.add_argument("--no-color", action="store_true",
                        help="Disable colored terminal output.")
    args = parser.parse_args()

    if args.no_color:
        _disable_colors()

    if not os.path.isfile(args.difffile):
        print(f"ERROR: File not found: {args.difffile}")
        sys.exit(1)

    with open(args.difffile, "r", errors="replace") as f:
        text = f.read()

    if not args.json:
        print(f"\n  ▲ INAV VTOL Configurator v{VERSION}")
        print(f"  Loading: {args.difffile}")

    parsed = parse_diff_all(text)

    if not args.json and parsed["version"]:
        print(f"  Firmware: INAV {parsed['version']} on {parsed['board']}")

    findings = run_vtol_checks(parsed)

    if args.json:
        output = [{
            "severity": f.severity,
            "category": f.category,
            "title": f.title,
            "detail": f.detail,
            "cli_fix": f.cli_fix,
        } for f in findings]
        print(json.dumps(output, indent=2))
    else:
        print_report(parsed, findings)


if __name__ == "__main__":
    main()
