# VTOL Configurator - Detailed Documentation

## Overview

The INAV VTOL Configurator validates VTOL configurations that use INAV's mixer_profile switching. It checks that both flight modes (multirotor and airplane) are properly configured and that the transition between them is safe.

## How INAV VTOL Works

INAV VTOL uses two **mixer profiles**:

- **mixer_profile 1** (default): One flight mode (typically MC for hover/takeoff/landing)
- **mixer_profile 2**: The other flight mode (typically FW for forward flight)

Switching is controlled by:
- **MIXER PROFILE 2** (mode 62): RC switch to select airplane profile
- **MIXER TRANSITION** (mode 63): Activates tilt servos and transition motors

Each mixer profile contains its own `platform_type`, motor mix (`mmix`), and servo mix (`smix`).

## Usage

```bash
# Validate VTOL config
python3 inav_vtol_configurator.py vtol_diff.txt

# JSON output for automation
python3 inav_vtol_configurator.py vtol_diff.txt --json
```

## VTOL Types Supported

### Tilt-Rotor
Motors physically tilt between vertical (hover) and horizontal (forward flight). Same motors used for lift and cruise. Tilt servos controlled via smix source 38 (Mixer Transition).

**Detection:** Motors appear in both MC and FW mmix → tilt motors.

### Separate Lift + Pusher
Dedicated lift motors (MC only) and pusher motor(s) (FW only). Common in quadplane configurations.

**Detection:** Motors in MC mmix only → lift. Motors in FW mmix only → pusher.

### Tailsitter
Uses `platform_type = TAILSITTER` which adds a 90° board alignment offset. The aircraft sits on its tail for takeoff and flips forward for cruise.

## Validation Checks

### Profile Validation
- Both MC and FW mixer profiles exist with correct `platform_type`
- Motor count appropriate for platform (e.g., tricopter = 3)
- Active motor verification (throttle > 0)

### Motor Mix Analysis
- **Tilt motors:** Appear in both profiles with positive throttle
- **Lift-only motors:** MC profile only (hover-dedicated)
- **Pusher motors:** FW profile only (forward flight)
- **Placeholder motors:** `throttle = -1` keeps indices aligned between profiles
- **Transition motors:** `-2.0 < throttle < -1.0` spin only during MIXER TRANSITION at `(abs(throttle) - 1) × 100%`
- **Yaw authority:** Motor yaw factors present (or tilt servo yaw for tricopters)

### Servo Mix Validation
- **Tilt servo rules:** smix with source 38 (Mixer Transition) present in MC profile
- **Yaw on tilt servos:** Tricopters need yaw (source 2) mixed onto tilt servos for hover yaw control
- **FW control surfaces:** Roll (source 0) and pitch (source 1) servo mixing in airplane profile
- **MC control surfaces:** Recommends adding aileron/elevator in MC profile for dive recovery

### Mode Switch Validation
- **MIXER PROFILE 2** (mode 62) assigned to an aux channel
- **MIXER TRANSITION** (mode 63) assigned (usually same switch, mid position)
- Validates channel and range assignments

### Transition Configuration
- **Automated RTH:** `mixer_automated_switch = ON` in both profiles for hands-off return
- **Transition timer:** `mixer_switch_trans_timer` for airspeed buildup time
- **Control profile linking:** `mixer_control_profile_linking = ON` ensures PIDs auto-switch

### Safety Checks
- **airmode_type:** Must be `STICK_CENTER` (not `THROTTLE_THRESHOLD`) when transition motors are used
- **Compass:** Required for MC navigation modes (GPS hold, RTH in hover)
- **MC control surfaces:** Having aileron/elevator in MC profile aids recovery from transition dives

## Common VTOL Issues

### Transition Motors Spinning Unexpectedly
`airmode_type = THROTTLE_THRESHOLD` causes transition motors (negative throttle) to activate based on throttle position instead of MIXER TRANSITION switch. Fix: `set airmode_type = STICK_CENTER`

### No Tilt During Transition
Missing smix source 38 rules in the MC profile. The tilt servos won't move when MIXER TRANSITION is activated.

### Wrong PIDs After Profile Switch
`mixer_control_profile_linking = OFF` means PIDs don't change when switching MC ↔ FW. You'd be flying a multirotor with airplane PIDs.

### Yaw Drift in Hover (Tricopter)
No yaw mixing on tilt servos. Tricopters use tilt servo differential for yaw, not motor yaw factors.

## Example: Tilt-Rotor Tricopter

```
mixer_profile 1                           # Hover mode
set platform_type = TRICOPTER
set mixer_control_profile_linking = ON
set mixer_automated_switch = ON
set mixer_switch_trans_timer = 30         # 3 seconds for airspeed

mmix 0  1.000  0.000  1.333  0.000       # Rear motor
mmix 1  1.000 -1.000 -0.667  0.000       # Front left
mmix 2  1.000  1.000 -0.667  0.000       # Front right

smix 0 3 2 100 0 -1                      # Yaw → left tilt servo
smix 1 4 2 -100 0 -1                     # Yaw → right tilt servo (reversed)
smix 2 3 38 45 150 -1                    # Transition → left tilt (45°, speed 150)
smix 3 4 38 45 150 -1                    # Transition → right tilt

mixer_profile 2                           # Forward flight
set platform_type = AIRPLANE
set mixer_control_profile_linking = ON
set mixer_automated_switch = ON

mmix 0  1.000  0.000  0.000  0.100       # Left motor (differential thrust)
mmix 1  1.000  0.000  0.000 -0.100       # Right motor

smix 0 1 0 50 0 -1                       # Left elevon: roll + pitch
smix 1 1 1 50 0 -1
smix 2 2 0 -50 0 -1                      # Right elevon: -roll + pitch
smix 3 2 1 50 0 -1
```
