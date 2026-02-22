# INAV Tuning Workflow - Step by Step

## Phase 1: New Build Configuration

### 1.1 Generate Starting PIDs

```bash
python3 inav_param_analyzer.py --setup 10 --voltage 6S
```

This outputs conservative PIDs, filters, and settings for your frame. Copy the CLI commands into INAV Configurator's CLI tab.

**Why conservative?** Too-low PIDs = sluggish but flyable. Too-high PIDs = oscillation and potential crash. We start low and increase from blackbox data.

### 1.2 Essential Settings

Before first flight, ensure these are configured:
- Failsafe → RTH (not DROP)
- Battery limits appropriate for your chemistry (LiPo vs Li-ion)
- At minimum BAT_CRIT_LOW and RX_LOST beepers enabled
- GPS + compass calibrated
- Motor direction and prop rotation verified

### 1.3 Validate Configuration

```bash
python3 inav_param_analyzer.py my_diff.txt --frame 10
```

Fix all CRITICAL findings before flying. Address WARNINGs where practical.

## Phase 2: Blackbox Setup

### 2.1 Configure Blackbox Fields

```
# In INAV CLI:
blackbox GYRO_RAW
blackbox RC_COMMAND
blackbox MOTORS
blackbox NAV_PID
blackbox NAV_POS
blackbox NAV_ACC
blackbox MAG
blackbox ACC
blackbox ATTI
blackbox PEAKS_R
blackbox PEAKS_P
blackbox PEAKS_Y
blackbox -RC_DATA
blackbox -SERVOS
save
```

### 2.2 Logging Rate

Keep `blackbox_rate_denom = 2` (half rate). This gives good analysis resolution while conserving flash. A 32MB flash lasts ~20-30 minutes at half rate with all fields.

### 2.3 Verify Flash Space

After configuring fields, do a short 30-second hover and check the log size. This tells you your actual per-minute rate.

## Phase 3: Flight Data Collection

### 3.1 PID Tuning Flight (~2 min)

In manual/acro mode:
1. **Roll:** 3-4 quick snap rolls left, then right. Return to level between each.
2. **Pitch:** 3-4 sharp pitch pumps up, then down.
3. **Yaw:** 3-4 yaw snaps left, then right.
4. **Throttle:** 2-3 punch-outs to full throttle, then chop back.

**Key:** Make discrete, separated moves. The analyzer needs clean step inputs with settling time between them. Continuous freestyle stick movement makes step detection difficult.

### 3.2 Navigation Flight (~3-5 min)

1. **Altitude hold:** Hover for 30s in alt-hold
2. **Position hold:** 30s in GPS pos-hold
3. **RTH:** Fly out 100-200m, trigger RTH
4. Let it complete the full return and landing

This data feeds the future navigation analyzer.

## Phase 4: Analysis

### 4.1 Blackbox Analysis

```bash
python3 inav_blackbox_analyzer.py flight.bbl --frame 10
```

Open the HTML report. Key metrics to evaluate:

| Metric | Good | Needs Work | Problem |
|--------|------|------------|---------|
| Overshoot | <10% | 10-25% | >25% |
| Delay | <30ms | 30-60ms | >60ms |
| RMS Error | <30 deg/s | 30-80 | >80 |
| Motor Saturation | <5% | 5-15% | >15% |

### 4.2 Apply Recommendations

The analyzer generates CLI commands. Review them - don't blindly paste:
- PID changes are typically small increments (±10-20%)
- Filter changes should be verified against your noise profile
- Some recommendations may conflict - use judgment

### 4.3 Cross-Reference

```bash
python3 inav_param_analyzer.py my_diff.txt --blackbox state.json
```

This catches cases where your config and flight behavior disagree (e.g., EZ Tune overwriting manual PIDs).

## Phase 5: Iterate

1. Apply the recommended changes
2. Fly another blackbox session
3. Analyze again
4. Compare with previous results

**Typical progression:**
- Flight 1: Conservative PIDs → sluggish, stable. Analyzer says "increase P"
- Flight 2: Higher P → more responsive, maybe some overshoot. Analyzer says "increase D slightly"
- Flight 3: Better P/D balance → clean response. Analyzer says "tune I for position holding"
- Flight 4-5: Fine-tuning. Tune quality score should improve each iteration

## Tips

- **One axis at a time:** If the analyzer recommends changes to roll AND pitch, apply both - they're usually related.
- **Don't chase perfection:** A tune quality of 70-80% is excellent for a long-range quad. Racing-level tuning (>90%) requires much more aggressive PIDs that trade stability for responsiveness.
- **Wind matters:** Tune in calm conditions first. Wind affects all measurements.
- **Props matter:** After changing props, the tune will shift. Re-analyze.
- **Weight matters:** Significant payload changes (camera, different battery) affect the tune. The heavier the quad, the more I-term matters.
