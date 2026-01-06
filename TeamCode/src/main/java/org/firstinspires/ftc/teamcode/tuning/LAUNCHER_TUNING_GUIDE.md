# Launcher Tuning Guide

## Overview

This guide helps you systematically tune your launcher to shoot 3 balls consistently from any position on the field. The tuning addresses:

1. **Initial RPM** - The flywheel velocity needed to reach the goal from each distance
2. **RPM Drop** - How much velocity is lost when each ball is shot
3. **Hood/Visor Compensation** - Adjusting launch angle to compensate for RPM drop

---

## Understanding the Physics

### Why RPM Drops After Each Shot
When a ball is launched, it absorbs energy from the flywheel, causing the RPM to drop. The flywheel motor tries to recover, but there's a delay. Instead of waiting for full recovery (which is slow), we compensate by adjusting the hood angle.

### How Hood Angle Compensates
- **Higher hood angle** = steeper launch = more height, less distance
- **Lower hood angle** = flatter launch = less height, more distance

When RPM drops, the ball has less energy, so we **increase the hood angle** to give it a steeper trajectory that uses the available energy more efficiently.

---

## Tuning Setup

### Equipment Needed
- Robot with launcher mechanism
- 3+ balls of each color
- FTC Dashboard connected (recommended)
- Measuring tape or known field positions
- Notebook or phone for recording values

### Distance Zones
Define your 3 distance zones based on your field:

| Zone | Description | Suggested Range |
|------|-------------|-----------------|
| CLOSE | Near the goal | 24-48 inches |
| MID | Mid-field | 48-84 inches |
| FAR | Far side | 84-120 inches |

---

## Tuning Procedure

### Step 1: Establish Baseline RPM for Each Distance

For each distance zone (CLOSE, MID, FAR):

1. Position robot at the **center** of that distance zone
2. Load 3 balls into the launcher
3. Run "Launcher Tuning" OpMode
4. Select the distance category (D-Pad Left/Right)
5. Press X to spin up the flywheel
6. Adjust velocity (Left Stick Y) until Ball 1 hits center of goal
7. **Record the velocity** - this is your baseline

**Tip:** Start with the default values and adjust from there.

### Step 2: Measure RPM Drop

For each distance zone:

1. Spin flywheel to baseline RPM
2. Wait for velocity to stabilize
3. Press A to shoot Ball 1
4. Note the "RPM After" and "RPM Drop" in telemetry
5. **Repeat for Balls 2 and 3** without resetting
6. Record all RPM drop values

**Expected Results:**
| Shot | Typical RPM Drop |
|------|------------------|
| Ball 1 | 50-150 TPS |
| Ball 2 | 50-150 TPS |
| Ball 3 | 50-150 TPS |

### Step 3: Tune Visor Position for Each Ball

For each distance zone:

1. **Ball 1:** Use baseline visor position (already tuned for full RPM)
2. **Ball 2:** After shooting Ball 1, adjust visor (Right Stick Y) until Ball 2 hits
   - Typically need to INCREASE visor position slightly
3. **Ball 3:** After shooting Ball 2, adjust visor until Ball 3 hits
   - Typically need to INCREASE visor position more

**Record all 3 visor positions for each distance zone.**

### Step 4: Verify and Fine-Tune

1. Press B to reset sequence
2. Shoot all 3 balls with your tuned values
3. Observe results:
   - All 3 hit? → You're done with this distance!
   - Some miss? → Use Fine adjustment mode (Right Bumper)
4. Repeat until consistent

---

## Recording Sheet

Use this template to record your tuning values:

### CLOSE Distance (_____ inches)

| Parameter | Value |
|-----------|-------|
| Baseline Velocity | _____ TPS |
| Ball 1 RPM Drop | _____ TPS |
| Ball 2 RPM Drop | _____ TPS |
| Ball 3 RPM Drop | _____ TPS |
| Visor Ball 1 | _____ |
| Visor Ball 2 | _____ |
| Visor Ball 3 | _____ |

### MID Distance (_____ inches)

| Parameter | Value |
|-----------|-------|
| Baseline Velocity | _____ TPS |
| Ball 1 RPM Drop | _____ TPS |
| Ball 2 RPM Drop | _____ TPS |
| Ball 3 RPM Drop | _____ TPS |
| Visor Ball 1 | _____ |
| Visor Ball 2 | _____ |
| Visor Ball 3 | _____ |

### FAR Distance (_____ inches)

| Parameter | Value |
|-----------|-------|
| Baseline Velocity | _____ TPS |
| Ball 1 RPM Drop | _____ TPS |
| Ball 2 RPM Drop | _____ TPS |
| Ball 3 RPM Drop | _____ TPS |
| Visor Ball 1 | _____ |
| Visor Ball 2 | _____ |
| Visor Ball 3 | _____ |

---

## Transferring Values to Code

After tuning, press **Left Bumper** to save values to the log. The log will output copy-paste ready code:

```java
// Example output for MID distance:
FLYWHEEL_POWER_COEFFICIENT_MID = 0.5000;
VISOR_POSITION_MID_1 = 0.240;
VISOR_POSITION_MID_2 = 0.260;
VISOR_POSITION_MID_3 = 0.280;
```

Copy these values to `LaunchSystem.java`.

---

## Troubleshooting

### Ball goes too high
- Decrease visor position
- Or decrease flywheel velocity

### Ball goes too low / falls short
- Increase visor position
- Or increase flywheel velocity

### Inconsistent shots
- Check ball condition (worn balls behave differently)
- Verify flywheel is reaching target RPM before shot
- Check for mechanical issues (loose belts, friction)

### Large RPM drops
- Consider increasing flywheel motor power
- Or accept larger visor compensation between shots

### Ball 3 consistently misses
- RPM may not recover enough between shots
- Try adding a small delay between shots in the launch sequence
- Or increase the visor compensation more aggressively

---

## Advanced: Continuous Distance Tuning

Once you have the 3 zones tuned, you can interpolate values for in-between distances. The system already does this based on Limelight distance detection, but you may want to add more zones for better accuracy.

Consider adding additional zones:
- VERY_CLOSE (under 24 inches)
- CLOSE_MID (36-60 inches)
- MID_FAR (72-96 inches)
- VERY_FAR (over 108 inches)

---

## Quick Reference: Controls

| Button | Function |
|--------|----------|
| Left Stick Y | Adjust flywheel velocity |
| Right Stick Y | Adjust visor/hood position |
| D-Pad Left/Right | Change distance category |
| A | Shoot current ball |
| B | Reset to Ball 1 |
| X | Spin up flywheel |
| Y | Stop flywheel |
| Right Bumper | Toggle Fine/Coarse adjustment |
| Left Bumper | Save values to log |

---

## FTC Dashboard Tips

1. Connect to `192.168.43.1:8080/dash` when running
2. Watch the "Actual Velocity" graph to see RPM recovery
3. Adjust values in real-time from the Dashboard variables panel
4. Use the graph to identify optimal timing between shots
