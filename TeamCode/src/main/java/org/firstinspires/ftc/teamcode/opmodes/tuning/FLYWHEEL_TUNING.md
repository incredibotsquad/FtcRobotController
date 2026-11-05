# Flywheel Tuning Guide

This walks you through tuning the launcher flywheel with the **FlywheelTuner** OpMode (group "Tests"). The goals are to:

1. reach target RPM quickly,
2. hold it steadily, and
3. recover as fast as possible after each shot.

The tuner edits the single gain set in `LaunchSubsystem` (`FLYWHEEL_*` fields). Match code (`LaunchReadinessCommand`) instead uses the **gain table** (`GAIN_TABLE_*` arrays): one row of P/I/D/kS/kV per launch distance, interpolated between rows. So tune once per distance (at that distance's RPM) and copy the result into that row. **Step 9 (saving values) is the step people forget.**

---

## 0. How the controller works (read once)

Every loop, `LaunchSubsystem.updateFlywheel(targetRPM)` picks the motor power:

| Situation | Power |
|---|---|
| More than `FLYWHEEL_BANG_BANG_THRESHOLD_RPM` below target (and bang-bang enabled) | **1.0** (full power) |
| Otherwise | **feedforward + PID**, limited to 0 to 1 |

- **Feedforward** = `kS + kV × targetTPS`, scaled by `NOMINAL_VOLTAGE / batteryVoltage`. This is the "predicted" power to hold a speed, and it should do about 90% of the work.
- **PID** corrects what feedforward misses. In practice that's **P only**: it pushes harder when the speed drops after a shot.
- Units: gains act on **ticks/sec**. GoBILDA BARE has 28 counts per rev, so 1100 RPM = 513 TPS.

Rough sense of scale:

- kS ≈ 0.05–0.15
- kV ≈ 0.0005–0.0007 (kV × 513 ≈ 0.3 power at 1100 RPM)
- P ≈ 0.001–0.02

---

## 1. Setup

1. **Battery:** fully charged. Put it on the robot and let it sit a minute.
2. **Robot:** on the floor or a cart with the launcher pointing somewhere safe. **The flywheel spins up as soon as you press START.**
3. **Panels:** open it in a browser on the Driver Hub network. You'll use:
   - **Graphs** tab: plot `targetRPM`, `actualRPM`, `leftRPM`, `rightRPM`, `power`.
   - **Configurables** tab: `LaunchSubsystem` (gains and switches) and `FlywheelTuner` (targets, feed power).
4. On the Driver Hub, pick **FlywheelTuner**, then INIT and START.

**Controls (gamepad 1):**

| Button | Action |
|---|---|
| Y | toggle high / low target RPM |
| A | toggle gate (open = shoot) |
| X | toggle feed (intake + transfer) |
| B | cycle step size (10 → 1 → 0.1 → **0.01** → 0.001 → 0.0001 → 0.00001) |
| Dpad up/down | P + / − step |
| Dpad right/left | kS + / − step |
| RB / LB | kV + / − step |
| Back | auto-adjust kV |
| Start | reset shot statistics |

The current step size is shown as `Step (B)`. **Check it before every adjustment.** A wrong step size is the most common mistake.

---

## 2. Sanity checks (2 minutes)

With the flywheel running at the high target:

- [ ] **`leftRPM` and `rightRPM` agree within ~5%.** If one reads about 0, that motor's encoder cable is unplugged. If one is negative, its direction or encoder is reversed. Fix this first: the controller uses the average, so a dead encoder makes it over-drive both motors.
- [ ] **`Loop ms` is under ~15 ms.** Above that, recovery will be slow no matter how you tune.
- [ ] **`Battery V` is believable** (12.5–13.5 V on a fresh battery).

**Set `LaunchSubsystem.NOMINAL_VOLTAGE` to the `Battery V` reading now.** You're tuning kS/kV at this voltage, and voltage compensation scales from it.

---

## 3. Prepare for feedforward tuning

In Panels → Configurables → `LaunchSubsystem`:

- `FLYWHEEL_BANG_BANG_ENABLED` = **false**
- `FLYWHEEL_P` = `FLYWHEEL_I` = `FLYWHEEL_D` = **0**

In `FlywheelTuner`:

- `highVelocityRPM` = your fastest expected shot (e.g. 1400)
- `lowVelocityRPM` = your slowest expected shot (e.g. 900)

With no PID and no bang-bang, power is pure feedforward. Any steady error you see is a feedforward error.

---

## 4. Tune kS (static friction)

kS is the power needed to just overcome friction.

1. Set `FLYWHEEL_KV` = **0** (write down the old value first).
2. Stop the OpMode, wait for the flywheel to stop, then INIT and START again so it begins from rest.
3. Power now equals kS. Set the step size to **0.01**.
4. Lower kS with **Dpad left** until the flywheel stops turning (or doesn't start), then raise it with **Dpad right** until it *just* keeps turning slowly.
5. Take that value and subtract one step. That's kS.
6. Put kV back to the old value (or 0.00058).

> Shortcut: if this is fiddly, kS = 0.05–0.1 works fine. kS matters most at low RPM.

---

## 5. Tune kV (velocity gain)

kV is the power needed per tick/sec of speed.

1. Make sure the high target is selected. Wait **3 seconds** for the speed to settle.
2. Press **Back**. kV rescales by `target / actual`.
3. Wait 3 seconds and press Back again. Repeat until `actualRPM` sits within ~10 RPM of target (usually 2–3 presses).
4. Press **Y** to switch to the low target, wait, and compare:

| What you see | Meaning | Fix |
|---|---|---|
| Both targets are close | kS and kV are good | go to step 6 |
| Both short (or both over) by the **same RPM** | kS is off | raise kS if short, lower if over (step 0.01), then redo Back at high |
| Error **grows with speed** | kV is off | press Back again at the high target |

**Done when:** actual is within ~2% of target at both targets with P = 0.

---

## 6. Tune P (shot recovery)

Now use real shots.

1. Set the step size to **0.001**. Start with `FLYWHEEL_P` = **0.001**.
2. Select the target you shoot at most. Press **Start** (gamepad) to reset the statistics.
3. Load balls: press **X** to turn the feed on and let the intake pull in up to 4.
4. Press **A** to open the gate and fire. After the volley, press A to close the gate.
5. Read the **Shots** section:
   - `Dip RPM last / worst`: how far the speed dropped during a shot
   - `Recovery ms last / avg`: time from the drop until back within `TARGET_RPM_TOLERANCE`
6. Write it down (table below). Raise P by one step and repeat.

| P | Avg recovery ms | Worst dip RPM | Oscillation? |
|---|---|---|---|
| 0.001 | | | |
| 0.002 | | | |
| 0.003 | | | |
| ... | | | |

Keep raising P until one of these happens:

- **Oscillation:** with the gate closed, `actualRPM` keeps wobbling above and below target, and `power` swings up and down continuously.
- **Recovery stops improving** for two steps in a row.

Then **use about 70% of the value where oscillation started** (or the best non-oscillating value).

**I and D:** leave both at 0.

- Voltage compensation handles the slow drift I would normally fix.
- With only 28 counts per rev, the velocity signal is noisy, and D amplifies that noise.

Only try a tiny I (e.g. 0.00001) if you still see a steady offset after step 5.

---

## 7. Tune bang-bang (spin-up and big dips)

1. Set `FLYWHEEL_BANG_BANG_ENABLED` = **true**, `FLYWHEEL_BANG_BANG_THRESHOLD_RPM` = **150**.
2. **Spin-up:** press **Y** a few times to switch targets. After each switch, read `Spin-up ms (last)` and watch the graph:
   - If `actualRPM` shoots **past** the target and comes back, that's overshoot. Raise the threshold by 25.
   - If there's no overshoot, try lowering it by 25 for a faster spin-up.
3. **Shots:** fire volleys again and compare `Recovery ms avg` with the step 6 result.
   - Full power only kicks in when a dip is **bigger than the threshold**. If your worst dip is about 100 RPM and the threshold is 150, it never triggers during shots, and all recovery comes from P.
   - To let it help with shots, set the threshold a bit **below** your typical dip, but above `TARGET_RPM_TOLERANCE` (50) so it doesn't fight the PID near target.
4. Pick the threshold with the best spin-up and recovery that **doesn't overshoot**.

> If you can't remove overshoot without losing the speed benefit, re-check kV (step 5). A low kV makes the hand-off from full power to PIDF dip and then overshoot.

---

## 8. Readiness tolerance and shot spacing

- **`TARGET_RPM_TOLERANCE`** (default 50): the robot only fires when it's within this many RPM of target.
  - Tighter (e.g. 30) gives more consistent shots but waits longer between shots.
  - Looser (e.g. 75) fires faster but with more spread.
  - Pick it by shooting at a real CELL and watching the spread.
- **Shot spacing:** a ball should only reach the flywheel *after* the previous dip has recovered. If `Recovery ms` is longer than the time between balls, the speed drops further with every ball in a volley. You'll see `Dip RPM` grow from shot to shot and the last shots fall short. To fix it:
  - lower `FlywheelTuner.transferMotorPower` to see what spacing works, then
  - set `SmartIntakeCommand.LAUNCH_TRANSFER_POWER` to match for matches.

---

## 9. SAVE THE VALUES

Panels edits and button changes are **lost when the Robot Controller app restarts**. Copy the final values into the defaults in `subsystems/LaunchSubsystem.java`:

```java
public static double FLYWHEEL_P = ...;
public static double FLYWHEEL_I = 0.0;
public static double FLYWHEEL_D = 0.0;
public static double FLYWHEEL_KS = ...;
public static double FLYWHEEL_KV = ...;
public static boolean FLYWHEEL_BANG_BANG_ENABLED = true;
public static double FLYWHEEL_BANG_BANG_THRESHOLD_RPM = ...;
public static double NOMINAL_VOLTAGE = ...;   // the voltage you tuned at
public static double TARGET_RPM_TOLERANCE = ...;
```

Then copy P / I / D / kS / kV into the **gain table row** for the distance whose RPM you tuned at (same index in every array):

```java
public static double[] GAIN_TABLE_DISTANCES_IN = {24, 48, 72, 96, 120};
public static double[] GAIN_TABLE_P  = {...};
public static double[] GAIN_TABLE_I  = {...};
public static double[] GAIN_TABLE_D  = {...};
public static double[] GAIN_TABLE_KS = {...};
public static double[] GAIN_TABLE_KV = {...};
```

Repeat steps 4–8 at each row's RPM (set `highVelocityRPM` in the tuner to the RPM from `LaunchReadinessCommand.SHOT_RPMS` for that distance). Keep all arrays the same length; telemetry warns if they differ. `USE_GAIN_TABLE = false` makes match code use the single `FLYWHEEL_*` set instead.

Commit them, with a note of the date and battery voltage.

---

## 10. Next: the distance → RPM table

Flywheel tuning makes the flywheel *hold* any RPM. Which RPM to use at each distance goes in `LaunchReadinessCommand.SHOT_DISTANCES_IN` / `SHOT_RPMS`:

1. Place the robot at a measured distance from your CELL (e.g. 24″, 48″, 72″, 96″, 120″, center of robot to center of CELL).
2. Aim the turret at the CELL. FlywheelTuner doesn't move the turret, so use **ServoPositionTuner** or set it by hand.
3. Adjust `FlywheelTuner.highVelocityRPM` until shots land in the center of the CELL opening, and record it.
4. Repeat for each distance and fill in both arrays (same length, sorted by distance).

---

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| One side's RPM about 0 | encoder cable unplugged on that motor |
| `actualRPM` noisy (±30 RPM) with the gate closed | P too high, or D > 0 |
| Never reaches target even at full power | target above what the motors can reach; lower it, or check for mechanical drag |
| Fine early, short shots late in a match | `NOMINAL_VOLTAGE` wrong, or `FLYWHEEL_VOLTAGE_COMPENSATION` off |
| Recovery slow despite high P | `Loop ms` high; check bulk caching and telemetry load |
| Overshoot after every spin-up | bang-bang threshold too small, or kV too low |
| Dips grow during a volley | balls fed faster than recovery; see step 8 |
