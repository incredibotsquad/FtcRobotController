# TODO

Open work items. Check items off (or delete them) when done; add the date when you add one.

## Laser ball counting (added 2026-11-16)

Sensors 1 and 4 are now analog goBILDA laser distance sensors read by
`subsystems/LaserBallCounter` (values tuned in the 2026-11-11 laser tests; `LaserIntakeTuner` re-tunes them).
Sensor 2 (beam break) corrects the count up to 4 when a ball rests there; sensor 3 is optional.

- [ ] **Check the baseline-jump fix (2026-11-17).** A single far reading right after START (intake bar
      moving) moved sensor 1's baseline 109 -> 120 mm, so the background then read "blocked" and the
      jam rule set the count to 4 with one ball. `LaserBallCounter.BASELINE_JUMP_MS` (250 ms) now makes a
      farther reading persist before the baseline moves. Watch the `LaserBallCounter` "baseline ... ->"
      log after START in TeleOp and the tuner; if it still jumps, look at what's in the laser's view
      when the bar moves.
- [ ] **Measure the laser baselines.** Enable `SensorTestOpMode` ("Test: Beam Break Sensors"),
      keep both lasers clear, and set `IntakeSubsystem.SENSOR1_BASELINE_MM` and
      `SENSOR4_BASELINE_MM` to the distances shown. Until then each sensor takes its first INIT
      reading as the baseline, which is wrong in auto (the top preload sits in front of sensor 4,
      so its exits wouldn't count).
- [ ] **Test sensor-4 exit counting on the robot.** The thresholds were tuned at sensor 1 with
      intake-speed feeds. At sensor 4 balls leave at full transfer speed. Touching pairs may count
      as one exit (safe, but the launch then runs to `MAX_FIRE_MS`). Watch logcat tags
      `IntakeCount` and `LaserBallCounter`. `SENSOR4_DURATION_RULE` is off on purpose (overcounting
      exits → reads empty with a ball left → intake takes a 5th, G407).
- [ ] **Test the sensor 2 full check (2026-11-16).** With 4 POLLEN the last rests at sensor 2; blocked
      `SENSOR2_FULL_MS` (250 ms) with the transfer forward raises the count to 4. Check that a ball
      riding past sensor 2 with 1-3 inside never holds it that long (log tag `IntakeCount`, reason
      "sensor 2 blocked"). Won't hold for NECTAR (larger, stack sits lower).
- [ ] **Test the sensor 4 leftover check (2026-11-17).** Launch n, let one stay behind: after the gate
      closes the count should go 0 -> 1 (`IntakeCount`, "ball left after launch"). Check the laser
      doesn't read blocked with sensor 4 clear and the gate closed (the gate in view, baseline drift).
- [ ] **Test the full eject (2026-11-17).** On reaching 4 the intake reverses `FULL_EJECT_MS` (500 ms).
      Check it pushes a straddler out without pulling the 4th back out of the transfer (if it does,
      the count stays 4 with 3 inside). It also runs at auto start (preloads = 4) and each time
      SmartIntakeCommand re-enters FULL (e.g. after BACK reverse).
- [ ] **Decide on the sensor 3 minimum.** `SENSOR3_MIN_ENABLED` is off. Check where the balls rest
      with 3 inside; if one always rests at sensor 3, turn it on (`SENSOR3_MIN_COUNT = 3`).
- [ ] **Re-check `JAM_MS` (400 ms) with the laser.** Slow singles stayed deep up to ~240 ms in the
      tests; make sure a slow ball doesn't trip the "jam = full" rule.

- [ ] **Tune intake counting with `LaserIntakeTuner` (2026-11-16).** Run trials of 1/2/3/4 balls fed
      the way they come in a match, press Y, copy the logged values (tag `LaserIntakeTuner`) into
      `LaserBallCounter` / `IntakeSubsystem` defaults. It also measures both baselines.
- [ ] **Launch counting tuner (sensor 4), next.** Sensor 4 currently shares `LaserBallCounter`'s
      static thresholds with sensor 1; give each sensor its own set before tuning the launch.

## Loop time vs. laser sampling (added 2026-11-16)

The laser updates about every 10 ms, but the counter only samples once per robot loop. The match
loop does much more than the test OpMode did (bulk reads on both hubs, Pinpoint I2C, Pedro, ~8
motor and ~6 servo writes, Limelight, telemetry). The 15–25 ms figure is an estimate, not a
measurement. At a slow loop the seam between touching balls can fall between samples (pair counted
as one), the 30 ms exit hold is only 1–2 samples, and the 115 ms duration rule is only accurate to
about one loop. Undercounting at sensor 1 is the G407-risky direction (`JAM_MS` is the backstop).

In order:

- [ ] **Measure the loop time.** Add a loop-time readout (average and max) to TeleOp and the
      autos; drive, intake and launch. Under ~10 ms: nothing to fix.
- [ ] **Check whether it hurts.** `LaserIntakeTuner` already replays its trials
      (`/sdcard/FIRST/data/laserIntakeTrials.csv`) at slower loop rates; compare its accuracy at
      the measured loop time.
- [ ] **If it does, speed up the loop.** Switch bulk caching to `MANUAL` with one `clearBulkCache()`
      per loop (in `Incredibot`, currently `AUTO`), skip unchanged motor/servo writes, poll the
      Limelight less often, keep `DebugTelemetry` off in matches.
- [ ] **Last resort:** read the two lasers on a dedicated ~5 ms background thread and run the
      counters there (more complex, adds hub traffic).

## Flywheel consistency (added 2026-11-17, POLLEN only)

Hardware: 2x Rhino 30A + 4 steel flywheels, 1:1 belt, 2 free hood rollers, 6 mm compression.
Estimate: ~2-3% RPM drop per ball, ~20-30 ms full-power recovery. Balls stream with no metering.

- [ ] **Build a target** at CELL distance/height; mark impacts by shot index (1-4) for 1/2/3/4-ball
      loads. Spread of impact height per index is the metric.
- [ ] **Log at loop rate** during bursts: flywheel RPM, battery voltage, sensor 4 exit timestamps.
      Get per-ball RPM dip, recovery time, ball spacing, and the sensor 4 -> wheel contact delay.
- [ ] **Measure the inertia** (power step, angular acceleration); estimate ~3650 g·cm².
- [ ] **Set the spacing with transfer power** (`SmartIntakeCommand.LAUNCH_TRANSFER_POWER`): spacing
      >= 2-3x recovery, without the last ball stalling in the dead zone.
- [ ] **Pre-ball boost:** full power timed from sensor 4 + the measured delay, for about the recovery time.
- [ ] **Ball-1 offset** if shot 1 still hits low (rollers at rest, starts from rest at the gate).
- [ ] **Dead zone:** look at a driven roller/flap so the last/lone ball arrives at transfer speed.
- [ ] **Hood rollers:** light and free-spinning (spin by hand, coast evenly).
- [ ] **Caliper 10-20 POLLEN**; check the 30A tread each event; buy spare 30A wheels.
- [ ] **Repeat at 12.0 / 12.8 / 13.5 V**, then rebuild `SHOT_DISTANCES_IN`/`SHOT_RPMS` from burst means.
- [ ] Optional: A/B test 70A Rhino (3601) wheels (both wheels, compression re-set).

## Shot table tuning (added 2026-11-18)

Tune with FlywheelTuner: robot on the spot, edits apply within 3 in of a row (`ROW_SNAP_IN`); press
gamepad 1 Y at the end and copy the logged rows into `LaunchReadinessCommand.SHOT_RPMS` and
`LaunchSubsystem.GAIN_TABLE_*`. Use a charged battery; the steady-flywheel wait and `LaunchShot` logs
are on. Most-used spots first (red-frame distances to their CELL):

| Spot | Distance | Nearest rows (shot / gain) |
|---|---|---|
| RED_LAUNCH_AUDIENCE_CELL_1 (RIGHT_SPOT_1) | 48.7 in | shot 48.75 / gain 47 + 52 |
| RED_LAUNCH_FAR_CELL_1 (LEFT_SPOT_1) | 49.7 in | shot 48.75 / gain 47 + 52 |
| RED_LAUNCH_AUDIENCE_CELL_2 (RIGHT_SPOT_2) | 43.5 in | shot 43.5 / gain 44 |
| RED_LAUNCH_FAR_CELL_2 (LEFT_SPOT_2) | 43.2 in | shot 43.5 / gain 44 |
| RED_START_FAR_WALL | 55.9 in | between 52 and 66 (no row) |

- [x] **52 in** (spot: red RIGHT start, 41.5, 8 = 51.0 in): RPM 2656, P 0.022, kV 0.0003145 (2026-11-18).
- [ ] **48.75 in** shot row (2668, placeholder): RIGHT_SPOT_1 / LEFT_SPOT_1. Gain row **47 in** too.
- [ ] **43.5 in** shot row (2515): RIGHT_SPOT_2 / LEFT_SPOT_2. Gain row **44 in** too.
- [ ] **42 / 44 in** shot rows (2515, placeholders): close shots; 42 = start of the table.
- [ ] **47 in** shot row (2668, placeholder).
- [ ] **66 in** shot row (2878) and gain row: longer TeleOp shots (and RED_START_FAR_WALL at 55.9 in
      interpolates 52-66: check a shot there).
- [ ] **88 in** shot row (3170) and gain row: end of the table, used for anything farther.
- [ ] kS is still 0.16 in every row; only re-fit if two speeds show a steady-state error (kS and kV
      can't be separated at one speed).

## Before competition

- [ ] **Re-add `@Disabled`** to `ServoPositionTuner`, `FlywheelTuner`, `FlywheelBenchTuner`,
      `LaserIntakeTuner`, `CameraLaunchTuner` and `IntakeTransferRattleTest` (Panels is illegal in matches, R704).
- [ ] **Continue camera tuning at `rightstart` (2026-11-18).** Resume run `20261118-001849-rightstart`
      (validation streak 1/3, starting settings s-06f97d39e66a, 2 calibration examples saved). Watch for
      a late 4th ball coming in under: if it keeps happening, fix the dead zone (or try the last-ball
      kick) before tuning RPM further.
- [ ] **CameraLaunchTuner first run (2026-11-17).** On the robot: check INIT shows sensors with nothing
      moving, both flywheel encoders read sensibly, the laptop handshake fires only when stable, the
      refill is refused after a burst until review/X, and LT clearing works. Then a manual-review run
      at one spot before trying `--reviewer openai/anthropic`. Promote validated values by hand (`export`).
- [ ] **Intake bar positions.** Retuned 2026-11-16 (`LEFT/RIGHT_RAISED/LOWERED`); CLAUDE.md still
      calls them placeholders. Confirm and update.
- [x] **Intake bar naming (2026-11-16).** Fixed: formula `RAISED + f * (LOWERED - RAISED)`;
      0.120 / 0.840 is DOWN (seen on the robot), so `LEFT/RIGHT_RAISED = 0.640 / 0.320`.
