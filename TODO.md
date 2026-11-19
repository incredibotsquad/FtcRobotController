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

## Shot table tuning (added 2026-11-18, reworked 2026-11-19)

The shot and gain tables now have four tuning points, 47 / 52 / 66 / 88 in. RPM is interpolated
between points and **extrapolated** outside them (`EXTRAPOLATE_SHOT_RPM`, limited to
`MIN/MAX_SHOT_RPM`); gains hold the nearest end row. Add a row (same index in `SHOT_DISTANCES_IN`,
`SHOT_RPMS` and every `GAIN_TABLE_*` array) when a spot needs its own tuning: FlywheelTuner only edits
rows within 3 in of the robot. Press gamepad 1 Y at the end and copy the logged rows into code.

- [x] **47 in** row: 2607 RPM, P 0.0015, kV 0.0003156 (tested at 60, 8 = 47.5 in: 2614 RPM, 2026-11-18).
- [x] **52 in** row: 2673 RPM, P 0.022, kV 0.0003145 (tested at 41.5, 8 = 51.0 in: 2660 RPM, 2026-11-18).
- [x] **66 in** row: 2928 RPM, P 0.0005, kV 0.00031 (tested at 12, 12 = 64.8 in: 2905 RPM, 2026-11-19).
- [ ] **88 in** row (3170, P 0.035 placeholders). Only reachable at (135, 9); consider moving it to ~69 in
      (red corner, 9, 9).
- [ ] Check the extrapolated close shots: auto spots at ~43.5 in (RIGHT/LEFT_SPOT_2, about 2561 RPM) and
      ~32 in (LEFT_SPOT_3, about 2409 RPM). Add a row there if they miss.
- [ ] kS is 0.16 in every row; only re-fit with steady-state data at two speeds.

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
