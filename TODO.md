# TODO

Open work items. Check items off (or delete them) when done; add the date when you add one.

## Laser ball counting (added 2026-11-16)

Sensors 1 and 4 are now analog goBILDA laser distance sensors read by
`subsystems/LaserBallCounter` (values tuned in the 2026-11-11 laser tests; `LaserIntakeTuner` re-tunes them).
Sensor 2 (beam break) corrects the count up to 4 when a ball rests there; sensor 3 is optional.

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

## Before competition

- [ ] **Re-add `@Disabled`** to `ServoPositionTuner`, `FlywheelTuner`, `FlywheelBenchTuner`,
      and `LaserIntakeTuner` (Panels is illegal in matches, R704).
- [ ] **Intake bar positions.** Retuned 2026-11-16 (`LEFT/RIGHT_RAISED/LOWERED`); CLAUDE.md still
      calls them placeholders. Confirm and update.
- [x] **Intake bar naming (2026-11-16).** Fixed: formula `RAISED + f * (LOWERED - RAISED)`;
      0.120 / 0.840 is DOWN (seen on the robot), so `LEFT/RIGHT_RAISED = 0.640 / 0.320`.
