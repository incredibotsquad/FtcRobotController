# DynamicAuto changes: shorter waits, LEFT spots 3 and 4, final launch

> **Update 2026-11-20:** the LEFT spot 3 described below (at the rear-wall FLOWER's approach) was
> unused and has been removed. The final-launch spot described as LEFT spot 4 is now **LEFT spot 3**
> (`LaunchPoint.LEFT_SPOT_3`, `Poses.RED_LAUNCH_FAR_CELL_3`, `DynamicAuto.FINAL_LAUNCH_AT_LEFT_SPOT_3`).
> The rest of this page is the original change note and keeps the old names.

This update makes `DynamicAuto` spend less time standing still. It also adds two new launch spots on
the LEFT side, with a final launch that runs automatically after the last intake.

All pose values are red-frame `{x, y, headingDeg}` and are rotated 180° for blue, as before.

## What changed

### 1. Shorter waits (timing values)

| Setting | File | Before | Now | What it controls |
|---|---|---|---|---|
| `INTAKE_TIMEOUT_MS` | `DynamicAuto` | 2500 | **1200** | How long the robot waits at the GARDEN for more balls |
| `FLOWER_WAIT_MS` | `DynamicAuto` | 2000 | **1200** | How long the robot waits at a FLOWER |
| `ROCK_COUNT` | `DynamicAuto` | 2 | **1**, back to **2** | Rocks back and forward at the GARDEN |
| `MAX_FIRE_MS` | `LaunchBallsCommand` | 3000 | **1500** | Longest a launch can take when the count misses a ball leaving |

Why these mattered:

- **GARDEN:** the 4th ball usually can't be reached from the intake pose, so the robot always
  waited the whole timeout. The rocking afterwards picks the 4th ball up anyway.
  (Since 2026-11-17 the GARDEN uses no bar, `GARDEN_NEEDS_BAR = false`: the intake pose moved
  closer to the wall (y 15 -> 11 -> 9) and the robot rocks from the intake pose, `ROCK_COUNT` 2,
  stopping as soon as the count reads 4.)
- **FLOWERs:** the robot rarely gets to 4 balls there, so it always waited the whole time.
- **Launch:** the top beam break sometimes misses a ball leaving, so the count sticks above 0.
  The launch then lasted the full backstop time after the robot was already empty.

### 2. Fix: auto randomly driving slowly

The 40% speed limit used for the slow intake drive-in and the rocking is written into Pedro's shared
`Constants.foresightConfig`. Pedro only resets it when the next path starts. If a run was stopped
during one of those slow paths, the 40% limit stuck for every later run until the app restarted.

`DynamicAuto.runOpMode()` now resets `maxPathSpeed` to "no limit" at the start of every run.

### 3. New launch spots: LEFT spot 3 and LEFT spot 4

`LaunchPoint` now has `LEFT_SPOT_1`, `LEFT_SPOT_2`, `LEFT_SPOT_3`, `LEFT_SPOT_4`, `RIGHT_SPOT_1` and
`RIGHT_SPOT_2`. Spots 3 and 4 both launch into the LEFT CELL.

| Spot | Red pose | Distance to the CELL | Notes |
|---|---|---|---|
| **LEFT spot 3** | (48, 120, 270°) | ~32 in | The rear-wall FLOWER's approach position (`RED_LEFT_FLOWER_APPROACH` x, y), with LEFT spot 2's heading (`RED_LAUNCH_FAR_CELL_2`), so it faces the HIVE. It has no array of its own: tuning either of those moves it. Pick it in INIT like any other spot. |
| **LEFT spot 4** | (16, 116, 315°) `RED_LAUNCH_FAR_CELL_4` | ~51 in | The final launch spot (see 4). |

Spot 3 is closer than the shot table starts (42 in), so the launcher uses the table's nearest RPM there.

### 4. Final launch at LEFT spot 4 after the last intake

New setting: `DynamicAuto.FINAL_LAUNCH_AT_LEFT_SPOT_4` (default `true`).

- After the **last** intake point in the plan, the robot drives a straight line, with linear heading,
  to LEFT spot 4 instead of back to the plan's launch point.
- It launches there if the LEFT CELL is up, then parks.
- **Don't** also add LEFT spot 4 to the launch sequence in INIT. If you do, the robot can go there twice.
- Set the flag to `false` in Panels to get the old behavior.

### 5. Turn before lowering the bar at an intake

New setting: `DynamicAuto.INTAKE_APPROACH_HEADING_TOLERANCE_DEG` (default 10°).

At an intake point's approach pose, the robot now turns in place until it faces the approach heading
before it lowers the bar and drives in. It gives up and goes on after `PATH_TIMEOUT_MS`.

This matters for LEFT spot 3 → rear-wall FLOWER. Both use the same position but face opposite ways,
so before this change the robot would have driven into the FLOWER mid-turn with the bar down.

### 6. Control points moved away from the HIVE

The HIVE spans roughly x 47-97, y 52-92. Clearances below assume an 18 in square robot.

| Control point | Before | Now | Why |
|---|---|---|---|
| `RED_CONTROL_AROUND_HIVE` | {24, 72} | **{-5, 88}** | The AROUND path (e.g. RIGHT spot 2 → LEFT spot 3) came within about 1.6 in of the HIVE's far corner. {16, 88} gave about 4 in of clearance; on 2026-11-20 RIGHT spot 2 → LEFT spot 2 still hit the HIVE at {11, 88}, so x went to the alliance wall and then 5 in past it, to -5 (a spline never reaches its control point, so the path stays on the field). Used for every launch-to-launch move around the HIVE. |
| `RED_CONTROL_AROUND_HIVE_AUDIENCE` / `_FAR` (replace `RED_CONTROL_AROUND_HIVE`) | {-7, 88} (one point) | **{12, 50} and {12, 98}** | RIGHT spot 2 → LEFT spot 2 still hit the HIVE. Two control points (nearer-to-start first) keep the path at x ~17-22 the whole way alongside the HIVE, instead of bowing toward the wall. Robot center stays 21-28 in from the HIVE legs (x ~47, y ~52-92) for every RIGHT ↔ LEFT pair, and x ≥ 14 from the wall. The one-point path was ~28 in on paper too, so if it still hits, check odometry drift or how the robot swings during the 180° turn. |
| `RED_CONTROL_AROUND_HIVE_AUDIENCE` / `_FAR` | {12, 50} and {12, 98} | **{17, 50} and {17, 98}** | 2026-11-20: both control points moved 5 in away from the alliance wall (x +5), which shifts the RIGHT spot 2 ↔ LEFT spot 2 path about the same amount toward the HIVE. Check HIVE clearance on the field. |
| `RED_CONTROL_WALL_FLOWER` | {32.58, 51.36} | **{2.58, 51.36}** | Splining from LEFT spot 3 to the alliance-wall FLOWER cut about 8 in into the HIVE. Now about 2 in of clearance. A spline never reaches its control point, so the robot stays about 4-5 in off the wall. |

### 7. No more pushing at a FLOWER or the GARDEN wall

All three autos (`DynamicAuto`, `LeftWallAuto`, `RightWallLGLFP`; RightWallLGLFP was removed on 2026-11-20):

- **FLOWER:** if the count reaches 4 while the robot is still driving in, it leaves right away.
  Before, it kept pushing toward the intake pose until the path ended, which looked like a wait.
- **Stall = arrived:** the drive-in and each rocking leg also end once the robot has moved slower
  than `STALL_SPEED_IN_PER_SEC` (2 in/s) for `STALL_MS` (300 ms). The GARDEN intake pose
  (`RED_GARDEN_INTAKE`, y 7) is past where the robot can reach. Before, the drive-in pushed against
  the wall for the whole `PATH_TIMEOUT_MS` (5 s) before intaking and rocking.
- **Back-out no longer stops:** backing out of a FLOWER with the bar down now ends
  `BACK_OUT_END_TOLERANCE_IN` (6 in) before the approach pose, so the next path starts while the
  robot is still moving. Before, it slowed to a stop at the approach pose first, which looked like a
  wait after getting 4.
- At the GARDEN the log now says why it didn't rock (`no rocking: full / ROCK_COUNT is 0 / park window`).

## How the auto improved

Time saved per step, from the settings above and the match logs:

| Step | Before | Now | Saved |
|---|---|---|---|
| GARDEN wait for the 4th ball | 2.5 s | 1.2 s | ~1.3 s |
| GARDEN rocking (each rock ≈ 1.2 s in the logs) | 2 rocks | 1 rock | ~1.2 s |
| FLOWER wait | 2.0 s | 1.2 s | ~0.8 s per FLOWER |
| Launch that hits the backstop | 3.0 s | 1.5 s | ~1.5 s per launch |
| Random slow driving after a stopped run | possible | fixed | whole paths at 40% speed |

With a plan like RIGHT spot 2 → GARDEN → LEFT spot 3 → rear-wall FLOWER → alliance-wall FLOWER →
LEFT spot 4, that adds up to roughly **5-7 s** less standing still per auto. That's an estimate from
the settings, not a side-by-side timing of old and new code.

In the logged test runs (RED, start RIGHT, plan: RIGHT spot 2, LEFT spot 3; GARDEN, rear-wall FLOWER,
alliance-wall FLOWER), the robot:

- launched at RIGHT spot 2 and intaked at the GARDEN;
- crossed AROUND the HIVE to LEFT spot 3 and launched;
- turned in place and intaked at the rear-wall FLOWER;
- was parked by about 21 s.

The last step is the first known issue below.

## Known issues (not fixed in this PR)

1. **TIP check fails at LEFT spot 3.** On the second visit, the Limelight sees the LEFT CELL's
   tags (32 and 33) from about 36-38 in away. They read about 44-46 in high, about 5 in lower than
   the expected 49.9 in, which puts them inside the "ambiguous" band (42.8 ± 3 in,
   `LimelightSubsystem.UP_DOWN_MARGIN_IN`).

   Every reading is thrown out, so the check is unconfirmed and the robot assumes the HIVE tipped.
   With no launch point left, it parks and skips the alliance-wall FLOWER.

   Next step: check the tag heights at spot 3 with `HiveDriveTest`, then fix the Limelight's camera
   height and tilt, or set `HiveTags.UP_TAG_Z_IN` / `DOWN_TAG_Z_IN` to the measured values.

2. **The robot can count as arrived before it has turned.** When a launch spot shares a position
   with the pose the robot is already at (LEFT spot 3 after the rear-wall FLOWER), the path is only
   about 2 in long. It "finishes" before the robot turns, so the TIP check starts while it still
   faces the wall. Waiting for the launch heading before the TIP check would fix this.

3. **The HIVE is out of view at RIGHT spot 2.** Facing 90°, the HIVE is about 35-45° to the side
   and the Limelight never sees it there. The TIP check at RIGHT spot 2 always ends unconfirmed, so
   the robot always crosses to the left side after the first intake, whether the HIVE tipped or not.

4. **`MAX_FIRE_MS` 1500 vs `EMPTY_CONFIRM_MS` 1000.** With the shorter backstop, the
   "sensors clear → count is 0" reset sometimes doesn't have time to happen. One log shows
   `kept 2 after launch: sensors clear ... only 950 ms`. The robot then believes it still holds 2
   balls. Consider `EMPTY_CONFIRM_MS` ≈ 800.

5. **Turret lock is on by default.** `LaunchSubsystem.isLaunchReadinessLocked` is now `true`, so the
   turret stays pointing straight ahead (TURRET_MID) and doesn't track the CELL. That includes auto.
   Review before merging. At LEFT spot 3 the CELL is about 22° off straight ahead.

## Testing

- `./gradlew :TeamCode:compileDebugJavaWithJavac` passes.
- Ran on the robot as DynamicAuto (RED); results are in "How the auto improved" and "Known issues" above.
- Not tested as BLUE.
