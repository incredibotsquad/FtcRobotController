# Camera-assisted launch tuning

This setup tunes the launcher at a fixed spot so the robot reliably scores **3 of 4** loaded POLLEN, or optionally all 4. It has two halves:

- **Robot:** the `CameraLaunchTuner` OpMode (Driver Station group **Tests**). The file is `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/tuning/CameraLaunchTuner.java`.
- **Laptop:** `tools/camera_launch_tuning/`, a Python program. It records each burst with a laptop camera, collects the robot's telemetry over ADB, and reviews the finished trial. A review can be done by you, by the OpenAI API, by the Anthropic API, or by an agent session reading files. It then checks any proposed change against bounds and sends the next settings to the robot.

The program reviews **saved frames of a finished trial** between bursts. Nothing watches a live feed.

Reference docs: [operator/config reference](tools/camera_launch_tuning/README.md) and [architecture and agent guide](tools/camera_launch_tuning/AGENT_GUIDE.md).

---

## 1. What the OpMode does

1. **INIT** shows sensor 1 and 4 laser distances against their baselines, the sensor 2 and 3 beam breaks, both flywheel encoder RPMs, battery voltage, and the current settings. **No motor or servo is commanded during INIT.**
2. **START** registers the intake, intake bar, gate and launcher. It raises the intake bar (TeleOp does the same) and moves the turret to the trial position. The drivetrain is never registered or commanded, so the robot stays still.
3. **Gamepad 1 A** arms a trial. The settings are frozen at this moment, and later changes wait for the next trial. The intake and transfer then run, and you feed balls until the robot counts 4.
4. **Settling:** only the production settling happens. That is `SmartIntakeCommand`'s FULL run-on (`FULL_RUN_ON_MS`), the full eject that pushes out a straddling 5th (`FULL_EJECT_MS`), and the delayed transfer stop (`TRANSFER_FULL_DELAY_MS`). After that, the intake is stopped.
5. **Spin-up:** the flywheel runs at the trial's RPM and gains. The robot fires only when all of these hold:
   - **Both** motors are within `STABLE_TOLERANCE_RPM` (40) of the target.
   - The motors are within `MOTOR_MISMATCH_RPM` (60) of each other.
   - The flywheel is not in bang-bang, and this has held for `STABLE_MS` (400 ms).
   - The turret has had time to settle.
   - The laptop has said it is recording. You can turn this last check off on the robot.
6. **Burst:** the real `LaunchBallsCommand` opens the gate. `SmartIntakeCommand` feeds the transfer at the trial's transfer power, and the trial's timeout (`MAX_FIRE_MS`) applies. Both statics are restored afterwards.
7. **Post:** logging continues for 1.5 s with the flywheel still running, so recovery is visible. Then the robot writes the trial's summary and per-loop telemetry and goes idle.
8. **Refill check:** after any burst, the next **A** is refused until one of these is true:
   - The laptop review of that trial confirmed that **every** loaded ball left, and the robot count is 0 with lasers 1 and 4 clear.
   - You checked the robot, including the transfer-to-flywheel dead zone, and pressed **X**.

   Storage sensors reading clear is not enough, because a ball resting above sensor 4 is invisible to them.

### Reused from production TeleOp

| Production code | Used for |
|---|---|
| `Incredibot` constructor | Every subsystem, same hardware names, bulk caching |
| `IntakeSubsystem` | Laser and beam-break counting, jam rule, sensor 2/4 corrections, stall guard |
| `SmartIntakeCommand` | Loading, FULL handling, and the transfer feed while the gate is open |
| `LaunchBallsCommand` | The whole firing state machine (simple feed, `CLEAR_MS`, `MAX_FIRE_MS`, empty confirmation) |
| `CloseGateCommand` | Gate default |
| `ReverseIntakeCommand` | Jam clearing |
| `LaunchSubsystem` flywheel controller | Bang-bang, voltage-compensated feedforward and PID, through `updateFlywheelWithGains()` |
| `LaunchReadinessCommand` turret speed constants | Turret settle wait |

### Different from TeleOp

- **Drive, odometry and Limelight are not registered.** There is no driving, no launch hold, and no pose.
- **No `LaunchReadinessCommand`.** RPM, P, kS and kV come from the trial settings, not the distance tables. The turret sits at a fixed position, not auto-aim. I and D stay at `LaunchSubsystem.FLYWHEEL_I`/`FLYWHEEL_D`.
- **The intake runs only after A**, and stops when the burst ends. TeleOp would start intaking again.
- **Launches fire on their own** once the speed is stable. There is no operator RB.
- **No other operator bindings.** Only the tuner controls listed below are active.

### Small additions to production classes

These are read-only or opt-in. TeleOp behavior is unchanged.

- `LaunchSubsystem.updateFlywheelWithGains(...)` is the same controller with gains passed in, so shared statics aren't edited.
- `LaunchBallsCommand.getEndReason()` and `wasEmptyConfirmedAtTimeout()`.
- `IntakeSubsystem.getIntakeMotorPower()`, `getTransferMotorPower()` and `readRawSensorsForDisplay()`. The last one reads the sensors without counting or writing outputs.
- `LaserBallCounter.readDistanceMmNow()`.

---

## 2. Setup

You need: this repo, JDK 17 with the Android SDK (Android Studio installs both), Python 3.10 or newer, Android platform-tools (`adb`), and a webcam or phone-as-webcam that the laptop can open.

### macOS

```bash
cd path/to/FtcRobotController/tools/camera_launch_tuning
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
# adb: Android Studio installs it in ~/Library/Android/sdk/platform-tools (the tool finds it there),
# or: brew install android-platform-tools
```

Camera permission: System Settings > Privacy & Security > Camera, then allow your terminal app.

### Windows (PowerShell)

```powershell
cd path\to\FtcRobotController\tools\camera_launch_tuning
py -3 -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
# adb: %LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe (found automatically), or add it to PATH
```

Camera permission: Settings > Privacy & security > Camera, then allow desktop apps.

### Linux

```bash
cd path/to/FtcRobotController/tools/camera_launch_tuning
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
sudo apt install android-tools-adb      # or ~/Android/Sdk/platform-tools/adb
sudo usermod -aG video "$USER"           # camera access, then log out and back in
```

### Connect ADB to the Control Hub

- **USB:** use the Control Hub's USB-C port, then run `adb devices`.
- **Wi-Fi:** join the robot's network, then run `adb connect 192.168.43.1:5555`.

If more than one device is listed, pass `--serial <id>` to every command.

### Build and install the robot app

Run these from the repo root, with the Control Hub connected over ADB:

```bash
./gradlew :TeamCode:assembleDebug      # compile check
./gradlew installDebug                 # install on the Control Hub
```

Windows uses `gradlew.bat`. The SDK's Gradle build needs JDK 17. Android Studio's bundled JDK works; on the command line, set `JAVA_HOME` to it.

### Local API keys (optional, for AI review only)

```bash
cp .env.example .env      # Windows: copy .env.example .env
```

Fill in only what you use: `OPENAI_API_KEY` / `OPENAI_MODEL` or `ANTHROPIC_API_KEY` / `ANTHROPIC_MODEL`. Blank models use the defaults, `gpt-6.2-astra` and `claude-opus-5-5`.

- `.env` is git-ignored and read as plain text. Nothing in it is executed.
- Variables already set in your shell win over the file. `--env-file PATH` points at another file.
- Keys are never printed, saved, or sent to the robot.
- **API keys are separate from a Codex or Claude login.** Signing in to an interactive agent doesn't give this program API access. To use your agent subscription instead, choose `--reviewer agent-file` (section 7).

Check everything:

```bash
python camera_launch_tuning.py doctor
python camera_launch_tuning.py cameras
```

---

## 3. Sensor calibration (before the first trial)

1. Start `CameraLaunchTuner` (INIT only) with **no balls in the robot**.
2. On the Driver Station, compare "Sensor 1 laser (mm / baseline)" and "Sensor 4 laser". With the sensors clear, the reading should be within a few mm of the baseline.
3. If it isn't, set `IntakeSubsystem.SENSOR1_BASELINE_MM` / `SENSOR4_BASELINE_MM` to the clear readings, rebuild, and reinstall. A baseline of 0 takes the first INIT reading, which is wrong if a ball is in front of the sensor then.
4. Sensors 2 and 3 should read "clear" with nothing in front of them.

The tuner doesn't invent calibration values. It uses whatever `IntakeSubsystem` is configured with, and the counting rules are the production ones.

## 4. Camera setup

- Mount the camera on a tripod **behind and slightly above the robot**. It should see the launcher exit, the whole ball flight, and the target CELL opening. Don't move it during a run; every trial must be comparable.
- Use 60 fps if the camera supports it (`--fps`). Use bright, even light and avoid backlighting from windows. Ball flight should be visible without motion smear.
- "left/right" mean as seen from behind the robot toward the target. Keep that orientation so labels stay consistent.
- Put the spot name and camera placement in the profile description, so later sessions can repeat them.

## 5. Spot profiles

A profile stores the **starting** settings and, after validation, the **selected** settings for one shooting spot.

```bash
# From the robot's production tables at a distance (robot center to CELL, inches):
python camera_launch_tuning.py spot-init --spot right1 --from-robot --distance 48.75 \
    --description "RIGHT_SPOT_1, tripod 2 m behind, centered"
# Or fully manual:
python camera_launch_tuning.py spot-init --spot right1 --target-rpm 2668 --p 0.023 --ks 0.16 \
    --kv 0.00032419 --transfer-power 1.0 --max-fire-ms 3000 --turret-position 0.55
```

**Turret starting position:** `--from-robot` uses the turret position stored in `CrossOpModeStorage`. To get a good one:

1. Run TeleOp at the spot and let the turret auto-aim.
2. Stop TeleOp.
3. Start `CameraLaunchTuner` and run `spot-init`.

You can also nudge the turret with the dpad (section 8, link off) and pass `--turret-position`.

## 6. A manual trial (first run)

```bash
python camera_launch_tuning.py run --spot right1 --reviewer manual --max-trials 3
```

1. Driver Station: select **CameraLaunchTuner**, INIT, then START.
2. The laptop sends the settings and asks you to press **gamepad 1 A**. Feed 4 balls.
3. When the robot spins up, the laptop starts recording and gives permission to fire. The robot launches once both motors are stable.
4. Watch `runs/<run>/trials/0001/video.mp4`. Label each ball in launch order: `s` scored, `h` short, `l` long, `f` left, `r` right, `k` stuck, `u` unknown.
5. At the checkpoint, press Enter for the next trial, `u` to discard the last trial, or `q` to quit.

**Without a laptop:** press gamepad 1 Y while idle to turn the laptop link off. The robot then fires as soon as the speed is stable, and the dpad edits RPM and turret. Telemetry is still saved on the robot. The laptop link is needed for reviews and refill authorization; without it, press **X** after checking that the robot is empty.

## 7. Automatic tuning

```bash
python camera_launch_tuning.py run --spot right1 --reviewer anthropic --auto-apply
python camera_launch_tuning.py run --spot right1 --reviewer openai --goal 4of4
python camera_launch_tuning.py run --spot right1 --reviewer agent-file   # a Codex/Claude session answers
```

For each trial, the reviewer gets about 16 timed frames from the burst, the encoder metrics, the robot summary, and the kept history. It labels each ball and may propose **one** change, which the program checks before applying. Without `--auto-apply`, every accepted proposal asks y/N first. The checks:

- Finite value, inside absolute limits, within the maximum step size.
- No change after a success, or after an inconclusive (ambiguous) trial.
- RPM changes need short or long misses in the matching direction, with good speed tracking.
- Turret changes need left/right misses.
- P / kV changes need encoder evidence: slow recovery, oscillation, or a steady-state error.
- kS changes need steady data at two operating speeds at least 300 RPM apart, because one speed can't separate kS from kV.
- A stuck ball while tracking is fine counts as **mechanical**, so gain changes are refused.

A run stops when the trial limit is reached (`--max-trials`, default 20), or when **3 consecutive goal successes at identical settings** validate the profile. Every trial counts toward the limit, including discarded ones.

The model only adapts through context (the saved history of kept trials). No model is trained.

## 8. Controls

| Input | Action |
|---|---|
| gamepad 1 **A** | Arm loading (refused until the robot is confirmed empty after a burst) |
| gamepad 1 **B** | Abort the current trial |
| gamepad 1 **X** | "Physically empty": you checked the robot, including the dead zone. Sets the count to 0 |
| gamepad 1 **LT** or gamepad 2 **BACK** (hold) | Reverse intake and transfer, flywheel off, gate closed. Release stops reversing |
| gamepad 1 **Y** (idle) | Toggle "laptop link required" |
| gamepad 1 **dpad** (idle, link off) | Up/down: RPM ±25. Left/right: turret ±0.005 |

## 9. Jams and the 4th ball

The default goal is 3 confirmed scores out of 4, so a 4th ball that sticks in the dead zone is acceptable for that trial. It must be **cleared before the next full set**, because loading on top of it would put 5 in the robot (G407).

To clear a jam or a stuck ball:

1. Hold **LT** to reverse. The flywheel stops and the gate stays closed.
2. Release LT.
3. Look into the transfer and the dead zone. Remove any ball by hand with the robot disabled if needed.
4. Press **X**, then **A**.

The laptop never authorizes a refill when a ball is stuck or unknown, or when the trial was discarded.

## 10. Undo

- **Live:** press `u` at the checkpoint after a trial. This discards the **immediately last kept trial**:
  - Its reading and proposal leave all future context and validation.
  - Any refill authorization from it is revoked.
  - The settings go back to what that trial ran with, or the session's initial settings if it was the first trial.
  - The robot applies the change at its idle checkpoint, never mid-shot.
- **Later, from a terminal:** `python camera_launch_tuning.py undo --run <run-id-or-folder>`. Add `--push` to also send the restored settings to the connected robot.
- Files are never deleted. The trial folder gets `DISCARDED.json`, and `audit.jsonl` records the discard.

## 11. Results and promotion

- `runs/<run>/` holds every trial: video, frame times, review frames, robot telemetry, metrics, the raw and normalized assessments, and decisions. It also has `history.jsonl` and `audit.jsonl`. List a run's trials with `history --run <run>`.
- When a run validates, its settings are saved as the spot's **selected** settings in `profiles/<spot>.json`. You can also run `select --run <run>`.
- **Production match tables are never changed automatically.** To promote settings, run `export --spot right1`. Copy its values by hand into `LaunchReadinessCommand.SHOT_DISTANCES_IN`/`SHOT_RPMS` and the matching `LaunchSubsystem.GAIN_TABLE_*` row, and check `LAUNCH_TRANSFER_POWER`/`MAX_FIRE_MS`. Then rebuild, test in TeleOp at that spot, and commit as usual.
- The tuner aims with a fixed turret position. TeleOp auto-aims, so confirm its aim matches at the spot.

## 12. Troubleshooting

| Symptom | Fix |
|---|---|
| `status.json not readable` | Start CameraLaunchTuner (INIT is enough), then check `adb devices` |
| `protocol mismatch` | The robot app and the laptop tool are from different versions: rebuild and reinstall |
| Robot shows "settings invalid" | Settings were outside the robot's limits: see `settings_error` in `status` |
| Never fires | Check both RPMs on the Driver Station. Look for a dead encoder (one reads ~0) or a target too high for the battery (spin-up timeout). Make sure the laptop sent permission (link required) |
| A refused after a burst | Expected until the review confirms every ball out or you press X after checking the dead zone |
| Count wrong at loading | Re-check the laser baselines (section 3) and the `IntakeCount` logcat tag |
| Camera won't open | Check OS camera permission. Run `cameras` to find the index and pass `--camera N` |
| AI review "failed" | Key missing (`doctor` shows set/missing), refusal, or no frames. Every ball is unknown and nothing changes |
| Clock offset large or uncertain | `doctor` shows the method. `uptime` fallback is about ±30 ms; review frames span the whole burst either way |

**What still needs real hardware:** robot behavior, ADB on your machine, camera capture, and live API calls. The offline tests (`python -m unittest discover -s tests`) only cover the laptop logic with fakes.
