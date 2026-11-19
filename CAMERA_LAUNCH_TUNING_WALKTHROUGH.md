# Launch tuner walkthrough

This is a step-by-step guide to a tuning session with `CameraLaunchTuner` and the laptop program. Parts 1 and 2 are done once. Parts 3–8 are repeated for each session. [CAMERA_LAUNCH_TUNING.md](CAMERA_LAUNCH_TUNING.md) has the full reference and troubleshooting.

**Where to run laptop commands:** in the `tools/camera_launch_tuning` folder, with the virtual environment active. Commands below say `python`. On macOS/Linux, if `python` isn't found, use `python3`.

**What you need:**
- The robot with a charged battery.
- A Driver Station with **gamepad 1** (gamepad 2 is optional).
- The laptop and a USB-C cable to the Control Hub (or Wi-Fi ADB).
- A webcam on a tripod.
- At least 4 POLLEN, plus one helper to feed balls and collect them.

---

## Part 1: One-time laptop setup (about 20 minutes)

1. **Get the code.** Pull the latest `BioBuzz` branch.
2. **Install Python 3.10 or newer.** Check with `python --version`.
3. **Create the environment and install the packages:**
   ```bash
   cd path/to/FtcRobotController/tools/camera_launch_tuning
   python -m venv .venv
   source .venv/bin/activate          # Windows PowerShell: .\.venv\Scripts\Activate.ps1
   pip install -r requirements.txt
   ```
   Every later session starts with `cd` into this folder and the `activate` line.
4. **Check that adb is available.** Run `adb version`. If it isn't found:
   - Android Studio installs it in its SDK `platform-tools` folder, and the tool looks there automatically.
   - Or install "Android SDK Platform-Tools" and add it to PATH.
   - Or pass `--adb /full/path/to/adb` to each command.
5. **Allow camera access** for your terminal:
   - **macOS:** System Settings > Privacy & Security > Camera.
   - **Windows:** Settings > Privacy & security > Camera, then allow desktop apps.
   - **Linux:** run `sudo usermod -aG video $USER`, then log out and back in.
6. **(Optional) AI review keys.** Skip this if you'll score trials yourself.
   ```bash
   cp .env.example .env      # Windows: copy .env.example .env
   ```
   Open `.env` and paste your key after `OPENAI_API_KEY=` or `ANTHROPIC_API_KEY=`. Leave the model lines blank to use the defaults.
   - The file is git-ignored. Never commit it.
   - A ChatGPT/Codex/Claude login is **not** an API key.
   - If Claude Code is installed (the `claude` command), you don't need a key: use `--reviewer claude-cli` (Part 7).
7. **Check the setup:**
   ```bash
   python camera_launch_tuning.py doctor
   python camera_launch_tuning.py cameras
   ```
   - `doctor` should show `[ok] cv2`. The `openai`/`anthropic` lines only matter if you use those reviewers.
   - The `claude` line shows `[ok]` when the Claude Code CLI is on PATH. To confirm it's logged in and can return results, run `python camera_launch_tuning.py doctor --probe-claude`. This makes one tiny call that uses a little of your Claude usage.
   - If all three modules show `[missing]`, the virtual environment isn't active. Every new terminal needs the `activate` line from step 3; the prompt then starts with `(.venv)`. You can also skip activating by running `.venv/bin/python camera_launch_tuning.py ...` (Windows: `.venv\Scripts\python camera_launch_tuning.py ...`).
   - `cameras` lists camera numbers, but the numbers don't tell you which camera is which (OpenCV's order differs from macOS's, and it can change after replugging). Check each with `python camera_launch_tuning.py preview --camera N --live`, and watch which camera's light turns on. `run` also saves a snapshot and asks you to confirm the view before the first trial.

## Part 2: One-time robot setup

1. **Connect the laptop to the Control Hub** (robot powered on):
   - **USB:** use a USB-C cable to the Control Hub.
   - **Wi-Fi:** join the robot's Wi-Fi, then run `adb connect 192.168.43.1:5555`.
     - **On macOS,** start the ADB server in its own Terminal tab first, with `adb kill-server` then `adb nodaemon server`, and leave that tab open. A normal background ADB server gets blocked by macOS's Local Network privacy and fails with "No route to host", and Terminal doesn't appear in that settings list.
     - Keep Android Studio closed while tuning; it restarts ADB and drops the Wi-Fi connection.
     - Add `--connect 192.168.43.1` to tuner commands so they reconnect automatically.
     - If you also need internet, use wired Ethernet and put it above Wi-Fi in System Settings > Network > ⋯ > Set Service Order.

   `adb devices` should list one device. If it lists two, add `--serial <id>` to every command.

   **Use the USB-C cable if you want internet during tuning.** Joining the robot's Wi-Fi takes the laptop off the internet, and the AI reviewers (claude-cli, openai, anthropic) need it. With the cable, the laptop stays on its normal Wi-Fi; only the Driver Station needs the robot's Wi-Fi. If you must use Wi-Fi ADB, either add a second internet connection (USB Wi-Fi adapter, Ethernet, or phone tethering over USB), or tune with `--reviewer manual` and run AI reviews later with `review`. Unplug the cable before driving.
2. **Install the app** from the repo root:
   ```bash
   ./gradlew installDebug          # Windows: gradlew.bat installDebug
   ```
   Reinstall whenever the robot code changes.
3. **Calibrate the lasers.** Do this once, and again if a sensor is moved.
   1. Empty the robot completely.
   2. On the Driver Station, select **CameraLaunchTuner** and press **INIT** only. Nothing moves in INIT.
   3. Read "Sensor 1 laser (mm / baseline)" and "Sensor 4 laser (mm / baseline)". The first number (the reading) should be within about 3 mm of the second (the baseline).
   4. If a reading is far off, put the clear reading into `IntakeSubsystem.SENSOR1_BASELINE_MM` or `SENSOR4_BASELINE_MM`, then rebuild and reinstall.
   5. "Sensor 2 beam" and "Sensor 3 beam" should read `clear`.
   6. Put a ball in front of each sensor in turn. Its reading should change (a laser reads closer, a beam reads BLOCKED).

---

## Part 3: Set up the session (about 10 minutes)

1. **Battery:** use a charged battery (above 12.5 V at rest). Use the same battery and charge level for every trial in a session, and write down which battery you used.
2. **Place the robot** on the spot you want to tune, for example the RIGHT_SPOT_1 tape mark. The robot doesn't drive during tuning; it stays exactly where you put it.
3. **Find the distance** from the robot's center to the target CELL, in inches. Either:
   - measure it with a tape (field coordinates are in `opmodes/auto/Poses.java`; the CELLs are in `LaunchTargets`), or
   - read it in TeleOp: turn on `DebugTelemetry.ENABLED` and look at "Launch: Distance (in)".
4. **Set the turret aim.** This is the easiest way to get a good starting turret position:
   1. Run the normal TeleOp (`IncredibotsMecanumDrive`) with the robot on the spot.
   2. Let the turret auto-aim at the CELL.
   3. Press STOP.

   The turret position stays in memory until the robot app restarts, and the tuner starts from it.
5. **Mount the camera:**
   - Put it on a tripod **to one side of the robot, square to the line from the robot to the HIVE**, about 6–8 ft away and at about CELL height (3.5–4 ft).
   - It must see the launcher exit, the whole arc, and the CELL opening.
   - Not from behind the robot: from there the balls fly straight away from the lens, and short and long can't be told apart.
   - Lock the tripod. If the camera moves, earlier trials are no longer comparable.
   - Keep people out of the frame during a trial; moving people confuse the motion trails.
   - Check the view with `python camera_launch_tuning.py preview --camera N --live`, and write the camera position in the spot description.
6. **Prepare the balls.** Use 4 POLLEN. If you can, use the same 4 every time and number them; sizes vary from ball to ball.

## Part 4: Create the spot profile (first time at a spot)

1. On the Driver Station, select **CameraLaunchTuner** and press **INIT** (not START yet).
2. On the laptop, check the robot is visible:
   ```bash
   python camera_launch_tuning.py status
   ```
   You should see `"phase": "INIT"` and sensor readings.
3. Create the profile with your distance. This copies TeleOp's RPM and gains for that distance, the turret position from Part 3, and the production transfer power and timeout:
   ```bash
   python camera_launch_tuning.py spot-init --spot right1 --from-robot --distance 48.75 \
       --description "RIGHT_SPOT_1, battery #3, camera on the right side, 7 ft, CELL height"
   ```
   - If it says a value is missing (most often `turret_position`, if TeleOp didn't run first), add it, for example `--turret-position 0.55`.
   - Each spot gets its own name, for example `right1`, `right2`, `left1`.
4. Check the profile with `python camera_launch_tuning.py spot-show --spot right1`.

## Part 5: Your first trial (manual scoring)

Start with manual scoring so you see each step happen.

1. **Start the session on the laptop:**
   ```bash
   python camera_launch_tuning.py run --spot right1 --reviewer manual --camera 0 --max-trials 3
   ```
   Use the camera number from Part 1, step 7. The laptop prints the run folder, sends the settings, and waits.
2. **Press START on the Driver Station.** The intake bar raises and the turret moves to the trial position. The Driver Station shows "Phase IDLE" and "Press gamepad 1 A to load 4 balls."
3. **When the laptop prints "press gamepad 1 A and feed 4 balls",** press **A** on gamepad 1.
   - Phase becomes `LOADING` and the intake and transfer start.
   - Your helper feeds balls into the intake **one at a time** until "Count" reads **4**.
   - The robot then runs a little longer to pull the last ball in, reverses the intake briefly to push out any 5th ball, and stops. **Don't feed a 5th ball.**
4. **Spin-up and launch happen automatically:**
   - Phase becomes `SPINUP` and the flywheel spins up.
   - The laptop prints "Recording...".
   - When both flywheel motors hold the target speed, the robot opens the gate and fires all the balls (phase `LAUNCHING`).
   - It logs for another 1.5 seconds, then stops the flywheel and returns to `IDLE`.
   - **Stand clear of the launcher and the target area during SPINUP and LAUNCHING.**
5. **Score the trial.**
   1. Open the video: `runs/<run folder>/trials/0001/video.mp4`.
   2. Label each ball in the order it left: `s` scored, `h` short, `l` long, `f` left, `r` right, `k` stuck (never left), `u` unknown (couldn't see it).
   3. Rate the footage (`good`, `partial` or `poor`).
   4. Optionally propose one change, for example `target_rpm` then `2718` if balls landed short. You'll only be asked to confirm it if it passes the checks.
6. **Read the result line.** For example:
   ```
   Trial 1: scored scored short stuck -> FAILURE (2 scored, 3 confirmed out, 0 unknown)
     refill: NOT allowed - 1 ball(s) not confirmed out (stuck or unknown): clear and press X
     validation streak: 0/3
   ```
7. **Get the robot ready for the next trial** (Part 6). Then, at the laptop prompt `[Enter] next trial | u = discard the last trial | q = quit`, press **Enter**.

## Part 6: Between trials (every time)

1. **Collect all the balls.** Get the same 4 back if you can.
2. **Check whether the robot will let you load:**
   - **"refill: allowed"** (every ball was confirmed out): press **A** when the laptop asks.
   - **"refill: NOT allowed"**, or the Driver Station says "Refill refused" when you press A:
     1. Hold **LT** on gamepad 1 (or BACK on gamepad 2) for 1–2 seconds. The intake and transfer reverse, the flywheel stops, and the gate stays closed. Catch anything that comes out.
     2. Release LT. The Driver Station asks you to check the robot and press X.
     3. **Look into the transfer and the area between the top of the transfer and the flywheel (the dead zone).** No sensor covers it, so a ball can sit there while the robot reads 0. If a ball is there, disable the robot and remove it by hand.
     4. Press **X** ("physically empty"). The count resets to 0.
     5. Press **A** to load the next set.
3. **Check the battery** on the Driver Station ("Battery / loop"). If it drops below about 12.3 V under load, swap to another charged battery. Note in your log that you swapped.

## Part 7: The tuning loop

Repeat Parts 5 and 6, steps 3–7 each time. How it works:

- **Settings only change between trials.** After each review, the laptop sends the next settings and the robot applies them while it's idle, never during a shot.
- **One change at a time.** A change is applied only after a clear failure, when the evidence supports it:
  - **Short or long misses**, with the flywheel holding speed: change `target_rpm`, at most 150 RPM per step.
  - **Left or right misses:** change `turret_position`, at most 0.01 per step.
  - **Flywheel not holding speed, or slow recovery between balls:** change kV or P.
  - **A ball stuck while the flywheel was fine:** that's mechanical. Fix the robot, not the gains.
- **No change after a success.** The same settings are kept so they can be confirmed.
- **The goal is validated** after **3 successes in a row with identical settings**. The default success is at least 3 of 4 balls scored. Use `--goal 4of4` to require all 4. The laptop prints `VALIDATED` and saves the settings as the spot's "selected" settings.
- **A run also stops** at `--max-trials` (default 20), or when you press `q`.

**Switching to AI review** once manual trials work:

```bash
python camera_launch_tuning.py run --spot right1 --reviewer anthropic     # or --reviewer openai
```

Add `--auto-apply` to accept checked proposals without the y/N question.

**Calibrate it first** with `--confirm-labels`:
- After each AI review, press Enter if the labels are right, or type the right ones (for example `l l l h`). Your labels are used for that trial.
- When you correct a trial, you can save it as a calibration example. Saved examples are shown to the reviewer on every later trial at this spot. You can also save one later with `example-add --run <run> --trial N`.
- The result line shows the agreement so far. When it stays near 100% for a few trials, drop `--confirm-labels`. The AI only sees saved frames from each finished trial. Whenever it isn't sure, it marks a ball unknown; that trial then counts as inconclusive and changes nothing.

**Reviewing with your Claude Code login (no API key), fully automatic:**

```bash
python camera_launch_tuning.py run --spot right1 --reviewer claude-cli
```

- After each trial, the tuner runs `claude -p` headless in that trial's `review_request/` folder.
- That session only has the Read tool: it can look at the frames and nothing else (no shell, no edits, no web).
- It returns the labels in the required format, and the loop continues by itself. Expect about 20–60 seconds per review.
- It uses your Claude Code setup's usage.
- Choose a model with `--claude-model opus` (or `sonnet`); the default is the CLI's default.
- If a review fails, the trial counts as inconclusive. The CLI's output is saved in `claude_cli_result.json` / `claude_cli_stderr.txt` in that folder.

**Reviewing with your own Codex/Claude session, by hand:**
1. Run with `--reviewer agent-file`.
2. After each trial, open `runs/<run>/trials/NNNN/review_request/REVIEW_REQUEST.md` in your agent session.
3. Have the agent write `assessment.json` in that folder. The laptop continues automatically.

## Part 8: Situations and what to do

| Situation | What to do |
|---|---|
| Something looks wrong mid-trial | Press **B** on gamepad 1: the trial aborts and the gate closes. If the gate had opened, clear and press X before reloading (Part 6) |
| A ball jams in the intake | Hold **LT** to reverse, release, check the robot, press **X**, then **A** |
| A trial was bad for reasons unrelated to tuning (someone walked in front of the camera, ball bounced off a person, wrong ball fed) | At the checkpoint prompt, type **u**. That trial is ignored from then on and the settings go back to what it ran with. Files are kept |
| You only notice a bad trial later | `python camera_launch_tuning.py undo --run <run folder name>` (add `--push` with the robot connected to send the restored settings) |
| Robot never fires | Phase stays `SPINUP`, then the trial ends with "spinup_timeout". Check that both "Flywheel L / R" numbers rise. One stuck near 0 means an encoder cable is loose. A low battery can't reach a high RPM |
| Trial ends with "camera_not_ready" | The laptop wasn't running or lost the connection. Restart `run`, then press A again. The balls are still loaded, so they don't need re-feeding |
| Count never reaches 4 | Press **B**. Check the laser baselines (Part 2, step 3). Make sure balls are fed one at a time |
| You need to stop for the day | Press `q` on the laptop and STOP on the Driver Station. Continue later with `run --spot right1 --resume <run folder name>` |
| You restarted the OpMode or app | The robot forgets an earlier burst happened. **Always check the dead zone by hand before the first A** |

## Part 9: Finish and use the results

1. **See what happened:**
   ```bash
   python camera_launch_tuning.py history --run <run folder name>
   ```
2. **If the run validated,** the selected settings are already saved in `profiles/right1.json`. To save an unvalidated result on purpose, run `select --run <run> --unvalidated`.
3. **Print the values to copy into code:**
   ```bash
   python camera_launch_tuning.py export --spot right1
   ```
4. **Copy them into the match code by hand.** The tuner never edits match code.
   - The RPM goes into `LaunchReadinessCommand.SHOT_DISTANCES_IN` / `SHOT_RPMS` at the spot's distance.
   - P, kS and kV go into the matching `LaunchSubsystem.GAIN_TABLE_*` row.
   - Check `LAUNCH_TRANSFER_POWER` and `MAX_FIRE_MS` too.
   - The two shot table arrays (`SHOT_DISTANCES_IN` and `SHOT_RPMS`) must stay the same length and sorted. The same goes for the gain table arrays.
5. **Check in TeleOp:**
   1. Rebuild and reinstall.
   2. Drive to the spot in TeleOp and launch several full sets with RB.
   3. Make sure the auto-aim turret position matches the tuned one. The tuner used a fixed aim, while TeleOp aims on its own.
6. Commit the code change when you're happy with it.
7. Repeat Parts 3–9 for the next spot.

## Quick checklist card

```
SESSION START                        EACH TRIAL
[ ] battery charged, noted           [ ] laptop says "press A"
[ ] robot on spot, turret aimed      [ ] A, feed 4 balls one at a time
[ ] camera locked, sees CELL         [ ] stand clear during SPINUP/LAUNCHING
[ ] adb devices shows the hub        [ ] score / review, read the result line
[ ] DS: CameraLaunchTuner INIT       [ ] collect balls
[ ] laptop: run --spot ...           [ ] refill allowed? else LT, check dead zone, X
[ ] DS: START                        [ ] Enter (or u to discard)
```
