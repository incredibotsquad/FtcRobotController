# camera_launch_tuning: operator and configuration reference

Laptop half of the camera-assisted launch tuner. For setup and the first run, see [CAMERA_LAUNCH_TUNING.md](../../CAMERA_LAUNCH_TUNING.md). Internals are in [AGENT_GUIDE.md](AGENT_GUIDE.md).

All commands run from this folder: `python camera_launch_tuning.py <command> [options]`. Global options:

- `--env-file PATH`: an explicit credentials file.
- `--version`: print the version.

## Commands

| Command | Purpose |
|---|---|
| `doctor [--probe-claude]` | Python modules, Claude Code CLI, key presence (never values), adb, devices, robot status, clock offset. `--probe-claude` makes one tiny headless call to confirm login and structured output |
| `cameras` | Cameras OpenCV can open, with their resolutions (numbers only: identify them with `preview`) |
| `preview --camera N [--live]` | Save a snapshot (`camera_preview.jpg`) and optionally show a live window, to identify a camera and aim it |
| `status` | The robot's `status.json`: phase, count, sensors, settings, errors |
| `spot-init --spot NAME` | Create a profile. `--from-robot --distance IN` uses production tables; any parameter flag overrides one value; `--force` replaces an existing profile |
| `spot-show --spot NAME` | Print a profile |
| `run --spot NAME` | Tuning session (options below) |
| `undo --run RUN [--push] [--reason TEXT]` | Discard the immediately last kept trial of a saved run |
| `history --run RUN` | Trials with status, outcome, labels, settings and changes, plus the validation streak |
| `review --run RUN --trial N --reviewer R` | Re-review a saved trial with another reviewer. Saves the answer and changes no decisions |
| `select --run RUN [--unvalidated]` | Record the run's last kept settings as the spot's selected settings |
| `example-add --run RUN --trial N [--labels 'l l l h']` | Save a person-labeled trial (its motion trails plus a rest frame) as a calibration example for the spot. Labels default to the trial's manual or corrected labels |
| `example-list --spot NAME` / `example-remove --spot NAME --id ID` | List or delete calibration examples |
| `export --spot NAME` | Print a promotion note for the selected settings. Changes no files |

`RUN` is a run id (a folder name under `runs/`) or a path. `adb` is found on PATH first, then in the usual Android SDK locations; override with `--adb` and pick a device with `--serial`.

Wi-Fi ADB: `--connect HOST[:PORT]` (port defaults to 5555) runs `adb connect` and re-connects once whenever a call finds the device gone, which happens when something restarts the ADB server. On macOS, run the server as `adb nodaemon server` in its own Terminal tab. A daemonized server is blocked by Local Network privacy ("No route to host").

### `run` options

| Option | Default | Meaning |
|---|---|---|
| `--reviewer` | `manual` | `manual`, `openai`, `anthropic`, `claude-cli` (local Claude Code login), or `agent-file` |
| `--goal` | `3of4` | `3of4`: at least 3 confirmed scores. `4of4`: all 4 |
| `--max-trials` | 20 | Stop after this many trials. Discarded trials count |
| `--validation-required` | 3 | Consecutive goal successes at identical settings needed to validate |
| `--auto-apply` | off | Apply accepted proposals without the y/N prompt |
| `--confirm-labels` | off | After each AI review, press Enter to accept its labels or type the right ones (e.g. `l l l h`). Corrections become the trial's labels, count toward the agreement score, and can be saved as examples |
| `--max-examples` | 2 | Calibration examples shown to the reviewer per trial (the newest ones for the spot; 0 = none) |
| `--resume RUN` | | Continue a run. Settings replay from its kept trials |
| `--start-from-selected` | off | Start from the profile's selected settings instead of its starting settings |
| `--camera` / `--width` / `--height` / `--fps` | 0 / 1280 / 720 / 60 | Capture device and format |
| `--frames` | 16 | Review frames per trial, from 300 ms before the gate opens to 1 s after the launch ends |
| `--skip-camera-check` | off | Don't save a start-of-run snapshot and ask to confirm the view |
| `--no-camera` | off | No recording. With `manual`, score by eye. AI reviewers then get no frames, so every ball is unknown |
| `--connect` | | Wi-Fi ADB address with auto-reconnect, for example `192.168.43.1` |
| `--agent-timeout` | 1800 s | How long `agent-file` waits for `assessment.json`, or a `claude-cli` review may run |
| `--claude` / `--claude-model` | PATH / CLI default | `claude-cli`: CLI path and model (alias such as `opus`/`sonnet`, or a full name) |

## Parameters and bounds

| Parameter | Unit | Laptop limits | Max step per change | Robot limits |
|---|---|---|---|---|
| `target_rpm` | flywheel RPM (1:1 belt, so motor RPM) | 1500–4500 | 150 | 1000–5500 |
| `p` | power per tick/s of error | 0–0.15 | 0.01 | 0–0.5 |
| `ks` | motor power | 0–0.35 | 0.03 | 0–0.5 |
| `kv` | power per tick/s at nominal voltage | 0.0001–0.0008 | 0.00003 | 0–0.002 |
| `transfer_power` | motor power | 0.4–1.0 | 0.1 | 0.2–1.0 |
| `max_fire_ms` | ms (whole number) | 1500–6000 | 500 | 500–10000 |
| `turret_position` | servo position | 0.2–1.0 | 0.01 | `TURRET_MIN`–`TURRET_MAX` |

Each session uses the tighter of the laptop limits (`clt/settings.py: DEFAULT_BOUNDS`) and the limits the robot reports (`CameraLaunchTuner.LIMIT_*`). The merged bounds are saved in `run.json`.

- **The robot rejects out-of-range settings as a whole**; it never clamps them.
- I and D are not tuned: the robot uses `LaunchSubsystem.FLYWHEEL_I`/`FLYWHEEL_D`.

## Ball labels and outcomes

**Labels**, assigned in launch order:

- `scored`
- `short`, `long`, `left`, `right` (as seen from behind the robot)
- `stuck` (never left)
- `unknown`

**Counting:**

- **Confirmed exits** are balls labeled scored, short, long, left or right.
- **Confirmed scores** are balls labeled scored.
- Unknown balls count as neither.

**Labels become `unknown`** when:

- confidence is below 0.6;
- the footage is rated poor;
- an index is duplicated, missing, or out of range;
- a label isn't recognized;
- the reviewer fails or refuses.

**Outcome:**

- `success`: confirmed scores reach the goal.
- `inconclusive`: unknown balls could still decide it, or there was no burst.
- `failure`: otherwise.

Inconclusive trials never trigger a change and break the validation streak.

**Refill authorization:** the laptop authorizes the robot to load the next set only when the review confirmed **every** loaded ball out and the robot's count ended at 0. Otherwise, clear the robot and press X.

## Calibrating the reviewer

The reviewer is calibrated through context, not training. Three things shape its labels:

1. **Outcome rules** in its instructions. Scored means the ball enters the up-facing CELL. Long means it goes over the HIVE. Short means it hits the outside of the HIVE or never reaches the raised CELL. Where a ball lands afterwards doesn't matter.
2. **Calibration examples:** person-labeled trials from the same spot and camera, shown before each new trial.
3. **Confirm mode** (`--confirm-labels`): you check every AI label until the agreement score shown after each trial stays high. Then drop the flag.

A trial is never used as its own example.

## Tuning rules (enforced in `clt/policy.py`)

One change per trial, and only after a conclusive failure.

- **`target_rpm`:** needs misses in one direction only. Lower it for balls that go over (long), raise it for balls that come in under (short). Over and under in the same trial are a consistency problem, so no RPM change is made. The flywheel must also be reaching its target.
- **`turret_position`:** needs left/right misses.
- **`p`:** needs slow recovery (a dip that isn't back within 40 RPM in 300 ms) or noise above 25 RPM.
- **`kv`:** needs a steady-state error above 40 RPM in the 300 ms before the gate opens.
- **`ks`:** also needs steady data at two RPMs at least 300 apart. kS and kV can't be separated at one speed.
- **`transfer_power`:** needs slow recovery between balls or a stuck ball.
- **`max_fire_ms`:** needs a launch that ended at `max_fire_ms`.
- **Gain changes are refused** when tracking is good. A stuck ball with good tracking is mechanical, not a gain problem.

## Files

```
tools/camera_launch_tuning/
  .env.example        blank credentials template (copy to .env)
  .env                your keys (git-ignored)
  runs/               recordings, telemetry, reviews (git-ignored)
  profiles/           spot profiles (git-ignored)
```

Each trial folder `runs/<run>/trials/NNNN/` contains:

| File | Contents |
|---|---|
| `robot_trial_*.json` | Robot summary: settings, timestamps, completion reason, counts, RPM at fire |
| `robot_trial_*.csv` | Per-loop telemetry |
| `video.mp4`, `frames.csv` | Clip and per-frame laptop time |
| `review_frames/`, `review_frames.json` | Frames sent for review, with times from gate open and clock offset |
| `metrics.json` | Steady error, motor mismatch, dips, recovery, ball spacing |
| `assessment_raw.json`, `assessment.json` | The reviewer's answer and the normalized labels |
| `decision.json` | Proposal, check result, accepted change, refill decision |
| `review_request/` | Agent-file bundle (`REVIEW_REQUEST.md`, `request.json`, `schema.json`, frames, `assessment.json`) |
| `DISCARDED.json` | Present if the trial was discarded |

Run level: `run.json`, `state.json`, `history.jsonl` (events) and `audit.jsonl` (discards and every command pushed).

## Robot-side tunables (Panels or code, `CameraLaunchTuner`)

| Name | Default | Meaning |
|---|---|---|
| `STABLE_TOLERANCE_RPM` | 40 | Each motor within this of the target... |
| `MOTOR_MISMATCH_RPM` | 60 | ...and of each other... |
| `STABLE_MS` | 400 | ...for this long, not in bang-bang |
| `SPINUP_TIMEOUT_MS` | 8000 | Trial ends with no launch if speed never stabilizes |
| `CAMERA_WAIT_TIMEOUT_MS` | 30000 | Trial ends with no launch if the laptop never permits firing |
| `POST_LOG_MS` | 1500 | Logging after the launch ends |
| `TURRET_EXTRA_SETTLE_MS` | 250 | Wait added to the estimated turret travel time |
| `LIMIT_*` | see above | Absolute limits for received settings |

## Credentials

- `OPENAI_API_KEY` / `OPENAI_MODEL` (default `gpt-6.2-astra`) and `ANTHROPIC_API_KEY` / `ANTHROPIC_MODEL` (default `claude-opus-5-5`).
- Sources: the shell environment, then `.env` (shell wins), or `--env-file`.
- The file is parsed as text; nothing in it is executed. Values never appear in output, prompts, run files or robot files.
- An interactive Codex/Claude login is not an API key. `--reviewer claude-cli` uses your Claude Code login through the CLI, with API keys removed from its environment. `--reviewer agent-file` works with any agent session you prompt by hand.
