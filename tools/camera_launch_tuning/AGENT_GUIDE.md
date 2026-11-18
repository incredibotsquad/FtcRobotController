# AGENT_GUIDE: camera_launch_tuning internals

This guide is for anyone (human or coding agent) changing this tool or the `CameraLaunchTuner` OpMode. Read the repo's `CLAUDE.md` first for the robot architecture.

## Architecture

```
 Driver Station          Control Hub (CameraLaunchTuner)              Laptop (this tool)
 gamepad A/B/X/LT  --->  state machine + production commands   <--->  Session loop
                         status.json / command.json / trials/   ADB   transport -> camera -> reviewer -> policy -> store
```

- **The robot owns the mechanism.** That covers counting, loading, firing, and the refusal to refill.
- **The laptop owns evidence and decisions:** video, review, proposal checks, history and undo.
- **The robot re-validates everything it receives.** Settings must be finite and inside `LIMIT_*`, and authorizations are tied to a trial id and the robot run id. It never trusts the laptop blindly.

## Files

| File | Role |
|---|---|
| `TeamCode/.../opmodes/tuning/CameraLaunchTuner.java` | Robot OpMode: phases, settings freeze, file protocol, telemetry log |
| `camera_launch_tuning.py` | Entry point |
| `clt/cli.py` | argparse commands |
| `clt/session.py` | The loop: arm, record, fire permission, fetch, review, decide, push, checkpoint, undo |
| `clt/transport.py` | `AdbTransport` (adb CLI), `LocalDirTransport` (tests), `build_command`, clock offset |
| `clt/camera.py` | OpenCV recorder (thread, per-frame laptop timestamps), frame selection and extraction |
| `clt/telemetry.py` | CSV parsing; steady error, mismatch, dips, recovery, ball spacing; policy flags |
| `clt/policy.py` | Labels, normalization (ambiguity becomes unknown), goals, refill eligibility, validation streak, proposal checks |
| `clt/settings.py` | Parameters, units, bounds, identity hashing, production-table interpolation |
| `clt/store.py` | Run folders, state, append-only history and audit, discard, profiles |
| `clt/env.py` | `.env` parsing (literal), precedence, redaction, default models |
| `clt/providers/` | `base.py` (schema, prompt, request), `manual.py`, `openai_provider.py`, `anthropic_provider.py`, `agent_file.py` |
| `tests/` | Offline unittest suite (no network, camera or robot) |

## Robot state machine

```
INIT --START--> IDLE --A (refill allowed, settings valid)--> LOADING
LOADING --SmartIntake FULL, eject done, transfer stopped--> SPINUP
SPINUP --both motors stable STABLE_MS, turret settled, fire permission (if link required)--> LAUNCHING
SPINUP --SPINUP_TIMEOUT_MS / CAMERA_WAIT_TIMEOUT_MS--> IDLE (no burst; balls still held)
LAUNCHING --LaunchBallsCommand ends--> POST --POST_LOG_MS--> IDLE (trial files written)
any trial phase --B--> IDLE (aborted; needs empty confirmation if the gate opened)
any --hold LT / gp2 BACK--> CLEARING --release--> AWAIT_EMPTY_ACK --X--> IDLE
```

- `needs_empty_confirmation` becomes true when a burst starts or after any clearing. It clears only through operator X, or through a laptop `refill_authorized_trial` matching the last burst's trial id, not revoked, with count 0 and lasers 1 and 4 clear.
- **When commands are read:**
  - Settings, `refill_authorized_trial` and `link_required` are read only in INIT, IDLE and AWAIT_EMPTY_ACK. These are the review checkpoints, so a change never applies mid-shot.
  - `fire_trial` is read in LOADING and SPINUP.
- **Trial settings are frozen at A.** `LAUNCH_TRANSFER_POWER` and `MAX_FIRE_MS` are set when the gate opens and restored when the launch ends, and also in `finally`.

## Transport protocol (version 1)

Files live in `/sdcard/FIRST/data/camera_launch_tuning/` (`AppUtil.ROBOT_DATA_DIR`). Every write is a temp file followed by a rename, on both sides.

**`status.json`** (robot to laptop, about every 250 ms, and on phase changes; not during the burst) contains:

- **Identity and time:** `protocol`, `opmode`, `robot_run_id` (the OpMode start time), `wall_ms` (`System.currentTimeMillis`), `boot_ms` (`elapsedRealtime`).
- **Trial state:** `phase`, `trial_id`, `trial_open`, `last_burst_trial_id`, `needs_empty_confirmation`, `refill_authorized_trial`, `fire_authorized_trial`, `link_required`, `command_seq_seen`.
- **Settings:** `settings` (the next trial's), `trial_settings` (the frozen running trial's, or null), `settings_error`, `message`.
- **Live readings:** `artifact_count`, `left_rpm`, `right_rpm`, `battery_v`, `s1_mm`/`s1_blocked`, `s2_blocked`, `s3_blocked`, `s4_mm`/`s4_blocked`, `s1_baseline_mm`, `s4_baseline_mm`.
- **`production`:** shot table, gain table, `FLYWHEEL_*`, bang-bang and voltage-compensation settings, `launch_transfer_power`, `max_fire_ms`, `clear_ms`, `simple_launch`, `max_artifacts`, turret min/max/mid, `stored_turret_position`.
- **`limits`:** `[min, max]` per parameter, plus `stable_tolerance_rpm` and `stable_ms`.

**`command.json`** (laptop to robot, pushed as `.tmp` then `mv -f`) contains:

- `protocol`, `seq`, `session_id`, `sent_ms`, `robot_run_id`. Fire and refill fields addressed to another run id are ignored.
- `settings`: `{settings_id, target_rpm, p, ks, kv, transfer_power, max_fire_ms, turret_position}`. It is applied when `settings_id` differs from the current one and the whole set validates.
- `fire_trial`, `refill_authorized_trial`, `revoked_trials` (discarded trial ids; these void any authorization), `link_required`.

**`trials/trial_<run>_<NNNN>.json`** contains:

- **Identity:** `trial_id`, `robot_run_id`, `settings`.
- **Outcome:** `completion_reason`, `burst_started`, `gate_opened`.
- **Counts:** `count_at_fire`, `count_at_end`, `robot_counted_exits`, `empty_confirmed_at_timeout`.
- **At the moment of firing:** `left_rpm_at_fire`, `right_rpm_at_fire`, `battery_v_at_fire`.
- **`timestamps`:** `armed_ms`, `loaded_ms`, `spinup_start_ms`, `stable_ms`, `fire_ms`, `launch_end_ms`, `post_end_ms`, `finished_ms`. All are wall ms; a timestamp is null if that step never happened.
- **Context:** `stable_tolerance_rpm`, `stable_window_ms`, `link_required`, `sensors_at_end`, `production`, `telemetry_rows`, `telemetry_truncated`.

**Completion reasons:**

- From `LaunchBallsCommand.EndReason`: `empty`, `max_fire_ms`, `single_ball_cleared`, `nothing_loaded`, `interrupted`.
- From the tuner: `spinup_timeout`, `camera_not_ready`, `aborted`, `operator_clear`.

**`trials/...csv`**: one row per loop from SPINUP through POST:

```
t_wall_ms, t_boot_ms, t_rel_ms, phase, target_rpm, left_rpm, right_rpm, mean_rpm, power, bang_bang, battery_v,
count, s1_blocked, s1_mm, s2_blocked, s3_blocked, s4_blocked, s4_mm, gate_open, intake_power, transfer_power,
smart_state, launch_running, loop_ms
```

Rows are kept in memory and written once at the end of the trial, so file I/O doesn't slow the burst loop. The cap is `MAX_ROWS`.

**Clock sync:** `AdbTransport.clock_offset()` takes the shortest-round-trip sample of `adb shell date +%s%N`, which gives ms resolution where the hub's toybox supports it. Otherwise it falls back to `/proc/uptime` plus the status file's `wall_ms - boot_ms`, which has about 10 ms resolution. The offset (robot minus laptop), its uncertainty and the method are saved in `review_frames.json`, and frames are mapped to robot time with them.

## Parameter units

See `clt/settings.py: UNITS` and README.md. RPM is motor RPM, which equals flywheel RPM with the 1:1 belt. The gains act on ticks/s (goBILDA bare motor, 28 CPR). kV is defined at `NOMINAL_VOLTAGE`, and the robot voltage-compensates the feedforward.

## Sensor assumptions

The tuner doesn't hard-code any of these; it uses `IntakeSubsystem` as configured.

- Sensors 1 (entrance) and 4 (top, below the gate) are analog goBILDA lasers (`AnalogInput` in `res/xml/biobuzz.xml`).
- Sensors 2 and 3 are digital beam breaks.
- **The dead zone between the top of the transfer and the flywheel isn't covered by any sensor.** That's why refill needs camera-confirmed exits of every loaded ball, or an operator X.
- The baselines come from `IntakeSubsystem.SENSOR*_BASELINE_MM`. A value of 0 means the first INIT reading is used.

## Provider adapters

All adapters implement `Reviewer.review(ReviewRequest, folder=None) -> raw dict` and share `ASSESSMENT_SCHEMA`. Every object in the schema has `additionalProperties: false` and lists all its properties as required, which is a strict-mode requirement. Nullable fields use a type list. Ranges and counts are deliberately not in the schema; `policy.normalize_assessment` and `policy.check_proposal` enforce them locally.

- **OpenAI** (`openai_provider.py`) uses the Responses API, `client.responses.create`:
  - The input is a system message plus a user message made of `input_text` parts and `input_image` parts (`data:image/jpeg;base64,...`, `detail: high`).
  - The output format is `text={"format": {"type": "json_schema", "name", "schema", "strict": True}}`.
  - It parses the `output_text` part. Refusal parts and `status == "incomplete"` raise `ReviewError`.
- **Anthropic** (`anthropic_provider.py`) uses the Messages API, `client.beta.messages.create`:
  - It sends `betas=["server-side-fallback-2026-07-01"]` and `fallbacks="default"`, with `output_config={"effort": "high", "format": {"type": "json_schema", "schema"}}`.
  - Images are base64 `image` blocks.
  - No `thinking` parameter is sent: it is always on for `claude-opus-5-5`, and disabling it or giving a budget is rejected.
  - A `stop_reason` of `refusal` or `max_tokens` raises `ReviewError`.
- **agent-file** writes `review_request/` and waits for `assessment.json`. It needs no key. Run your own agent session pointed at the folder.
- **manual** asks at the terminal.

A `ReviewError`, no frames, or a refusal means every ball is unknown. That makes the trial inconclusive: no change and no refill. **Keys are only read by the SDK clients from the environment.** They are never placed in prompts or files, and a test checks that no saved file contains a key.

## Persistence and undo

- `state.json` is the trial index. `history.jsonl` and `audit.jsonl` are append-only.
- **Current settings are replayed** from `run.json:initial_settings` plus the `accepted_change` of each kept trial, in order. Nothing stores "current settings" separately.
- **Discarding** marks the last kept trial as `discarded` and records the removed change. It also sets `refill_eligible=false`, clears `validated`, writes `DISCARDED.json` and appends an audit entry.
  - Replaying the kept trials then gives the settings the discarded trial ran with.
  - The session pushes those settings, plus `revoked_trials`; the robot applies them at idle.
- `context_history()`, `validation_status()` and `refill_eligible()` all skip discarded trials.

## Limitations

- The review sees about 16 frames, so very fast events between frames can be missed and labeled unknown. Raise `--frames` or the camera fps.
- Clock alignment is ±(round trip / 2 + resolution), usually 5–30 ms. Frames span the whole burst, so ordering is still visible.
- The fixed turret position doesn't validate TeleOp's auto-aim.
- The settling rules are the production ones. The tuner adds none of its own.
- The tuner has no I/D tuning and no visor/hood (there is none on this robot).
- Only POLLEN is in scope.
- Robot behavior, ADB, camera and live API calls are not covered by the tests. They need the physical setup.

## Tests

```bash
python -m unittest discover -s tests -v
```

The suite runs on the Python standard library only.

| Test file | Covers |
|---|---|
| `test_env.py` | Literal `.env` parsing, precedence, explicit-path errors, redaction, the blank `.env.example` |
| `test_policy.py` | Proposal bounds and evidence rules, kS/kV identifiability, ambiguity, goals, refill, validation streaks |
| `test_store_undo.py` | Settings replay, discard and restore, preserved files, audit, CLI undo |
| `test_transport.py` | Command contract, adb command lines (atomic push and rename, serial, pull paths), clock offset parsing and fallback, local transport |
| `test_providers.py` | Strict-schema rules, OpenAI and Anthropic request and response contracts with fake clients, refusal handling, agent-file round trip and timeout, manual |
| `test_session.py` | Telemetry metrics, `process_trial` (stuck 4th, refill, undo revocation, applied and rejected proposals, no frames, keys never saved, validation), and a simulated-robot handshake |

## Extending

- **New parameter:**
  1. Add it to `CameraLaunchTuner.Settings` (JSON fields, `validate()`, `LIMIT_*`) and apply it in the trial.
  2. Add it to `PARAMS`, `UNITS` and `DEFAULT_BOUNDS`, and to the `build_command` keys.
  3. Give it an evidence rule in `check_proposal`, and add tests.
  4. Bump `PROTOCOL_VERSION` on both sides if the meaning of an existing field changes.
- **New provider:** subclass `Reviewer`, return a dict matching `ASSESSMENT_SCHEMA`, and raise `ReviewError` on refusal or malformed output. Register it in `providers/__init__.py:make_reviewer` and the CLI choices, and add a fake-client contract test.
- **Keep production behavior unchanged.** Only add read-only accessors or opt-in overloads to production classes, as with `updateFlywheelWithGains` and `getEndReason`.
