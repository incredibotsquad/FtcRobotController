package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeBarSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;

import java.util.Locale;

/**
 * One-press "empty the robot" shot. Schedule with whenPressed().
 *
 * LaunchReadinessCommand keeps the flywheel at speed and the turret on target the whole time.
 * The press starts the steady-flywheel wait (WAIT_FOR_STEADY_FLYWHEEL; a flywheel already steady
 * before the press counts) and, with the kick, the transfer reverse at the same moment.
 *
 * SIMPLE_LAUNCH (default on): none of the kick logic below. The gate opens, the transfer feeds
 * forward at launch power the whole time (no reverse), and the launch ends once the count reads 0
 * for CLEAR_MS (or MAX_FIRE_MS), the same way for any number of artifacts: no lone-ball or
 * last-ball special cases. Turn it off to bring back the kick behavior described next.
 *
 * Kick (every launch, when KICK_ENABLED and not SIMPLE_LAUNCH): an artifact resting against the closed gate starts from
 * rest, and the dead zone between the top of the transfer and the flywheel (above sensor 4) can
 * stop it. So every launch starts with a run-up:
 *  1. On the press the transfer runs in REVERSE, gate still closed (REVERSE_BEFORE_GATE), for a
 *     time that depends on how many artifacts are loaded (KICK_REVERSE_MS_BY_COUNT): a full stack
 *     can only back up a little before the bottom artifact is pushed out of the intake. Pulling
 *     the top artifact off the gate first keeps the opening gate from catching in its holes.
 *     The gate opens GATE_OPEN_DELAY_MS into the reverse, or later once the flywheel is steady
 *     (the steady wait runs at the same time as the reverse, from the press).
 *  2. Transfer runs forward at full launch power, so the top artifact reaches the dead zone moving.
 *     If the gate opened late, the transfer stops after the reverse and waits until the gate has
 *     been open MIN_GATE_LEAD_MS.
 *  The intake motor runs forward (KICK_INTAKE_POWER) during the reverse and for KICK_INTAKE_EXTRA_MS
 *  into the forward drive, holding in artifacts the reversed transfer pushes down. After that it is
 *  off again, as in any launch, so nothing new is pulled in mid-shot.
 *
 * Then:
 *  - 2+ artifacts: each one pushes the one ahead of it through. Ends once IntakeSubsystem counts 0
 *    (plus CLEAR_MS so the last one gets through).
 *  - 1 artifact: the count can't be trusted (it drops when the ball passes sensor 4 even if it then
 *    stalls above it and falls back). The ball must come back up to sensor 4 within
 *    KICK_RETURN_TIMEOUT_MS, and then clear it for CLEAR_MS. If it doesn't come back, or sits on
 *    sensor 4 for KICK_RETRY_MS, it is kicked again, up to MAX_KICKS times in total.
 *  - Either way, MAX_FIRE_MS is a hard backstop.
 * The transfer and intake are driven through IntakeSubsystem's launch overrides, which
 * SmartIntakeCommand applies.
 *
 * Requires only the gate. LaunchReadinessCommand (launcher) keeps aiming and spinning the
 * flywheel during the burst, and SmartIntakeCommand (intake) keeps running the transfer.
 * When this ends, CloseGateCommand (gate default) takes over.
 *
 * Intake bar: on the press the bar is commanded up and the launch starts right away (no wait for
 * it to finish moving). It is kept raised for the whole launch, so an operator toggle mid-launch is
 * undone; afterwards it stays up until the operator lowers it. The bar is not a requirement, so
 * pressing the bar button never cancels a launch.
 *
 * Launch hold: for the whole command it asks DriveSubsystem to hold the
 * robot's pose against defense. DriveRobotCommand does the holding and lets the driver break out
 * with the sticks; a breakout does NOT stop the launch. Turned off by DriveSubsystem.LAUNCH_HOLD_ENABLED.
 * The drive is not a requirement, so DriveRobotCommand keeps running.
 */
@Configurable
public class LaunchBallsCommand extends CommandBase {

    /**
     * Wait for a steady flywheel before opening the gate (added 2026-11-18): both motors within
     * STEADY_TOLERANCE_RPM of the target, not in bang-bang, for STEADY_MS. Gives up after
     * MAX_STEADY_WAIT_MS and fires anyway (logged), so a launch is never blocked for long.
     * Every launch also logs the flywheel speed at the gate opening and at each exit (tag LaunchShot).
     */
    public static boolean WAIT_FOR_STEADY_FLYWHEEL = true;
    public static double STEADY_TOLERANCE_RPM = 50;
    public static long STEADY_MS = 150;
    public static long MAX_STEADY_WAIT_MS = 1000;

    /**
     * Keep the gate open this long after the last artifact is gone so it clears the flywheel. 500 since
     * 2026-11-20 (was 250): a slow last ball could stall above sensor 4, where no sensor sees it, and
     * stay inside when the gate closed.
     */
    public static long CLEAR_MS = 500;
    /** Hard cap on how long the gate stays open, in case the artifact count misses an exit. */
    public static long MAX_FIRE_MS = 1500;

    /**
     * Debug: logs the flywheel RPM, count and sensor 4 every loop of each launch (tag LaunchTrace, a few
     * lines at the end of the launch). It found the kick-reverse miscount (2026-11-20). Off: it formats
     * a string every loop.
     */
    public static boolean LOG_RPM_TRACE = false;

    /**
     * Master switch: true = plain forward feed for any artifact count (no transfer reverse, no
     * lone-ball or last-ball handling), overriding KICK_ENABLED and LAST_BALL_KICK_ENABLED.
     * false = those two flags decide. Read when the launch starts.
     */
    public static boolean SIMPLE_LAUNCH = false; // kick on (2026-11-18): reverse briefly (by count), then feed

    /** Reverse-then-forward run-up at the start of every launch. false = straight forward feed. */
    public static boolean KICK_ENABLED = true;
    /**
     * Start the kick's reverse on the press with the gate still closed (2026-11-19). The top
     * artifact rests (and spins) against the closed gate; a gate swinging open while an artifact
     * presses on it can catch in the artifact's holes and jam the launch. false = the old order
     * (gate opens and the reverse starts in the same loop).
     */
    public static boolean REVERSE_BEFORE_GATE = true;
    /** REVERSE_BEFORE_GATE: open the gate this long into the reverse (if the flywheel is steady by then). */
    public static long GATE_OPEN_DELAY_MS = 60;
    /**
     * REVERSE_BEFORE_GATE: the forward feed starts no sooner than this after the gate was commanded
     * open, so it can't push the artifact into a gate still moving. Only waits when the gate opened
     * late (flywheel not steady in time); normally the reverse is long enough. 0 = never wait.
     */
    public static long MIN_GATE_LEAD_MS = 100;
    /**
     * Reverse time (ms) by artifacts loaded at the press: index 0 = 1 artifact ... index 3 = 4.
     * More artifacts = shorter, since the bottom one is close to falling out the intake.
     * PLACEHOLDERS except the 1-artifact value (the old single-ball kick time): tune on the robot.
     */
    public static long[] KICK_REVERSE_MS_BY_COUNT = {500, 350, 275, 200};
    /** Reverse transfer power during the kick (positive number; applied as reverse). */
    public static double KICK_REVERSE_POWER = 1.0;
    /** Intake motor power during the kick (forward, holds artifacts in). */
    public static double KICK_INTAKE_POWER = 1.0;
    /** Keep the intake running this long after the transfer switches to forward. */
    public static long KICK_INTAKE_EXTRA_MS = 250;

    /** 1 artifact only: sensor 4 blocked this long while driving forward = didn't make it; kick again. */
    public static long KICK_RETRY_MS = 300;
    /** 1 artifact only: after the reverse it must come back up to sensor 4 within this long, or is kicked again. */
    public static long KICK_RETURN_TIMEOUT_MS = 1000;
    /** 1 artifact only: maximum kicks per launch (the first one included). */
    public static int MAX_KICKS = 2;
    /**
     * Lone-ball handling (sensor-4 completion with re-kicks, above) for a launch that starts with 1
     * artifact. false (default since 2026-11-18): a lone ball is launched like any other count, with
     * its own KICK_REVERSE_MS_BY_COUNT reverse, and the launch ends when the count reads 0.
     */
    public static boolean SINGLE_BALL_HANDLING_ENABLED = false;

    // LAST-BALL KICK (experimental; LAST_BALL_KICK_ENABLED = false turns it off live in Panels)
    /**
     * In a 2+ artifact launch, the last one has nothing behind it to push it through the dead zone.
     * When the count drops to 1 mid-launch, give it its own kick: reverse LAST_BALL_REVERSE_MS
     * (intake running), then forward, with the same sensor-4 completion and retries as a lone ball.
     */
    public static boolean LAST_BALL_KICK_ENABLED = false; // off since 2026-11-18: the per-count reverse is enough
    /** Reverse time for the last artifact's kick. Longer than a full-stack reverse: it has room. */
    public static long LAST_BALL_REVERSE_MS = 500;
    /** Wait this long after the count drops to 1 before reversing (lets the previous one clear the flywheel). */
    public static long LAST_BALL_KICK_DELAY_MS = 0;

    private enum State { WAIT_STEADY, FIRING, DONE }

    /** Why the last launch ended (read by test OpModes; doesn't change behavior). */
    public enum EndReason { RUNNING, NOTHING_LOADED, EMPTY, MAX_FIRE_MS, SINGLE_BALL_CLEARED, INTERRUPTED, JAMMED }

    private EndReason endReason = EndReason.RUNNING;
    private boolean emptyConfirmedAtTimeout = false;

    private final LaunchGateSubsystem gate;
    private final IntakeSubsystem intake;
    private final DriveSubsystem drive;
    private final IntakeBarSubsystem bar;
    /** null = no steady-flywheel wait and no speed logging. */
    private final LaunchSubsystem launcher;
    private long waitStartMs;
    private long steadySinceMs;
    private long gateOpenMs;
    private int lastCount;
    private double minRpm;
    private final StringBuilder rpmTrace = new StringBuilder();
    private final StringBuilder shotLog = new StringBuilder();

    private State state;
    private long stateStartMs;
    private long emptySinceMs;

    private int countAtStart;
    /** Kick logic for this launch (latched at the press so a Panels toggle mid-launch can't half-apply). */
    private boolean kickActive;
    private boolean lastBallKickActive;
    private boolean singleBall;
    /** LAST-BALL KICK: when the count first hit 1 mid-launch, or -1. */
    private long lastBallSinceMs;
    /** LAST-BALL KICK: the kick has been started (only once per launch). */
    private boolean lastBallKickStarted;

    // Kick
    private boolean reversing;
    private long reverseMs;
    private long kickPhaseStartMs;   // start of the reverse, then start of the forward drive
    private int kicks;
    /** REVERSE_BEFORE_GATE for this launch (latched at the press). */
    private boolean reverseFirst;
    /** The first forward feed after the reverse has been logged. */
    private boolean feedLogged;

    // Single-artifact completion
    private long sensor4BlockedSinceMs;
    private long sensor4ClearSinceMs;
    /** The ball has been back at sensor 4 since the last kick's reverse (so a clear sensor means it left). */
    private boolean ballReturned;

    public LaunchBallsCommand(LaunchGateSubsystem gate, IntakeSubsystem intake, DriveSubsystem drive,
                              IntakeBarSubsystem bar) {
        this(gate, intake, drive, bar, null);
    }

    /** With the launcher: waits for a steady flywheel before opening the gate and logs shot speeds. */
    public LaunchBallsCommand(LaunchGateSubsystem gate, IntakeSubsystem intake, DriveSubsystem drive,
                              IntakeBarSubsystem bar, LaunchSubsystem launcher) {
        this.gate = gate;
        this.intake = intake;
        this.drive = drive;
        this.bar = bar;
        this.launcher = launcher;
        addRequirements(gate);
    }

    @Override
    public void initialize() {
        endReason = EndReason.RUNNING;
        emptyConfirmedAtTimeout = false;
        emptySinceMs = -1;
        countAtStart = intake.getArtifactCount();
        intake.resetTransferCurrentPeak();
        if (intake.isJammed()) {
            Log.w("LaunchBallsCommand", "Launch refused: transfer jammed (" + intake.getJamReason() + "), hold reverse (BACK) to clear");
            endReason = EndReason.JAMMED;
            enter(State.DONE);
            return;
        }
        if (countAtStart == 0) {
            Log.i("LaunchBallsCommand", "No artifacts loaded, nothing to launch");
            endReason = EndReason.NOTHING_LOADED;
            enter(State.DONE);
            return;
        }
        kickActive = !SIMPLE_LAUNCH && KICK_ENABLED;
        lastBallKickActive = kickActive && LAST_BALL_KICK_ENABLED;
        // One ball: the count says so. (Sensor 3 used to confirm it, but sensors 2/3 are ignored for now.)
        singleBall = kickActive && SINGLE_BALL_HANDLING_ENABLED && countAtStart == 1;
        kicks = 0;
        reversing = false;
        sensor4BlockedSinceMs = -1;
        sensor4ClearSinceMs = -1;
        ballReturned = true; // without a kick the ball never goes below sensor 4
        lastBallSinceMs = -1;
        lastBallKickStarted = false;
        reverseFirst = kickActive && REVERSE_BEFORE_GATE;
        feedLogged = false;
        gateOpenMs = 0;

        drive.requestLaunchHold();

        if (!bar.isRaised()) bar.raise(); // command the bar up; the launch doesn't wait for it

        waitStartMs = nowMs();
        // A flywheel already steady before the press counts from when it got steady (no extra STEADY_MS).
        steadySinceMs = launcher != null ? launcher.getFlywheelSteadySinceMs() : -1;
        if (reverseFirst) {
            startKick(waitStartMs, singleBall ? 1 : countAtStart); // reverse now, gate still closed
            enter(State.WAIT_STEADY); // gate opens after GATE_OPEN_DELAY_MS and a steady flywheel
        } else if (WAIT_FOR_STEADY_FLYWHEEL && launcher != null) {
            enter(State.WAIT_STEADY); // gate stays closed until the flywheel holds its speed
        } else {
            openGateAndFire(nowMs(), "no wait");
        }
    }

    /** WAIT_STEADY: why the gate may open now, or null to keep it closed. */
    private String gateOpenReason(long now) {
        boolean waitForFlywheel = WAIT_FOR_STEADY_FLYWHEEL && launcher != null;
        if (waitForFlywheel) {
            if (flywheelSteady()) {
                if (steadySinceMs < 0) steadySinceMs = now;
            } else {
                steadySinceMs = -1;
            }
        }
        if (reverseFirst && now - waitStartMs < GATE_OPEN_DELAY_MS) return null;
        if (!waitForFlywheel) return "no wait";
        if (steadySinceMs >= 0 && now - steadySinceMs >= STEADY_MS) return "steady";
        if (now - waitStartMs >= MAX_STEADY_WAIT_MS) {
            Log.w("LaunchBallsCommand", "Flywheel not steady after " + MAX_STEADY_WAIT_MS + " ms, firing anyway");
            return "NOT steady, gave up waiting";
        }
        return null;
    }

    /** Opens the gate and starts the feed (and the kick). */
    private void openGateAndFire(long now, String why) {
        gate.openGate();
        enter(State.FIRING);
        gateOpenMs = now;
        lastCount = intake.getArtifactCount();
        shotLog.setLength(0);
        rpmTrace.setLength(0);
        String speed = "";
        if (launcher != null) {
            minRpm = launcher.getCurrentFlywheelRPM();
            speed = String.format(Locale.US, ", target %.0f, L/R %.0f/%.0f RPM, %.2f V, waited %d ms (%s)",
                    launcher.getTargetRPM(), launcher.getLeftFlywheelRPM(), launcher.getRightFlywheelRPM(),
                    launcher.getBatteryVoltage(), now - waitStartMs, why);
        }
        Log.i("LaunchBallsCommand", "Launch: " + countAtStart + " artifact(s), "
                + (kickActive ? (reverseFirst ? "kick, reverse before gate" : "kick") : "simple feed") + speed);
        if (launcher != null) Log.i("LaunchShot", "gate open" + speed);
        if (kickActive && !reverseFirst) startKick(now, singleBall ? 1 : countAtStart);
    }

    /** Both motors within STEADY_TOLERANCE_RPM of the target and not in bang-bang. */
    private boolean flywheelSteady() {
        double target = launcher.getTargetRPM();
        return target > 0 && !launcher.isFlywheelInBangBang()
                && Math.abs(launcher.getLeftFlywheelRPM() - target) <= STEADY_TOLERANCE_RPM
                && Math.abs(launcher.getRightFlywheelRPM() - target) <= STEADY_TOLERANCE_RPM;
    }

    @Override
    public void execute() {
        long now = nowMs();
        if (state == State.DONE) return;
        if (intake.isJammed()) { // IntakeSubsystem stopped the transfer and intake
            Log.w("LaunchBallsCommand", "Launch stopped: transfer jammed (" + intake.getJamReason() + ")");
            endReason = EndReason.JAMMED;
            enter(State.DONE);
            return;
        }

        // Keep the bar up for the whole launch (undoes an operator toggle mid-launch).
        if (!bar.isRaised()) bar.raise();

        if (state == State.WAIT_STEADY) {
            if (reverseFirst) updateKick(now); // reverse running; holds the transfer if it ends first
            String why = gateOpenReason(now);
            if (why != null) openGateAndFire(now, why);
            return;
        }
        logShots(now);

        if (!updateKick(now)) {
            // Still reversing: no completion checks yet, only the backstop below.
        } else if (singleBall) {
            executeSingleBall(now);
        } else if (startLastBallKickIfNeeded(now)) {
            // LAST-BALL KICK: now handled as a lone ball (singleBall = true) from the next loop.
        } else if (intake.getArtifactCount() == 0) {
            if (emptySinceMs < 0) emptySinceMs = now;
            // Also sensor 4 clear: a ball there with the count at 0 didn't reach the flywheel
            // (IntakeSubsystem raises the count back to 1 if it stays, LAUNCH_STUCK_BALL_MS).
            if (intake.isSensor4Blocked()) emptySinceMs = now;
            if (now - emptySinceMs >= CLEAR_MS) {
                endReason = EndReason.EMPTY;
                enter(State.DONE);
            }
        } else {
            emptySinceMs = -1;
        }

        if (state == State.FIRING && now - stateStartMs >= MAX_FIRE_MS) {
            Log.w("LaunchBallsCommand", "Hit MAX_FIRE_MS with count " + intake.getArtifactCount());
            // count too high? sensors clear while feeding -> 0
            emptyConfirmedAtTimeout = intake.confirmEmptyAfterLaunch();
            endReason = EndReason.MAX_FIRE_MS;
            enter(State.DONE);
        }
    }

    /**
     * Advances the kick: ends the reverse after reverseMs (transfer back to full forward) and stops
     * the intake KICK_INTAKE_EXTRA_MS later. Returns false while still reversing.
     */
    private boolean updateKick(long now) {
        if (reversing) {
            if (now - kickPhaseStartMs < reverseMs) return false;
            if (reverseFirst && (state != State.FIRING || now - gateOpenMs < MIN_GATE_LEAD_MS)) {
                intake.setLaunchTransferOverride(0); // reverse done, gate not (long enough) open: hold
                return false;
            }
            if (reverseFirst && !feedLogged) {
                feedLogged = true;
                Log.i("LaunchBallsCommand", String.format(Locale.US, "Feed forward at +%d ms after the press, gate opened at +%d ms",
                        now - waitStartMs, gateOpenMs - waitStartMs));
            }
            reversing = false;
            kickPhaseStartMs = now;
            intake.clearLaunchTransferOverride(); // back to full forward launch power
        }
        if (intake.hasLaunchIntakeOverride() && now - kickPhaseStartMs >= KICK_INTAKE_EXTRA_MS) {
            intake.clearLaunchIntakeOverride(); // intake off for the rest of the launch
        }
        return true;
    }

    /**
     * LAST-BALL KICK: in a 2+ artifact launch, once the count has been 1 for LAST_BALL_KICK_DELAY_MS,
     * switch to lone-ball handling and kick it. Returns true if the kick started this loop.
     */
    private boolean startLastBallKickIfNeeded(long now) {
        if (!lastBallKickActive || lastBallKickStarted || countAtStart < 2) return false;
        if (intake.getArtifactCount() != 1) {
            lastBallSinceMs = -1;
            return false;
        }
        if (lastBallSinceMs < 0) lastBallSinceMs = now;
        if (now - lastBallSinceMs < LAST_BALL_KICK_DELAY_MS) return false;

        lastBallKickStarted = true;
        singleBall = true;
        kicks = 0;              // the last ball gets its own MAX_KICKS
        enter(State.FIRING);    // and its own MAX_FIRE_MS backstop
        Log.i("LaunchBallsCommand", "Last ball: extra kick");
        startKick(now, 1, LAST_BALL_REVERSE_MS);
        return true;
    }

    /** Lone ball: sensor-based completion with retries (see class comment). */
    private void executeSingleBall(long now) {
        boolean s4Blocked = intake.isSensor4Blocked();

        // After a reverse the ball is BELOW sensor 4, so a clear sensor doesn't mean it launched.
        // Wait for it to come back up past the sensor before the clear timer can count.
        if (!ballReturned) {
            if (s4Blocked) {
                ballReturned = true;
            } else {
                if (now - kickPhaseStartMs >= KICK_RETURN_TIMEOUT_MS && kicks < MAX_KICKS) {
                    Log.i("LaunchBallsCommand", "Ball didn't come back up, kick " + (kicks + 1) + "/" + MAX_KICKS);
                    startKick(now, 1, lastBallKickStarted ? LAST_BALL_REVERSE_MS : reverseMsFor(1)); // LAST-BALL KICK retries use its reverse time
                }
                return;
            }
        }

        if (s4Blocked) {
            sensor4ClearSinceMs = -1;
            if (sensor4BlockedSinceMs < 0) sensor4BlockedSinceMs = now;
            if (now - sensor4BlockedSinceMs >= KICK_RETRY_MS) {
                if (kicks < MAX_KICKS) {
                    Log.i("LaunchBallsCommand", "Ball didn't clear, kick " + (kicks + 1) + "/" + MAX_KICKS);
                    startKick(now, 1, lastBallKickStarted ? LAST_BALL_REVERSE_MS : reverseMsFor(1)); // LAST-BALL KICK retries use its reverse time
                } // else: out of kicks, keep pushing forward until MAX_FIRE_MS
            }
        } else {
            sensor4BlockedSinceMs = -1;
            if (sensor4ClearSinceMs < 0) sensor4ClearSinceMs = now;
            if (now - sensor4ClearSinceMs >= CLEAR_MS) {
                endReason = EndReason.SINGLE_BALL_CLEARED;
                enter(State.DONE);
            }
        }
    }

    /** Starts a reverse-then-forward run-up sized for {@code artifacts} loaded; intake runs to hold them in. */
    private void startKick(long now, int artifacts) {
        startKick(now, artifacts, reverseMsFor(artifacts));
    }

    private void startKick(long now, int artifacts, long reverseTimeMs) {
        kicks++;
        reversing = true;
        reverseMs = Math.max(0, reverseTimeMs);
        kickPhaseStartMs = now;
        sensor4BlockedSinceMs = -1;
        sensor4ClearSinceMs = -1;
        ballReturned = false;
        intake.setLaunchTransferOverride(-Math.abs(KICK_REVERSE_POWER));
        intake.setLaunchIntakeOverride(Math.abs(KICK_INTAKE_POWER));
        Log.i("LaunchBallsCommand", "Kick " + kicks + ": " + artifacts + " artifact(s), reverse " + reverseMs + " ms");
    }

    private static long reverseMsFor(int artifacts) {
        long[] table = KICK_REVERSE_MS_BY_COUNT;
        if (table.length == 0) return 0;
        int i = Math.max(0, Math.min(artifacts - 1, table.length - 1));
        return Math.max(0, table[i]);
    }

    /** Per-exit flywheel speed: each count drop while firing is one artifact past sensor 4. */
    private void logShots(long now) {
        if (launcher == null || state != State.FIRING) return;
        double rpm = launcher.getCurrentFlywheelRPM();
        minRpm = Math.min(minRpm, rpm);
        if (LOG_RPM_TRACE) {
            rpmTrace.append(String.format(Locale.US, "%s%d:%.0f/%.0f/%d%s", rpmTrace.length() > 0 ? " " : "", now - gateOpenMs,
                    launcher.getLeftFlywheelRPM(), launcher.getRightFlywheelRPM(), intake.getArtifactCount(),
                    intake.isSensor4Blocked() ? "X" : "-"));
        }
        int count = intake.getArtifactCount();
        if (count < lastCount) {
            shotLog.append(String.format(Locale.US, "%s+%d ms: %.0f/%.0f", shotLog.length() > 0 ? ", " : "",
                    now - gateOpenMs, launcher.getLeftFlywheelRPM(), launcher.getRightFlywheelRPM()));
        }
        lastCount = count;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted && endReason == EndReason.RUNNING) endReason = EndReason.INTERRUPTED;
        if (launcher != null && gateOpenMs > 0 && state != State.WAIT_STEADY) {
            Log.i("LaunchShot", String.format(Locale.US, "done (%s): target %.0f, exits at [%s] L/R RPM, burst min %.0f RPM, %.2f V, transfer peak %.1f A",
                    endReason.name().toLowerCase(Locale.US), launcher.getTargetRPM(), shotLog, minRpm, launcher.getBatteryVoltage(),
                    intake.getTransferCurrentPeakA()));
            if (LOG_RPM_TRACE && rpmTrace.length() > 0) {
                // ms after gate open : left/right RPM / count / sensor 4 (X blocked), every loop.
                Log.i("LaunchTrace", "start " + countAtStart + " artifact(s), target " + Math.round(launcher.getTargetRPM())
                        + " | format: ms:L/R/count/sensor4");
                for (int i = 0; i < rpmTrace.length(); i += 1500) {
                    Log.i("LaunchTrace", rpmTrace.substring(i, Math.min(rpmTrace.length(), i + 1500)));
                }
            }
        }
        intake.clearLaunchTransferOverride();
        intake.clearLaunchIntakeOverride();
        gate.closeGate();
        drive.releaseLaunchHold();
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }

    public EndReason getEndReason() {
        return endReason;
    }

    /** At MAX_FIRE_MS: confirmEmptyAfterLaunch() reset the count to 0. */
    public boolean wasEmptyConfirmedAtTimeout() {
        return emptyConfirmedAtTimeout;
    }

    /** Gate still closed: waiting for a steady flywheel (and, reversing first, GATE_OPEN_DELAY_MS). */
    public boolean isWaitingForFlywheel() {
        return state == State.WAIT_STEADY;
    }

    private void enter(State next) {
        state = next;
        stateStartMs = nowMs();
    }

    private static long nowMs() {
        return System.nanoTime() / 1_000_000L;
    }
}
