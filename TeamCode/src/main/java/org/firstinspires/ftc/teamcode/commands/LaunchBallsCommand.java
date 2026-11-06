package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;

/**
 * One-press "empty the robot" shot. Schedule with whenPressed().
 *
 * Presumes the launcher is already ready: LaunchReadinessCommand keeps the flywheel at speed and
 * the turret on target, so there is no readiness wait. The gate opens immediately on the press.
 *
 * Kick (every launch, when KICK_ENABLED): an artifact resting against the closed gate starts from
 * rest, and the dead zone between the top of the transfer and the flywheel (above sensor 4) can
 * stop it. So every launch starts with a run-up:
 *  1. Gate opens and the transfer runs in REVERSE for a time that depends on how many artifacts
 *     are loaded (KICK_REVERSE_MS_BY_COUNT): a full stack can only back up a little before the
 *     bottom artifact is pushed out of the intake.
 *  2. Transfer runs forward at full launch power, so the top artifact reaches the dead zone moving.
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
 * Launch hold: for the whole command it asks DriveSubsystem to hold the
 * robot's pose against defense. DriveRobotCommand does the holding and lets the driver break out
 * with the sticks; a breakout does NOT stop the launch. Turned off by DriveSubsystem.LAUNCH_HOLD_ENABLED.
 * The drive is not a requirement, so DriveRobotCommand keeps running.
 */
@Configurable
public class LaunchBallsCommand extends CommandBase {

    /** Keep the gate open this long after the last artifact is gone so it clears the flywheel. */
    public static long CLEAR_MS = 250;
    /** Hard cap on how long the gate stays open, in case the artifact count misses an exit. */
    public static long MAX_FIRE_MS = 3000;

    /** Reverse-then-forward run-up at the start of every launch. false = straight forward feed. */
    public static boolean KICK_ENABLED = true;
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

    // LAST-BALL KICK (experimental; LAST_BALL_KICK_ENABLED = false turns it off live in Panels)
    /**
     * In a 2+ artifact launch, the last one has nothing behind it to push it through the dead zone.
     * When the count drops to 1 mid-launch, give it its own kick: reverse LAST_BALL_REVERSE_MS
     * (intake running), then forward, with the same sensor-4 completion and retries as a lone ball.
     */
    public static boolean LAST_BALL_KICK_ENABLED = true;
    /** Reverse time for the last artifact's kick. Longer than a full-stack reverse: it has room. */
    public static long LAST_BALL_REVERSE_MS = 500;
    /** Wait this long after the count drops to 1 before reversing (lets the previous one clear the flywheel). */
    public static long LAST_BALL_KICK_DELAY_MS = 0;

    private enum State { FIRING, DONE }

    private final LaunchGateSubsystem gate;
    private final IntakeSubsystem intake;
    private final DriveSubsystem drive;

    private State state;
    private long stateStartMs;
    private long emptySinceMs;

    private int countAtStart;
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

    // Single-artifact completion
    private long sensor4BlockedSinceMs;
    private long sensor4ClearSinceMs;
    /** The ball has been back at sensor 4 since the last kick's reverse (so a clear sensor means it left). */
    private boolean ballReturned;

    public LaunchBallsCommand(LaunchGateSubsystem gate, IntakeSubsystem intake, DriveSubsystem drive) {
        this.gate = gate;
        this.intake = intake;
        this.drive = drive;
        addRequirements(gate);
    }

    @Override
    public void initialize() {
        emptySinceMs = -1;
        countAtStart = intake.getArtifactCount();
        if (countAtStart == 0) {
            Log.i("LaunchBallsCommand", "No artifacts loaded, nothing to launch");
            enter(State.DONE);
            return;
        }
        // One ball: count says so, or the stack shows only the top slot filled (fills top-down).
        singleBall = KICK_ENABLED
                && (countAtStart == 1 || (intake.isSensor4Blocked() && !intake.isSensor3Blocked()));
        kicks = 0;
        reversing = false;
        sensor4BlockedSinceMs = -1;
        sensor4ClearSinceMs = -1;
        ballReturned = true; // without a kick the ball never goes below sensor 4
        lastBallSinceMs = -1;
        lastBallKickStarted = false;

        drive.requestLaunchHold();
        gate.openGate();
        enter(State.FIRING);
        if (KICK_ENABLED) startKick(nowMs(), singleBall ? 1 : countAtStart);
    }

    @Override
    public void execute() {
        long now = nowMs();
        if (state != State.FIRING) return;

        if (!updateKick(now)) {
            // Still reversing: no completion checks yet, only the backstop below.
        } else if (singleBall) {
            executeSingleBall(now);
        } else if (startLastBallKickIfNeeded(now)) {
            // LAST-BALL KICK: now handled as a lone ball (singleBall = true) from the next loop.
        } else if (intake.getArtifactCount() == 0) {
            if (emptySinceMs < 0) emptySinceMs = now;
            if (now - emptySinceMs >= CLEAR_MS) enter(State.DONE);
        } else {
            emptySinceMs = -1;
        }

        if (state == State.FIRING && now - stateStartMs >= MAX_FIRE_MS) {
            Log.w("LaunchBallsCommand", "Hit MAX_FIRE_MS with count " + intake.getArtifactCount());
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
        if (!LAST_BALL_KICK_ENABLED || !KICK_ENABLED || lastBallKickStarted || countAtStart < 2) return false;
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
            if (now - sensor4ClearSinceMs >= CLEAR_MS) enter(State.DONE);
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

    @Override
    public void end(boolean interrupted) {
        intake.clearLaunchTransferOverride();
        intake.clearLaunchIntakeOverride();
        gate.closeGate();
        drive.releaseLaunchHold();
    }

    @Override
    public boolean isFinished() {
        return state == State.DONE;
    }

    private void enter(State next) {
        state = next;
        stateStartMs = nowMs();
    }

    private static long nowMs() {
        return System.nanoTime() / 1_000_000L;
    }
}
