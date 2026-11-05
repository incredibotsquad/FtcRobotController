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
 *  FIRING: gate open. SmartIntakeCommand sees the open gate and runs the transfer, feeding
 *      every artifact into the flywheel. Ends once IntakeSubsystem counts 0 artifacts
 *      (plus CLEAR_MS so the last one gets through), or after MAX_FIRE_MS as a backstop.
 *
 * Single-ball kick: a lone ball resting against the closed gate starts from rest, and the dead zone
 * between the top of the transfer and the flywheel (above sensor 4) stops it. With 2+ balls the
 * one behind pushes it through. So when exactly one ball is loaded:
 *  1. Gate opens and the transfer runs in REVERSE for KICK_REVERSE_MS, pulling the ball down.
 *  2. Transfer runs forward at full launch power: the ball reaches the dead zone moving.
 *  3. The ball must come back up to sensor 4 (within KICK_RETURN_TIMEOUT_MS, else kick again).
 *     If sensor 4 then stays blocked for KICK_RETRY_MS (stuck, or fell back out of the dead zone),
 *     kick again, up to MAX_KICKS times.
 * It's done when sensor 4 has stayed clear for CLEAR_MS. The count alone can't be trusted here: it
 * drops when the ball passes sensor 4, even if the ball then stalls above it and falls back.
 * The transfer is driven through IntakeSubsystem's launch override, which SmartIntakeCommand applies.
 *
 * Requires only the gate. LaunchReadinessCommand (launcher) keeps aiming and spinning the
 * flywheel during the burst, and SmartIntakeCommand (intake) keeps running the transfer.
 * The intake is only read. When this ends, CloseGateCommand (gate default) takes over.
 *
 * Launch hold: for the whole command it asks DriveSubsystem to hold the
 * robot's pose against defense. DriveRobotCommand does the holding and lets the driver break out
 * with the sticks; a breakout does NOT stop the launch. Turned off by DriveSubsystem.LAUNCH_HOLD_ENABLED.
 * The drive is not a requirement, so DriveRobotCommand keeps running.
 */
@Configurable
public class LaunchBallsCommand extends CommandBase {

    /** Keep the gate open this long after the count reaches 0 so the last artifact clears the flywheel. */
    public static long CLEAR_MS = 250;
    /** Hard cap on how long the gate stays open, in case the artifact count misses an exit. */
    public static long MAX_FIRE_MS = 3000;

    /** Single-ball kick (see class comment). false = treat one ball like any other launch. */
    public static boolean SINGLE_BALL_KICK_ENABLED = true;
    /** How long to reverse the transfer before driving forward. Longer = longer run-up. */
    public static long KICK_REVERSE_MS = 500;
    /** Reverse transfer power during the kick (positive number; applied as reverse). */
    public static double KICK_REVERSE_POWER = 1.0;
    /** Sensor 4 blocked this long while driving forward = ball didn't make it; kick again. */
    public static long KICK_RETRY_MS = 300;
    /** After the reverse, the ball must come back up to sensor 4 within this long, or it's kicked again. */
    public static long KICK_RETURN_TIMEOUT_MS = 1000;
    /** Maximum kicks per launch (the first one included). */
    public static int MAX_KICKS = 2;

    private enum State { FIRING, DONE }

    private final LaunchGateSubsystem gate;
    private final IntakeSubsystem intake;
    private final DriveSubsystem drive;

    private State state;
    private long stateStartMs;
    private long emptySinceMs;

    // Single-ball kick
    private boolean singleBall;
    private boolean reversing;
    private long kickPhaseStartMs;
    private int kicks;
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
        if (intake.getArtifactCount() == 0) {
            Log.i("LaunchBallsCommand", "No artifacts loaded, nothing to launch");
            enter(State.DONE);
            return;
        }
        // One ball: count says so, or the stack shows only the top slot filled (fills top-down).
        singleBall = SINGLE_BALL_KICK_ENABLED
                && (intake.getArtifactCount() == 1 || (intake.isSensor4Blocked() && !intake.isSensor3Blocked()));
        kicks = 0;
        sensor4BlockedSinceMs = -1;
        sensor4ClearSinceMs = -1;

        drive.requestLaunchHold();
        gate.openGate();
        enter(State.FIRING);
        if (singleBall) startKick(nowMs());
    }

    @Override
    public void execute() {
        long now = nowMs();
        switch (state) {
            case FIRING:
                if (singleBall) {
                    executeSingleBall(now);
                } else if (intake.getArtifactCount() == 0) {
                    if (emptySinceMs < 0) emptySinceMs = now;
                    if (now - emptySinceMs >= CLEAR_MS) enter(State.DONE);
                } else {
                    emptySinceMs = -1;
                }
                if (now - stateStartMs >= MAX_FIRE_MS) {
                    Log.w("LaunchBallsCommand", "Hit MAX_FIRE_MS with count " + intake.getArtifactCount());
                    enter(State.DONE);
                }
                break;

            case DONE:
                break;
        }
    }

    /** Reverse-then-forward kick for a lone ball, with retries. */
    private void executeSingleBall(long now) {
        if (reversing) {
            if (now - kickPhaseStartMs >= KICK_REVERSE_MS) {
                reversing = false;
                kickPhaseStartMs = now;
                intake.clearLaunchTransferOverride(); // back to full forward launch power
            }
            return;
        }

        boolean s4Blocked = intake.isSensor4Blocked();

        // After a reverse the ball is BELOW sensor 4, so a clear sensor doesn't mean it launched.
        // Wait for it to come back up past the sensor before the clear timer can count.
        if (!ballReturned) {
            if (s4Blocked) {
                ballReturned = true;
            } else {
                if (now - kickPhaseStartMs >= KICK_RETURN_TIMEOUT_MS && kicks < MAX_KICKS) {
                    Log.i("LaunchBallsCommand", "Ball didn't come back up, kick " + (kicks + 1) + "/" + MAX_KICKS);
                    startKick(now);
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
                    startKick(now);
                } // else: out of kicks, keep pushing forward until MAX_FIRE_MS
            }
        } else {
            sensor4BlockedSinceMs = -1;
            if (sensor4ClearSinceMs < 0) sensor4ClearSinceMs = now;
            if (now - sensor4ClearSinceMs >= CLEAR_MS) enter(State.DONE);
        }
    }

    private void startKick(long now) {
        kicks++;
        reversing = true;
        kickPhaseStartMs = now;
        sensor4BlockedSinceMs = -1;
        sensor4ClearSinceMs = -1;
        ballReturned = false;
        intake.setLaunchTransferOverride(-Math.abs(KICK_REVERSE_POWER));
    }

    @Override
    public void end(boolean interrupted) {
        intake.clearLaunchTransferOverride();
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
