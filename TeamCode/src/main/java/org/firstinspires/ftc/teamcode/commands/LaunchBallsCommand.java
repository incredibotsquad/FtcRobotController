package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;

/**
 * One-press "empty the robot" shot. Schedule with whenPressed().
 *
 *  WAITING: gate closed until LaunchSubsystem.isReadyToLaunch() (flywheel at speed, turret
 *      on target, valid shot solution). Gives up after READY_TIMEOUT_MS so a bad press
 *      doesn't leave a command hanging.
 *  FIRING:  gate open. SmartIntakeCommand sees the open gate and runs the transfer, feeding
 *      every artifact into the flywheel. Ends once IntakeSubsystem counts 0 artifacts
 *      (plus CLEAR_MS so the last one gets through), or after MAX_FIRE_MS as a backstop.
 *
 * Requires only the gate. LaunchReadinessCommand (launcher) keeps aiming and spinning the
 * flywheel during the burst, and SmartIntakeCommand (intake) keeps running the transfer.
 * The intake is only read. When this ends, CloseGateCommand (gate default) takes over.
 */
@Configurable
public class LaunchBallsCommand extends CommandBase {

    /** Give up if the launcher isn't ready within this long after the press. */
    public static long READY_TIMEOUT_MS = 500;
    /** Keep the gate open this long after the count reaches 0 so the last artifact clears the flywheel. */
    public static long CLEAR_MS = 250;
    /** Hard cap on how long the gate stays open, in case the artifact count misses an exit. */
    public static long MAX_FIRE_MS = 2500;

    private enum State { WAITING, FIRING, DONE }

    private final LaunchSubsystem launcher;
    private final LaunchGateSubsystem gate;
    private final IntakeSubsystem intake;

    private State state;
    private long stateStartMs;
    private long emptySinceMs;

    public LaunchBallsCommand(LaunchSubsystem launcher, LaunchGateSubsystem gate, IntakeSubsystem intake) {
        this.launcher = launcher;
        this.gate = gate;
        this.intake = intake;
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
        gate.closeGate();
        enter(State.WAITING);
    }

    @Override
    public void execute() {
        long now = nowMs();
        switch (state) {
            case WAITING:
                if (launcher.isReadyToLaunch()) {
                    gate.openGate();
                    enter(State.FIRING);
                } else if (now - stateStartMs >= READY_TIMEOUT_MS) {
                    Log.w("LaunchBallsCommand", "Launcher not ready after " + READY_TIMEOUT_MS + " ms, cancelling");
                    enter(State.DONE);
                }
                break;

            case FIRING:
                if (intake.getArtifactCount() == 0) {
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

    @Override
    public void end(boolean interrupted) {
        gate.closeGate();
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
