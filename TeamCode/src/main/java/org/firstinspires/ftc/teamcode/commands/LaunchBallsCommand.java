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
    public static long MAX_FIRE_MS = 2500;

    private enum State { FIRING, DONE }

    private final LaunchGateSubsystem gate;
    private final IntakeSubsystem intake;
    private final DriveSubsystem drive;

    private State state;
    private long stateStartMs;
    private long emptySinceMs;

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
        drive.requestLaunchHold();
        gate.openGate();
        enter(State.FIRING);
    }

    @Override
    public void execute() {
        long now = nowMs();
        switch (state) {
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
