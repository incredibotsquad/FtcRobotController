package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

/**
 * Default command for the intake. Runs forever and picks a state every loop:
 *
 *  LAUNCHING (gate open): only the transfer runs, feeding artifacts up past sensor 4 into
 *      the launcher. The intake motor stays off so nothing new is pulled in mid-shot.
 *      IntakeSubsystem is told it is launching, so artifacts leaving the top are subtracted.
 *  INTAKING (gate closed, not full): intake and transfer run.
 *  FULL (gate closed, count at MAX_ARTIFACTS): everything stops (G407: control at most 4).
 *
 * The artifact count itself comes from IntakeSubsystem.periodic(), which the scheduler runs
 * before this command each loop. Other commands that require the intake (e.g.
 * ReverseIntakeCommand) interrupt this one; the scheduler restarts it when they finish.
 */
@Configurable
public class SmartIntakeCommand extends CommandBase {

    /** Transfer power while feeding the launcher. Lower it if shots are too close together. */
    public static double LAUNCH_TRANSFER_POWER = 1.0;

    public enum State { LAUNCHING, INTAKING, FULL }

    private final IntakeSubsystem intake;
    private final LaunchGateSubsystem gate;
    // Kept for future position-based behavior (e.g. only intake outside the launch zone).
    @SuppressWarnings("unused")
    private final OdometrySubsystem odometry;

    private State state;

    public SmartIntakeCommand(IntakeSubsystem intake, LaunchGateSubsystem gate, OdometrySubsystem odometry) {
        this.intake = intake;
        this.gate = gate;
        this.odometry = odometry;
        // Only the intake is required: the gate is read, never commanded, so launch commands
        // that require the gate are not interrupted by this one.
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        state = null; // force the first execute() to apply outputs
    }

    @Override
    public void execute() {
        State next;
        if (gate.isGateOpen()) {
            next = State.LAUNCHING;
        } else if (intake.isFull()) {
            next = State.FULL;
        } else {
            next = State.INTAKING;
        }

        intake.setLaunching(next == State.LAUNCHING);

        switch (next) {
            case LAUNCHING:
                intake.stopIntakeMotorOnly();
                // A launch command may override the transfer (e.g. LaunchBallsCommand's single-ball kick).
                intake.setTransferMotorPower(intake.hasLaunchTransferOverride()
                        ? intake.getLaunchTransferOverride()
                        : LAUNCH_TRANSFER_POWER);
                break;
            case INTAKING:
                intake.startIntakeAndTransfer();
                break;
            case FULL:
                // Only write on the transition so the motors aren't commanded every loop
                if (state != State.FULL) intake.stopIntakeAndTransfer();
                break;
        }

        state = next;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setLaunching(false);
        intake.stopIntakeAndTransfer();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public State getState() {
        return state;
    }
}
