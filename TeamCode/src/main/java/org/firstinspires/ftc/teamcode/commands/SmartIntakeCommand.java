package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

/**
 * Default command for the intake. Runs forever and picks a state every loop:
 *
 *  LAUNCHING (gate open): the transfer feeds artifacts up past sensor 4 into the launcher.
 *      The intake motor stays off so nothing new is pulled in mid-shot, except while a launch
 *      command overrides it (LaunchBallsCommand's kick).
 *      IntakeSubsystem is told it is launching, so artifacts leaving the top are subtracted.
 *  INTAKING (gate closed, not full): intake and transfer run.
 *  FULL (gate closed, count at MAX_ARTIFACTS): the intake stops (G407: control at most 4); the
 *      transfer stops IntakeSubsystem.TRANSFER_FULL_DELAY_MS later.
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
    /** FULL: the transfer has been stopped (after its run-on). */
    private boolean transferStopped;

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
        } else if (intake.isFullAndDoneIntaking()) { // full, after the run-on that pulls the last one in
            next = State.FULL;
        } else {
            next = State.INTAKING;
        }

        intake.setLaunching(next == State.LAUNCHING);

        switch (next) {
            case LAUNCHING:
                // Intake off unless a launch command asks for it (LaunchBallsCommand's kick runs it
                // so artifacts pushed down by the reversed transfer aren't lost out the front).
                if (intake.hasLaunchIntakeOverride()) {
                    intake.setIntakeMotorPower(intake.getLaunchIntakeOverride());
                } else {
                    intake.stopIntakeMotorOnly();
                }
                // A launch command may override the transfer (e.g. LaunchBallsCommand's kick).
                intake.setTransferMotorPower(intake.hasLaunchTransferOverride()
                        ? intake.getLaunchTransferOverride()
                        : LAUNCH_TRANSFER_POWER);
                break;
            case INTAKING:
                intake.startIntakeAndTransfer();
                break;
            case FULL:
                // Only write on transitions so the motors aren't commanded every loop. The intake
                // stops at once; the transfer runs on until IntakeSubsystem.TRANSFER_FULL_DELAY_MS is up.
                if (state != State.FULL) {
                    intake.stopIntakeMotorOnly();
                    transferStopped = false;
                }
                if (!transferStopped && intake.isFullAndTransferDone()) {
                    intake.stopTransferMotorOnly();
                    transferStopped = true;
                }
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
