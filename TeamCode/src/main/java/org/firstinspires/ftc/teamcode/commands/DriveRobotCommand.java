package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * Default drive command: robot-centric driver control through Pedro, plus the launch hold.
 *
 * While DriveSubsystem.shouldHoldForLaunch() is true (a launch is running and the hold is enabled):
 *  - sticks centered  -> hold the current pose (engages the first loop the sticks are centered)
 *  - stick moved while holding -> break out: manual control for the rest of this launch
 * Otherwise the driver drives normally. The launch itself is never stopped by a breakout.
 */
@Configurable
public class DriveRobotCommand extends CommandBase {

    /** Any stick beyond this counts as the driver driving (and breaks a launch hold). */
    public static double STICK_DEADBAND = 0.15;

    private final DriveSubsystem drive;
    private final GamepadEx driverGamepad;

    public DriveRobotCommand(DriveSubsystem driveSubsystem, GamepadEx driverGamepad) {
        this.drive = driveSubsystem;
        this.driverGamepad = driverGamepad;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // GamepadEx.getLeftY() is already +up. Pedro: +forward, +strafe LEFT, +turn counter-clockwise.
        double forward = driverGamepad.getLeftY();
        double strafe = -driverGamepad.getLeftX();
        double turn = -driverGamepad.getRightX();

        boolean sticksActive = Math.abs(forward) > STICK_DEADBAND
                || Math.abs(strafe) > STICK_DEADBAND
                || Math.abs(turn) > STICK_DEADBAND;

        if (drive.shouldHoldForLaunch()) {
            if (!sticksActive) {
                drive.holdCurrentPose();
                return;
            }
            if (drive.isHolding()) {
                drive.breakOutOfLaunchHold();
            }
        }

        // Normal driving. Also exits a hold if LAUNCH_HOLD_ENABLED was switched off mid-hold.
        drive.drive(forward, strafe, turn);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
