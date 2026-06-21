package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutoAimTurretCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class Incredibot extends Robot {
    // enum to specify opmode type
    public enum OpModeType {
        TELEOP,
        AUTO
    }

    // Subsystems
    public final DriveSubsystem driveSubsystem;
    public final IntakeSubsystem intakeSubsystem;
    public final LaunchSubsystem launchSubsystem;
    public final TurretSubsystem turretSubsystem;
    public final OdometrySubsystem odometrySubsystem;

    // Hardware
    private final HardwareMap hwMap;

    public Incredibot(HardwareMap hardwareMap, OpModeType opModeType, GamepadEx driverGamepad, Telemetry telemetry) {
        this.hwMap = hardwareMap;

        // Initialize subsystems - they initialize their own hardware
        driveSubsystem = new DriveSubsystem(hwMap, telemetry);
        intakeSubsystem = new IntakeSubsystem(hwMap, telemetry);
        launchSubsystem = new LaunchSubsystem(hwMap, telemetry);
        odometrySubsystem = new OdometrySubsystem(hwMap, telemetry);
        turretSubsystem = new TurretSubsystem(hwMap, telemetry);

        if (opModeType == OpModeType.TELEOP) {
            initTeleop(driverGamepad);
        } else if (opModeType == OpModeType.AUTO) {
            initAuto();
        }
    }

    public void initTeleop(GamepadEx driverGamepad) {
        CommandScheduler.getInstance().reset();

        // 3. Assign default commands or button bindings
        // The default command gets automatically scheduled when there is no other command for the subsystem.
        driveSubsystem.setDefaultCommand(new InstantCommand(
                () -> driveSubsystem.drive(
                        driverGamepad.getLeftY(),
                        driverGamepad.getLeftX(),
                        driverGamepad.getRightX()
                )));

        // Assign the background tracking loop here!
        // The scheduler will now call execute() on this command every single frame.
        turretSubsystem.setDefaultCommand(new AutoAimTurretCommand(turretSubsystem, odometrySubsystem));
    }

    public void initAuto() {
        // Clear out any lingering commands or bindings from previous runs
        CommandScheduler.getInstance().reset();

        // Notice: We don't bind ANY gamepads here.
        // The robot will rely purely on scripted sequential commands.
    }
}