package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutoFireCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchReadinessCommand;
import org.firstinspires.ftc.teamcode.commands.DriveRobotCommand;
import org.firstinspires.ftc.teamcode.commands.SmartIntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;

public class Incredibot extends Robot {
    // enum to specify opmode type
    public enum OpModeType {
        TELEOP,
        AUTO
    }

    // Subsystems
    public final DriveSubsystem driveSubsystem;
    public final IntakeSubsystem intakeSubsystem;
    public final LaunchGateSubsystem launchGateSubsystem;
    public final LaunchSubsystem launchSubsystem;
    public final OdometrySubsystem odometrySubsystem;

    // Hardware
    private final HardwareMap hwMap;

    public Incredibot(HardwareMap hardwareMap, OpModeType opModeType, GamepadEx driverGamepad, GamepadEx operatorGamepad, Telemetry telemetry) {
        this.hwMap = hardwareMap;

        // Initialize subsystems - they initialize their own hardware
        driveSubsystem = new DriveSubsystem(hwMap, telemetry);
        intakeSubsystem = new IntakeSubsystem(hwMap, telemetry);
        launchGateSubsystem = new LaunchGateSubsystem(hwMap, telemetry);
        odometrySubsystem = new OdometrySubsystem(hwMap, telemetry);
        launchSubsystem = new LaunchSubsystem(hwMap, telemetry);

        if (opModeType == OpModeType.TELEOP) {
            initTeleop(driverGamepad, operatorGamepad);
        } else if (opModeType == OpModeType.AUTO) {
            initAuto();
        }

    }

    public void initTeleop(GamepadEx driverGamepad, GamepadEx operatorGamepad) {
        CommandScheduler.getInstance().reset();

        // 3. Assign default commands or button bindings
        // The default command gets automatically scheduled when there is no other command for the subsystem.

        driveSubsystem.setDefaultCommand(new DriveRobotCommand(driveSubsystem, driverGamepad));

        // 2. AUTO-FIRE TOGGLE (While held)
//        operatorGamepad
//                .getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
//                .whileHeld(new AutoFireCommand(launchSubsystem, launchGateSubsystem, odometrySubsystem));

//        initCommon();
    }

    public void initAuto() {
        // Clear out any lingering commands or bindings from previous runs
        CommandScheduler.getInstance().reset();

        // Notice: We don't bind ANY gamepads here.
        // The robot will rely purely on scripted sequential commands.

        initCommon();
    }

    private void initCommon() {

        // It will start at match start and manage itself based on sensor data
        intakeSubsystem.setDefaultCommand(new SmartIntakeCommand(intakeSubsystem, launchGateSubsystem));

        // Assign the background tracking loop here!
        // The scheduler will now call execute() on this command every single frame.
        launchSubsystem.setDefaultCommand(new LaunchReadinessCommand(launchSubsystem, odometrySubsystem));
    }
}