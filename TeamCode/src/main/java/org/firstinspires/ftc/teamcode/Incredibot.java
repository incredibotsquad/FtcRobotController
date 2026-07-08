package org.firstinspires.ftc.teamcode;


import android.util.Log;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AutoFireCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchBallsCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchReadinessCommand;
import org.firstinspires.ftc.teamcode.commands.DriveRobotCommand;
import org.firstinspires.ftc.teamcode.commands.SmartIntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchKickSubsystem;
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
    public final LaunchSubsystem launchSubsystem;
    public final LaunchGateSubsystem launchGateSubsystem;
    public final LaunchKickSubsystem launchKickSubsystem;
    public final OdometrySubsystem odometrySubsystem;

    // Hardware
    private final HardwareMap hwMap;
    private TelemetryManager telemetry;

    public Incredibot(HardwareMap hardwareMap, OpModeType opModeType, GamepadEx driverGamepad, GamepadEx operatorGamepad, TelemetryManager telemetry) {
        this.hwMap = hardwareMap;
        this.telemetry = telemetry;

        // Initialize subsystems - they initialize their own hardware
        driveSubsystem = new DriveSubsystem(hwMap, telemetry);
        intakeSubsystem = new IntakeSubsystem(hwMap, telemetry);
        launchGateSubsystem = new LaunchGateSubsystem(hwMap, telemetry);
        odometrySubsystem = new OdometrySubsystem(hwMap, telemetry);
        launchSubsystem = new LaunchSubsystem(hwMap, telemetry);
        launchKickSubsystem = new LaunchKickSubsystem(hardwareMap, telemetry);

        if (opModeType == OpModeType.TELEOP) {
            initTeleop(driverGamepad, operatorGamepad);
        } else if (opModeType == OpModeType.AUTO) {
            initAuto();
        }
    }

    public void initTeleop(GamepadEx driverGamepad, GamepadEx operatorGamepad) {
        CommandScheduler.getInstance().reset();

        //TODO: MAKE SURE THE LAUNCHER GATE IS CLOSED UPON BOT START - WE WILL DO IT MANUALLY BUT WE NEED TO DO IT IN CODE AS WELL
        //THIS CANNOT BE HERE IN INIT - HAS TO BE DONE FIRST THING AFTER START
        launchGateSubsystem.closeGate();

        initCommon();

        // 3. Assign default commands or button bindings
        // The default command gets automatically scheduled when there is no other command for the subsystem.

        driveSubsystem.setDefaultCommand(new DriveRobotCommand(driveSubsystem, driverGamepad));

        //TODO: CREATE AN OVERRIDE TO LOCK THE TURRET IN CENTER POSITION IN CASE ODOMETRY MESSES UP
        //TODO: ADD AN OPTION TO INITIALIZE AN ALLIANCE COLOR IN TELEOP

        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).
                whenHeld(new LaunchBallsCommand(launchSubsystem, launchGateSubsystem));

//        register(odometrySubsystem);

        // 2. AUTO-FIRE TOGGLE (While held)
        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whileHeld(new AutoFireCommand(launchSubsystem, launchGateSubsystem, odometrySubsystem));
    }

    public void initAuto() {
        // Clear out any lingering commands or bindings from previous runs
        CommandScheduler.getInstance().reset();

        initCommon();

        // Notice: We don't bind ANY gamepads here.
        // The robot will rely purely on scripted sequential commands.

    }

    private void initCommon() {
        register(driveSubsystem, intakeSubsystem, launchGateSubsystem, launchSubsystem, odometrySubsystem, launchKickSubsystem);

        // It will start at match start and manage itself based on sensor data
        intakeSubsystem.setDefaultCommand(new SmartIntakeCommand(intakeSubsystem, launchGateSubsystem));

        // Assign the background tracking loop here!
        // The scheduler will now call execute() on this command every single frame.
        launchSubsystem.setDefaultCommand(new LaunchReadinessCommand(launchSubsystem, odometrySubsystem, telemetry));
    }
}