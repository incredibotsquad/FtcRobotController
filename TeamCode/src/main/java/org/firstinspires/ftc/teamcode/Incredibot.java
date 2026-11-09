package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.CloseGateCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchBallsCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchReadinessCommand;
import org.firstinspires.ftc.teamcode.commands.DriveRobotCommand;
import org.firstinspires.ftc.teamcode.commands.RelocalizeCommand;
import org.firstinspires.ftc.teamcode.commands.RelocalizeWithLimelightCommand;
import org.firstinspires.ftc.teamcode.commands.SmartIntakeCommand;
import org.firstinspires.ftc.teamcode.commands.ReverseIntakeCommand;
import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.common.HiveState;
import org.firstinspires.ftc.teamcode.common.LaunchTargets;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeBarSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;

@Configurable
public class Incredibot extends Robot {
    /**
     * Drive-only mode: just the drivetrain (and odometry, for the pose), with the turret locked at
     * TURRET_MID. Intake, transfer, gate, flywheel, Limelight relocalization and all operator
     * buttons are off; their hardware is never commanded, so motors stay at 0 and those servos
     * unpowered. Read when the OpMode starts
     * (INIT/START), so set it in Panels before pressing INIT.
     */
    public static boolean DRIVE_ONLY_MODE = false;

    // enum to specify opmode type
    public enum OpModeType {
        TELEOP,
        AUTO
    }

    // Subsystems
    public final DriveSubsystem driveSubsystem;
    public final IntakeSubsystem intakeSubsystem;
    public final IntakeBarSubsystem intakeBarSubsystem;
    public final LaunchSubsystem launchSubsystem;
    public final LaunchGateSubsystem launchGateSubsystem;
    public final OdometrySubsystem odometrySubsystem;
    public final LimelightSubsystem limelightSubsystem;
    /** The launcher's default command, set by initTeleop()/initAuto() (null in drive-only mode). Read by FlywheelTuner. */
    public LaunchReadinessCommand launchReadinessCommand;
    // Hardware
    private final HardwareMap hwMap;
    private TelemetryManager telemetry;

    public Incredibot(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.hwMap = hardwareMap;
        this.telemetry = telemetry;

        // Read all hub data in one bulk call per loop: faster loops = faster flywheel recovery.
        for (LynxModule hub : hwMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Initialize subsystems - they initialize their own hardware
        driveSubsystem = new DriveSubsystem(hwMap, telemetry);
        intakeSubsystem = new IntakeSubsystem(hwMap, telemetry);
        intakeBarSubsystem = new IntakeBarSubsystem(hwMap, telemetry); // does not move the servos
        launchGateSubsystem = new LaunchGateSubsystem(hwMap, telemetry);
        odometrySubsystem = new OdometrySubsystem(driveSubsystem, telemetry); // reads the Pinpoint via Pedro's Follower
        launchSubsystem = new LaunchSubsystem(hwMap, telemetry);
        limelightSubsystem = new LimelightSubsystem(hwMap, telemetry);
    }

    public void initialize(OpModeType opModeType, GamepadEx driverGamepad, GamepadEx operatorGamepad) {
        if (opModeType == OpModeType.TELEOP) {
            initTeleop(driverGamepad, operatorGamepad);
        } else if (opModeType == OpModeType.AUTO) {
            initAuto();
        }
    }

    public void initTeleop(GamepadEx driverGamepad, GamepadEx operatorGamepad) {
        CommandScheduler.getInstance().reset();
        LaunchTargets.clearForcedCell(); // an AUTO launch step may have left one set

        if (DRIVE_ONLY_MODE) {
            initDriveOnly();
            driveSubsystem.setDefaultCommand(new DriveRobotCommand(driveSubsystem, driverGamepad));
            return;
        }

        initCommon();

        // 3. Assign default commands or button bindings
        // The default command gets automatically scheduled when there is no other command for the subsystem.

        driveSubsystem.setDefaultCommand(new DriveRobotCommand(driveSubsystem, driverGamepad));

        //toggle for shooting on the move
        operatorGamepad.getGamepadButton(GamepadKeys.Button.X)
                        .toggleWhenPressed(
                                new InstantCommand(() -> LaunchReadinessCommand.ENABLE_MOVING_SHOT_COMPENSATION = true),
                                new InstantCommand(() -> LaunchReadinessCommand.ENABLE_MOVING_SHOT_COMPENSATION = false)
                        );

        // 3. TURRET LOCK OVERRIDE - Pressing START toggles the turret lock
        operatorGamepad.getGamepadButton(GamepadKeys.Button.START)
                .toggleWhenPressed(
                        new InstantCommand(() -> launchSubsystem.setLaunchReadinessLock(true)),
                        new InstantCommand(() -> launchSubsystem.setLaunchReadinessLock(false))
                );

        operatorGamepad.getGamepadButton(GamepadKeys.Button.BACK)
                .whileHeld(new ReverseIntakeCommand(intakeSubsystem));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new LaunchBallsCommand(launchGateSubsystem, intakeSubsystem, driveSubsystem, intakeBarSubsystem));

        operatorGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new RelocalizeCommand(odometrySubsystem, telemetry));

        // Intake bar: A lowers it in front of the robot (FLOWER POLLEN pickup) / raises it again.
        // The TeleOp already holds it RAISED from INIT, so the first press lowers it.
        operatorGamepad.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new InstantCommand(intakeBarSubsystem::toggle, intakeBarSubsystem));
    }

    public void initAuto() {
        // Clear out any lingering commands or bindings from previous runs
        CommandScheduler.getInstance().reset();
        LaunchTargets.clearForcedCell();

        if (DRIVE_ONLY_MODE) {
            initDriveOnly();
            return;
        }

        initCommon();

        // G304.G: every robot starts the match with exactly 4 POLLEN pre-loads. Without this the
        // count starts at 0 and SmartIntakeCommand runs the intake until the jam check marks it full.
        intakeSubsystem.setArtifactCount(IntakeSubsystem.MAX_ARTIFACTS);

        // Both HIVES start in their match-start state; the Limelight tracks TIPS from here.
        HiveState.resetToMatchStart();

        // Notice: We don't bind ANY gamepads here.
        // The robot will rely purely on scripted sequential commands.
    }

    /**
     * Only the drivetrain and odometry run. The turret is parked at TURRET_MID once (the servo
     * holds it there); no other subsystem is registered or commanded. Called at START.
     */
    private void initDriveOnly() {
        register(driveSubsystem, odometrySubsystem);
        launchSubsystem.setTurretPosition(LaunchSubsystem.TURRET_MID);
    }

    private void initCommon() {
        register(driveSubsystem, intakeSubsystem, intakeBarSubsystem, launchGateSubsystem, launchSubsystem, odometrySubsystem, limelightSubsystem);

        // 1. SET THE DEFAULT COMMAND FOR THE GATE
        // This will run at match start and any time the gate isn't being used by a launcher command.
        launchGateSubsystem.setDefaultCommand(new CloseGateCommand(launchGateSubsystem));

        // It will start at match start and manage itself based on sensor data
        intakeSubsystem.setDefaultCommand(new SmartIntakeCommand(intakeSubsystem, launchGateSubsystem, odometrySubsystem));

        // Assign the background tracking loop here!
        // The scheduler will now call execute() on this command every single frame.
        launchReadinessCommand = new LaunchReadinessCommand(launchSubsystem, odometrySubsystem, telemetry);
        launchSubsystem.setDefaultCommand(launchReadinessCommand);
    }
}