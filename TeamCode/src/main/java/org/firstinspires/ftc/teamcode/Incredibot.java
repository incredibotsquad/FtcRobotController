package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HiveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

public class Incredibot extends Robot {

    public enum OpModeType { TELEOP, AUTO }

    public final DriveSubsystem      driveSubsystem;
    public final IntakeSubsystem     intakeSubsystem;
    public final LaunchSubsystem     launchSubsystem;
    public final LaunchGateSubsystem launchGateSubsystem;
    public final OdometrySubsystem   odometrySubsystem;
    // Pipeline 0 = the AprilTag HIVE pipeline configured in Limelight Manager (see docs/HiveSubsystem.md)
    public final HiveSubsystem       hiveSubsystem;

    private final TelemetryManager telemetry;

    public Incredibot(HardwareMap hardwareMap, TelemetryManager telemetry) {
        this.telemetry = telemetry;

        driveSubsystem      = new DriveSubsystem(hardwareMap, telemetry);
        intakeSubsystem     = new IntakeSubsystem(hardwareMap, telemetry);
        launchGateSubsystem = new LaunchGateSubsystem(hardwareMap, telemetry);
        odometrySubsystem   = new OdometrySubsystem(hardwareMap, telemetry);
        launchSubsystem     = new LaunchSubsystem(hardwareMap, telemetry);
        hiveSubsystem       = new HiveSubsystem(hardwareMap, telemetry, 0);
        hiveSubsystem.start(CrossOpModeStorage.allianceColor);
    }

    public void initialize(OpModeType opModeType) {
        CommandScheduler.getInstance().reset();
        register(driveSubsystem, intakeSubsystem, launchGateSubsystem,
                launchSubsystem, odometrySubsystem, hiveSubsystem);
    }
}
