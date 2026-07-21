package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.opmodes.auto.Poses;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem; // Adjust to your actual drive subsystem name
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

public class RelocalizeCommand extends CommandBase {
    private final OdometrySubsystem odometry;
    private Pose resetPose;

    private TelemetryManager telemetry;

    public RelocalizeCommand(OdometrySubsystem odometry, TelemetryManager telemetry) {
        this.odometry = odometry;
        this.telemetry = telemetry;
        // This command doesn't "run" over time, it just sets a value, 
        // so we don't necessarily need to require the subsystem unless 
        // we want to interrupt current paths.
    }

    @Override
    public void initialize() {
        Poses poses = new Poses(CrossOpModeStorage.allianceColor == AllianceColors.RED);
        resetPose = poses.RELOCALIZE_STANCE;
    }

    @Override
    public void execute() {
        CrossOpModeStorage.currentPose = new Pose2d(resetPose.getX(), resetPose.getY(), new Rotation2d(resetPose.getHeading()));
        odometry.resetPose(CrossOpModeStorage.currentPose.getX(), CrossOpModeStorage.currentPose.getY(), Math.toDegrees(CrossOpModeStorage.currentPose.getHeading()));
        telemetry.addData("Relocalized to: ", resetPose);
    }

    @Override
    public boolean isFinished() {
        return true; // Finishes immediately after setting the pose
    }
}