package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

import java.util.ArrayList;
import java.util.List;

public class RelocalizeCommand extends CommandBase {
    private final LimelightSubsystem limelight;
    private final OdometrySubsystem odometry;
    private final DriveSubsystem drive;
    private final LaunchSubsystem launchSubsystem;

    private final List<Pose2d> samples = new ArrayList<>();
    private final ElapsedTime windowTimer = new ElapsedTime();

    // CONFIGURATION
    private static final double COLLECTION_WINDOW_MS = 200; // Collect for 0.2 seconds
    private static final int MIN_REQUIRED_SAMPLES = 5;      // Need at least 5 frames

    public RelocalizeCommand(LimelightSubsystem limelight,
                             OdometrySubsystem odometry,
                             LaunchSubsystem launchSubsystem,
                             DriveSubsystem drive) {
        this.limelight = limelight;
        this.odometry = odometry;
        this.launchSubsystem = launchSubsystem;
        this.drive = drive;

        windowTimer.reset();

        addRequirements(limelight);
    }

    @Override
    public void execute() {
        // Only sample if the robot is nearly still to prevent motion blur
        if (!drive.isEffectivelyStationary()) {
            samples.clear();
            windowTimer.reset();
            return;
        }

        Log.i("Relocalize command", "Inside execute");

        // Try to get a frame from Limelight
        Pose2d currentFrame = limelight.getLatestFieldPose();
        if (currentFrame != null) {
            samples.add(currentFrame);
        }

        // Once the window expires, process the samples
        if (windowTimer.milliseconds() >= COLLECTION_WINDOW_MS) {
            if (samples.size() >= MIN_REQUIRED_SAMPLES) {
                processAndApply();
            }
            samples.clear();
            windowTimer.reset();
        }
    }

    private void processAndApply() {
        if (samples.isEmpty()) return;

        // 1. Calculate the raw average (mean) of the samples
        double rawSumX = 0, rawSumY = 0, rawSumHeading = 0;
        for (Pose2d p : samples) {
            rawSumX += p.getX();
            rawSumY += p.getY();
            rawSumHeading += p.getHeading();
        }
        double meanX = rawSumX / samples.size();
        double meanY = rawSumY / samples.size();

        // 2. Calculate Standard Deviation to find how "spread out" the data is
        double varianceSumX = 0, varianceSumY = 0;
        for (Pose2d p : samples) {
            varianceSumX += Math.pow(p.getX() - meanX, 2);
            varianceSumY += Math.pow(p.getY() - meanY, 2);
        }
        double stdDevX = Math.sqrt(varianceSumX / samples.size());
        double stdDevY = Math.sqrt(varianceSumY / samples.size());

        // 3. Filter the samples: Keep only those within 1.5 Standard Deviations
        // This removes the "jumps" where the camera briefly sees a wrong tag or floor reflection
        List<Pose2d> filteredSamples = new ArrayList<>();
        double threshold = 1.5;

        for (Pose2d p : samples) {
            boolean isOutlierX = Math.abs(p.getX() - meanX) > (stdDevX * threshold);
            boolean isOutlierY = Math.abs(p.getY() - meanY) > (stdDevY * threshold);

            if (!isOutlierX && !isOutlierY) {
                filteredSamples.add(p);
            }
        }

        // 4. If we still have enough clean samples, calculate the final average
        if (filteredSamples.size() >= 3) {
            double finalX = 0, finalY = 0, finalHeading = 0;
            for (Pose2d p : filteredSamples) {
                finalX += p.getX();
                finalY += p.getY();
                finalHeading += p.getHeading();
            }

            // This is the Pose of the TURRET on the field
            Pose2d turretFieldPose = new Pose2d(
                    finalX / filteredSamples.size(),
                    finalY / filteredSamples.size(),
                    new Rotation2d(finalHeading / filteredSamples.size())
            );

            // --- TURRET COMPENSATION LOGIC ---

            // 1. Get the current turret angle relative to the robot chassis (in Radians)
            // Replace 'odometry.getTurretAngle()' with your actual method
            double turretAngleRelativeToRobot = launchSubsystem.getTurretAngleRadians();

            // 2. The true robot heading is the (Limelight Heading - Turret Angle)
            // Example: Limelight sees 90deg, Turret is turned 30deg right. Robot is actually at 60deg.
            double robotHeading = turretFieldPose.getHeading() - turretAngleRelativeToRobot;

            // 3. Construct the corrected Robot Pose
            Pose2d correctedRobotPose = new Pose2d(
                    turretFieldPose.getX(),
                    turretFieldPose.getY(),
                    new Rotation2d(robotHeading)
            );

            // Final Sanity Check: Don't let the camera teleport the robot more than 12 inches
            double distanceToOdometry = correctedRobotPose.getTranslation().getDistance(odometry.getPose().getTranslation());

            if (distanceToOdometry < 12.0) {
                // Update odometry with the chassis-relative corrected pose
                odometry.updatePoseFromLimelight(correctedRobotPose);
            }

        }
    }
}