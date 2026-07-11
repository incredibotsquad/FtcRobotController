package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

import java.util.ArrayList;
import java.util.List;

public class RelocalizeCommand extends CommandBase {
    private final LimelightSubsystem limelight;
    private final OdometrySubsystem odometry;
    private final DriveSubsystem drive;

    private final List<Pose2d> samples = new ArrayList<>();
    private final ElapsedTime windowTimer = new ElapsedTime();

    // CONFIGURATION
    private static final double COLLECTION_WINDOW_MS = 200; // Collect for 0.2 seconds
    private static final int MIN_REQUIRED_SAMPLES = 5;      // Need at least 5 frames

    public RelocalizeCommand(LimelightSubsystem limelight,
                             OdometrySubsystem odometry,
                             DriveSubsystem drive) {
        this.limelight = limelight;
        this.odometry = odometry;
        this.drive = drive;

        windowTimer.reset();
    }

    @Override
    public void execute() {
        // Only sample if the robot is nearly still to prevent motion blur
        if (!drive.isEffectivelyStationary()) {
            samples.clear();
            windowTimer.reset();
            return;
        }

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

            Pose2d filteredPose = new Pose2d(
                    finalX / filteredSamples.size(),
                    finalY / filteredSamples.size(),
                    new Rotation2d(finalHeading / filteredSamples.size())
            );

            // Final Sanity Check: Don't let the camera teleport the robot more than 12 inches
            double distanceToOdometry = filteredPose.getTranslation().getDistance(odometry.getPose().getTranslation());
            if (distanceToOdometry < 12.0) {
                odometry.updatePoseFromLimelight(filteredPose);
            }
        }
    }
}