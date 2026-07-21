package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

import java.util.ArrayList;
import java.util.List;

@Configurable
public class RelocalizeWithLimelightCommand extends CommandBase {
    private final LimelightSubsystem limelight;
    private final OdometrySubsystem odometry;
    private final DriveSubsystem drive;
    private final LaunchSubsystem launchSubsystem;

    private final List<Pose> samples = new ArrayList<>();
    private final ElapsedTime windowTimer = new ElapsedTime();

    // CONFIGURATION
    public static final double COLLECTION_WINDOW_MS = 200; // Collect for 0.2 seconds
    public static final int MIN_REQUIRED_SAMPLES = 5;      // Need at least 5 frames

    public static double STD_DEV_THRESHOLD = 1.5;

    public static boolean ENABLE_RELOCALIZATION = false;

    public RelocalizeWithLimelightCommand(LimelightSubsystem limelight,
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
        if(!ENABLE_RELOCALIZATION)
            return;

        Log.i("Relocalize command", "Inside execute.");

        // Only sample if the robot is nearly still to prevent motion blur
        if (!drive.isEffectivelyStationary()) {
            samples.clear();
            windowTimer.reset();
            return;
        }

        // Try to get a frame from Limelight
        Pose currentFrame = limelight.getLatestFieldPose();

        if (currentFrame != null) {
            samples.add(currentFrame);
            Log.i("Relocalize command", "Inside execute. New pose: " + currentFrame.toString());
        }

        // Once the window expires, process the samples
        if (windowTimer.milliseconds() >= COLLECTION_WINDOW_MS) {
            if (samples.size() >= MIN_REQUIRED_SAMPLES) {
                Log.i("Relocalize command", "Collected samples: " + samples.size());

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
        for (Pose p : samples) {
            rawSumX += p.getX();
            rawSumY += p.getY();
            rawSumHeading += p.getHeading();
        }
        double meanX = rawSumX / samples.size();
        double meanY = rawSumY / samples.size();

        // 2. Calculate Standard Deviation to find how "spread out" the data is
        double varianceSumX = 0, varianceSumY = 0;
        for (Pose p : samples) {
            varianceSumX += Math.pow(p.getX() - meanX, 2);
            varianceSumY += Math.pow(p.getY() - meanY, 2);
        }
        double stdDevX = Math.sqrt(varianceSumX / samples.size());
        double stdDevY = Math.sqrt(varianceSumY / samples.size());

        // 3. Filter the samples: Keep only those within 1.5 Standard Deviations
        // This removes the "jumps" where the camera briefly sees a wrong tag or floor reflection
        List<Pose> filteredSamples = new ArrayList<>();
        double threshold = STD_DEV_THRESHOLD;

        for (Pose p : samples) {
            boolean isOutlierX = Math.abs(p.getX() - meanX) > (stdDevX * threshold);
            boolean isOutlierY = Math.abs(p.getY() - meanY) > (stdDevY * threshold);

            Log.i("Relocalize command", "Filtering samples. X out:" + isOutlierX + " y out: " + isOutlierY);

            if (!isOutlierX && !isOutlierY) {
                Log.i("Relocalize command", "Filtering samples. Added point: " + p.toString());
                filteredSamples.add(p);
            }
        }

        Log.i("Relocalize command", "Number of filtered samples: " + filteredSamples.size());

        // 4. If we still have enough clean samples, calculate the final average
        if (filteredSamples.size() >= 3) {
            // 1. Get the average field-relative pose reported by Limelight
            double sumX = 0, sumY = 0, sumHeading = 0;
            for (Pose p : filteredSamples) {
                sumX += p.getX();
                sumY += p.getY();
                sumHeading += p.getHeading();
            }

            // This is the absolute angle the Limelight is facing on the field
            double absoluteLimelightHeading = sumHeading / filteredSamples.size();
            double avgX = sumX / filteredSamples.size();
            double avgY = sumY / filteredSamples.size();

            // 2. Get the current Turret Servo Position from the LaunchSubsystem
            double currentServoPos = launchSubsystem.getTurretPosition();

            // 3. Reverse the Servo -> Degree math from LaunchReadinessCommand
            // Note: servoPosAdjustment = servoOffsetDegrees / TOTAL_SERVO_RANGE
            double servoPosAdjustment = LaunchSubsystem.TURRET_MID - currentServoPos;
            double servoOffsetDegrees = servoPosAdjustment * LaunchSubsystem.TOTAL_SERVO_RANGE;

            // 4. Reverse the Gear Ratio to get degrees relative to Chassis
            // Note: servoOffsetDegrees = relativeTargetAngle * GEAR_RATIO
            double relativeTurretAngle = servoOffsetDegrees / LaunchSubsystem.GEAR_RATIO;

            Log.i("Relocalize command", "relativeTurretAngle: " + relativeTurretAngle);

            // 5. Calculate true Robot Heading
            // Since absoluteTarget = robotHeading + relativeAngle
            double robotHeadingDegrees = Math.toDegrees(absoluteLimelightHeading) - relativeTurretAngle;

            // Normalize heading to [-180, 180]
            while (robotHeadingDegrees > 180) robotHeadingDegrees -= 360;
            while (robotHeadingDegrees < -180) robotHeadingDegrees += 360;

            // 6. Create the corrected pose
            Pose2d correctedRobotPose = new Pose2d(
                    avgX,
                    avgY,
                    Rotation2d.fromDegrees(robotHeadingDegrees)
            );

            Log.i("Relocalize command", "Odometry pose old: " + odometry.getPose().toString());

            Log.i("Relocalize command", "Odometry pose new: Calculated robot pose: " + correctedRobotPose.toString());

            // Final Sanity Check: Don't let the camera teleport the robot more than 12 inches
            double distanceToOdometry = correctedRobotPose.getTranslation().getDistance(odometry.getPose().getTranslation());

            Log.i("Relocalize command", "Odometry pose: Distance to odometry: " + distanceToOdometry);

            if (distanceToOdometry < 12.0) {
                Log.i("Relocalize command", "Performed all filtering - calling odometry to update pose");

                // Update odometry with the chassis-relative corrected pose
                odometry.updatePoseFromLimelight(correctedRobotPose);
            }
        }
    }
}