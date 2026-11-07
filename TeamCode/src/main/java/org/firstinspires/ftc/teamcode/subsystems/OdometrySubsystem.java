package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.common.DebugTelemetry;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;

/**
 * Robot pose for the rest of the code, in FTCLib types (inches, heading radians, field frame).
 *
 * The Pinpoint is owned by Pedro's PinpointLocalizer inside DriveSubsystem's Follower (pod offsets
 * and directions live in pedro/Constants.localizerConfig). DriveSubsystem.periodic() runs
 * follower.update() first (it is registered first, and FTCLib runs periodic() in registration
 * order), then this periodic() copies the fresh pose, estimates velocity/acceleration, and writes
 * the pose to CrossOpModeStorage. Pose resets go through the Follower so Pedro and this class
 * always agree.
 */
public class OdometrySubsystem extends SubsystemBase {

    private final DriveSubsystem drive;
    private final TelemetryManager telemetry;

    private Pose2d currentPose = new Pose2d();
    private Pose2d previousPose = new Pose2d();
    private Translation2d fieldVelocity = new Translation2d(0, 0);
    private Translation2d fieldAcceleration = new Translation2d(0, 0);
    private long previousUpdateNanos = 0;

    public OdometrySubsystem(DriveSubsystem drive, TelemetryManager telemetry) {
        this.drive = drive;
        this.telemetry = telemetry;
        currentPose = toPose2d(drive.getPose());
        previousPose = currentPose;
    }

    public Pose2d getPose() {
        return currentPose;
    }

    public Translation2d getFieldVelocity() {
        return fieldVelocity;
    }

    public Translation2d getFieldAcceleration() {
        return fieldAcceleration;
    }

    public double getFieldSpeedInchesPerSecond() {
        return Math.hypot(fieldVelocity.getX(), fieldVelocity.getY());
    }

    /** Sets the pose (e.g. start of AUTO, carry-over into TELEOP, fixed-stance relocalization). */
    public void resetPose(double xInches, double yInches, double rotationDegrees) {
        Log.i("Odometry subsystem", "Reset pose. X: " + xInches + " Y: " + yInches + " Angle: " + rotationDegrees);
        applyPose(new Pose2d(xInches, yInches, new Rotation2d(Math.toRadians(rotationDegrees))));
    }

    /** Vision correction from RelocalizeWithLimelightCommand. */
    public void updatePoseFromLimelight(Pose2d correctedPose) {
        Log.i("Odometry Subsystem", "Updating pose from limelight. Old pose: " + currentPose + " New pose: " + correctedPose);
        applyPose(correctedPose);
    }

    /**
     * Writes the pose into the Pinpoint via the Follower and treats it as a reset, not motion:
     * otherwise the next periodic() sees the jump as a one-loop velocity spike (which feeds the
     * moving-shot lead and the speed gate).
     */
    private void applyPose(Pose2d pose) {
        drive.setPose(new Pose(pose.getX(), pose.getY(), pose.getHeading()));
        currentPose = pose;
        previousPose = pose;
        fieldVelocity = new Translation2d(0, 0);
        fieldAcceleration = new Translation2d(0, 0);
        previousUpdateNanos = System.nanoTime();
        CrossOpModeStorage.currentPose = pose;
    }

    private void updateVelocityEstimate(Pose2d newPose) {
        long now = System.nanoTime();

        if (previousUpdateNanos != 0) {
            double dtSeconds = (now - previousUpdateNanos) / 1.0e9;

            if (dtSeconds > 0.0) {
                Translation2d previousVelocity = fieldVelocity;
                fieldVelocity = new Translation2d(
                        (newPose.getX() - previousPose.getX()) / dtSeconds,
                        (newPose.getY() - previousPose.getY()) / dtSeconds
                );
                fieldAcceleration = new Translation2d(
                        (fieldVelocity.getX() - previousVelocity.getX()) / dtSeconds,
                        (fieldVelocity.getY() - previousVelocity.getY()) / dtSeconds
                );
            }
        }

        previousPose = newPose;
        previousUpdateNanos = now;
    }

    private static Pose2d toPose2d(Pose p) {
        return new Pose2d(p.x(), p.y(), new Rotation2d(p.heading()));
    }

    @Override
    public void periodic() {
        Pose2d newPose = toPose2d(drive.getPose());

        //sometimes the follower will return 0,0. don't use that for anything
        if (newPose.getX() == 0 && newPose.getY() == 0) return;

        updateVelocityEstimate(newPose);

        currentPose = newPose;
        CrossOpModeStorage.currentPose = currentPose;

        if (DebugTelemetry.ENABLED) telemetry.addData("Odometry: X Position", newPose.getX());
        if (DebugTelemetry.ENABLED) telemetry.addData("Odometry: Y Position", newPose.getY());
        if (DebugTelemetry.ENABLED) telemetry.addData("Odometry: Heading (Deg)", Math.toDegrees(newPose.getHeading()));
    }
}
