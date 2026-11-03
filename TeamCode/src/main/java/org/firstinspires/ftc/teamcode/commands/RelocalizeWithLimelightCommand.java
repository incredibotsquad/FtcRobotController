package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

/**
 * One-shot X/Y correction of the odometry pose from the HIVE AprilTags.
 *
 * Collects SAMPLES_NEEDED good Limelight frames while the robot is stationary, averages them, and
 * writes the result into the Pinpoint. The heading is kept from odometry (the Pinpoint IMU is more
 * accurate than the tags). A frame is good when it has at least MIN_TAGS tags whose individual
 * estimates agree within MAX_SPREAD_IN. Gives up after TIMEOUT_MS without changing the pose.
 *
 * Declares no requirements: it only reads the drive and Limelight and writes the odometry pose,
 * so it never interrupts driving or aiming.
 */
@Configurable
public class RelocalizeWithLimelightCommand extends CommandBase {

    /** Master switch. false = the command finishes immediately and never touches the pose. Live in Panels. */
    public static boolean ENABLED = false;

    public static int SAMPLES_NEEDED = 5;
    public static int MIN_TAGS = 2;
    public static double MAX_SPREAD_IN = 3.0;
    public static long TIMEOUT_MS = 2000;

    private final LimelightSubsystem limelight;
    private final OdometrySubsystem odometry;
    private final DriveSubsystem drive;

    private long startMs;
    private long lastFrameMs;
    private int samples;
    private double sumX, sumY;
    private boolean done;

    public RelocalizeWithLimelightCommand(LimelightSubsystem limelightSubsystem,
                                          OdometrySubsystem odometrySubsystem,
                                          DriveSubsystem driveSubsystem) {
        this.limelight = limelightSubsystem;
        this.odometry = odometrySubsystem;
        this.drive = driveSubsystem;
    }

    @Override
    public void initialize() {
        startMs = System.currentTimeMillis();
        lastFrameMs = 0;
        samples = 0;
        sumX = 0;
        sumY = 0;
        done = !ENABLED;
        if (!ENABLED) Log.i("RelocalizeWithLimelight", "Disabled (ENABLED = false), pose unchanged");
    }

    @Override
    public void execute() {
        if (done) return;
        if (!ENABLED) { // turned off while running
            done = true;
            return;
        }

        if (System.currentTimeMillis() - startMs > TIMEOUT_MS) {
            Log.w("RelocalizeWithLimelight", "Timed out with " + samples + "/" + SAMPLES_NEEDED + " samples, pose unchanged");
            done = true;
            return;
        }

        long frameMs = limelight.getFrameTimestampMs();
        if (frameMs == lastFrameMs) return; // same Limelight frame as last loop
        lastFrameMs = frameMs;

        if (!drive.isEffectivelyStationary()) {
            // Moving invalidates the samples collected so far (they were taken at another position).
            samples = 0;
            sumX = 0;
            sumY = 0;
            return;
        }

        LimelightSubsystem.PositionEstimate est = limelight.estimateRobotPosition(odometry.getPose().getHeading());
        if (est == null || est.tagCount < MIN_TAGS || est.spreadIn > MAX_SPREAD_IN) return;

        sumX += est.position.getX();
        sumY += est.position.getY();
        samples++;

        if (samples >= SAMPLES_NEEDED) {
            Pose2d current = odometry.getPose();
            Pose2d corrected = new Pose2d(sumX / samples, sumY / samples, new Rotation2d(current.getHeading()));
            odometry.updatePoseFromLimelight(corrected);
            done = true;
        }
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
