package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

/**
 * Limelight fine-tuning for the turret aim. Odometry does the aiming; this only adds a small,
 * smoothed correction to the robot position the turret aims from, and only while the Limelight
 * has clean data. Odometry's pose itself is never changed (paths and auto are unaffected).
 *
 * Each new Limelight frame, the robot position is estimated from the tags of OUR HIVE only
 * (LimelightSubsystem.estimateRobotPosition with the odometry heading). The frame is clean when:
 *  - the robot is slower than MAX_SPEED_IN_PER_SEC and turning slower than MAX_TURN_DEG_PER_SEC
 *    (the frame is a few tens of ms old, so motion would smear it),
 *  - at least MIN_TAGS tags of our HIVE are seen and their estimates agree within MAX_SPREAD_IN,
 *  - the estimate is within MAX_CORRECTION_IN of odometry (a bigger jump means bad data, e.g. a
 *    misread tag or a wrong camera setup; it's rejected rather than clamped).
 * The correction (vision minus odometry) moves SMOOTHING of the way to each clean frame's value.
 * Without clean data it is held for HOLD_MS (short dropouts don't make the turret twitch), then
 * fades to zero over FADE_MS: plain odometry again.
 *
 * Aiming from tag-relative position also cancels errors in the placeholder HIVE coordinates
 * (LaunchTargets.RED_HIVE_X etc.): the tag and CELL positions share them.
 *
 * Turret angle only by default. CORRECT_DISTANCE also uses it for the shot-table distance (RPM);
 * off by default because the shot table was tuned with odometry distances.
 */
@Configurable
public class TurretVisionCorrection {

    /** Master switch, live in Panels. false = odometry only. */
    public static boolean ENABLED = true;
    /** Also use the corrected position for the distance -> RPM lookup. */
    public static boolean CORRECT_DISTANCE = false;

    public static int MIN_TAGS = 2;
    /** Largest disagreement between single-tag estimates in one frame, inches. */
    public static double MAX_SPREAD_IN = 2.0;
    /** Reject frames that would move the aim point more than this from odometry, inches. */
    public static double MAX_CORRECTION_IN = 8.0;
    public static double MAX_SPEED_IN_PER_SEC = 20.0;
    public static double MAX_TURN_DEG_PER_SEC = 45.0;
    /** 0..1: fraction of the way the correction moves toward each clean frame. */
    public static double SMOOTHING = 0.3;
    /** Keep the last correction this long after the last clean frame. */
    public static long HOLD_MS = 300;
    /** Then fade it out over this long (0 = drop at once). */
    public static long FADE_MS = 300;

    private static final String TAG = "TurretVision";

    private final LimelightSubsystem limelight; // null = no Limelight: odometry only
    private final OdometrySubsystem odometry;

    /** Smoothed correction, field inches (vision - odometry), at full weight. */
    private double offX = 0, offY = 0;
    private long lastCleanMs = -1;
    private long lastFrameMs = 0;
    private double lastHeadingRad = Double.NaN;
    private long lastHeadingMs = 0;
    private double turnDegPerSec = 0;
    private double weight = 0;
    private String reason = "no data yet";
    private boolean wasUsingVision = false;

    public TurretVisionCorrection(LimelightSubsystem limelight, OdometrySubsystem odometry) {
        this.limelight = limelight;
        this.odometry = odometry;
    }

    /** Call once per loop. Returns the robot position to aim from (odometry + correction). */
    public Translation2d update(Pose2d pose, AllianceColors alliance) {
        long now = System.nanoTime() / 1_000_000L;
        updateTurnRate(pose.getHeading(), now);

        if (!ENABLED || limelight == null) {
            reset(!ENABLED ? "off" : "no Limelight");
            return pose.getTranslation();
        }

        long frameMs = limelight.getFrameTimestampMs();
        if (frameMs != lastFrameMs) {
            lastFrameMs = frameMs;
            evaluateFrame(pose, alliance, now);
        } else if (limelight.getObservations().isEmpty()) {
            reason = "no tags in view";
        }

        // Weight: 1 while fresh, then fade to 0.
        long age = lastCleanMs < 0 ? Long.MAX_VALUE : now - lastCleanMs;
        if (age <= HOLD_MS) {
            weight = 1;
        } else if (FADE_MS > 0 && age < HOLD_MS + FADE_MS) {
            weight = 1.0 - (double) (age - HOLD_MS) / FADE_MS;
        } else {
            weight = 0;
            offX = 0;
            offY = 0;
        }

        boolean usingVision = weight > 0;
        if (usingVision != wasUsingVision) {
            Log.i(TAG, usingVision
                    ? String.format("Using Limelight: correction (%.1f, %.1f) in", offX, offY)
                    : "Odometry only: " + reason);
            wasUsingVision = usingVision;
        }
        return new Translation2d(pose.getX() + weight * offX, pose.getY() + weight * offY);
    }

    private void evaluateFrame(Pose2d pose, AllianceColors alliance, long now) {
        double speed = odometry.getFieldSpeedInchesPerSecond();
        if (speed > MAX_SPEED_IN_PER_SEC) { reason = "moving too fast"; return; }
        if (turnDegPerSec > MAX_TURN_DEG_PER_SEC) { reason = "turning too fast"; return; }

        LimelightSubsystem.PositionEstimate est = limelight.estimateRobotPosition(pose.getHeading(), alliance);
        if (est == null) { reason = "no tags of our HIVE"; return; }
        if (est.tagCount < MIN_TAGS) { reason = est.tagCount + " tag(s), need " + MIN_TAGS; return; }
        if (est.spreadIn > MAX_SPREAD_IN) { reason = String.format("tags disagree (%.1f in)", est.spreadIn); return; }

        double dx = est.position.getX() - pose.getX();
        double dy = est.position.getY() - pose.getY();
        if (Math.hypot(dx, dy) > MAX_CORRECTION_IN) {
            reason = String.format("too far from odometry (%.1f in)", Math.hypot(dx, dy));
            return;
        }

        // Continue from what is applied now (it may be part-faded), then smooth toward this frame.
        offX *= weight;
        offY *= weight;
        double a = Math.max(0, Math.min(1, SMOOTHING));
        offX += a * (dx - offX);
        offY += a * (dy - offY);
        lastCleanMs = now;
        weight = 1;
        reason = est.tagCount + " tags";
    }

    private void updateTurnRate(double headingRad, long now) {
        if (!Double.isNaN(lastHeadingRad) && now > lastHeadingMs) {
            double d = Math.toDegrees(headingRad - lastHeadingRad);
            d = LaunchReadinessCommand.normalizeDeg(d);
            turnDegPerSec = Math.abs(d) * 1000.0 / (now - lastHeadingMs);
        }
        lastHeadingRad = headingRad;
        lastHeadingMs = now;
    }

    private void reset(String why) {
        offX = 0;
        offY = 0;
        weight = 0;
        lastCleanMs = -1;
        reason = why;
        wasUsingVision = false;
    }

    /** For telemetry: "LIMELIGHT (dx, dy)" or "odometry (reason)". */
    public String status() {
        return weight > 0
                ? String.format("LIMELIGHT (%.1f, %.1f) in%s", weight * offX, weight * offY, weight < 1 ? " fading" : "")
                : "odometry (" + reason + ")";
    }

    public boolean isUsingVision() {
        return weight > 0;
    }
}
