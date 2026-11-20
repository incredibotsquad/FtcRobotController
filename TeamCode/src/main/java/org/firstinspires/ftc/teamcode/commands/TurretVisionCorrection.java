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
 * CORRECT_DISTANCE (on) also uses the corrected position for the shot-table distance (RPM). The
 * shot rows were tuned from fresh start poses, where odometry and vision agree.
 */
@Configurable
public class TurretVisionCorrection {

    /** Master switch, live in Panels. false = odometry only. */
    public static boolean ENABLED = true;
    /** Also use the corrected position for the distance -> RPM lookup. */
    public static boolean CORRECT_DISTANCE = true; // on since 2026-11-19: odometry drifted 8-10 in in a run

    public static int MIN_TAGS = 2;
    /** Largest disagreement between single-tag estimates in one frame, inches. */
    public static double MAX_SPREAD_IN = 3.0; // 2.0 -> 3.0 (2026-11-19): 5-tag frames (both CELLs) sat at 2.0-2.9 in
    /** Reject frames that would move the aim point more than this from odometry, inches. */
    public static double MAX_CORRECTION_IN = 12.0; // 8 -> 12 (2026-11-19): real drift reached 8-10 in with tags agreeing
    public static double MAX_SPEED_IN_PER_SEC = 20.0;
    public static double MAX_TURN_DEG_PER_SEC = 45.0;
    /** 0..1: fraction of the way the correction moves toward each clean frame. */
    public static double SMOOTHING = 0.3;
    /** Keep the last correction this long after the last clean frame. */
    public static long HOLD_MS = 300;
    /** Then fade it out over this long (0 = drop at once). */
    public static long FADE_MS = 300;
    /** Odometry moving more than this in one loop is a pose reset (relocalize): the correction restarts. */
    public static double POSE_JUMP_RESET_IN = 3.0;

    /**
     * Threshold check logging (tag TurretVision): every STATS_LOG_MS with new frames, a summary of the
     * frames that had enough tags: how many were used, how many MAX_SPREAD_IN / MAX_CORRECTION_IN
     * rejected, the rejected values, and how many would have passed at 1.5x and 2x the threshold.
     * Each spread / correction rejection is also logged on its own, at most once per REJECT_LOG_MS.
     */
    public static boolean LOG_THRESHOLD_STATS = true;
    public static long STATS_LOG_MS = 10000;
    public static long REJECT_LOG_MS = 1000;

    private static final String TAG = "TurretVision";

    // Threshold stats since the last summary (frames with >= MIN_TAGS of our tags, robot slow).
    private int statCandidates, statUsed, statSpreadRejected, statCorrectionRejected, statFewTags, statNoTags, statMoving;
    private final java.util.List<Double> statSpreadRejectedValues = new java.util.ArrayList<>();
    private final java.util.List<Double> statCorrectionRejectedValues = new java.util.ArrayList<>();
    private double statMaxSpreadUsed = 0, statMaxCorrectionUsed = 0;
    private long statSinceMs = -1, lastRejectLogMs = 0;

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
    private Pose2d lastPose = null;

    public TurretVisionCorrection(LimelightSubsystem limelight, OdometrySubsystem odometry) {
        this.limelight = limelight;
        this.odometry = odometry;
    }

    /** Call once per loop. Returns the robot position to aim from (odometry + correction). */
    public Translation2d update(Pose2d pose, AllianceColors alliance) {
        long now = System.nanoTime() / 1_000_000L;
        updateTurnRate(pose.getHeading(), now);
        // Relocalized: odometry now holds what the correction was adding, so drop the correction.
        if (lastPose != null && lastPose.getTranslation().getDistance(pose.getTranslation()) > POSE_JUMP_RESET_IN) {
            Log.i(TAG, String.format("odometry jumped %.1f in (relocalized): correction reset",
                    lastPose.getTranslation().getDistance(pose.getTranslation())));
            reset("odometry reset");
        }
        lastPose = pose;

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
        if (statSinceMs < 0) statSinceMs = now;
        maybeLogStats(now);
        double speed = odometry.getFieldSpeedInchesPerSecond();
        if (speed > MAX_SPEED_IN_PER_SEC) { reason = "moving too fast"; statMoving++; return; }
        if (turnDegPerSec > MAX_TURN_DEG_PER_SEC) { reason = "turning too fast"; statMoving++; return; }

        LimelightSubsystem.PositionEstimate est = limelight.estimateRobotPosition(pose.getHeading(), alliance);
        if (est == null) { reason = "no tags of our HIVE"; statNoTags++; return; }
        if (est.tagCount < MIN_TAGS) { reason = est.tagCount + " tag(s), need " + MIN_TAGS; statFewTags++; return; }

        double dx = est.position.getX() - pose.getX();
        double dy = est.position.getY() - pose.getY();
        double correction = Math.hypot(dx, dy);
        statCandidates++;
        if (est.spreadIn > MAX_SPREAD_IN) {
            reason = String.format("tags disagree (%.1f in)", est.spreadIn);
            statSpreadRejected++;
            statSpreadRejectedValues.add(est.spreadIn);
            logRejection(now, String.format(java.util.Locale.US, "REJECTED by MAX_SPREAD_IN %.1f: spread %.2f in, %d tags, correction would be %.1f in",
                    MAX_SPREAD_IN, est.spreadIn, est.tagCount, correction));
            return;
        }
        if (correction > MAX_CORRECTION_IN) {
            reason = String.format("too far from odometry (%.1f in)", correction);
            statCorrectionRejected++;
            statCorrectionRejectedValues.add(correction);
            logRejection(now, String.format(java.util.Locale.US, "REJECTED by MAX_CORRECTION_IN %.1f: correction %.2f in (dx %.1f, dy %.1f), %d tags, spread %.2f in",
                    MAX_CORRECTION_IN, correction, dx, dy, est.tagCount, est.spreadIn));
            return;
        }
        statUsed++;
        statMaxSpreadUsed = Math.max(statMaxSpreadUsed, est.spreadIn);
        statMaxCorrectionUsed = Math.max(statMaxCorrectionUsed, correction);

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

    private void logRejection(long now, String line) {
        if (!LOG_THRESHOLD_STATS || now - lastRejectLogMs < REJECT_LOG_MS) return;
        lastRejectLogMs = now;
        Log.i(TAG, line);
    }

    /** Summary of the threshold checks since the last one (see LOG_THRESHOLD_STATS). */
    private void maybeLogStats(long now) {
        if (!LOG_THRESHOLD_STATS || now - statSinceMs < STATS_LOG_MS) return;
        if (statCandidates + statFewTags + statNoTags + statMoving > 0) {
            Log.i(TAG, String.format(java.util.Locale.US,
                    "THRESHOLDS last %.0f s: %d frames with >= %d tags: %d used, %d rejected by spread (> %.1f in): %s, %d rejected by correction (> %.1f in): %s"
                            + " | largest used: spread %.2f in, correction %.2f in | also skipped: %d too few tags, %d none of our tags, %d moving/turning",
                    (now - statSinceMs) / 1000.0, statCandidates, MIN_TAGS, statUsed,
                    statSpreadRejected, MAX_SPREAD_IN, describe(statSpreadRejectedValues, MAX_SPREAD_IN),
                    statCorrectionRejected, MAX_CORRECTION_IN, describe(statCorrectionRejectedValues, MAX_CORRECTION_IN),
                    statMaxSpreadUsed, statMaxCorrectionUsed, statFewTags, statNoTags, statMoving));
        }
        statCandidates = statUsed = statSpreadRejected = statCorrectionRejected = statFewTags = statNoTags = statMoving = 0;
        statSpreadRejectedValues.clear();
        statCorrectionRejectedValues.clear();
        statMaxSpreadUsed = statMaxCorrectionUsed = 0;
        statSinceMs = now;
    }

    /** "min-max, median m; would pass at 1.5x: n, at 2x: n" for rejected values. */
    private static String describe(java.util.List<Double> values, double threshold) {
        if (values.isEmpty()) return "none";
        java.util.List<Double> v = new java.util.ArrayList<>(values);
        java.util.Collections.sort(v);
        int at15 = 0, at2 = 0;
        for (double x : v) {
            if (x <= threshold * 1.5) at15++;
            if (x <= threshold * 2.0) at2++;
        }
        return String.format(java.util.Locale.US, "%.2f-%.2f in, median %.2f; would pass at %.1f in: %d, at %.1f in: %d",
                v.get(0), v.get(v.size() - 1), v.get(v.size() / 2), threshold * 1.5, at15, threshold * 2, at2);
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

    /** Correction currently applied to the aim position, field inches (already weighted). */
    public double getAppliedOffsetX() {
        return weight * offX;
    }

    public double getAppliedOffsetY() {
        return weight * offY;
    }

    /** 1 = full vision correction, 0 = odometry only, in between = fading. */
    public double getWeight() {
        return weight;
    }

    /** Why the last frame was used or rejected. */
    public String getReason() {
        return reason;
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
