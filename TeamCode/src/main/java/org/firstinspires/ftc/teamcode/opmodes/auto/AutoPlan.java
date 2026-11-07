package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.common.AllianceColors;

import java.util.ArrayList;
import java.util.List;

/**
 * The configurable part of {@link DynamicAuto}, edited on the Driver Station during INIT:
 * alliance, start wall, the sequence of launch points and the sequence of intake points, each
 * with how to drive there (straight line or spline).
 *
 * Static, so a plan survives between OpMode runs until the Robot Controller app restarts.
 */
public final class AutoPlan {

    /** A point in a sequence plus how the robot drives to it, and how long to wait after it. */
    public static final class Leg<T> {
        public final T point;
        public final boolean spline;
        /** Wait this long after this step (launch point: after each launch there; intake point: after intaking). */
        public long delayMs = 0;

        Leg(T point, boolean spline) {
            this.point = point;
            this.spline = spline;
        }

        public String moveLabel() {
            return (spline ? "spline" : "line") + (delayMs > 0 ? String.format(", then wait %.1f s", delayMs / 1000.0) : "");
        }
    }

    /** Delay steps and limit for the INIT editor. */
    public static final long DELAY_STEP_MS = 500;
    public static final long MAX_DELAY_MS = 15000;

    public static AllianceColors alliance = AllianceColors.RED;
    /** Wait this long after START before the first move. */
    public static long startDelayMs = 0;
    /** true = start touching the audience wall, false = the wall opposite it. */
    public static boolean startAtAudienceWall = true;
    /** Park spot in the LOADING ZONE: true = RIGHT (toward the audience wall), false = LEFT (toward the opposite wall). */
    public static boolean parkTowardAudience = false;

    /** Moving between launch points across the HIVE: true = AROUND it, false = THROUGH it. */
    public static boolean hiveRouteAround = true;

    public static String hiveRouteLabel() {
        return hiveRouteAround ? "AROUND the HIVE" : "THROUGH the HIVE";
    }

    public static String parkLabel() {
        return parkTowardAudience ? "RIGHT (toward audience wall)" : "LEFT (toward far wall)";
    }

    public static final List<Leg<LaunchPoint>> launchPoints = new ArrayList<>();
    public static final List<Leg<IntakePoint>> intakePoints = new ArrayList<>();

    private AutoPlan() {}

    public static boolean isRed() {
        return alliance == AllianceColors.RED;
    }

    public static void addLaunch(LaunchPoint point, boolean spline) {
        launchPoints.add(new Leg<>(point, spline));
    }

    public static void addIntake(IntakePoint point, boolean spline) {
        intakePoints.add(new Leg<>(point, spline));
    }

    /** Changes the delay after the last point of {@code list} by {@code steps} DELAY_STEP_MS (no-op if empty). */
    static <T> void adjustLastDelay(List<Leg<T>> list, int steps) {
        if (list.isEmpty() || steps == 0) return;
        Leg<T> last = list.get(list.size() - 1);
        last.delayMs = clampDelay(last.delayMs + steps * DELAY_STEP_MS);
    }

    static long clampDelay(long ms) {
        return Math.max(0, Math.min(MAX_DELAY_MS, ms));
    }

    static <T> void removeLast(List<T> list) {
        if (!list.isEmpty()) list.remove(list.size() - 1);
    }
}
