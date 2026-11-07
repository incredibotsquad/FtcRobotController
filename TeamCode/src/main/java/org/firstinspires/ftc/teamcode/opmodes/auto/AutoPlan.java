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

    /** A point in a sequence plus how the robot drives to it. */
    public static final class Leg<T> {
        public final T point;
        public final boolean spline;

        Leg(T point, boolean spline) {
            this.point = point;
            this.spline = spline;
        }

        public String moveLabel() {
            return spline ? "spline" : "line";
        }
    }

    public static AllianceColors alliance = AllianceColors.RED;
    /** true = start touching the audience wall, false = the wall opposite it. */
    public static boolean startAtAudienceWall = true;

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

    static <T> void removeLast(List<T> list) {
        if (!list.isEmpty()) list.remove(list.size() - 1);
    }
}
