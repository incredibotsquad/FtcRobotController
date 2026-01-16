package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC;

import com.acmerobotics.dashboard.config.Config;
import com.sun.tools.javac.util.List;

/**
 * Lookup-table + interpolation shooter model.
 *
 * - Linear interpolation between the two nearest distance breakpoints
 * - Clamps to safe ranges
 * - Optional visor snap (to avoid servo buzz / over-sensitive interpolation)
 * - Optional distance smoothing hook (pass in a filtered distance if you have it)
 */
@Config
public class LaunchParametersLookup {
    public static boolean USE_OVERRIDE_VALUES = false;
    public static double customRPM = 0;
    public static double customVisor1 = 0;
    public static double customVisor2 = 0;
    public static double customVisor3 = 0;

    // ---- Tune these limits to your robot ----
    private static final double VISOR_MIN = 0.01;
    private static final double VISOR_MAX = 0.70;
    private static final double RPM_MIN   = 0.35;
    private static final double RPM_MAX   = 0.60;

    // If interpolated visor would move more than this from current position,
    // we "snap" to nearest table visor to avoid oscillation/chatter.
    private static final double VISOR_SNAP_THRESHOLD = 0.03;

    // If target visor is within this of current, keep current (deadband).
    private static final double VISOR_DEADBAND = 0.01;

    // RPM interpolation threshold - only interpolate if more than this from nearest reference point
    private static final double RPM_INTERP_THRESHOLD = 0.01;

    /** A single calibrated point. */
    public static class LaunchParameters {
        public final double d;      // distance (inches)
        public final double rpm;    // coefficient 0..1 (or whatever you use)
        public final double visor1;  // servo position 0..1
        public final double visor2;  // servo position 0..1
        public final double visor3;  // servo position 0..1


        public LaunchParameters(double d, double rpm, double visor1, double visor2, double visor3) {
            this.d = d;
            this.rpm = rpm;
            this.visor1 = visor1;
            this.visor2 = visor2;
            this.visor3 = visor3;
        }
    }

    /**
     * Your "YES" points, sorted by distance.
     * Feel free to add more rows as you tune.
     */
    private static final LaunchParameters[] TABLE = new LaunchParameters[] {
        new LaunchParameters(26.0,  0.38, 0.01, 0.01, 0.01),
        new LaunchParameters(28.0,  0.38, 0.01, 0.01, 0.01),
        new LaunchParameters(32.0,  0.40, 0.01, 0.01, 0.01),
        new LaunchParameters(37.0,  0.40, 0.01, 0.01, 0.01),
        new LaunchParameters(42.0,  0.40, 0.01, 0.01, 0.01),
        new LaunchParameters(47.0,  0.40, 0.01, 0.01, 0.01),
        new LaunchParameters(54.0,  0.40, 0.01, 0.01, 0.01),
        new LaunchParameters(62.6,  0.425, 0.01, 0.01, 0.01),
        new LaunchParameters(67.0,  0.425, 0.01, 0.01, 0.01),

        new LaunchParameters(72.0,  0.45, 0.15, 0.15, 0.15),
        new LaunchParameters(76.0,  0.45, 0.15, 0.15, 0.15),
        new LaunchParameters(81.0,  0.45, 0.15, 0.15, 0.15),

        new LaunchParameters(86.0,   0.45, 0.25, 0.25,0.25),
        new LaunchParameters(93.0,   0.47, 0.35, 0.35,0.35),

        new LaunchParameters(127.0,  0.525, 0.69,0.62,0.60),
        new LaunchParameters(129.0,  0.525, 0.69,0.62,0.61),
        new LaunchParameters(133.0,  0.525, 0.67,0.64,0.64),
        new LaunchParameters(137.0,  0.54, 0.67,0.64,0.64),
        new LaunchParameters(141.0,  0.545, 0.66,0.62,0.62),
        new LaunchParameters(147.0,  0.555, 0.68,0.64,0.64),
    };

    /**
     * Main API.
     *
     * @param distanceInches measured distance to goal
     */
    public static BallLaunchParameters getBallLaunchParameters(double distanceInches) {
        if (TABLE.length == 0) throw new IllegalStateException("Shooter table is empty.");

        if (USE_OVERRIDE_VALUES)
            return new BallLaunchParameters(distanceInches, customRPM, customVisor1, customVisor2, customVisor3);

        // Truncate distance to 2 decimal places to reduce fluctuation sensitivity
        double d = Math.floor(distanceInches * 10.0) / 10.0;

        // Clamp distance to table range (no extrapolation).
        d = clamp(d, TABLE[0].d, TABLE[TABLE.length - 1].d);

        // Find bracketing points.
        int hi = upperIndex(d);
        int lo = Math.max(0, hi - 1);

        LaunchParameters a = TABLE[lo];
        LaunchParameters b = TABLE[hi];

        // If exact match or table has duplicates at same distance.
        if (Math.abs(b.d - a.d) < 1e-9) {
            double visor1 = clamp(a.visor1, VISOR_MIN, VISOR_MAX);
            double visor2 = clamp(a.visor2, VISOR_MIN, VISOR_MAX);
            double visor3 = clamp(a.visor3, VISOR_MIN, VISOR_MAX);

            double rpm   = clamp(a.rpm, RPM_MIN, RPM_MAX);

            return new BallLaunchParameters(distanceInches, rpm * FLYWHEEL_FULL_TICKS_PER_SEC, visor1, visor2, visor3);
        }

        // Linear interpolation factor.
        double t = (d - a.d) / (b.d - a.d);

        // Interpolate visor normally
        double visorInterp1 = lerp(a.visor1, b.visor1, t);
        double visorInterp2 = lerp(a.visor2, b.visor2, t);
        double visorInterp3 = lerp(a.visor3, b.visor3, t);

        // For RPM: only interpolate if distance is more than threshold from both reference points
        // Otherwise use the nearest reference point's RPM value
        double distFromLo = Math.abs(d - a.d);
        double distFromHi = Math.abs(d - b.d);
        double rpmResult;

        if (distFromLo <= RPM_INTERP_THRESHOLD) {
            // Close enough to low reference point - use its RPM
            rpmResult = a.rpm;
        } else if (distFromHi <= RPM_INTERP_THRESHOLD) {
            // Close enough to high reference point - use its RPM
            rpmResult = b.rpm;
        } else {
            // Not close to either - interpolate
            rpmResult = lerp(a.rpm, b.rpm, t);
        }

        // Clamp outputs.
        visorInterp1 = clamp(visorInterp1, VISOR_MIN, VISOR_MAX);
        visorInterp2 = clamp(visorInterp2, VISOR_MIN, VISOR_MAX);
        visorInterp3 = clamp(visorInterp3, VISOR_MIN, VISOR_MAX);

        rpmResult   = clamp(rpmResult,   RPM_MIN,   RPM_MAX);

        return new BallLaunchParameters(distanceInches, rpmResult * FLYWHEEL_FULL_TICKS_PER_SEC, visorInterp1, visorInterp2, visorInterp3);
    }

    // ---------------- helpers ----------------

    /** Returns the first index i such that TABLE[i].d >= d. */
    private static int upperIndex(double d) {
        int lo = 0, hi = TABLE.length - 1;
        while (lo < hi) {
            int mid = (lo + hi) >>> 1;
            if (TABLE[mid].d >= d) hi = mid;
            else lo = mid + 1;
        }
        return lo;
    }

    private static double lerp(double a, double b, double t) {
        return a + t * (b - a);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
