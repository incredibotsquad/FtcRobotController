package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.teamcode.Actions.LaunchFlywheelAction.FLYWHEEL_FULL_TICKS_PER_SEC;

import com.sun.tools.javac.util.List;

/**
 * Lookup-table + interpolation shooter model.
 *
 * - Linear interpolation between the two nearest distance breakpoints
 * - Clamps to safe ranges
 * - Optional visor snap (to avoid servo buzz / over-sensitive interpolation)
 * - Optional distance smoothing hook (pass in a filtered distance if you have it)
 */
public class LaunchParametersLookup {

    // ---- Tune these limits to your robot ----
    private static final double VISOR_MIN = 0.01;
    private static final double VISOR_MAX = 0.67;
    private static final double RPM_MIN   = 0.38;
    private static final double RPM_MAX   = 0.55;

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
        public final double visor;  // servo position 0..1
        public final double rpm;    // coefficient 0..1 (or whatever you use)

        public LaunchParameters(double d, double visor, double rpm) {
            this.d = d;
            this.visor = visor;
            this.rpm = rpm;
        }
    }

    /**
     * Your "YES" points, sorted by distance.
     * Feel free to add more rows as you tune.
     */
    private static final LaunchParameters[] TABLE = new LaunchParameters[] {
        new LaunchParameters(26.0,  0.01, 0.38),
        new LaunchParameters(32.0,  0.01, 0.40),
        new LaunchParameters(37.0,  0.01, 0.40),
        new LaunchParameters(42.0,  0.01, 0.40),
        new LaunchParameters(47.0,  0.01, 0.40),
        new LaunchParameters(54.0,  0.01, 0.40),
        new LaunchParameters(62.6,  0.01, 0.425),
        new LaunchParameters(67.0,  0.01, 0.425),

        new LaunchParameters(72.0,  0.15, 0.45),
        new LaunchParameters(76.0,  0.15, 0.45),
        new LaunchParameters(81.0,  0.15, 0.45),

        new LaunchParameters(86.0,  0.25, 0.45),
        new LaunchParameters(93.0,  0.35, 0.47),

        new LaunchParameters(123.0, 0.60, 0.52),
        new LaunchParameters(130.0, 0.65, 0.525),
        new LaunchParameters(136.0, 0.65, 0.53),
        new LaunchParameters(140.0, 0.67, 0.54),
        new LaunchParameters(150.0, 0.67, 0.55),
    };

    /**
     * Main API.
     *
     * @param distanceInches measured distance to goal
     */
    public static BallLaunchParameters getBallLaunchParameters(double distanceInches) {
        if (TABLE.length == 0) throw new IllegalStateException("Shooter table is empty.");

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
            double visor = clamp(a.visor, VISOR_MIN, VISOR_MAX);
            double rpm   = clamp(a.rpm, RPM_MIN, RPM_MAX);
            return new BallLaunchParameters(distanceInches, rpm * FLYWHEEL_FULL_TICKS_PER_SEC, visor, visor, visor);
        }

        // Linear interpolation factor.
        double t = (d - a.d) / (b.d - a.d);

        // Interpolate visor normally
        double visorInterp = lerp(a.visor, b.visor, t);

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
        visorInterp = clamp(visorInterp, VISOR_MIN, VISOR_MAX);
        rpmResult   = clamp(rpmResult,   RPM_MIN,   RPM_MAX);

        return new BallLaunchParameters(distanceInches, rpmResult * FLYWHEEL_FULL_TICKS_PER_SEC, visorInterp, visorInterp, visorInterp);
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
