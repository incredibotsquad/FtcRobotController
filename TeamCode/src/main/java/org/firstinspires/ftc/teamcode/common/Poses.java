package org.firstinspires.ftc.teamcode.common;

import com.pedropathing.math.Pose;

/**
 * Field poses for autonomous paths and relocalization, in Pedro coordinates
 * (inches, 144 x 144, origin at the red/audience corner, +X toward blue, +Y away from the audience).
 *
 * Poses are written once for the RED alliance, with headings in degrees. The BIOBUZZ field has
 * 180 deg ROTATIONAL symmetry about the field center (not mirror symmetry): FLOWERS at (0,48) and
 * (144,96), red LOADING ZONE x 0-11 / y 96.5-119.5 vs blue x 133-144 / y 24.5-47.5, red audience
 * CELL up at start vs blue far CELL up. So blue poses are:
 *     x -> 144 - x,   y -> 144 - y,   heading -> heading + 180
 *
 * Add BIOBUZZ poses here as autonomous paths are designed.
 */
public class Poses {

    private static final double FIELD_SIZE = 144;

    private final boolean isRed;

    /** Robot in the red/audience corner, against both walls, facing the red wall. VERIFY on the field. */
    public final Pose RELOCALIZE_STANCE;

    public Poses(boolean isRed) {
        this.isRed = isRed;

        RELOCALIZE_STANCE = of(9, 9, 180);
    }

    /** Red-alliance pose (heading in degrees); rotated 180 deg about the field center for blue. */
    private Pose of(double x, double y, double headingDeg) {
        if (isRed) {
            return new Pose(x, y, Math.toRadians(normalizeDeg(headingDeg)));
        }
        return new Pose(FIELD_SIZE - x, FIELD_SIZE - y, Math.toRadians(normalizeDeg(headingDeg + 180)));
    }

    /** Wraps to (-180, 180]. */
    private static double normalizeDeg(double deg) {
        deg %= 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }
}
