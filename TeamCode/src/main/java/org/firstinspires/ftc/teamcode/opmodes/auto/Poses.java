package org.firstinspires.ftc.teamcode.opmodes.auto;

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

    /** Max follower power for auto paths. */
    public static double MAX_SPEED = 0.6;

    /** Robot in the red/audience corner, against both walls, facing the red wall. VERIFY on the field. */
    public final Pose RELOCALIZE_STANCE;

    // ---------------------------------------------------------------- starting poses
    /** Start against the audience wall, on the right of the HIVE (driver's view), facing the red wall. */
    public final Pose STARTING_POSE_RIGHT_HIVE;
    /** Start against the far wall, on the left of the HIVE (driver's view), facing the red wall. */
    public final Pose STARTING_POSE_LEFT_HIVE;

    // ---------------------------------------------------------------- shooting poses
    public final Pose SHOT_ON_LEFT_HIVE;
    public final Pose SHOT_ON_RIGHT_HIVE;

    // ---------------------------------------------------------------- intake poses
    public final Pose INTAKE_GARDEN;
    /** At the rear-wall FLOWER (48, 144). */
    public final Pose INTAKE_FAR_FLOWER;
    /** Bezier control point for the path to INTAKE_FAR_FLOWER (heading unused). */
    public final Pose FAR_FLOWER_CONTROL_POINT;
    /** At the red-wall FLOWER (0, 48). */
    public final Pose INTAKE_CLOSE_FLOWER;
    /** Bezier control points for the path to INTAKE_CLOSE_FLOWER (heading unused). */
    public final Pose INTAKE_CLOSE_FLOWER_CONTROL_1;
    public final Pose INTAKE_CLOSE_FLOWER_CONTROL_2;

    // ---------------------------------------------------------------- park poses (red LOADING ZONE: x 0-11, y 96.5-119.5)
    public final Pose PARK_POSE_LEFT_SIDE;
    public final Pose PARK_POSE_RIGHT_SIDE;

    public Poses(boolean isRed) {
        this.isRed = isRed;

        RELOCALIZE_STANCE = of(8, 6.5, 180);

        STARTING_POSE_RIGHT_HIVE = of(56, 10, 180);
        STARTING_POSE_LEFT_HIVE = of(65, 133, 180);

        SHOT_ON_LEFT_HIVE = of(39.5176, 118.4389, 90);
        SHOT_ON_RIGHT_HIVE = of(56, 10, 180);

        INTAKE_GARDEN = of(12, 10, 180);
        INTAKE_FAR_FLOWER = of(50, 127, 90);
        FAR_FLOWER_CONTROL_POINT = of(54.85, 109.79, 0);
        INTAKE_CLOSE_FLOWER = of(16, 48, 180);
        INTAKE_CLOSE_FLOWER_CONTROL_1 = of(13.46, 93.43, 0);
        INTAKE_CLOSE_FLOWER_CONTROL_2 = of(32.58, 51.36, 0);

        PARK_POSE_LEFT_SIDE = of(8, 115, 270);
        PARK_POSE_RIGHT_SIDE = of(8, 100, 90);
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
