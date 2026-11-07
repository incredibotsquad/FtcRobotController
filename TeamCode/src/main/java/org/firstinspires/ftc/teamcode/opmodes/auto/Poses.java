package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.common.HiveCell;

/**
 * Field poses for autonomous, in Pedro coordinates (inches, 144 x 144, origin at the red/audience
 * corner, +X toward blue, +Y away from the audience).
 *
 * Every value is written once for the RED alliance as {x, y, headingDeg}. The BIOBUZZ field has
 * 180 deg ROTATIONAL symmetry about the field center (not mirror symmetry), so blue poses are
 *     x -> 144 - x,   y -> 144 - y,   heading -> heading + 180.
 * The rotation moves things from one wall to the opposite one: red's AUDIENCE-wall start becomes a
 * blue FAR-wall pose, and red's AUDIENCE CELL spots land by blue's FAR CELL. The lookups below
 * ({@link #start}, {@link #launch}) take field-side names and handle that.
 *
 * The red values are public static arrays so they can be tuned live in Panels (pit only, R704);
 * a new Poses object picks up the edits. Copy tuned values back into the code.
 *
 * Every value below is a PLACEHOLDER: drive the robot to each spot and read the pose off the
 * odometry telemetry.
 */
@Configurable
public class Poses {

    private static final double FIELD_SIZE = 144;

    // ================================================================ red-frame values {x, y, headingDeg}

    /** Robot in the red/audience corner, against both walls, facing the red wall. Matches RelocalizeCommand. */
    public static double[] RED_RELOCALIZE_STANCE = {8, 6.5, 180};

    // ---- start: touching the audience wall, or the wall opposite it
    public static double[] RED_START_AUDIENCE_WALL = {42.5, 8, 90};
    public static double[] RED_START_FAR_WALL = {65, 133, 180};

    // ---- launch spots, 2 per CELL. Each must be 46-88 in from its CELL (the
    // LaunchReadinessCommand.SHOT_DISTANCES_IN range) and on the CELL's outer side (an up CELL's
    // opening faces away from y = 72). The turret aims, so the heading only has to suit the drive.
    // Red AUDIENCE CELL is at (60, 54), red FAR CELL at (60, 90) (LaunchTargets).
    public static double[] RED_LAUNCH_AUDIENCE_CELL_1 = {42.5, 10, 90};   // ~44.6 in: BELOW the 46 in table minimum
    public static double[] RED_LAUNCH_AUDIENCE_CELL_2 = {30, 24, 90};   // ~53 in
    public static double[] RED_LAUNCH_FAR_CELL_1 = {40, 134, 90};       // ~48 in
    public static double[] RED_LAUNCH_FAR_CELL_2 = {30, 120, 90};       // ~53 in

    // ---- intake points: drive to the approach pose, then straight in to the intake pose
    /** GARDEN (red: tape along the audience wall, x 0-23, 4 POLLEN from the corner). */

    public static double[] RED_GARDEN_APPROACH = {8, 24, 270};
    public static double[] RED_GARDEN_INTAKE = {8, 15, 270};

    /** FLOWER on the alliance wall (red: (0, 48)). */
    public static double[] RED_RIGHT_FLOWER_APPROACH = {24, 48, 180};
    public static double[] RED_RIGHT_FLOWER_INTAKE = {16, 48, 180};

    /** FLOWER on the side wall (red: rear wall, (48, 144)). */
    public static double[] RED_LEFT_FLOWER_APPROACH = {48, 120, 90};
    public static double[] RED_LEFT_FLOWER_INTAKE = {48, 128, 90};

    /** Whether each intake point lowers the intake bar (and backs out to the approach pose before raising it). */
    public static boolean GARDEN_NEEDS_BAR = true;
    public static boolean WALL_FLOWER_NEEDS_BAR = true;
    public static boolean SIDE_FLOWER_NEEDS_BAR = true;

    // ---- park (red LOADING ZONE: x 0-11, y 96.5-119.5; partly inside is enough)
    public static double[] RED_PARK = {10, 108, 90};

    // ---- spline control points, one per destination (heading unused). A spline to a point bends
    // through its control point; the path starts wherever the robot is, so check each on the field.
    public static double[] RED_CONTROL_LAUNCH_AUDIENCE_CELL_1 = {48, 30, 0};
    public static double[] RED_CONTROL_LAUNCH_AUDIENCE_CELL_2 = {30, 36, 0};
    public static double[] RED_CONTROL_LAUNCH_FAR_CELL_1 = {48, 114, 0};
    public static double[] RED_CONTROL_LAUNCH_FAR_CELL_2 = {30, 108, 0};
    public static double[] RED_CONTROL_GARDEN = {40, 24, 0};
    public static double[] RED_CONTROL_WALL_FLOWER = {32.58, 51.36, 0};
    public static double[] RED_CONTROL_SIDE_FLOWER = {54.85, 109.79, 0};
    public static double[] RED_CONTROL_PARK = {24, 108, 0};

    // ================================================================ lookups for one alliance

    private final boolean isRed;

    public Poses(boolean isRed) {
        this.isRed = isRed;
    }

    public Pose relocalizeStance() {
        return of(RED_RELOCALIZE_STANCE);
    }

    /** Start touching the audience wall (true) or the wall opposite it (false). */
    public Pose start(boolean audienceWall) {
        // Rotated, red's FAR-wall start is on blue's audience wall.
        return of(audienceWall == isRed ? RED_START_AUDIENCE_WALL : RED_START_FAR_WALL);
    }

    public Pose launch(LaunchPoint p) {
        if (isRedAudienceCell(p.cell)) {
            return of(p.spot == 1 ? RED_LAUNCH_AUDIENCE_CELL_1 : RED_LAUNCH_AUDIENCE_CELL_2);
        }
        return of(p.spot == 1 ? RED_LAUNCH_FAR_CELL_1 : RED_LAUNCH_FAR_CELL_2);
    }

    public Pose launchControl(LaunchPoint p) {
        if (isRedAudienceCell(p.cell)) {
            return of(p.spot == 1 ? RED_CONTROL_LAUNCH_AUDIENCE_CELL_1 : RED_CONTROL_LAUNCH_AUDIENCE_CELL_2);
        }
        return of(p.spot == 1 ? RED_CONTROL_LAUNCH_FAR_CELL_1 : RED_CONTROL_LAUNCH_FAR_CELL_2);
    }

    public Pose intakeApproach(IntakePoint p) {
        switch (p) {
            case GARDEN: return of(RED_GARDEN_APPROACH);
            case RIGHT_FLOWER: return of(RED_RIGHT_FLOWER_APPROACH);
            default: return of(RED_LEFT_FLOWER_APPROACH);
        }
    }

    public Pose intake(IntakePoint p) {
        switch (p) {
            case GARDEN: return of(RED_GARDEN_INTAKE);
            case RIGHT_FLOWER: return of(RED_RIGHT_FLOWER_INTAKE);
            default: return of(RED_LEFT_FLOWER_INTAKE);
        }
    }

    public Pose intakeControl(IntakePoint p) {
        switch (p) {
            case GARDEN: return of(RED_CONTROL_GARDEN);
            case RIGHT_FLOWER: return of(RED_CONTROL_WALL_FLOWER);
            default: return of(RED_CONTROL_SIDE_FLOWER);
        }
    }

    public Pose park() {
        return of(RED_PARK);
    }

    public Pose parkControl() {
        return of(RED_CONTROL_PARK);
    }

    // ================================================================ helpers

    /** The red-frame AUDIENCE CELL values are this alliance's AUDIENCE CELL for red, FAR CELL for blue. */
    private boolean isRedAudienceCell(HiveCell cell) {
        return (cell == HiveCell.AUDIENCE) == isRed;
    }

    /** Red-alliance {x, y, headingDeg}; rotated 180 deg about the field center for blue. */
    private Pose of(double[] p) {
        double x = p[0], y = p[1], headingDeg = p[2];
        if (!isRed) {
            x = FIELD_SIZE - x;
            y = FIELD_SIZE - y;
            headingDeg += 180;
        }
        return new Pose(x, y, Math.toRadians(normalizeDeg(headingDeg)));
    }

    /** Wraps to (-180, 180]. */
    private static double normalizeDeg(double deg) {
        deg %= 360.0;
        if (deg > 180.0) deg -= 360.0;
        if (deg <= -180.0) deg += 360.0;
        return deg;
    }
}
