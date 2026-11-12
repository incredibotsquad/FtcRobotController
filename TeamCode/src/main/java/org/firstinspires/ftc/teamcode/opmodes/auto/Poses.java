package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;


/**
 * Field poses for autonomous, in Pedro coordinates (inches, 144 x 144, origin at the red/audience
 * corner, +X toward blue, +Y away from the audience).
 *
 * Every value is written once for the RED alliance as {x, y, headingDeg}. The BIOBUZZ field has
 * 180 deg ROTATIONAL symmetry about the field center (not mirror symmetry), so blue poses are
 *     x -> 144 - x,   y -> 144 - y,   heading -> heading + 180.
 * The lookups below take the driver's LEFT / RIGHT ({@link Side}): the same side is the same place
 * relative to your alliance, so one plan works for both. RIGHT = the red values named AUDIENCE
 * (red's audience-wall side), LEFT = the ones named FAR. (Physically, blue's RIGHT is at the far
 * wall: the rotation moves things to the opposite wall.)
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
    public static double[] RED_START_AUDIENCE_WALL = {41.5, 8, 90};
    public static double[] RED_START_FAR_WALL = {30.5, 136, 270};

    // ---- launch spots, 2 per CELL. Each must be 46-88 in from its CELL (the
    // LaunchReadinessCommand.SHOT_DISTANCES_IN range) and on the CELL's outer side (an up CELL's
    // opening faces away from y = 72). The turret aims, so the heading only has to suit the drive.
    // Red AUDIENCE CELL is at (60, 54), red FAR CELL at (60, 90) (LaunchTargets).
    public static double[] RED_LAUNCH_AUDIENCE_CELL_1 = {42.5, 10, 90};   // ~48.75 in (shot row 48.75 = 2668 RPM)
    public static double[] RED_LAUNCH_AUDIENCE_CELL_2 = {30, 24, 90};   // ~53 in
    public static double[] RED_LAUNCH_FAR_CELL_1 = {40, 134, 270};       // ~48 in
    public static double[] RED_LAUNCH_FAR_CELL_2 = {30.5, 120, 270};       // ~53 in
    // LEFT spot 3 has no array of its own: x, y from RED_LEFT_FLOWER_APPROACH, heading from
    // RED_LAUNCH_FAR_CELL_2 (faces the HIVE so the Limelight can see it; tune those two). ~32 in from
    // the CELL, closer than the 46 in shot table, so the launcher uses the table's nearest RPM.
    /** LEFT spot 4: DynamicAuto's final launch, after the last intake point (straight line, linear heading). */
    public static double[] RED_LAUNCH_FAR_CELL_4 = {16, 116, 315};       // ~51 in

    // ---- intake points: drive to the approach pose, then straight in to the intake pose
    /** GARDEN (red: tape along the audience wall, x 0-23, 4 POLLEN from the corner). */

    public static double[] RED_GARDEN_APPROACH = {8, 24, 270};
    public static double[] RED_GARDEN_INTAKE = {8, 15, 270};

    /** FLOWER on the alliance wall (red: (0, 48)). */
    public static double[] RED_RIGHT_FLOWER_APPROACH = {24, 48, 180};
    public static double[] RED_RIGHT_FLOWER_INTAKE = {14, 48, 180};

    /** FLOWER on the side wall (red: rear wall, (48, 144)). */
    public static double[] RED_LEFT_FLOWER_APPROACH = {48, 120, 90};
    public static double[] RED_LEFT_FLOWER_INTAKE = {48, 130, 90};

    /** Whether each intake point lowers the intake bar (and backs out to the approach pose before raising it). */
    public static boolean GARDEN_NEEDS_BAR = true;
    public static boolean WALL_FLOWER_NEEDS_BAR = true;
    public static boolean SIDE_FLOWER_NEEDS_BAR = true;

    // ---- park (red LOADING ZONE: x 0-11, y 96.5-119.5; partly inside is enough, don't touch the
    // wall or LEAVE may not count). Two spots, named by the wall they're toward:
    /** LEFT: the end of the zone toward the wall opposite the audience (red: high y). */
    public static double[] RED_PARK_LEFT = {10, 114, 90};
    /** RIGHT: the end of the zone toward the audience wall (red: low y). */
    public static double[] RED_PARK_RIGHT = {10, 96, 270};

    // ---- spline control points, one per destination (heading unused). A spline to a point bends
    // through its control point; the path starts wherever the robot is, so check each on the field.
    public static double[] RED_CONTROL_LAUNCH_AUDIENCE_CELL_1 = {48, 30, 0};
    public static double[] RED_CONTROL_LAUNCH_AUDIENCE_CELL_2 = {30, 36, 0};
    public static double[] RED_CONTROL_LAUNCH_FAR_CELL_1 = {48, 114, 0};
    public static double[] RED_CONTROL_LAUNCH_FAR_CELL_2 = {30, 108, 0};
    /** Only used if LEFT spot 4 is added to a plan as a spline; the automatic final launch drives a straight line. */
    public static double[] RED_CONTROL_LAUNCH_FAR_CELL_4 = {24, 108, 0};
    public static double[] RED_CONTROL_GARDEN = {40, 24, 0};
    public static double[] RED_CONTROL_WALL_FLOWER = {2.58, 51.36, 0}; // was {32.58, 51.36}: x -30, the path from LEFT spot 3 hit the HIVE
    public static double[] RED_CONTROL_SIDE_FLOWER = {54.85, 109.79, 0};
    /**
     * Moving between launch points on opposite sides of the HIVE (audience-side CELL <-> far-side
     * CELL), from any spot to any spot, picked in INIT. AROUND: a spline through this control
     * point, past the HIVE on the alliance-wall side.
     */
    public static double[] RED_CONTROL_AROUND_HIVE = {16, 88, 0}; // was {24, 72}: x -8, y +16, away from the HIVE
    /**
     * THROUGH: straight lines via these two waypoints {x, y} (the robot drives to each one), under
     * the HIVE between them. Used in travel order, the one nearer the start first, so it works in
     * both directions. Unlike every other pose, blue MIRRORS these across the field's center line
     * x = 72 (x -> 144 - x, y unchanged) instead of rotating them.
     */
    public static double[] RED_THROUGH_HIVE_WAYPOINT_AUDIENCE = {60, 30};
    public static double[] RED_THROUGH_HIVE_WAYPOINT_FAR = {60, 120};
    public static double[] RED_CONTROL_PARK_LEFT = {24, 114, 0};
    public static double[] RED_CONTROL_PARK_RIGHT = {24, 102, 0};

    // ================================================================ lookups for one alliance

    private final boolean isRed;

    public Poses(boolean isRed) {
        this.isRed = isRed;
    }

    public Pose relocalizeStance() {
        return of(RED_RELOCALIZE_STANCE);
    }

    /** Start on the driver's RIGHT (red: RED_START_AUDIENCE_WALL) or LEFT (RED_START_FAR_WALL). */
    public Pose start(Side side) {
        return of(side == Side.RIGHT ? RED_START_AUDIENCE_WALL : RED_START_FAR_WALL);
    }

    /** RIGHT spots are the RED_LAUNCH_AUDIENCE_CELL_* values, LEFT spots RED_LAUNCH_FAR_CELL_*. */
    public Pose launch(LaunchPoint p) {
        if (p.side == Side.RIGHT) {
            return of(p.spot == 1 ? RED_LAUNCH_AUDIENCE_CELL_1 : RED_LAUNCH_AUDIENCE_CELL_2);
        }
        switch (p.spot) {
            case 1: return of(RED_LAUNCH_FAR_CELL_1);
            case 2: return of(RED_LAUNCH_FAR_CELL_2);
            case 3: return of(launchFarCell3()); // rear-wall FLOWER approach x, y; LEFT spot 2 heading
            default: return of(RED_LAUNCH_FAR_CELL_4);
        }
    }

    /** LEFT spot 3 {x, y, headingDeg}: the rear-wall FLOWER approach position, facing like LEFT spot 2. */
    private static double[] launchFarCell3() {
        return new double[] {RED_LEFT_FLOWER_APPROACH[0], RED_LEFT_FLOWER_APPROACH[1], RED_LAUNCH_FAR_CELL_2[2]};
    }

    public Pose launchControl(LaunchPoint p) {
        if (p.side == Side.RIGHT) {
            return of(p.spot == 1 ? RED_CONTROL_LAUNCH_AUDIENCE_CELL_1 : RED_CONTROL_LAUNCH_AUDIENCE_CELL_2);
        }
        switch (p.spot) {
            case 1: return of(RED_CONTROL_LAUNCH_FAR_CELL_1);
            case 2: return of(RED_CONTROL_LAUNCH_FAR_CELL_2);
            case 3: return of(RED_CONTROL_SIDE_FLOWER);
            default: return of(RED_CONTROL_LAUNCH_FAR_CELL_4);
        }
    }

    /** Spline control point for a move between launch points AROUND the HIVE. */
    public Pose hiveAroundControl() {
        return of(RED_CONTROL_AROUND_HIVE);
    }

    /**
     * Waypoints for a move from {@code start} THROUGH the HIVE, nearer-to-start first, each with
     * heading {@code headingRad}. Blue mirrors them across x = 72 (see RED_THROUGH_HIVE_WAYPOINT_*).
     */
    public Pose[] hiveThroughWaypoints(Pose start, double headingRad) {
        Pose a = mirrored(RED_THROUGH_HIVE_WAYPOINT_AUDIENCE, headingRad);
        Pose b = mirrored(RED_THROUGH_HIVE_WAYPOINT_FAR, headingRad);
        return start.distance(a) <= start.distance(b) ? new Pose[] {a, b} : new Pose[] {b, a};
    }

    /** {x, y} in the red frame; blue mirrors x across the field's center line (y unchanged). */
    private Pose mirrored(double[] p, double headingRad) {
        return new Pose(isRed ? p[0] : FIELD_SIZE - p[0], p[1], headingRad);
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

    /** Park at the driver's RIGHT end of the LOADING ZONE (RED_PARK_RIGHT) or LEFT end (RED_PARK_LEFT). */
    public Pose park(Side side) {
        return of(side == Side.RIGHT ? RED_PARK_RIGHT : RED_PARK_LEFT);
    }

    public Pose parkControl(Side side) {
        return of(side == Side.RIGHT ? RED_CONTROL_PARK_RIGHT : RED_CONTROL_PARK_LEFT);
    }

    // ================================================================ helpers

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
