package org.firstinspires.ftc.teamcode.common;

import com.bylazar.configurables.annotations.Configurable;

/**
 * Field geometry of the HIVE AprilTags (36h11, 3.25 in), manual section 9.9.
 *
 * Each CELL has a cluster of 4 tags on its bottom face, facing down and tilted 30 deg. The tags
 * move when the HIVE TIPS, so a tag's field position depends on whether its CELL is up or down.
 * The two states differ by about 14 in of height, which is how the state is told apart.
 *
 * Y/Z values are DERIVED from the manual drawings (about +/-1.5 in). Measure them on a real
 * field or from the CAD and update here (all live-tunable in Panels).
 *
 * Field frame: inches, origin at the red/audience corner, +X toward blue, +Y away from the audience.
 */
@Configurable
public final class HiveTags {

    public static final int FIRST_ID = 30;
    public static final int LAST_ID = 45;

    /** Tag-center X offsets from the HIVE center, ordered -X to +X (Fig 9-15). */
    public static double[] TAG_X_OFFSETS_IN = {-6.5, -2.75, 2.75, 6.5};

    /** Up-facing CELL: tag centers this far from y = 72, at this height. */
    public static double UP_TAG_Y_OFFSET_IN = 13.1;
    public static double UP_TAG_Z_IN = 49.9;

    /** Down-facing CELL: tag centers this far from y = 72, at this height. */
    public static double DOWN_TAG_Y_OFFSET_IN = 11.7;
    public static double DOWN_TAG_Z_IN = 35.6;

    private HiveTags() {}

    public static boolean isHiveTag(int id) {
        return id >= FIRST_ID && id <= LAST_ID;
    }

    /** 30-37 are red, 38-45 blue. */
    public static AllianceColors allianceOf(int id) {
        return id <= 37 ? AllianceColors.RED : AllianceColors.BLUE;
    }

    /** 30-33 red far, 34-37 red audience, 38-41 blue audience, 42-45 blue far. */
    public static HiveCell cellOf(int id) {
        return (id >= 34 && id <= 41) ? HiveCell.AUDIENCE : HiveCell.FAR;
    }

    /** Height threshold between the up and down states. */
    public static double upDownThresholdZ() {
        return (UP_TAG_Z_IN + DOWN_TAG_Z_IN) / 2.0;
    }

    public static double fieldX(int id) {
        double hiveX = allianceOf(id) == AllianceColors.RED ? LaunchTargets.RED_HIVE_X : LaunchTargets.BLUE_HIVE_X;
        int base = id - (((id - FIRST_ID) / 4) * 4 + FIRST_ID); // 0..3 within the cluster
        // Audience clusters read ascending in +X; far clusters are mounted rotated 180 deg, so descending.
        int index = cellOf(id) == HiveCell.AUDIENCE ? base : 3 - base;
        return hiveX + TAG_X_OFFSETS_IN[index];
    }

    public static double fieldY(int id, boolean cellUp) {
        double offset = cellUp ? UP_TAG_Y_OFFSET_IN : DOWN_TAG_Y_OFFSET_IN;
        return cellOf(id) == HiveCell.FAR
                ? LaunchTargets.FIELD_CENTER_Y + offset
                : LaunchTargets.FIELD_CENTER_Y - offset;
    }

    public static double fieldZ(boolean cellUp) {
        return cellUp ? UP_TAG_Z_IN : DOWN_TAG_Z_IN;
    }
}
