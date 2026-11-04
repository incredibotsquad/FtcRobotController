package org.firstinspires.ftc.teamcode.common;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.bylazar.configurables.annotations.Configurable;

/**
 * Launch target positions for the HIVE CELLS, and the logic that picks which CELL to aim at.
 *
 * Field frame (inches, 144 x 144):
 *   Pedro Pathing coordinates: red is on the low-X side, blue on the high-X side.
 *   Y is assumed to run from the audience wall (0) to the far wall (144).
 * Each HIVE sits near field center with one CELL on the audience side and one on the far
 * side. The manual's "18.8 in apart" is the GAP between the CELLS, not center to center.
 * The up-facing CELL's opening faces outward (away from y=72) and 30 deg up, with its
 * center about CELL_Y_OFFSET from field center at roughly 58 in high (manual Fig 9-9 to 9-11).
 *
 * Which CELL to aim at is decided by a {@link Selector}, picked by SELECTOR:
 *  - NEAREST:       {@link NearestCellSelector}, the nearest of the alliance's two CELLS (odometry only).
 *  - VISION_HEIGHT: {@link HiveStateSelector}, the CELL HiveState says is up. LimelightSubsystem keeps
 *                   HiveState current from the AprilTag heights; until a HIVE is seen it holds the
 *                   match-start state.
 *  - VISION_ROLL:   {@link RollStateSelector}, the CELL HiveState's roll-based state says is up.
 *                   LimelightSubsystem only runs the roll method while this selector is active.
 * All selectors always exist and keep their own state, so changing SELECTOR (also live in
 * Panels) switches cleanly with no other changes.
 */
@Configurable
public class LaunchTargets {

    public static double FIELD_CENTER_Y = 72.0;

    // PLACEHOLDERS: measure each HIVE's X position from the field CAD / AprilTag reference holes.
    public static double RED_HIVE_X = 60.0;
    public static double BLUE_HIVE_X = 84.0;

    /**
     * Y distance from field center to the center of the up-facing CELL's opening.
     * DERIVED from the manual drawings (~+/-2 in): the opening spans about 12.3-19.3 in from
     * y=72 (center 16.5 in), and the CELL box center is about 11.3 in. Verify against the field CAD.
     */
    public static double CELL_Y_OFFSET = 16.5;

    /**
     * The other CELL must be at least this much closer before the target switches.
     * Stops the turret swinging back and forth when the robot is about equidistant.
     */
    public static double SWITCH_HYSTERESIS_IN = 6.0;

    /** How the target CELL is picked. */
    public enum SelectorMode {
        /** Nearest CELL (odometry only). */
        NEAREST,
        /** Up CELL from the AprilTag heights (HiveState.getUpCell). */
        VISION_HEIGHT,
        /** Up CELL from the AprilTag roll angles (HiveState.getRollUpCell). */
        VISION_ROLL
    }

    public static SelectorMode SELECTOR = SelectorMode.NEAREST;

    /** Decides which of an alliance's two CELLS to aim at. */
    public interface Selector {
        HiveCell select(Pose2d robotPose, AllianceColors alliance);
    }

    private static final Selector nearestSelector = new NearestCellSelector();
    private static final Selector visionSelector = new HiveStateSelector();
    private static final Selector rollSelector = new RollStateSelector();
    private static HiveCell lastSelectedCell = HiveCell.FAR;

    private LaunchTargets() {}

    private static Selector activeSelector() {
        switch (SELECTOR) {
            case VISION_HEIGHT: return visionSelector;
            case VISION_ROLL:   return rollSelector;
            default:            return nearestSelector;
        }
    }

    /** True while the roll selector is active; LimelightSubsystem only runs the roll method then. */
    public static boolean isRollSelectorActive() {
        return SELECTOR == SelectorMode.VISION_ROLL;
    }

    /** Target point for the given robot pose. Call every loop from the aiming code. */
    public static Translation2d getTarget(Pose2d robotPose, AllianceColors alliance) {
        lastSelectedCell = activeSelector().select(robotPose, alliance);
        return getCellPosition(alliance, lastSelectedCell);
    }

    /** CELL chosen by the most recent getTarget() call (for telemetry / lights). */
    public static HiveCell getLastSelectedCell() {
        return lastSelectedCell;
    }

    public static Translation2d getCellPosition(AllianceColors alliance, HiveCell cell) {
        double x = alliance == AllianceColors.RED ? RED_HIVE_X : BLUE_HIVE_X;
        double y = cell == HiveCell.FAR
                ? FIELD_CENTER_Y + CELL_Y_OFFSET
                : FIELD_CENTER_Y - CELL_Y_OFFSET;
        return new Translation2d(x, y);
    }

    /** Aims at the alliance's up-facing CELL as tracked by HiveState (Limelight). */
    public static class HiveStateSelector implements Selector {
        @Override
        public HiveCell select(Pose2d robotPose, AllianceColors alliance) {
            return HiveState.getUpCell(alliance);
        }
    }

    /** Aims at the alliance's up-facing CELL as tracked by the roll method (HiveState roll state). */
    public static class RollStateSelector implements Selector {
        @Override
        public HiveCell select(Pose2d robotPose, AllianceColors alliance) {
            return HiveState.getRollUpCell(alliance);
        }
    }

    /** Aims at whichever of the alliance's CELLS is closer, with hysteresis. */
    public static class NearestCellSelector implements Selector {
        private HiveCell current = null;

        @Override
        public HiveCell select(Pose2d robotPose, AllianceColors alliance) {
            Translation2d robot = robotPose.getTranslation();
            double toFar = robot.getDistance(getCellPosition(alliance, HiveCell.FAR));
            double toAudience = robot.getDistance(getCellPosition(alliance, HiveCell.AUDIENCE));

            if (current == null) {
                current = toFar <= toAudience ? HiveCell.FAR : HiveCell.AUDIENCE;
            } else if (current == HiveCell.FAR && toAudience < toFar - SWITCH_HYSTERESIS_IN) {
                current = HiveCell.AUDIENCE;
            } else if (current == HiveCell.AUDIENCE && toFar < toAudience - SWITCH_HYSTERESIS_IN) {
                current = HiveCell.FAR;
            }
            return current;
        }
    }
}
