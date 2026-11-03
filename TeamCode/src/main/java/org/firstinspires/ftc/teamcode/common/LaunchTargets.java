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
 * side, about 18.8" apart, so each CELL is CELL_Y_OFFSET either side of center.
 *
 * Which CELL to aim at is decided by a {@link Selector}. The default aims at the nearest
 * of the alliance's two CELLS. A Limelight-based selector can be swapped in later with
 * {@link #setSelector(Selector)} without changing any callers.
 */
@Configurable
public class LaunchTargets {

    public static double FIELD_CENTER_Y = 72.0;

    // PLACEHOLDERS: measure each HIVE's X position from the field CAD / AprilTag reference holes.
    public static double RED_HIVE_X = 60.0;
    public static double BLUE_HIVE_X = 84.0;

    /** Half the 18.8" CELL spacing. */
    public static double CELL_Y_OFFSET = 9.4;

    /**
     * The other CELL must be at least this much closer before the target switches.
     * Stops the turret swinging back and forth when the robot is about equidistant.
     */
    public static double SWITCH_HYSTERESIS_IN = 6.0;

    /** Decides which of an alliance's two CELLS to aim at. */
    public interface Selector {
        HiveCell select(Pose2d robotPose, AllianceColors alliance);
    }

    private static Selector selector = new NearestCellSelector();
    private static HiveCell lastSelectedCell = HiveCell.FAR;

    private LaunchTargets() {}

    public static void setSelector(Selector newSelector) {
        selector = newSelector;
    }

    /** Target point for the given robot pose. Call every loop from the aiming code. */
    public static Translation2d getTarget(Pose2d robotPose, AllianceColors alliance) {
        lastSelectedCell = selector.select(robotPose, alliance);
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
