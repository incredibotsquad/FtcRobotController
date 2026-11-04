package org.firstinspires.ftc.teamcode.common;

/**
 * Which CELL of each alliance's HIVE is currently facing up (the only valid launch target, G417).
 *
 * Starts at the match-start state from the manual (Fig 9-17 / 10-2): red AUDIENCE CELL up,
 * blue FAR CELL up. Every TIP swaps the up CELL. LimelightSubsystem updates this when it sees a
 * CELL's AprilTags. Static, so it carries over from AUTO to TELEOP like CrossOpModeStorage.
 */
public final class HiveState {

    public static final HiveCell RED_START_UP_CELL = HiveCell.AUDIENCE;
    public static final HiveCell BLUE_START_UP_CELL = HiveCell.FAR;

    private static HiveCell redUpCell = RED_START_UP_CELL;
    private static HiveCell blueUpCell = BLUE_START_UP_CELL;
    private static long redUpdatedAtMs = 0;
    private static long blueUpdatedAtMs = 0;

    private HiveState() {}

    /** Call at the start of AUTO: both HIVES are in their match-start state. */
    public static synchronized void resetToMatchStart() {
        redUpCell = RED_START_UP_CELL;
        blueUpCell = BLUE_START_UP_CELL;
        redUpdatedAtMs = 0;
        blueUpdatedAtMs = 0;
    }

    public static synchronized HiveCell getUpCell(AllianceColors alliance) {
        return alliance == AllianceColors.RED ? redUpCell : blueUpCell;
    }

    /** System time (ms) of the last vision update for this HIVE, or 0 if never seen. */
    public static synchronized long getLastUpdateMs(AllianceColors alliance) {
        return alliance == AllianceColors.RED ? redUpdatedAtMs : blueUpdatedAtMs;
    }

    public static synchronized void setUpCell(AllianceColors alliance, HiveCell upCell, long nowMs) {
        if (alliance == AllianceColors.RED) {
            redUpCell = upCell;
            redUpdatedAtMs = nowMs;
        } else {
            blueUpCell = upCell;
            blueUpdatedAtMs = nowMs;
        }
    }

    public static HiveCell other(HiveCell cell) {
        return cell == HiveCell.FAR ? HiveCell.AUDIENCE : HiveCell.FAR;
    }
}
