package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.common.HiveCell;

/**
 * Where the robot can stand to launch: two spots per CELL of our HIVE, so alliance partners can
 * shoot at the same CELL without colliding. CELLS are named by field side (AUDIENCE / FAR), as in
 * HiveState and LaunchTargets, so the names are right for both alliances. Positions: {@link Poses}.
 */
public enum LaunchPoint {
    AUDIENCE_CELL_SPOT_1(HiveCell.AUDIENCE, 1),
    AUDIENCE_CELL_SPOT_2(HiveCell.AUDIENCE, 2),
    FAR_CELL_SPOT_1(HiveCell.FAR, 1),
    FAR_CELL_SPOT_2(HiveCell.FAR, 2);

    /** The CELL launched into from this spot. */
    public final HiveCell cell;
    /** 1 or 2. */
    public final int spot;

    LaunchPoint(HiveCell cell, int spot) {
        this.cell = cell;
        this.spot = spot;
    }

    public String label() {
        return cell + " CELL, spot " + spot;
    }
}
