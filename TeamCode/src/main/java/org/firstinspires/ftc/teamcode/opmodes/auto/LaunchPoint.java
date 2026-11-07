package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.common.HiveCell;

/**
 * Where the robot can stand to launch: two spots on each side (driver's LEFT / RIGHT, see
 * {@link Side}), each launching into that side's CELL of our HIVE, so alliance partners can shoot
 * at the same CELL without colliding. Positions: {@link Poses} (red frame: RIGHT = the
 * RED_LAUNCH_AUDIENCE_CELL_* values, LEFT = RED_LAUNCH_FAR_CELL_*).
 */
public enum LaunchPoint {
    LEFT_SPOT_1(Side.LEFT, 1),
    LEFT_SPOT_2(Side.LEFT, 2),
    RIGHT_SPOT_1(Side.RIGHT, 1),
    RIGHT_SPOT_2(Side.RIGHT, 2);

    public final Side side;
    /** 1 or 2. */
    public final int spot;

    LaunchPoint(Side side, int spot) {
        this.side = side;
        this.spot = spot;
    }

    /** The CELL launched into from here, by physical field side (for HiveState / LaunchTargets). */
    public HiveCell cell(boolean isRed) {
        return side.cell(isRed);
    }

    public String label() {
        return side + " spot " + spot;
    }
}
