package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.common.HiveCell;

/**
 * Where the robot can stand to launch: two spots on each side (driver's LEFT / RIGHT, see
 * {@link Side}), each launching into that side's CELL of our HIVE, so alliance partners can shoot
 * at the same CELL without colliding. Positions: {@link Poses} (red frame: RIGHT = the
 * RED_LAUNCH_AUDIENCE_CELL_* values, LEFT = RED_LAUNCH_FAR_CELL_*).
 *
 * LEFT_SPOT_3 is the final-launch spot (partly in the LOADING ZONE): DynamicAuto drives there in a
 * straight line after the last intake point (DynamicAuto.FINAL_LAUNCH_AT_LEFT_SPOT_3). It can also be
 * added to a plan in INIT. (Until 2026-11-20 this was LEFT spot 4; the old LEFT spot 3, at the rear-wall
 * FLOWER's approach position, was unused and removed.)
 */
public enum LaunchPoint {
    LEFT_SPOT_1(Side.LEFT, 1),
    LEFT_SPOT_2(Side.LEFT, 2),
    LEFT_SPOT_3(Side.LEFT, 3),
    RIGHT_SPOT_1(Side.RIGHT, 1),
    RIGHT_SPOT_2(Side.RIGHT, 2);

    public final Side side;
    /** 1 to 3 (3 is LEFT only). */
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
