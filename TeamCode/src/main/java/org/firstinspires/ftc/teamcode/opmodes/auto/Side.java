package org.firstinspires.ftc.teamcode.opmodes.auto;

import org.firstinspires.ftc.teamcode.common.HiveCell;

/**
 * Left / right as seen by a driver of either alliance, standing behind their alliance wall facing
 * the field. The field is rotationally symmetric, so the same side means the same place relative
 * to your alliance: RIGHT is red's audience-wall side, LEFT red's far-wall side (blue: the reverse,
 * physically). Poses' red-frame values are written for red; blue gets them rotated 180 deg.
 *
 * HiveState and LaunchTargets name CELLS by physical field side (AUDIENCE / FAR); {@link #cell}
 * converts.
 */
public enum Side {
    LEFT, RIGHT;

    /** This side's CELL of our HIVE, by physical field side (red RIGHT = AUDIENCE, blue RIGHT = FAR). */
    public HiveCell cell(boolean isRed) {
        return (this == RIGHT) == isRed ? HiveCell.AUDIENCE : HiveCell.FAR;
    }

    /** The side a physical CELL is on, for this alliance. */
    public static Side of(HiveCell cell, boolean isRed) {
        return (cell == HiveCell.AUDIENCE) == isRed ? RIGHT : LEFT;
    }
}
