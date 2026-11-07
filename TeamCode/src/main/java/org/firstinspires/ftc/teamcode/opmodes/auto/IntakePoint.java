package org.firstinspires.ftc.teamcode.opmodes.auto;

/**
 * Where the robot can pick up POLLEN on our side of the field. Positions: {@link Poses}.
 * Whether a point needs the intake bar is part of the point ({@link #needsBar()}).
 */
public enum IntakePoint {
    /** Our GARDEN: the tape line with 4 POLLEN (red: audience wall, blue: rear wall). */
    GARDEN,
    /** The FLOWER on our alliance wall (red: (0, 48), blue: (144, 96)). */
    RIGHT_FLOWER,
    /** The FLOWER on the side wall on our half (red: rear wall (48, 144), blue: audience wall (96, 0)). */
    LEFT_FLOWER;

    /** Whether this point lowers the intake bar (set per point in Poses, live in Panels). */
    public boolean needsBar() {
        switch (this) {
            case GARDEN: return Poses.GARDEN_NEEDS_BAR;
            case RIGHT_FLOWER: return Poses.WALL_FLOWER_NEEDS_BAR;
            default: return Poses.SIDE_FLOWER_NEEDS_BAR;
        }
    }

    public String label(boolean isRed) {
        switch (this) {
            case GARDEN: return "GARDEN line";
            case RIGHT_FLOWER: return "FLOWER on alliance wall";
            default: return "FLOWER on " + (isRed ? "rear" : "audience") + " wall";
        }
    }
}
