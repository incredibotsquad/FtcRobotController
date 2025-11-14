package org.firstinspires.ftc.teamcode.common;

import java.util.List;

public class AprilTagConstants {
    public static int RED_ALLIANCE_TAG_ID = 24;
    public static int BLUE_ALLIANCE_TAG_ID = 20;
    public static List<GamePattern> patterns = List.of(
            new GamePattern(21, List.of(GameColors.GREEN, GameColors.PURPLE, GameColors.PURPLE)),
            new GamePattern(22, List.of(GameColors.PURPLE, GameColors.GREEN, GameColors.PURPLE)),
            new GamePattern(23, List.of(GameColors.PURPLE, GameColors.PURPLE, GameColors.GREEN)));
}
