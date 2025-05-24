package org.firstinspires.ftc.teamcode;

public class RobotConstants {

    public static final int tickPerRev = 2000;
    public static final double wheelCircumferenceMM = 48 * Math.PI;
    public static final double MMPerTick = wheelCircumferenceMM / tickPerRev;

    public static double HORIZONTAL_CLAW_OPEN = 0.65;
    public static double HORIZONTAL_CLAW_CLOSE = 0.4;
    public static int HORIZONTAL_SLIDE_VELOCITY = 1000;
    public static int VERTICAL_SLIDE_VELOCITY = 1000;

    public static double VERTICAL_CLAW_OPEN = 0.65;
    public static double VERTICAL_CLAW_CLOSE = 0.4;

    public enum GAME_COLORS{
        RED,
        BLUE,
        OTHER
    }

}
