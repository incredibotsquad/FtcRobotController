package org.firstinspires.ftc.teamcode;

public class RobotConstants {

    public static final int tickPerRev = 2000;
    public static final double wheelCircumferenceMM = 48 * Math.PI;
    public static final double MMPerTick = wheelCircumferenceMM / tickPerRev;

    public enum GAME_COLORS{
        RED,
        BLUE,
        OTHER
    }

}
