package org.firstinspires.ftc.teamcode.common;

public class BallEntry {
    public int index;
    public int intakePosition;
    public int launchPosition;
    public GameColors ballColor;

    public BallEntry(int index, int intakePosition, int launchPosition, GameColors ballColor) {
        this.index = index;
        this.intakePosition = intakePosition;
        this.launchPosition = launchPosition;
        this.ballColor = ballColor;
    }
}
