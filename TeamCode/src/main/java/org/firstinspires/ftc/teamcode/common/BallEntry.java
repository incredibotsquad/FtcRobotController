package org.firstinspires.ftc.teamcode.common;

public class BallEntry {
    public int index;
    public double intakePosition;
    public double launchPosition;
    public GameColors ballColor;

    public BallEntry(int index, double intakePosition, double launchPosition, GameColors ballColor) {
        this.index = index;
        this.intakePosition = intakePosition;
        this.launchPosition = launchPosition;
        this.ballColor = ballColor;
    }
}
