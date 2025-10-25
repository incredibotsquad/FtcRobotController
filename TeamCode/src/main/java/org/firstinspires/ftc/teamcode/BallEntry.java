package org.firstinspires.ftc.teamcode;

public class BallEntry {
    public int index;
    public double intakePosition;
    public double launchPosition;
    public GameColors ballColor;

    public BallEntry(int index, double intakePosition, double launchPosition, GameColors ballColor) {
        this.intakePosition = intakePosition;
        this.launchPosition = launchPosition;
        this.ballColor = ballColor;
    }
}
