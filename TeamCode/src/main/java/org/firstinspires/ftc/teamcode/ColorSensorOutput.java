package org.firstinspires.ftc.teamcode;

public class ColorSensorOutput {
    public GameConstants.GAME_COLORS detectedColor;
    public double distance;

    public ColorSensorOutput(GameConstants.GAME_COLORS color, double distance) {
        this.detectedColor = color;
        this.distance = distance;
    }
}
