package org.firstinspires.ftc.teamcode;

public class ColorSenorOutput {
    public GameConstants.GAME_COLORS detectedColor;
    public double distance;

    public ColorSenorOutput (GameConstants.GAME_COLORS color, double distance) {
        this.detectedColor = color;
        this.distance = distance;
    }
}
