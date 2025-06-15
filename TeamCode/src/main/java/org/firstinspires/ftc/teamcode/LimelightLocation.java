package org.firstinspires.ftc.teamcode;

public class LimelightLocation {
    public double translation;
    public double extension;
    public double rotation;
    public double score;
    public GameConstants.GAME_COLORS color;
    public double rotScore;
    public double distScore;
    public double yScore;
    public double orientationAngle; // Actual orientation angle in degrees
    public double rawTranslation; // Original X position before correction
    public double rawExtension;   // Original Y position before correction

    public LimelightLocation(double trans, double extension, double rotation, GameConstants.GAME_COLORS color) {
        this.translation = trans;
        this.extension = extension;
        this.rotation = rotation;
        this.color = color;
        this.orientationAngle = 0; // Initialize to 0
        this.rawTranslation = trans; // Default to same as corrected
        this.rawExtension = extension; // Default to same as corrected
    }
}