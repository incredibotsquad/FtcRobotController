package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.GameConstants;
import org.firstinspires.ftc.teamcode.LimelightLocation;

public class SampleAutoCoordinates {
    private int multiplier = 1;    //used to flip coordinates between blue (1) and red (-1)

    public double heading;
    public double reverseHeading;
    public double basketHeading;
    public double minTransVelocity;
    public double minAccel;
    public double maxAccel;

    public Pose2d INIT_POS;
    public Pose2d PICK_SAMPLE_1;
    public LimelightLocation SAMPLE1_LIMELIGHT_LOCATION;
    public Pose2d PICK_SAMPLE_2;
    public LimelightLocation SAMPLE2_LIMELIGHT_LOCATION;
    public Pose2d PICK_SAMPLE_3;
    public LimelightLocation SAMPLE3_LIMELIGHT_LOCATION;
    public Pose2d DROP_SAMPLE_IN_BASKET;
    public Pose2d PARK;

    public SampleAutoCoordinates(int multiplier) {
        this.multiplier = multiplier;    //used to flip coordinates between blue (1) and red (-1)

        minTransVelocity = 30;
        minAccel = -20;
        maxAccel = 40;

        heading = Math.toRadians(-90 * multiplier);
        reverseHeading = Math.toRadians(90 * multiplier);

        basketHeading = Math.toRadians(-90 * multiplier - 45);

        INIT_POS = new Pose2d(39.25 * multiplier, 63.5 * multiplier, heading);

        PICK_SAMPLE_1 = new Pose2d(48 * multiplier, 48 * multiplier, heading);
        SAMPLE1_LIMELIGHT_LOCATION = new LimelightLocation(0, 14, 90, 0, GameConstants.GAME_COLORS.YELLOW, 0);

        PICK_SAMPLE_2 = new Pose2d(58 * multiplier, 48 * multiplier, heading);
        SAMPLE2_LIMELIGHT_LOCATION = new LimelightLocation(0, 14, 0, 0, GameConstants.GAME_COLORS.YELLOW, 0);

        PICK_SAMPLE_3 = new Pose2d(63 * multiplier, 48 * multiplier, heading);
        SAMPLE3_LIMELIGHT_LOCATION = new LimelightLocation(-6, 14, 90, 0, GameConstants.GAME_COLORS.YELLOW, 0);

        DROP_SAMPLE_IN_BASKET = new Pose2d(62 * multiplier, 57 * multiplier, basketHeading);

        PARK = new Pose2d(-50*multiplier, -50*multiplier, Math.toRadians((-90 * multiplier) + 270));
    }

}
