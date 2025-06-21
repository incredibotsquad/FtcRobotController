package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.GameConstants;
import org.firstinspires.ftc.teamcode.LimelightLocation;

public class SpecimenAutoCoordinates {
    private int multiplier = 1;    //used to flip coordinates between blue (1) and red (-1)

    public double heading;
    public double reverseHeading;

    public double minTransVelocity;
    public double minAccel;
    public double maxAccel;

    public Pose2d INIT_POS;
    public Pose2d PICK_SAMPLE_1;
    LimelightLocation SAMPLE1_LIMELIGHT_LOCATION;

    public Pose2d PICK_SAMPLE_2;
    LimelightLocation SAMPLE2_LIMELIGHT_LOCATION;

    public Pose2d PICK_SAMPLE_3;
    LimelightLocation SAMPLE3_LIMELIGHT_LOCATION;

    public Pose2d PICK_SPECIMEN;
    public Pose2d PICK_SPECIMEN_SLOW;

    public Pose2d BRACE_RUNGS_FOR_SPECIMEN_ONE;
    public Pose2d BRACE_RUNGS_FOR_SPECIMEN_TWO;
    public Pose2d BRACE_RUNGS_FOR_SPECIMEN_THREE;
    public Pose2d BRACE_RUNGS_FOR_SPECIMEN_FOUR;
    public Pose2d PARK;

    public SpecimenAutoCoordinates(int multiplier) {
        this.multiplier = multiplier;    //used to flip coordinates between blue (1) and red (-1)

        minTransVelocity = 30;
        minAccel = -20;
        maxAccel = 40;

        heading = Math.toRadians(-90 * multiplier);
        reverseHeading = Math.toRadians(90 * multiplier);

        INIT_POS = new Pose2d(-15.25 * multiplier, 63.5 * multiplier, reverseHeading);

        PICK_SAMPLE_1 = new Pose2d(-48 * multiplier, 46 * multiplier, heading);
        SAMPLE1_LIMELIGHT_LOCATION = new LimelightLocation(0, 8.5, 90, 0,0, GameConstants.GAME_COLORS.YELLOW, 0);

        PICK_SAMPLE_2 = new Pose2d(-61 * multiplier, 46 * multiplier, heading);
        SAMPLE2_LIMELIGHT_LOCATION = new LimelightLocation(0, 10, 90, 0, 0, GameConstants.GAME_COLORS.YELLOW, 0);

        PICK_SAMPLE_3 = new Pose2d(-65 * multiplier, 46 * multiplier, heading);
        SAMPLE3_LIMELIGHT_LOCATION = new LimelightLocation(5, 8, 90, 0, 0, GameConstants.GAME_COLORS.YELLOW, 0);

        PICK_SPECIMEN = new Pose2d(-42 * multiplier, 50 * multiplier, reverseHeading);
        PICK_SPECIMEN_SLOW = new Pose2d(-42 * multiplier, 62.5 * multiplier, reverseHeading);

        BRACE_RUNGS_FOR_SPECIMEN_ONE = new Pose2d(8 * multiplier, 30 * multiplier, reverseHeading);
        BRACE_RUNGS_FOR_SPECIMEN_TWO = new Pose2d(7 * multiplier, 30 * multiplier, reverseHeading);

        BRACE_RUNGS_FOR_SPECIMEN_THREE = new Pose2d(6 * multiplier, 30 * multiplier, reverseHeading);
        BRACE_RUNGS_FOR_SPECIMEN_FOUR = new Pose2d(5 * multiplier, 30 * multiplier, reverseHeading);

        PARK = new Pose2d(-50 * multiplier, 50 * multiplier, reverseHeading);
    }

}
