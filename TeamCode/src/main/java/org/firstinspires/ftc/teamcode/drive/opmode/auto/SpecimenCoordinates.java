package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class SpecimenCoordinates {
    private int multiplier = 1;    //used to flip coordinates between blue (1) and red (-1)

    public double heading;
    public double reverseHeading;

    public double minTransVelocity;
    public double minAccel;
    public double maxAccel;

    public Pose2d INIT_POS;
    public Vector2d SLIDE_NEXT_TO_SAMP_1;
    public Vector2d SLIDE_BEHIND_SAMP_1;
    public Vector2d PUSH_SAMP_1;
    public Vector2d SLIDE_BEHIND_SAMP_2;
    public Vector2d PUSH_SAMP_2;

    public Vector2d PICK_SPECIMEN;
    public Vector2d PICK_SPECIMEN_SLOW;

    public Pose2d BRACE_RUNGS_FOR_SPECIMEN_ONE;
    public Pose2d BRACE_RUNGS_FOR_SPECIMEN_TWO;
    public Pose2d BRACE_RUNGS_FOR_SPECIMEN_THREE;
    public Pose2d BRACE_RUNGS_FOR_SPECIMEN_FOUR;
    public Pose2d PARK;

    public SpecimenCoordinates(int multiplier) {
        this.multiplier = multiplier;    //used to flip coordinates between blue (1) and red (-1)

        minTransVelocity = 30;
        minAccel = -20;
        maxAccel = 40;

        heading = Math.toRadians(-90 * multiplier);
        reverseHeading = Math.toRadians(90 * multiplier);

        INIT_POS = new Pose2d(-16 * multiplier, 60.75 * multiplier, heading);
        SLIDE_NEXT_TO_SAMP_1 = new Vector2d(-37 * multiplier, 25 * multiplier);
        SLIDE_BEHIND_SAMP_1 = new Vector2d(-43.5 * multiplier, 15 * multiplier);
        PUSH_SAMP_1 = new Vector2d(SLIDE_BEHIND_SAMP_1.x, 50 * multiplier);
        SLIDE_BEHIND_SAMP_2 = new Vector2d(-55 * multiplier, 15 * multiplier);
        PUSH_SAMP_2 = new Vector2d(SLIDE_BEHIND_SAMP_2.x, 51 * multiplier);

        PICK_SPECIMEN = new Vector2d(-43 * multiplier, 56 * multiplier); //57.5
        PICK_SPECIMEN_SLOW = new Vector2d(PICK_SPECIMEN.x, 57 * multiplier); //58

        BRACE_RUNGS_FOR_SPECIMEN_ONE = new Pose2d(6 * multiplier, 30 * multiplier, heading);
        BRACE_RUNGS_FOR_SPECIMEN_TWO = new Pose2d(2.5 * multiplier, 30 * multiplier, heading);
        BRACE_RUNGS_FOR_SPECIMEN_THREE = new Pose2d(0.5 * multiplier, 30 * multiplier, heading);
        BRACE_RUNGS_FOR_SPECIMEN_FOUR = new Pose2d(-4 * multiplier, 30 * multiplier, heading);
        PARK = new Pose2d(PUSH_SAMP_1.x, PUSH_SAMP_1.y, Math.toRadians((-90 * multiplier) + 270));
    }

}
