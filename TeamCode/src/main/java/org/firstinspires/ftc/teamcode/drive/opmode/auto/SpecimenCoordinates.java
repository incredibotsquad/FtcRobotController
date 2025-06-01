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
    public Vector2d SLIDE_BETWEEN_SAMP1_SAMP2;
    public Vector2d SLIDE_BETWEEN_SAMP2_SAMP3;

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

        SLIDE_BETWEEN_SAMP1_SAMP2 = new Vector2d(-37 * multiplier, 25 * multiplier);

        SLIDE_BETWEEN_SAMP2_SAMP3 = new Vector2d(-55 * multiplier, 15 * multiplier);

        PICK_SPECIMEN = new Vector2d(-43 * multiplier, 56 * multiplier);
        PICK_SPECIMEN_SLOW = new Vector2d(PICK_SPECIMEN.x, 57 * multiplier);

        BRACE_RUNGS_FOR_SPECIMEN_ONE = new Pose2d(6 * multiplier, 30 * multiplier, heading);
        BRACE_RUNGS_FOR_SPECIMEN_TWO = new Pose2d(2.5 * multiplier, 30 * multiplier, heading);
        BRACE_RUNGS_FOR_SPECIMEN_THREE = new Pose2d(0.5 * multiplier, 30 * multiplier, heading);
        BRACE_RUNGS_FOR_SPECIMEN_FOUR = new Pose2d(-4 * multiplier, 30 * multiplier, heading);

        PARK = new Pose2d(-50*multiplier, -50*multiplier, Math.toRadians((-90 * multiplier) + 270));
    }

}
