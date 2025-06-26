package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {

    public static final int tickPerRev = 2000;
    public static final double wheelCircumferenceMM = 48 * Math.PI;
    public static final double MMPerTick = wheelCircumferenceMM / tickPerRev;

    public static double HORIZONTAL_SLIDE_TICKS_PER_INCH = 538 / (1.575 * Math.PI); //Total ticks to extend slide divided by slide length

    public static int HORIZONTAL_SLIDE_MAX_POS = 1550;
    public static int HORIZONTAL_SLIDE_INCREMENT = 200;

    public static int VERTICAL_SLIDE_MAX_POS = 2350;
    public static int VERTICAL_SLIDE_INCREMENT = 100;
    public static double VERTICAL_SHOULDER_RAMP_STEP = 0.05;
    public static double HORIZONTAL_TURRET_CENTER = 0.64;
    public static double HORIZONTAL_TURRET_MAX_POS = 0.9;
    public static double HORIZONTAL_TURRET_MIN_POS = 0.36;
    public static double HORIZONTAL_TURRET_INCREMENT = 0.001;
    public static double HORIZONTAL_WRIST_INCREMENT = 0.005;

    public static double HORIZONTAL_WRIST_MIN_POS = 0.2;
    public static double HORIZONTAL_WRIST_MAX_POS = 0.8;

    public static double PICKUP_ARM_LENGTH = 7; // 8 inches

    public static double TURRET_OFFSET_FROM_CAMERA = 5;

    public static int IMAGE_WIDTH = 1280;
    public static int IMAGE_HEIGHT = 720;

    public static double COLOR_SENSOR_DISTANCE_THRESHOLD = 2;   // in cm.

    public static int SLIDE_POSITION_TOLERANCE = 50;
    public static double VERTICAL_SHOULDER_POSITION_TOLERANCE = 0.05;

    public static double HORIZONTAL_CLAW_OPEN = 0.6;
    public static double HORIZONTAL_CLAW_CLOSE = 0.32;
    public static int HORIZONTAL_SLIDE_VELOCITY = 2000;


    // HORIZONTAL PICK SAMPLE STATE VALUES
    public static int HORIZONTAL_SLIDE_PICK_SAMPLE = 0;
    public static double HORIZONTAL_TURRET_PICK_SAMPLE = 0.5;
    public static double HORIZONTAL_SHOULDER_PICK_SAMPLE = 0.504;
    public static double HORIZONTAL_ELBOW_PICK_SAMPLE = 0.2;
    public static double HORIZONTAL_WRIST_PICK_SAMPLE = 0.52;

    // HORIZONTAL CAMERA READY VALUES
    public static double HORIZONTAL_ELBOW_CAMERA_READY = HORIZONTAL_ELBOW_PICK_SAMPLE;
    public static double HORIZONTAL_TURRET_CAMERA_READY = HORIZONTAL_TURRET_CENTER;
    public static int HORIZONTAL_SLIDE_CAMERA_READY = 0;
    public static double HORIZONTAL_SHOULDER_CAMERA_READY = 0.385;
    public static double HORIZONTAL_WRIST_CAMERA_READY = 0.5;

    // HORIZONTAL HANG SPECIMEN VALUES
    public static int HORIZONTAL_SLIDE_HANG_SPECIMEN = 750;

    // HORIZONTAL TRANSFER STATE VALUES
    public static int HORIZONTAL_SLIDE_TRANSFER = 0;
    public static double HORIZONTAL_TURRET_TRANSFER = HORIZONTAL_TURRET_CENTER;
    public static double HORIZONTAL_SHOULDER_TRANSFER = 0.41;
    public static double HORIZONTAL_SHOULDER_AFTER_TRANSFER = 0.46;
    public static double HORIZONTAL_ELBOW_TRANSFER = 0.85;
    public static double HORIZONTAL_ELBOW_AFTER_TRANSFER = 0.5;
    public static double HORIZONTAL_WRIST_TRANSFER = 0.5;

    // HORIZONTAL RESTING STATE VALUES
    public static int HORIZONTAL_SLIDE_RESTING = HORIZONTAL_SLIDE_CAMERA_READY;
    public static double HORIZONTAL_TURRET_RESTING = HORIZONTAL_TURRET_CENTER;
    public static double HORIZONTAL_SHOULDER_RESTING = HORIZONTAL_SHOULDER_CAMERA_READY;
    public static double HORIZONTAL_ELBOW_RESTING = HORIZONTAL_ELBOW_TRANSFER;
    public static double HORIZONTAL_WRIST_RESTING = HORIZONTAL_WRIST_TRANSFER;


    //HORIZONTAL ENTER EXIT SUB STATE VALUES
    public static double HORIZONTAL_ELBOW_ENTER_EXIT_SUB = HORIZONTAL_ELBOW_PICK_SAMPLE;
    public static double HORIZONTAL_TURRET_ENTER_EXIT_SUB = HORIZONTAL_TURRET_CENTER;
    public static int HORIZONTAL_SLIDE_ENTER_EXIT_SUB = 0;
    public static double HORIZONTAL_SHOULDER_ENTER_EXIT_SUB = 0.47;
    public static double HORIZONTAL_WRIST_ENTER_EXIT_SUB = HORIZONTAL_WRIST_TRANSFER;

    // HORIZONTAL PICK SPECIMEN STATE VALUES
    public static int HORIZONTAL_SLIDE_PICK_SPECIMEN = HORIZONTAL_SLIDE_TRANSFER;
    public static double HORIZONTAL_TURRET_PICK_SPECIMEN = HORIZONTAL_TURRET_MAX_POS;
    public static double HORIZONTAL_SHOULDER_PICK_SPECIMEN = 0.48;
    public static double HORIZONTAL_ELBOW_TRANSITION_TO_PICK_SPECIMEN = 0.5;
    public static double HORIZONTAL_ELBOW_PICK_SPECIMEN = HORIZONTAL_ELBOW_TRANSFER;
    public static double HORIZONTAL_WRIST_PICK_SPECIMEN = 0.18;

    public static double VERTICAL_CLAW_OPEN = 0.6;
    public static double VERTICAL_CLAW_CLOSE =  0.2;
    public static int VERTICAL_SLIDE_VELOCITY = 3500;


    // VERTICAL DROP SAMPLE OB-ZONE VALUES
    public static int VERTICAL_SLIDE_DROP_SAMPLE_OBZONE = 500;
    public static double VERTICAL_SHOULDER_DROP_SAMPLE_OBZONE = 0.9;
    public static double VERTICAL_ELBOW_DROP_SAMPLE_OBZONE = 1;
    public static double VERTICAL_WRIST_DROP_SAMPLE_OBZONE = 0;

    // VERTICAL TRANSFER STATE VALUES
    public static int VERTICAL_SLIDE_TRANSFER = 400;
    public static double VERTICAL_SHOULDER_TRANSFER = 0.74;
    public static double VERTICAL_ELBOW_TRANSFER = 0;
    public static double VERTICAL_WRIST_TRANSFER = 0.68;

    // VERTICAL RESTING STATE VALUES
    public static int VERTICAL_SLIDE_RESTING = 0;
    public static double VERTICAL_SHOULDER_RESTING = 0.8;
    public static double VERTICAL_ELBOW_RESTING = VERTICAL_ELBOW_TRANSFER;
    public static double VERTICAL_WRIST_RESTING = VERTICAL_WRIST_TRANSFER;

    // VERTICAL PICK SPECIMEN VALUES
    public static int VERTICAL_SLIDE_PICK_SPECIMEN_STEP_1 = 600;
    public static int VERTICAL_SLIDE_PICK_SPECIMEN_STEP_2 = 0;
    public static double VERTICAL_SHOULDER_PICK_SPECIMEN = 0.605;
    public static double VERTICAL_ELBOW_PICK_SPECIMEN = 0.2;
    public static double VERTICAL_WRIST_PICK_SPECIMEN = 0.02;

    // VERTICAL HANG HIGH SPECIMEN VALUES
    public static int VERTICAL_SLIDE_HANG_SPECIMEN = 200;
    public static double VERTICAL_SHOULDER_HANG_SPECIMEN = 0.2;
    public static double VERTICAL_ELBOW_HANG_SPECIMEN = 0;
    public static double VERTICAL_WRIST_HANG_SPECIMEN = VERTICAL_WRIST_PICK_SPECIMEN;

    // VERTICAL SNAP HIGH SPECIMEN VALUES
    public static int VERTICAL_SLIDE_SNAP_SPECIMEN = 750;

    // VERTICAL LOW BASKET VALUES
    public static int VERTICAL_SLIDE_DROP_LOW_SAMPLE = 440;
    public static double VERTICAL_SHOULDER_DROP_LOW_SAMPLE = 0.12;
    public static double VERTICAL_ELBOW_DROP_LOW_SAMPLE = 0.6;
    public static double VERTICAL_WRIST_DROP_LOW_SAMPLE = VERTICAL_WRIST_TRANSFER;

    // VERTICAL HIGH BASKET VALUES
    public static int VERTICAL_SLIDE_DROP_HIGH_SAMPLE = 2350;
    public static double VERTICAL_SHOULDER_DROP_HIGH_SAMPLE = 0.12;
    public static double VERTICAL_ELBOW_DROP_HIGH_SAMPLE = 0.55;
    public static double VERTICAL_WRIST_DROP_HIGH_SAMPLE = VERTICAL_WRIST_TRANSFER;

}
