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

    public static double HORIZONTAL_TURRET_MAX_POS = 0.75;
    public static double HORIZONTAL_TURRET_MIN_POS = 0.225;
    public static double HORIZONTAL_TURRET_INCREMENT = 0.005;
    public static double HORIZONTAL_WRIST_INCREMENT = 0.005;

    public static double PICKUP_ARM_LENGTH = 7; // 8 inches
    public static double TURRET_CENTER_POSITION = 0.475;

    public static double TURRET_OFFSET_FROM_CAMERA = 5;

    public static int IMAGE_WIDTH = 1280;
    public static int IMAGE_HEIGHT = 720;

    public static double COLOR_SENSOR_DISTANCE_THRESHOLD = 2;   // in cm.

    public static int SLIDE_POSITION_TOLERANCE = 50;

    public static double HORIZONTAL_CLAW_OPEN = 0.85;
    public static double HORIZONTAL_CLAW_CLOSE = 0.57;
    public static int HORIZONTAL_SLIDE_VELOCITY = 2000;

    // HORIZONTAL RESTING STATE VALUES
    public static int HORIZONTAL_SLIDE_RESTING = 0;
    public static double HORIZONTAL_TURRET_RESTING = 0;
    public static double HORIZONTAL_SHOULDER_RESTING = 0;
    public static double HORIZONTAL_ELBOW_RESTING = 0;
    public static double HORIZONTAL_WRIST_RESTING = 0;



    // HORIZONTAL PICK SAMPLE STATE VALUES
    public static int HORIZONTAL_SLIDE_PICK_SAMPLE = 0;
    public static double HORIZONTAL_TURRET_PICK_SAMPLE = 0.5;
    public static double HORIZONTAL_SHOULDER_PICK_SAMPLE = 0.465;
    public static double HORIZONTAL_ELBOW_PICK_SAMPLE = 0.1;
    public static double HORIZONTAL_WRIST_PICK_SAMPLE = 0.52;

    // HORIZONTAL CAMERA READY VALUES
    public static double HORIZONTAL_ELBOW_CAMERA_READY = HORIZONTAL_ELBOW_PICK_SAMPLE;
    public static double HORIZONTAL_TURRET_CAMERA_READY = TURRET_CENTER_POSITION;
    public static int HORIZONTAL_SLIDE_CAMERA_READY = HORIZONTAL_SLIDE_RESTING;
    public static double HORIZONTAL_SHOULDER_CAMERA_READY = 0.35;
    public static double HORIZONTAL_WRIST_CAMERA_READY = 0.52;


    // HORIZONTAL TRANSFER STATE VALUES
    public static int HORIZONTAL_SLIDE_TRANSFER = 0;
    public static double HORIZONTAL_TURRET_TRANSFER = TURRET_CENTER_POSITION;
    public static double HORIZONTAL_SHOULDER_TRANSFER = 0.36;
    public static double HORIZONTAL_SHOULDER_AFTER_TRANSFER = 0.4;
    public static double HORIZONTAL_ELBOW_TRANSFER = 0.8;
    public static double HORIZONTAL_ELBOW_AFTER_TRANSFER = 0.5;
    public static double HORIZONTAL_WRIST_TRANSFER = 0.52;

    //HORIZONTAL ENTER EXIT SUB STATE VALUES
    public static double HORIZONTAL_ELBOW_ENTER_EXIT_SUB = HORIZONTAL_ELBOW_PICK_SAMPLE;
    public static double HORIZONTAL_TURRET_ENTER_EXIT_SUB = TURRET_CENTER_POSITION;
    public static int HORIZONTAL_SLIDE_ENTER_EXIT_SUB = 0;
    public static double HORIZONTAL_SHOULDER_ENTER_EXIT_SUB = 0.43;
    public static double HORIZONTAL_WRIST_ENTER_EXIT_SUB = HORIZONTAL_WRIST_TRANSFER;

    // HORIZONTAL PICK SPECIMEN STATE VALUES
    public static int HORIZONTAL_SLIDE_PICK_SPECIMEN = HORIZONTAL_SLIDE_TRANSFER;
    public static double HORIZONTAL_TURRET_PICK_SPECIMEN = HORIZONTAL_TURRET_TRANSFER;
    public static double HORIZONTAL_SHOULDER_PICK_SPECIMEN = HORIZONTAL_SHOULDER_AFTER_TRANSFER;
    public static double HORIZONTAL_ELBOW_PICK_SPECIMEN = HORIZONTAL_ELBOW_AFTER_TRANSFER;
    public static double HORIZONTAL_WRIST_PICK_SPECIMEN = HORIZONTAL_WRIST_TRANSFER;


    public static double VERTICAL_CLAW_OPEN = 0.95;
    public static double VERTICAL_CLAW_CLOSE = 0.55;
    public static int VERTICAL_SLIDE_VELOCITY = 3000;

    // VERTICAL RESTING STATE VALUES
    public static int VERTICAL_SLIDE_RESTING = 0;
    public static double VERTICAL_SHOULDER_RESTING = 0.8;
    public static double VERTICAL_ELBOW_RESTING = 0.85;
    public static double VERTICAL_WRIST_RESTING = 0.82;

    // VERTICAL DROP SAMPLE OB-ZONE VALUES
    public static int VERTICAL_SLIDE_DROP_SAMPLE_OBZONE = 440;
    public static double VERTICAL_SHOULDER_DROP_SAMPLE_OBZONE = 0.85;
    public static double VERTICAL_ELBOW_DROP_SAMPLE_OBZONE = 0;
    public static double VERTICAL_WRIST_DROP_SAMPLE_OBZONE = 0;

    // VERTICAL TRANSFER STATE VALUES
    public static int VERTICAL_SLIDE_TRANSFER = 330;
    public static double VERTICAL_SHOULDER_TRANSFER = 0.74;
    public static double VERTICAL_ELBOW_TRANSFER = 0.85;
    public static double VERTICAL_WRIST_TRANSFER = 0.82;

    // VERTICAL PICK SPECIMEN VALUES
    public static int VERTICAL_SLIDE_PICK_SPECIMEN = 450;
    public static double VERTICAL_SHOULDER_PICK_SPECIMEN = 0.78;
    public static double VERTICAL_ELBOW_PICK_SPECIMEN = 0.14;
    public static double VERTICAL_WRIST_PICK_SPECIMEN = VERTICAL_WRIST_TRANSFER;

    // VERTICAL SNAP HIGH SPECIMEN VALUES
    public static int VERTICAL_SLIDE_SNAP_HIGH_SPECIMEN = 450;
    public static double VERTICAL_SHOULDER_SNAP_HIGH_SPECIMEN = 0.26;
    public static double VERTICAL_ELBOW_SNAP_HIGH_SPECIMEN = 0.3;
    public static double VERTICAL_WRIST_SNAP_HIGH_SPECIMEN = VERTICAL_WRIST_TRANSFER;

    // VERTICAL LOW BASKET VALUES
    public static int VERTICAL_SLIDE_DROP_LOW_SAMPLE = 440;
    public static double VERTICAL_SHOULDER_DROP_LOW_SAMPLE = 0.15;
    public static double VERTICAL_ELBOW_DROP_LOW_SAMPLE = 0.6;
    public static double VERTICAL_WRIST_DROP_LOW_SAMPLE = 0.82;

    // VERTICAL HIGH BASKET VALUES
    public static int VERTICAL_SLIDE_DROP_HIGH_SAMPLE = 2350;
    public static double VERTICAL_SHOULDER_DROP_HIGH_SAMPLE = 0.15;
    public static double VERTICAL_ELBOW_DROP_HIGH_SAMPLE = 0.55;
    public static double VERTICAL_WRIST_DROP_HIGH_SAMPLE = 0.82;

}
