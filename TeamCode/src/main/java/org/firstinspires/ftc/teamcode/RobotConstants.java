package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {

    public static final int tickPerRev = 2000;
    public static final double wheelCircumferenceMM = 48 * Math.PI;
    public static final double MMPerTick = wheelCircumferenceMM / tickPerRev;

    public static double COLOR_SENSOR_DISTANCE_THRESHOLD = 2;   // in cm.

    public static int SLIDE_POSITION_TOLERANCE = 50;

    public static double HORIZONTAL_CLAW_OPEN = 1;
    public static double HORIZONTAL_CLAW_CLOSE = 0.8;
    public static int HORIZONTAL_SLIDE_VELOCITY = 1000;

    // HORIZONTAL RESTING STATE VALUES
    public static int HORIZONTAL_SLIDE_RESTING = 0;
    public static double HORIZONTAL_TURRET_RESTING = 0;
    public static double HORIZONTAL_SHOULDER_RESTING = 0;
    public static double HORIZONTAL_ELBOW_RESTING = 0;
    public static double HORIZONTAL_WRIST_RESTING = 0;

    // HORIZONTAL PICK SAMPLE STATE VALUES
    public static int HORIZONTAL_SLIDE_PICK_SAMPLE = 0;
    public static double HORIZONTAL_TURRET_PICK_SAMPLE = 0.5;
    public static double HORIZONTAL_SHOULDER_PICK_SAMPLE = 0.47;
    public static double HORIZONTAL_ELBOW_PICK_SAMPLE = 0.1;
    public static double HORIZONTAL_WRIST_PICK_SAMPLE = 0;

    // HORIZONTAL TRANSFER STATE VALUES
    public static int HORIZONTAL_SLIDE_TRANSFER = 0;
    public static double HORIZONTAL_TURRET_TRANSFER = 0.5;
    public static double HORIZONTAL_SHOULDER_TRANSFER = 0.34;
    public static double HORIZONTAL_SHOULDER_AFTER_TRANSFER = 0.4;
    public static double HORIZONTAL_ELBOW_TRANSFER = 0.7;
    public static double HORIZONTAL_ELBOW_AFTER_TRANSFER = 0.5;
    public static double HORIZONTAL_WRIST_TRANSFER = 0.52;

    // HORIZONTAL PICK SPECIMEN STATE VALUES
    public static int HORIZONTAL_SLIDE_PICK_SPECIMEN = HORIZONTAL_SLIDE_TRANSFER;
    public static double HORIZONTAL_TURRET_PICK_SPECIMEN = HORIZONTAL_TURRET_TRANSFER;
    public static double HORIZONTAL_SHOULDER_PICK_SPECIMEN = HORIZONTAL_SHOULDER_AFTER_TRANSFER;
    public static double HORIZONTAL_ELBOW_PICK_SPECIMEN = HORIZONTAL_ELBOW_AFTER_TRANSFER;
    public static double HORIZONTAL_WRIST_PICK_SPECIMEN = HORIZONTAL_WRIST_TRANSFER;


    public static double VERTICAL_CLAW_OPEN = 0.95;
    public static double VERTICAL_CLAW_CLOSE = 0.5;
    public static int VERTICAL_SLIDE_VELOCITY = 1500;

    // VERTICAL RESTING STATE VALUES
    public static int VERTICAL_SLIDE_RESTING = 0;
    public static double VERTICAL_SHOULDER_RESTING = 0.8;
    public static double VERTICAL_ELBOW_RESTING = 0.85;
    public static double VERTICAL_WRIST_RESTING = 0.5;

    // VERTICAL DROP SAMPLE OB-ZONE VALUES
    public static int VERTICAL_SLIDE_DROP_SAMPLE_OBZONE = 150;
    public static double VERTICAL_SHOULDER_DROP_SAMPLE_OBZONE = 0.85;
    public static double VERTICAL_ELBOW_DROP_SAMPLE_OBZONE = 0;
    public static double VERTICAL_WRIST_DROP_SAMPLE_OBZONE = 0;

    // VERTICAL TRANSFER STATE VALUES
    public static int VERTICAL_SLIDE_TRANSFER = 110;
    public static double VERTICAL_SHOULDER_TRANSFER = 0.73;
    public static double VERTICAL_ELBOW_TRANSFER = 0.85;
    public static double VERTICAL_WRIST_TRANSFER = 0.5;

    // VERTICAL PICK SPECIMEN VALUES
    public static int VERTICAL_SLIDE_PICK_SPECIMEN = 150;
    public static double VERTICAL_SHOULDER_PICK_SPECIMEN = 0.85;
    public static double VERTICAL_ELBOW_PICK_SPECIMEN = 0.23;
    public static double VERTICAL_WRIST_PICK_SPECIMEN = VERTICAL_WRIST_TRANSFER;

    // VERTICAL SNAP HIGH SPECIMEN VALUES
    public static int VERTICAL_SLIDE_SNAP_HIGH_SPECIMEN = 0;
    public static double VERTICAL_SHOULDER_SNAP_HIGH_SPECIMEN = 0;
    public static double VERTICAL_ELBOW_SNAP_HIGH_SPECIMEN = 0;
    public static double VERTICAL_WRIST_SNAP_HIGH_SPECIMEN = 0;

    // VERTICAL LOW BASKET VALUES
    public static int VERTICAL_SLIDE_DROP_LOW_SAMPLE = 300;
    public static double VERTICAL_SHOULDER_DROP_LOW_SAMPLE = 0.08;
    public static double VERTICAL_ELBOW_DROP_LOW_SAMPLE = 0;
    public static double VERTICAL_WRIST_DROP_LOW_SAMPLE = 0.5;

    // VERTICAL HIGH BASKET VALUES
    public static int VERTICAL_SLIDE_DROP_HIGH_SAMPLE = 500;
    public static double VERTICAL_SHOULDER_DROP_HIGH_SAMPLE = 0.08;
    public static double VERTICAL_ELBOW_DROP_HIGH_SAMPLE = 0.5;
    public static double VERTICAL_WRIST_DROP_HIGH_SAMPLE = 0.5;

}
