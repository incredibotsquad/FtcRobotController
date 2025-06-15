package org.firstinspires.ftc.teamcode;

public class LimelightConstants {
    public static double IDEAL_ASPECT_RATIO = 3.5 / 1.5; // Expected width:height ratio for a properly aligned sample
    public static double IDEAL_Y = 0; // Set to 0 for camera-only testing

    // Pipeline mappings
    public static int RED_PIPELINE = 2;
    public static int YELLOW_PIPELINE = 3;
    public static int BLUE_PIPELINE = 4;

    public static double LIME_LIGHT_MOUNT_ANGLE = 30; // a1
    public static double LIME_LIGHT_LENS_HEIGHT_INCHES = 7.5; // h1
    public static double LIME_LIGHT_OFFSET = -3; // h1
    public static double SAMPLE_HEIGHT_INCHES = 0; // h2
    public static double TELESCOPE_OFFSET = 0; // Set to 0 for camera-only testing
    public static double X_WEIGHT = 2;
    public static double Y_WEIGHT = 3;
    public static double ROT_WEIGHT = 20;

    // Camera intrinsic parameters for Limelight 3A
    // Based on OV5647 sensor specifications and Limelight 3A documentation
    public static double SENSOR_WIDTH_PIXELS = 640.0; // Limelight 3A resolution: 640x480 @ 90FPS
    public static double SENSOR_HEIGHT_PIXELS = 480.0; // Limelight 3A resolution: 640x480 @ 90FPS
    public static double HORIZONTAL_FOV_DEGREES = 54.5; // Limelight 3A horizontal FOV
    public static double VERTICAL_FOV_DEGREES = 42.0; // Limelight 3A vertical FOV
    public static double SENSOR_WIDTH_MM = 3.76; // OV5647 sensor image area: 3.76 × 2.74 mm
    public static double SENSOR_HEIGHT_MM = 2.74; // OV5647 sensor image area: 3.76 × 2.74 mm
    public static double FOCAL_LENGTH_PIXELS = (SENSOR_WIDTH_PIXELS / 2.0) / Math.tan(Math.toRadians(HORIZONTAL_FOV_DEGREES / 2.0));
}
