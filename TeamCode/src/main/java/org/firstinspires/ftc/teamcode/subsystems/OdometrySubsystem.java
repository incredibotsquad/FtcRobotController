package org.firstinspires.ftc.teamcode.subsystems;


import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;

public class OdometrySubsystem extends SubsystemBase {

    private final GoBildaPinpointDriver pinpoint;
    private Pose2d currentPose = new Pose2d();
    private Pose2d previousPose = new Pose2d();
    private Translation2d fieldVelocity = new Translation2d(0, 0);
    private Translation2d fieldAcceleration = new Translation2d(0, 0);
    private long previousUpdateNanos = 0;

    private TelemetryManager telemetry;

    public static double PINPOINT_X_OFFSET_INCH = -2.0;
    public static double PINPOINT_Y_OFFSET_INCH = -5.0;
    public static String PINPOINT_HARDWARE_NAME = "pinpoint";
    public static DistanceUnit PINPOINT_DISTANCE_UNIT = DistanceUnit.INCH;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_X_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_ENCODER_DIRECTION = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_ENCODER_RESOLUTION = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;

    public OdometrySubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {

        this.telemetry = telemetry;

        // Initialize the Pinpoint hardware device from the map
        // Note: Avoid I2C Port 0 if possible, as the Control Hub IMU shares it.
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, PINPOINT_HARDWARE_NAME);

        // 1. Configure your hardware specifics
        // Set the resolution depending on your exact pods (e.g., goBILDA 4-bar pods)
        pinpoint.setEncoderResolution(PINPOINT_ENCODER_RESOLUTION);

        // 2. Define your physical pod offsets (in millimeters relative to the center of rotation)
        // Adjust these numbers based on where you physically bolted your pods!
        pinpoint.setOffsets(PINPOINT_X_OFFSET_INCH, PINPOINT_Y_OFFSET_INCH, PINPOINT_DISTANCE_UNIT);

        // 3. Set directions if your pods read backward
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, 
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Reset the position to (0,0) heading 0 upon initialization
        pinpoint.resetPosAndIMU();

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (timer.milliseconds() < 500) {
            Log.i("Odometry subsystem", "Resetting pinpoint ");
        }
        pinpoint.update();

//        Pose2D resetPose = pinpoint.getPosition();
//        Log.i("Odometry subsystem", "Pose after resetting pinpoint: X: " + resetPose.getX(DistanceUnit.INCH) + " Y: " + resetPose.getY(DistanceUnit.INCH) + " Angle: " + resetPose.getHeading(AngleUnit.DEGREES));

    }

    public Pose2d getPose() {

        Log.i("Odometry subsystem", "Get pose: will return: " + currentPose.toString());
        return currentPose;
    }

    public Translation2d getFieldVelocity() {
        return fieldVelocity;
    }

    public Translation2d getFieldAcceleration() {
        return fieldAcceleration;
    }

    public double getFieldSpeedInchesPerSecond() {
        return Math.hypot(fieldVelocity.getX(), fieldVelocity.getY());
    }

    // Allows you to reset the location (e.g., at the start of Autonomous)
    public void resetPose(double xInches, double yInches, double rotationDegrees) {

        Log.i("Odometry subsystem", "Reset pose. X: " + xInches + " Y: " + yInches + " Angle: " + rotationDegrees);

        pinpoint.setPosX(xInches, DistanceUnit.INCH);
        pinpoint.setPosY(yInches, DistanceUnit.INCH);
        pinpoint.setHeading(Math.toRadians(rotationDegrees), AngleUnit.RADIANS);

        //force an update so the hardware registers it
        pinpoint.update();

        currentPose = new Pose2d(xInches, yInches, new Rotation2d(Math.toRadians(rotationDegrees)));

        Log.i("Odometry subsystem", "current pose set to:"  + currentPose.toString());

        previousPose = currentPose;
        fieldVelocity = new Translation2d(0, 0);
        fieldAcceleration = new Translation2d(0, 0);
        previousUpdateNanos = System.nanoTime();
    }

    // Inside OdometrySubsystem.java

    /**
     * Updates the internal Pinpoint position with a new pose.
     * We use this to correct drift using Limelight.
     */
    public void updatePoseFromLimelight(Pose2d correctedPose) {

        Log.i("Odometry Subsystem", "Updating pose from limelight. Old pose: " + currentPose.toString() + " New pose: " + correctedPose.toString());

        // We update the Pinpoint's internal X, Y and Heading
        pinpoint.setPosX(correctedPose.getX(), DistanceUnit.INCH);
        pinpoint.setPosY(correctedPose.getY(), DistanceUnit.INCH);
        pinpoint.setHeading(correctedPose.getHeading(), AngleUnit.RADIANS);

        pinpoint.update();
    }

    private void updateVelocityEstimate(Pose2d newPose) {
        long now = System.nanoTime();

        if (previousUpdateNanos != 0) {
            double dtSeconds = (now - previousUpdateNanos) / 1.0e9;

            if (dtSeconds > 0.0) {
                Translation2d previousVelocity = fieldVelocity;
                fieldVelocity = new Translation2d(
                        (newPose.getX() - previousPose.getX()) / dtSeconds,
                        (newPose.getY() - previousPose.getY()) / dtSeconds
                );
                fieldAcceleration = new Translation2d(
                        (fieldVelocity.getX() - previousVelocity.getX()) / dtSeconds,
                        (fieldVelocity.getY() - previousVelocity.getY()) / dtSeconds
                );
            }
        }

        previousPose = newPose;
        previousUpdateNanos = now;
    }

    @Override
    public void periodic() {
        // CRITICAL: Must be called every loop to pull fresh numbers from the coprocessor
        pinpoint.update();

        // Convert Pinpoint values (inches/millimeters) to FTCLib's standard units
        // FTCLib natively handles position as a Pose2d(x, y, Rotation2d)
        double xInches = pinpoint.getPosX(DistanceUnit.INCH); // Returns position in inches
        double yInches = pinpoint.getPosY(DistanceUnit.INCH);
        double headingRadians = pinpoint.getHeading(AngleUnit.RADIANS);

        Pose2d newPose = new Pose2d(xInches, yInches, new Rotation2d(headingRadians));
        updateVelocityEstimate(newPose);

        currentPose = newPose;
        CrossOpModeStorage.currentPose = currentPose;

        Log.i("Odometry Periodic", "Current pose: " + currentPose.toString());

        telemetry.addData("Odometry: X Position", xInches);
        telemetry.addData("Odometry: Y Position", yInches);
        telemetry.addData("Odometry: Heading (Deg)", Math.toDegrees(headingRadians) + 360); //adding for ease of readability
//        telemetry.addData("Odometry: X Velocity", fieldVelocity.getX());
//        telemetry.addData("Odometry: Y Velocity", fieldVelocity.getY());
//        telemetry.addData("Odometry: Speed", getFieldSpeedInchesPerSecond());
    }
}
