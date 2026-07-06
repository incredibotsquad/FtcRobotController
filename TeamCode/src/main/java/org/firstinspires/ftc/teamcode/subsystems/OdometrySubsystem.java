package org.firstinspires.ftc.teamcode.subsystems;


import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.drivers.GoBildaPinpointDriver; // Path to your driver

public class OdometrySubsystem extends SubsystemBase {

    private final GoBildaPinpointDriver pinpoint;
    private Pose2d currentPose = new Pose2d();

    private TelemetryManager telemetry;

    public OdometrySubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {

        this.telemetry = telemetry;

        // Initialize the Pinpoint hardware device from the map
        // Note: Avoid I2C Port 0 if possible, as the Control Hub IMU shares it.
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // 1. Configure your hardware specifics
        // Set the resolution depending on your exact pods (e.g., goBILDA 4-bar pods)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // 2. Define your physical pod offsets (in millimeters relative to the center of rotation)
        // Adjust these numbers based on where you physically bolted your pods!
        pinpoint.setOffsets(-2.0, -5.0, DistanceUnit.INCH);

        // 3. Set directions if your pods read backward
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, 
                                      GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Reset the position to (0,0) heading 0 upon initialization
        pinpoint.resetPosAndIMU();

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (timer.milliseconds() < 500) {
//            Log.i("Odometry subsystem", "Resetting pinpoint ");
        }
        pinpoint.update();

//        Pose2D resetPose = pinpoint.getPosition();
//        Log.i("Odometry subsystem", "Pose after resetting pinpoint: X: " + resetPose.getX(DistanceUnit.INCH) + " Y: " + resetPose.getY(DistanceUnit.INCH) + " Angle: " + resetPose.getHeading(AngleUnit.DEGREES));

    }

    public Pose2d getPose() {
        return currentPose;
    }

    // Allows you to reset the location (e.g., at the start of Autonomous)
    public void resetPose(double xInches, double yInches, double rotationDegrees) {

        Log.i("Odometry subsystem", "Reset pose. X: " + xInches + " Y: " + yInches + " Angle: " + rotationDegrees);

        pinpoint.setPosX(xInches, DistanceUnit.INCH);
        pinpoint.setPosY(yInches, DistanceUnit.INCH);
        pinpoint.setHeading(Math.toRadians(rotationDegrees), AngleUnit.RADIANS);

        //force an update so the hardware registers it
        pinpoint.update();
    }

    @Override
    public void periodic() {
        // CRITICAL: Must be called every loop to pull fresh numbers from the coprocessor
        pinpoint.update();

        // Convert Pinpoint values (inches/millimeters) to FTCLib's standard units
        // FTCLib natively handles position as a Pose2d(x, y, Rotation2d)
        double xInches = pinpoint.getPosX(DistanceUnit.INCH); // Returns position in inches
        double yInches = pinpoint.getPosY(DistanceUnit.INCH);

//        double headingDegrees = pinpoint.getHeading(AngleUnit.DEGREES);


        double headingRadians = pinpoint.getHeading(AngleUnit.RADIANS);

        currentPose = new Pose2d(xInches, yInches, new Rotation2d(headingRadians));
        CrossOpModeStorage.currentPose = currentPose;

//        Log.i("Odometry", "X Position: " + xInches + " Y Position: " + yInches + " Heading: " + headingDegrees);

        telemetry.addData("Odometry: X Position", xInches);
        telemetry.addData("Odometry: Y Position", yInches);
        telemetry.addData("Odometry: Heading (Deg)", Math.toDegrees(headingRadians) + 360); //adding for ease of readability
    }
}