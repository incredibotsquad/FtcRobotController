package org.firstinspires.ftc.teamcode.subsystems;


import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

        Log.i("Odometry subsystem", "Resetting pinpoint ");
    }

    public Pose2d getPose() {
        return currentPose;
    }

    public void resetPose(Pose2d newPose) {
        // Allows you to reset the location (e.g., at the start of Autonomous)

        Log.i("Odometry subsystem", "Reset pose. X: " + newPose.getX() + " Y: " + newPose.getY() + " Angle: " + newPose.getRotation().getDegrees());

        pinpoint.setPosition(new Pose2D(
            DistanceUnit.INCH,
            newPose.getX(),
            newPose.getY(),
            AngleUnit.RADIANS,
            newPose.getRotation().getRadians()
        ));

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
        double headingDegrees = pinpoint.getHeading(AngleUnit.DEGREES);

        currentPose = new Pose2d(xInches, yInches, new Rotation2d(Math.toRadians(headingDegrees)));
        CrossOpModeStorage.currentPose = currentPose;

//        Log.i("Odometry", "X Position: " + xInches + " Y Position: " + yInches + " Heading: " + headingDegrees);

        telemetry.addData("Odometry: X Position", xInches);
        telemetry.addData("Odometry: Y Position", yInches);
        telemetry.addData("Odometry: Heading (Deg)", headingDegrees);
    }
}