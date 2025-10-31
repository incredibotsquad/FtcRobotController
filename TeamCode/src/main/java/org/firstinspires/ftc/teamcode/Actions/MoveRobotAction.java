package org.firstinspires.ftc.teamcode.Actions;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class MoveRobotAction implements Action {

    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private RobotHardware robotHardware;
    private Pose2D targetPosition;
    private boolean initialized;
    private ElapsedTime timer;
    private double tolerance;
    public MoveRobotAction(RobotHardware robotHardware, Pose2D targetPosition, double tolerance) {
        this.robotHardware = robotHardware;
        this.targetPosition = targetPosition;
        this.tolerance = tolerance;
        this.initialized = false;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {

        if (!initialized) {
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            initialized = true;
        }

        //run this loop every 10 ms.
        if (timer.milliseconds() < 10) return true;

        timer.reset();

        //get the current robot position
        Pose2D currPos = robotHardware.getCurrentRobotPose();

        Log.i("Move Robot Action", "Target Heading: " + targetPosition.getHeading(AngleUnit.DEGREES));
        Log.i("Move Robot Action", "Current Heading: " + currPos.getHeading(AngleUnit.DEGREES));

        double rangeError = 0;
        double headingError = currPos.getHeading(AngleUnit.DEGREES) - targetPosition.getHeading(AngleUnit.DEGREES);
        double yawError = 0;

        Log.i("Move Robot Action", "Heading Error: " + targetPosition.getHeading(AngleUnit.DEGREES));
        Log.i("Move Robot Action", "Tolerance: " + tolerance);

        if (Math.abs(headingError) < tolerance) {
            return false;   //dont need to move bot anymore
        }

        // Use the speed and turn "gains" to calculate how we want the robot to move.
        double drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        double turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
        double strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        moveRobot(drive, strafe, turn);

        return true;
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double frontLeftPower    =  x - y - yaw;
        double frontRightPower   =  x + y + yaw;
        double backLeftPower     =  x + y - yaw;
        double backRightPower    =  x - y + yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        robotHardware.setDriveMotorPowers(frontRightPower, frontLeftPower, backRightPower, backLeftPower);
    }
}
