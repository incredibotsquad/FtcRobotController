package org.firstinspires.ftc.teamcode.drive.opmode.auto.actions;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class VerticalShoulderAction implements Action {
    private RobotHardware robotHardware;
    private double targetPosition;
    private boolean initialized = false;
    private boolean waitForAction = false;
    private boolean shortWait = false;
    private ElapsedTime timer;

    private ElapsedTime incrementalTimer;

    public VerticalShoulderAction(RobotHardware robotHardware, double position, boolean waitForAction, boolean shortWait) {
        this.robotHardware = robotHardware;
        this.targetPosition = position;
        this.waitForAction = waitForAction;
        this.shortWait = shortWait;
    }

    @Override
    public boolean run (@NonNull TelemetryPacket packet) {
        double currentServoPosition = robotHardware.getVerticalShoulderServoPosition();
        double delta = targetPosition - currentServoPosition;


        if (currentServoPosition <= (RobotConstants.VERTICAL_SHOULDER_TRANSFER - 0.1) &&
                targetPosition > RobotConstants.VERTICAL_SHOULDER_HANG_SPECIMEN &&
                targetPosition <= RobotConstants.VERTICAL_SHOULDER_TRANSFER &&
                delta > 0.1) {

            if (incrementalTimer == null || incrementalTimer.milliseconds() > 50) {
                if (incrementalTimer == null) {
                    incrementalTimer = new ElapsedTime(MILLISECONDS);
                }

                Log.i("=== INCREDIBOTS / VerticalShoulderAction ===", " Incrementally setting servo position. Current: " + currentServoPosition + " Target: " + targetPosition);

                robotHardware.setVerticalShoulderServoPosition(currentServoPosition + delta / 2);
                incrementalTimer.reset();
            }

            return true;
        }
        else {
            if (!initialized) {
                Log.i("=== INCREDIBOTS / VerticalShoulderAction ===", " COMPLETED INCREMENTAL SERVO OPERATION - STRAIGHT SET CALL");

                initialized = true;
                robotHardware.setVerticalShoulderServoPosition(targetPosition);
                timer = new ElapsedTime(MILLISECONDS);
            }

            if (waitForAction) {
                if (shortWait) {
                    //return immediately if shoulder is already at position
                    if (Math.abs(robotHardware.getVerticalShoulderServoPosition() - targetPosition) < RobotConstants.VERTICAL_SHOULDER_POSITION_TOLERANCE)
                        return false;

                    return (timer.milliseconds() < 250);
                }

                //tell RR we need to keep running if shoulder is not in the right position yet
                return (Math.abs(robotHardware.getVerticalShoulderServoPosition() - targetPosition) > RobotConstants.VERTICAL_SHOULDER_POSITION_TOLERANCE);
            }

            return false;
        }
    }
}