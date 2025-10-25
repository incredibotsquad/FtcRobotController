package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.core.commands.Command;
import dev.nextftc.hardware.impl.FeedbackServoEx;
import dev.nextftc.hardware.impl.ServoEx;

@Config
public class SetFeedbackServoPosition extends Command {
    ServoEx servo;
    double targetPosition;

    double positionTolerance;

    public static double DEFAULT_POSITION_TOLERANCE = 0.01;

    public SetFeedbackServoPosition(ServoEx servo, double targetPosition) {
        this(servo, targetPosition, DEFAULT_POSITION_TOLERANCE);
    }

    public SetFeedbackServoPosition(ServoEx servo, double targetPosition, double positionTolerance) {
        this.servo = servo;
        this.targetPosition = targetPosition;
        this.positionTolerance = positionTolerance;
        requires(this);
    }

    @Override
    public void start() {
        // executed when the command begins
        servo.setPosition(targetPosition);
    }

    @Override
    public boolean isDone() {

//        double currentPos = servo.getCurrentPosition();
        double currentPos = servo.getPosition();
        Log.i("SetFeedbackServoPosition", "Current Position: " + currentPos);

        return Math.abs(currentPos - targetPosition) < positionTolerance;
    }
}
