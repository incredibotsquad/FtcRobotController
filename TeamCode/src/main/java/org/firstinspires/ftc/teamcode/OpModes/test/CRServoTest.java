package org.firstinspires.ftc.teamcode.OpModes.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.common.EncodedCRServo;

// TeleOp test class for manual tuning and testing
@TeleOp(name = "Cont. Rotation Axon Test", group = "test")
public class CRServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        CRServo crservo = hardwareMap.crservo.get("rightHorizSlide");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "rightHorizSlideEncoder");
        EncodedCRServo servo = new EncodedCRServo(crservo, encoder);

        waitForStart();

        while (!isStopRequested()) {
            servo.update();

            // Manual controls for target and PID tuning
            if (gamepad1.dpadUpWasPressed()) {
                servo.changeTargetRotation(15);
            }
            if (gamepad1.dpadDownWasPressed()) {
                servo.changeTargetRotation(-15);
            }
            if (gamepad1.bWasPressed()) {
                servo.setTargetRotation(0);
            }

            if (gamepad1.aWasPressed()) {
                servo.setKP(servo.getKP() + 0.001);
            }
            if (gamepad1.xWasPressed()) {
                servo.setKP(Math.max(0, servo.getKP() - 0.001));
            }

            if (gamepad1.rightBumperWasPressed()) {
                servo.setKI(servo.getKI() + 0.0001);
            }
            if (gamepad1.leftBumperWasPressed()) {
                servo.setKI(Math.max(0, servo.getKI() - 0.0001));
            }

            if (gamepad1.backWasPressed()) {
                servo.setKP(0.015);
                servo.setKI(0.0005);
                servo.setKD(0.0025);
                servo.resetPID();
            }

            telemetry.addData("Starting angle", servo.STARTPOS);
            telemetry.addLine(servo.log());
            telemetry.addData("NTRY", servo.ntry);
            telemetry.update();
        }
    }
}