package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="ResetLift", group="TeleOp")
public class ResetLift extends LinearOpMode {

    // Declare OpMode members.
    public static String servo1Name = "LeftLiftServo";
    public static double servo1Position = 0.1;

    public static String servo2Name = "RightLiftServo";
    public static double servo2Position = 0.1;

    private Servo Servo1;
    private Servo Servo2;

    @Override
    public void runOpMode() {

        while (opModeInInit()) {
            Servo1 = hardwareMap.get(Servo.class, servo1Name);
            Servo2 = hardwareMap.get(Servo.class, servo2Name);
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.aWasPressed()) {
                Servo1.setPosition(servo1Position);
                Servo2.setPosition(servo2Position);
            }
        }
    }
}
