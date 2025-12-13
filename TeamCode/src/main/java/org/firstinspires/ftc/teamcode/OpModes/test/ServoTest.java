package org.firstinspires.ftc.teamcode.OpModes.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@Disabled

@Disabled
@Config
@TeleOp(name="ServoTest", group="Tests")
public class ServoTest extends LinearOpMode {

    // Declare OpMode members.
    public static String servo1Name = "SpindexServo";
    public static double servo1Position = 0.5;

    public static boolean enableServo2 = false;
    public static String servo2Name = "LaunchKickServo";
    public static double servo2Position = 0.5;

    public static boolean enableServo3 = false;
    public static String servo3Name = "LaunchTurretServo";
    public static double servo3Position = 0.5;

    public static boolean enableServo4 = false;
    public static String servo4Name = "LaunchVisorServo";
    public static double servo4Position = 0.5;


    // Close: 0.42
    // Open: 0.55
    private ElapsedTime runtime = new ElapsedTime();
    private Servo Servo1;
    private Servo Servo2;

    private Servo Servo3;
    private Servo Servo4;

    //0.54, 0.35
    //0.17, 0
    //

    //Spindexer Slot 1 intake: 0.225 outtake: 0.08
    //Spindexer Slot 2 intake: 0.6 outtake: 0.4
    //Spindexer Slot 3 intake: 0.98 outtake: 0.8

    @Override
    public void runOpMode() {

        while (opModeInInit()) {
            Servo1 = hardwareMap.get(Servo.class, servo1Name);

            if (enableServo2) {
                Servo2 = hardwareMap.get(Servo.class, servo2Name);
            }

            if (enableServo3) {
                Servo3 = hardwareMap.get(Servo.class, servo3Name);
            }

            if (enableServo4) {
                Servo4 = hardwareMap.get(Servo.class, servo4Name);
            }

        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.aWasPressed()) {
                Servo1.setPosition(servo1Position);

                if (enableServo2) {
                    Servo2.setPosition(servo2Position);
                }

                if (enableServo3) {
                    Servo3.setPosition(servo3Position);
                }

                if (enableServo4) {
                    Servo4.setPosition(servo4Position);
                }
            }
//            Servo1.setPosition(0);
//            sleep(1500);
//            Servo1.setPosition(1);
//            sleep(1500);
//            Servo1.setPosition(0);
//            sleep(1500);
//            Servo1.setPosition(1);
//            sleep(1500);
//            Servo1.setPosition(0);
//            sleep(1500);
//            Servo1.setPosition(1);
//            sleep(1500);
//            Servo1.setPosition(0);
//            sleep(1500);
//            Servo1.setPosition(1);
//            sleep(1500);
//            Servo1.setPosition(0);
//            sleep(1500);
//            Servo1.setPosition(1);
//            sleep(1500);
//            Servo1.setPosition(0);
        }
    }
}
