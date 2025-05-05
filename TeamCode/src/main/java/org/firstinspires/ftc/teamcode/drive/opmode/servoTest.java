package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TeleOp
@Config
public class servoTest extends LinearOpMode {
    private static final Logger log = LoggerFactory.getLogger(servoTest.class);
    public CRServo axon;
    public static double ServoPosition = 0;

    public static double power = 1;


    @Override
    public void runOpMode() throws InterruptedException {
        axon = hardwareMap.get(CRServo.class, "axon");
        waitForStart();
        while(opModeIsActive()){

            axon.setPower(power);

        }
    }
}
