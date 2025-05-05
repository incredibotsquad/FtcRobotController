package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class HuskyLensVision extends LinearOpMode {
    private HuskyLens huskyLens;
    private double HuskyLensWidthInches = (double) 20/3;
    private double HuskyLensLenghtInches = 5;
    private double HuskySampleX;
    private double HuskySampleY;
    private double HuskyServoRadians;
    private double HuskyServoDegrees;
    private double HuskyServoPosition;
    private Servo servo;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        while(opModeIsActive() || opModeInInit()){

            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
            HuskyLens.Block[] blocks = huskyLens.blocks();

            for(HuskyLens.Block block:blocks){

                if(block.id == 1){

                    telemetry.addData("DETECTED:", "RED");

                }
                else {
                    telemetry.addData("X pos: ", block.x);
                    telemetry.addData("Y pos: ", block.y);

                    HuskySampleX = ((double) block.x/320) * HuskyLensWidthInches;
                    HuskySampleY = ((double) block.y/240) * HuskyLensLenghtInches;

                    HuskyServoRadians = Math.atan(HuskySampleY/HuskySampleX);
                    HuskyServoDegrees = -Math.toDegrees(HuskyServoRadians) + 90;

                    servo.setPosition(Math.abs(HuskyServoDegrees/180));

                    telemetry.addData("Need to move x inches: ", HuskySampleX);
                    telemetry.addData("Need to move y inches: ", HuskySampleY);
                    telemetry.addData("Need to move Servo Radians: ", HuskyServoRadians);
                    telemetry.addData("Need to move Servo Degrees", HuskyServoDegrees);
                }

                telemetry.addData("Found object ID: ", block.id);
                telemetry.update();
            }
            telemetry.clearAll();
        }
    }
}
