package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
@Autonomous(name = "IncredibotsAutoDoNothing", group = "Autonomous")
/* This auto opmode will do the following steps:
 1) Robot will start from the inside edge of third tile from closest vertical wall
 2) The robot will hang the preloaded specimen on the top rung
 3) The robot will then go back to the observation area to pick another specimen
 4) The robot will hang the second specimen
 5) The robot will go park in the ascent zone.
 */
public class IncredibotsAutoDoNothing extends LinearOpMode {

    public static double START_X = 0;
    public static double START_Y = 0;
    public static double START_H = Math.toRadians(90);
    Pose2d startPose = new Pose2d(START_X, START_Y, START_H);

    RobotHardware myHardware;
    IncredibotsArmControl armControl;
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new RobotHardware(this.hardwareMap);
        armControl = new IncredibotsArmControl(gamepad2, myHardware);
        drive = new MecanumDrive(this.hardwareMap, startPose);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}