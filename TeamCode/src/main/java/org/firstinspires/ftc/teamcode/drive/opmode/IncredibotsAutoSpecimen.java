package org.firstinspires.ftc.teamcode.drive.opmode;
import android.util.Log;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "IncredibotsAutoSpecimen", group = "Autonomous")
/* This auto opmode will do the following steps:
 1) Robot will start from the inside edge of third tile from closest vertical wall
 2) The robot will hang the preloaded specimen on the top rung
 3) The robot will then go back to the observation area to pick another specimen
 4) The robot will hang the second specimen
 5) The robot will go park in the ascent zone.
 */
public class IncredibotsAutoSpecimen extends LinearOpMode {

    public static int CLAW_ARM_DIFFERENCE_FOR_SNAP = 250;
    public static int CLAW_ARM_DIFFERENCE_FOR_PICK = 150;

    public static double START_X = -15;
    public static double START_Y = 65.75;
    public static double START_H = Math.toRadians(-90);
    Pose2d startPose = new Pose2d(START_X, START_Y, START_H);

    public static double START2_X = -17.5;
    public static double START2_Y = 62.75;
    public static double START2_H = Math.toRadians(0);
    Pose2d start2Pose = new Pose2d(START2_X, START2_Y, START2_H);


    //position to snap the preloaded specimen
//    public static double STEP1_X = 0;
//    public static double STEP1_Y = 62.75; //52.5
//    public static double STEP1_H = START_H;
    public static double STEP1_X = -8;
    public static double STEP1_Y = 52.5; //52.5
    public static double STEP1_H = START_H;
    Pose2d step1Pose = new Pose2d(STEP1_X, STEP1_Y, STEP1_H);

    //position to pick the second specimen
    public static double STEP2_X = -60;
    public static double STEP2_Y = 46;
    public static double STEP2_H = Math.toRadians(90);
    Pose2d step2Pose = new Pose2d(STEP2_X, STEP2_Y, STEP2_H);

    //position to pick the second specimen
    public static double STEP3_X = STEP2_X;
    public static double STEP3_Y = 52;
    public static double STEP3_H = STEP2_H;
    Pose2d step3Pose = new Pose2d(STEP3_X, STEP3_Y, STEP3_H);

    //position to snap the second specimen
    public static double STEP4_X = 0;
    public static double STEP4_Y = 60;
    public static double STEP4_H = STEP1_H;
    Pose2d step4Pose = new Pose2d(STEP4_X, STEP4_Y, STEP4_H);

    //parking position
    public static double END_X = -58;
    public static double END_Y = 60;
    public static double END_H = STEP4_H;
    Pose2d endPose = new Pose2d(END_X, END_Y, END_H);

    RobotHardware myHardware;
    IncredibotsArmControl armControl;
    MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new RobotHardware(this.hardwareMap);
        armControl = new IncredibotsArmControl(gamepad2, myHardware);
        drive = new MecanumDrive(this.hardwareMap, startPose);

        Action dropLoadedSpecimen = drive.actionBuilder(startPose)
                .strafeToLinearHeading(step1Pose.position, STEP1_H)
                .build();

//        Action dropLoadedSpecimen = drive.actionBuilder(start2Pose)
//                .lineToX(step1Pose.position.x)
//                .build();

        Action positionToPickNextSpecimen = drive.actionBuilder(step1Pose)
                .strafeToLinearHeading(step2Pose.position, STEP2_H)
                .build();

        Action moveForwardToPickNextSpecimen = drive.actionBuilder(step2Pose)
                .lineToY(step3Pose.position.y)
                .build();

        Action moveToSnapSecondSpecimen = drive.actionBuilder(step3Pose)
                .strafeToLinearHeading(step4Pose.position, STEP4_H)
                .build();

        Action park = drive.actionBuilder(step4Pose)
                .strafeToConstantHeading(endPose.position)
                .build();

        //make sure claw is closed.
        myHardware.operateClawServo(false, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");

        waitForStart();

        while (opModeIsActive()) {
            Log.i("=== heading ===", "start: " + myHardware.getRobotYawRadians());

            Actions.runBlocking(
                new SequentialAction(
                        dropLoadedSpecimen
                )
            );

            myHardware.setClawArmPositionAndVelocityAndWait(IncredibotsArmControl.CLAW_ARM_SNAP_SPECIMEN_B - CLAW_ARM_DIFFERENCE_FOR_SNAP, IncredibotsArmControl.CLAW_ARM_VELOCITY);
            myHardware.operateClawServo(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION);
            myHardware.setClawArmPositionAndVelocityAndWait(IncredibotsArmControl.CLAW_ARM_VERTICAL, IncredibotsArmControl.CLAW_ARM_VELOCITY);
            myHardware.operateClawServo(false, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION);
            myHardware.setClawArmPositionAndVelocityAndWait(IncredibotsArmControl.CLAW_ARM_RESTING_BACK, IncredibotsArmControl.CLAW_ARM_VELOCITY);

            Actions.runBlocking(
                new SequentialAction(
                        positionToPickNextSpecimen
                )
            );

            myHardware.setClawArmPositionAndVelocityAndWait(IncredibotsArmControl.CLAW_ARM_PICK_SPECIMEN_ENTER_SUB_X - CLAW_ARM_DIFFERENCE_FOR_PICK, IncredibotsArmControl.CLAW_ARM_VELOCITY);
            myHardware.operateClawServo(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION);

            Actions.runBlocking(
                new SequentialAction(
                    moveForwardToPickNextSpecimen
                )
            );

            myHardware.operateClawServo(false, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION);

            //wait for the claw to close
            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                Log.i("Incredibots Auto Specimen", "Sleeping thread interrupted");
            }

            myHardware.setClawArmPositionAndVelocityAndWait(IncredibotsArmControl.CLAW_ARM_VERTICAL, IncredibotsArmControl.CLAW_ARM_VELOCITY);

            Actions.runBlocking(
                    new SequentialAction(
                            moveToSnapSecondSpecimen
                    )
            );

            myHardware.setClawArmPositionAndVelocityAndWait(IncredibotsArmControl.CLAW_ARM_SNAP_SPECIMEN_B - CLAW_ARM_DIFFERENCE_FOR_SNAP, IncredibotsArmControl.CLAW_ARM_VELOCITY);
            myHardware.operateClawServo(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION);
            myHardware.setClawArmPositionAndVelocityAndWait(IncredibotsArmControl.CLAW_ARM_VERTICAL, IncredibotsArmControl.CLAW_ARM_VELOCITY);
            myHardware.operateClawServo(false, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION);
            myHardware.setClawArmPositionAndVelocityAndWait(IncredibotsArmControl.CLAW_ARM_RESTING_BACK, IncredibotsArmControl.CLAW_ARM_VELOCITY);

            Log.i("=== heading ===", "after turning: " + myHardware.getRobotYawRadians());

            Actions.runBlocking(
                    new SequentialAction(
                            park
                    )
            );

            telemetry.update();

            break;
        }



//        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
//                .lineToYSplineHeading(33, Math.toRadians(0))
//                .waitSeconds(2)
//                .setTangent(Math.toRadians(90))
//                .lineToY(48)
//                .lineToX(32)
//                .setTangent(Math.toRadians(0))
//                .strafeTo(new Vector2d(44.5, 30))
//                .turn(Math.toRadians(180))
//                .lineToX(47.5)
//                .waitSeconds(3);

        //strafes in front of blue tape for net zone

        //https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/
        //https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/

    }
}