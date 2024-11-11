package org.firstinspires.ftc.teamcode.drive.opmode;
import android.util.Log;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "IncredibotsAutoSpecimenBlue", group = "Autonomous")
/* This auto opmode will do the following steps:
 1) Robot will start from the inside edge of third tile from closest vertical wall
 2) The robot will hang the preloaded specimen on the top rung
 3) The robot will then go back to the observation area to pick another specimen
 4) The robot will hang the second specimen
 5) The robot will go park in the ascent zone.
 */
public class IncredibotsAutoSpecimenBlue extends LinearOpMode {
//
//    public static int AUTO_CLAW_ARM_POSITION_TO_HANG_SPECIMEN = 1700;
//    public static int AUTO_CLAW_ARM_POSITION_TO_SNAP_SPECIMEN = 1250;
//    public static int CLAW_ARM_DIFFERENCE_FOR_SNAP = 250;
//    public static int CLAW_ARM_DIFFERENCE_FOR_PICK = 80;
//    public static int CLAW_ARM_VERTICAL = 2500;
//
//
//    public static double START_X = -15;
//    public static double START_Y = 60.75;
//    public static double START_H = Math.toRadians(90);
//    Pose2d startPose = new Pose2d(START_X, START_Y, START_H);
//
//    //position to snap the preloaded specimen
//    public static double STEP1_X = 7;
//    public static double STEP1_Y = 29;
//    public static double STEP1_H = START_H;
//    Pose2d step1Pose = new Pose2d(STEP1_X, STEP1_Y, STEP1_H);
//
//    //position to pick the second specimen
//    public static double STEP2_X = -54;
//    public static double STEP2_Y = 40;
//    public static double STEP2_H = STEP1_H;
//    Pose2d step2Pose = new Pose2d(STEP2_X, STEP2_Y, STEP2_H);
//
//    //position to pick the second specimen - just moving forward
//    public static double STEP3_X = STEP2_X;
//    public static double STEP3_Y = 44.75;
//    public static double STEP3_H = STEP2_H;
//    Pose2d step3Pose = new Pose2d(STEP3_X, STEP3_Y, STEP3_H);
//
//    //position to snap the second specimen
//    public static double STEP4_X = -2;
//    public static double STEP4_Y = 46.5;
//    public static double STEP4_H = Math.toRadians(-90);
//    Pose2d step4Pose = new Pose2d(STEP4_X, STEP4_Y, STEP4_H);
//
//    //parking position
//    public static double END_X = -54;
//    public static double END_Y = 58;
//    public static double END_H = STEP4_H;
//    Pose2d endPose = new Pose2d(END_X, END_Y, END_H);
//


    RobotHardware myHardware;
    IncredibotsArmControl armControl;
    MecanumDrive drive;

    // round 1 - drop preloaded specimen and move on sample 1 to observation zone
    public static Pose2d INIT_POS = new Pose2d(-15, 60.75, Math.toRadians(90));

    public static Pose2d HANG_SPEC_1 = new Pose2d(-5, 33, Math.toRadians(90));

    public static Pose2d BACK_POST_HANG = new Pose2d(-36, 35, Math.toRadians(90));

    public static Pose2d ALIGN_NEXT_TO_SAMP_1 = new Pose2d(-36, 9, Math.toRadians(90));
    //-
    public static Pose2d SLIDE_UNDER_SAMP_1 = new Pose2d(-42, 9, Math.toRadians(90));
    //-
    public static Pose2d PUSH_SAMP_1 = new Pose2d(-43, 60, Math.toRadians(90));

    // round 2 - move sample 2  to observation zone, pick and hang specimen 2
    //-
    public static Pose2d ALIGN_NEXT_TO_SAMP_2 = new Pose2d(-43, 13, Math.toRadians(90));
    //-
    public static Pose2d SLIDE_UNDER_SAMP_2 = new Pose2d(-51, 13, Math.toRadians(90));
    // adjust a bit on x to allow room for hang

    public static Pose2d HANG_SPEC_2 = new Pose2d(-2, 33, Math.toRadians(90));
    //-
    public static Pose2d PUSH_SAMP_2 = new Pose2d(-51, 60, Math.toRadians(90));

    // round 3 - move sample 3  to observation zone, pick and hang specimen 3

    //-
    public static Pose2d PICK_SPEC = new Pose2d(-62, 50, Math.toRadians(90));
    //-
    public static Pose2d ALIGN_NEXT_TO_FINAL_SAMP = new Pose2d(-51, 13, Math.toRadians(90));
    // adjust a bit on x to allow room for hang
    public static Pose2d HANG_SPEC_3 = new Pose2d(1, 33, Math.toRadians(90));
    //-
    public static Pose2d BRACE_WALL_SAMP_3 = new Pose2d(-62, 13, Math.toRadians(90));
    //-
    public static Pose2d PUSH_SAMP_3 = new Pose2d(-62, 60, Math.toRadians(90));
    //-
    public static Pose2d BACK_OPEN_ARM = new Pose2d(-62, 40, Math.toRadians(90));

    // round 4 - pick and hand spc 4
    // adjust a bit on x to allow room for hang
    public static Pose2d HANG_SPEC_4 = new Pose2d(3, 33, Math.toRadians(90));

    public static Pose2d HANG_SPEC_5 = new Pose2d(3, 33, Math.toRadians(90));

    //-
    public static Pose2d PARK_POS = new Pose2d(-52, 62, Math.toRadians(90));


    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new RobotHardware(this.hardwareMap);
        armControl = new IncredibotsArmControl(gamepad2, myHardware);
        drive = new MecanumDrive(this.hardwareMap, INIT_POS);

        // round 1: Robot hangs first preload and pushes first sample (2nd Trajectory)
        Action stepOne = drive.actionBuilder(INIT_POS)
                .strafeToLinearHeading(HANG_SPEC_1.position, HANG_SPEC_1.heading)
                .strafeToLinearHeading(BACK_POST_HANG.position, BACK_POST_HANG.heading)
                .strafeToLinearHeading(ALIGN_NEXT_TO_SAMP_1.position, ALIGN_NEXT_TO_SAMP_1.heading)
                .strafeToLinearHeading(SLIDE_UNDER_SAMP_1.position, SLIDE_UNDER_SAMP_1.heading)
                .strafeToLinearHeading(PUSH_SAMP_1.position, PUSH_SAMP_1.heading)
                .build();

        // round 2: Robot aligns with remaining two samples and pushes them in (2nd Trajectory)
        Action stepTwo = drive.actionBuilder(PUSH_SAMP_1) //PUSH_SAMP_1
                .strafeToLinearHeading(ALIGN_NEXT_TO_SAMP_2.position, ALIGN_NEXT_TO_SAMP_2.heading)
                .strafeToLinearHeading(SLIDE_UNDER_SAMP_2.position, SLIDE_UNDER_SAMP_2.heading)
                .strafeToLinearHeading(PUSH_SAMP_2.position, PUSH_SAMP_2.heading)
                .strafeToLinearHeading(ALIGN_NEXT_TO_FINAL_SAMP.position, ALIGN_NEXT_TO_FINAL_SAMP.heading)
                .strafeToLinearHeading(BRACE_WALL_SAMP_3.position, BRACE_WALL_SAMP_3.heading)
                .strafeToLinearHeading(PUSH_SAMP_3.position, PUSH_SAMP_3.heading)
                .strafeToLinearHeading(BACK_OPEN_ARM.position, BACK_OPEN_ARM.heading)
                .build();

        Action stepThree = drive.actionBuilder(BACK_OPEN_ARM)
                //Round 3: Goes to pick up 2nd, 3rd, 4th, and 5th specimens
                .strafeToLinearHeading(PICK_SPEC.position, PICK_SPEC.heading)
                .waitSeconds(0.35)

                .strafeToLinearHeading(HANG_SPEC_2.position, HANG_SPEC_2.heading)
                .waitSeconds(0.35)
                .strafeToLinearHeading(PICK_SPEC.position, PICK_SPEC.heading)
                .waitSeconds(0.35)
                .strafeToLinearHeading(HANG_SPEC_3.position, HANG_SPEC_3.heading)
                .waitSeconds(0.35)
                .strafeToLinearHeading(PICK_SPEC.position, PICK_SPEC.heading)
                .waitSeconds(0.35)
                .strafeToLinearHeading(HANG_SPEC_4.position, HANG_SPEC_4.heading)
                .waitSeconds(0.35)
                .strafeToLinearHeading(PICK_SPEC.position, PICK_SPEC.heading)
                .waitSeconds(0.35)
                .strafeToLinearHeading(HANG_SPEC_5.position, HANG_SPEC_5.heading)
                .waitSeconds(0.35)
                .strafeToLinearHeading(PARK_POS.position, PARK_POS.heading)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            stepOne
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            stepTwo
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            stepThree
                    )
            );
        }
    }

//    @Override
//    public void runOpMode() throws InterruptedException {
//        myHardware = new RobotHardware(this.hardwareMap);
//        armControl = new IncredibotsArmControl(gamepad2, myHardware);
//        drive = new MecanumDrive(this.hardwareMap, startPose);
//
//        Action dropLoadedSpecimen = drive.actionBuilder(startPose)
//                .strafeToConstantHeading(step1Pose.position)
//                .build();
//
//        Action positionToPickNextSpecimen = drive.actionBuilder(step1Pose)
//                .lineToY(STEP2_Y)
//                .strafeToConstantHeading(step2Pose.position)
//                .turn(Math.toRadians(-2)) //there seems to be a strafing error that we can account for
//                .build();
//
//        Action moveForwardToPickNextSpecimen = drive.actionBuilder(step2Pose)
//                .lineToY(step3Pose.position.y)
//                .build();
//
//        Action moveToSnapSecondSpecimen = drive.actionBuilder(step3Pose)
//                .strafeToLinearHeading(step4Pose.position, STEP4_H)
//                .build();
//
//        Action park = drive.actionBuilder(step4Pose)
//                .strafeToConstantHeading(endPose.position)
//                .build();
//
//        //make sure claw is closed.
//        myHardware.operateClawServo(false, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION);
//
//        // Wait for the game to start (driver presses PLAY)
//        telemetry.addData("Status", "Initialized");
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            Log.i("=== heading ===", "start: " + myHardware.getRobotYawRadians());
//
//            myHardware.setClawArmPositionAndVelocity(AUTO_CLAW_ARM_POSITION_TO_HANG_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY);
//
//            Actions.runBlocking(
//                new SequentialAction(
//                        dropLoadedSpecimen
//                )
//            );
//
//            myHardware.setClawArmPositionAndVelocityAndWait(AUTO_CLAW_ARM_POSITION_TO_SNAP_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY);
//            myHardware.operateClawServo(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION);
//            myHardware.setClawArmPositionAndVelocityAndWait(CLAW_ARM_VERTICAL, IncredibotsArmControl.CLAW_ARM_VELOCITY);
//
//            Actions.runBlocking(
//                new SequentialAction(
//                        positionToPickNextSpecimen
//                )
//            );
//
//            myHardware.setClawArmPositionAndVelocityAndWait(IncredibotsArmControl.CLAW_ARM_PICK_SPECIMEN_ENTER_SUB_X - CLAW_ARM_DIFFERENCE_FOR_PICK, IncredibotsArmControl.CLAW_ARM_VELOCITY);
//            myHardware.operateClawServo(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION);
//
//            Actions.runBlocking(
//                new SequentialAction(
//                    moveForwardToPickNextSpecimen
//                )
//            );
//
//            myHardware.operateClawServo(false, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION);
//
//            //wait for the claw to close
//            try {
//                Thread.sleep(1000);
//            } catch (InterruptedException e) {
//                Log.i("Incredibots Auto Specimen", "Sleeping thread interrupted");
//            }
//
//            myHardware.setClawArmPositionAndVelocityAndWait(CLAW_ARM_VERTICAL, IncredibotsArmControl.CLAW_ARM_VELOCITY);//was CLAW_ARM_VERTICAL
//
//            Actions.runBlocking(
//                    new SequentialAction(
//                            moveToSnapSecondSpecimen
//                    )
//            );
//
//            myHardware.setClawArmPositionAndVelocityAndWait(IncredibotsArmControl.CLAW_ARM_SNAP_SPECIMEN_B - CLAW_ARM_DIFFERENCE_FOR_SNAP, IncredibotsArmControl.CLAW_ARM_VELOCITY);
//            myHardware.operateClawServo(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION);
//            myHardware.setClawArmPositionAndVelocityAndWait(CLAW_ARM_VERTICAL, IncredibotsArmControl.CLAW_ARM_VELOCITY);
//            myHardware.operateClawServo(false, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION);
//            myHardware.setClawArmPositionAndVelocityAndWait(IncredibotsArmControl.CLAW_ARM_RESTING_BACK, IncredibotsArmControl.CLAW_ARM_VELOCITY);
//            myHardware.operateClawServo(false, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION);
//
//            Log.i("=== heading ===", "after turning: " + myHardware.getRobotYawRadians());
//
//            Actions.runBlocking(
//                    new SequentialAction(
//                            park
//                    )
//            );
//
//            telemetry.update();
//
//            break;
//        }
//
//
//
////        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
////                .lineToYSplineHeading(33, Math.toRadians(0))
////                .waitSeconds(2)
////                .setTangent(Math.toRadians(90))
////                .lineToY(48)
////                .lineToX(32)
////                .setTangent(Math.toRadians(0))
////                .strafeTo(new Vector2d(44.5, 30))
////                .turn(Math.toRadians(180))
////                .lineToX(47.5)
////                .waitSeconds(3);
//
//        //strafes in front of blue tape for net zone
//
//        //https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/
//        //https://rr.brott.dev/docs/v1-0/guides/centerstage-auto/
//
//    }
}