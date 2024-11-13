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


    //start with a robot centric position
    public static Pose2d INIT_POS = new Pose2d(-27, 63, Math.toRadians(-90));
    public static Pose2d GET_CLOSE_TO_FIRST_SAMPLE = new Pose2d(-37, 27, Math.toRadians(-90));
    public static Pose2d MOVE_BEYOND_FIRST_SAMPLE = new Pose2d(GET_CLOSE_TO_FIRST_SAMPLE.position.x, 15, Math.toRadians(-90));
    public static Pose2d STRAFE_BEHIND_FIRST_SAMPLE = new Pose2d(-45, MOVE_BEYOND_FIRST_SAMPLE.position.y, Math.toRadians(-90));
    public static Pose2d PUSH_FIRST_SAMPLE = new Pose2d(STRAFE_BEHIND_FIRST_SAMPLE.position.x, 56, Math.toRadians(-90));
    public static Pose2d STRAFE_BEHIND_SECOND_SAMPLE = new Pose2d(PUSH_FIRST_SAMPLE.position.x - 10, MOVE_BEYOND_FIRST_SAMPLE.position.y, Math.toRadians(-90));
    public static Pose2d STRAFE_BEHIND_THIRD_SAMPLE = new Pose2d(STRAFE_BEHIND_SECOND_SAMPLE.position.x - 10, MOVE_BEYOND_FIRST_SAMPLE.position.y, Math.toRadians(-90));
    public static Pose2d PICK_SPECIMEN = new Pose2d(STRAFE_BEHIND_THIRD_SAMPLE.position.x, 45, Math.toRadians(-90));
    public static Pose2d MOVE_FORWARD_TO_PICK_SPECIMEN = new Pose2d(PICK_SPECIMEN.position.x, 62, Math.toRadians(-90));
    public static Pose2d MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_ONE = new Pose2d(-5, PICK_SPECIMEN.position.y, Math.toRadians(-90));
    public static Pose2d MOVE_TO_BRACE_SUB_SPECIMEN_ONE = new Pose2d(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_ONE.position.x, 30, Math.toRadians(-90));
    public static Pose2d MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_TWO = new Pose2d(-1, MOVE_FORWARD_TO_PICK_SPECIMEN.position.y, Math.toRadians(-90));
    public static Pose2d MOVE_TO_BRACE_SUB_SPECIMEN_TWO = new Pose2d(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_TWO.position.x, 30, Math.toRadians(-90));

    private Action GetArmControlAction(int position, int velocity, boolean waitForAction) {
        return new ArmMotionAsRRAction(myHardware, position, velocity, waitForAction);
    }

    private Action GetSlideControlAction(int position, int velocity, boolean waitForAction) {
        return new SlideMotionAsRRAction(myHardware, position, velocity, waitForAction);
    }

    private Action GetClawControlAction(boolean open, double openPosition, double closePosition, boolean waitForAction) {
        return new ClawMotionAsRRAction(myHardware, open, openPosition, closePosition, waitForAction);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new RobotHardware(this.hardwareMap);
        armControl = new IncredibotsArmControl(gamepad2, myHardware);
        drive = new MecanumDrive(this.hardwareMap, INIT_POS);


        Action pushSampleOne = drive.actionBuilder(INIT_POS)
                .strafeToConstantHeading(GET_CLOSE_TO_FIRST_SAMPLE.position)
                .lineToYConstantHeading(MOVE_BEYOND_FIRST_SAMPLE.position.y)
                .strafeToConstantHeading(STRAFE_BEHIND_FIRST_SAMPLE.position)
                .setTangent(Math.toRadians(-90))
                .lineToYConstantHeading(PUSH_FIRST_SAMPLE.position.y)
                .build();

        //this action is to push in samples 2 and 3
        Action pushSamplesTwoAndThree = drive.actionBuilder(PUSH_FIRST_SAMPLE)
                .lineToYConstantHeading(MOVE_BEYOND_FIRST_SAMPLE.position.y)
                .strafeToConstantHeading(STRAFE_BEHIND_SECOND_SAMPLE.position)
                .setTangent(Math.toRadians(-90))
                .lineToYConstantHeading(PUSH_FIRST_SAMPLE.position.y)
                .lineToYConstantHeading(MOVE_BEYOND_FIRST_SAMPLE.position.y)
                .strafeToConstantHeading(STRAFE_BEHIND_THIRD_SAMPLE.position)
                .setTangent(Math.toRadians(-90))
                .lineToYConstantHeading(PUSH_FIRST_SAMPLE.position.y)
                .build();

        Action pickFirstSpecimen = drive.actionBuilder(PUSH_FIRST_SAMPLE)
                .strafeToConstantHeading(PICK_SPECIMEN.position)
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .stopAndAdd(GetClawControlAction(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, false))
                .setTangent(Math.toRadians(-90))
                .lineToYConstantHeading(MOVE_FORWARD_TO_PICK_SPECIMEN.position.y)
                .stopAndAdd(GetClawControlAction(false, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_AUTO_HANG_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .strafeToConstantHeading(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_ONE.position)
                .setTangent(Math.toRadians(-90))
                .lineToYConstantHeading(MOVE_TO_BRACE_SUB_SPECIMEN_ONE.position.y)
                .stopAndAdd(GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_HANG_SPECIMEN_HIGH, IncredibotsArmControl.SLIDE_VELOCITY, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_AUTO_SNAP_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, true))
                .waitSeconds(0.75)
                .stopAndAdd(GetClawControlAction(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, false))
                .build();

        Action pickSecondSpecimen = drive.actionBuilder(MOVE_TO_BRACE_SUB_SPECIMEN_ONE)
                .stopAndAdd(GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_RESTING, IncredibotsArmControl.SLIDE_VELOCITY, false))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .lineToYConstantHeading(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_ONE.position.y)
                .strafeToConstantHeading(PICK_SPECIMEN.position)
                .setTangent(Math.toRadians(-90))
                .lineToYConstantHeading(MOVE_FORWARD_TO_PICK_SPECIMEN.position.y)
                .stopAndAdd(GetClawControlAction(false, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_AUTO_HANG_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .strafeToConstantHeading(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_TWO.position)
                .setTangent(Math.toRadians(-90))
                .lineToYConstantHeading(MOVE_TO_BRACE_SUB_SPECIMEN_TWO.position.y)
                .stopAndAdd(GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_HANG_SPECIMEN_HIGH, IncredibotsArmControl.SLIDE_VELOCITY, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_AUTO_SNAP_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, true))
                .build();

        Action goPark = drive.actionBuilder(MOVE_TO_BRACE_SUB_SPECIMEN_TWO)
                .stopAndAdd(GetClawControlAction(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, false))
                .build();

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(pushSampleOne)
            );

//            Actions.runBlocking(
//                    new SequentialAction(pushSamplesTwoAndThree)
//            );

            Actions.runBlocking(
                    new SequentialAction(pickFirstSpecimen)
            );

            Actions.runBlocking(
                    new SequentialAction(pickSecondSpecimen)
            );

            Log.i("====INCREDIBOTS ====", "");
        }
    }
}