package org.firstinspires.ftc.teamcode.drive.opmode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name = "IncredibotsAuto3SpecimenRed", group = "Autonomous")
/* This auto opmode will do the following steps:
 1) Robot will start from the inside edge of third tile from closest vertical wall
 2) The robot will hang the preloaded specimen on the top rung
 3) The robot will then go back to the observation area to pick another specimen
 4) The robot will hang the second specimen
 5) The robot will go park in the observation zone.
 */
public class IncredibotsAuto3SpecimenRed extends LinearOpMode {

    RobotHardware myHardware;
    IncredibotsArmControl armControl;
    MecanumDrive drive;

    public static double heading = Math.toRadians(90);

    // ==== OLDER STRATEGY VALUES ======
    public static Pose2d GET_CLOSE_TO_FIRST_SAMPLE = new Pose2d(37, -27, heading);
    public static Pose2d MOVE_BEYOND_FIRST_SAMPLE = new Pose2d(GET_CLOSE_TO_FIRST_SAMPLE.position.x, -15, heading);
    public static Pose2d STRAFE_BEHIND_FIRST_SAMPLE = new Pose2d(45, MOVE_BEYOND_FIRST_SAMPLE.position.y, heading);
    public static Pose2d PUSH_FIRST_SAMPLE = new Pose2d(STRAFE_BEHIND_FIRST_SAMPLE.position.x, -56, heading);
    public static Pose2d STRAFE_BEHIND_SECOND_SAMPLE = new Pose2d(PUSH_FIRST_SAMPLE.position.x + 10, MOVE_BEYOND_FIRST_SAMPLE.position.y, heading);
    public static Pose2d STRAFE_BEHIND_THIRD_SAMPLE = new Pose2d(STRAFE_BEHIND_SECOND_SAMPLE.position.x + 10, MOVE_BEYOND_FIRST_SAMPLE.position.y, heading);
    public static Pose2d PICK_SPECIMEN = new Pose2d(STRAFE_BEHIND_THIRD_SAMPLE.position.x, -45, heading);
    public static Pose2d MOVE_FORWARD_TO_PICK_SPECIMEN = new Pose2d(PICK_SPECIMEN.position.x, -62, heading);
    public static Pose2d GET_IN_FRONT_OF_SECOND_SAMPLE = new Pose2d(61, -45, heading);
    public static Pose2d DROP_SECOND_SAMPLE = new Pose2d(GET_IN_FRONT_OF_SECOND_SAMPLE.position.x, -58, heading);
    // ==== OLDER STRATEGY VALUES ======

    public static Pose2d INIT_POS = new Pose2d(27, -63, heading);

    //round 1
    public static Pose2d GET_IN_FRONT_OF_FIRST_SAMPLE = new Pose2d(51, -45, heading);
    public static Pose2d DROP_FIRST_SAMPLE = new Pose2d(GET_IN_FRONT_OF_FIRST_SAMPLE.position.x, -58, heading);
    public static Pose2d MOVE_FORWARD_TO_PICK_FIRST_SPECIMEN = new Pose2d(DROP_FIRST_SAMPLE.position.x, -61, heading);
    public static Pose2d MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_ONE = new Pose2d(4, -40, heading);
    public static Pose2d MOVE_TO_BRACE_SUB_SPECIMEN_ONE = new Pose2d(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_ONE.position.x, -33, heading);

    //round 2
    public static Pose2d STRAFE_TO_SECOND_SPECIMEN = new Pose2d(GET_IN_FRONT_OF_FIRST_SAMPLE.position.x, -52, heading);
    public static Pose2d MOVE_FORWARD_TO_PICK_SECOND_SPECIMEN = new Pose2d(STRAFE_TO_SECOND_SPECIMEN.position.x, -59, heading);

    public static Pose2d MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_TWO = new Pose2d(0, -40, heading);
    public static Pose2d MOVE_TO_BRACE_SUB_SPECIMEN_TWO = new Pose2d(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_TWO.position.x, -33, heading);

    //round 3
    public static Pose2d MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_THREE = new Pose2d(-4, -40, heading);
    public static Pose2d MOVE_TO_BRACE_SUB_SPECIMEN_THREE = new Pose2d(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_THREE.position.x, -33, heading);

    public static Pose2d OBSERVATION_PARK = new Pose2d(61, -56, heading);




    private Action GetArmControlAction(int position, int velocity, boolean waitForAction) {
        return new ArmMotionAsRRAction(myHardware, position, velocity, waitForAction);
    }

    private Action GetSlideControlAction(int position, boolean waitForAction) {
        return new SlideMotionAsRRAction(myHardware, position, waitForAction);
    }

    private Action GetClawControlAction(boolean open, double openPosition, double closePosition, boolean waitForAction) {
        return new ClawMotionAsRRAction(myHardware, open, openPosition, closePosition, waitForAction);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new RobotHardware(this.hardwareMap);
        armControl = new IncredibotsArmControl(gamepad2, myHardware);
        drive = new MecanumDrive(this.hardwareMap, INIT_POS);

        Action pickAndDropSampleOne = drive.actionBuilder(INIT_POS)
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SAMPLE_A, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .stopAndAdd(GetClawControlAction(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, false))
                .strafeToConstantHeading(GET_IN_FRONT_OF_FIRST_SAMPLE.position)
                .waitSeconds(0.5) //wait for arm to stabilize and claw to open
                .stopAndAdd(GetClawControlAction(false, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_RESTING_BACK, IncredibotsArmControl.CLAW_ARM_VELOCITY + 200, true))
                .setTangent(heading)
                .lineToYConstantHeading(DROP_FIRST_SAMPLE.position.y)
                .stopAndAdd(GetClawControlAction(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, false))
                .build();

        Action pickAndSnapSpecimenOne = drive.actionBuilder(DROP_FIRST_SAMPLE)
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, true))
                .lineToYConstantHeading(MOVE_FORWARD_TO_PICK_FIRST_SPECIMEN.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 20))
                .stopAndAdd(GetClawControlAction(false, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_AUTO_HANG_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .strafeToConstantHeading(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_ONE.position)
                .setTangent(heading)
                .lineToYConstantHeading(MOVE_TO_BRACE_SUB_SPECIMEN_ONE.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 20))
                .stopAndAdd(GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_HANG_SPECIMEN_HIGH, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_AUTO_SNAP_SPECIMEN, IncredibotsArmControl.CLAW_ARM_AUTO_VELOCITY_SNAP_SAMPLE, true))
                .waitSeconds(0.75)
                .stopAndAdd(GetClawControlAction(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, false))
                .build();

        Action pickAndSnapSpecimenTwo = drive.actionBuilder(MOVE_TO_BRACE_SUB_SPECIMEN_ONE)
                .stopAndAdd(GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_RESTING, false))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY + 200, true))
                .strafeToConstantHeading(STRAFE_TO_SECOND_SPECIMEN.position)
                .setTangent(heading)
                .lineToYConstantHeading(MOVE_FORWARD_TO_PICK_SECOND_SPECIMEN.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 20))
                .stopAndAdd(GetClawControlAction(false, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_AUTO_HANG_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .strafeToConstantHeading(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_TWO.position)
                .setTangent(heading)
                .lineToYConstantHeading(MOVE_TO_BRACE_SUB_SPECIMEN_TWO.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 20))
                .stopAndAdd(GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_HANG_SPECIMEN_HIGH, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_AUTO_SNAP_SPECIMEN, IncredibotsArmControl.CLAW_ARM_AUTO_VELOCITY_SNAP_SAMPLE, true))
                .waitSeconds(0.75)
                .stopAndAdd(GetClawControlAction(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, false))
                .build();

        Action pickAndSnapSpecimenThree = drive.actionBuilder(MOVE_TO_BRACE_SUB_SPECIMEN_TWO)
                .stopAndAdd(GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_RESTING, false))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY + 200, true))
                .strafeToConstantHeading(STRAFE_TO_SECOND_SPECIMEN.position)
                .setTangent(heading)
                .lineToYConstantHeading(MOVE_FORWARD_TO_PICK_SECOND_SPECIMEN.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 20))
                .stopAndAdd(GetClawControlAction(false, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_AUTO_HANG_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .strafeToConstantHeading(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_THREE.position)
                .setTangent(heading)
                .lineToYConstantHeading(MOVE_TO_BRACE_SUB_SPECIMEN_THREE.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(-10, 20))
                .stopAndAdd(GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_HANG_SPECIMEN_HIGH, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_AUTO_SNAP_SPECIMEN, IncredibotsArmControl.CLAW_ARM_AUTO_VELOCITY_SNAP_SAMPLE, true))
                .waitSeconds(0.75)
                .stopAndAdd(GetClawControlAction(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, false))
                .build();

        Action goParkInObservationZone = drive.actionBuilder(MOVE_TO_BRACE_SUB_SPECIMEN_THREE)
                .afterDisp(0.1, GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_RESTING, false))
                .afterDisp(0.1, GetArmControlAction(IncredibotsArmControl.CLAW_ARM_RESTING_BACK, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .strafeToConstantHeading(OBSERVATION_PARK.position)
                .build();

        Action pushSampleOne = drive.actionBuilder(INIT_POS)
                .strafeToConstantHeading(GET_CLOSE_TO_FIRST_SAMPLE.position)
                .lineToYConstantHeading(MOVE_BEYOND_FIRST_SAMPLE.position.y)
                .strafeToConstantHeading(STRAFE_BEHIND_FIRST_SAMPLE.position)
                .setTangent(heading)
                .lineToYConstantHeading(PUSH_FIRST_SAMPLE.position.y) //drop the picked sample
                .build();

        Action pickAndDropSampleTwo = drive.actionBuilder(drive.pose)
                .strafeToConstantHeading(GET_IN_FRONT_OF_SECOND_SAMPLE.position)
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SAMPLE_A, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .stopAndAdd(GetClawControlAction(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, false))
                .waitSeconds(1) //wait for arm to stabilize and claw to open
                .stopAndAdd(GetClawControlAction(false, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_RESTING_BACK, IncredibotsArmControl.CLAW_ARM_VELOCITY + 200, true))
                .setTangent(heading)
                .lineToYConstantHeading(DROP_SECOND_SAMPLE.position.y)
                .stopAndAdd(GetClawControlAction(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, false))
                .build();

        //this action is to push in samples 2 and 3
        Action pushSamplesTwoAndThree = drive.actionBuilder(PUSH_FIRST_SAMPLE)
                .lineToYConstantHeading(MOVE_BEYOND_FIRST_SAMPLE.position.y)
                .strafeToConstantHeading(STRAFE_BEHIND_SECOND_SAMPLE.position)
                .setTangent(heading)
                .lineToYConstantHeading(PUSH_FIRST_SAMPLE.position.y)
                .lineToYConstantHeading(MOVE_BEYOND_FIRST_SAMPLE.position.y)
                .strafeToConstantHeading(STRAFE_BEHIND_THIRD_SAMPLE.position)
                .setTangent(heading)
                .lineToYConstantHeading(PUSH_FIRST_SAMPLE.position.y)
                .build();

        Action pickFirstSpecimen = drive.actionBuilder(PUSH_FIRST_SAMPLE)
                .strafeToConstantHeading(PICK_SPECIMEN.position)
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .stopAndAdd(GetClawControlAction(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, false))
                .setTangent(heading)
                .lineToYConstantHeading(MOVE_FORWARD_TO_PICK_SPECIMEN.position.y)
                .stopAndAdd(GetClawControlAction(false, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_AUTO_HANG_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .strafeToConstantHeading(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_ONE.position)
                .setTangent(heading)
                .lineToYConstantHeading(MOVE_TO_BRACE_SUB_SPECIMEN_ONE.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(-30, 50))
                .stopAndAdd(GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_HANG_SPECIMEN_HIGH, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_AUTO_SNAP_SPECIMEN, IncredibotsArmControl.CLAW_ARM_AUTO_VELOCITY_SNAP_SAMPLE, true))
                .waitSeconds(0.75)
                .stopAndAdd(GetClawControlAction(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, false))
                .build();

        Action pickSecondSpecimen = drive.actionBuilder(MOVE_TO_BRACE_SUB_SPECIMEN_ONE)
                .stopAndAdd(GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_RESTING, false))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .lineToYConstantHeading(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_ONE.position.y)
                .strafeToConstantHeading(PICK_SPECIMEN.position)
                .setTangent(heading)
                .lineToYConstantHeading(MOVE_FORWARD_TO_PICK_SPECIMEN.position.y)
                .stopAndAdd(GetClawControlAction(false, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_AUTO_HANG_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .strafeToConstantHeading(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_TWO.position)
                .setTangent(heading)
                .lineToYConstantHeading(MOVE_TO_BRACE_SUB_SPECIMEN_TWO.position.y)
                .stopAndAdd(GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_HANG_SPECIMEN_HIGH, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_AUTO_SNAP_SPECIMEN, IncredibotsArmControl.CLAW_ARM_AUTO_VELOCITY_SNAP_SAMPLE, true))
                .waitSeconds(0.75)
                .stopAndAdd(GetClawControlAction(true, IncredibotsArmControl.CLAW_OPEN_POSITION, IncredibotsArmControl.CLAW_CLOSE_POSITION, false))
                .build();

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            pickAndDropSampleOne,
                            pickAndSnapSpecimenOne,
                            pickAndSnapSpecimenTwo,
                            pickAndSnapSpecimenThree,
                            goParkInObservationZone) //, pushSampleOne)
            );

//            Actions.runBlocking(
//                    new SequentialAction(pushSamplesTwoAndThree)
//            );
//
//            Actions.runBlocking(
//                    new SequentialAction(pickFirstSpecimen)
//            );
//
//            Actions.runBlocking(
//                    new SequentialAction(pickSecondSpecimen)
//            );
//
//            Actions.runBlocking(
//                    new SequentialAction(goPark)
//            );
        }
    }
}