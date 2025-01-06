package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.IncredibotsArmControl;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;

@Config
//@Autonomous(name = "IncredibotsAuto3SpecimenBlue", group = "Autonomous")
/* This auto opmode will make the robot do the following steps:
 1) start from the end line of the observation zone
 2) pick first sample and drop in observation zone
 3) The robot will then cycle through picking specimen and hanging them 3 times
 4) The robot will go park in the observation zone.
 */
public class IncredibotsAuto3SpecimenBlue extends IncredibotsAuto {

    public static double heading = Math.toRadians(-90);

    public static Pose2d INIT_POS = new Pose2d(-16, 63, heading);

    //snap preloaded specimen


    //round 1
    public static Pose2d GET_IN_FRONT_OF_FIRST_SAMPLE = new Pose2d(-51, 46, heading);
    public static Pose2d DROP_FIRST_SAMPLE = new Pose2d(GET_IN_FRONT_OF_FIRST_SAMPLE.position.x, 56, heading);
    public static Pose2d MOVE_FORWARD_TO_PICK_FIRST_SPECIMEN = new Pose2d(DROP_FIRST_SAMPLE.position.x, 62, heading);
    public static Pose2d MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_ONE = new Pose2d(-4, 40, heading);
    public static Pose2d MOVE_TO_BRACE_SUB_SPECIMEN_ONE = new Pose2d(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_ONE.position.x, 32, heading);

    //round 2
    public static Pose2d STRAFE_TO_SECOND_SPECIMEN = new Pose2d(GET_IN_FRONT_OF_FIRST_SAMPLE.position.x + 7, 52, heading);
    public static Pose2d MOVE_FORWARD_TO_PICK_SECOND_SPECIMEN = new Pose2d(STRAFE_TO_SECOND_SPECIMEN.position.x, 61, heading);

    public static Pose2d MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_TWO = new Pose2d(0, 40, heading);
    public static Pose2d MOVE_TO_BRACE_SUB_SPECIMEN_TWO = new Pose2d(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_TWO.position.x, 32, heading);

    //round 3
    public static Pose2d MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_THREE = new Pose2d(4, 40, heading);
    public static Pose2d MOVE_TO_BRACE_SUB_SPECIMEN_THREE = new Pose2d(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_THREE.position.x, 32, heading);

    public static Pose2d OBSERVATION_PARK = new Pose2d(-58, 56, heading);

    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new RobotHardware(this.hardwareMap);
        armControl = new IncredibotsArmControl(gamepad2, myHardware);
        drive = new MecanumDrive(this.hardwareMap, INIT_POS);

        Action snapPreloadedSpecimen = drive.actionBuilder(INIT_POS)
                .strafeToConstantHeading(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_ONE.position)
                .setTangent(heading)
                .lineToYConstantHeading(MOVE_TO_BRACE_SUB_SPECIMEN_ONE.position.y, new TranslationalVelConstraint(40), new ProfileAccelConstraint(-20, 40))
                .build();

        Action pickAndDropSampleOne = drive.actionBuilder(INIT_POS)
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SAMPLE, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .stopAndAdd(GetClawControlAction(true, false, false))
                .strafeToConstantHeading(GET_IN_FRONT_OF_FIRST_SAMPLE.position)
                .waitSeconds(0.5) //wait for arm to stabilize and claw to open
                .stopAndAdd(GetClawControlAction(false, true, false))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_RESTING_BACK, IncredibotsArmControl.CLAW_ARM_VELOCITY + 200, true))
                .setTangent(heading)
                .lineToYConstantHeading(DROP_FIRST_SAMPLE.position.y)
                .stopAndAdd(GetClawControlAction(true, false, false))
                .build();

        Action pickAndSnapSpecimenOne = drive.actionBuilder(DROP_FIRST_SAMPLE)
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, true))
                .lineToYConstantHeading(MOVE_FORWARD_TO_PICK_FIRST_SPECIMEN.position.y, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-10, 30))
                .stopAndAdd(GetClawControlAction(false, true, false))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_HANG_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .strafeToConstantHeading(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_ONE.position)
                .setTangent(heading)
                .lineToYConstantHeading(MOVE_TO_BRACE_SUB_SPECIMEN_ONE.position.y, new TranslationalVelConstraint(40), new ProfileAccelConstraint(-20, 40))
                .stopAndAdd(GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_HANG_SPECIMEN, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_SNAP_SPECIMEN, IncredibotsArmControl.CLAW_ARM_AUTO_VELOCITY_SNAP_SPECIMEN, true))
                //.waitSeconds(0.6)
                .stopAndAdd(GetClawControlAction(true, true, true))
                .build();

        Action pickAndSnapSpecimenTwo = drive.actionBuilder(MOVE_TO_BRACE_SUB_SPECIMEN_ONE)
                .stopAndAdd(GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_RESTING, false))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .strafeToConstantHeading(STRAFE_TO_SECOND_SPECIMEN.position)
                .setTangent(heading)
                .lineToYConstantHeading(MOVE_FORWARD_TO_PICK_SECOND_SPECIMEN.position.y, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-10, 30))
                .stopAndAdd(GetClawControlAction(false, true, false))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_HANG_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .strafeToConstantHeading(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_TWO.position)
                .setTangent(heading)
                .lineToYConstantHeading(MOVE_TO_BRACE_SUB_SPECIMEN_TWO.position.y, new TranslationalVelConstraint(40), new ProfileAccelConstraint(-20, 40))
                .stopAndAdd(GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_HANG_SPECIMEN, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_SNAP_SPECIMEN, IncredibotsArmControl.CLAW_ARM_AUTO_VELOCITY_SNAP_SPECIMEN, true))
                //.waitSeconds(0.6)
                .stopAndAdd(GetClawControlAction(true, true, true))
                .build();

        Action pickAndSnapSpecimenThree = drive.actionBuilder(MOVE_TO_BRACE_SUB_SPECIMEN_TWO)
                .stopAndAdd(GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_RESTING, false))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .strafeToConstantHeading(STRAFE_TO_SECOND_SPECIMEN.position)
                .setTangent(heading)
                .lineToYConstantHeading(MOVE_FORWARD_TO_PICK_SECOND_SPECIMEN.position.y, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-10, 30))
                .stopAndAdd(GetClawControlAction(false, true, false))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_HANG_SPECIMEN, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .strafeToConstantHeading(MOVE_IN_FRONT_OF_RUNGS_SPECIMEN_THREE.position)
                .setTangent(heading)
                .lineToYConstantHeading(MOVE_TO_BRACE_SUB_SPECIMEN_THREE.position.y, new TranslationalVelConstraint(40), new ProfileAccelConstraint(-20, 40))
                .stopAndAdd(GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_HANG_SPECIMEN, true))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_SNAP_SPECIMEN, IncredibotsArmControl.CLAW_ARM_AUTO_VELOCITY_SNAP_SPECIMEN, true))
                //.waitSeconds(0.6)
                .stopAndAdd(GetClawControlAction(true, true, true))
                .build();

        Action goParkInObservationZone = drive.actionBuilder(MOVE_TO_BRACE_SUB_SPECIMEN_THREE)
                .afterDisp(0.1, GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_RESTING, false))
                .afterDisp(0.1, GetArmControlAction(IncredibotsArmControl.CLAW_ARM_RESTING_BACK, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .strafeToConstantHeading(OBSERVATION_PARK.position)
                .build();


//        Action pushSampleOne = drive.actionBuilder(INIT_POS)
//                .strafeToConstantHeading(GET_CLOSE_TO_FIRST_SAMPLE.position)
//                .lineToYConstantHeading(MOVE_BEYOND_FIRST_SAMPLE.position.y)
//                .strafeToConstantHeading(STRAFE_BEHIND_FIRST_SAMPLE.position)
//                .setTangent(heading)
//                .lineToYConstantHeading(PUSH_FIRST_SAMPLE.position.y) //drop the picked sample
//                .build();

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            snapPreloadedSpecimen
//                            pickAndDropSampleOne,
//                            pickAndSnapSpecimenOne,
//                            pickAndSnapSpecimenTwo,
//                            pickAndSnapSpecimenThree,
//                            goParkInObservationZone) //, pushSampleOne)
                    )
            );

            break;  //stop the opmode after done

        }
    }
}