package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner.ArmMotionAsRRAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner.ClawMotionAsRRAction;
import org.firstinspires.ftc.teamcode.drive.opmode.IncredibotsArmControl;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner.SlideMotionAsRRAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner.WristMotionAsRRAction;

public abstract class IncredibotsAuto extends LinearOpMode {
    public RobotHardware myHardware;
    public IncredibotsArmControl armControl;
    public MecanumDrive drive;

    protected Action GetArmControlAction(int position, int velocity, boolean waitForAction) {
        return new ArmMotionAsRRAction(myHardware, position, velocity, waitForAction);
    }

    protected Action GetArmControlAction(int position, int velocity, boolean waitForAction, boolean shortWait) {
        return new ArmMotionAsRRAction(myHardware, position, velocity, waitForAction, shortWait);
    }

    protected Action GetSlideControlAction(int position, boolean waitForAction) {
        return new SlideMotionAsRRAction(myHardware, position, waitForAction);
    }

    protected Action GetSideControlAction(int position, int velocity, boolean waitForAction, boolean shortWait) {
        return new SlideMotionAsRRAction(myHardware, position, velocity, waitForAction, shortWait);
    }

    protected Action GetSlideControlAction(int position, boolean waitForAction, boolean shortWait) {
        return new SlideMotionAsRRAction(myHardware, position, waitForAction, shortWait);
    }

    protected Action GetClawControlAction(boolean open, boolean waitForAction, boolean shortWait) {
        return new ClawMotionAsRRAction(myHardware, open, waitForAction, shortWait);
    }

    protected Action GetWristControlAction(double position, boolean waitForAction, boolean shortWait) {
        return new WristMotionAsRRAction(myHardware, position, waitForAction, shortWait);
    }

    @Config
    @Disabled
    @Autonomous(name = "Blue_1_Spec_3_Samples", group = "Autonomous")
    public static class Blue_1_Spec_3_Samples extends IncredibotsAuto {

        public static final int multiplier = 1; //used to flip coordinates between blue (1)

        public static double minTransVelocity = 30;
        public static double minAccel = -20;
        public static double maxAccel = 40;// and red (-1)

        public static double heading = Math.toRadians(-90 * multiplier);
        public static double reverseHeading = Math.toRadians(90 * multiplier);
        public static double basketHeading = Math.toRadians(-90 * multiplier - 21);
        public static double farSampleHeading = Math.toRadians(-90 * multiplier + 17);
        public static Pose2d INIT_POS = new Pose2d(41 * multiplier, 60.75 * multiplier, heading);

        public static Pose2d DROP_SAMPLES = new Pose2d(60.5 * multiplier, 54 * multiplier, basketHeading);

        public static Pose2d BRACE_RUNGS_FOR_SPECIMEN = new Pose2d(0 * multiplier, 30 * multiplier, heading);

        private boolean runOnce = false;

        @Override
        public void runOpMode() throws InterruptedException {
            myHardware = new RobotHardware(this.hardwareMap);
            armControl = new IncredibotsArmControl(gamepad2, myHardware);
            drive = new MecanumDrive(this.hardwareMap, INIT_POS);

            Action hangPreloadedSpecimen = drive.actionBuilder(INIT_POS)
                    .strafeToConstantHeading(BRACE_RUNGS_FOR_SPECIMEN.position, new TranslationalVelConstraint(minTransVelocity + 10), new ProfileAccelConstraint(minAccel -10, maxAccel + 10))
                    .build();

            Action navigateToPickSamples = drive.actionBuilder(BRACE_RUNGS_FOR_SPECIMEN)
                    .setTangent(reverseHeading)
                    .splineToConstantHeading(DROP_SAMPLES.position, (-90 * multiplier) + 90, new TranslationalVelConstraint(minTransVelocity), new ProfileAccelConstraint(minAccel, maxAccel))
                    .turnTo(DROP_SAMPLES.heading, new TurnConstraints(drive.PARAMS.maxAngVel / 2, -drive.PARAMS.maxAngAccel / 2, drive.PARAMS.maxAngAccel / 2))
                    .build();

            Action positionForMiddleSample = drive.actionBuilder(DROP_SAMPLES)
                    .turnTo(heading, new TurnConstraints(drive.PARAMS.maxAngVel / 2, -drive.PARAMS.maxAngAccel / 2, drive.PARAMS.maxAngAccel / 2))
                    .build();

            Action turnToDropMiddleSample = drive.actionBuilder(new Pose2d(DROP_SAMPLES.position, heading))
                    .turnTo(DROP_SAMPLES.heading, new TurnConstraints(drive.PARAMS.maxAngVel / 2, -drive.PARAMS.maxAngAccel / 2, drive.PARAMS.maxAngAccel / 2))
                    .build();

            Action positionForFarSample = drive.actionBuilder(new Pose2d(DROP_SAMPLES.position, heading))
                    .turnTo(farSampleHeading, new TurnConstraints(drive.PARAMS.maxAngVel / 2, -drive.PARAMS.maxAngAccel / 2, drive.PARAMS.maxAngAccel / 2))
                    .build();

            Action turnToDropFarSample = drive.actionBuilder(new Pose2d(DROP_SAMPLES.position, farSampleHeading))
                    .turnTo(heading, new TurnConstraints(drive.PARAMS.maxAngVel, -drive.PARAMS.maxAngAccel / 2, drive.PARAMS.maxAngAccel / 2))
                    .build();

            waitForStart();

            while (opModeIsActive()) {

                if(!runOnce){
                    runOnce = true;
                }
                else {
                    continue;
                }

                ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

                //drop preloaded specimen
                Actions.runBlocking(
                        new SequentialAction(
                                new ParallelAction(
                                        armControl.GetHangSpecimenActionSequence_Fast(),
                                        hangPreloadedSpecimen
                                ),
                                armControl.GetSnapSpecimenActionSequence()
                        )
                );

                Log.i("=== INCREDIBOTS  ===", "DRIVE POSE X: " + drive.pose.position.x + " Y: " + drive.pose.position.y + " Heading: " + Math.toDegrees(drive.pose.heading.toDouble()));

                Actions.runBlocking(
                        new SequentialAction(
                                navigateToPickSamples
                        )
                );

                Log.i("=== INCREDIBOTS  ===", "DRIVE POSE X: " + drive.pose.position.x + " Y: " + drive.pose.position.y + " Heading: " + Math.toDegrees(drive.pose.heading.toDouble()));

                //go to pick samples and drop first one
                Actions.runBlocking(
                        new SequentialAction(
    //                            navigateToPickSamples,
                                GetWristControlAction(IncredibotsArmControl.WRIST_PICK_SAMPLE, false, false),
                                armControl.GetClawArmAfterHighSampleActionSequence(),
                                GetArmControlAction(IncredibotsArmControl.CLAW_ARM_ENTER_SUB, IncredibotsArmControl.CLAW_ARM_VELOCITY / 3, true, false),
                                GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SAMPLE + 30, IncredibotsArmControl.CLAW_ARM_VELOCITY / 3, true, false),
                                GetClawControlAction(false, true, false),
                                armControl.GetHighBasketActionSequence(),
                                GetClawControlAction(true, true, true)
                        )
                );


                //pick and drop second sample
                Actions.runBlocking(
                        new SequentialAction(
                                armControl.GetClawArmAfterHighSampleActionSequence(),
                                positionForMiddleSample,
                                GetSlideControlAction(IncredibotsArmControl.MAX_SLIDE_POSITION_ARM_FORWARDS_LOW - 150, false),   //middle sample doesnt need to go as far
                                GetWristControlAction(IncredibotsArmControl.WRIST_PICK_SAMPLE, false, false),
                                GetArmControlAction(IncredibotsArmControl.CLAW_ARM_ENTER_SUB, IncredibotsArmControl.CLAW_ARM_VELOCITY / 3, true, false),
                                GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SAMPLE + 30, IncredibotsArmControl.CLAW_ARM_VELOCITY / 3, true, false),
                                GetClawControlAction(false, true, false),
                                GetWristControlAction(0.3, false, false),
                                armControl.GetHighBasketActionSequence(),
                                GetClawControlAction(true, true, true)
                        )
                );

                //pick and drop third sample
                Actions.runBlocking(
                        new SequentialAction(
                                armControl.GetClawArmAfterHighSampleActionSequence(),
                                positionForFarSample,
                                GetWristControlAction(IncredibotsArmControl.WRIST_PICK_SAMPLE, false, false),
                                GetArmControlAction(IncredibotsArmControl.CLAW_ARM_ENTER_SUB, IncredibotsArmControl.CLAW_ARM_VELOCITY / 3, true, false),
                                GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SAMPLE + 30, IncredibotsArmControl.CLAW_ARM_VELOCITY / 3, true, false),
                                GetClawControlAction(false, true, false),
                                GetWristControlAction(0.3, false, false),
                                turnToDropFarSample,
                                armControl.GetHighBasketActionSequence(),
                                GetClawControlAction(true, true, true)
                        )
                );

                //bring arm back to resting for TeleOp
                Actions.runBlocking(
                        new SequentialAction(
                                armControl.GetClawArmAfterHighSampleActionSequence(),
                                armControl.GetRestingActionSequence()
                        )
                );

                Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME TOTAL: " + elapsedTime.milliseconds());

                //break;
            }
        }
    }

    @Config
    @Autonomous(name = "Blue_4_Samples", group = "Autonomous")
    public static class Blue_4_Samples  extends IncredibotsAuto {
        public static final int multiplier = 1; //used to flip coordinates between blue (1) and red (-1)

        public static double heading = Math.toRadians(-90 * multiplier);
        public static double basketHeading = Math.toRadians(-90 * multiplier - 22);
        public static double farSampleHeading = Math.toRadians(-90 * multiplier + 21); //17
        public static Pose2d INIT_POS = new Pose2d(41 * multiplier, 60.75 * multiplier, heading);
        public static Pose2d DROP_PRELOADED_SAMPLE = new Pose2d(60.5 * multiplier, 54 * multiplier, heading);

        @Override
        public void runOpMode() throws InterruptedException {
            myHardware = new RobotHardware(this.hardwareMap);
            armControl = new IncredibotsArmControl(gamepad2, myHardware);
            drive = new MecanumDrive(this.hardwareMap, INIT_POS);

            Action positionToDropPreloadedSample = drive.actionBuilder(INIT_POS)
                    .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_DROP_SAMPLE_HIGH, IncredibotsArmControl.CLAW_ARM_VELOCITY, false, false))
                    .strafeToLinearHeading(DROP_PRELOADED_SAMPLE.position, basketHeading)
                    .build();

            Action positionForThirdSample = drive.actionBuilder(new Pose2d(DROP_PRELOADED_SAMPLE.position, basketHeading))
                    .turnTo(heading, new TurnConstraints(drive.PARAMS.maxAngVel / 2, -drive.PARAMS.maxAngAccel / 2, drive.PARAMS.maxAngAccel / 2))
                    .build();

            Action turnToDropThirdSample = drive.actionBuilder(new Pose2d(DROP_PRELOADED_SAMPLE.position, heading))
                    .turnTo(basketHeading, new TurnConstraints(drive.PARAMS.maxAngVel / 2, -drive.PARAMS.maxAngAccel / 2, drive.PARAMS.maxAngAccel / 2))
                    .build();

            Action positionForFourthSampleStep1 = drive.actionBuilder(new Pose2d(DROP_PRELOADED_SAMPLE.position, basketHeading))
                    .turnTo(heading, new TurnConstraints(drive.PARAMS.maxAngVel / 4, -drive.PARAMS.maxAngAccel / 4, drive.PARAMS.maxAngAccel / 3))
                    .build();

            Action positionForFourthSampleStep2 = drive.actionBuilder(new Pose2d(DROP_PRELOADED_SAMPLE.position, heading))
                    .turnTo(farSampleHeading, new TurnConstraints(drive.PARAMS.maxAngVel / 3, -drive.PARAMS.maxAngAccel / 3, drive.PARAMS.maxAngAccel / 3))
                    .build();

            Action turnToDropFourthSample = drive.actionBuilder(new Pose2d(DROP_PRELOADED_SAMPLE.position, farSampleHeading))
                    .turnTo(basketHeading, new TurnConstraints(drive.PARAMS.maxAngVel / 2, -drive.PARAMS.maxAngAccel / 2, drive.PARAMS.maxAngAccel / 2))
                    .build();

            waitForStart();

            while (opModeIsActive()) {

                ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

                //drop preloaded sample
                Actions.runBlocking(
                        new SequentialAction(
                                GetClawControlAction(false, false, false),
                                new ParallelAction(
                                        positionToDropPreloadedSample,
                                        GetSideControlAction(IncredibotsArmControl.MAX_SLIDE_POSITION_ARM_FORWARDS_LOW, IncredibotsArmControl.SLIDE_VELOCITY_EXPANDING / 4, false, false)
                                ),
                                armControl.GetHighBasketActionSequence(),
                                GetClawControlAction(true, true, true)
                        )
                );

                //pick and drop sample 2
                Actions.runBlocking(
                        new SequentialAction(
                                armControl.GetClawArmAfterHighSampleActionSequence(),
                                GetArmControlAction(IncredibotsArmControl.CLAW_ARM_ENTER_SUB, (int)(IncredibotsArmControl.CLAW_ARM_VELOCITY * 0.34), true, false),
                                GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SAMPLE + 30, (int)(IncredibotsArmControl.CLAW_ARM_VELOCITY *0.34), true, true),
                                GetClawControlAction(false, true, false),
                                armControl.GetHighBasketActionSequence(),
                                GetClawControlAction(true, true, true)
                        )
                );

                //pick and drop sample 3
                Actions.runBlocking(
                        new SequentialAction(
                                new ParallelAction(
                                        armControl.GetClawArmAfterHighSampleActionSequence(),
                                        positionForThirdSample
                                ),
                                GetSlideControlAction(IncredibotsArmControl.MAX_SLIDE_POSITION_ARM_FORWARDS_LOW - 200, false),  //middle sample doesnt need to go as far
                                GetArmControlAction(IncredibotsArmControl.CLAW_ARM_ENTER_SUB, (int)(IncredibotsArmControl.CLAW_ARM_VELOCITY *0.34), true, false),
                                GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SAMPLE + 30, (int)(IncredibotsArmControl.CLAW_ARM_VELOCITY *0.34), true, true),
                                GetClawControlAction(false, true, false),
                                new ParallelAction(
                                        GetWristControlAction(0.3, false, false),
                                        turnToDropThirdSample,
                                        armControl.GetHighBasketActionSequence()
                                ),
                                GetClawControlAction(true, true, true)
                        )
                );

                //pick and drop sample 4
    //            Actions.runBlocking(
    //                    new SequentialAction(
    //                            armControl.GetClawArmAfterHighSampleActionSequence(),
    //                            new ParallelAction(
    //                                    GetSlideControlAction(IncredibotsArmControl.MAX_SLIDE_POSITION_ARM_FORWARDS_LOW - 80, false),   //far sample optimization
    //                                    GetWristControlAction(IncredibotsArmControl.WRIST_ENTER_SUB, false, false),
    //                                    positionForFourthSampleStep1,
    //                                    GetArmControlAction(IncredibotsArmControl.CLAW_ARM_ENTER_SUB, (int)(IncredibotsArmControl.CLAW_ARM_VELOCITY *0.37), true, true)
    //                            ),
    //                            positionForFourthSampleStep2,
    //                            GetWristControlAction(IncredibotsArmControl.WRIST_PICK_SAMPLE, true, false),
    //                            GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SAMPLE + 30, (int)(IncredibotsArmControl.CLAW_ARM_VELOCITY *0.34), true, true),
    //                            GetClawControlAction(false, true, false),
    //                            new ParallelAction(
    //                                    GetWristControlAction(0.3, false, false),
    //                                    turnToDropFourthSample,
    //                                    armControl.GetHighBasketActionSequence()
    //                            ),
    //                            GetClawControlAction(true, true, true)
    //                    )
    //            );

                //bring arm back to resting for TeleOp
                Actions.runBlocking(
                        new SequentialAction(
                                armControl.GetClawArmAfterHighSampleActionSequence(),
                                armControl.GetRestingActionSequence()
                        )
                );

                Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME TOTAL: " + elapsedTime.milliseconds());

                break;
            }
        }
    }

    @Config
    @Autonomous(name = "Blue_4_Specimen", group = "Autonomous")
    public static class Blue_4_Specimen extends IncredibotsAuto {

        private static final int multiplier = 1;    //used to flip coordinates between blue (1) and red (-1)

        public static final double heading = Math.toRadians(-90 * multiplier);
        public static final double reverseHeading = Math.toRadians(90 * multiplier);

        public static double minTransVelocity = 30;
        public static double minAccel = -20;
        public static double maxAccel = 40;

        public static Pose2d INIT_POS = new Pose2d(-16 * multiplier, 60.75 * multiplier, heading);
        public static Vector2d SLIDE_NEXT_TO_SAMP_1 = new Vector2d(-37 * multiplier, 25 * multiplier);
        public static Vector2d SLIDE_BEHIND_SAMP_1 = new Vector2d(-43.5 * multiplier, 15 * multiplier);
        public static Vector2d PUSH_SAMP_1 = new Vector2d(SLIDE_BEHIND_SAMP_1.x, 50 * multiplier);
        public static Vector2d SLIDE_BEHIND_SAMP_2 = new Vector2d(-55 * multiplier, 15 * multiplier);
        public static Vector2d PUSH_SAMP_2 = new Vector2d(SLIDE_BEHIND_SAMP_2.x, 51 * multiplier);

        public static Vector2d PICK_SPECIMEN = new Vector2d(-43 * multiplier, 57.5 * multiplier);
        public static Vector2d PICK_SPECIMEN_SLOW = new Vector2d(PICK_SPECIMEN.x, 58 * multiplier);

        public static Pose2d BRACE_RUNGS_FOR_SPECIMEN_ONE = new Pose2d(6 * multiplier, 30 * multiplier, heading);
        public static Pose2d BRACE_RUNGS_FOR_SPECIMEN_TWO = new Pose2d(2.5 * multiplier, 30 * multiplier, heading);
        public static Pose2d BRACE_RUNGS_FOR_SPECIMEN_THREE = new Pose2d(0.5 * multiplier, 30 * multiplier, heading);
        public static Pose2d BRACE_RUNGS_FOR_SPECIMEN_FOUR = new Pose2d(-4 * multiplier, 30 * multiplier, heading);
        public static Pose2d PARK = new Pose2d(PUSH_SAMP_1.x, PUSH_SAMP_1.y, Math.toRadians((-90 * multiplier) + 270));

        @Override
        public void runOpMode() throws InterruptedException {
            myHardware = new RobotHardware(this.hardwareMap);
            armControl = new IncredibotsArmControl(gamepad2, myHardware);
            drive = new MecanumDrive(this.hardwareMap, INIT_POS);

            //Put all the actions here, but replace the sample pushing with continuous splines below

            Action robotPreloadedSpecimen = drive.actionBuilder(INIT_POS)
                    .strafeToConstantHeading(BRACE_RUNGS_FOR_SPECIMEN_ONE.position)
                    .build();

            Action SamplePushingWithSplines = drive.actionBuilder(BRACE_RUNGS_FOR_SPECIMEN_ONE)
                    .setTangent(reverseHeading)
                    .splineToConstantHeading(SLIDE_NEXT_TO_SAMP_1, heading, new TranslationalVelConstraint(minTransVelocity), new ProfileAccelConstraint(minAccel, maxAccel))
                    .splineToConstantHeading(SLIDE_BEHIND_SAMP_1, reverseHeading, new TranslationalVelConstraint(minTransVelocity), new ProfileAccelConstraint(minAccel, maxAccel))
                    .splineToConstantHeading(PUSH_SAMP_1, reverseHeading, new TranslationalVelConstraint(minTransVelocity), new ProfileAccelConstraint(minAccel, maxAccel))
                    .splineToConstantHeading(SLIDE_BEHIND_SAMP_1, heading, new TranslationalVelConstraint(minTransVelocity), new ProfileAccelConstraint(minAccel, maxAccel))
                    .splineToConstantHeading(SLIDE_BEHIND_SAMP_2, reverseHeading, new TranslationalVelConstraint(minTransVelocity), new ProfileAccelConstraint(minAccel, maxAccel))
                    .splineToConstantHeading(PUSH_SAMP_2, reverseHeading, new TranslationalVelConstraint(minTransVelocity), new ProfileAccelConstraint(minAccel, maxAccel))
                    .build();

            Action moveToPickSpecimenTwo = drive.actionBuilder(new Pose2d(PUSH_SAMP_2, heading))
                    .setTangent(heading)
                    .splineToConstantHeading(PICK_SPECIMEN_SLOW, reverseHeading, new TranslationalVelConstraint(20), new ProfileAccelConstraint(minAccel, 20))
                    .stopAndAdd(GetClawControlAction(false, true, false))
                    .build();

            Action pickAndSnapSpecimenTwo = drive.actionBuilder(new Pose2d(PICK_SPECIMEN, heading))
                    .strafeToConstantHeading(BRACE_RUNGS_FOR_SPECIMEN_TWO.position)
                    .build();

            Action moveToPickSpecimenThree = drive.actionBuilder(BRACE_RUNGS_FOR_SPECIMEN_TWO)
                    .setTangent(reverseHeading)
                    .splineToConstantHeading(PICK_SPECIMEN, reverseHeading, new TranslationalVelConstraint(minTransVelocity + 20), new ProfileAccelConstraint(minAccel - 5 , maxAccel + 5))
                    .stopAndAdd(GetClawControlAction(false, true, false))
                    .build();

            Action pickAndSnapSpecimenThree = drive.actionBuilder(new Pose2d(PICK_SPECIMEN, heading))
                    .strafeToConstantHeading(BRACE_RUNGS_FOR_SPECIMEN_THREE.position)
                    .build();

            Action moveToPickSpecimenFour = drive.actionBuilder(BRACE_RUNGS_FOR_SPECIMEN_THREE)
                    .setTangent(reverseHeading)
                    .splineToConstantHeading(PICK_SPECIMEN, reverseHeading, new TranslationalVelConstraint(minTransVelocity + 20), new ProfileAccelConstraint(minAccel - 5, maxAccel + 5))
                    .stopAndAdd(GetClawControlAction(false, true, false))
                    .build();

            Action pickAndSnapSpecimenFour = drive.actionBuilder(new Pose2d(PICK_SPECIMEN, heading))
                    .setTangent(Math.toRadians(-90 * multiplier + 90))
                    .splineToConstantHeading(BRACE_RUNGS_FOR_SPECIMEN_FOUR.position, heading, new TranslationalVelConstraint(minTransVelocity + 20), new ProfileAccelConstraint(minAccel-10, maxAccel + 10))
                    .build();

            Action park = drive.actionBuilder(BRACE_RUNGS_FOR_SPECIMEN_FOUR)
                    .setTangent(reverseHeading)
                    .splineToConstantHeading(PARK.position, PARK.heading)
                    .build();

            waitForStart();

            while (opModeIsActive()) {
                ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

                //snap preloaded specimen
                Actions.runBlocking(
                        new SequentialAction(
                            new ParallelAction(
                                    armControl.GetHangSpecimenActionSequence_Fast(),
                                    robotPreloadedSpecimen),
                            armControl.GetSnapSpecimenActionSequence()
                        )
                );

                Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER PRELOADED SNAP: " + timer.milliseconds());


                //push in samples on spike marks
                Actions.runBlocking(
                        new ParallelAction(
                                armControl.GetArmVerticalActionSequence(),
                                SamplePushingWithSplines)
                );

                Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER PUSHING IN SPIKES: " + timer.milliseconds());


                //move to pick specimen 2
                Actions.runBlocking(
                        new ParallelAction(
                            armControl.GetPickSpecimenActionSequence(),
                                moveToPickSpecimenTwo
                        )
                );

                //pick and snap specimen 2
                Actions.runBlocking(
                        new SequentialAction(
                            new ParallelAction(
                                    armControl.GetHangSpecimenActionSequence(),
                                    pickAndSnapSpecimenTwo
                            ),
                            armControl.GetSnapSpecimenActionSequence()
                ));

                Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER SNAPPING SPECIMEN 2: " + timer.milliseconds());


                //move to pick specimen 3
                Actions.runBlocking(
                        new ParallelAction(
                                armControl.GetPickSpecimenActionSequence(),
                                moveToPickSpecimenThree
                        )
                );

                //pick and snap specimen3
                Actions.runBlocking(
                        new SequentialAction(
                                new ParallelAction(
                                        armControl.GetHangSpecimenActionSequence(),
                                        pickAndSnapSpecimenThree
                                ),
                                armControl.GetSnapSpecimenActionSequence()
                        )
                );

                Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER SNAPPING SPECIMEN 3: " + timer.milliseconds());


                //move to pick specimen 4
                Actions.runBlocking(
                        new ParallelAction(
                                armControl.GetPickSpecimenActionSequence(),
                                moveToPickSpecimenFour
                        )
                );

                //pick and snap specimen 4
                Actions.runBlocking(
                        new SequentialAction(
                                new ParallelAction(
                                        armControl.GetHangSpecimenActionSequence(),
                                        pickAndSnapSpecimenFour
                                ),
                                armControl.GetSnapSpecimenActionSequence()
                        )
                );

                Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER SNAPPING SPECIMEN 4: " + timer.milliseconds());

                Actions.runBlocking(
                        new ParallelAction(
                                park,
                                armControl.GetRestingActionSequenceNoWait()
                        )
                );

                telemetry.addData("Elapsed Time: TOTAL: ", timer.milliseconds());
                Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME: " + timer.milliseconds());
                telemetry.update();

                break;
            }
        }
    }
}
