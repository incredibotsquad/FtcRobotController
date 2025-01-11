package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.IncredibotsArmControl;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous(name = "Red_4_Samples", group = "Autonomous")
public class Red_4_Samples  extends IncredibotsAuto {
    public static final int multiplier = -1; //used to flip coordinates between blue (1) and red (-1)

    public static double heading = Math.toRadians(-90 * multiplier);
    public static double basketHeading = Math.toRadians(-90 * multiplier - 45);
    public static double leftSampleHeading = Math.toRadians(-90 * multiplier + 35);
    public static Pose2d INIT_POS = new Pose2d(41 * multiplier, 60.75 * multiplier, heading);
    public static Pose2d DROP_SAMPLE_POS = new Pose2d(61 * multiplier, 60 * multiplier, basketHeading);
    public static Pose2d RIGHT_SAMPLE_POS = new Pose2d(49 * multiplier, 42 * multiplier, heading); //43
    public static Pose2d MIDDLE_SAMPLE_POS = new Pose2d(59 * multiplier, 42 * multiplier, heading);
    public static Pose2d LEFT_SAMPLE_POS = new Pose2d(59 * multiplier, 38.5 * multiplier, leftSampleHeading); //59 - 39.5
    public static Vector2d END_POS = new Vector2d(48 * multiplier, 48 * multiplier);

    public static double minTransVelocity = 30;
    public static double minAccel = -20;
    public static double maxAccel = 40;

    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new RobotHardware(this.hardwareMap);
        armControl = new IncredibotsArmControl(gamepad2, myHardware);
        drive = new MecanumDrive(this.hardwareMap, INIT_POS);

        Action positionToDropPreloadedSample = drive.actionBuilder(INIT_POS)
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.DROP_SAMPLE_HIGH_ARM, IncredibotsArmControl.CLAW_ARM_VELOCITY, false, false))
                .strafeToLinearHeading(DROP_SAMPLE_POS.position, DROP_SAMPLE_POS.heading)
                .build();

        Action positionToPickRightSample = drive.actionBuilder(DROP_SAMPLE_POS)
                .strafeToLinearHeading(RIGHT_SAMPLE_POS.position, RIGHT_SAMPLE_POS.heading, new TranslationalVelConstraint(minTransVelocity), new ProfileAccelConstraint(minAccel, maxAccel))
                .build();

        Action positionToDropRightSample = drive.actionBuilder(RIGHT_SAMPLE_POS)
                .strafeToLinearHeading(DROP_SAMPLE_POS.position, DROP_SAMPLE_POS.heading, new TranslationalVelConstraint(minTransVelocity), new ProfileAccelConstraint(minAccel, maxAccel))
                .build();

        Action positionToPickMiddleSample = drive.actionBuilder(DROP_SAMPLE_POS)
                .strafeToLinearHeading(MIDDLE_SAMPLE_POS.position, MIDDLE_SAMPLE_POS.heading, new TranslationalVelConstraint(minTransVelocity), new ProfileAccelConstraint(minAccel, maxAccel))
                .build();

        Action positionToDropMiddleSample = drive.actionBuilder(MIDDLE_SAMPLE_POS)
                .strafeToLinearHeading(DROP_SAMPLE_POS.position, DROP_SAMPLE_POS.heading, new TranslationalVelConstraint(minTransVelocity), new ProfileAccelConstraint(minAccel, maxAccel))
                .build();

        Action positionToPickLeftSample = drive.actionBuilder(DROP_SAMPLE_POS)
                .strafeToLinearHeading(LEFT_SAMPLE_POS.position, LEFT_SAMPLE_POS.heading, new TranslationalVelConstraint(minTransVelocity), new ProfileAccelConstraint(minAccel, maxAccel))
                .build();

        Action positionToDropLeftSample = drive.actionBuilder(LEFT_SAMPLE_POS)
                .strafeToLinearHeading(DROP_SAMPLE_POS.position, DROP_SAMPLE_POS.heading, new TranslationalVelConstraint(minTransVelocity), new ProfileAccelConstraint(minAccel, maxAccel))
                .build();

        Action moveForwardToAvoidGettingStuck = drive.actionBuilder(DROP_SAMPLE_POS)
                .strafeToConstantHeading(END_POS)
                .build();

        waitForStart();

        while (opModeIsActive()) {

            ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            //drop preloaded sample
            Actions.runBlocking(
                    new SequentialAction(
                            GetClawControlAction(false, false, false),
                            new ParallelAction(
                                    positionToDropPreloadedSample
                            ),
                            armControl.GetHighBasketActionSequence(),
                            armControl.GetSampleEjectActionSequence(),
                            GetWristControlAction(IncredibotsArmControl.ENTER_SUB_WRIST, false, false),
                            GetSlideControlAction(IncredibotsArmControl.ENTER_SUB_SLIDE, IncredibotsArmControl.SLIDE_VELOCITY_EXPANDING, true, false)
                    )
            );

            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER DROPPING PRELOAD: " + elapsedTime.milliseconds());

            //pick and drop right sample
            Actions.runBlocking(
                    new SequentialAction(
                            positionToPickRightSample,
                            armControl.GetPickSampleActionSequence(),
                            new SleepAction(0.1),   //make sure sample gets picked
                            GetIntakeControlAction(false, true, true),  //else it kills RR
                            GetArmControlAction(IncredibotsArmControl.DROP_SAMPLE_HIGH_ARM, IncredibotsArmControl.CLAW_ARM_VELOCITY, false),
                            GetWristControlAction(IncredibotsArmControl.DROP_SAMPLE_HIGH_WRIST, true, false),
                            positionToDropRightSample,
                            armControl.GetHighBasketActionSequence(),
                            armControl.GetSampleEjectActionSequence(),
                            new SleepAction(0.3),   //make sure sample falls
                            GetWristControlAction(IncredibotsArmControl.PICK_SAMPLE_WRIST, true, false),
                            GetClawControlAction(false, false, false),
                            GetSlideControlAction(IncredibotsArmControl.PICK_SAMPLE_SLIDE, IncredibotsArmControl.SLIDE_VELOCITY_EXPANDING, true, true)
                    )
            );

            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER DROPPING RIGHT SAMPLE: " + elapsedTime.milliseconds());

            Actions.runBlocking(
                    new SequentialAction(
                            positionToPickMiddleSample,
                            GetIntakeControlAction(true, false, false),
                            armControl.GetPickSampleActionSequence(),
                            new SleepAction(0.1),   //make sure sample gets picked
                            GetIntakeControlAction(false, true, true),  //else it kills RR
                            GetArmControlAction(IncredibotsArmControl.DROP_SAMPLE_HIGH_ARM, IncredibotsArmControl.CLAW_ARM_VELOCITY, false),
                            GetWristControlAction(IncredibotsArmControl.DROP_SAMPLE_HIGH_WRIST, true, false),
                            positionToDropMiddleSample,
                            armControl.GetHighBasketActionSequence(),
                            armControl.GetSampleEjectActionSequence(),
                            new SleepAction(0.3),   //make sure sample falls
                            GetWristControlAction(IncredibotsArmControl.PICK_SAMPLE_WRIST, true, false),
                            GetClawControlAction(false, false, false),
                            GetSlideControlAction(IncredibotsArmControl.PICK_SAMPLE_SLIDE, IncredibotsArmControl.SLIDE_VELOCITY_EXPANDING, true, true)
                    )
            );

            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER DROPPING MIDDLE SAMPLE: " + elapsedTime.milliseconds());

            Actions.runBlocking(
                    new SequentialAction(
                            positionToPickLeftSample,
                            GetIntakeControlAction(true, false, false),
                            armControl.GetPickSampleActionSequence(),
                            new SleepAction(0.1),   //make sure sample gets picked
                            GetIntakeControlAction(false, true, true),  //else it kills RR
                            GetArmControlAction(IncredibotsArmControl.DROP_SAMPLE_HIGH_ARM, IncredibotsArmControl.CLAW_ARM_VELOCITY, false),
                            GetWristControlAction(IncredibotsArmControl.DROP_SAMPLE_HIGH_WRIST, true, false),
                            positionToDropLeftSample,
                            armControl.GetHighBasketActionSequence(),
                            armControl.GetSampleEjectActionSequence(),
                            new SleepAction(0.3),   //make sure sample falls
                            GetWristControlAction(IncredibotsArmControl.WRIST_PRELOAD_RESTING, true, false),
                            GetClawControlAction(false, false, false),
                            GetSlideControlAction(IncredibotsArmControl.SLIDE_POSITION_RESTING, IncredibotsArmControl.SLIDE_VELOCITY_EXPANDING, true, true),
                            new ParallelAction(
                                    moveForwardToAvoidGettingStuck,
                                    GetArmControlAction(IncredibotsArmControl.CLAW_ARM_RESTING_BACK, IncredibotsArmControl.CLAW_ARM_VELOCITY, false)
                            )
                    )
            );

            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME TOTAL: " + elapsedTime.milliseconds());

            break;
        }
    }
}
