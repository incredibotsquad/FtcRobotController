package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.IncredibotsArmControl;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous(name = "Blue_4_Samples", group = "Autonomous")
public class Blue_4_Samples  extends IncredibotsAuto {
    public static final int multiplier = 1; //used to flip coordinates between blue (1) and red (-1)

    public static double heading = Math.toRadians(-90 * multiplier);
    public static double basketHeading = Math.toRadians(-90 * multiplier - 45);
    public static double rightSampleHeading = Math.toRadians(-90 * multiplier - 30);
    public static double leftSampleHeading = Math.toRadians(-90 * multiplier + 30);
    public static Pose2d INIT_POS = new Pose2d(41 * multiplier, 60.75 * multiplier, heading);
    public static Pose2d DROP_SAMPLE_POS = new Pose2d(60 * multiplier, 60 * multiplier, basketHeading);
    public static Pose2d RIGHT_SAMPLE_POS = new Pose2d(49 * multiplier, 43 * multiplier, heading);
    public static Pose2d MIDDLE_SAMPLE_POS = new Pose2d(59 * multiplier, 43 * multiplier, heading);
    public static Pose2d LEFT_SAMPLE_POS = new Pose2d(59 * multiplier, 39.5 * multiplier, leftSampleHeading); //59 - 46

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
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.DROP_SAMPLE_HIGH_ARM, IncredibotsArmControl.CLAW_ARM_VELOCITY, false))
                .stopAndAdd(GetWristControlAction(IncredibotsArmControl.DROP_SAMPLE_HIGH_WRIST, true,false))
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
                //.turn(leftSampleHeading, new TurnConstraints(drive.PARAMS.maxAngVel / 2, -drive.PARAMS.maxAngAccel / 2, drive.PARAMS.maxAngAccel / 2))
                .build();

        Action positionToDropLeftSample = drive.actionBuilder(LEFT_SAMPLE_POS)
                .strafeToLinearHeading(DROP_SAMPLE_POS.position, DROP_SAMPLE_POS.heading, new TranslationalVelConstraint(minTransVelocity), new ProfileAccelConstraint(minAccel, maxAccel))
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
                            armControl.GetSampleEjectActionSequence()
//                            new ParallelAction(
//                                    GetHighBasketSampleDropIntakeMotionAsRRAction(),
//                                    GetClawControlAction(true, true, false)
//                            )
                    )
            );

            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER DROPPING PRELOAD: " + elapsedTime.milliseconds());

            //pick and drop right sample
            Actions.runBlocking(
                    new SequentialAction(
                            GetWristControlAction(IncredibotsArmControl.ENTER_SUB_WRIST, false, false),
                            GetSideControlAction(IncredibotsArmControl.SLIDE_POSITION_RESTING, IncredibotsArmControl.SLIDE_VELOCITY_EXPANDING, true, false),
                            positionToPickRightSample,
                            armControl.GetPickSampleActionSequence(),
                            positionToDropRightSample,
                            armControl.GetHighBasketActionSequence(),
                            armControl.GetSampleEjectActionSequence()
//                            new ParallelAction(
//                                    GetHighBasketSampleDropIntakeMotionAsRRAction(),
//                                    GetClawControlAction(true, true, false)
//                            )
                    )
            );

            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER DROPPIING RIGHT SAMPLE: " + elapsedTime.milliseconds());

            Actions.runBlocking(
                    new SequentialAction(
                            GetWristControlAction(IncredibotsArmControl.ENTER_SUB_WRIST, false, false),
                            GetSideControlAction(IncredibotsArmControl.SLIDE_POSITION_RESTING, IncredibotsArmControl.SLIDE_VELOCITY_EXPANDING, true, false),
                            positionToPickMiddleSample,
                            armControl.GetPickSampleActionSequence(),
                            new ParallelAction(
                                    positionToDropMiddleSample,
                                    GetArmControlAction(IncredibotsArmControl.DROP_SAMPLE_HIGH_ARM, IncredibotsArmControl.CLAW_ARM_VELOCITY, false)
                            ),
                            armControl.GetHighBasketActionSequence(),
                            armControl.GetSampleEjectActionSequence()
//                            new ParallelAction(
//                                    GetHighBasketSampleDropIntakeMotionAsRRAction(),
//                                    GetClawControlAction(true, true, false)
//                            )
                    )
            );

            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER DROPPIING MIDDLE SAMPLE: " + elapsedTime.milliseconds());

            Actions.runBlocking(
                    new SequentialAction(
                            GetWristControlAction(IncredibotsArmControl.ENTER_SUB_WRIST, false, false),
                            GetSideControlAction(IncredibotsArmControl.SLIDE_POSITION_RESTING, IncredibotsArmControl.SLIDE_VELOCITY_EXPANDING, true, false),
                            positionToPickLeftSample,
                            armControl.GetPickSampleActionSequence(),
                            new ParallelAction(
                                    positionToDropLeftSample,
                                    GetArmControlAction(IncredibotsArmControl.DROP_SAMPLE_HIGH_ARM, IncredibotsArmControl.CLAW_ARM_VELOCITY, false)
                            ),
                            armControl.GetHighBasketActionSequence(),
                            armControl.GetSampleEjectActionSequence()
//                            new ParallelAction(
//                                    GetHighBasketSampleDropIntakeMotionAsRRAction(),
//                                    GetClawControlAction(true, true, false)
//                            )
                    )
            );

            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER DROPPIING LEFT SAMPLE: " + elapsedTime.milliseconds());

            Actions.runBlocking(
                    new SequentialAction(
                        GetWristControlAction(IncredibotsArmControl.ENTER_SUB_WRIST, false, false),
                        GetSideControlAction(IncredibotsArmControl.SLIDE_POSITION_RESTING, IncredibotsArmControl.SLIDE_VELOCITY_EXPANDING, true, false),
                        GetArmControlAction(IncredibotsArmControl.CLAW_ARM_RESTING_BACK, IncredibotsArmControl.CLAW_ARM_VELOCITY, true)
                    )
            );

            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME TOTAL: " + elapsedTime.milliseconds());

            break;
        }
    }
}
