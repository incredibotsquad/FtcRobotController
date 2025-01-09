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
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.IncredibotsArmControl;


@Config
@Disabled
@Autonomous(name = "Blue_1_Spec_3_Samples", group = "Autonomous")
public class Blue_1_Spec_3_Samples extends IncredibotsAuto {

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
                            GetWristControlAction(IncredibotsArmControl.PICK_SAMPLE_WRIST, false, false),
                            armControl.GetClawArmAfterHighSampleActionSequence(),
                            GetArmControlAction(IncredibotsArmControl.ENTER_SUB_ARM, IncredibotsArmControl.CLAW_ARM_VELOCITY / 3, true, false),
                            GetArmControlAction(IncredibotsArmControl.PICK_SAMPLE_ARM + 30, IncredibotsArmControl.CLAW_ARM_VELOCITY / 3, true, false),
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
                            GetWristControlAction(IncredibotsArmControl.PICK_SAMPLE_WRIST, false, false),
                            GetArmControlAction(IncredibotsArmControl.ENTER_SUB_ARM, IncredibotsArmControl.CLAW_ARM_VELOCITY / 3, true, false),
                            GetArmControlAction(IncredibotsArmControl.PICK_SAMPLE_ARM + 30, IncredibotsArmControl.CLAW_ARM_VELOCITY / 3, true, false),
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
                            GetWristControlAction(IncredibotsArmControl.PICK_SAMPLE_WRIST, false, false),
                            GetArmControlAction(IncredibotsArmControl.ENTER_SUB_ARM, IncredibotsArmControl.CLAW_ARM_VELOCITY / 3, true, false),
                            GetArmControlAction(IncredibotsArmControl.PICK_SAMPLE_ARM + 30, IncredibotsArmControl.CLAW_ARM_VELOCITY / 3, true, false),
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
