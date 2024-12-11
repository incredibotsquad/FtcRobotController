package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@Autonomous(name = "Blue_4_Samples", group = "Autonomous")
public class Blue_4_Samples  extends IncredibotsAuto {

    public static final int multiplier = 1; //used to flip coordinates between blue and red

    public static double heading = Math.toRadians(-90 * multiplier);
    public static double reverseHeading = Math.toRadians(90 * multiplier);
    public static double basketHeading = Math.toRadians(-135 * multiplier);
    public static Pose2d INIT_POS = new Pose2d(41 * multiplier, 60.75 * multiplier, heading);
    public static Pose2d DROP_LOADED_SAMPLE = new Pose2d(48 * multiplier, 48 * multiplier, basketHeading);
    public static Pose2d BASKET_POS = new Pose2d(59.2 * multiplier, 60 * multiplier, basketHeading);

    public static Vector2d PICK_SAMPLE = new Vector2d( BASKET_POS.position.x, 45 * multiplier);

    public static double leftSampleHeading = Math.toRadians(-90 * multiplier + 14);
    public static double middleSampleHeading = heading;
    public static double rightSampleHeading = Math.toRadians(-90 * multiplier - 18);

    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new RobotHardware(this.hardwareMap);
        armControl = new IncredibotsArmControl(gamepad2, myHardware);
        drive = new MecanumDrive(this.hardwareMap, INIT_POS);

        boolean ranOnce = false;

        Action positionToDropPreloadedSample = drive.actionBuilder(INIT_POS)
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_DROP_SAMPLE_HIGH, IncredibotsArmControl.CLAW_ARM_VELOCITY, false, false))
                .strafeToLinearHeading(DROP_LOADED_SAMPLE.position, DROP_LOADED_SAMPLE.heading)
                .build();

        Action dropPreloadedSample = drive.actionBuilder(DROP_LOADED_SAMPLE)
                .lineToYConstantHeading(BASKET_POS.position.y, new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10, 20))
                .stopAndAdd(GetClawControlAction(true, true, true))
                .build();

        Action turnToPickMiddleSample = drive.actionBuilder(BASKET_POS)
                .turnTo(middleSampleHeading)
                .build();

        Action pickMiddleSample = drive.actionBuilder(new Pose2d(BASKET_POS.position, heading))
                .setTangent(middleSampleHeading)
                .lineToYConstantHeading(PICK_SAMPLE.y, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-10, 20))
                .stopAndAdd(GetClawControlAction(false, true, false))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_DROP_SAMPLE_HIGH, IncredibotsArmControl.CLAW_ARM_VELOCITY, true, false))
                .strafeToLinearHeading(DROP_LOADED_SAMPLE.position, DROP_LOADED_SAMPLE.heading)
                .build();

        Action dropMiddleSample = drive.actionBuilder(DROP_LOADED_SAMPLE)
                .lineToYConstantHeading(BASKET_POS.position.y, new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10, 20))
                .stopAndAdd(GetClawControlAction(true, true, true))
                .build();

        Action turnToPickLeftSample = drive.actionBuilder(BASKET_POS)
                .turnTo(leftSampleHeading)
                .build();

        Action pickLeftSample = drive.actionBuilder(new Pose2d(BASKET_POS.position, leftSampleHeading))
                .setTangent(leftSampleHeading)
                .lineToYConstantHeading(PICK_SAMPLE.y, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-10, 20))
                .stopAndAdd(GetClawControlAction(false, true, false))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_DROP_SAMPLE_HIGH, IncredibotsArmControl.CLAW_ARM_VELOCITY, true, false))
                .strafeToLinearHeading(DROP_LOADED_SAMPLE.position, DROP_LOADED_SAMPLE.heading)
                .build();

        Action dropLeftSample = drive.actionBuilder(DROP_LOADED_SAMPLE)
                .lineToYConstantHeading(BASKET_POS.position.y, new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10, 20))
                .stopAndAdd(GetClawControlAction(true, true, true))
                .build();

        Action turnToPickRightSample = drive.actionBuilder(BASKET_POS)
                .turnTo(rightSampleHeading)
                .build();

        Action pickRightSample = drive.actionBuilder(new Pose2d(BASKET_POS.position, rightSampleHeading))
                .setTangent(rightSampleHeading)
                .lineToYConstantHeading(PICK_SAMPLE.y, new TranslationalVelConstraint(30), new ProfileAccelConstraint(-10, 20))
                .stopAndAdd(GetClawControlAction(false, true, false))
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_DROP_SAMPLE_HIGH, IncredibotsArmControl.CLAW_ARM_VELOCITY, true, false))
                .strafeToLinearHeading(DROP_LOADED_SAMPLE.position, DROP_LOADED_SAMPLE.heading)
                .build();

        Action dropRightSample = drive.actionBuilder(DROP_LOADED_SAMPLE)
                .lineToYConstantHeading(BASKET_POS.position.y, new TranslationalVelConstraint(10), new ProfileAccelConstraint(-10, 20))
                .stopAndAdd(GetClawControlAction(true, true, true))
                .build();

        waitForStart();

        while (opModeIsActive()) {

            if (!ranOnce) {
                ranOnce = true;

                ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

                //drop preloaded sample
                Actions.runBlocking(
                        new SequentialAction(
                                positionToDropPreloadedSample,
                                armControl.GetHighBasketActionSequence(),
                                dropPreloadedSample,
                                armControl.GetClawArmAfterHighSampleActionSequence()
                        )
                );

                //pick and drop middle sample
                Actions.runBlocking(
                        new SequentialAction(
                                turnToPickMiddleSample,
                                armControl.GetPickSampleActionSequence(),
                                pickMiddleSample,
                                armControl.GetHighBasketActionSequence(),
                                dropMiddleSample,
                                armControl.GetClawArmAfterHighSampleActionSequence()
                        )
                );

                //pick and drop left sample
                Actions.runBlocking(
                        new SequentialAction(
                                turnToPickLeftSample,
                                armControl.GetPickSampleActionSequence(),
                                pickLeftSample,
                                armControl.GetHighBasketActionSequence(),
                                dropLeftSample,
                                armControl.GetClawArmAfterHighSampleActionSequence()
                        )
                );

                //pick and drop right sample
                Actions.runBlocking(
                        new SequentialAction(
                                turnToPickRightSample,
                                armControl.GetPickSampleActionSequence(),
                                pickRightSample,
                                armControl.GetHighBasketActionSequence(),
                                dropRightSample,
                                armControl.GetClawArmAfterHighSampleActionSequence()
                        )
                );

                Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME TOTAL: " + elapsedTime.milliseconds());
            }

            //break;
        }
    }
}
