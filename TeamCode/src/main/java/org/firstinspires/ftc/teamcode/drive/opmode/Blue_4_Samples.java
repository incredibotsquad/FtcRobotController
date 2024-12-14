package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
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
    public static final int multiplier = 1; //used to flip coordinates between blue (1) and red (-1)

    public static double heading = Math.toRadians(-90 * multiplier);
    public static double basketHeading = Math.toRadians(-90 * multiplier - 21);
    public static double farSampleHeading = Math.toRadians(-90 * multiplier + 21); //17
    public static Pose2d INIT_POS = new Pose2d(41 * multiplier, 60.75 * multiplier, heading);
    public static Pose2d DROP_PRELOADED_SAMPLE = new Pose2d(60.5 * multiplier, 54 * multiplier, heading);

    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new RobotHardware(this.hardwareMap);
        armControl = new IncredibotsArmControl(gamepad2, myHardware);
        drive = new MecanumDrive(this.hardwareMap, INIT_POS);

        boolean ranOnce = false;

        Action positionToDropPreloadedSample = drive.actionBuilder(INIT_POS)
                .stopAndAdd(GetArmControlAction(IncredibotsArmControl.CLAW_ARM_DROP_SAMPLE_HIGH, IncredibotsArmControl.CLAW_ARM_VELOCITY, false, false))
                .strafeToLinearHeading(DROP_PRELOADED_SAMPLE.position, basketHeading)
                .build();

        Action positionForThirdSample = drive.actionBuilder(new Pose2d(DROP_PRELOADED_SAMPLE.position, basketHeading))
                .turnTo(heading, new TurnConstraints(drive.PARAMS.maxAngVel / 2, -drive.PARAMS.maxAngAccel / 2, drive.PARAMS.maxAngAccel / 2))
                .build();

        Action turnToDropThirdSample = drive.actionBuilder(new Pose2d(DROP_PRELOADED_SAMPLE.position, heading))
                .turnTo(basketHeading, new TurnConstraints(drive.PARAMS.maxAngVel / 3, -drive.PARAMS.maxAngAccel / 3, drive.PARAMS.maxAngAccel / 3))
                .build();

        Action positionForFourthSample = drive.actionBuilder(new Pose2d(DROP_PRELOADED_SAMPLE.position, basketHeading))
                .turnTo(farSampleHeading, new TurnConstraints(drive.PARAMS.maxAngVel / 2, -drive.PARAMS.maxAngAccel / 2, drive.PARAMS.maxAngAccel / 2))
                .build();

        Action turnToDropFourthSample = drive.actionBuilder(new Pose2d(DROP_PRELOADED_SAMPLE.position, farSampleHeading))
                .turnTo(basketHeading, new TurnConstraints(drive.PARAMS.maxAngVel / 3, -drive.PARAMS.maxAngAccel / 3, drive.PARAMS.maxAngAccel / 3))
                .build();

        waitForStart();

        while (opModeIsActive()) {

            ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            //drop preloaded sample
            Actions.runBlocking(
                    new SequentialAction(
                            GetClawControlAction(false, false, false),
                            positionToDropPreloadedSample,
                            armControl.GetHighBasketActionSequence(),
                            GetClawControlAction(true, true, true)
                    )
            );

            //pick and drop sample 2
            Actions.runBlocking(
                    new SequentialAction(
                            armControl.GetClawArmAfterHighSampleActionSequence(),
                            GetArmControlAction(IncredibotsArmControl.CLAW_ARM_ENTER_SUB, (int)(IncredibotsArmControl.CLAW_ARM_VELOCITY * 0.4), true, false),
                            GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SAMPLE + 30, (int)(IncredibotsArmControl.CLAW_ARM_VELOCITY *0.4), true, false),
                            GetClawControlAction(false, true, false),
                            armControl.GetHighBasketActionSequence(),
                            GetClawControlAction(true, true, true)
                    )
            );

            //pick and drop sample 3
            Actions.runBlocking(
                    new SequentialAction(
                            armControl.GetClawArmAfterHighSampleActionSequence(),
                            new ParallelAction(
                                    positionForThirdSample,
                                    GetSlideControlAction(IncredibotsArmControl.MAX_SLIDE_POSITION_ARM_FORWARDS_LOW - 200, false)   //middle sample doesnt need to go as far
                            ),
                            GetArmControlAction(IncredibotsArmControl.CLAW_ARM_ENTER_SUB, (int)(IncredibotsArmControl.CLAW_ARM_VELOCITY *0.4), true, false),
                            GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SAMPLE + 30, (int)(IncredibotsArmControl.CLAW_ARM_VELOCITY *0.4), true, false),
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
            Actions.runBlocking(
                    new SequentialAction(
                            armControl.GetClawArmAfterHighSampleActionSequence(),
                            new ParallelAction(
                                    GetSlideControlAction(IncredibotsArmControl.MAX_SLIDE_POSITION_ARM_FORWARDS_LOW - 80, false),   //far sample optimization
                                    GetWristControlAction(IncredibotsArmControl.WRIST_ENTER_SUB, false, false),
                                    GetArmControlAction(IncredibotsArmControl.CLAW_ARM_ENTER_SUB, (int)(IncredibotsArmControl.CLAW_ARM_VELOCITY *0.4), true, false)
                            ),
                            positionForFourthSample,
                            GetWristControlAction(IncredibotsArmControl.WRIST_PICK_SAMPLE, true, false),
                            GetArmControlAction(IncredibotsArmControl.CLAW_ARM_PICK_SAMPLE + 30, (int)(IncredibotsArmControl.CLAW_ARM_VELOCITY *0.4), true, false),
                            GetClawControlAction(false, true, false),
                            new ParallelAction(
                                    GetWristControlAction(0.3, false, false),
                                    turnToDropFourthSample,
                                    armControl.GetHighBasketActionSequence()
                            ),
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

            break;
        }
    }
}
