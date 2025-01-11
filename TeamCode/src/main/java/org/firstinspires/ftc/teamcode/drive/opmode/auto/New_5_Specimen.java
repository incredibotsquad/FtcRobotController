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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.IncredibotsArmControl;


@Config
@Disabled
@Autonomous(name = "New_5_Specimen", group = "Autonomous")
public class New_5_Specimen extends IncredibotsAuto {

    private static final int multiplier = 1;    //used to flip coordinates between blue (1) and red (-1)

    public static final double heading = Math.toRadians(-90 * multiplier);
    public static final double reverseHeading = Math.toRadians(90 * multiplier);

    public static double minTransVelocity = 30;
    public static double minAccel = -20;
    public static double maxAccel = 40;

    public static Pose2d INIT_POS = new Pose2d(-16 * multiplier, 60.75 * multiplier, heading);

    public static Vector2d PICK_SPIKES = new Vector2d(-48 * multiplier, 45 * multiplier);
    public static double leftSpikeHeading = Math.toRadians(-90 * multiplier + 22);
    public static double rightSpikeHeading = Math.toRadians(-90 * multiplier - 22);

    public static Vector2d PICK_SPECIMEN = new Vector2d(-43 * multiplier, 57.5 * multiplier);
    public static Vector2d PICK_SPECIMEN_SLOW = new Vector2d(PICK_SPECIMEN.x, 58 * multiplier);

    public static Pose2d BRACE_RUNGS_FOR_SPECIMEN_ONE = new Pose2d(6 * multiplier, 30 * multiplier, heading);
    public static Pose2d BRACE_RUNGS_FOR_SPECIMEN_TWO = new Pose2d(2.5 * multiplier, 30 * multiplier, heading);
    public static Pose2d BRACE_RUNGS_FOR_SPECIMEN_THREE = new Pose2d(0.5 * multiplier, 30 * multiplier, heading);
    public static Pose2d BRACE_RUNGS_FOR_SPECIMEN_FOUR = new Pose2d(-4 * multiplier, 30 * multiplier, heading);
    public static Pose2d PARK = new Pose2d(-50 * multiplier, 50 * multiplier, Math.toRadians((-90 * multiplier) + 270));

    @Override
    public void runOpMode() throws InterruptedException {
        myHardware = new RobotHardware(this.hardwareMap);
        armControl = new IncredibotsArmControl(gamepad2, myHardware);
        drive = new MecanumDrive(this.hardwareMap, INIT_POS);

        Action robotPreloadedSpecimen = drive.actionBuilder(INIT_POS)
                .strafeToConstantHeading(BRACE_RUNGS_FOR_SPECIMEN_ONE.position)
                .build();

        Action positionToPickSpikes = drive.actionBuilder(BRACE_RUNGS_FOR_SPECIMEN_ONE)
                .setTangent(reverseHeading)
                .splineToConstantHeading(PICK_SPIKES, Math.toRadians((-90 * multiplier) + 270), new TranslationalVelConstraint(minTransVelocity), new ProfileAccelConstraint(minAccel, maxAccel))
                .build();

        Action moveToPickLeftSpike = drive.actionBuilder(new Pose2d(PICK_SPIKES, heading))
                .turnTo(leftSpikeHeading, new TurnConstraints(drive.PARAMS.maxAngVel / 2, -drive.PARAMS.maxAngAccel / 2, drive.PARAMS.maxAngAccel / 2))
                .build();

        Action moveToPickRightSpike = drive.actionBuilder(new Pose2d(PICK_SPIKES, leftSpikeHeading))
                .turnTo(rightSpikeHeading, new TurnConstraints(drive.PARAMS.maxAngVel / 2, -drive.PARAMS.maxAngAccel / 2, drive.PARAMS.maxAngAccel / 2))
                .build();

        Action moveToPickSpecimenTwo = drive.actionBuilder(new Pose2d(PICK_SPIKES, rightSpikeHeading))
                .turnTo(heading, new TurnConstraints(drive.PARAMS.maxAngVel / 2, -drive.PARAMS.maxAngAccel / 2, drive.PARAMS.maxAngAccel / 2))
                .setTangent(heading)
                .splineToConstantHeading(PICK_SPECIMEN_SLOW, reverseHeading, new TranslationalVelConstraint(20), new ProfileAccelConstraint(minAccel, 20))
                .stopAndAdd(GetClawControlAction(false, true, false))
                .build();

        Action pickAndSnapSpecimenTwo = drive.actionBuilder(new Pose2d(PICK_SPECIMEN_SLOW, heading))
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


            Actions.runBlocking(
                    new SequentialAction(
                        new ParallelAction(
                                armControl.GetArmVerticalActionSequence(),
                                positionToPickSpikes
                        ),
                        armControl.GetPickSampleActionSequence(),
                        //new SleepAction(100), //sleep for 100 ms to pick the spike up
                        armControl.GetDropSampleObZoneActionSequence()
                    )
            );

            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER DROPPING FIRST SPIKE: " + timer.milliseconds());

            /*
            Actions.runBlocking(
                    new SequentialAction(
                            moveToPickLeftSpike,
                            armControl.GetPickSpikeActionSequence(),
                            new SleepAction(100), //sleep for 100 ms to pick the spike up
                            armControl.GetDropSampleObZoneActionSequence()
                    )
            );


            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER DROPPING SECOND SPIKE: " + timer.milliseconds());

            Actions.runBlocking(
                    new SequentialAction(
                            moveToPickRightSpike,
                            armControl.GetPickSpikeActionSequence(),
                            new SleepAction(100), //sleep for 100 ms to pick the spike up
                            armControl.GetDropSampleObZoneActionSequence()
                    )
            );

            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER DROPPING THIRD SPIKE: " + timer.milliseconds());

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

            //move to pick specimen 5
            Actions.runBlocking(
                    new ParallelAction(
                            armControl.GetPickSpecimenActionSequence(),
                            moveToPickSpecimenFour
                    )
            );

            //pick and snap specimen 5
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    armControl.GetHangSpecimenActionSequence(),
                                    pickAndSnapSpecimenFour
                            ),
                            armControl.GetSnapSpecimenActionSequence()
                    )
            );

            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER SNAPPING SPECIMEN 5: " + timer.milliseconds());

            Actions.runBlocking(
                    new ParallelAction(
                            park,
                            armControl.GetRestingActionSequenceNoWait()
                    )
            );




            telemetry.addData("Elapsed Time: TOTAL: ", timer.milliseconds());
            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME: " + timer.milliseconds());
            telemetry.update();


             */
            break;
        }
    }
}
