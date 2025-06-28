package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GameConstants;
import org.firstinspires.ftc.teamcode.HorizontalPickupVector;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.RobotControl;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.actions.VerticalClawAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.actions.VerticalShoulderAction;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Blue_Specimen_v2", group = "Autonomous")
public class Blue_Specimen_v2 extends BaseAuto{
    private static final int multiplier = 1;    //used to flip coordinates between blue (1) and red (-1)

    private final SpecimenAutoCoordinates coordinates = new SpecimenAutoCoordinates(multiplier);

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this.hardwareMap);
        robotControl = new RobotControl(gamepad2, robotHardware, telemetry);
        mecanumDrive = new MecanumDrive(this.hardwareMap, coordinates.INIT_POS);

        robotControl.setGameColor(GameConstants.GAME_COLORS.BLUE);
        robotControl.SetCurrentRobotStateToEnterExitSub();
//        robotHardware.startLimelight();

        //Put all the actions here, but replace the sample pushing with continuous splines below

        Action snapPreloadedSpecimen = mecanumDrive.actionBuilder(coordinates.INIT_POS)
                .strafeToConstantHeading(coordinates.BRACE_RUNGS_FOR_SPECIMEN_ONE.position)
                .build();

        Action newMoveToPickSample1Step1 = mecanumDrive.actionBuilder(coordinates.BRACE_RUNGS_FOR_SPECIMEN_ONE)
                .setTangent(coordinates.reverseHeading)
                .splineToLinearHeading(new Pose2d(coordinates.FRONT_OF_RUNGS, coordinates.reverseHeading), coordinates.reverseHeading)
                .build();

        Action newMoveToPickSample1Step2 = mecanumDrive.actionBuilder(new Pose2d(coordinates.FRONT_OF_RUNGS, coordinates.reverseHeading))
                .setTangent(coordinates.sweepRightHeading)
                .splineToLinearHeading(coordinates.PICK_SAMPLE_1, coordinates.PICK_SAMPLE_1.heading, new TranslationalVelConstraint(35), new ProfileAccelConstraint(-20, 35))
                .build();


        Action moveToPickSample2 = mecanumDrive.actionBuilder(coordinates.PICK_SAMPLE_1)
                .setTangent(coordinates.sweepRightHeading)
                .splineToLinearHeading(new Pose2d(coordinates.PICK_SAMPLE_2.position, coordinates.heading), coordinates.sweepRightHeading)
                .build();

        Action moveToPickSpecimenStep1 = mecanumDrive.actionBuilder(coordinates.PICK_SAMPLE_2)
                .setTangent(coordinates.heading)
                .splineToLinearHeading(coordinates.PICK_SPECIMEN, coordinates.reverseHeading)
                .build();

        Action moveToPickSpecimenStep2 = mecanumDrive.actionBuilder(coordinates.PICK_SPECIMEN)
                .setTangent(coordinates.reverseHeading)
                .splineToLinearHeading(coordinates.PICK_SPECIMEN_SLOW, coordinates.reverseHeading, new TranslationalVelConstraint(20), new ProfileAccelConstraint(coordinates.minAccel, 20))
                .build();

        Action moveToSnapSpecimen2 = mecanumDrive.actionBuilder(coordinates.PICK_SPECIMEN_SLOW)
                .setTangent(coordinates.sweepLeftHeading)
                .splineToLinearHeading(coordinates.BRACE_RUNGS_FOR_SPECIMEN_TWO, coordinates.heading, new TranslationalVelConstraint(coordinates.minTransVelocity), new ProfileAccelConstraint(coordinates.minAccel, coordinates.maxAccel))
                .lineToY(coordinates.BRACE_RUNGS_FOR_SPECIMEN_TWO_DELTA.position.y)
                .build();

        Action moveToPickSpecimen3 = mecanumDrive.actionBuilder(coordinates.BRACE_RUNGS_FOR_SPECIMEN_TWO)
                .setTangent(coordinates.reverseHeading)
                .splineToLinearHeading(coordinates.PICK_SPECIMEN, coordinates.sweepRightHeading)
                .setTangent(coordinates.reverseHeading)
                .lineToY(coordinates.PICK_SPECIMEN_SLOW.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(coordinates.minAccel, 20))
                .build();

        Action moveToSnapSpecimen3 = mecanumDrive.actionBuilder(coordinates.PICK_SPECIMEN_SLOW)
                .setTangent(coordinates.sweepLeftHeading)
                .splineToLinearHeading(coordinates.BRACE_RUNGS_FOR_SPECIMEN_THREE, coordinates.heading, new TranslationalVelConstraint(coordinates.minTransVelocity), new ProfileAccelConstraint(coordinates.minAccel, coordinates.maxAccel))
                .lineToY(coordinates.BRACE_RUNGS_FOR_SPECIMEN_THREE_DELTA.position.y)
                .build();


        Action moveToPickSpecimen4 = mecanumDrive.actionBuilder(coordinates.BRACE_RUNGS_FOR_SPECIMEN_THREE)
                .setTangent(coordinates.reverseHeading)
//                .splineToConstantHeading(coordinates.SPLINE_INTERMEDIATE_PICK.position, coordinates.SPLINE_INTERMEDIATE_PICK.heading)
                .splineToConstantHeading(coordinates.PICK_SPECIMEN.position, coordinates.sweepRightHeading)
                .setTangent(coordinates.reverseHeading)
                .lineToY(coordinates.PICK_SPECIMEN_SLOW.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(coordinates.minAccel, 20))
                .build();

        Action moveToSnapSpecimen4 = mecanumDrive.actionBuilder(coordinates.PICK_SPECIMEN_SLOW)
                .setTangent(coordinates.sweepLeftHeading)
                .splineToLinearHeading(coordinates.BRACE_RUNGS_FOR_SPECIMEN_FOUR, coordinates.heading, new TranslationalVelConstraint(coordinates.minTransVelocity), new ProfileAccelConstraint(coordinates.minAccel, coordinates.maxAccel))
//                .splineToConstantHeading(coordinates.BRACE_RUNGS_FOR_SPECIMEN_FOUR.position, coordinates.heading)
                .build();


        Action park = mecanumDrive.actionBuilder(coordinates.BRACE_RUNGS_FOR_SPECIMEN_THREE)
                .setTangent(coordinates.reverseHeading)
                .splineToConstantHeading(coordinates.PARK.position, coordinates.sweepRightHeading)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            //snap preloaded specimen
            Actions.runBlocking(
                    new ParallelAction(
                            new VerticalClawAction(robotHardware, false, false, false),
                            new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_HANG_SPECIMEN, false, false),
                            snapPreloadedSpecimen,
                            robotControl.GetHangSpecimenActionSequence_Fast()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            robotControl.GetSnapSpecimenActionSequence_Fast(),
                            newMoveToPickSample1Step1
                    )
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER SNAPPING PRELOAD: " + timer.milliseconds());

            Actions.runBlocking(
                    new ParallelAction(
                            newMoveToPickSample1Step2,
                            robotControl.GetEnterExitSubActionSequence(true),
                            new InstantAction(() -> {
                                List<HorizontalPickupVector> choices = new ArrayList<>();
                                choices.add(robotControl.GetHorizontalPickupVectorFromLimelightLocation(coordinates.SAMPLE1_LIMELIGHT_LOCATION));
                                robotControl.SetSampleChoices(choices);
                            })
                    )
            );

            Actions.runBlocking(
                    robotControl.GetPickSampleActionSequence(false)
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER PICKING SAMPLE 1: " + timer.milliseconds());

            Actions.runBlocking(
                    new ParallelAction(
                            robotControl.GetTransferSampleActionSequence(),
                            moveToPickSample2,
                            new InstantAction(() -> {
                                List<HorizontalPickupVector> choices = new ArrayList<>();
                                choices.add(robotControl.GetHorizontalPickupVectorFromLimelightLocation(coordinates.SAMPLE2_LIMELIGHT_LOCATION));
                                robotControl.SetSampleChoices(choices);
                            })
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    robotControl.GetTransferToObZoneActionSequence(false),
                                    robotControl.GetPickSampleActionSequence(false)
                            ),
                            robotControl.GetTransferToObZoneActionSequence(true)
                    )
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER PICKING SAMPLE 2: " + timer.milliseconds());

            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    moveToPickSpecimenStep1,
                                    robotControl.GetPickSpecimenActionSequence(true)
                            ),
                            new SleepAction(0.1),
                            moveToPickSpecimenStep2
                    )
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER TRANSFERRING 2 SAMPLES: " + timer.milliseconds());

            //pick and snap specimen 2
            Actions.runBlocking(
                    new SequentialAction(
                            new SleepAction(0.1),
                            new VerticalClawAction(robotHardware, false, true, false),
                            new ParallelAction(
                                    robotControl.GetHangSpecimenActionSequence_Fast(),
                                    moveToSnapSpecimen2
                            )
                    )
            );

            Actions.runBlocking(
                    robotControl.GetSnapSpecimenActionSequence_Fast()
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER SNAPPING SPECIMEN 2: " + timer.milliseconds());


            //pick and snap specimen 3
            Actions.runBlocking(
                    new ParallelAction(
                            moveToPickSpecimen3,
                            robotControl.GetPickSpecimenActionSequence(false)
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            new SleepAction(0.1),
                            new VerticalClawAction(robotHardware, false, true, false),
                            new ParallelAction(
                                    robotControl.GetHangSpecimenActionSequence_Fast(),
                                    moveToSnapSpecimen3
                            )
                    )
            );

            Actions.runBlocking(
                    robotControl.GetSnapSpecimenActionSequence_Fast()
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER SNAPPING SPECIMEN 3: " + timer.milliseconds());

            //pick and snap specimen 4
//            Actions.runBlocking(
//                    new ParallelAction(
//                            moveToPickSpecimen4,
//                            robotControl.GetPickSpecimenActionSequence(false)
//                    )
//            );
//
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new SleepAction(0.1),
//                            new VerticalClawAction(robotHardware, false, true, false),
//                            new ParallelAction(
//                                    robotControl.GetHangSpecimenActionSequence_Fast(),
//                                    moveToSnapSpecimen4
//                            )
//                    )
//            );
//
//            Actions.runBlocking(
//                    new ParallelAction(
//                            robotControl.GetSnapSpecimenActionSequence_Fast()
//                    )
//            );

//            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER SNAPPING SPECIMEN 4: " + timer.milliseconds());


            //park
            Actions.runBlocking(
                    new ParallelAction(
                            park,
                            new InstantAction(() -> {
                                robotHardware.setVerticalWristServoPosition(RobotConstants.VERTICAL_WRIST_TRANSFER);
                                robotHardware.setVerticalShoulderServoPosition(RobotConstants.VERTICAL_SHOULDER_PICK_SPECIMEN);
                                robotHardware.setVerticalSlidePosition(RobotConstants.VERTICAL_SLIDE_RESTING);
                            })
                    )
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "TOTAL ELAPSED TIME: " + timer.milliseconds());

            break;

        }

//    robotHardware.stopLimelight();

    }}
