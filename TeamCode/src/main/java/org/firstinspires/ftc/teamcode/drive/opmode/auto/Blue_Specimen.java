package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
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

@Autonomous(name = "Blue_Specimen", group = "Autonomous")
public class Blue_Specimen extends BaseAuto{
    private static final int multiplier = 1;    //used to flip coordinates between blue (1) and red (-1)

    private final SpecimenAutoCoordinates coordinates = new SpecimenAutoCoordinates(multiplier);

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this.hardwareMap);
        robotControl = new RobotControl(gamepad2, robotHardware, telemetry);
        mecanumDrive = new MecanumDrive(this.hardwareMap, coordinates.INIT_POS);


        robotControl.setGameColor(GameConstants.GAME_COLORS.BLUE);
        robotControl.SetCurrentRobotStateToEnterExitSub();
        robotHardware.startLimelight();

        //Put all the actions here, but replace the sample pushing with continuous splines below

        Action snapPreloadedSpecimen = mecanumDrive.actionBuilder(coordinates.INIT_POS)
                .strafeToConstantHeading(coordinates.BRACE_RUNGS_FOR_SPECIMEN_ONE.position)
                .build();

        Action newMoveToPickSample1 = mecanumDrive.actionBuilder(coordinates.BRACE_RUNGS_FOR_SPECIMEN_ONE)
                .setTangent(coordinates.reverseHeading)
                .splineToLinearHeading(coordinates.PICK_SAMPLE_1, coordinates.PICK_SAMPLE_1.heading, new TranslationalVelConstraint(32), new ProfileAccelConstraint(-32, 32))
                .build();

//        Action moveToPickSample1 = mecanumDrive.actionBuilder(coordinates.INIT_POS)
//                .splineToConstantHeading(coordinates.PICK_SAMPLE_1.position, coordinates.PICK_SAMPLE_1.heading)
//                .build();

        Action moveToPickSample2 = mecanumDrive.actionBuilder(coordinates.PICK_SAMPLE_1)
                .strafeToConstantHeading(coordinates.PICK_SAMPLE_2.position)
                .build();

//        Action moveToPickSample3 = mecanumDrive.actionBuilder(coordinates.PICK_SAMPLE_2)
//                .strafeToConstantHeading(coordinates.PICK_SAMPLE_3.position)
//                .build();

        Action moveToPickSpecimenStep1 = mecanumDrive.actionBuilder(coordinates.PICK_SAMPLE_3)
                .setTangent(coordinates.heading)
                .splineToLinearHeading(coordinates.PICK_SPECIMEN, coordinates.reverseHeading)
                .build();

        Action moveToPickSpecimenStep2 = mecanumDrive.actionBuilder(coordinates.PICK_SPECIMEN)
                .setTangent(coordinates.reverseHeading)
                .lineToY(coordinates.PICK_SPECIMEN_SLOW.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(coordinates.minAccel, 20))
                .build();

//        Action moveToSnapSpecimen1 = mecanumDrive.actionBuilder(coordinates.PICK_SPECIMEN_SLOW)
//                .setTangent(coordinates.heading)
//                .splineToConstantHeading(coordinates.BRACE_RUNGS_FOR_SPECIMEN_ONE.position, coordinates.heading)
//                .build();

//        Action moveToPickSpecimen2 = mecanumDrive.actionBuilder(coordinates.BRACE_RUNGS_FOR_SPECIMEN_ONE)
//                .setTangent(coordinates.reverseHeading)
//                .splineToConstantHeading(coordinates.PICK_SPECIMEN.position, coordinates.reverseHeading)
//                .lineToY(coordinates.PICK_SPECIMEN_SLOW.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(coordinates.minAccel, 20))
//                .build();

        Action moveToSnapSpecimen2 = mecanumDrive.actionBuilder(coordinates.PICK_SPECIMEN_SLOW)
                .setTangent(coordinates.heading)
                .splineToConstantHeading(coordinates.SPLINE_INTERMEDIATE_SNAP.position, coordinates.sweepLeftHeading)
                .splineToConstantHeading(coordinates.BRACE_RUNGS_FOR_SPECIMEN_TWO.position, coordinates.heading)
                .build();

//        Action strafeAfterBracingRungForSpecimen2 = mecanumDrive.actionBuilder(coordinates.BRACE_RUNGS_FOR_SPECIMEN_TWO)
//                .setTangent(0)
//                .strafeToConstantHeading(coordinates.BRACE_RUNGS_FOR_SPECIMEN_ONE.position)
//                .build();

        Action moveToPickSpecimen3 = mecanumDrive.actionBuilder(coordinates.BRACE_RUNGS_FOR_SPECIMEN_TWO)
                .setTangent(coordinates.reverseHeading)
                .splineToConstantHeading(coordinates.SPLINE_INTERMEDIATE_PICK.position, coordinates.SPLINE_INTERMEDIATE_PICK.heading)
                .splineToConstantHeading(coordinates.PICK_SPECIMEN.position, coordinates.reverseHeading)
                .lineToY(coordinates.PICK_SPECIMEN_SLOW.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(coordinates.minAccel, 20))
                .build();

        Action moveToSnapSpecimen3 = mecanumDrive.actionBuilder(coordinates.PICK_SPECIMEN_SLOW)
                .setTangent(coordinates.heading)
                .splineToConstantHeading(coordinates.SPLINE_INTERMEDIATE_SNAP.position, coordinates.sweepLeftHeading)
                .splineToConstantHeading(coordinates.BRACE_RUNGS_FOR_SPECIMEN_THREE.position, coordinates.heading)
                .build();

        Action strafeAfterBracingRungForSpecimen3 = mecanumDrive.actionBuilder(coordinates.BRACE_RUNGS_FOR_SPECIMEN_THREE)
                .setTangent(0)
                .strafeToConstantHeading(coordinates.BRACE_RUNGS_FOR_SPECIMEN_ONE.position)
                .build();

        Action moveToPickSpecimen4 = mecanumDrive.actionBuilder(coordinates.BRACE_RUNGS_FOR_SPECIMEN_THREE)
                .setTangent(coordinates.reverseHeading)
                .splineToConstantHeading(coordinates.SPLINE_INTERMEDIATE_PICK.position, coordinates.SPLINE_INTERMEDIATE_PICK.heading)
                .splineToConstantHeading(coordinates.PICK_SPECIMEN.position, coordinates.reverseHeading)
                .lineToY(coordinates.PICK_SPECIMEN_SLOW.position.y, new TranslationalVelConstraint(20), new ProfileAccelConstraint(coordinates.minAccel, 20))
                .build();

        Action moveToSnapSpecimen4 = mecanumDrive.actionBuilder(coordinates.PICK_SPECIMEN_SLOW)
                .setTangent(coordinates.heading)
                .splineToConstantHeading(coordinates.SPLINE_INTERMEDIATE_SNAP.position, coordinates.sweepLeftHeading)
                .splineToConstantHeading(coordinates.BRACE_RUNGS_FOR_SPECIMEN_FOUR.position, coordinates.heading)
                .build();

        Action strafeAfterBracingRungForSpecimen4 = mecanumDrive.actionBuilder(coordinates.BRACE_RUNGS_FOR_SPECIMEN_FOUR)
                .setTangent(0)
                .strafeToConstantHeading(coordinates.BRACE_RUNGS_FOR_SPECIMEN_ONE.position)
                .build();

        Action park = mecanumDrive.actionBuilder(coordinates.BRACE_RUNGS_FOR_SPECIMEN_FOUR)
                .setTangent(coordinates.reverseHeading)
                .splineToConstantHeading(coordinates.PARK.position, coordinates.PARK.heading)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            //snap preloaded specimen
            Actions.runBlocking(
                    new ParallelAction(
                            new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_HANG_SPECIMEN, false, false),
                            snapPreloadedSpecimen,
                            robotControl.GetHangSpecimenActionSequence_Fast()
                    )
            );

            Actions.runBlocking(
                    robotControl.GetSnapSpecimenActionSequence()
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER SNAPPING PRELOAD: " + timer.milliseconds());

            Actions.runBlocking(
                    new ParallelAction(
                            newMoveToPickSample1,
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
                            new SleepAction(0.2),
                            new VerticalClawAction(robotHardware, false, true, false),
                            new ParallelAction(
                                    robotControl.GetHangSpecimenActionSequence_Fast(),
                                    moveToSnapSpecimen2
                            )
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
//                            strafeAfterBracingRungForSpecimen2,
                            robotControl.GetSnapSpecimenActionSequence()
                    )
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
                            new SleepAction(0.2),
                            new VerticalClawAction(robotHardware, false, true, false),
                            new ParallelAction(
                                    robotControl.GetHangSpecimenActionSequence_Fast(),
                                    moveToSnapSpecimen3
                            )
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
//                            strafeAfterBracingRungForSpecimen3,
                            robotControl.GetSnapSpecimenActionSequence()
                    )
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER SNAPPING SPECIMEN 3: " + timer.milliseconds());

            //pick and snap specimen 4
            Actions.runBlocking(
                    new ParallelAction(
                            moveToPickSpecimen4,
                            robotControl.GetPickSpecimenActionSequence(false)
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            new SleepAction(0.2),
                            new VerticalClawAction(robotHardware, false, true, false),
                            new ParallelAction(
                                    robotControl.GetHangSpecimenActionSequence_Fast(),
                                    moveToSnapSpecimen4
                            )
                    )
            );

            Actions.runBlocking(
                    new ParallelAction(
//                            strafeAfterBracingRungForSpecimen4,
                            robotControl.GetSnapSpecimenActionSequence()
                    )
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER SNAPPING SPECIMEN 4: " + timer.milliseconds());


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
            /*

//            pick sample 1
            Actions.runBlocking(
                    new ParallelAction(
                            new InstantAction(() -> {
                                List<HorizontalPickupVector> choices = new ArrayList<>();
                                choices.add(robotControl.GetHorizontalPickupVectorFromLimelightLocation(coordinates.SAMPLE1_LIMELIGHT_LOCATION));
                                robotControl.SetSampleChoices(choices);
                            }),
                            moveToPickSample1,
                            robotControl.GetEnterExitSubActionSequence(true)
                    )
            );

            Actions.runBlocking(
                    robotControl.GetPickSampleActionSequence(false)
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER PICKING SAMPLE 1: " + timer.milliseconds());

            Actions.runBlocking(
                    new ParallelAction(
//                            robotControl.GetTransferToObZoneActionSequence(),
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
                    new ParallelAction(
                            robotControl.GetTransferToObZoneActionSequence(false),
                            robotControl.GetPickSampleActionSequence(false)
                    )
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER PICKING SAMPLE 2: " + timer.milliseconds());

            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    robotControl.GetTransferSampleActionSequence(),
                                    robotControl.GetEnterExitSubActionSequence(true)
                            ),
                            moveToPickSample3,
                            new InstantAction(() -> {
                                List<HorizontalPickupVector> choices = new ArrayList<>();
                                choices.add(robotControl.GetHorizontalPickupVectorFromLimelightLocation(coordinates.SAMPLE3_LIMELIGHT_LOCATION));
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
                            new InstantAction(()-> {
                                robotHardware.setHorizontalShoulderServoPosition(RobotConstants.HORIZONTAL_SHOULDER_ENTER_EXIT_SUB);
                                robotHardware.setHorizontalTurretServoPosition(RobotConstants.HORIZONTAL_TURRET_CENTER);
                            })
                    )
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER PICKING SAMPLE 3: " + timer.milliseconds());

            Actions.runBlocking(
                    new SequentialAction(
                            moveToPickSpecimenStep1,
                            new InstantAction(() -> robotHardware.setHorizontalClawState(true)),
                            new SleepAction(0.2),
                            new ParallelAction(
                                    robotControl.GetPickSpecimenActionSequence(),
                                    moveToPickSpecimenStep2
                            )
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            new VerticalClawAction(robotHardware, false, true, false),
                            new ParallelAction(
                                    robotControl.GetHangSpecimenActionSequence(),
                                    moveToSnapSpecimen1
                            )
                    )
            );

            Actions.runBlocking(
                    robotControl.GetSnapSpecimenActionSequence()
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "SPECIMEN 1 X: " + mecanumDrive.localizer.getPose().position.x);
            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "SPECIMEN 1 Y: " + mecanumDrive.localizer.getPose().position.y);
            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "SPECIMEN 1 HEADING: " + Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble()));


            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER SNAPPING SPECIMEN 1: " + timer.milliseconds());


            //pick and snap specimen 2
            Actions.runBlocking(
                    new ParallelAction(
                            moveToPickSpecimen2,
                            robotControl.GetPickSpecimenActionSequence()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            new VerticalClawAction(robotHardware, false, true, false),
                            new ParallelAction(
                                    robotControl.GetHangSpecimenActionSequence(),
                                    moveToSnapSpecimen2
                            )
                    )
            );

            Actions.runBlocking(
                    robotControl.GetSnapSpecimenActionSequence()
            );
            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "SPECIMEN 2 X: " + mecanumDrive.localizer.getPose().position.x);
            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "SPECIMEN 2 Y: " + mecanumDrive.localizer.getPose().position.y);
            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "SPECIMEN 2 HEADING: " + Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble()));

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER SNAPPING SPECIMEN 2: " + timer.milliseconds());

            //pick and snap specimen 3
            Actions.runBlocking(
                    new ParallelAction(
                            moveToPickSpecimen3,
                            robotControl.GetPickSpecimenActionSequence()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            new VerticalClawAction(robotHardware, false, true, false),
                            new ParallelAction(
                                    robotControl.GetHangSpecimenActionSequence(),
                                    moveToSnapSpecimen3
                            )
                    )
            );

            Actions.runBlocking(
                    robotControl.GetSnapSpecimenActionSequence()
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "SPECIMEN 3 X: " + mecanumDrive.localizer.getPose().position.x);
            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "SPECIMEN 3 Y: " + mecanumDrive.localizer.getPose().position.y);
            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "SPECIMEN 3 HEADING: " + Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble()));


            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER SNAPPING SPECIMEN 3: " + timer.milliseconds());

            //pick and snap specimen 4
            Actions.runBlocking(
                    new ParallelAction(
                            moveToPickSpecimen4,
                            robotControl.GetPickSpecimenActionSequence()
                    )
            );

            Actions.runBlocking(
                    new SequentialAction(
                            new VerticalClawAction(robotHardware, false, true, false),
                            new ParallelAction(
                                    robotControl.GetHangSpecimenActionSequence(),
                                    moveToSnapSpecimen4
                            )
                    )
            );

            Actions.runBlocking(
                    robotControl.GetSnapSpecimenActionSequence()
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "SPECIMEN 4 X: " + mecanumDrive.localizer.getPose().position.x);
            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "SPECIMEN 4 Y: " + mecanumDrive.localizer.getPose().position.y);
            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "SPECIMEN 4 HEADING: " + Math.toDegrees(mecanumDrive.localizer.getPose().heading.toDouble()));

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER SNAPPING SPECIMEN 3: " + timer.milliseconds());

            Actions.runBlocking(
                    park
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "TOTAL ELAPSED TIME: " + timer.milliseconds());

            break;

             */
    }

    robotHardware.stopLimelight();

}}
