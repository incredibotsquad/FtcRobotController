package org.firstinspires.ftc.teamcode.drive.opmode.auto;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.RobotControl;

@Autonomous(name = "Blue_4_Specimen", group = "Autonomous")
public class Blue_4_Specimen extends BaseAuto{
    private static final int multiplier = 1;    //used to flip coordinates between blue (1) and red (-1)

    private SpecimenCoordinates coordinates = new SpecimenCoordinates(multiplier);

    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this.hardwareMap);
        robotControl = new RobotControl(gamepad2, robotHardware, telemetry);
        mecanumDrive = new MecanumDrive(this.hardwareMap, coordinates.INIT_POS);

        //Put all the actions here, but replace the sample pushing with continuous splines below

        Action robotPreloadedSpecimen = mecanumDrive.actionBuilder(coordinates.INIT_POS)
                .strafeToConstantHeading(coordinates.BRACE_RUNGS_FOR_SPECIMEN_ONE.position)
                .build();

        Action moveBetweenSamplesOneAndTwo = mecanumDrive.actionBuilder(coordinates.BRACE_RUNGS_FOR_SPECIMEN_ONE)
                .setTangent(coordinates.reverseHeading)
                .splineToConstantHeading(coordinates.SLIDE_BETWEEN_SAMP1_SAMP2, coordinates.heading, new TranslationalVelConstraint(coordinates.minTransVelocity), new ProfileAccelConstraint(coordinates.minAccel, coordinates.maxAccel))
                .build();

        Action moveBetweenSamplesTwoAndThree = mecanumDrive.actionBuilder(coordinates.BRACE_RUNGS_FOR_SPECIMEN_ONE)
                .setTangent(coordinates.reverseHeading)
                .strafeToConstantHeading(coordinates.SLIDE_BETWEEN_SAMP2_SAMP3, new TranslationalVelConstraint(coordinates.minTransVelocity), new ProfileAccelConstraint(coordinates.minAccel, coordinates.maxAccel))
                .build();



//        Action SamplePushingWithSplines = mecanumDrive.actionBuilder(coordinates.BRACE_RUNGS_FOR_SPECIMEN_ONE)
//                .setTangent(coordinates.reverseHeading)
//                .splineToConstantHeading(coordinates.SLIDE_NEXT_TO_SAMP_1, coordinates.heading, new TranslationalVelConstraint(coordinates.minTransVelocity), new ProfileAccelConstraint(coordinates.minAccel, coordinates.maxAccel))
//                .splineToConstantHeading(coordinates.SLIDE_BEHIND_SAMP_1, coordinates.reverseHeading, new TranslationalVelConstraint(coordinates.minTransVelocity), new ProfileAccelConstraint(coordinates.minAccel, coordinates.maxAccel))
//                .splineToConstantHeading(coordinates.PUSH_SAMP_1, coordinates.reverseHeading, new TranslationalVelConstraint(coordinates.minTransVelocity), new ProfileAccelConstraint(coordinates.minAccel, coordinates.maxAccel))
//                .splineToConstantHeading(coordinates.SLIDE_BEHIND_SAMP_1, coordinates.heading, new TranslationalVelConstraint(coordinates.minTransVelocity), new ProfileAccelConstraint(coordinates.minAccel, coordinates.maxAccel))
//                .splineToConstantHeading(coordinates.SLIDE_BEHIND_SAMP_2, coordinates.reverseHeading, new TranslationalVelConstraint(coordinates.minTransVelocity), new ProfileAccelConstraint(coordinates.minAccel, coordinates.maxAccel))
//                .splineToConstantHeading(coordinates.PUSH_SAMP_2, coordinates.reverseHeading, new TranslationalVelConstraint(coordinates.minTransVelocity), new ProfileAccelConstraint(coordinates.minAccel, coordinates.maxAccel))
//                .build();
//
//        Action moveToPickSpecimenTwo = mecanumDrive.actionBuilder(new Pose2d(coordinates.PUSH_SAMP_2, coordinates.heading))
//                .setTangent(coordinates.heading)
//                .splineToConstantHeading(coordinates.PICK_SPECIMEN_SLOW, coordinates.reverseHeading, new TranslationalVelConstraint(20), new ProfileAccelConstraint(coordinates.minAccel, 20))
//                .stopAndAdd(GetClawControlAction(false, true, false))
//                .build();
//
//        Action pickAndSnapSpecimenTwo = mecanumDrive.actionBuilder(new Pose2d(coordinates.PICK_SPECIMEN, coordinates.heading))
//                .strafeToConstantHeading(BRACE_RUNGS_FOR_SPECIMEN_TWO.position)
//                .build();
//
//        Action moveToPickSpecimenThree = mecanumDrive.actionBuilder(BRACE_RUNGS_FOR_SPECIMEN_TWO)
//                .setTangent(reverseHeading)
//                .splineToConstantHeading(PICK_SPECIMEN, reverseHeading, new TranslationalVelConstraint(minTransVelocity + 20), new ProfileAccelConstraint(minAccel - 5 , maxAccel + 5))
//                .stopAndAdd(GetClawControlAction(false, true, false))
//                .build();
//
//        Action pickAndSnapSpecimenThree = mecanumDrive.actionBuilder(new Pose2d(PICK_SPECIMEN, heading))
//                .strafeToConstantHeading(BRACE_RUNGS_FOR_SPECIMEN_THREE.position)
//                .build();
//
//        Action moveToPickSpecimenFour = mecanumDrive.actionBuilder(BRACE_RUNGS_FOR_SPECIMEN_THREE)
//                .setTangent(reverseHeading)
//                .splineToConstantHeading(PICK_SPECIMEN, reverseHeading, new TranslationalVelConstraint(minTransVelocity + 20), new ProfileAccelConstraint(minAccel - 5, maxAccel + 5))
//                .stopAndAdd(GetClawControlAction(false, true, false))
//                .build();
//
//        Action pickAndSnapSpecimenFour = mecanumDrive.actionBuilder(new Pose2d(PICK_SPECIMEN, heading))
//                .setTangent(Math.toRadians(-90 * multiplier + 90))
//                .splineToConstantHeading(BRACE_RUNGS_FOR_SPECIMEN_FOUR.position, heading, new TranslationalVelConstraint(minTransVelocity + 20), new ProfileAccelConstraint(minAccel-10, maxAccel + 10))
//                .build();
//
//        Action park = mecanumDrive.actionBuilder(BRACE_RUNGS_FOR_SPECIMEN_FOUR)
//                .setTangent(reverseHeading)
//                .splineToConstantHeading(PARK.position, PARK.heading)
//                .build();
//
        waitForStart();
//
        while (opModeIsActive()) {
            ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

            //snap preloaded specimen
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    robotPreloadedSpecimen,
                                    robotControl.GetHangSpecimenActionSequence()
                            ),
                            robotControl.GetSnapSpecimenActionSequence()
                    )
            );

            Log.i("=== INCREDIBOTS / SPECIMEN AUTO ===", "ELAPSED TIME AFTER PRELOADED SNAP: " + timer.milliseconds());

//
//            //snap preloaded specimen
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new ParallelAction(
//                                    robotControl.GetHangSpecimenActionSequence_Fast(),
//                                    robotPreloadedSpecimen),
//                            robotControl.GetSnapSpecimenActionSequence()
//                    )
//            );
//
//            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER PRELOADED SNAP: " + timer.milliseconds());
//
//
//            //push in samples on spike marks
//            Actions.runBlocking(
//                    new ParallelAction(
//                            robotControl.GetArmHorizontalActionSequence(),
//                            SamplePushingWithSplines)
//            );
//
//            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER PUSHING IN SPIKES: " + timer.milliseconds());
//
//
//            //move to pick specimen 2
//            Actions.runBlocking(
//                    new ParallelAction(
//                            robotControl.GetPickSpecimenActionSequence(),
//                            moveToPickSpecimenTwo
//                    )
//            );
//
//            //pick and snap specimen 2
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new ParallelAction(
//                                    robotControl.GetHangSpecimenActionSequence(),
//                                    pickAndSnapSpecimenTwo
//                            ),
//                            robotControl.GetSnapSpecimenActionSequence()
//                    ));
//
//            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER SNAPPING SPECIMEN 2: " + timer.milliseconds());
//
//
//            //move to pick specimen 3
//            Actions.runBlocking(
//                    new ParallelAction(
//                            robotControl.GetPickSpecimenActionSequence(),
//                            moveToPickSpecimenThree
//                    )
//            );
//
//            //pick and snap specimen3
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new ParallelAction(
//                                    robotControl.GetHangSpecimenActionSequence(),
//                                    pickAndSnapSpecimenThree
//                            ),
//                            robotControl.GetSnapSpecimenActionSequence()
//                    )
//            );
//
//            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER SNAPPING SPECIMEN 3: " + timer.milliseconds());
//
//
//            //move to pick specimen 4
//            Actions.runBlocking(
//                    new ParallelAction(
//                            robotControl.GetPickSpecimenActionSequence(),
//                            moveToPickSpecimenFour
//                    )
//            );
//
//            //pick and snap specimen 4
//            Actions.runBlocking(
//                    new SequentialAction(
//                            new ParallelAction(
//                                    robotControl.GetHangSpecimenActionSequence(),
//                                    pickAndSnapSpecimenFour
//                            ),
//                            robotControl.GetSnapSpecimenActionSequence()
//                    )
//            );
//
//            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME AFTER SNAPPING SPECIMEN 4: " + timer.milliseconds());
//
//            Actions.runBlocking(
//                    new ParallelAction(
//                            park,
//                            robotControl.GetRestingActionSequenceNoWait()
//                    )
//            );
//
//            telemetry.addData("Elapsed Time: TOTAL: ", timer.milliseconds());
//            Log.i("=== INCREDIBOTS  ===", "ELAPSED TIME: " + timer.milliseconds());
//            telemetry.update();
//
//            break;
//        }
    }
}}
