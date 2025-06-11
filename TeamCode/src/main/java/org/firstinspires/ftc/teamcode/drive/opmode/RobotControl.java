package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ColorSenorOutput;
import org.firstinspires.ftc.teamcode.GameConstants;
import org.firstinspires.ftc.teamcode.HorizontalPickupVector;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.actions.HorizontalClawAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.actions.HorizontalElbowAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.actions.HorizontalShoulderAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.actions.HorizontalSlideAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.actions.HorizontalTurretAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.actions.HorizontalWristAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.actions.VerticalClawAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.actions.VerticalElbowAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.actions.VerticalShoulderAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.actions.VerticalSlideAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.actions.VerticalWristAction;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class RobotControl
{
    GameConstants.GAME_COLORS gameColor;

    private Gamepad gamepad2;
    private RobotHardware robotHardware;
    private FtcDashboard dashboard;
    private List<Action> runningActions;

    public TensorFlow VisionCalibration;
    OpenCvWebcam webcam;
    SampleDetectionPipelineV2 sampleDetectionPipeline;

    List<HorizontalPickupVector> sampleChoices;

    private enum ROBOT_STATE {
        NONE,
        RESTING,
        RESET_ENCODERS,
        ENTER_EXIT_SUB,
        PICK_SAMPLE,
        TRANSFER_SAMPLE,
        TRANSFER_TO_OB_ZONE,
        LOW_BASKET,
        HIGH_BASKET,
        PICK_SPECIMEN,
        SNAP_SPECIMEN,
        ROBOT_HANG
    }

    private ROBOT_STATE currentRobotState;
    private ROBOT_STATE targetRobotState;
    private boolean stateTransitionInProgress;

    private Telemetry telemetry;

    private boolean readyToPickSample;

    public RobotControl(Gamepad gamepad, RobotHardware robotHardware, Telemetry telemetry) {
        gamepad2 = gamepad;
        this.telemetry = telemetry;
        this.robotHardware = robotHardware;
        currentRobotState = ROBOT_STATE.NONE;
        targetRobotState = ROBOT_STATE.NONE;
        stateTransitionInProgress = false;
        dashboard = FtcDashboard.getInstance();
        runningActions = new ArrayList<>();
        readyToPickSample = false;
        sampleChoices = new ArrayList<>();

        VisionCalibration = new TensorFlow();
        int camMonitorViewId = robotHardware.hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", robotHardware.hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                robotHardware.hardwareMap.get(WebcamName.class, "Webcam 1"), camMonitorViewId);

        sampleDetectionPipeline = new SampleDetectionPipelineV2();
        webcam.setPipeline(sampleDetectionPipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            //override onOpened to start streaming
            @Override
            public void onOpened() {
                webcam.startStreaming(RobotConstants.IMAGE_WIDTH, RobotConstants.IMAGE_HEIGHT, OpenCvCameraRotation.UPRIGHT);
            }
            //Handle error handling
            @Override
            public void onError(int errorCode) {
                telemetry.addData("Got error number: ", errorCode);
            }
        });
    }

    public void ProcessInputs(Telemetry telemetry) {

        CreateStateFromButtonPress();

        TranslateStateIntoActions();

        ProcessActions();

//        ProcessSafetyChecks();
//
        ProcessDPad();
        ProcessBumpers();
        ProcessJoystick();
        ProcessPickSampleState();
    }

    private void ProcessSafetyChecks() {
        robotHardware.horizontalSlideSafetyChecks();
        robotHardware.verticalSlideSafetyChecks();
    }

    public void setGameColor(GameConstants.GAME_COLORS gameColor) {
        sampleDetectionPipeline.setColorMode(gameColor);
        this.gameColor = gameColor;
    }

    //Function to create a state from Gamepad inputs
    private void CreateStateFromButtonPress() {

        ROBOT_STATE newTargetRobotState =  targetRobotState;

        //Back button maps to resting
        if (gamepad2.back) {
            newTargetRobotState = ROBOT_STATE.RESTING;
        }

        //Back + Start to reset the slide encoder
        if (gamepad2.back && gamepad2.start) {
            newTargetRobotState = ROBOT_STATE.RESET_ENCODERS;
        }

        //Hang the robot with start + left/right triggers
        if (gamepad2.start && gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
            newTargetRobotState = ROBOT_STATE.ROBOT_HANG;
        }

        if (gamepad2.a){
            //left trigger + A to pick specimen
            if (gamepad2.left_trigger > 0) {
                newTargetRobotState = ROBOT_STATE.PICK_SPECIMEN;
            }
            //right trigger + A to pick samples
            else if (gamepad2.right_trigger > 0) {
                newTargetRobotState = ROBOT_STATE.PICK_SAMPLE;
            }
        }

        if (gamepad2.b) {
            //left trigger + B to snap specimen
            if (gamepad2.left_trigger > 0) {
                newTargetRobotState = ROBOT_STATE.SNAP_SPECIMEN;
            }
            // right trigger + B for low basket
            else if (gamepad2.right_trigger > 0) {
                newTargetRobotState = ROBOT_STATE.LOW_BASKET;
            }
        }

        if (gamepad2.x) {
            if (gamepad2.left_trigger > 0) {
                newTargetRobotState = ROBOT_STATE.TRANSFER_TO_OB_ZONE;
            }
            else if (gamepad2.right_trigger > 0) {
                newTargetRobotState = ROBOT_STATE.ENTER_EXIT_SUB;
            }
        }

        if (gamepad2.y) {
            if (gamepad2.right_trigger > 0) {
                newTargetRobotState = ROBOT_STATE.HIGH_BASKET;
            }
        }

        if (newTargetRobotState != currentRobotState) {

            Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "TRANSITIONING STATE CURRENT: " + currentRobotState + " TARGET: " + newTargetRobotState);

            if (newTargetRobotState != targetRobotState) {
                //STOP ALL PROCESSING - STATE TRANSITION WAS GOING ON WHEN NEW STATE WAS CALLED IN
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "STOPPING! STATE TRANSITION WAS GOING ON WHEN NEW TARGET WAS CALLED IN. OLD TARGET: " + targetRobotState + " NEW TARGET: " + newTargetRobotState);

                robotHardware.stopRobotAndMechanisms();
                runningActions.clear();
                stateTransitionInProgress = false;
            }

            targetRobotState = newTargetRobotState;
        }
    }


    private void GetSampleChoicesFromCameraInputs() {

        sampleChoices.clear();  //remove any previous choices

        if (sampleDetectionPipeline.latestRects != null && sampleDetectionPipeline.latestDistances != null) {

            for (int loop = 0; loop < sampleDetectionPipeline.latestRects.length; loop++) {
                double realX = sampleDetectionPipeline.GetRealXinches(loop);
                double realY = sampleDetectionPipeline.GetRealYinches(loop);
                double realOrientation = sampleDetectionPipeline.GetRealSampleOrientation(loop);

                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Real Vertical: " + realY);
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Real Horizontal: " + realX);
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Real Orientation: " + realOrientation);

                TensorFlow.CalibrationResult result = VisionCalibration.calibrate((float) (realX - RobotConstants.TURRET_OFFSET_FROM_CAMERA), (float) realY, (float) realOrientation);

                double calibratedYOffset = result.calibratedY;
                double calibratedXOffset = result.calibratedX;
                double calibratedSampleOrientation = result.calibratedAngle;

                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Calibrated Vertical: " + calibratedYOffset);
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Calibrated Horizontal: " + calibratedXOffset);
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Calibrated Orientation: " + calibratedSampleOrientation);

                double accountForPickupArm = Math.sqrt((RobotConstants.PICKUP_ARM_LENGTH * RobotConstants.PICKUP_ARM_LENGTH) - (calibratedXOffset * calibratedXOffset));
                int horizontalSlidePosition = (int) ((calibratedYOffset - accountForPickupArm) * RobotConstants.HORIZONTAL_SLIDE_TICKS_PER_INCH);

                if (horizontalSlidePosition < 0 || horizontalSlidePosition > RobotConstants.HORIZONTAL_SLIDE_MAX_POS) continue; // no need to add an option which we cannot reach
                if (Math.abs(calibratedXOffset) >= RobotConstants.PICKUP_ARM_LENGTH) continue;    // cannot reach beyond pickup arm length

                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. horizontalSlidePosition: " + horizontalSlidePosition);

                double turretMovementAngle = Math.toDegrees(Math.atan(calibratedXOffset / accountForPickupArm));

                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. turretMovementAngle: " + turretMovementAngle);

                double turretServoPos = RobotConstants.TURRET_CENTER_POSITION - (turretMovementAngle / 300);

                double clawParallelOrientation = RobotConstants.HORIZONTAL_WRIST_TRANSFER + (turretMovementAngle / 300);    //this will keep the claw parallel to the robot

                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. clawParallelOrientation: " + clawParallelOrientation);

                double wristTargetAngle = 180 - calibratedSampleOrientation - 90;   //wrist has to be perpendicular to the sample
                double horizontalWristPosition = clawParallelOrientation + (wristTargetAngle / 300);

                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. horizontalWristPosition: " + horizontalWristPosition);

                sampleChoices.add(new HorizontalPickupVector(horizontalSlidePosition, turretServoPos, horizontalWristPosition));
            }

            //sort by vertical distance
            sampleChoices.sort(Comparator.comparingInt(choice -> choice.slidePosition));
        }

        Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "==================================================================");
        Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Choice Count: " + sampleChoices.size());
        Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "==================================================================");

        for (HorizontalPickupVector choice: sampleChoices) {
            Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. horizontalSlidePosition: " + choice.slidePosition);
            Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. turretMovementAngle: " + choice.turretPosition);
            Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. horizontalWristPosition: " + choice.clawOrientation);
            Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "==================================================================");
        }
    }


    //function to take the new target state and turn it into robot movements
    private void TranslateStateIntoActions() {

//        if (currentRobotState == targetRobotState) return;

        if (!stateTransitionInProgress) {

            switch (targetRobotState) {
                case RESTING:
                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: RESTING");

                    //get the list of actions and put it in running actions
                    runningActions.add(GetRestingActionSequence());

                    break;

                case RESET_ENCODERS:
                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: RESET ENCODERS");

                    //TODO: add code here
                    runningActions.add(GetResetEncodersActionSequence());
                    break;

                case ENTER_EXIT_SUB:
                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: ENTER EXIT SUB");

                    runningActions.add(GetEnterExitSubActionSequence());
                    break;

                case PICK_SAMPLE:   //PICK SAMPLE
                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: PICK SAMPLE");

                    runningActions.add(GetPickSampleActionSequence());

                    break;

                case TRANSFER_SAMPLE:
                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: TRANSFER SAMPLE");

                    runningActions.add(GetTransferSampleActionSequence());

                    break;

                case TRANSFER_TO_OB_ZONE:
                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: TRANSFER TO OBSERVATION ZONE");

                    runningActions.add(GetTransferToObZoneActionSequence());

                    break;

                case LOW_BASKET:   //LOW BASKET
                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: LOW BASKET");

                    runningActions.add(GetLowBasketActionSequence());

                    break;
                case HIGH_BASKET:   //HIGH BASKET
                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: HIGH BASKET");

                    runningActions.add(GetHighBasketActionSequence());

                    break;

                case PICK_SPECIMEN:   //PICK SPECIMEN
                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: PICK SPECIMEN");

                    runningActions.add(GetPickSpecimenActionSequence());

                    break;

                case SNAP_SPECIMEN:   //SNAP SPECIMEN
                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: SNAP SPECIMEN");

                    runningActions.add(GetSnapSpecimenActionSequence());

                    break;

                case ROBOT_HANG: //SLIDE CLOSED, ARM RESTING
                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: ROBOT HANG");

                    //TODO: add code here

                    break;
            }

        }
    }

    private void ProcessActions()
    {
        TelemetryPacket packet = new TelemetryPacket();

        // we have actions to run, state transition in proogress
        if (!runningActions.isEmpty()) {
            stateTransitionInProgress = true;
        }

        // run actions and add pending ones to new list
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());

            if (action.run(packet)) {
                newActions.add(action); //add if action indicates it needs to run again
            }
        }
        runningActions = newActions;
        dashboard.sendTelemetryPacket(packet);

        if (runningActions.isEmpty()) {
            Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "DONE RUNNING ACTIONS");
            currentRobotState = targetRobotState;
            targetRobotState = ROBOT_STATE.NONE;
            stateTransitionInProgress = false;

            //NOTE: CAREFUL NOT TO CONSTRUCT LOOPS HERE
            // STATE A -> STATE B -> STATE A
            //set new state based on older target

            //TODO: IF WE GO FROM ENTER SUB TO PICK SAMPLE, GO BACK TO ENTER SUB
            //TODO: IF WE GO FROM RESTING TO PICK SAMPLE

            switch (targetRobotState) {
                case LOW_BASKET:
                case HIGH_BASKET:
//                    targetRobotState = ROBOT_STATE.PICK_SAMPLE;
                    break;
                case SNAP_SPECIMEN:
//                    targetRobotState = ROBOT_STATE.PICK_SPECIMEN;
                    break;
                default:
                    targetRobotState = ROBOT_STATE.NONE;
            }
        }
    }

    public void ProcessPickSampleState() {
        if (currentRobotState != ROBOT_STATE.PICK_SAMPLE || targetRobotState == ROBOT_STATE.TRANSFER_SAMPLE) return;

        Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "ProcessPickSampleState");

        ColorSenorOutput colorSenorOutput = robotHardware.getDetectedColorAndDistance();

        if (colorSenorOutput.detectedColor == this.gameColor || colorSenorOutput.detectedColor == GameConstants.GAME_COLORS.YELLOW) {
            if (colorSenorOutput.distance < RobotConstants.COLOR_SENSOR_DISTANCE_THRESHOLD) {
                robotHardware.setHorizontalClawState(false);
                this.targetRobotState = ROBOT_STATE.TRANSFER_SAMPLE;
            }
        }
    }

    public Action GetRestingActionSequence() {

        Action verticalActions = new ParallelAction(
                new VerticalClawAction(robotHardware, false, false, false),
                new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_RESTING, false, false),
                new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_RESTING, false, false),
                new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_RESTING, false, false),
                new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_RESTING, true, false)  //wait for slide
        );

        Action horizontalActions = new ParallelAction(
                new HorizontalTurretAction(robotHardware, RobotConstants.HORIZONTAL_TURRET_RESTING, false, false),
                new HorizontalClawAction(robotHardware, false, false, false),
                new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_RESTING, false, false),
                new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_RESTING, false, false),
                new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_RESTING, false, false),
                new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_RESTING, true, false)  //wait for slide
        );

        return new ParallelAction(
                verticalActions,
                horizontalActions);
    }

    public Action GetResetEncodersActionSequence() {
        //TODO: add proper code here
        return new SleepAction(0);
    }

    public Action GetVerticalActionsForTransfer() {
        Action verticalActions = new SequentialAction(
                new ParallelAction(
                        new VerticalClawAction(robotHardware, false, false, false),
                        new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_TRANSFER, false, false),
                        new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_TRANSFER, false, false),
                        new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_TRANSFER, false, false),
                        new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_TRANSFER, false, false)
                ),
                new VerticalClawAction(robotHardware, true, false, false)
        );

        return verticalActions;
    }

    public Action GetEnterExitSubActionSequence() {

        Action verticalActions = GetVerticalActionsForTransfer();

        Action horizontalActions = new ParallelAction(
                new HorizontalClawAction(robotHardware, true, false, false),
                new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_ENTER_EXIT_SUB, false, false),
                new HorizontalTurretAction(robotHardware, RobotConstants.HORIZONTAL_TURRET_ENTER_EXIT_SUB, false, false),
                new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_ENTER_EXIT_SUB, true, false),
                new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_ENTER_EXIT_SUB, true, false),
                new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_ENTER_EXIT_SUB, false,false)
        );

        return new ParallelAction(
                verticalActions,
                horizontalActions
        );
    }

    public Action GetPickSampleActionSequence() {


        //THE CONSTANTS BELOW NEED TO COME FROM THE CAMERA
        GetSampleChoicesFromCameraInputs();

        //TODO: THIS SHOULD START A RED LIGHT OR SOMETHING
        if (sampleChoices.isEmpty()) return new SleepAction(0.05);

        // TODO: NEED TO MAKE SURE WE HAVE A GOOD WAY TO FIND OUT IF WE HAVE A SAMPLE
        // IF WE DONT, THEN WE NEED TO GO TO OTHER CHOICES.
        HorizontalPickupVector choice = sampleChoices.get(0);

        Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetPickSampleActionSequence: SampleChoice: Slide: " + choice.slidePosition);
        Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetPickSampleActionSequence: SampleChoice: Turret: " + choice.turretPosition);
        Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetPickSampleActionSequence: SampleChoice: Claw: " + choice.clawOrientation);

        Action verticalActions = GetVerticalActionsForTransfer();

        Action horizontalActions = new SequentialAction(
                new HorizontalSlideAction(robotHardware, choice.slidePosition, true, false),
                new ParallelAction(
                        new HorizontalClawAction(robotHardware, true, false, false),
                        new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_PICK_SAMPLE, false, false),
                        new HorizontalTurretAction(robotHardware, choice.turretPosition, false, false),
                        new HorizontalSlideAction(robotHardware, choice.slidePosition, true, false),
                        new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_PICK_SAMPLE, true, false),
                        new HorizontalWristAction(robotHardware, choice.clawOrientation, false,false)
                ),
                new InstantAction(() -> readyToPickSample = true)
        );

        //uncomment below to go back to state before integrating camera input
//        Action horizontalActions = new SequentialAction(
//                new ParallelAction(
//                    new HorizontalClawAction(robotHardware, true, false, false),
//                    new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_PICK_SAMPLE, false, false),
//                    new HorizontalTurretAction(robotHardware, RobotConstants.HORIZONTAL_TURRET_PICK_SAMPLE, false, false),
//                    new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_PICK_SAMPLE, true, false),
//                    new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_PICK_SAMPLE, true, false),
//                    new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_PICK_SAMPLE, false,false)
//            ),
//                new InstantAction(() -> readyToPickSample = true)
//        );

        return new ParallelAction(
//                verticalActions,
                horizontalActions);
    }

    public Action GetTransferSampleActionSequence() {
        // VERTICAL STATE SHOULD HAVE ALREADY BEEN DONE DURING PICK SAMPLE
        Action verticalActions = GetVerticalActionsForTransfer();

        Action horizontalActions = new ParallelAction(
                new HorizontalClawAction(robotHardware, false, false, false),
                new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_TRANSFER, true, false),
                new HorizontalTurretAction(robotHardware, RobotConstants.HORIZONTAL_TURRET_TRANSFER, false, false),
                new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_TRANSFER, false, false),
                new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_TRANSFER, false, false),
                new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_TRANSFER, false, false)
        );

        return new SequentialAction(
                new ParallelAction(
                        horizontalActions,
                        verticalActions
                ),
                new VerticalClawAction(robotHardware, false, true, false),
                new HorizontalClawAction(robotHardware, true, true, true),
                new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_AFTER_TRANSFER, false, false),
                new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_AFTER_TRANSFER, false, false)
        );

    }

    private Action GetTransferToObZoneActionSequence() {

        //TODO: MOVE ROBOT TO OB ZONE ??

        Action horizontalActions = new ParallelAction(
                new HorizontalTurretAction(robotHardware, RobotConstants.HORIZONTAL_TURRET_TRANSFER, false, false),
                new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_AFTER_TRANSFER, false, false),
                new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_AFTER_TRANSFER, false, false),
                new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_TRANSFER, false, false),
                new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_TRANSFER, false, false)
        );

        Action verticalActions = new SequentialAction(
                new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_DROP_SAMPLE_OBZONE, true, false),
                new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_DROP_SAMPLE_OBZONE, true, false),
                new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_DROP_SAMPLE_OBZONE, true, false),
                new VerticalClawAction(robotHardware, true, false, false)
//                new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_DROP_SAMPLE_OBZONE, false, false)
        );

        return new ParallelAction(
                horizontalActions,
                verticalActions
        );
    }

    private Action GetLowBasketActionSequence() {

        // TODO: MOVE ROBOT TO BASKET COORDINATES

        Action horizontalActions = new ParallelAction(
                new HorizontalTurretAction(robotHardware, RobotConstants.HORIZONTAL_TURRET_TRANSFER, false, false),
                new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_AFTER_TRANSFER, false, false),
                new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_AFTER_TRANSFER, false, false),
                new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_TRANSFER, false, false),
                new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_TRANSFER, false, false)
        );

        Action verticalActions = new ParallelAction(
                new VerticalClawAction(robotHardware, false, false, false),
                new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_DROP_LOW_SAMPLE, true, false),
                new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_DROP_LOW_SAMPLE, false, false),
                new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_DROP_LOW_SAMPLE, false, false),
                new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_DROP_LOW_SAMPLE, true, false)
        );

        return new SequentialAction(
                new ParallelAction(
                        horizontalActions,
                        verticalActions
                ),
                new SleepAction(0.5),
                new VerticalClawAction(robotHardware, true, true, false)
        );
    }

    private Action GetHighBasketActionSequence() {

        // TODO: MOVE ROBOT TO BASKET COORDINATES

        // HORIZONTAL STATE SHOULD HAVE ALREADY BEEN DONE DURING TRANSFER
        Action horizontalActions = new ParallelAction(
                new HorizontalTurretAction(robotHardware, RobotConstants.HORIZONTAL_TURRET_TRANSFER, false, false),
                new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_AFTER_TRANSFER, false, false),
                new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_AFTER_TRANSFER, false, false),
                new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_TRANSFER, false, false),
                new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_TRANSFER, false, false)
        );

        Action verticalActions = new ParallelAction(
                new VerticalClawAction(robotHardware, false, false, false),
                new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_DROP_HIGH_SAMPLE, true, false),
                new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_DROP_HIGH_SAMPLE, false, false),
                new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_DROP_HIGH_SAMPLE, false, false),
                new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_DROP_HIGH_SAMPLE, true, false)
        );

        return new SequentialAction(
                new ParallelAction(
                        horizontalActions,
                        verticalActions
                ),
                new SleepAction(0.5),
                new VerticalClawAction(robotHardware, true, true, false)
        );
    }

    private Action GetPickSpecimenActionSequence() {
        //TODO: MOVE ROBOT TO PICK SPECIMEN

        // HORIZONTAL STATE SHOULD HAVE ALREADY BEEN DONE DURING TRANSFER
        Action horizontalActions = new ParallelAction(
                new HorizontalTurretAction(robotHardware, RobotConstants.HORIZONTAL_TURRET_PICK_SPECIMEN, false, false),
                new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_PICK_SPECIMEN, false, false),
                new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_PICK_SPECIMEN, false, false),
                new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_PICK_SPECIMEN, false, false),
                new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_PICK_SPECIMEN, false, false)
        );

        Action verticalActions = new SequentialAction(
                new ParallelAction(
                        new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_PICK_SPECIMEN, true, false),
                        new VerticalClawAction(robotHardware, false, false, false) //close the claw to make sure we pass thru the slides
                ),
                new ParallelAction(
                        new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_PICK_SPECIMEN, true, false),
                        new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_PICK_SPECIMEN, true, false),
                        new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_PICK_SPECIMEN, false, false)
                ),
                new VerticalClawAction(robotHardware, true, false, false)
        );

        return new ParallelAction(
                    horizontalActions,
                    verticalActions);
    }

    private Action GetSnapSpecimenActionSequence() {
        //TODO: MOVE ROBOT TO SNAP SPECIMEN

        // HORIZONTAL STATE SHOULD HAVE ALREADY BEEN DONE DURING PICK SPECIMEN
        Action horizontalActions = new ParallelAction(
                new HorizontalTurretAction(robotHardware, RobotConstants.HORIZONTAL_TURRET_PICK_SPECIMEN, false, false),
                new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_PICK_SPECIMEN, false, false),
                new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_PICK_SPECIMEN, false, false),
                new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_PICK_SPECIMEN, false, false),
                new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_PICK_SPECIMEN, false, false)
        );

        Action verticalActions = new SequentialAction(
                new ParallelAction(
                        new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_SNAP_HIGH_SPECIMEN, true, false),
                        new VerticalClawAction(robotHardware, false, false, false) //close the claw to make sure we pass thru the slides
                ),
                new ParallelAction(
                        new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_SNAP_HIGH_SPECIMEN, true, false),
                        new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_SNAP_HIGH_SPECIMEN, true, false),
                        new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_SNAP_HIGH_SPECIMEN, false, false)
                ),
                new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_RESTING, true, false)
        );

        return new ParallelAction(
                horizontalActions,
                verticalActions
        );
    }

    /// Function to move the slide and turret in response to DPAD presses
    private void ProcessDPad() {
        if (gamepad2.dpad_up) { //slide out
            int slidePos = robotHardware.getHorizontalSlidePosition();
            slidePos = Math.min (slidePos + RobotConstants.HORIZONTAL_SLIDE_INCREMENT, RobotConstants.HORIZONTAL_SLIDE_MAX_POS);
            robotHardware.setHorizontalSlidePosition(slidePos);
        }

        if (gamepad2.dpad_down) {   //slide in
            int slidePos = robotHardware.getHorizontalSlidePosition();
            slidePos = Math.max (slidePos - RobotConstants.HORIZONTAL_SLIDE_INCREMENT, 0);
            robotHardware.setHorizontalSlidePosition(slidePos);
        }

        if (gamepad2.dpad_left) {   //turret left
            double turretPos = robotHardware.getHorizontalTurretServoPosition();
            turretPos = Math.min(turretPos + RobotConstants.HORIZONTAL_TURRET_INCREMENT, RobotConstants.HORIZONTAL_TURRET_MAX_POS);
            robotHardware.setHorizontalTurretServoPosition(turretPos);
        }

        if (gamepad2.dpad_right) {  //turret right
            double turretPos = robotHardware.getHorizontalTurretServoPosition();
            turretPos = Math.max(turretPos - RobotConstants.HORIZONTAL_TURRET_INCREMENT, RobotConstants.HORIZONTAL_TURRET_MIN_POS);
            robotHardware.setHorizontalTurretServoPosition(turretPos);
        }
    }

    /// Function to open / close the horizontal claw in response to bumper buttons
    private void ProcessBumpers(){
        if (gamepad2.right_bumper) {
            robotHardware.setHorizontalClawState(true);
        }

        if (gamepad2.left_bumper) {
            robotHardware.setHorizontalClawState(false);
        }
    }


    ///Function to move the horizontal wrist in response to the left joystick movement
    private void ProcessJoystick() {
        if (gamepad2.left_stick_x < 0) {
            double wristPos = robotHardware.getHorizontalWristServoPosition();
            wristPos = Math.max(wristPos - RobotConstants.HORIZONTAL_WRIST_INCREMENT, 0);
            robotHardware.setHorizontalWristServoPosition(wristPos);
        } else if (gamepad2.left_stick_x > 0) {
            double wristPos = robotHardware.getHorizontalWristServoPosition();
            wristPos = Math.min(wristPos + RobotConstants.HORIZONTAL_WRIST_INCREMENT, 1);
            robotHardware.setHorizontalWristServoPosition(wristPos);
        }

    }
}