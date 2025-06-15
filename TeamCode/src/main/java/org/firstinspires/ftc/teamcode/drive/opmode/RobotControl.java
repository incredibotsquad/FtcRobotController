package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ColorSenorOutput;
import org.firstinspires.ftc.teamcode.GameConstants;
import org.firstinspires.ftc.teamcode.HorizontalPickupVector;
import org.firstinspires.ftc.teamcode.LimelightHelper;
import org.firstinspires.ftc.teamcode.LimelightLocation;
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
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.List;

public class RobotControl
{
    GameConstants.GAME_COLORS gameColor;

    private Gamepad gamepad2;
    private RobotHardware robotHardware;
    private FtcDashboard dashboard;
    private List<Action> runningActions;

    public TensorFlow VisionCalibration;
    VisionPortal visionPortal;
//    SampleDetectionPipelineV2 sampleDetectionPipeline;

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
        HANG_SPECIMEN,
        SNAP_SPECIMEN,
        ROBOT_HANG,
        CAMERA_READY
    }

    private ROBOT_STATE currentRobotState;
    private ROBOT_STATE targetRobotState;
    private boolean stateTransitionInProgress;
    private boolean cameraSettingsApplied = false;

    private Telemetry telemetry;

    private LimelightHelper limelightHelper;

    public RobotControl(Gamepad gamepad, RobotHardware robotHardware, Telemetry telemetry) {
        gamepad2 = gamepad;
        this.telemetry = telemetry;
        this.robotHardware = robotHardware;
        currentRobotState = ROBOT_STATE.NONE;
        targetRobotState = ROBOT_STATE.NONE;
        stateTransitionInProgress = false;
        dashboard = FtcDashboard.getInstance();
        runningActions = new ArrayList<>();
        sampleChoices = new ArrayList<>();
        limelightHelper = new LimelightHelper(robotHardware, telemetry);


//        VisionCalibration = new TensorFlow();
//
//        // Create the sample detection pipeline
//        sampleDetectionPipeline = new SampleDetectionPipelineV2();
//
//        // Create a VisionPortal with the pipeline
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        // Configure the builder
//        builder.setCamera(robotHardware.hardwareMap.get(WebcamName.class, "Webcam 1"))
//            .setCameraResolution(new Size(640, 480))
//            .addProcessor(sampleDetectionPipeline)
//            .enableLiveView(true)
//            .setAutoStopLiveView(true);
//
//        // Build the VisionPortal
//        visionPortal = builder.build();
//
//        // Set the VisionPortal in the pipeline
//        sampleDetectionPipeline.setVisionPortal(visionPortal);
//
//        // Update camera settings
//        sampleDetectionPipeline.updateCameraSettings(visionPortal);
    }

    public void ProcessInputs(Telemetry telemetry) {
        // Check if camera settings need to be applied
//        if (!cameraSettingsApplied) {
//            tryApplyCameraSettings();
//        }

        CreateStateFromButtonPress();

        TranslateStateIntoActions();

        ProcessActions();

//        ProcessSafetyChecks();

        ProcessDPad();
        ProcessBumpers();
        ProcessJoystickForHorizontalWrist();

    }
    
    /**
     * Try to apply camera settings if the VisionPortal is streaming
     * This is called periodically from ProcessInputs until settings are successfully applied
     */
//    private void tryApplyCameraSettings() {
//        if (visionPortal != null && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
//            // Apply camera settings - don't force update if already applied in the pipeline
//            if (sampleDetectionPipeline.updateCameraSettings(visionPortal)) {
//                cameraSettingsApplied = true;
//                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "Camera settings successfully applied");
//            }
//        } else {
//            Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "Waiting for camera to start streaming before applying settings...");
//        }
//    }

    private void ProcessSafetyChecks() {
        robotHardware.horizontalSlideSafetyChecks();
        robotHardware.verticalSlideSafetyChecks();
    }

    public void setGameColor(GameConstants.GAME_COLORS gameColor) {
//        sampleDetectionPipeline.setColorMode(gameColor);
        this.gameColor = gameColor;
    }
    
    /**
     * Stop and close the VisionPortal when no longer needed
     */
    public void stopVision() {
        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }
    }
    
    /**
     * Pause or resume vision processing
     * 
     * @param pause True to pause, false to resume
     */
    public void pauseVision(boolean pause) {
        if (visionPortal != null) {
            if (pause) {
                visionPortal.stopStreaming();
            } else {
                visionPortal.resumeStreaming();
            }
        }
    }

    //Function to create a state from Gamepad inputs
    private void CreateStateFromButtonPress() {

        ROBOT_STATE newTargetRobotState =  ROBOT_STATE.NONE;

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

        if (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
            newTargetRobotState = ROBOT_STATE.TRANSFER_TO_OB_ZONE;
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
                newTargetRobotState = ROBOT_STATE.CAMERA_READY;
            }
            else if (gamepad2.right_trigger > 0) {
                newTargetRobotState = ROBOT_STATE.ENTER_EXIT_SUB;
            }
        }

        if (gamepad2.y) {
            if (gamepad2.left_trigger > 0) {
                newTargetRobotState = ROBOT_STATE.HANG_SPECIMEN;
            }
            else if (gamepad2.right_trigger > 0) {
                newTargetRobotState = ROBOT_STATE.HIGH_BASKET;
            }
        }

        if (newTargetRobotState != ROBOT_STATE.NONE) {

            Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "TRANSITIONING STATE CURRENT: " + currentRobotState + " TARGET: " + newTargetRobotState);

            if (targetRobotState != ROBOT_STATE.NONE && newTargetRobotState != targetRobotState) {
                //STOP ALL PROCESSING - STATE TRANSITION WAS GOING ON WHEN NEW STATE WAS CALLED IN
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "STOPPING! STATE TRANSITION WAS GOING ON WHEN NEW TARGET WAS CALLED IN. OLD TARGET: " + targetRobotState + " NEW TARGET: " + newTargetRobotState);

                robotHardware.stopRobotAndMechanisms();
                runningActions.clear();
                //we partially achieved the transition - while actions were not completed, it is important to
                //update the current state since behavior depends on current state.
                currentRobotState = targetRobotState;
                stateTransitionInProgress = false;
            }

            targetRobotState = newTargetRobotState;
        }
    }

    private void GetSampleChoicesFromCameraInputs() {

        sampleChoices.clear();

        try {
            // Get detections from Limelight using appropriate pipelines
            List<LimelightLocation> allLocations = limelightHelper.getDetectionsForMode(this.gameColor);

            Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetSampleChoicesFromCameraInputs: allLocations count: " + allLocations.size());


            for (int i =0; i < allLocations.size(); i++) {
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "==============================================");
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetSampleChoicesFromCameraInputs: Location : " + (i + 1));
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetSampleChoicesFromCameraInputs: X: " + allLocations.get(i).translation);
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetSampleChoicesFromCameraInputs: Y: " + allLocations.get(i).extension);
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetSampleChoicesFromCameraInputs: Angle: " + allLocations.get(i).orientationAngle);
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetSampleChoicesFromCameraInputs: Color: " + allLocations.get(i).color);
            }

            if (!allLocations.isEmpty()) {
                LimelightLocation best = limelightHelper.getBest(allLocations, this.gameColor);


                double accountForPickupArm = Math.sqrt((RobotConstants.PICKUP_ARM_LENGTH * RobotConstants.PICKUP_ARM_LENGTH) - (best.translation * best.translation));
                int horizontalSlidePosition = (int) ((best.extension - accountForPickupArm) * RobotConstants.HORIZONTAL_SLIDE_TICKS_PER_INCH);

//                if (horizontalSlidePosition < 0 || horizontalSlidePosition > RobotConstants.HORIZONTAL_SLIDE_MAX_POS) continue; // no need to add an option which we cannot reach
//                if (Math.abs(best.translation) >= RobotConstants.PICKUP_ARM_LENGTH) continue;    // cannot reach beyond pickup arm length

                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetSampleChoicesFromCameraInputs. horizontalSlidePosition: " + horizontalSlidePosition);

                double turretMovementAngle = Math.toDegrees(Math.atan(best.translation / accountForPickupArm));

                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetSampleChoicesFromCameraInputs. turretMovementAngle: " + turretMovementAngle);

                double turretServoPos = RobotConstants.TURRET_CENTER_POSITION - (turretMovementAngle / 300);

                double clawParallelOrientation = RobotConstants.HORIZONTAL_WRIST_TRANSFER + (turretMovementAngle / 300);    //this will keep the claw parallel to the robot

                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetSampleChoicesFromCameraInputs. clawParallelOrientation: " + clawParallelOrientation);

                double wristTargetAngle = 180 - best.orientationAngle - 90;   //wrist has to be perpendicular to the sample
                double horizontalWristPosition = clawParallelOrientation + (wristTargetAngle / 300);

                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetSampleChoicesFromCameraInputs. horizontalWristPosition: " + horizontalWristPosition);

                sampleChoices.add(new HorizontalPickupVector(horizontalSlidePosition, turretServoPos, horizontalWristPosition));

                displayLimelightTelemetry(best, allLocations);
            } else {
                displayLimelightTelemetry(null, null);
            }

        } catch (Exception e) {
            telemetry.addData("Error", e.getMessage());
            telemetry.update();
        }
    }

//    private void GetSampleChoicesFromCameraInputs() {
//
//        sampleChoices.clear();  //remove any previous choices
//
//        if (sampleDetectionPipeline.latestRects != null && sampleDetectionPipeline.latestDistances != null) {
//
//            for (int loop = 0; loop < sampleDetectionPipeline.latestRects.length; loop++) {
//                double realX = sampleDetectionPipeline.GetRealXinches(loop);
//                double realY = sampleDetectionPipeline.GetRealYinches(loop);
//                double realOrientation = sampleDetectionPipeline.GetRealSampleOrientation(loop);
//
//                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Real Vertical: " + realY);
//                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Real Horizontal: " + realX);
//                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Real Orientation: " + realOrientation);
//
//                TensorFlow.CalibrationResult result = VisionCalibration.calibrate((float) (realX - RobotConstants.TURRET_OFFSET_FROM_CAMERA), (float) realY, (float) realOrientation);
//
////                double calibratedYOffset = result.calibratedY;
////                double calibratedXOffset = result.calibratedX;
////                double calibratedSampleOrientation = result.calibratedAngle;
//
//                double calibratedYOffset = realY;
//                double calibratedXOffset = realX;
//                double calibratedSampleOrientation = realOrientation;
//
//                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Calibrated Vertical: " + calibratedYOffset);
//                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Calibrated Horizontal: " + calibratedXOffset);
//                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Calibrated Orientation: " + calibratedSampleOrientation);
//
//                double accountForPickupArm = Math.sqrt((RobotConstants.PICKUP_ARM_LENGTH * RobotConstants.PICKUP_ARM_LENGTH) - (calibratedXOffset * calibratedXOffset));
//                int horizontalSlidePosition = (int) ((calibratedYOffset - accountForPickupArm) * RobotConstants.HORIZONTAL_SLIDE_TICKS_PER_INCH);
//
//                if (horizontalSlidePosition < 0 || horizontalSlidePosition > RobotConstants.HORIZONTAL_SLIDE_MAX_POS) continue; // no need to add an option which we cannot reach
//                if (Math.abs(calibratedXOffset) >= RobotConstants.PICKUP_ARM_LENGTH) continue;    // cannot reach beyond pickup arm length
//
//                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. horizontalSlidePosition: " + horizontalSlidePosition);
//
//                double turretMovementAngle = Math.toDegrees(Math.atan(calibratedXOffset / accountForPickupArm));
//
//                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. turretMovementAngle: " + turretMovementAngle);
//
//                double turretServoPos = RobotConstants.TURRET_CENTER_POSITION - (turretMovementAngle / 300);
//
//                double clawParallelOrientation = RobotConstants.HORIZONTAL_WRIST_TRANSFER + (turretMovementAngle / 300);    //this will keep the claw parallel to the robot
//
//                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. clawParallelOrientation: " + clawParallelOrientation);
//
//                double wristTargetAngle = 180 - calibratedSampleOrientation - 90;   //wrist has to be perpendicular to the sample
//                double horizontalWristPosition = clawParallelOrientation + (wristTargetAngle / 300);
//
//                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. horizontalWristPosition: " + horizontalWristPosition);
//
//                sampleChoices.add(new HorizontalPickupVector(horizontalSlidePosition, turretServoPos, horizontalWristPosition));
//            }
//
//            //sort by vertical distance
//            sampleChoices.sort(Comparator.comparingInt(choice -> choice.slidePosition));
//        }
//
//        Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "==================================================================");
//        Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. Choice Count: " + sampleChoices.size());
//        Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "==================================================================");
//
//        for (HorizontalPickupVector choice: sampleChoices) {
//            Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. horizontalSlidePosition: " + choice.slidePosition);
//            Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. turretMovementAngle: " + choice.turretPosition);
//            Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetHorizontalPickupVectorFromCameraInputs. horizontalWristPosition: " + choice.wristOrientation);
//            Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "==================================================================");
//        }
//    }


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

                case CAMERA_READY:
                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: CAMERA READY SUB");

                    runningActions.add(GetCameraReadyActionSequence());
                    break;
                case ENTER_EXIT_SUB:
                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: ENTER EXIT SUB");

                    if (currentRobotState == ROBOT_STATE.PICK_SAMPLE) {
                        runningActions.add(GetEnterExitSubActionSequence(false));
                    } else {
                        runningActions.add(GetEnterExitSubActionSequence(true));
                    }

                    break;

                case PICK_SAMPLE:   //PICK SAMPLE
                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: PICK SAMPLE");

                    if (currentRobotState == ROBOT_STATE.CAMERA_READY){
                        //get slide / turret / wrist from camera
                        GetSampleChoicesFromCameraInputs();//use current slide / turret / wrist position
                    }
                    else {
                        //NOTE: if we are going through enter/exit sub, operators are manually adjusting positions
                        //in such a case, the only choice should be the current position
                        sampleChoices.clear();
                        sampleChoices.add(new HorizontalPickupVector(robotHardware.getHorizontalSlidePosition(), robotHardware.getHorizontalTurretServoPosition(), robotHardware.getHorizontalWristServoPosition()));
                    }

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

                case HANG_SPECIMEN:
                    Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: HANG SPECIMEN");

                    runningActions.add(GetHangSpecimenActionSequence());

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

        // we have actions to run, state transition in progress
        if (!runningActions.isEmpty()) {
            stateTransitionInProgress = true;

            // run actions and add pending ones to new list
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());

                if (action.run(packet)) {
                    newActions.add(action); //add if action indicates it needs to run again
                }
            }
            dashboard.sendTelemetryPacket(packet);
            runningActions = newActions;

            //this has to be inside the outer if, else the current robot state
            //will get wiped out - the next big loop will check that runningactions
            //is empty and will set the current state to NONE since we would have set
            //the target state to NONE in the previous big loop when we finished
            //running actions.
            if (runningActions.isEmpty()) {
                currentRobotState = targetRobotState;
                targetRobotState = ROBOT_STATE.NONE;
                stateTransitionInProgress = false;

                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "ProcessActions: DONE RUNNING ACTIONS");
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "ProcessActions: CURRENT ROBOT STATE: " + currentRobotState);
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "ProcessActions: TARGET ROBOT STATE: " + targetRobotState);

                //NOTE: CAREFUL NOT TO CONSTRUCT LOOPS HERE
                // STATE A -> STATE B -> STATE A
                //set new state based on older target

                //TODO: IF WE GO FROM ENTER SUB TO PICK SAMPLE, GO BACK TO ENTER SUB
                //TODO: IF WE GO FROM RESTING TO PICK SAMPLE

//            switch (currentRobotState) {
//                case LOW_BASKET:
//                case HIGH_BASKET:
////                    targetRobotState = ROBOT_STATE.PICK_SAMPLE;
//                    break;
//                case SNAP_SPECIMEN:
////                    targetRobotState = ROBOT_STATE.PICK_SPECIMEN;
//                    break;
//                default:
//                    targetRobotState = ROBOT_STATE.NONE;
//            }
            }
        }
    }

    public void ProcessColorForPickedSample() {

        Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "ProcessColorForPickedSample");

        ColorSenorOutput colorSenorOutput = robotHardware.getDetectedColorAndDistance();

        if (colorSenorOutput.detectedColor != this.gameColor && colorSenorOutput.detectedColor != GameConstants.GAME_COLORS.YELLOW) {
            if (colorSenorOutput.distance < RobotConstants.COLOR_SENSOR_DISTANCE_THRESHOLD) {
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "ProcessColorForPickedSample - EJECTING BAD SAMPLE");

                robotHardware.setHorizontalShoulderServoPosition(RobotConstants.HORIZONTAL_SHOULDER_ENTER_EXIT_SUB);
                robotHardware.setHorizontalClawState(true);
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

        Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetVerticalActionsForTransfer: Vertical Shoulder Position: " + robotHardware.getVerticalShoulderServoPosition());

        Action verticalActions = new SequentialAction(
                //only close the claw if the shoulder is not at its target position
                ((robotHardware.getVerticalShoulderServoPosition() - RobotConstants.VERTICAL_SHOULDER_TRANSFER) > 0.01)?
                        new VerticalClawAction(robotHardware, false, true, false) : new NullAction(),
                new ParallelAction(
                        new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_TRANSFER, true, false),
                        new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_TRANSFER, false, false),
                        new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_TRANSFER, false, false)
                ),
                //need to wait for a bit to let the wrist move to the other side when changing from observation zone
                (currentRobotState == ROBOT_STATE.TRANSFER_TO_OB_ZONE)? new SleepAction(0.5) : new NullAction(),
                new ParallelAction(
                        new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_TRANSFER, false, false),
                        new VerticalClawAction(robotHardware, true, false, false)
                )
        );

        return verticalActions;
    }

    public Action GetCameraReadyActionSequence(){
        Action verticalActions = GetVerticalActionsForTransfer();

        Action horizontalActions = new ParallelAction(
                new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_CAMERA_READY, false, false),
                new HorizontalTurretAction(robotHardware, RobotConstants.HORIZONTAL_TURRET_CAMERA_READY, false, false),
                new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_CAMERA_READY, true, false),
                new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_CAMERA_READY, false, false),
                new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_CAMERA_READY, false,false),
                new InstantAction(() -> robotHardware.setColorSensorLEDState(true))
        );

        return new SequentialAction(
                 verticalActions,
                horizontalActions
        );
    }

    public Action GetEnterExitSubActionSequence(boolean openClaw) {

        Action verticalActions = GetVerticalActionsForTransfer();

        Action horizontalActions = new ParallelAction(
                new HorizontalClawAction(robotHardware, openClaw, false, false),
                new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_ENTER_EXIT_SUB, false, false),
                new HorizontalTurretAction(robotHardware, RobotConstants.HORIZONTAL_TURRET_ENTER_EXIT_SUB, false, false),
                new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_ENTER_EXIT_SUB, true, false),
                new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_ENTER_EXIT_SUB, true, false),
                new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_ENTER_EXIT_SUB, false,false),
                new InstantAction(() -> robotHardware.setColorSensorLEDState(true))
        );

        return new ParallelAction(
                verticalActions,
                horizontalActions
        );
    }

    public Action GetPickSampleActionSequence() {

        //TODO: THIS SHOULD START A RED LIGHT OR SOMETHING
        if (sampleChoices.isEmpty()) return new SleepAction(0.05);

        // TODO: NEED TO MAKE SURE WE HAVE A GOOD WAY TO FIND OUT IF WE HAVE A SAMPLE
        // IF WE DON'T, THEN WE NEED TO GO TO OTHER CHOICES.
        HorizontalPickupVector choice = sampleChoices.get(0);

        Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetPickSampleActionSequence: SampleChoice: Slide: " + choice.slidePosition);
        Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetPickSampleActionSequence: SampleChoice: Turret: " + choice.turretPosition);
        Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "GetPickSampleActionSequence: SampleChoice: Claw: " + choice.wristOrientation);

        Action verticalActions = GetVerticalActionsForTransfer();

        Action horizontalActions = new SequentialAction(
                new HorizontalSlideAction(robotHardware, choice.slidePosition, true, true),
                new ParallelAction(
                        new HorizontalClawAction(robotHardware, true, false, false),
                        new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_PICK_SAMPLE, false, false),
                        new HorizontalTurretAction(robotHardware, choice.turretPosition, false, false),
                        new HorizontalSlideAction(robotHardware, choice.slidePosition, false, true),
                        new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_PICK_SAMPLE, true, true),
                        new HorizontalWristAction(robotHardware, choice.wristOrientation, false,false),
                        new InstantAction(() -> robotHardware.setColorSensorLEDState(true))
                ),
                new HorizontalClawAction(robotHardware, false, true, false),
                new InstantAction(this::ProcessColorForPickedSample)
        );

        return new ParallelAction(
                verticalActions,
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
                new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_TRANSFER, false, false),
                new InstantAction(() -> robotHardware.setColorSensorLEDState(true))
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

    public Action GetTransferToObZoneActionSequence() {

        //TODO: MOVE ROBOT TO OB ZONE ??

        Action horizontalActions = new ParallelAction(
                new HorizontalTurretAction(robotHardware, RobotConstants.HORIZONTAL_TURRET_TRANSFER, false, false),
                new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_AFTER_TRANSFER, false, false),
                new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_AFTER_TRANSFER, false, false),
                new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_TRANSFER, false, false),
                new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_TRANSFER, false, false),
                new InstantAction(() -> robotHardware.setColorSensorLEDState(true))
        );

        Action verticalActions = new SequentialAction(
                new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_DROP_SAMPLE_OBZONE, true, false),
                new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_DROP_SAMPLE_OBZONE, true, false),
                new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_DROP_SAMPLE_OBZONE, true, false),
                new VerticalClawAction(robotHardware, true, false, false)
//                new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_DROP_SAMPLE_OBZONE, false, false)
        );

        return new SequentialAction(
                GetTransferSampleActionSequence(),
                new ParallelAction(
                    horizontalActions,
                    verticalActions)
        );
    }

    public Action GetLowBasketActionSequence() {

        // TODO: MOVE ROBOT TO BASKET COORDINATES

        Action horizontalActions = new ParallelAction(
                new HorizontalTurretAction(robotHardware, RobotConstants.HORIZONTAL_TURRET_TRANSFER, false, false),
                new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_AFTER_TRANSFER, false, false),
                new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_AFTER_TRANSFER, false, false),
                new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_TRANSFER, false, false),
                new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_TRANSFER, false, false),
                new InstantAction(() -> robotHardware.setColorSensorLEDState(true))
        );

        Action verticalActions = new ParallelAction(
                new VerticalClawAction(robotHardware, false, false, false),
                new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_DROP_LOW_SAMPLE, true, false),
                new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_DROP_LOW_SAMPLE, false, false),
                new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_DROP_LOW_SAMPLE, false, false),
                new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_DROP_LOW_SAMPLE, true, false)
        );

        return new SequentialAction(
                GetTransferSampleActionSequence(),
                new ParallelAction(
                        horizontalActions,
                        verticalActions
                ),
                new SleepAction(0.5),
                new VerticalClawAction(robotHardware, true, true, false)
        );
    }

    public Action GetHighBasketActionSequence() {

        // TODO: MOVE ROBOT TO BASKET COORDINATES

        // HORIZONTAL STATE SHOULD HAVE ALREADY BEEN DONE DURING TRANSFER
        Action horizontalActions = new ParallelAction(
                new HorizontalTurretAction(robotHardware, RobotConstants.HORIZONTAL_TURRET_TRANSFER, false, false),
                new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_AFTER_TRANSFER, false, false),
                new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_AFTER_TRANSFER, false, false),
                new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_TRANSFER, false, false),
                new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_TRANSFER, false, false),
                new InstantAction(() -> robotHardware.setColorSensorLEDState(true))
        );

        Action verticalActions = new ParallelAction(
                new VerticalClawAction(robotHardware, false, false, false),
                new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_DROP_HIGH_SAMPLE, true, false),
                new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_DROP_HIGH_SAMPLE, false, false),
                new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_DROP_HIGH_SAMPLE, false, false),
                new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_DROP_HIGH_SAMPLE, true, false)
        );

        return new SequentialAction(
                GetTransferSampleActionSequence(),
                new ParallelAction(
                        horizontalActions,
                        verticalActions
                ),
                new SleepAction(0.5),
                new VerticalClawAction(robotHardware, true, true, false)
        );
    }

    private Action GetHorizontalActionsForSpecimen() {
        Action horizontalActions = new SequentialAction(
                new ParallelAction(
                        new HorizontalTurretAction(robotHardware, RobotConstants.HORIZONTAL_TURRET_PICK_SPECIMEN, false, false),
                        new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_PICK_SPECIMEN, false, false),
                        new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_PICK_SPECIMEN, false, false),
                        new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_PICK_SPECIMEN, false, false),
                        new InstantAction(()-> robotHardware.setColorSensorLEDState(false))
                ),
                new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_PICK_SPECIMEN, false, false)
        );

        return horizontalActions;
    }

    public Action GetPickSpecimenActionSequence() {
        //TODO: MOVE ROBOT TO PICK SPECIMEN
        Action horizontalActions = GetHorizontalActionsForSpecimen();

        Action verticalActions = new SequentialAction(
                new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_PICK_SPECIMEN, true, false),
                new ParallelAction(
                        new VerticalClawAction(robotHardware, false, false, false), //close the claw to make sure we pass thru the slides
                        new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_PICK_SPECIMEN, false, false),
                        new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_PICK_SPECIMEN, false, false)
                ),
                new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_PICK_SPECIMEN, true, false),
                new VerticalClawAction(robotHardware, true, false, false),
                new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_RESTING, false, false)
        );

        return new SequentialAction(
                    horizontalActions,
                    verticalActions);
    }

    public Action GetHangSpecimenActionSequence() {
        //TODO: MOVE ROBOT TO SNAP SPECIMEN

        // HORIZONTAL STATE SHOULD HAVE ALREADY BEEN DONE DURING PICK SPECIMEN
        Action horizontalActions = GetHorizontalActionsForSpecimen();

        Action verticalActions = new SequentialAction(
                new VerticalClawAction(robotHardware, false, true, false), //close the claw to make sure we pass thru the slides
                new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_HANG_SPECIMEN, true, true),
                new ParallelAction(
                        new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_HANG_SPECIMEN, false, false),
                        new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_HANG_SPECIMEN, false, false),
                        new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_HANG_SPECIMEN, false, false)
                )
        );

        return new ParallelAction(
                horizontalActions,
                verticalActions
        );
    }

    public Action GetSnapSpecimenActionSequence() {
        //TODO: MOVE ROBOT TO SNAP SPECIMEN

        return new SequentialAction(
                GetHangSpecimenActionSequence(),
                new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_SNAP_SPECIMEN, true, false),
                new VerticalClawAction(robotHardware, true, false, false)
        );
    }

    /// Function to move the slide and turret in response to DPAD presses
    private void ProcessDPad() {
        if (gamepad2.dpad_up) { //slide out

            if (currentRobotState == ROBOT_STATE.PICK_SPECIMEN ||
            currentRobotState == ROBOT_STATE.HANG_SPECIMEN ||
            currentRobotState == ROBOT_STATE.SNAP_SPECIMEN) {

                int slidePos = robotHardware.getVerticalSlidePosition();
                slidePos = Math.min(slidePos + RobotConstants.VERTICAL_SLIDE_INCREMENT, RobotConstants.VERTICAL_SLIDE_MAX_POS);
                robotHardware.setVerticalSlidePosition(slidePos);

            } else {
                int slidePos = robotHardware.getHorizontalSlidePosition();
                slidePos = Math.min(slidePos + RobotConstants.HORIZONTAL_SLIDE_INCREMENT, RobotConstants.HORIZONTAL_SLIDE_MAX_POS);
                robotHardware.setHorizontalSlidePosition(slidePos);
            }
        }

        if (gamepad2.dpad_down) {   //slide in
            if (currentRobotState == ROBOT_STATE.PICK_SPECIMEN ||
                    currentRobotState == ROBOT_STATE.HANG_SPECIMEN ||
                    currentRobotState == ROBOT_STATE.SNAP_SPECIMEN) {

                int slidePos = robotHardware.getVerticalSlidePosition();
                slidePos = Math.max(slidePos - RobotConstants.VERTICAL_SLIDE_INCREMENT, 0);
                robotHardware.setVerticalSlidePosition(slidePos);

            } else {
                int slidePos = robotHardware.getHorizontalSlidePosition();
                slidePos = Math.max(slidePos - RobotConstants.HORIZONTAL_SLIDE_INCREMENT, 0);
                robotHardware.setHorizontalSlidePosition(slidePos);
            }
        }

        if (gamepad2.dpad_left) {   //turret left
            //nothing to do when working with specimen
            if (currentRobotState == ROBOT_STATE.PICK_SPECIMEN ||
                    currentRobotState == ROBOT_STATE.HANG_SPECIMEN ||
                    currentRobotState == ROBOT_STATE.SNAP_SPECIMEN) { return; }

            double turretPos = robotHardware.getHorizontalTurretServoPosition();
            turretPos = Math.min(turretPos + RobotConstants.HORIZONTAL_TURRET_INCREMENT, RobotConstants.HORIZONTAL_TURRET_MAX_POS);
            robotHardware.setHorizontalTurretServoPosition(turretPos);
        }

        if (gamepad2.dpad_right) {  //turret right
            //nothing to do when working with specimen
            if (currentRobotState == ROBOT_STATE.PICK_SPECIMEN ||
                    currentRobotState == ROBOT_STATE.HANG_SPECIMEN ||
                    currentRobotState == ROBOT_STATE.SNAP_SPECIMEN) { return; }

            double turretPos = robotHardware.getHorizontalTurretServoPosition();
            turretPos = Math.max(turretPos - RobotConstants.HORIZONTAL_TURRET_INCREMENT, RobotConstants.HORIZONTAL_TURRET_MIN_POS);
            robotHardware.setHorizontalTurretServoPosition(turretPos);
        }
    }

    /// Function to open / close the horizontal claw in response to bumper buttons
    private void ProcessBumpers(){
        if (gamepad2.right_bumper) {
            if (currentRobotState == ROBOT_STATE.PICK_SPECIMEN) {
                robotHardware.setVerticalClawState(true);
            }
            else {
                robotHardware.setHorizontalClawState(true);
            }
        }

        if (gamepad2.left_bumper) {
            if (currentRobotState == ROBOT_STATE.PICK_SPECIMEN) {
                robotHardware.setVerticalClawState(false);
            }
            else {
                robotHardware.setHorizontalClawState(false);
            }
        }
    }

    ///Function to move the horizontal wrist in response to the left joystick movement
    private void ProcessJoystickForHorizontalWrist() {
        if (gamepad2.right_stick_x < 0) {
            double wristPos = robotHardware.getHorizontalWristServoPosition();
            wristPos = Math.max(wristPos - RobotConstants.HORIZONTAL_WRIST_INCREMENT, 0);
            robotHardware.setHorizontalWristServoPosition(wristPos);
        }

        if (gamepad2.right_stick_x > 0) {
            double wristPos = robotHardware.getHorizontalWristServoPosition();
            wristPos = Math.min(wristPos + RobotConstants.HORIZONTAL_WRIST_INCREMENT, 1);
            robotHardware.setHorizontalWristServoPosition(wristPos);
        }

    }

    private void displayLimelightTelemetry(LimelightLocation best, List<LimelightLocation> locations) {
        int totalDetections = (locations != null)? locations.size(): 0;

        telemetry.addData("Status", "Running");
        telemetry.addData("Detection Mode", gameColor.toString());
        telemetry.addData("Total Detections", totalDetections);

        // Show controls
        telemetry.addLine("Controls:");
        telemetry.addLine("A=Red, B=Blue, Y=Yellow, X=Red+Yellow, RB=Blue+Yellow");

        // Debug: Show all detected colors
        if (locations != null && !locations.isEmpty()) {
            telemetry.addLine("=== ALL DETECTIONS ===");
            for (int i = 0; i < Math.min(locations.size(), 3); i++) {
                LimelightLocation loc = locations.get(i);
                telemetry.addData("Detection " + (i+1),
                        String.format("%s: X=%.1f Y=%.1f", loc.color, loc.translation, loc.extension));
            }
        }

        if (best != null && totalDetections > 0 &&
                !(best.translation == 0 && best.extension == 0 && best.rotation == 0)) {
            telemetry.addLine("=== BEST TARGET ===");
            telemetry.addData("X Position", String.format("%.2f", best.translation));
            telemetry.addData("Y Position", String.format("%.2f", best.extension));
            telemetry.addData("Orientation", String.format("%.1f°", best.orientationAngle));
            telemetry.addData("Color", best.color.toString());
            telemetry.addData("Raw X", String.format("%.2f", best.rawTranslation));
            telemetry.addData("Raw Y", String.format("%.2f", best.rawExtension));
            telemetry.addData("Rotation Score", String.format("%.2f", best.rotation));
        } else {
            telemetry.addLine("No valid targets found for " + gameColor);
        }

        telemetry.update();
    }
}
