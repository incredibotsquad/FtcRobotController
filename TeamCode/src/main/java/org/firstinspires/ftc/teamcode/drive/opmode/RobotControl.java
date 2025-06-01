package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ColorSenorOutput;
import org.firstinspires.ftc.teamcode.GameConstants;
import org.firstinspires.ftc.teamcode.R;
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

import java.util.ArrayList;
import java.util.List;

public class RobotControl
{
    GameConstants.GAME_COLORS gameColor;

    private Gamepad gamepad2;
    private RobotHardware robotHardware;
    private FtcDashboard dashboard;
    private List<Action> runningActions;


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

    private boolean readyToPickSample;

    public RobotControl(Gamepad gamepad, RobotHardware robotHardware) {
        gamepad2 = gamepad;
        this.robotHardware = robotHardware;
        currentRobotState = ROBOT_STATE.NONE;
        targetRobotState = ROBOT_STATE.NONE;
        stateTransitionInProgress = false;
        dashboard = FtcDashboard.getInstance();
        runningActions = new ArrayList<>();
        readyToPickSample = false;
    }

    public void ProcessInputs(Telemetry telemetry) {

        CreateStateFromButtonPress();

        ProcessStateNew();

//        ProcessState();

//        ProcessSafetyChecks();
//
//        ProcessDPad();
//
//        HandleManualOverride();

        ProcessPickSampleState();

        HandleMotorCurrentProblems();
    }

    private void ProcessSafetyChecks() {
        robotHardware.horizontalSlideSafetyChecks();
        robotHardware.verticalSlideSafetyChecks();
    }

    public void setGameColor(GameConstants.GAME_COLORS gameColor) {
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
        }

        if (gamepad2.y) {
            if (gamepad2.right_trigger > 0) {
                newTargetRobotState = ROBOT_STATE.HIGH_BASKET;
            }
        }

        if (newTargetRobotState != currentRobotState) {

            Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "TRANSITIONING STATE CURRENT: " + currentRobotState + " TARGET: " + newTargetRobotState);

            if (newTargetRobotState != targetRobotState) {
                //TODO: STOP ALL PROCESSING - STATE TRANSITION WAS GOING ON WHEN NEW STATE WAS CALLED IN

//                for (Action action : runningActions) {
//                    action.
//                }

                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "STOPPING! STATE TRANSITION WAS GOING ON WHEN NEW TARGET WAS CALLED IN. OLD TARGET: " + targetRobotState + " NEW TARGET: " + newTargetRobotState);
            }

            targetRobotState = newTargetRobotState;
        }
    }

    //function to take the new target state and turn it into robot movements
    private void ProcessStateNew() {

        if (currentRobotState == targetRobotState) return;

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
                    currentRobotState = targetRobotState;
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

            stateTransitionInProgress = true;
        }

        ProcessActions();

        if (runningActions.isEmpty()) {
            Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "DONE RUNNING ACTIONS");
            currentRobotState = targetRobotState;
            stateTransitionInProgress = false;

            //set new state based on older target
            switch (targetRobotState) {
                case PICK_SAMPLE:
//                    targetRobotState = ROBOT_STATE.TRANSFER_SAMPLE;
                    break;
                case LOW_BASKET:
                case HIGH_BASKET:
                    targetRobotState = ROBOT_STATE.PICK_SAMPLE;
                    break;
                case SNAP_SPECIMEN:
                    targetRobotState = ROBOT_STATE.PICK_SPECIMEN;
                    break;
            }
        }
    }

    private void ProcessActions()
    {
        TelemetryPacket packet = new TelemetryPacket();

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());

            Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "Executing Action: " + action.toString());

            if (action.run(packet)) {
                newActions.add(action); //add if action indicates it needs to run again
            }
        }
        runningActions = newActions;
        dashboard.sendTelemetryPacket(packet);
    }

    public void ProcessPickSampleState() {
        if (currentRobotState != ROBOT_STATE.PICK_SAMPLE || targetRobotState == ROBOT_STATE.TRANSFER_SAMPLE) return;

        Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "ProcessPickSampleState");

        ColorSenorOutput colorSenorOutput = robotHardware.getDetectedColorAndDistance();

        if (colorSenorOutput.detectedColor == this.gameColor || colorSenorOutput.detectedColor == GameConstants.GAME_COLORS.OTHER) {
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

    public Action GetPickSampleActionSequence() {

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

        //TODO: THE CONSTANTS BELOW NEED TO COME FROM THE CAMERA
        Action horizontalActions = new SequentialAction(
                new ParallelAction(
                    new HorizontalClawAction(robotHardware, true, false, false),
                    new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_PICK_SAMPLE, false, false),
                    new HorizontalTurretAction(robotHardware, RobotConstants.HORIZONTAL_TURRET_PICK_SAMPLE, false, false),
                    new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_PICK_SAMPLE, true, false),
                    new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_PICK_SAMPLE, true, false),
                    new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_PICK_SAMPLE, false,false)
            ),
                new InstantAction(() -> readyToPickSample = true)
        );

        return new ParallelAction(
                verticalActions,
                horizontalActions);
    }

    public Action GetTransferSampleActionSequence() {
        // VERTICAL STATE SHOULD HAVE ALREADY BEEN DONE DURING PICK SAMPLE
        Action verticalActions = new SequentialAction(
                new ParallelAction(
                    new VerticalClawAction(robotHardware, false, true, false),
                    new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_TRANSFER, false, false),
                    new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_TRANSFER, false, false),
                    new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_TRANSFER, false, false),
                    new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_TRANSFER, false, false)
                ),
                new VerticalClawAction(robotHardware, true, true, false)
        );

        Action horizontalActions = new ParallelAction(
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
                new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_TRANSFER, false, false),
                new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_TRANSFER, false, false),
                new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_TRANSFER, false, false),
                new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_TRANSFER, false, false)
        );

        Action verticalActions = new ParallelAction(
                new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_DROP_LOW_SAMPLE, false, false),
                new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_DROP_LOW_SAMPLE, false, false),
                new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_DROP_LOW_SAMPLE, false, false),
                new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_DROP_LOW_SAMPLE, false, false)
        );

        return new SequentialAction(
                new ParallelAction(
                        horizontalActions,
                        verticalActions
                ),
                new VerticalClawAction(robotHardware, true, true, false)
        );
    }

    private Action GetHighBasketActionSequence() {

        // TODO: MOVE ROBOT TO BASKET COORDINATES

        // HORIZONTAL STATE SHOULD HAVE ALREADY BEEN DONE DURING TRANSFER
        Action horizontalActions = new ParallelAction(
                new HorizontalTurretAction(robotHardware, RobotConstants.HORIZONTAL_TURRET_TRANSFER, false, false),
                new HorizontalShoulderAction(robotHardware, RobotConstants.HORIZONTAL_SHOULDER_TRANSFER, false, false),
                new HorizontalElbowAction(robotHardware, RobotConstants.HORIZONTAL_ELBOW_TRANSFER, false, false),
                new HorizontalWristAction(robotHardware, RobotConstants.HORIZONTAL_WRIST_TRANSFER, false, false),
                new HorizontalSlideAction(robotHardware, RobotConstants.HORIZONTAL_SLIDE_TRANSFER, false, false)
        );

        Action verticalActions = new ParallelAction(
                new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_DROP_HIGH_SAMPLE, false, false),
                new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_DROP_HIGH_SAMPLE, false, false),
                new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_DROP_HIGH_SAMPLE, false, false),
                new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_DROP_HIGH_SAMPLE, false, false)
        );

        return new SequentialAction(
                new ParallelAction(
                        horizontalActions,
                        verticalActions
                ),
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

        Action verticalActions = new ParallelAction(
                new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_PICK_SPECIMEN, false, false),
                new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_PICK_SPECIMEN, false, false),
                new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_PICK_SPECIMEN, false, false),
                new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_PICK_SPECIMEN, false, false)
        );

        return new ParallelAction(
                horizontalActions,
                verticalActions
        );
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

        Action verticalActions = new ParallelAction(
                new VerticalShoulderAction(robotHardware, RobotConstants.VERTICAL_SHOULDER_SNAP_HIGH_SPECIMEN, false, false),
                new VerticalSlideAction(robotHardware, RobotConstants.VERTICAL_SLIDE_SNAP_HIGH_SPECIMEN, false, false),
                new VerticalElbowAction(robotHardware, RobotConstants.VERTICAL_ELBOW_SNAP_HIGH_SPECIMEN, false, false),
                new VerticalWristAction(robotHardware, RobotConstants.VERTICAL_WRIST_SNAP_HIGH_SPECIMEN, false, false)
        );

        return new ParallelAction(
                horizontalActions,
                verticalActions
        );
    }

    private void ProcessState() {

        if (currentRobotState == targetRobotState) return;

        switch (targetRobotState) {

            case RESTING:
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: RESTING");

                robotHardware.setVerticalClawState(false);
                robotHardware.setVerticalShoulderServoPosition(RobotConstants.VERTICAL_SHOULDER_RESTING);
                robotHardware.setVerticalElbowServoPosition(RobotConstants.VERTICAL_ELBOW_RESTING);
                robotHardware.setVerticalWristServoPosition(RobotConstants.VERTICAL_WRIST_RESTING);
                robotHardware.setVerticalSlidePosition(RobotConstants.VERTICAL_SLIDE_RESTING);

                robotHardware.setHorizontalTurretServoPosition(RobotConstants.HORIZONTAL_TURRET_RESTING);
                robotHardware.setHorizontalClawState(false);
                robotHardware.setHorizontalShoulderServoPosition(RobotConstants.HORIZONTAL_SHOULDER_RESTING);
                robotHardware.setHorizontalElbowServoPosition(RobotConstants.HORIZONTAL_ELBOW_RESTING);
                robotHardware.setHorizontalWristServoPosition(RobotConstants.HORIZONTAL_WRIST_RESTING);
                robotHardware.setHorizontalSlidePosition(RobotConstants.HORIZONTAL_SLIDE_RESTING);

                // WHEN THE SLIDES ARE DONE MOVING, WE ARE DONE PROCESSING
                if ((Math.abs(robotHardware.getHorizontalSlidePosition() - RobotConstants.HORIZONTAL_SLIDE_RESTING) < RobotConstants.SLIDE_POSITION_TOLERANCE)  &&
                        Math.abs(robotHardware.getVerticalSlidePosition() - RobotConstants.VERTICAL_SLIDE_RESTING)  < RobotConstants.SLIDE_POSITION_TOLERANCE) {
                    currentRobotState = targetRobotState;
                }


                break;

            case RESET_ENCODERS:
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: RESET ENCODERS");

                currentRobotState = targetRobotState;
                break;

            case PICK_SAMPLE:   //PICK SAMPLE
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: PICK SAMPLE");

                //TODO: THESE NEED TO COME FROM THE CAMERA
                robotHardware.setHorizontalClawState(true);
                robotHardware.setHorizontalTurretServoPosition(RobotConstants.HORIZONTAL_TURRET_PICK_SAMPLE);
                robotHardware.setHorizontalShoulderServoPosition(RobotConstants.HORIZONTAL_SHOULDER_PICK_SAMPLE);
                robotHardware.setHorizontalElbowServoPosition(RobotConstants.HORIZONTAL_ELBOW_PICK_SAMPLE);
                robotHardware.setHorizontalWristServoPosition(RobotConstants.HORIZONTAL_WRIST_PICK_SAMPLE);
                robotHardware.setHorizontalSlidePosition(RobotConstants.HORIZONTAL_SLIDE_PICK_SAMPLE);

                // VERTICAL GET READY FOR TRANSFER
                robotHardware.setVerticalClawState(true);
                robotHardware.setVerticalShoulderServoPosition(RobotConstants.VERTICAL_SHOULDER_TRANSFER);
                robotHardware.setVerticalElbowServoPosition(RobotConstants.VERTICAL_ELBOW_TRANSFER);
                robotHardware.setVerticalWristServoPosition(RobotConstants.VERTICAL_WRIST_TRANSFER);
                robotHardware.setVerticalSlidePosition(RobotConstants.VERTICAL_SLIDE_TRANSFER);

                currentRobotState = targetRobotState;
                break;

            case TRANSFER_SAMPLE:
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: TRANSFER SAMPLE");

                robotHardware.setHorizontalTurretServoPosition(RobotConstants.HORIZONTAL_TURRET_TRANSFER);
                robotHardware.setHorizontalShoulderServoPosition(RobotConstants.HORIZONTAL_SHOULDER_TRANSFER);
                robotHardware.setHorizontalElbowServoPosition(RobotConstants.HORIZONTAL_ELBOW_TRANSFER);
                robotHardware.setHorizontalWristServoPosition(RobotConstants.HORIZONTAL_WRIST_TRANSFER);
                robotHardware.setHorizontalSlidePosition(RobotConstants.HORIZONTAL_SLIDE_TRANSFER);

                // VERTICAL STATE SHOULD HAVE ALREADY BEEN DONE DURING PICK SAMPLE
                robotHardware.setVerticalClawState(true);
                robotHardware.setVerticalShoulderServoPosition(RobotConstants.VERTICAL_SHOULDER_TRANSFER);
                robotHardware.setVerticalElbowServoPosition(RobotConstants.VERTICAL_ELBOW_TRANSFER);
                robotHardware.setVerticalWristServoPosition(RobotConstants.VERTICAL_WRIST_TRANSFER);
                robotHardware.setVerticalSlidePosition(RobotConstants.VERTICAL_SLIDE_TRANSFER);

                robotHardware.setVerticalClawState(false);  //close vertical claw
                try {
                    Thread.sleep(100);
                }
                catch (InterruptedException e) {
                }

                robotHardware.setHorizontalClawState(true); //open horizontal claw

                currentRobotState = targetRobotState;

                break;

            case TRANSFER_TO_OB_ZONE:
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: TRANSFER TO OBSERVATION ZONE");

                //TODO: MOVE ROBOT TO OB ZONE
                robotHardware.setHorizontalTurretServoPosition(RobotConstants.HORIZONTAL_TURRET_TRANSFER);
                robotHardware.setHorizontalShoulderServoPosition(RobotConstants.HORIZONTAL_SHOULDER_TRANSFER);
                robotHardware.setHorizontalElbowServoPosition(RobotConstants.HORIZONTAL_ELBOW_TRANSFER);
                robotHardware.setHorizontalWristServoPosition(RobotConstants.HORIZONTAL_WRIST_TRANSFER);
                robotHardware.setHorizontalSlidePosition(RobotConstants.HORIZONTAL_SLIDE_TRANSFER);

                robotHardware.setVerticalShoulderServoPosition(RobotConstants.VERTICAL_SHOULDER_DROP_SAMPLE_OBZONE);
                robotHardware.setVerticalElbowServoPosition(RobotConstants.VERTICAL_ELBOW_DROP_SAMPLE_OBZONE);
                robotHardware.setVerticalWristServoPosition(RobotConstants.VERTICAL_WRIST_DROP_SAMPLE_OBZONE);
                robotHardware.setVerticalSlidePosition(RobotConstants.VERTICAL_SLIDE_DROP_SAMPLE_OBZONE);

                robotHardware.setVerticalClawState(true);  //close vertical claw
                try {
                    Thread.sleep(100);
                }
                catch (InterruptedException e) {
                }

                currentRobotState = targetRobotState;

                break;

            case LOW_BASKET:   //LOW BASKET
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: LOW BASKET");

                // HORIZONTAL STATE SHOULD HAVE ALREADY BEEN DONE DURING TRANSFER
                robotHardware.setHorizontalTurretServoPosition(RobotConstants.HORIZONTAL_TURRET_TRANSFER);
                robotHardware.setHorizontalShoulderServoPosition(RobotConstants.HORIZONTAL_SHOULDER_TRANSFER);
                robotHardware.setHorizontalElbowServoPosition(RobotConstants.HORIZONTAL_ELBOW_TRANSFER);
                robotHardware.setHorizontalWristServoPosition(RobotConstants.HORIZONTAL_WRIST_TRANSFER);
                robotHardware.setHorizontalSlidePosition(RobotConstants.HORIZONTAL_SLIDE_TRANSFER);

                // TODO: MOVE ROBOT TO BASKET COORDINATES

                robotHardware.setVerticalSlidePosition(RobotConstants.VERTICAL_SLIDE_DROP_LOW_SAMPLE);
                robotHardware.setVerticalElbowServoPosition(RobotConstants.VERTICAL_ELBOW_DROP_LOW_SAMPLE);
                robotHardware.setVerticalWristServoPosition(RobotConstants.VERTICAL_WRIST_DROP_LOW_SAMPLE);
                robotHardware.setVerticalShoulderServoPosition(RobotConstants.VERTICAL_SHOULDER_DROP_LOW_SAMPLE);

                if (1 == 1) {   // TODO: update to check robot position
                    robotHardware.setVerticalClawState(true);  //open vertical claw
                    currentRobotState = targetRobotState;
                    targetRobotState = ROBOT_STATE.PICK_SAMPLE;
                }

                break;
            case HIGH_BASKET:   //HIGH BASKET
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: HIGH BASKET");

                // HORIZONTAL STATE SHOULD HAVE ALREADY BEEN DONE DURING TRANSFER
                robotHardware.setHorizontalTurretServoPosition(RobotConstants.HORIZONTAL_TURRET_TRANSFER);
                robotHardware.setHorizontalShoulderServoPosition(RobotConstants.HORIZONTAL_SHOULDER_TRANSFER);
                robotHardware.setHorizontalElbowServoPosition(RobotConstants.HORIZONTAL_ELBOW_TRANSFER);
                robotHardware.setHorizontalWristServoPosition(RobotConstants.HORIZONTAL_WRIST_TRANSFER);
                robotHardware.setHorizontalSlidePosition(RobotConstants.HORIZONTAL_SLIDE_TRANSFER);

                // TODO: MOVE ROBOT TO BASKET COORDINATES

                robotHardware.setVerticalShoulderServoPosition(RobotConstants.VERTICAL_SHOULDER_DROP_HIGH_SAMPLE);
                robotHardware.setVerticalSlidePosition(RobotConstants.VERTICAL_SLIDE_DROP_HIGH_SAMPLE);
                robotHardware.setVerticalElbowServoPosition(RobotConstants.VERTICAL_ELBOW_DROP_HIGH_SAMPLE);
                robotHardware.setVerticalWristServoPosition(RobotConstants.VERTICAL_WRIST_DROP_HIGH_SAMPLE);

                if (1 == 1) {   // TODO: update to check robot position
                    robotHardware.setVerticalClawState(true);  //open vertical claw
                    currentRobotState = targetRobotState;
                    targetRobotState = ROBOT_STATE.PICK_SAMPLE;
                }

                break;

            case PICK_SPECIMEN:   //PICK SPECIMEN
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: PICK SPECIMEN");

                //TODO: MOVE ROBOT TO PICK SPECIMEN

                // HORIZONTAL STATE SHOULD HAVE ALREADY BEEN DONE DURING TRANSFER
                robotHardware.setHorizontalTurretServoPosition(RobotConstants.HORIZONTAL_TURRET_PICK_SPECIMEN);
                robotHardware.setHorizontalShoulderServoPosition(RobotConstants.HORIZONTAL_SHOULDER_PICK_SPECIMEN);
                robotHardware.setHorizontalElbowServoPosition(RobotConstants.HORIZONTAL_ELBOW_PICK_SPECIMEN);
                robotHardware.setHorizontalWristServoPosition(RobotConstants.HORIZONTAL_WRIST_PICK_SPECIMEN);
                robotHardware.setHorizontalSlidePosition(RobotConstants.HORIZONTAL_SLIDE_PICK_SPECIMEN);

                robotHardware.setVerticalShoulderServoPosition(RobotConstants.VERTICAL_SHOULDER_PICK_SPECIMEN);
                robotHardware.setVerticalSlidePosition(RobotConstants.VERTICAL_SLIDE_PICK_SPECIMEN);
                robotHardware.setVerticalElbowServoPosition(RobotConstants.VERTICAL_ELBOW_PICK_SPECIMEN);
                robotHardware.setVerticalWristServoPosition(RobotConstants.VERTICAL_WRIST_PICK_SPECIMEN);

                currentRobotState = targetRobotState;

                break;

            case SNAP_SPECIMEN:   //SNAP SPECIMEN
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: SNAP SPECIMEN");

                //TODO: MOVE ROBOT TO SNAP SPECIMEN

                // HORIZONTAL STATE SHOULD HAVE ALREADY BEEN DONE DURING PICK SPECIMEN
                robotHardware.setHorizontalTurretServoPosition(RobotConstants.HORIZONTAL_TURRET_PICK_SPECIMEN);
                robotHardware.setHorizontalShoulderServoPosition(RobotConstants.HORIZONTAL_SHOULDER_PICK_SPECIMEN);
                robotHardware.setHorizontalElbowServoPosition(RobotConstants.HORIZONTAL_ELBOW_PICK_SPECIMEN);
                robotHardware.setHorizontalWristServoPosition(RobotConstants.HORIZONTAL_WRIST_PICK_SPECIMEN);
                robotHardware.setHorizontalSlidePosition(RobotConstants.HORIZONTAL_SLIDE_PICK_SPECIMEN);

                robotHardware.setVerticalSlidePosition(RobotConstants.VERTICAL_SLIDE_SNAP_HIGH_SPECIMEN);
                robotHardware.setVerticalElbowServoPosition(RobotConstants.VERTICAL_ELBOW_SNAP_HIGH_SPECIMEN);
                robotHardware.setVerticalWristServoPosition(RobotConstants.VERTICAL_WRIST_SNAP_HIGH_SPECIMEN);
                robotHardware.setVerticalShoulderServoPosition(RobotConstants.VERTICAL_SHOULDER_SNAP_HIGH_SPECIMEN);

                currentRobotState = targetRobotState;

                break;

            case ROBOT_HANG: //SLIDE CLOSED, ARM RESTING
                Log.i("=== INCREDIBOTS / ROBOT CONTROL ===", "PROCESSING STATE: ROBOT HANG");

                break;
        }
    }

    private void ProcessBumpers() {
        // if the right bumper is pressed it opens the claw
        if (gamepad2.right_bumper) {

//            if (robotHardware.isIntakeOn()) {
//                robotHardware.operateIntake(false);
//            }
//
//            if (readyToDropHighSample) {
//                robotHardware.ejectSampleFromIntake();
//
//                try {
//                    Thread.sleep(250);
//                } catch (InterruptedException e) {
//                    Thread.currentThread().interrupt();
//                }
//
//                robotHardware.operateClawServo((CLAW_CLOSE_POSITION + CLAW_OPEN_POSITION) / 2);
//
//                try {
//                    Thread.sleep(250);
//                } catch (InterruptedException e) {
//                    Thread.currentThread().interrupt();
//                }
//
//                robotHardware.operateClawServo(true);
//
//                try {
//                    Thread.sleep(250);
//                } catch (InterruptedException e) {
//                    Thread.currentThread().interrupt();
//                }
//                //move the robot arm back
//                robotState = ROBOT_STATE.CLAW_ARM_AFTER_HIGH_SAMPLE;
//
//                readyToDropHighSample = false;
//            }
//            else {
//                robotHardware.operateClawServo(true);
//            }
//
//        }
//        // if the left bumper is pressed it closes the claw
//        else if (gamepad2.left_bumper) {
//            //telemetry.addLine("left bumper pressed");
//            robotHardware.operateClawServo(false);
//
//            if (robotHardware.isIntakeOn()) {
//                robotHardware.operateIntake(true);
//            }
//
//        }
        }

//    private void HandleManualOverride() {
        // if the back button is pressed it switches manual ovverides value
//        if (gamepad2.left_stick_button && gamepad2.right_stick_button){
//            MANUAL_OVERRIDE = !MANUAL_OVERRIDE;
//            Log.i("=== INCREDIBOTS ===", "Manual Override: " + MANUAL_OVERRIDE);
//        }
//
//
//        // if manual override is true it will allow the joysticks to control the arms
//        // allow this only when robot has started (helpful in reset) or when picking samples
//        if (MANUAL_OVERRIDE && (robotState == ROBOT_STATE.PICK_SAMPLE || robotState == ROBOT_STATE.NONE || robotState == ROBOT_STATE.SNAP_SPECIMEN || robotState == ROBOT_STATE.HANG_SPECIMEN)) {
//
//            float leftYSignal = gamepad2.left_stick_y;
//
//            // If the left joystick is greater than zero, it moves the left arm up
//            if (leftYSignal > 0) {
//                robotHardware.setClawArmPositionAndVelocity(robotHardware.getClawArmMotorPos() + MANUAL_OVERRIDE_ARM_POSITION_DELTA, CLAW_ARM_VELOCITY * 2);
//            }
//
//            // If the left joystick is less than zero, it moves the left arm down
//            else if (leftYSignal < 0){
//                robotHardware.setClawArmPositionAndVelocity(robotHardware.getClawArmMotorPos() - MANUAL_OVERRIDE_ARM_POSITION_DELTA, CLAW_ARM_VELOCITY * 2);
//            }
//        }
//    }

//    private int GetMaxSlidePosition()
//    {
        //DEPENDING ON HOW THE CLAW ARM IS, THE SLIDE IS PERMITTED TO MOVE CERTAIN MAX DISTANCES.
//        int maxSlidePosition = -1;
//
//        if (robotHardware.getClawArmMotorPos() < DROP_SAMPLE_HIGH_ARM - 100) { //ARM IS BEHIND ROBOT
//            maxSlidePosition = MAX_SLIDE_POSITION_ARM_BACKWARDS_HIGH;
//        }
//        else if (robotHardware.getClawArmMotorPos() > DROP_SAMPLE_HIGH_ARM + 100) {
//            maxSlidePosition = MAX_SLIDE_POSITION_ARM_FORWARDS_LOW;
//        }
//
//        return maxSlidePosition;

//    }

//    private void ProcessDPad() {

        //DEPENDING ON HOW THE CLAW ARM IS, THE SLIDE IS PERMITTED TO MOVE CERTAIN MAX DISTANCES.
//        int maxSlidePosition = GetMaxSlidePosition();
//        int oldSlidePos = robotHardware.getSlidePos();

//        if (gamepad2.dpad_left) { //move wrist down
//            robotHardware.operateWristServo(robotHardware.getWristServoPosition() - 0.01);
//        }
//
//        if (gamepad2.dpad_right) { //move wrist up
//            robotHardware.operateWristServo(robotHardware.getWristServoPosition() + 0.01);
//        }

//        if (gamepad2.dpad_up){
//            Log.i("=== INCREDIBOTS ===", "PROCESSING DPAD UP");
//
//            //SLIDE CANNOT EXPAND BEYOND THE FAR POSITION FOR IT TO BE UNDER LIMITS
//            if (maxSlidePosition < 0) { //no max applies
//                robotHardware.setSlidePosition(robotHardware.getSlidePos() + MANUAL_OVERRIDE_SLIDE_POSITION_DELTA);
//            }
//            else {
//                robotHardware.setSlidePosition(Math.min(robotHardware.getSlidePos() + MANUAL_OVERRIDE_SLIDE_POSITION_DELTA, maxSlidePosition));
//            }
//
//            if (enableArmAdjustmentWithSlide) {
//
//                Log.i("=== INCREDIBOTS ===", "PROCESSING DPAD: ADJUSTING ARM POSITION WITH SLIDE POSITION");
//
//                if (robotHardware.getSlidePos() > oldSlidePos) {    //slide extended - lower arm, increase wrist position
//                    robotHardware.setClawArmPositionAndVelocity(robotHardware.getClawArmMotorPos() + ARM_DELTA_WITH_SLIDE_MOTION, CLAW_ARM_VELOCITY / 5);
//                    robotHardware.operateWristServo(robotHardware.getWristServoPosition() + WRIST_DELTA_WITH_SLIDE_MOTION);
//                }
//            }
//        }

        //process Dpad down input to retract linear slide
//        if (gamepad2.dpad_down){
//            Log.i("=== INCREDIBOTS ===", "PROCESSING DPAD DOWN");

        //SLIDE POSITION CANNOT BE LESS THAN 0
        // EXCEPT IF WE ARE DOING IT TO RESET THE SLIDE IN CASE OF AN ERROR
        // THAT IS WHEN THE ARM STATE WOULD BE NONE
//            robotHardware.setSlidePosition(robotHardware.getSlidePos() - MANUAL_OVERRIDE_SLIDE_POSITION_DELTA);
//
//            if (enableArmAdjustmentWithSlide) {
//
//                Log.i("=== INCREDIBOTS ===", "PROCESSING DPAD: ADJUSTING ARM POSITION WITH SLIDE POSITION");
//
//                if (robotHardware.getSlidePos() < oldSlidePos) {    //slide retracted - raise arm, decrease wrist position
//                    robotHardware.setClawArmPositionAndVelocity(robotHardware.getClawArmMotorPos() - ARM_DELTA_WITH_SLIDE_MOTION, CLAW_ARM_VELOCITY / 5);
//                    robotHardware.operateWristServo(robotHardware.getWristServoPosition() - WRIST_DELTA_WITH_SLIDE_MOTION);
//                }
//            }
//        }
    }

    private void HandleColorDetection() {
//        if (!robotHardware.isIntakeOn()) {
//            return; //do nothing if intake was not operating
//        }
//
//        if (gameColor == RobotConstants.GAME_COLORS.BLUE && robotHardware.getDetectedColor() == RobotConstants.GAME_COLORS.RED) {
//            while (robotHardware.getDetectedColor() == RobotConstants.GAME_COLORS.RED) {
//                robotHardware.operateWristServo(WRIST_SPIT_OUT);
//                robotHardware.operateIntake(false);
//            }
//            robotHardware.operateWristServo(ENTER_SUB_WRIST);
//            robotHardware.operateIntake(true);
//        }
//
//        if (gameColor == RobotConstants.GAME_COLORS.RED && robotHardware.getDetectedColor() == RobotConstants.GAME_COLORS.BLUE) {
//            while (robotHardware.getDetectedColor() == RobotConstants.GAME_COLORS.BLUE) {
//                robotHardware.operateWristServo(WRIST_SPIT_OUT);
//                robotHardware.operateIntake(false);
//            }
//            robotHardware.operateWristServo(ENTER_SUB_WRIST);
//            robotHardware.operateIntake(true);
//        }
    }

    private void HandleMotorCurrentProblems() {
//        if (((DcMotorEx) armMotor).isOverCurrent()){
//            telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
//        }
    }

}