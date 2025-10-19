package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Actions.IntakeWheels;
import org.firstinspires.ftc.teamcode.Actions.LaunchFlywheel;
import org.firstinspires.ftc.teamcode.Actions.LaunchGate;
import org.firstinspires.ftc.teamcode.Actions.LaunchKick;
import org.firstinspires.ftc.teamcode.Actions.Spindex;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import java.util.ArrayList;
import java.util.List;

public class MechanismControl {
    private Gamepad gamepad2;
    private RobotHardware robotHardware;
    private List<Action> runningActions;
    private FtcDashboard dashboard;
    private Telemetry telemetry;

    private enum ROBOT_STATE {
        NONE,
        INTAKE_ON,
        INTAKE_OFF,
        LAUNCH_CLOSE,
        LAUNCH_MID,
        LAUNCH_FAR,
        LIFT
    }

    private ROBOT_STATE currentRobotState;
    private ROBOT_STATE targetRobotState;
    private boolean stateTransitionInProgress;

    public MechanismControl(Gamepad gamepad, RobotHardware robotHardware, Telemetry telemetry) {
        gamepad2 = gamepad;
        this.telemetry = telemetry;
        this.robotHardware = robotHardware;
        currentRobotState = ROBOT_STATE.NONE;
        targetRobotState = ROBOT_STATE.NONE;
        stateTransitionInProgress = false;
        runningActions = new ArrayList<>();
        dashboard = FtcDashboard.getInstance();
    }

    public void ProcessInputs() {
        CreateStateFromButtonPress();

        TranslateStateIntoActions();

        ProcessActions();

    }

    //Function to create a state from Gamepad inputs
    private void CreateStateFromButtonPress() {

        ROBOT_STATE newTargetRobotState =  ROBOT_STATE.NONE;

        if (gamepad2.aWasPressed()) {
            if (currentRobotState != ROBOT_STATE.INTAKE_ON) {
                newTargetRobotState = ROBOT_STATE.INTAKE_ON;
            }
            else {
                newTargetRobotState = ROBOT_STATE.INTAKE_OFF;
            }
        }

        if (gamepad2.yWasPressed()) {
            newTargetRobotState = ROBOT_STATE.LAUNCH_CLOSE;
        }

        if (gamepad2.xWasPressed()) {
            newTargetRobotState = ROBOT_STATE.LAUNCH_MID;
        }

        if (gamepad2.bWasPressed()) {
            newTargetRobotState = ROBOT_STATE.LAUNCH_FAR;
        }

        if (newTargetRobotState != ROBOT_STATE.NONE) {

            Log.i("=== MECHANISM CONTROL ===", "GAMEPAD INPUTS RECEIVED FOR STATE CHANGE CURRENT: " + currentRobotState + " TARGET: " + newTargetRobotState);

            if (targetRobotState != ROBOT_STATE.NONE && newTargetRobotState != targetRobotState) {
                //STOP ALL PROCESSING - STATE TRANSITION WAS GOING ON WHEN NEW STATE WAS CALLED IN
                Log.i("=== MECHANISM CONTROL ===", "STOPPING! STATE TRANSITION WAS GOING ON WHEN NEW TARGET WAS CALLED IN. OLD TARGET: " + targetRobotState + " NEW TARGET: " + newTargetRobotState);

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


    public void Rishi (){
        currentRobotState = ROBOT_STATE.NONE;

        targetRobotState = ROBOT_STATE.LAUNCH_FAR;

        stateTransitionInProgress = false;

    }

    private void TranslateStateIntoActions() {

//        if (currentRobotState == targetRobotState) return;

        if (!stateTransitionInProgress) {

            switch (targetRobotState) {
                case INTAKE_ON:
                case INTAKE_OFF:
                    Log.i("=== MECHANISM CONTROL ===", "PROCESSING STATE: INTAKE ON / OFF");

                    //get the list of actions and put it in running actions
                    runningActions.add(new ParallelAction(
                            new LaunchFlywheel(robotHardware, LaunchPositions.NONE),
                            new LaunchGate(robotHardware, false),
                            new LaunchKick(robotHardware, false)
                    ));

                    if (targetRobotState == ROBOT_STATE.INTAKE_ON) {
                        //get the list of actions and put it in running actions
                        runningActions.add(new ParallelAction(
                                new IntakeWheels(robotHardware, true),
                                new Spindex(robotHardware, true)
                        ));
                    } else {
                        runningActions.add(new ParallelAction(
                                new IntakeWheels(robotHardware, false),
                                new Spindex(robotHardware, false)
                        ));
                    }

                    break;

                case LAUNCH_CLOSE:
                case LAUNCH_MID:
                case LAUNCH_FAR:

                    LaunchPositions launchPosition = LaunchPositions.CLOSE;

                    if (targetRobotState == ROBOT_STATE.LAUNCH_MID) {
                        launchPosition = LaunchPositions.MID;
                    }

                    if (targetRobotState == ROBOT_STATE.LAUNCH_FAR) {
                        launchPosition = LaunchPositions.FAR;
                    }

                    Log.i("=== MECHANISM CONTROL ===", "PROCESSING STATE: LAUNCH");

                    runningActions.add(new SequentialAction(
                            new LaunchFlywheel(robotHardware, launchPosition),
                            new ParallelAction(
                                new LaunchGate(robotHardware, true),
                                new IntakeWheels(robotHardware, false),
                                new Spindex(robotHardware, false)
                            ),
                            new LaunchKick(robotHardware, true),
                            new LaunchKick(robotHardware, false)
                    ));
                    break;

                case LIFT:
                    Log.i("=== MECHANISM CONTROL ===", "PROCESSING STATE: LIFT");

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

                Log.i("=== MECHANISM CONTROL ===", "ProcessActions: DONE RUNNING ACTIONS");
                Log.i("=== MECHANISM CONTROL ===", "ProcessActions: CURRENT ROBOT STATE: " + currentRobotState);
                Log.i("=== MECHANISM CONTROL ===", "ProcessActions: TARGET ROBOT STATE: " + targetRobotState);

                //NOTE: CAREFUL NOT TO CONSTRUCT LOOPS HERE
                // STATE A -> STATE B -> STATE A
                //set new state based on older target

            }
        }
    }

}
