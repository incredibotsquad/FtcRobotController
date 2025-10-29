package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightAprilTagHelper;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import java.util.ArrayList;
import java.util.List;

public class MechanismControl {
    private Gamepad gamepad2;
    private RobotHardware robotHardware;
    private Spindex spindex;
    private IntakeSystem intakeSystem;
    private LaunchSystem launchSystem;

    private LimelightAprilTagHelper limelightAprilTagHelper;
    private List<Action> runningActions;
    private FtcDashboard dashboard;
    private Telemetry telemetry;

    private AllianceColors allianceColor;

    private enum ROBOT_STATE {
        NONE,
        INTAKE,
        LAUNCH_ONE,
        LAUNCH_GREEN,
        LAUNCH_PURPLE,
        LAUNCH_ALL,
        LIFT
    }

    private ROBOT_STATE currentRobotState;
    private ROBOT_STATE targetRobotState;
    private boolean stateTransitionInProgress;

    public MechanismControl(Gamepad gamepad, RobotHardware robotHardware, Telemetry telemetry) {
        gamepad2 = gamepad;
        this.telemetry = telemetry;
        this.robotHardware = robotHardware;
        this.spindex = new Spindex(robotHardware);
        this.intakeSystem = new IntakeSystem(robotHardware, this.spindex);
        this.launchSystem = new LaunchSystem(robotHardware, this.spindex);
        this.limelightAprilTagHelper = new LimelightAprilTagHelper(robotHardware);

        currentRobotState = ROBOT_STATE.NONE;
        targetRobotState = ROBOT_STATE.NONE;
        stateTransitionInProgress = false;
        runningActions = new ArrayList<>();
        dashboard = FtcDashboard.getInstance();
    }

    public void setAllianceColor(AllianceColors color) {
        this.allianceColor = color;
        limelightAprilTagHelper.setAllianceColor(color);
    }

    public void ProcessInputs() {
        CreateStateFromButtonPress();

        CreateStateAutomatically();

        TranslateStateIntoActions();

        //this has to be done after translating any state into actions
        CheckForBallsToIntake();

        ProcessActions();
    }


    //Function to create a state from Gamepad inputs
    private void CreateStateFromButtonPress() {

        ROBOT_STATE newTargetRobotState =  ROBOT_STATE.NONE;

        if (gamepad2.bWasPressed()) {
            newTargetRobotState = ROBOT_STATE.LAUNCH_ONE;
        }

        if (gamepad2.aWasPressed()) {
            newTargetRobotState = ROBOT_STATE.LAUNCH_GREEN;
        }

        if (gamepad2.xWasPressed()) {
            newTargetRobotState = ROBOT_STATE.LAUNCH_PURPLE;
        }

        if (gamepad2.right_trigger > 0.5) {
            newTargetRobotState = ROBOT_STATE.LAUNCH_ALL;
        }

        if (gamepad2.startWasPressed()) {
            newTargetRobotState = ROBOT_STATE.LIFT;
        }

        if (newTargetRobotState != ROBOT_STATE.NONE) {

            Log.i("== MECHANISM CONTROL ==", "GAMEPAD INPUTS RECEIVED FOR STATE CHANGE CURRENT: " + currentRobotState + " TARGET: " + newTargetRobotState);

            if (targetRobotState != ROBOT_STATE.NONE && newTargetRobotState != targetRobotState) {
                //STOP ALL PROCESSING - STATE TRANSITION WAS GOING ON WHEN NEW STATE WAS CALLED IN
                Log.i("== MECHANISM CONTROL ==", "STOPPING! STATE TRANSITION WAS GOING ON WHEN NEW TARGET WAS CALLED IN. OLD TARGET: " + targetRobotState + " NEW TARGET: " + newTargetRobotState);

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

    private void CreateStateAutomatically() {
        //IF SPINDEXER IS EMPTY AND WE HAVE ALREADY NOT STARTED THE PROCESS OF INTAKING, START IT
        if (spindex.isEmpty() && currentRobotState != ROBOT_STATE.INTAKE && targetRobotState != ROBOT_STATE.INTAKE) {
            Log.i("== MECHANISM CONTROL ==", "CreateStateAutomatically");

            targetRobotState = ROBOT_STATE.INTAKE;
        }
    }

    private void TranslateStateIntoActions() {

//        if (currentRobotState == targetRobotState) return;

        if (!stateTransitionInProgress) {

            switch (targetRobotState) {

                case INTAKE:
                    Log.i("== MECHANISM CONTROL ==", "PROCESSING STATE: INTAKE");

                    //get the list of actions and put it in running actions
                    runningActions.add(new ParallelAction(
                        launchSystem.getLockLauncherForIntakeAction(),
                        intakeSystem.getTurnOnAction(),
                        //launchSystem.getTurnOnAction()
                            new NullAction()
                    ));

                    break;

                case LAUNCH_ONE:
                    Log.i("== MECHANISM CONTROL ==", "PROCESSING STATE: LAUNCH ONE");
                    runningActions.add(launchSystem.getLaunchNextBallAction());
                    break;

                case LAUNCH_GREEN:
                    Log.i("== MECHANISM CONTROL ==", "PROCESSING STATE: LAUNCH GREEN");
                    runningActions.add(launchSystem.getLaunchGreenBallAction());
                    break;

                case LAUNCH_PURPLE:
                    Log.i("== MECHANISM CONTROL ==", "PROCESSING STATE: LAUNCH PURPLE");
                    runningActions.add(launchSystem.getLaunchPurpleBallAction());
                    break;

                case LAUNCH_ALL:
                    Log.i("== MECHANISM CONTROL ==", "PROCESSING STATE: LAUNCH ALL");
                    runningActions.add(launchSystem.getLaunchAllBallsAction());
                    break;

                case LIFT:
                    Log.i("== MECHANISM CONTROL ==", "PROCESSING STATE: LIFT");

                    runningActions.add(new ParallelAction(
                            intakeSystem.getTurnOffAction(),
                            launchSystem.getTurnOffAction()
                    ));
                    break;
            }

        }
    }

    private void CheckForBallsToIntake() {
        if (currentRobotState == ROBOT_STATE.INTAKE ) {
            if (!spindex.isFull())
            {
                //set targetrobotstate to intake so that processactions does not clear it out
                //if it get cleared out, ball intake will not work properly
                //all state based actions are added before this function and all actions
                //are processed after this function - so setting the target state here should be safe
                targetRobotState = ROBOT_STATE.INTAKE;
                runningActions.add(intakeSystem.checkForBallIntakeAndGetAction());
            }
            else
            {
                runningActions.add(spindex.moveToNextFullSlotAction());
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

                Log.i("== MECHANISM CONTROL ==", "ProcessActions: DONE RUNNING ACTIONS");
                Log.i("== MECHANISM CONTROL ==", "ProcessActions: CURRENT ROBOT STATE: " + currentRobotState);
                Log.i("== MECHANISM CONTROL ==", "ProcessActions: TARGET ROBOT STATE: " + targetRobotState);

                //NOTE: CAREFUL NOT TO CONSTRUCT LOOPS HERE
                // STATE A -> STATE B -> STATE A
                //set new state based on older target

            }
        }
    }




}
