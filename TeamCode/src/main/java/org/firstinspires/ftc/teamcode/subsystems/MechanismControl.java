package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.Actions.ResetSpindexerAction.SPINDEXER_INCREMENT;

import android.util.Log;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.LimelightAprilTagHelper;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

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

    private enum ROBOT_STATE {
        NONE,
        INTAKE,
        REVERSE_INTAKE,
        LAUNCH_ONE,
        LAUNCH_GREEN,
        LAUNCH_PURPLE,
        LAUNCH_ALL,
        LAUNCH_MANUAL_ONE_CLOSE,
        LAUNCH_MANUAL_ONE_MID,
        LAUNCH_MANUAL_ONE_FAR,
        PARK,
        RESET_SPINDEXER
    }

    private ROBOT_STATE currentRobotState;
    private ROBOT_STATE targetRobotState;
    private boolean stateTransitionInProgress;
    public MechanismControl(Gamepad gamepad, RobotHardware robotHardware, LimelightAprilTagHelper limelightAprilTagHelper, Telemetry telemetry) {
        gamepad2 = gamepad;
        this.telemetry = telemetry;
        this.robotHardware = robotHardware;
        this.limelightAprilTagHelper = limelightAprilTagHelper;

        this.spindex = new Spindex(robotHardware);

        this.intakeSystem = new IntakeSystem(robotHardware, this.spindex);
        this.launchSystem = new LaunchSystem(robotHardware, this.spindex, limelightAprilTagHelper);

        currentRobotState = ROBOT_STATE.NONE;
        targetRobotState = ROBOT_STATE.NONE;
        stateTransitionInProgress = false;
        runningActions = new ArrayList<>();
        dashboard = FtcDashboard.getInstance();
    }

    public void processInputs() {
        CreateStateFromButtonPress();

        CreateStateAutomatically();

        TranslateStateIntoActions();

        ProcessActions();

        ProcessBackButton();
        ProcessDPad();

        //this has to be done after translating any state into actions
        CheckForBallsToIntake();

        //process the yaw and rotate the turret always - except when parking
        if (currentRobotState != ROBOT_STATE.PARK && targetRobotState != ROBOT_STATE.PARK) {
            launchSystem.AlignTurretToGoal();
        }

        //keep warm only if we are intake mode. Else this will interfere with launch parameters
        if ((currentRobotState == ROBOT_STATE.INTAKE || targetRobotState == ROBOT_STATE.INTAKE) && (!stateTransitionInProgress)) {
//            Log.i("== MECHANISM CONTROL ==", "Calling to keep launcher warm from main loop. Current state:" + currentRobotState + " target state: " + targetRobotState);
            launchSystem.KeepLauncherWarm();
        }


        //TODO: this is for profiling flywheel velocity at launches
//        if (currentRobotState != ROBOT_STATE.LAUNCH_ALL && targetRobotState == ROBOT_STATE.LAUNCH_ALL && stateTransitionInProgress) {
//            Log.i("MECHANISM CONTROL" , "FLYWHEEL VELOCITY: " + robotHardware.getFlywheelMotorVelocityInTPS());
//        }

        //update the light to reflect the number of balls in the spindex.
        intakeSystem.updateStatusLight();
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
            newTargetRobotState = ROBOT_STATE.PARK;
        }

//        if (gamepad2.backWasPressed()) {
////            newTargetRobotState = ROBOT_STATE.REVERSE_INTAKE;
//        }

        if (gamepad2.left_trigger > 0.5 && gamepad2.yWasPressed()) {
            newTargetRobotState = ROBOT_STATE.LAUNCH_MANUAL_ONE_CLOSE;
        }

        if (gamepad2.left_trigger > 0.5 && gamepad2.xWasPressed()) {
            newTargetRobotState = ROBOT_STATE.LAUNCH_MANUAL_ONE_MID;
        }

        if (gamepad2.left_trigger > 0.5 && gamepad2.bWasPressed()) {
            newTargetRobotState = ROBOT_STATE.LAUNCH_MANUAL_ONE_FAR;
        }

        if (gamepad2.dpadDownWasPressed()) {
            Log.i("== MECHANISM CONTROL ==", "GAMEPAD INPUTS RECEIVED FOR DPAD DOWN");
            newTargetRobotState = ROBOT_STATE.RESET_SPINDEXER;
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
        if (spindex.isEmpty()
                && currentRobotState != ROBOT_STATE.INTAKE
                && targetRobotState != ROBOT_STATE.INTAKE
                && currentRobotState != ROBOT_STATE.PARK
                && targetRobotState != ROBOT_STATE.PARK
                && currentRobotState != ROBOT_STATE.RESET_SPINDEXER
                && targetRobotState != ROBOT_STATE.RESET_SPINDEXER) {

            Log.i("== MECHANISM CONTROL ==", "CreateStateAutomatically - setting to INTAKE");
            targetRobotState = ROBOT_STATE.INTAKE;
        }
    }

    private void TranslateStateIntoActions() {

//        if (currentRobotState == targetRobotState) return;
//        Log.i("== MECHANISM CONTROL ==","TranslateStateIntoActions: current state: " + currentRobotState);
//        Log.i("== MECHANISM CONTROL ==","TranslateStateIntoActions: target state: " + targetRobotState);
//        Log.i("== MECHANISM CONTROL ==","TranslateStateIntoActions: stateTransitionInProgress: " + stateTransitionInProgress);


        if (!stateTransitionInProgress) {

            switch (targetRobotState) {

                case INTAKE:
                    Log.i("== MECHANISM CONTROL ==", "PROCESSING STATE: INTAKE");

                    //get the list of actions and put it in running actions
                    runningActions.add(
                            new SequentialAction(
                                    launchSystem.getKeepWarmAction(),
                                    intakeSystem.getTurnOnAction()
                            ));
                    break;

                case REVERSE_INTAKE:
                    Log.i("== MECHANISM CONTROL ==", "PROCESSING STATE: REVERSE_INTAKE");
                    runningActions.add(
                            new SequentialAction(
                                    spindex.moveToNextFullSlotAction(), //move to full slot so we dont end up spitting out a ball that we took in
                                    intakeSystem.getReverseIntakeAction(true)
                            ));
                    break;

                case LAUNCH_ONE:
                    Log.i("== MECHANISM CONTROL ==", "PROCESSING STATE: LAUNCH ONE");
                    runningActions.add(
                            new ParallelAction(
                                intakeSystem.getTurnOffAction(),
                                launchSystem.getLaunchNextBallAction()
                            ));
                    break;

                case LAUNCH_GREEN:
                    Log.i("== MECHANISM CONTROL ==", "PROCESSING STATE: LAUNCH GREEN");
                    runningActions.add(
                            new ParallelAction(
                                    intakeSystem.getTurnOffAction(),
                                    launchSystem.getLaunchGreenBallAction()
                            ));
                    break;

                case LAUNCH_PURPLE:
                    Log.i("== MECHANISM CONTROL ==", "PROCESSING STATE: LAUNCH PURPLE");
                    runningActions.add(
                            new ParallelAction(
                                    intakeSystem.getTurnOffAction(),
                                    launchSystem.getLaunchPurpleBallAction()
                            ));
                    break;

                case LAUNCH_ALL:
                    Log.i("== MECHANISM CONTROL ==", "PROCESSING STATE: LAUNCH ALL");
                    runningActions.add(
                            new ParallelAction(
                                    intakeSystem.getTurnOffAction(),
                                    launchSystem.getLaunchAllBallsAction()
                            ));
                    break;

                case LAUNCH_MANUAL_ONE_CLOSE:
                    Log.i("== MECHANISM CONTROL ==", "PROCESSING STATE: LAUNCH MANUAL ONE CLOSE");
                    runningActions.add(
                            new ParallelAction(
                                    intakeSystem.getTurnOffAction(),
                                    launchSystem.getLaunchNextBallCloseAction()
                            ));
                    break;

                case LAUNCH_MANUAL_ONE_MID:
                    Log.i("== MECHANISM CONTROL ==", "PROCESSING STATE: LAUNCH MANUAL ONE MID");
                    runningActions.add(
                            new ParallelAction(
                                    intakeSystem.getTurnOffAction(),
                                    launchSystem.getLaunchNextBallMidAction()
                            ));
                    break;

                case LAUNCH_MANUAL_ONE_FAR:
                    Log.i("== MECHANISM CONTROL ==", "PROCESSING STATE: MANUAL ONE FAR");
                    runningActions.add(
                            new ParallelAction(
                                    intakeSystem.getTurnOffAction(),
                                    launchSystem.getLaunchNextBallFarAction()
                            ));
                    break;

                case PARK:
                    Log.i("== MECHANISM CONTROL ==", "PROCESSING STATE: PARK");

                    runningActions.add(new ParallelAction(
                            intakeSystem.getTurnOffAction(),
                            launchSystem.getTurnOffAction()
                    ));
                    break;

                case RESET_SPINDEXER:
                    Log.i("== MECHANISM CONTROL ==", "PROCESSING STATE: RESET SPINDEXER");

                    runningActions.add(
                            intakeSystem.getResetAction()
                    );

                    break;
            }
        }
    }

    private void CheckForBallsToIntake() {
;        if (currentRobotState == ROBOT_STATE.INTAKE && targetRobotState != ROBOT_STATE.REVERSE_INTAKE && intakeSystem.isOn) {
//            if (!spindex.isFull())
            {
                //set targetrobotstate to intake so that processactions does not clear it out
                //running the actions with target robot state = none will set the current to none
                //for ex intake is running and we check for balls without a new target. In this case,
                //the control loop will run once and set the current to target - which will both end up
                //as none - then the intake action will not happen continuously.

                //We fixed this by setting the current state to target only if target is not NONE

                //all state based actions are added before this function and all actions
                //are processed after this function - so setting the target state here should be safe
                //targetRobotState = ROBOT_STATE.INTAKE;

//                Log.i("Mechanism Control", "Adding CheckForBallsToIntake Action Automatically");
                runningActions.add(intakeSystem.checkForBallIntakeAndGetActionTeleop());
            }
//            else
//            {
//                if (intakeSystem.isOn) {
////                Log.i("Mechanism Control", "Spindex Full: added moveToNextFullSlotAction");
////                    runningActions.add(new SequentialAction(
////                            intakeSystem.indexAnyUnknowns(),
////                            spindex.moveToNextFullSlotAction()
////                    ));
//                }
//            }
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
                //when running actions, it is possible that we queued up automated actions based on current state - ball intake for example
                //in that case, the target would not change - it would stay NONE - we are running the ball intake in the current INTAKE state.
                //this function should not set the current state to NONE in such cases.
                if (targetRobotState != ROBOT_STATE.NONE)
                    currentRobotState = targetRobotState;

                targetRobotState = ROBOT_STATE.NONE;
                stateTransitionInProgress = false;

//                Log.i("== MECHANISM CONTROL ==", "ProcessActions: DONE RUNNING ACTIONS");
//                Log.i("== MECHANISM CONTROL ==", "ProcessActions: CURRENT ROBOT STATE: " + currentRobotState);
//                Log.i("== MECHANISM CONTROL ==", "ProcessActions: TARGET ROBOT STATE: " + targetRobotState);

                //NOTE: CAREFUL NOT TO CONSTRUCT LOOPS HERE
                // STATE A -> STATE B -> STATE A
                //set new state based on older target

            }
        }
    }

    public void ProcessBackButton() {
        if (gamepad2.backWasPressed()) {
            //this is an emergency operation - we clear out the actions and we clear out state transition flag
            // so that the state change can actually happen.
            runningActions.clear();
            stateTransitionInProgress = false;
            targetRobotState = ROBOT_STATE.REVERSE_INTAKE;
        }
    }

    private void ProcessDPad() {

        if (gamepad2.dpadLeftWasPressed()) {
            int currentPos = robotHardware.getSpindexPosition();
            robotHardware.setSpindexPosition(currentPos - SPINDEXER_INCREMENT);
        }

        if (gamepad2.dpadRightWasPressed()) {
            int currentPos = robotHardware.getSpindexPosition();
            robotHardware.setSpindexPosition(currentPos + SPINDEXER_INCREMENT);
        }

        if (gamepad2.dpadUpWasPressed()) {
            //this is an emergency operation - we clear out the actions and we clear out state transition flag
            // so that the state change can actually happen.
            runningActions.clear();
            stateTransitionInProgress = false;
            targetRobotState = ROBOT_STATE.INTAKE;
        }
    }
}
