package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.lang.Math;

@Config
public class IncredibotsArmControl
{
    private Gamepad gamepad2;
    private RobotHardware robotHardware;

    // CLAW ARM CONSTANTS
    public static int CLAW_ARM_RESTING_BACK = 0;
    public static int CLAW_ARM_VELOCITY = 750;
    public static int CLAW_ARM_AUTO_VELOCITY_SNAP_SPECIMEN = 1400;
    public static int CLAW_ARM_ADJUSTMENT_WITH_SLIDE = 8;

    //CLAW CONSTANTS
    public static double CLAW_OPEN_POSITION = 0.1;
    public static double CLAW_CLOSE_POSITION = 0.55;

    //WRIST CONSTANTS
    public static double WRIST_PRELOAD = 0.2;

    //SLIDE CONSTANTS
    public static int SLIDE_POSITION_RESTING = 0;
    public static int SLIDE_VELOCITY_EXPANDING = 3000;
    public static int SLIDE_VELOCITY_CONTRACTING = SLIDE_VELOCITY_EXPANDING;
    public static int MAX_SLIDE_POSITION_ARM_FORWARDS_LOW = 1250;
    public static int MAX_SLIDE_POSITION_ARM_BACKWARDS_HIGH = 2300;

    //MANUAL OVERRIDE CONSTANTS
    private boolean MANUAL_OVERRIDE = false;
    private static int MANUAL_OVERRIDE_ARM_POSITION_DELTA = 25;
    private static int MANUAL_OVERRIDE_SLIDE_POSITION_DELTA = 100;

    //PICKING SAMPLES: RT/A
    public static int CLAW_ARM_PICK_SAMPLE = 1354;
    public static int SLIDE_POSITION_PICK_SAMPLE = 0;
    public static double WRIST_PICK_SAMPLE = 0.2;

    //ENTER EXIT SUB: RT/X
    public static int CLAW_ARM_ENTER_SUB = 1360;
    public static int SLIDE_ENTER_SUB = 0;
    public static double WRIST_ENTER_SUB = 0.5;

    //DROP SAMPLES
    public static int CLAW_ARM_DROP_SAMPLE_HIGH = 650;//847
    public static int CLAW_ARM_AFTER_DROP_SAMPLE_HIGH = 720;
    public static int CLAW_ARM_DROP_SAMPLE_LOW = 600;
    public static double WRIST_DROP_SAMPLE = 0.5;
    public static int SLIDE_POSITION_HIGH_BASKET = 3000;
    public static int SLIDE_POSITION_LOW_BASKET = 700;

    //PICK SPECIMEN
    public static int CLAW_ARM_PICK_SPECIMEN = 0;
    public static double WRIST_PICK_SPECIMEN = 0.2;

    //HANG SPECIMEN
    public static int CLAW_ARM_HANG_SPECIMEN = 670;
    public static int SLIDE_POSITION_HANG_SPECIMEN = 260;
    public static double WRIST_HANG_SPECIMEN = 0.1;

    //SNAP SPECIMEN
    public static int CLAW_ARM_SNAP_SPECIMEN = 500;
    public static int SLIDE_POSITION_SNAP_SPECIMEN = 400;
    public static double WRIST_SNAP_SPECIMEN = 0.1;

    private enum ARM_STATE {
        NONE,
        RESTING,
        ROBOT_HANG,
        PICK_SPECIMEN,
        PICK_SAMPLE,
        SNAP_SPECIMEN,
        LOW_BASKET,
        ARM_VERTICAL,
        ENTER_EXIT_SUB,
        HANG_SPECIMEN,
        HIGH_BASKET,
        CLAW_ARM_AFTER_HIGH_SAMPLE,
        RESET_SLIDE_ENCODER
    }

    private ARM_STATE armState;

    private boolean readyToDropHighSample = false;
    private boolean readyToPickUpSample = false;
    private boolean processedPickSample = false;

    public IncredibotsArmControl(Gamepad gamepad, RobotHardware robotHardware) {
        gamepad2 = gamepad;
        this.robotHardware = robotHardware;
        armState = ARM_STATE.NONE;
    }

    public void ProcessInputs(Telemetry telemetry) {

        CreateStateFromButtonPress();
        ProcessButtons();

        ProcessBumpers();

        ProcessDPad();

        HandleManualOverride();
    }

    private void CreateStateFromButtonPress() {

        ARM_STATE prevArmState = armState;

        if (gamepad2.back) {
            armState = ARM_STATE.RESTING;
        }

        if (gamepad2.back && gamepad2.start) {
            armState = ARM_STATE.RESET_SLIDE_ENCODER;
        }

        if (gamepad2.start && gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
            armState = ARM_STATE.ROBOT_HANG;
        }

        if (gamepad2.a){
            if (gamepad2.left_trigger > 0) {
                armState = ARM_STATE.PICK_SPECIMEN;
            }
            else if (gamepad2.right_trigger > 0) {
                armState = ARM_STATE.PICK_SAMPLE;
                processedPickSample = false;
            }
        }

        if (gamepad2.b) {
            if (gamepad2.left_trigger > 0) {
                armState = ARM_STATE.SNAP_SPECIMEN;
            }
            else if (gamepad2.right_trigger > 0) {
                armState = ARM_STATE.LOW_BASKET;
            }
        }

        if (gamepad2.x) {
            if (gamepad2.left_trigger > 0) {
                armState = ARM_STATE.ARM_VERTICAL;
            }
            else if (gamepad2.right_trigger > 0) {
                armState = ARM_STATE.ENTER_EXIT_SUB;
            }
        }

        if (gamepad2.y) {
            if (gamepad2.left_trigger > 0) {
                armState = ARM_STATE.HANG_SPECIMEN;
            }
            else if (gamepad2.right_trigger > 0){
                armState = ARM_STATE.HIGH_BASKET;
            }
        }

        // if previous state was not NONE (some motion was ongoing / pending)
        // and if new state is not NONE (some motion is going to happen)
        // and if previous and new are different
        // then cancel whatever was going on in the previous state and process the new one.
        // any pending actions for the previous state will not get executed since the state itself will change.
        if (prevArmState != ARM_STATE.NONE && armState != ARM_STATE.NONE && prevArmState != armState) {
            Log.i("=== INCREDIBOTS ===", "STOPPING ARMS SINCE ARM STATE CHANGE. OLD: " + prevArmState + " NEW: " + armState);
            robotHardware.stopSlide();
            robotHardware.stopClawArm();
        }

        if (armState != prevArmState) {
            Log.i("=== INCREDIBOTS ===", "NEW ARM STATE: " + armState);
        }
    }

    //functions to take gamepad inputs and turn it into movements
    private void ProcessButtons() {

        switch (armState) {
            case RESET_SLIDE_ENCODER:
                Log.i("=== INCREDIBOTS ===", "PROCESSING RESET_SLIDE_ENCODER");

                robotHardware.stopAndResetSlideEncoder();
                armState = ARM_STATE.NONE;
                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;

            case RESTING:
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: RESTING");

//                armState = ARM_STATE.NONE;  //clear out state to avoid reprocessing
//                readyToPickUpSample = false;
//                readyToDropHighSample = false;
//                Actions.runBlocking(GetRestingActionSequence());

                if (!robotHardware.isSlideMotorBusy() && robotHardware.getSlidePos() != SLIDE_POSITION_RESTING) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING RESTING - SLIDE IS BUSY");
                    robotHardware.setSlidePosition(SLIDE_POSITION_RESTING);
                }

                //has to be a separate check since the previous action will make the slide motor busy
                if (!robotHardware.isSlideMotorBusy() || Math.abs(SLIDE_POSITION_RESTING - robotHardware.getSlidePos()) < 10) {
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_RESTING_BACK, CLAW_ARM_VELOCITY);
                    robotHardware.operateClawServo(false);
                    Log.i("=== INCREDIBOTS ===", "PROCESSING RESTING - SLIDE FINISHED MOVING - STARTING ARM. ARM_STATE: "+ armState);
                }

                if (Math.abs(CLAW_ARM_RESTING_BACK - robotHardware.getClawArmMotorPos()) < 50) {
                    Log.i("=== INCREDIBOTS ===", "DONE PROCESSING STATE: RESTING, CALLING TO RESET ARM ENCODER NOW");
                    robotHardware.stopAndResetArmEncoder();
                    armState = ARM_STATE.NONE;
                }

                readyToPickUpSample = false;
                readyToDropHighSample = false;

                break;

            case ROBOT_HANG: //SLIDE CLOSED, ARM RESTING
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: ROBOT HANG");

//                armState = ARM_STATE.NONE; //clear out state to avoid reprocessing
//                Actions.runBlocking(GetRobotHangActionSequence());

                if (!robotHardware.isSlideMotorBusy() && robotHardware.getSlidePos() != SLIDE_POSITION_RESTING) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING ROBOT HANG - SLIDE IS BUSY");
                    robotHardware.setSlidePosition(SLIDE_POSITION_RESTING);
                }

                //has to be a separate check since the previous action will make the slide motor busy
                if (!robotHardware.isSlideMotorBusy() || Math.abs(SLIDE_POSITION_RESTING - robotHardware.getSlidePos()) < 10) {
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_RESTING_BACK, CLAW_ARM_VELOCITY);
                    robotHardware.operateClawServo(false);
                    armState = ARM_STATE.NONE;
                    Log.i("=== INCREDIBOTS ===", "PROCESSING ROBOT HANG - SLIDE FINISHED MOVING - STARTING ARM. ARM_STATE: "+ armState);
                }

                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;

            case PICK_SPECIMEN:   //PICK SPECIMEN
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: PICK SPECIMEN");

//                armState = ARM_STATE.NONE; //clear out state to avoid reprocessing
//                Actions.runBlocking(GetPickSpecimenActionSequence());
//                readyToPickUpSample = false;
//                readyToDropHighSample = false;

                robotHardware.operateClawServo(true);
                robotHardware.operateWristServo(WRIST_PICK_SPECIMEN);

                if (!robotHardware.isSlideMotorBusy() && robotHardware.getSlidePos() != SLIDE_POSITION_RESTING) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING PICK_SPECIMEN - SLIDE IS BUSY");
                    robotHardware.setSlidePosition(SLIDE_POSITION_RESTING);
                }

                if (!robotHardware.isSlideMotorBusy() || Math.abs(SLIDE_POSITION_RESTING - robotHardware.getSlidePos()) < 10) {    //move arm after moving the slide
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_PICK_SPECIMEN, CLAW_ARM_VELOCITY);
                    armState = ARM_STATE.NONE;
                    Log.i("=== INCREDIBOTS ===", "PROCESSING PICK_SPECIMEN - SLIDE IS FINISHED MOVING - STARTING ARM. ARM_STATE: " + armState);
                }
                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;

            case ARM_VERTICAL:
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: ARM VERTICAL");

//                armState = ARM_STATE.NONE; //clear out state to avoid reprocessing
//                Actions.runBlocking(GetArmVerticalActionSequence());

                if (!robotHardware.isSlideMotorBusy() && robotHardware.getSlidePos() != SLIDE_POSITION_RESTING) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING ARM VERTICAL - SLIDE IS BUSY");
                    robotHardware.setSlidePosition(SLIDE_POSITION_RESTING);
                }

                if (!robotHardware.isSlideMotorBusy() || Math.abs(SLIDE_POSITION_RESTING - robotHardware.getSlidePos()) < 10) {    //move arm after moving the slide
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_HANG_SPECIMEN, CLAW_ARM_VELOCITY);
                    robotHardware.operateWristServo(WRIST_PRELOAD);
                    armState = ARM_STATE.NONE; //clear out state to avoid reprocessing
                    Log.i("=== INCREDIBOTS ===", "PROCESSING ARM VERTICAL - SLIDE IS FINISHED MOVING. STARTING ARM. ARM_STATE: " + armState);
                }

                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;

            case HANG_SPECIMEN:   //HANG SPECIMEN - HIGH RUNG
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: HANG SPECIMEN");

//                armState = ARM_STATE.NONE; //clear out state to avoid reprocessing
//                Actions.runBlocking(GetHangSpecimenActionSequence());

                robotHardware.operateWristServo(WRIST_HANG_SPECIMEN);
                robotHardware.operateClawServo(false);

                if (robotHardware.getClawArmMotorPos() != CLAW_ARM_HANG_SPECIMEN) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING HANG SPECIMEN - ARM IS BUSY");
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_HANG_SPECIMEN, CLAW_ARM_VELOCITY);
                }

                if (Math.abs(CLAW_ARM_HANG_SPECIMEN - robotHardware.getClawArmMotorPos()) < 10) {
                    robotHardware.setSlidePosition(SLIDE_POSITION_HANG_SPECIMEN);
                    armState = ARM_STATE.NONE; //clear out state to avoid reprocessing

                    Log.i("=== INCREDIBOTS ===", "PROCESSING HANG SPECIMEN - ARM IS FINISHED MOVING. STARTING SLIDE. ARM_STATE: " + armState);
                }

                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;

            case SNAP_SPECIMEN:   //SNAP SPECIMEN
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: SNAP SPECIMEN");

                armState = ARM_STATE.NONE; //clear out state to avoid reprocessing
                Actions.runBlocking(GetSnapSpecimenActionSequence());

//                if (robotHardware.getSlidePos() != SLIDE_POSITION_SNAP_SPECIMEN) {
//                    robotHardware.setSlidePosition(SLIDE_POSITION_SNAP_SPECIMEN);
//                     Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: SNAP SPECIMEN - SLIDE IS BUSY");
//                }
//
//                if (!robotHardware.isSlideMotorBusy() || Math.abs(SLIDE_POSITION_SNAP_SPECIMEN - robotHardware.getSlidePos()) < 10) {    //move arm after moving the slide
//                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_SNAP_SPECIMEN, CLAW_ARM_VELOCITY);
//                    robotHardware.operateWristServo(WRIST_SNAP_SPECIMEN);
//                    armState = ARM_STATE.NONE; //clear out state to avoid reprocessing
//                    Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: SNAP SPECIMEN - SLIDE IS FINISHED MOVING. STARTING ARM. ARM STATE: " + armState);
//                }
//
                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;

            case PICK_SAMPLE:   //PICK SAMPLE

                if (!processedPickSample) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: PICK SAMPLE");
                    //Actions.runBlocking(GetPickSampleActionSequence());

                    if (!robotHardware.isSlideMotorBusy() && robotHardware.getSlidePos() != SLIDE_POSITION_PICK_SAMPLE) {
                        robotHardware.setSlidePosition(SLIDE_POSITION_PICK_SAMPLE);
                        Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: PICK SAMPLE - SLIDE IS BUSY");
                    }

                    if (!robotHardware.isSlideMotorBusy() || Math.abs(SLIDE_POSITION_PICK_SAMPLE - robotHardware.getSlidePos()) < 10) {    //move arm after moving the slide
                        robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_PICK_SAMPLE, CLAW_ARM_VELOCITY);
                        robotHardware.operateWristServo(WRIST_PICK_SAMPLE);
                        robotHardware.operateClawServo(true);
                        Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: PICK SAMPLE - SLIDE IS FINISHED MOVING. STARTING ARM");
                    }

                    readyToPickUpSample = true;
                    processedPickSample = true;
                }
                // WE DONT CLEAR THE ARM STATE HERE SINCE ITS USED TO ADJUST THE ARM BASED ON THE SLIDE POSITION
                readyToDropHighSample = false;
                break;

            case HIGH_BASKET:   //HIGH BASKET
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: HIGH BASKET");

//                armState = ARM_STATE.NONE; //clear out state to avoid reprocessing
//                Actions.runBlocking(GetHighBasketActionSequence());
//                readyToDropHighSample = true;
//                readyToPickUpSample = false;

                if (!robotHardware.isClawArmMotorBusy() && robotHardware.getClawArmMotorPos() != CLAW_ARM_DROP_SAMPLE_HIGH) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING HIGH BASKET - CLAW ARM IS BUSY");
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_DROP_SAMPLE_HIGH, CLAW_ARM_VELOCITY);
                    robotHardware.operateWristServo(WRIST_DROP_SAMPLE);
                }

                if (Math.abs(CLAW_ARM_DROP_SAMPLE_HIGH - robotHardware.getClawArmMotorPos()) < 10) { //move slide after claw arm is done moving
                    robotHardware.setSlidePosition(SLIDE_POSITION_HIGH_BASKET);
                    armState = ARM_STATE.NONE;
                    readyToDropHighSample = true;
                    Log.i("=== INCREDIBOTS ===", "PROCESSING HIGH BASKET - CLAW ARM IS DONE MOVING - STARTING SLIDE. ARM_STATE: " + armState);
                }

                readyToPickUpSample = false;
                break;
            case LOW_BASKET:   //LOW BASKET
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: LOW BASKET");

//                armState = ARM_STATE.NONE; //clear out arm state to avoid reprocessing
//                Actions.runBlocking(GetLowBasketActionSequence());

                if (!robotHardware.isClawArmMotorBusy() && robotHardware.getClawArmMotorPos() != CLAW_ARM_DROP_SAMPLE_LOW) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING RT/B - CLAW ARM IS BUSY");
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_DROP_SAMPLE_LOW, CLAW_ARM_VELOCITY);
                }

                if (Math.abs(CLAW_ARM_DROP_SAMPLE_LOW - robotHardware.getClawArmMotorPos()) < 10) {  //move slide after getting arm in position
                    robotHardware.setSlidePosition(SLIDE_POSITION_LOW_BASKET);
                    robotHardware.operateWristServo(WRIST_DROP_SAMPLE);
                    armState = ARM_STATE.NONE;
                    Log.i("=== INCREDIBOTS ===", "PROCESSING RT/B CLAW ARM IS DONE MOVING - STARTING SLIDE. BUTTONSTATE: " + armState);
                }

                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;

            case ENTER_EXIT_SUB:   //ENTER /EXIT SUB
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: ENTER EXIT SUB");

                if (readyToPickUpSample) { //move arms in parallel after picking up sample
                    Log.i("=== INCREDIBOTS ===", "PROCESSING RT/X - MOVING SLIDE / ARM IN PARALLEL AFTER SAMPLE");
                    robotHardware.operateWristServo(WRIST_ENTER_SUB);
                    robotHardware.setSlidePosition(SLIDE_ENTER_SUB);
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_ENTER_SUB, CLAW_ARM_VELOCITY);
                    armState = ARM_STATE.NONE;
                }
                else {

//                    armState = ARM_STATE.NONE; //clear out arm state to avoid reprocessing
//                    Actions.runBlocking(GetEnterExitSubActionSequence());

                    // NOTE: CANNOT USE RR ACTIONS HERE - WE NEED TO WAIT FOR THE SLIDE TO COLLAPSE
                    // DURING THAT TIME, INPUTS TO ROBOT ARE NOT GETTING PROCESSED.
                    robotHardware.operateWristServo(WRIST_ENTER_SUB);

                    if (!robotHardware.isSlideMotorBusy() && robotHardware.getSlidePos() != SLIDE_ENTER_SUB) {
                        Log.i("=== INCREDIBOTS ===", "PROCESSING RT/X - SLIDE IS MOVING");
                        robotHardware.setSlidePosition(SLIDE_ENTER_SUB);
                    }

                    if (!robotHardware.isSlideMotorBusy() || Math.abs(SLIDE_ENTER_SUB - robotHardware.getSlidePos()) < 10) {    //move claw arm after slide is done moving
                        robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_ENTER_SUB, CLAW_ARM_VELOCITY);
                        armState = ARM_STATE.NONE;
                        Log.i("=== INCREDIBOTS ===", "PROCESSING RT/X - SLIDE IS DONE MOVING - STARTING ARM. BUTTONSTATE: " + armState);
                    }
                }
                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;

            case CLAW_ARM_AFTER_HIGH_SAMPLE:
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: CLAW ARM AFTER HIGH SAMPLE");

//                armState = ARM_STATE.ENTER_EXIT_SUB;  //clear out arm state to avoid reprocessing
//                Actions.runBlocking(GetClawArmAfterHighSampleActionSequence());


                if (!robotHardware.isClawArmMotorBusy() && robotHardware.getClawArmMotorPos() != CLAW_ARM_AFTER_DROP_SAMPLE_HIGH) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING CLAW ARM AFTER HIGH SAMPLE - CLAW ARM IS BUSY");
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_AFTER_DROP_SAMPLE_HIGH, CLAW_ARM_VELOCITY/2);
                }

                if (Math.abs(CLAW_ARM_AFTER_DROP_SAMPLE_HIGH - robotHardware.getClawArmMotorPos()) < 10) {
                    armState = ARM_STATE.ENTER_EXIT_SUB;
                }

                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;
        }
    }

    private void ProcessBumpers() {
        // if the right bumper is pressed it opens the claw
        if (gamepad2.right_bumper) {
            //telemetry.addLine("right bumper pressed");
            robotHardware.operateClawServo(true);

            if (readyToDropHighSample) {
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }

                //move the robot arm back
                armState = ARM_STATE.CLAW_ARM_AFTER_HIGH_SAMPLE;

                readyToDropHighSample = false;
            }
        }
        // if the left bumper is pressed it closes the claw
        else if (gamepad2.left_bumper) {
            //telemetry.addLine("left bumper pressed");
            robotHardware.operateClawServo(false);
        }
    }

    private void HandleManualOverride() {
        // if the back button is pressed it switches manual ovverides value
        if (gamepad2.left_stick_button && gamepad2.right_stick_button){
            MANUAL_OVERRIDE = !MANUAL_OVERRIDE;
            Log.i("=== INCREDIBOTS ===", "Manual Override: " + MANUAL_OVERRIDE);
        }


        // if manual override is true it will allow the joysticks to control the arms
        if (MANUAL_OVERRIDE) {

//            if (armState != ARM_STATE.NONE) { //stop any ongoing motions when entering manual override
//                robotHardware.stopClawArm();
//                robotHardware.stopSlide();
//                armState = ARM_STATE.NONE;
//            }

            //if manual override is enabled and slide position is more than limits, setit
            if (GetMaxSlidePosition() > 0 && robotHardware.getSlidePos() > GetMaxSlidePosition()) {
                Log.i("=== INCREDIBOTS ===", "Manual Override: Setting slide position based on arm angle");
                robotHardware.setSlidePosition(GetMaxSlidePosition());
            }

            float leftYSignal = gamepad2.left_stick_y * -1;

            // If the left joystick is greater than zero, it moves the left arm up
            if (leftYSignal > 0) {
                robotHardware.setClawArmPositionAndVelocity(robotHardware.getClawArmMotorPos() + MANUAL_OVERRIDE_ARM_POSITION_DELTA, CLAW_ARM_VELOCITY);
            }
            // If the left joystick is less than zero, it moves the left arm down
            else if (leftYSignal < 0){
                robotHardware.setClawArmPositionAndVelocity(robotHardware.getClawArmMotorPos() - MANUAL_OVERRIDE_ARM_POSITION_DELTA, CLAW_ARM_VELOCITY);
            }
        }
    }

    private int GetMaxSlidePosition()
    {
        //DEPENDING ON HOW THE CLAW ARM IS, THE SLIDE IS PERMITTED TO MOVE CERTAIN MAX DISTANCES.
        int maxSlidePosition = -1;

        if (robotHardware.getClawArmMotorPos() < CLAW_ARM_DROP_SAMPLE_HIGH) { //ARM IS BEHIND ROBOT
            maxSlidePosition = MAX_SLIDE_POSITION_ARM_BACKWARDS_HIGH;
        }
        else if (robotHardware.getClawArmMotorPos() > CLAW_ARM_DROP_SAMPLE_HIGH + 100) {
            maxSlidePosition = MAX_SLIDE_POSITION_ARM_FORWARDS_LOW;
        }

        return maxSlidePosition;
    }

    private void ProcessDPad() {

        //DEPENDING ON HOW THE CLAW ARM IS, THE SLIDE IS PERMITTED TO MOVE CERTAIN MAX DISTANCES.
         int maxSlidePosition = GetMaxSlidePosition();

        if (gamepad2.dpad_up){
            Log.i("=== INCREDIBOTS ===", "PROCESSING DPAD UP");

            //SLIDE CANNOT EXPAND BEYOND THE FAR POSITION FOR IT TO BE UNDER LIMITS
            if (maxSlidePosition < 0) { //no max applies
                robotHardware.setSlidePosition(robotHardware.getSlidePos() + MANUAL_OVERRIDE_SLIDE_POSITION_DELTA);
            }
            else {
                robotHardware.setSlidePosition(Math.min(robotHardware.getSlidePos() + MANUAL_OVERRIDE_SLIDE_POSITION_DELTA, maxSlidePosition));
            }
        }

        //process Dpad down input to retract linear slide
        if (gamepad2.dpad_down){
            Log.i("=== INCREDIBOTS ===", "PROCESSING DPAD DOWN");

            //SLIDE POSITION CANNOT BE LESS THAN 0
            robotHardware.setSlidePosition(Math.max(robotHardware.getSlidePos() - MANUAL_OVERRIDE_SLIDE_POSITION_DELTA, 0));
        }

        if (armState == ARM_STATE.PICK_SAMPLE) {

            // adjust arm position based on slide position
            // the arm needs to adjust 8 units more if the slide is fully extended
            int slideMinMaxDiff = MAX_SLIDE_POSITION_ARM_FORWARDS_LOW - SLIDE_POSITION_PICK_SAMPLE;
            int slideDelta = robotHardware.getSlidePos() -  SLIDE_POSITION_PICK_SAMPLE;
            double slideProportion = ((double)slideDelta / (double)slideMinMaxDiff);
            int newArmPosition = CLAW_ARM_PICK_SAMPLE + (int)Math.ceil(CLAW_ARM_ADJUSTMENT_WITH_SLIDE * slideProportion);

            if (Math.abs(robotHardware.getClawArmMotorPos() - newArmPosition) > 3 && !robotHardware.isClawArmMotorBusy()) {
                Log.i("=== INCREDIBOTS ===", "ADJUSTING ARM FOR SAMPLE PICK OLD:" + robotHardware.getClawArmMotorPos() + " NEW: " + newArmPosition);

                robotHardware.setClawArmPositionAndVelocity(newArmPosition, CLAW_ARM_VELOCITY);
            }
        }
    }

    public Action GetRestingActionSequence() {
        Action slideActionResting = new SlideMotionAsRRAction(robotHardware, SLIDE_POSITION_RESTING, true, false);
        Action armActionResting = new ArmMotionAsRRAction(robotHardware, CLAW_ARM_RESTING_BACK, CLAW_ARM_VELOCITY, true, false);
        Action clawActionResting = new ClawMotionAsRRAction(robotHardware, true, false, false);
        Action wristActionResting = new WristMotionAsRRAction(robotHardware, WRIST_PRELOAD, false, false);

        return new SequentialAction(
                slideActionResting,
                new ParallelAction(
                        clawActionResting,
                        wristActionResting),
                armActionResting);
    }

    public Action GetRobotHangActionSequence() {
        Action slideActionRobotHang = new SlideMotionAsRRAction(robotHardware, SLIDE_POSITION_RESTING, true);
        Action armActionRobotHang = new ArmMotionAsRRAction(robotHardware, CLAW_ARM_RESTING_BACK, CLAW_ARM_VELOCITY, false);
        Action clawActionRobotHang = new ClawMotionAsRRAction(robotHardware, false, false, false);
        Action wristActionRobotHang = new WristMotionAsRRAction(robotHardware, WRIST_PRELOAD, false, false);

        return new SequentialAction(
                slideActionRobotHang,
                new ParallelAction(
                        armActionRobotHang,
                        clawActionRobotHang,
                        wristActionRobotHang));
    }

    public Action GetPickSampleActionSequence() {
        Action slideActionPickSample = new SlideMotionAsRRAction(robotHardware, SLIDE_POSITION_PICK_SAMPLE, true, false);
        Action armActionPickSample = new ArmMotionAsRRAction(robotHardware, CLAW_ARM_PICK_SAMPLE, CLAW_ARM_VELOCITY, true);
        Action clawActionPickSample = new ClawMotionAsRRAction(robotHardware, true, false, false);
        Action wristActionPickSample = new WristMotionAsRRAction(robotHardware, WRIST_PICK_SAMPLE, false, false);

        return new SequentialAction(
                slideActionPickSample,
                new ParallelAction(
                        armActionPickSample,
                        clawActionPickSample,
                        wristActionPickSample));
    }

    public Action GetHangSpecimenActionSequence() {
        Action armActionHangSpecimen = new ArmMotionAsRRAction(robotHardware, CLAW_ARM_HANG_SPECIMEN, CLAW_ARM_VELOCITY, true, false);
        Action slideActionHangSpecimen = new SlideMotionAsRRAction(robotHardware, SLIDE_POSITION_HANG_SPECIMEN, false);
        Action wristActionHangSpecimen = new WristMotionAsRRAction(robotHardware, WRIST_HANG_SPECIMEN, false, false);
        Action clawActionHangSpecimen = new ClawMotionAsRRAction(robotHardware, false, false, false);

        return new SequentialAction(
                        new ParallelAction(
                                wristActionHangSpecimen,
                                clawActionHangSpecimen,
                                armActionHangSpecimen
                        ),
                        slideActionHangSpecimen);
    }

    public Action GetArmVerticalActionSequence() {
        Action armActionVertical = new ArmMotionAsRRAction(robotHardware, CLAW_ARM_HANG_SPECIMEN, CLAW_ARM_VELOCITY, false);
        Action slideActionVertical = new SlideMotionAsRRAction(robotHardware, SLIDE_POSITION_RESTING, true);
        Action wristActionVertical = new WristMotionAsRRAction(robotHardware, WRIST_PRELOAD, false, false);

        return new SequentialAction(
                slideActionVertical,
                new ParallelAction(
                        armActionVertical,
                        wristActionVertical
                ));
    }

    public Action GetSnapSpecimenActionSequence() {
        Action slideActionSnapSpecimen = new SlideMotionAsRRAction(robotHardware, SLIDE_POSITION_SNAP_SPECIMEN, true);
        Action armActionSnapSpecimen = new ArmMotionAsRRAction(robotHardware, CLAW_ARM_SNAP_SPECIMEN, CLAW_ARM_VELOCITY, true);
        Action wristActionSnapSpecimen = new WristMotionAsRRAction(robotHardware, WRIST_SNAP_SPECIMEN, false, false);
        Action clawActionSnapSpecimen = new ClawMotionAsRRAction(robotHardware, true, false, false);

        return new SequentialAction(
                slideActionSnapSpecimen,
                armActionSnapSpecimen,
                new ParallelAction(
                        clawActionSnapSpecimen,
                        wristActionSnapSpecimen)
        );
    }

    public Action GetLowBasketActionSequence() {
        Action armActionLowBasket = new ArmMotionAsRRAction(robotHardware, CLAW_ARM_DROP_SAMPLE_LOW, CLAW_ARM_VELOCITY, true, false);
        Action slideActionLowBasket = new SlideMotionAsRRAction(robotHardware, SLIDE_POSITION_LOW_BASKET, false);
        Action wristActionLowBasket = new WristMotionAsRRAction(robotHardware, WRIST_DROP_SAMPLE, false, false);

        return new SequentialAction(
                armActionLowBasket,
                new ParallelAction(
                        slideActionLowBasket,
                        wristActionLowBasket
                ));
    }

    public Action GetHighBasketActionSequence(){
        Action armActionHighBasket = new ArmMotionAsRRAction(robotHardware, CLAW_ARM_DROP_SAMPLE_HIGH, CLAW_ARM_VELOCITY, true, false);
        Action wristActionHighBasket = new WristMotionAsRRAction(robotHardware, WRIST_DROP_SAMPLE, false, false);
        Action slideActionHighBasket = new SlideMotionAsRRAction(robotHardware, SLIDE_POSITION_HIGH_BASKET, false);

        return new SequentialAction(
                armActionHighBasket,
                new ParallelAction(
                        wristActionHighBasket,
                        slideActionHighBasket
                ));
    }

    public Action GetHighBasketActionSequenceForAuto(){
        Action armActionHighBasket = new ArmMotionAsRRAction(robotHardware, CLAW_ARM_DROP_SAMPLE_HIGH, CLAW_ARM_VELOCITY, true, true);
        Action wristActionHighBasket = new WristMotionAsRRAction(robotHardware, WRIST_DROP_SAMPLE, false, false);
        Action slideActionHighBasket = new SlideMotionAsRRAction(robotHardware, SLIDE_POSITION_HIGH_BASKET, true, false);

        return new SequentialAction(
                armActionHighBasket,
                slideActionHighBasket,
                wristActionHighBasket
            );
    }

    public Action GetPickSpecimenActionSequence() {
        Action clawActionPickSpecimen = new ClawMotionAsRRAction(robotHardware, true, false, false);
        Action wristActionPickSpecimen = new WristMotionAsRRAction(robotHardware, WRIST_PICK_SPECIMEN, false, false);
        Action slideActionPickSpecimen = new SlideMotionAsRRAction(robotHardware, SLIDE_POSITION_RESTING, true);
        Action armActionPickSpecimen = new ArmMotionAsRRAction(robotHardware, CLAW_ARM_PICK_SPECIMEN, CLAW_ARM_VELOCITY, false);

       return new SequentialAction(
                slideActionPickSpecimen,
                new ParallelAction(
                        armActionPickSpecimen,
                        clawActionPickSpecimen,
                        wristActionPickSpecimen
                ));
    }

    public Action GetEnterExitSubActionSequence() {
        Action wristActionEnterExitSub = new WristMotionAsRRAction(robotHardware, WRIST_ENTER_SUB, false, false);
        Action slideActionEnterExitSub = new SlideMotionAsRRAction(robotHardware, SLIDE_ENTER_SUB, true, false);
        Action armActionEnterExitSub = new ArmMotionAsRRAction(robotHardware, CLAW_ARM_ENTER_SUB, CLAW_ARM_VELOCITY, false);

        armState = ARM_STATE.NONE;

        return new SequentialAction(
                slideActionEnterExitSub,
                new ParallelAction(
                        armActionEnterExitSub,
                        wristActionEnterExitSub));
    }

    public Action GetClawArmAfterHighSampleActionSequence() {
        Action armActionAfterHighSample = new ArmMotionAsRRAction(robotHardware, CLAW_ARM_AFTER_DROP_SAMPLE_HIGH, CLAW_ARM_VELOCITY/2, true);

        return new SequentialAction(armActionAfterHighSample);
    }
}