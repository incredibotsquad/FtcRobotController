package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner.ArmMotionAsRRAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner.ClawMotionAsRRAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner.SlideMotionAsRRAction;
import org.firstinspires.ftc.teamcode.drive.opmode.auto.roadrunner.WristMotionAsRRAction;

import java.lang.Math;

@Config
public class IncredibotsArmControl
{
    RobotConstants.GAME_COLORS gameColor;

    private Gamepad gamepad2;
    private RobotHardware robotHardware;

    // CLAW ARM CONSTANTS
    public static int CLAW_ARM_RESTING_BACK = 0;
    public static int CLAW_ARM_VELOCITY = 3000;
    public static int CLAW_ARM_AUTO_VELOCITY_SNAP_SPECIMEN = 1400;

    //CLAW CONSTANTS
    public static double CLAW_OPEN_POSITION = 0.65;
    public static double CLAW_CLOSE_POSITION = 0.4;

    //WRIST CONSTANTS
    public static double WRIST_PRELOAD_RESTING = 0.1;

    //SLIDE CONSTANTS
    public static int SLIDE_POSITION_RESTING = 0;
    public static int SLIDE_VELOCITY_EXPANDING = 3000;
    public static int SLIDE_VELOCITY_CONTRACTING = SLIDE_VELOCITY_EXPANDING;
    public static int MAX_SLIDE_POSITION_ARM_FORWARDS_LOW = 1250;
    public static int MAX_SLIDE_POSITION_ARM_BACKWARDS_HIGH = 0;

    //MANUAL OVERRIDE CONSTANTS
    private boolean MANUAL_OVERRIDE = true;
    public static int MANUAL_OVERRIDE_ARM_POSITION_DELTA = 100;
    public static int MANUAL_OVERRIDE_SLIDE_POSITION_DELTA = 200;
    public static int ARM_DELTA_WITH_SLIDE_MOTION = 120;

    //PICKING SAMPLES: RT/A
    public static int PICK_SAMPLE_ARM = 3200; // 1350
    public static int PICK_SAMPLE_SLIDE = 0;
    public static double PICK_SAMPLE_WRIST = 0.1;

    //ENTER EXIT SUB: RT/X
    public static int ENTER_SUB_ARM = 3000;
    public static int ENTER_SUB_SLIDE = 0;
    public static double ENTER_SUB_WRIST = 0.1;

    //DROP SAMPLE HIGH
    public static int DROP_SAMPLE_HIGH_ARM = 1900;
    public static int DROP_SAMPLE_HIGH_ARM_AFTER_DROP = 720;
    public static int DROP_SAMPLE_HIGH_SLIDE = 2100;
    public static double DROP_SAMPLE_HIGH_WRIST = 0.7;
    public static double DROP_SAMPLE_HIGH_OVER_BASKET_WRIST = 1;

    //DROP SAMPLE LOW
    public static int DROP_SAMPLE_LOW_ARM = 1800;
    public static double DROP_SAMPLE_LOW_WRIST = 0.6;
    public static int DROP_SAMPLE_LOW_SLIDE = 600;

    //PICK SPECIMEN
    public static int PICK_SPECIMEN_ARM = 0;
    public static double PICK_SPECIMEN_WRIST = 0.25;

    //HANG SPECIMEN
    public static int HANG_SPECIMEN_ARM = 1600;
    public static int HANG_SPECIMEN_SLIDE = 150;
    public static double HANG_SPECIMEN_WRIST = 0.1;

    //SNAP SPECIMEN
    public static int SNAP_SPECIMEN_ARM = 1300;
    public static int SNAP_SPECIMEN_SLIDE = 325;
    public static double SNAP_SPECIMEN_WRIST = 0.1;

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
        RESET_ENCODERS
    }

    private ARM_STATE armState;
    private boolean readyToDropHighSample = false;

    public IncredibotsArmControl(Gamepad gamepad, RobotHardware robotHardware) {
        gamepad2 = gamepad;
        this.robotHardware = robotHardware;
        armState = ARM_STATE.NONE;
    }

    public void ProcessInputs(Telemetry telemetry) {

        CreateStateFromButtonPress();

        ProcessState();

        ProcessBumpers();

        ProcessDPad();

        HandleManualOverride();

        HandleColorDetection();

        HandleMotorCurrentProblems();
    }

    public void setGameColor(RobotConstants.GAME_COLORS gameColor) {
        this.gameColor = gameColor;
    }

    //Function to create a state from Gamepad inputs
    private void CreateStateFromButtonPress() {

        ARM_STATE prevArmState = armState;

        //Back button maps to resting
        if (gamepad2.back) {
            armState = ARM_STATE.RESTING;
        }

        //Back + Start to reset the slide encoder
        if (gamepad2.back && gamepad2.start) {
            armState = ARM_STATE.RESET_ENCODERS;
        }

        //Hang the robot with start + left/right triggers
        if (gamepad2.start && gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
            armState = ARM_STATE.ROBOT_HANG;
        }

        if (gamepad2.a){
            //left trigger + A to pick specimen
            if (gamepad2.left_trigger > 0) {
                armState = ARM_STATE.PICK_SPECIMEN;
            }
            //right trigger + A to pick samples
            else if (gamepad2.right_trigger > 0) {
                armState = ARM_STATE.PICK_SAMPLE;
            }
        }

        if (gamepad2.b) {
            //left trigger + B to snap specimen
            if (gamepad2.left_trigger > 0) {
                armState = ARM_STATE.SNAP_SPECIMEN;
            }
            // right trigger + B for low basket
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
    private void ProcessState() {

        switch (armState) {
            case RESET_ENCODERS:
                Log.i("=== INCREDIBOTS ===", "PROCESSING RESET_SLIDE_ENCODER");

                robotHardware.stopAndResetSlideEncoder();
                robotHardware.stopAndResetArmEncoder();
                armState = ARM_STATE.NONE;
                readyToDropHighSample = false;
                break;

            case RESTING:
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: RESTING");

                robotHardware.operateWristServo(WRIST_PRELOAD_RESTING);
                robotHardware.stopIntake();

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

                readyToDropHighSample = false;
                break;

            case ROBOT_HANG: //SLIDE CLOSED, ARM RESTING
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: ROBOT HANG");

                if (!robotHardware.isSlideMotorBusy() && robotHardware.getSlidePos() != SLIDE_POSITION_RESTING) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING ROBOT HANG - SLIDE IS BUSY");
                    robotHardware.setSlidePosition(SLIDE_POSITION_RESTING);
                }

                //has to be a separate check since the previous action will make the slide motor busy
                if (!robotHardware.isSlideMotorBusy() || Math.abs(SLIDE_POSITION_RESTING - robotHardware.getSlidePos()) < 10) {
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_RESTING_BACK, CLAW_ARM_VELOCITY);
                    robotHardware.operateClawServo(false);
                    robotHardware.operateWristServo(ENTER_SUB_WRIST);
                    armState = ARM_STATE.NONE;
                    Log.i("=== INCREDIBOTS ===", "PROCESSING ROBOT HANG - SLIDE FINISHED MOVING - STARTING ARM. ARM_STATE: "+ armState);
                }

                readyToDropHighSample = false;
                break;

            case ARM_VERTICAL:
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: ARM VERTICAL");

                if (!robotHardware.isSlideMotorBusy() && robotHardware.getSlidePos() != SLIDE_POSITION_RESTING) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING ARM VERTICAL - SLIDE IS BUSY");
                    robotHardware.setSlidePosition(SLIDE_POSITION_RESTING);
                }

                if (!robotHardware.isSlideMotorBusy() || Math.abs(SLIDE_POSITION_RESTING - robotHardware.getSlidePos()) < 10) {    //move arm after moving the slide
                    robotHardware.setClawArmPositionAndVelocity(HANG_SPECIMEN_ARM, CLAW_ARM_VELOCITY);
                    armState = ARM_STATE.NONE; //clear out state to avoid reprocessing
                    Log.i("=== INCREDIBOTS ===", "PROCESSING ARM VERTICAL - SLIDE IS FINISHED MOVING. STARTING ARM. ARM_STATE: " + armState);
                }

                readyToDropHighSample = false;
                break;

            case PICK_SPECIMEN:   //PICK SPECIMEN
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: PICK SPECIMEN");

                robotHardware.operateClawServo(true);
                robotHardware.operateWristServo(PICK_SPECIMEN_WRIST);

                if (!robotHardware.isSlideMotorBusy() && robotHardware.getSlidePos() != SLIDE_POSITION_RESTING) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING PICK_SPECIMEN - SLIDE IS BUSY");
                    robotHardware.setSlidePosition(SLIDE_POSITION_RESTING);
                }

                if (!robotHardware.isSlideMotorBusy() || Math.abs(SLIDE_POSITION_RESTING - robotHardware.getSlidePos()) < 10) {    //move arm after moving the slide
                    robotHardware.setClawArmPositionAndVelocity(PICK_SPECIMEN_ARM, CLAW_ARM_VELOCITY);
                    armState = ARM_STATE.NONE;
                    Log.i("=== INCREDIBOTS ===", "PROCESSING PICK_SPECIMEN - SLIDE IS FINISHED MOVING - STARTING ARM. ARM_STATE: " + armState);
                }

                readyToDropHighSample = false;
                break;

            case HANG_SPECIMEN:   //HANG SPECIMEN - HIGH RUNG
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: HANG SPECIMEN");

                robotHardware.operateWristServo(HANG_SPECIMEN_WRIST);
                robotHardware.operateClawServo(false);

                if (robotHardware.getClawArmMotorPos() != HANG_SPECIMEN_ARM) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING HANG SPECIMEN - ARM IS BUSY");
                    robotHardware.setClawArmPositionAndVelocity(HANG_SPECIMEN_ARM, CLAW_ARM_VELOCITY);
                }

                if (Math.abs(HANG_SPECIMEN_ARM - robotHardware.getClawArmMotorPos()) < 10) {
                    robotHardware.setSlidePosition(HANG_SPECIMEN_SLIDE);
                    armState = ARM_STATE.NONE; //clear out state to avoid reprocessing
                    Log.i("=== INCREDIBOTS ===", "PROCESSING HANG SPECIMEN - ARM IS FINISHED MOVING. STARTING SLIDE. ARM_STATE: " + armState);
                }

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
                readyToDropHighSample = false;
                break;

            case PICK_SAMPLE:   //PICK SAMPLE
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: PICK SAMPLE");

                // Maintain the slide position and the wrist position - this only moves the arm down
                robotHardware.setClawArmPositionAndVelocity(PICK_SAMPLE_ARM, CLAW_ARM_VELOCITY / 3);

                armState = ARM_STATE.NONE;
                readyToDropHighSample = false;
                break;

            case HIGH_BASKET:   //HIGH BASKET
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: HIGH BASKET");

                if (!robotHardware.isClawArmMotorBusy() && robotHardware.getClawArmMotorPos() != DROP_SAMPLE_HIGH_ARM) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING HIGH BASKET - CLAW ARM IS BUSY");
                    robotHardware.setClawArmPositionAndVelocity(DROP_SAMPLE_HIGH_ARM, CLAW_ARM_VELOCITY);
                    robotHardware.operateWristServo(DROP_SAMPLE_HIGH_WRIST);
                    robotHardware.stopIntake();
                }

                if (Math.abs(DROP_SAMPLE_HIGH_ARM - robotHardware.getClawArmMotorPos()) < 10) { //move slide after claw arm is done moving
                    robotHardware.setSlidePosition(DROP_SAMPLE_HIGH_SLIDE);
                    Log.i("=== INCREDIBOTS ===", "PROCESSING HIGH BASKET - CLAW ARM IS DONE MOVING - STARTING SLIDE. ARM_STATE: " + armState);
                }

                if (Math.abs(DROP_SAMPLE_HIGH_SLIDE - robotHardware.getSlidePos()) < 10) {    //move wrist after moving the slide
                    Log.i("=== INCREDIBOTS ===", "PROCESSING HIGH BASKET - ARM + SLIDE DONE MOVING - STARTING WRIST. ARM_STATE: " + armState);
                    robotHardware.operateWristServo(DROP_SAMPLE_HIGH_OVER_BASKET_WRIST);
                    armState = ARM_STATE.NONE;
                    readyToDropHighSample = true;
                }

                break;
            case LOW_BASKET:   //LOW BASKET
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: LOW BASKET");

                if (!robotHardware.isClawArmMotorBusy() && robotHardware.getClawArmMotorPos() != DROP_SAMPLE_LOW_ARM) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING RT/B - CLAW ARM IS BUSY");
                    robotHardware.setClawArmPositionAndVelocity(DROP_SAMPLE_LOW_ARM, CLAW_ARM_VELOCITY);
                    robotHardware.operateWristServo(DROP_SAMPLE_LOW_WRIST);
                    robotHardware.stopIntake();
                }

                if (Math.abs(DROP_SAMPLE_LOW_ARM - robotHardware.getClawArmMotorPos()) < 10) {  //move slide after getting arm in position
                    robotHardware.setSlidePosition(DROP_SAMPLE_LOW_SLIDE);
                    armState = ARM_STATE.NONE;
                    Log.i("=== INCREDIBOTS ===", "PROCESSING RT/B CLAW ARM IS DONE MOVING - STARTING SLIDE. BUTTONSTATE: " + armState);
                }

                readyToDropHighSample = false;
                break;

            case ENTER_EXIT_SUB:   //ENTER /EXIT SUB
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: ENTER EXIT SUB." );

                //this mode will flip flop between entering and exiting sub
                //when entering, we need a particular sequence of things
                //when exiting, they can be in parallel.

                //can execute things in parallel if the arm is close to its target
                //this indicates the robot is in the position to pick samples from sub
                if (Math.abs(ENTER_SUB_ARM - robotHardware.getClawArmMotorPos()) < 400) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING RT/X - MOVING SLIDE / ARM IN PARALLEL AFTER SAMPLE");
                    robotHardware.operateWristServo(ENTER_SUB_WRIST);
                    robotHardware.setClawArmPositionAndVelocity(ENTER_SUB_ARM, CLAW_ARM_VELOCITY);
                    robotHardware.setSlidePosition(ENTER_SUB_SLIDE);
                    armState = ARM_STATE.NONE;
                }
                else {
                    robotHardware.operateWristServo(ENTER_SUB_WRIST);
                    robotHardware.operateClawServo(false);

                    if (!robotHardware.isSlideMotorBusy() && robotHardware.getSlidePos() != ENTER_SUB_SLIDE) {
                        Log.i("=== INCREDIBOTS ===", "PROCESSING ENTER EXIT SUB - SLIDE IS MOVING");
                        robotHardware.setSlidePosition(ENTER_SUB_SLIDE);
                    }

                    if (!robotHardware.isSlideMotorBusy() || Math.abs(ENTER_SUB_SLIDE - robotHardware.getSlidePos()) < 10) {    //move claw arm after slide is done moving
                        robotHardware.setClawArmPositionAndVelocity(ENTER_SUB_ARM, CLAW_ARM_VELOCITY);
                        robotHardware.operateIntake(true);
                        armState = ARM_STATE.NONE;
                        Log.i("=== INCREDIBOTS ===", "PROCESSING  ENTER EXIT SUB - SLIDE IS DONE MOVING - STARTING ARM. BUTTONSTATE: " + armState);
                    }
                }

                readyToDropHighSample = false;
                break;

            case CLAW_ARM_AFTER_HIGH_SAMPLE:
                Log.i("=== INCREDIBOTS ===", "PROCESSING STATE: CLAW ARM AFTER HIGH SAMPLE");

                robotHardware.operateWristServo(DROP_SAMPLE_HIGH_WRIST);

                if (!robotHardware.isSlideMotorBusy() && robotHardware.getSlidePos() != ENTER_SUB_SLIDE) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING CLAW_ARM_AFTER_HIGH_SAMPLE - SLIDE IS MOVING");
                    robotHardware.setSlidePosition(ENTER_SUB_SLIDE);
                }

                if (!robotHardware.isSlideMotorBusy() || Math.abs(ENTER_SUB_SLIDE - robotHardware.getSlidePos()) < 10) {    //move claw arm after slide is done moving
                    armState = ARM_STATE.ENTER_EXIT_SUB;
                    Log.i("=== INCREDIBOTS ===", "PROCESSING  CLAW_ARM_AFTER_HIGH_SAMPLE - SLIDE IS DONE MOVING - STARTING ARM. BUTTONSTATE: " + armState);
                }

                readyToDropHighSample = false;
                break;
        }
    }

    private void ProcessBumpers() {
        // if the right bumper is pressed it opens the claw
        if (gamepad2.right_bumper) {

            if (robotHardware.isIntakeOn()) {
                robotHardware.operateIntake(false);
            }

            if (readyToDropHighSample) {
                robotHardware.ejectSampleFromIntake();
                robotHardware.operateClawServo(true);

                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }

                //move the robot arm back
                armState = ARM_STATE.CLAW_ARM_AFTER_HIGH_SAMPLE;

                readyToDropHighSample = false;
            }
            else {
                robotHardware.operateClawServo(true);
            }

        }
        // if the left bumper is pressed it closes the claw
        else if (gamepad2.left_bumper) {
            //telemetry.addLine("left bumper pressed");
            robotHardware.operateClawServo(false);

            if (robotHardware.isIntakeOn()) {
                robotHardware.operateIntake(true);
            }

        }
    }

    private void HandleManualOverride() {
        // if the back button is pressed it switches manual ovverides value
        if (gamepad2.left_stick_button && gamepad2.right_stick_button){
            MANUAL_OVERRIDE = !MANUAL_OVERRIDE;
            Log.i("=== INCREDIBOTS ===", "Manual Override: " + MANUAL_OVERRIDE);
        }


        // if manual override is true it will allow the joysticks to control the arms
        // allow this only when robot has started (helpful in reset) or when picking samples
        if (MANUAL_OVERRIDE && (armState == ARM_STATE.PICK_SAMPLE || armState == ARM_STATE.NONE || armState == ARM_STATE.SNAP_SPECIMEN || armState == ARM_STATE.HANG_SPECIMEN)) {

            float leftYSignal = gamepad2.left_stick_y;

            // If the left joystick is greater than zero, it moves the left arm up
            if (leftYSignal > 0) {
                robotHardware.setClawArmPositionAndVelocity(robotHardware.getClawArmMotorPos() + MANUAL_OVERRIDE_ARM_POSITION_DELTA, CLAW_ARM_VELOCITY * 2);
            }

            // If the left joystick is less than zero, it moves the left arm down
            else if (leftYSignal < 0){
                robotHardware.setClawArmPositionAndVelocity(robotHardware.getClawArmMotorPos() - MANUAL_OVERRIDE_ARM_POSITION_DELTA, CLAW_ARM_VELOCITY * 2);
            }
        }
    }

    private int GetMaxSlidePosition()
    {
        //DEPENDING ON HOW THE CLAW ARM IS, THE SLIDE IS PERMITTED TO MOVE CERTAIN MAX DISTANCES.
        int maxSlidePosition = -1;

        if (robotHardware.getClawArmMotorPos() < DROP_SAMPLE_HIGH_ARM - 100) { //ARM IS BEHIND ROBOT
            maxSlidePosition = MAX_SLIDE_POSITION_ARM_BACKWARDS_HIGH;
        }
        else if (robotHardware.getClawArmMotorPos() > DROP_SAMPLE_HIGH_ARM + 100) {
            maxSlidePosition = MAX_SLIDE_POSITION_ARM_FORWARDS_LOW;
        }

        return maxSlidePosition;
    }

    private void ProcessDPad() {

        //DEPENDING ON HOW THE CLAW ARM IS, THE SLIDE IS PERMITTED TO MOVE CERTAIN MAX DISTANCES.
        int maxSlidePosition = GetMaxSlidePosition();
        int oldSlidePos = robotHardware.getSlidePos();

        if (gamepad2.dpad_up){
            Log.i("=== INCREDIBOTS ===", "PROCESSING DPAD UP");

            //SLIDE CANNOT EXPAND BEYOND THE FAR POSITION FOR IT TO BE UNDER LIMITS
            if (maxSlidePosition < 0) { //no max applies
                robotHardware.setSlidePosition(robotHardware.getSlidePos() + MANUAL_OVERRIDE_SLIDE_POSITION_DELTA);
            }
            else {
                robotHardware.setSlidePosition(Math.min(robotHardware.getSlidePos() + MANUAL_OVERRIDE_SLIDE_POSITION_DELTA, maxSlidePosition));
            }

//            if (enableArmAdjustmentWithSlide) {
//
//                Log.i("=== INCREDIBOTS ===", "PROCESSING DPAD: ADJUSTING ARM POSITION WITH SLIDE POSITION");
//
//                if (robotHardware.getSlidePos() > oldSlidePos) {    //slide extended - lower arm
//                    robotHardware.setClawArmPositionAndVelocity(robotHardware.getClawArmMotorPos() + ARM_DELTA_WITH_SLIDE_MOTION, CLAW_ARM_VELOCITY / 4);
//                }
//            }
        }

        //process Dpad down input to retract linear slide
        if (gamepad2.dpad_down){
            Log.i("=== INCREDIBOTS ===", "PROCESSING DPAD DOWN");

            //SLIDE POSITION CANNOT BE LESS THAN 0
            // EXCEPT IF WE ARE DOING IT TO RESET THE SLIDE IN CASE OF AN ERROR
            // THAT IS WHEN THE ARM STATE WOULD BE NONE
            robotHardware.setSlidePosition(robotHardware.getSlidePos() - MANUAL_OVERRIDE_SLIDE_POSITION_DELTA);

//            if (enableArmAdjustmentWithSlide) {
//
//                Log.i("=== INCREDIBOTS ===", "PROCESSING DPAD: ADJUSTING ARM POSITION WITH SLIDE POSITION");
//
//                if (robotHardware.getSlidePos() < oldSlidePos) {    //slide extended - lower arm
//                    robotHardware.setClawArmPositionAndVelocity(robotHardware.getClawArmMotorPos() - ARM_DELTA_WITH_SLIDE_MOTION, CLAW_ARM_VELOCITY / 4);
//                }
//            }
        }
    }

    private void HandleColorDetection() {
        if (!robotHardware.isIntakeOn()) {
            return; //do nothing if intake was not operating
        }

        if (gameColor == RobotConstants.GAME_COLORS.BLUE && robotHardware.getDetectedColor() == RobotConstants.GAME_COLORS.RED) {
            while (robotHardware.getDetectedColor() == RobotConstants.GAME_COLORS.RED) {
                robotHardware.operateWristServo(0.35);
                robotHardware.operateIntake(false);
            }
            robotHardware.operateWristServo(ENTER_SUB_WRIST);
            robotHardware.operateIntake(true);
        }

        if (gameColor == RobotConstants.GAME_COLORS.RED && robotHardware.getDetectedColor() == RobotConstants.GAME_COLORS.BLUE) {
            while (robotHardware.getDetectedColor() == RobotConstants.GAME_COLORS.BLUE) {
                robotHardware.operateWristServo(0.35);
                robotHardware.operateIntake(false);
            }
            robotHardware.operateWristServo(ENTER_SUB_WRIST);
            robotHardware.operateIntake(true);
        }
    }

    private void HandleMotorCurrentProblems() {
//        if (((DcMotorEx) armMotor).isOverCurrent()){
//            telemetry.addLine("MOTOR EXCEEDED CURRENT LIMIT!");
//        }
    }

    public Action GetRestingActionSequence() {
        Action slideActionResting = new SlideMotionAsRRAction(robotHardware, SLIDE_POSITION_RESTING, true, false);
        Action armActionResting = new ArmMotionAsRRAction(robotHardware, CLAW_ARM_RESTING_BACK, CLAW_ARM_VELOCITY, true, false);
        Action clawActionResting = new ClawMotionAsRRAction(robotHardware, true, false, false);
        Action wristActionResting = new WristMotionAsRRAction(robotHardware, WRIST_PRELOAD_RESTING, false, false);

        return new SequentialAction(
                slideActionResting,
                new ParallelAction(
                        clawActionResting,
                        wristActionResting),
                armActionResting);
    }

    public Action GetRestingActionSequenceNoWait() {
        Action slideActionResting = new SlideMotionAsRRAction(robotHardware, SLIDE_POSITION_RESTING, false, false);
        Action armActionResting = new ArmMotionAsRRAction(robotHardware, CLAW_ARM_RESTING_BACK, CLAW_ARM_VELOCITY, false, false);
        Action clawActionResting = new ClawMotionAsRRAction(robotHardware, true, false, false);
        Action wristActionResting = new WristMotionAsRRAction(robotHardware, WRIST_PRELOAD_RESTING, false, false);

        return new ParallelAction(
                slideActionResting,
                clawActionResting,
                wristActionResting,
                armActionResting);
    }


    public Action GetRobotHangActionSequence() {
        Action slideActionRobotHang = new SlideMotionAsRRAction(robotHardware, SLIDE_POSITION_RESTING, true);
        Action armActionRobotHang = new ArmMotionAsRRAction(robotHardware, CLAW_ARM_RESTING_BACK, CLAW_ARM_VELOCITY, false);
        Action clawActionRobotHang = new ClawMotionAsRRAction(robotHardware, false, false, false);
        Action wristActionRobotHang = new WristMotionAsRRAction(robotHardware, WRIST_PRELOAD_RESTING, false, false);

        return new SequentialAction(
                slideActionRobotHang,
                new ParallelAction(
                        armActionRobotHang,
                        clawActionRobotHang,
                        wristActionRobotHang));
    }

    public Action GetPickSampleActionSequence() {
        Action slideActionPickSample = new SlideMotionAsRRAction(robotHardware, PICK_SAMPLE_SLIDE, true, false);
        Action armActionPickSample = new ArmMotionAsRRAction(robotHardware, PICK_SAMPLE_ARM, CLAW_ARM_VELOCITY, true, false);
        Action clawActionPickSample = new ClawMotionAsRRAction(robotHardware, true, false, false);
        Action wristActionPickSample = new WristMotionAsRRAction(robotHardware, PICK_SAMPLE_WRIST, false, false);

        return new SequentialAction(
                new ParallelAction(
                        wristActionPickSample,
                        clawActionPickSample
                ),
                slideActionPickSample,
                armActionPickSample
        );
    }

    public Action GetHangSpecimenActionSequence_Fast() {
        Action armActionHangSpecimen = new ArmMotionAsRRAction(robotHardware, HANG_SPECIMEN_ARM, CLAW_ARM_VELOCITY, true, false);
        Action slideActionHangSpecimen = new SlideMotionAsRRAction(robotHardware, HANG_SPECIMEN_SLIDE, false);
        Action wristActionHangSpecimen = new WristMotionAsRRAction(robotHardware, HANG_SPECIMEN_WRIST, false, false);
        Action clawActionHangSpecimen = new ClawMotionAsRRAction(robotHardware, false, false, false);

        return new ParallelAction(
                        wristActionHangSpecimen,
                        clawActionHangSpecimen,
                        armActionHangSpecimen,
                        slideActionHangSpecimen);
    }

    public Action GetHangSpecimenActionSequence() {
        Action armActionHangSpecimen = new ArmMotionAsRRAction(robotHardware, HANG_SPECIMEN_ARM, CLAW_ARM_VELOCITY, true, false);
        Action slideActionHangSpecimen = new SlideMotionAsRRAction(robotHardware, HANG_SPECIMEN_SLIDE, false);
        Action wristActionHangSpecimen = new WristMotionAsRRAction(robotHardware, HANG_SPECIMEN_WRIST, false, false);
        Action clawActionHangSpecimen = new ClawMotionAsRRAction(robotHardware, false, false, false);

        return new SequentialAction(
                        wristActionHangSpecimen,
                        new ParallelAction(
                                clawActionHangSpecimen,
                                armActionHangSpecimen
                        ),
                        slideActionHangSpecimen);
    }

    public Action GetArmVerticalActionSequence() {
        Action armActionVertical = new ArmMotionAsRRAction(robotHardware, HANG_SPECIMEN_ARM, CLAW_ARM_VELOCITY, false);
        Action slideActionVertical = new SlideMotionAsRRAction(robotHardware, SLIDE_POSITION_RESTING, true);
        Action wristActionVertical = new WristMotionAsRRAction(robotHardware, WRIST_PRELOAD_RESTING, false, false);

        return new SequentialAction(
                slideActionVertical,
                new ParallelAction(
                        armActionVertical,
                        wristActionVertical
                ));
    }

    public Action GetSnapSpecimenActionSequence() {
        Action slideActionSnapSpecimen = new SlideMotionAsRRAction(robotHardware, SNAP_SPECIMEN_SLIDE, true);
        Action armActionSnapSpecimen = new ArmMotionAsRRAction(robotHardware, SNAP_SPECIMEN_ARM, CLAW_ARM_VELOCITY, true);
        Action wristActionSnapSpecimen = new WristMotionAsRRAction(robotHardware, SNAP_SPECIMEN_WRIST, false, false);
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
        Action armActionLowBasket = new ArmMotionAsRRAction(robotHardware, DROP_SAMPLE_LOW_ARM, CLAW_ARM_VELOCITY, true, false);
        Action slideActionLowBasket = new SlideMotionAsRRAction(robotHardware, DROP_SAMPLE_LOW_SLIDE, false);
        Action wristActionLowBasket = new WristMotionAsRRAction(robotHardware, DROP_SAMPLE_HIGH_WRIST, false, false);

        return new SequentialAction(
                armActionLowBasket,
                new ParallelAction(
                        slideActionLowBasket,
                        wristActionLowBasket
                ));
    }

    public Action GetHighBasketActionSequence(){
        Action armActionHighBasket = new ArmMotionAsRRAction(robotHardware, DROP_SAMPLE_HIGH_ARM, CLAW_ARM_VELOCITY, true, false);
        Action wristActionHighBasket = new WristMotionAsRRAction(robotHardware, DROP_SAMPLE_HIGH_WRIST, true, false);
        Action slideActionHighBasket1 = new SlideMotionAsRRAction(robotHardware, DROP_SAMPLE_HIGH_SLIDE -500, true, false);
        Action slideActionHighBasket2 = new SlideMotionAsRRAction(robotHardware, DROP_SAMPLE_HIGH_SLIDE, true, false);
        Action clawActionHighBasket = new ClawMotionAsRRAction(robotHardware, false, true, true);

        return new SequentialAction(
                new ParallelAction(
                        clawActionHighBasket
                ),
                armActionHighBasket,
                slideActionHighBasket1,
                new ParallelAction(
                        slideActionHighBasket2,
                        wristActionHighBasket
                )
            );
    }

    public Action GetHighBasketActionSequenceForAuto(){
        Action armActionHighBasket = new ArmMotionAsRRAction(robotHardware, DROP_SAMPLE_HIGH_ARM, CLAW_ARM_VELOCITY, true, true);
        Action wristActionHighBasket = new WristMotionAsRRAction(robotHardware, DROP_SAMPLE_HIGH_WRIST, false, false);
        Action slideActionHighBasket = new SlideMotionAsRRAction(robotHardware, DROP_SAMPLE_HIGH_SLIDE, true, false);

        return new SequentialAction(
                armActionHighBasket,
                slideActionHighBasket,
                wristActionHighBasket
            );
    }

    public Action GetPickSpecimenActionSequence() {
        Action clawActionPickSpecimen = new ClawMotionAsRRAction(robotHardware, true, false, false);
        Action wristActionPickSpecimen = new WristMotionAsRRAction(robotHardware, PICK_SPECIMEN_WRIST, false, false);
        Action slideActionPickSpecimen = new SlideMotionAsRRAction(robotHardware, SLIDE_POSITION_RESTING, true);
        Action armActionPickSpecimen = new ArmMotionAsRRAction(robotHardware, PICK_SPECIMEN_ARM, CLAW_ARM_VELOCITY, false);

       return new SequentialAction(
                slideActionPickSpecimen,
                new ParallelAction(
                        armActionPickSpecimen,
                        clawActionPickSpecimen,
                        wristActionPickSpecimen
                ));
    }

    public Action GetEnterExitSubActionSequence() {
        Action wristActionEnterExitSub = new WristMotionAsRRAction(robotHardware, ENTER_SUB_WRIST, false, false);
        Action clawActionEnterExitSub = new ClawMotionAsRRAction(robotHardware, true, false, false);
        Action slideActionEnterExitSub = new SlideMotionAsRRAction(robotHardware, ENTER_SUB_SLIDE, true, false);
        Action armActionEnterExitSub = new ArmMotionAsRRAction(robotHardware, ENTER_SUB_ARM, CLAW_ARM_VELOCITY / 2, true, true);

        return new SequentialAction(
                slideActionEnterExitSub,
                new ParallelAction(
                        armActionEnterExitSub,
                        clawActionEnterExitSub,
                        wristActionEnterExitSub));
    }

    public Action GetClawArmAfterHighSampleActionSequence() {
        Action armActionAfterHighSample = new ArmMotionAsRRAction(robotHardware, DROP_SAMPLE_HIGH_ARM_AFTER_DROP, CLAW_ARM_VELOCITY/2, true, false);
        Action wristActionAfterHighSample = new WristMotionAsRRAction(robotHardware, PICK_SAMPLE_WRIST, true, true);
        Action clawActionAfterHighSample = new ClawMotionAsRRAction(robotHardware, true, false, false);
        Action slideActionAfterHighSample = new SlideMotionAsRRAction(robotHardware, MAX_SLIDE_POSITION_ARM_FORWARDS_LOW, true, false);

        return new SequentialAction(
                wristActionAfterHighSample,
                armActionAfterHighSample,
                slideActionAfterHighSample,
                clawActionAfterHighSample
        );
    }
}