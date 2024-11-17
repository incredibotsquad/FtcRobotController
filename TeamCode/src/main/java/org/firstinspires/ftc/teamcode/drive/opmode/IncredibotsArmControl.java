package org.firstinspires.ftc.teamcode.drive.opmode;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;

import java.lang.Math;

@Config
public class IncredibotsArmControl
{
    Gamepad gamepad2;
    RobotHardware robotHardware;

    // Claw Arm constants
    public static int CLAW_ARM_RESTING_BACK = 0;
    public static int CLAW_ARM_VELOCITY = 750;
    public static int CLAW_ARM_AUTO_VELOCITY_SNAP_SPECIMEN = 1400;

    public static int CLAW_ARM_PICK_SAMPLE_A = 1440;

    public static int CLAW_ARM_DROP_SAMPLE_HIGH = 847;
    public static int CLAW_ARM_DROP_SAMPLE_LOW = CLAW_ARM_DROP_SAMPLE_HIGH; //896

    public static int CLAW_ARM_PICK_SPECIMEN = 112;
    public static int CLAW_ARM_AFTER_DROP_SAMPLE_HIGH = 720;

    public static int CLAW_ARM_AUTO_HANG_SPECIMEN = 775;
    public static int CLAW_ARM_AUTO_SNAP_SPECIMEN = 620;

    public static int CLAW_ARM_TELE_HANG_SPECIMEN_HIGH_Y = CLAW_ARM_AUTO_HANG_SPECIMEN;
    public static int CLAW_ARM_TELE_SNAP_SPECIMEN_HIGH_B = CLAW_ARM_AUTO_SNAP_SPECIMEN;

    public static int CLAW_ARM_ENTER_SUB = 1350;

    //claw positions
    public static double CLAW_OPEN_POSITION = 0.55;
    public static double CLAW_CLOSE_POSITION = 0.42;

    //Slide movement position
    public static int SLIDE_POSITION_RESTING = 0;
    public static int SLIDE_VELOCITY_EXPANDING = 4000;
    public static int SLIDE_VELOCITY_CONTRACTING = 10000;

    public static int SLIDE_POSITION_HANG_SPECIMEN_HIGH = 870;

    public static int SLIDE_POSITION_HIGH_BASKET = 8100;
    public static int SLIDE_POSITION_LOW_BASKET = SLIDE_POSITION_RESTING; //1170

    //override...
    private boolean MANUAL_OVERRIDE = false;
    private static int MANUAL_OVERRIDE_ARM_POSITION_DELTA = 150;
    private static int MANUAL_OVERRIDE_SLIDE_POSITION_DELTA = 250;
    public static int MAX_SLIDE_POSITION_ARM_BACKWARDS_LOW = 0;
    public static int MAX_SLIDE_POSITION_ARM_FORWARDS_LOW = 3750;

    private enum BUTTON_STATE {
        NONE,
        BUTTON_BACK,
        BUTTON_HANG,
        BUTTON_LT_A,
        BUTTON_RT_A,
        BUTTON_LT_B,
        BUTTON_RT_B,
        BUTTON_LT_X,
        BUTTON_RT_X,
        BUTTON_LT_Y,
        BUTTON_RT_Y,
        CLAW_ARM_AFTER_HIGH_SAMPLE,
        RESET_SLIDE_ENCODER
    }

    private BUTTON_STATE buttonState;

    private boolean readyToDropHighSample = false;
    private boolean readyToPickUpSample = false;

    public IncredibotsArmControl(Gamepad gamepad, RobotHardware robotHardware) {
        gamepad2 = gamepad;
        this.robotHardware = robotHardware;
        buttonState = BUTTON_STATE.NONE;
    }

    public void ProcessInputs(Telemetry telemetry) {

        CreateStateFromButtonPress();
        ProcessButtons();

        ProcessBumpers();

        ProcessDPad();

        HandleManualOverride();
    }

    private void CreateStateFromButtonPress() {

        BUTTON_STATE prevButtonState = buttonState;

        if (gamepad2.back) {
            buttonState = BUTTON_STATE.BUTTON_BACK;
        }

        if (gamepad2.back && gamepad2.start) {
            buttonState = BUTTON_STATE.RESET_SLIDE_ENCODER;
        }

        if (gamepad2.start && gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
            buttonState = BUTTON_STATE.BUTTON_HANG;
        }

        if (gamepad2.a){
            if (gamepad2.left_trigger > 0) {
                buttonState = BUTTON_STATE.BUTTON_LT_A;
            }
            else if (gamepad2.right_trigger > 0) {
                buttonState = BUTTON_STATE.BUTTON_RT_A;
            }
        }

        if (gamepad2.b) {
            if (gamepad2.left_trigger > 0) {
                buttonState = BUTTON_STATE.BUTTON_LT_B;
            }
            else if (gamepad2.right_trigger > 0) {
                buttonState = BUTTON_STATE.BUTTON_RT_B;
            }
        }

        if (gamepad2.x) {
            if (gamepad2.left_trigger > 0) {
                buttonState = BUTTON_STATE.BUTTON_LT_X;
            }
            else if (gamepad2.right_trigger > 0) {
                buttonState = BUTTON_STATE.BUTTON_RT_X;
            }
        }

        if (gamepad2.y) {
            if (gamepad2.left_trigger > 0) {
                buttonState = BUTTON_STATE.BUTTON_LT_Y;
            }
            else if (gamepad2.right_trigger > 0){
                buttonState = BUTTON_STATE.BUTTON_RT_Y;
            }
        }

        // if previous state was not NONE (some motion was ongoing / pending)
        // and if new state is not NONE (some motion is going to happen)
        // and if previous and new are different
        // then cancel whatever was going on in the previous state and process the new one.
        // any pending actions for the previous state will not get executed since the state itself will change.
        if (prevButtonState != BUTTON_STATE.NONE && buttonState != BUTTON_STATE.NONE && prevButtonState != buttonState) {
            Log.i("=== INCREDIBOTS ===", "STOPPING ARMS SINCE BUTTON STATE CHANGE. OLD: " + prevButtonState + " NEW: " + buttonState);
            robotHardware.stopSlide();
            robotHardware.stopClawArm();
        }

        if (buttonState != prevButtonState) {
            Log.i("=== INCREDIBOTS ===", "NEW BUTTON STATE: " + buttonState);
        }
    }

    //functions to take gamepad inputs and turn it into movements
    private void ProcessButtons() {

        switch (buttonState) {
            case RESET_SLIDE_ENCODER:
                Log.i("=== INCREDIBOTS ===", "PROCESSING RESET_SLIDE_ENCODER");

                robotHardware.stopAndResetSlideEncoder();
                buttonState = BUTTON_STATE.NONE;
                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;
            case BUTTON_BACK:
                Log.i("=== INCREDIBOTS ===", "PROCESSING BACK BUTTON");

                if (robotHardware.getSlidePos() != SLIDE_POSITION_RESTING) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING BACK BUTTON - SLIDE IS BUSY");
                    robotHardware.setSlidePosition(SLIDE_POSITION_RESTING);
                }

                //has to be a separate check since the previous action will make the slide motor busy
                if (!robotHardware.isSlideMotorBusy() || Math.abs(SLIDE_POSITION_RESTING - robotHardware.getSlidePos()) < 10) {
                    robotHardware.stopSlide();  //this should alleviate issues where slide motor get hot
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_RESTING_BACK, CLAW_ARM_VELOCITY);
                    robotHardware.operateClawServo(false, CLAW_OPEN_POSITION, CLAW_CLOSE_POSITION);
                    buttonState = BUTTON_STATE.NONE;
                    Log.i("=== INCREDIBOTS ===", "PROCESSING BACK BUTTON - SLIDE FINISHED MOVING - STARTING ARM. BUTTONSTATE: "+ buttonState);
                }
                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;
            case BUTTON_HANG: //SLIDE CLOSED, ARM RESTING
                Log.i("=== INCREDIBOTS ===", "PROCESSING HANG BUTTON");

                if (robotHardware.getSlidePos() != SLIDE_POSITION_RESTING) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING BACK BUTTON - SLIDE IS BUSY");
                    robotHardware.setSlidePosition(SLIDE_POSITION_RESTING);
                }

                //has to be a separate check since the previous action will make the slide motor busy
                if (!robotHardware.isSlideMotorBusy() || Math.abs(SLIDE_POSITION_RESTING - robotHardware.getSlidePos()) < 10) {
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_RESTING_BACK, CLAW_ARM_VELOCITY);
                    robotHardware.operateClawServo(false, CLAW_OPEN_POSITION, CLAW_CLOSE_POSITION);
                    buttonState = BUTTON_STATE.NONE;
                    Log.i("=== INCREDIBOTS ===", "PROCESSING BACK BUTTON - SLIDE FINISHED MOVING - STARTING ARM. BUTTONSTATE: "+ buttonState);
                }
                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;
            case BUTTON_LT_A:   //PICK SPECIMEN
                Log.i("=== INCREDIBOTS ===", "PROCESSING LT/A");

                robotHardware.operateClawServo(true, CLAW_OPEN_POSITION, CLAW_CLOSE_POSITION);

                if (robotHardware.getSlidePos() != SLIDE_POSITION_RESTING) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING LT/A - SLIDE IS BUSY");
                    robotHardware.setSlidePosition(SLIDE_POSITION_RESTING);
                }

                if (!robotHardware.isSlideMotorBusy() || Math.abs(SLIDE_POSITION_RESTING - robotHardware.getSlidePos()) < 10) {    //move arm after moving the slide
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_PICK_SPECIMEN, CLAW_ARM_VELOCITY);
                    buttonState = BUTTON_STATE.NONE;
                    Log.i("=== INCREDIBOTS ===", "PROCESSING LT/A - SLIDE IS FINISHED MOVING - STARTING ARM. BUTTONSTAE: " + buttonState);
                }
                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;
            case BUTTON_LT_Y:   //HANG SPECIMEN - HIGH RUNG
                Log.i("=== INCREDIBOTS ===", "PROCESSING LT/Y");

                if (robotHardware.getClawArmMotorPos() != CLAW_ARM_TELE_HANG_SPECIMEN_HIGH_Y) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING LT/Y - CLAW ARM IS MOVING");
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_TELE_HANG_SPECIMEN_HIGH_Y, CLAW_ARM_VELOCITY);
                }

                if (!robotHardware.isClawArmMotorBusy() || Math.abs(CLAW_ARM_TELE_HANG_SPECIMEN_HIGH_Y - robotHardware.getClawArmMotorPos()) < 10) {  //set slide after claw arm
                    robotHardware.setSlidePosition(SLIDE_POSITION_HANG_SPECIMEN_HIGH);
                    robotHardware.operateClawServo(false, CLAW_OPEN_POSITION, CLAW_CLOSE_POSITION);
                    buttonState = BUTTON_STATE.NONE;
                    Log.i("=== INCREDIBOTS ===", "PROCESSING LT/Y - CLAW ARM IS DONE MOVING - STARTING SLIDE. BUTTONSTATE: " + buttonState);
                }
                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;
            case BUTTON_LT_B:   //SNAP SPECIMEN
                Log.i("=== INCREDIBOTS ===", "PROCESSING LT/B");

                if (robotHardware.getSlidePos() != SLIDE_POSITION_HANG_SPECIMEN_HIGH) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING LT/B - SLIDE IS BUSY");
                    robotHardware.setSlidePosition(SLIDE_POSITION_HANG_SPECIMEN_HIGH);
                }

                if (!robotHardware.isSlideMotorBusy() || Math.abs(SLIDE_POSITION_HANG_SPECIMEN_HIGH - robotHardware.getSlidePos()) < 10) {    //move claw arm after slide is done moving
                    Log.i("=== INCREDIBOTS ===", "PROCESSING LT/B - SLIDE IS DONE MOVING - STARTING ARM");
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_TELE_SNAP_SPECIMEN_HIGH_B, CLAW_ARM_VELOCITY);
                    buttonState = BUTTON_STATE.NONE;
                }

                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;
            case  BUTTON_LT_X: //ARM VERTICAL TO GET READY TO HANG SPECIMEN
                Log.i("=== INCREDIBOTS ===", "PROCESSING LT/X");

                robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_TELE_HANG_SPECIMEN_HIGH_Y, CLAW_ARM_VELOCITY);
                robotHardware.setSlidePosition(SLIDE_POSITION_RESTING);
                buttonState = BUTTON_STATE.NONE;

                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;
            case BUTTON_RT_A:   //PICK SAMPLE
                Log.i("=== INCREDIBOTS ===", "PROCESSING RT/A");

                if (robotHardware.getSlidePos() != SLIDE_POSITION_RESTING) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING RT/A - SLIDE IS MOVING");
                    robotHardware.setSlidePosition(SLIDE_POSITION_RESTING);
                }

                if (!robotHardware.isSlideMotorBusy() || Math.abs(SLIDE_POSITION_RESTING - robotHardware.getSlidePos()) < 10) {    //move claw arm after slide is done moving
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_PICK_SAMPLE_A, CLAW_ARM_VELOCITY);
                    robotHardware.operateClawServo(true, CLAW_OPEN_POSITION, CLAW_CLOSE_POSITION);
                    buttonState = BUTTON_STATE.NONE;
                    readyToPickUpSample = true;
                    Log.i("=== INCREDIBOTS ===", "PROCESSING RT/A - SLIDE IS DONE MOVING - STARTING ARM. BUTTON STATE: " + buttonState);
                }
                readyToDropHighSample = false;
                break;
            case BUTTON_RT_Y:   //HIGH BASKET
                Log.i("=== INCREDIBOTS ===", "PROCESSING RT/Y");

                if (robotHardware.getClawArmMotorPos() != CLAW_ARM_DROP_SAMPLE_HIGH) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING RT/Y - CLAW ARM IS BUSY");
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_DROP_SAMPLE_HIGH, CLAW_ARM_VELOCITY);
                }

                if (!robotHardware.isClawArmMotorBusy() || Math.abs(CLAW_ARM_DROP_SAMPLE_HIGH - robotHardware.getClawArmMotorPos()) < 10) { //move slide after claw arm is done moving
                    robotHardware.setSlidePosition(SLIDE_POSITION_HIGH_BASKET);
                    buttonState = BUTTON_STATE.NONE;
                    readyToDropHighSample = true;
                    Log.i("=== INCREDIBOTS ===", "PROCESSING RT/Y - CLAW ARM IS DONE MOVING - STARTING SLIDE. BUTTONSTATE: " + buttonState);
                }
                readyToPickUpSample = false;
                break;
            case BUTTON_RT_B:   //LOW BASKET
                Log.i("=== INCREDIBOTS ===", "PROCESSING RT/B");

                if (robotHardware.getClawArmMotorPos() != CLAW_ARM_DROP_SAMPLE_LOW) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING RT/B - CLAW ARM IS BUSY");
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_DROP_SAMPLE_LOW, CLAW_ARM_VELOCITY);
                }

                if (!robotHardware.isClawArmMotorBusy() || Math.abs(CLAW_ARM_DROP_SAMPLE_LOW - robotHardware.getClawArmMotorPos()) < 10) {  //move slide after getting arm in position
                    robotHardware.setSlidePosition(SLIDE_POSITION_LOW_BASKET);
                    buttonState = BUTTON_STATE.NONE;
                    Log.i("=== INCREDIBOTS ===", "PROCESSING RT/B CLAW ARM IS DONE MOVING - STARTING SLIDE. BUTTONSTATE: " + buttonState);
                }
                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;
            case BUTTON_RT_X:   //ENTER SUB
                Log.i("=== INCREDIBOTS ===", "PROCESSING RT/X");

                if (readyToPickUpSample) { //move arms in parallel after picking up sample
                    Log.i("=== INCREDIBOTS ===", "PROCESSING RT/X - MOVING SLIDE / ARM IN PARALLEL AFTER SAMPLE");
                    robotHardware.setSlidePosition(SLIDE_POSITION_RESTING);
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_ENTER_SUB, CLAW_ARM_VELOCITY);
                    buttonState = BUTTON_STATE.NONE;
                }
                else {
                    if (robotHardware.getSlidePos() != SLIDE_POSITION_RESTING) {
                        Log.i("=== INCREDIBOTS ===", "PROCESSING RT/X - SLIDE IS MOVING");
                        robotHardware.setSlidePosition(SLIDE_POSITION_RESTING);
                    }

                    if (!robotHardware.isSlideMotorBusy() || Math.abs(SLIDE_POSITION_RESTING - robotHardware.getSlidePos()) < 10) {    //move claw arm after slide is done moving
                        robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_ENTER_SUB, CLAW_ARM_VELOCITY);
                        buttonState = BUTTON_STATE.NONE;
                        Log.i("=== INCREDIBOTS ===", "PROCESSING RT/X - SLIDE IS DONE MOVING - STARTING ARM. BUTTONSTATE: " + buttonState);
                    }
                }
                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;

            case CLAW_ARM_AFTER_HIGH_SAMPLE:
                Log.i("=== INCREDIBOTS ===", "PROCESSING CLAW ARM AFTER HIGH SAMPLE");

                if (robotHardware.getClawArmMotorPos() != CLAW_ARM_AFTER_DROP_SAMPLE_HIGH) {
                    Log.i("=== INCREDIBOTS ===", "PROCESSING CLAW ARM AFTER HIGH SAMPLE - CLAW ARM IS BUSY");
                    robotHardware.setClawArmPositionAndVelocity(CLAW_ARM_AFTER_DROP_SAMPLE_HIGH, CLAW_ARM_VELOCITY);
                }

                if (!robotHardware.isClawArmMotorBusy() || Math.abs(CLAW_ARM_AFTER_DROP_SAMPLE_HIGH - robotHardware.getClawArmMotorPos()) < 10) {
                    buttonState = BUTTON_STATE.BUTTON_RT_X;
                }

//
//                if (!robotHardware.isClawArmMotorBusy() || Math.abs(CLAW_ARM_AFTER_DROP_SAMPLE_HIGH - robotHardware.getClawArmMotorPos()) < 10) {
//                    robotHardware.setSlidePosition(SLIDE_POSITION_RESTING);
//                    buttonState = BUTTON_STATE.NONE;
//                    Log.i("=== INCREDIBOTS ===", "PROCESSING CLAW ARM AFTER HIGH SAMPLE - CLAW ARM IS DONE MOVING - STARTING SLIDE. BUTTONSTATE: " + buttonState);
//                }
                readyToPickUpSample = false;
                readyToDropHighSample = false;
                break;
        }
    }

    private void ProcessBumpers() {
        // if the right bumper is pressed it opens the claw
        if (gamepad2.right_bumper) {
            //telemetry.addLine("right bumper pressed");
            robotHardware.operateClawServo(true, CLAW_OPEN_POSITION, CLAW_CLOSE_POSITION);

            if (readyToDropHighSample) {
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }

                //move the robot arm back
                buttonState = BUTTON_STATE.CLAW_ARM_AFTER_HIGH_SAMPLE;

                readyToDropHighSample = false;
            }
        }
        // if the left bumper is pressed it closes the claw
        else if (gamepad2.left_bumper) {
            //telemetry.addLine("left bumper pressed");
            robotHardware.operateClawServo(false, CLAW_OPEN_POSITION, CLAW_CLOSE_POSITION);
        }
    }

    private void HandleManualOverride() {
        // if the back button is pressed it switches manual ovverides value
        if (gamepad2.left_stick_button && gamepad2.right_stick_button){
            MANUAL_OVERRIDE = !MANUAL_OVERRIDE;
        }

        // if manual override is true it will allow the joysticks to control the arms
        if (MANUAL_OVERRIDE) {

            if (buttonState != BUTTON_STATE.NONE) { //stop any ongoing motions when entering manual override
                robotHardware.stopClawArm();
                robotHardware.stopSlide();
                buttonState = BUTTON_STATE.NONE;
            }

            float leftYSignal = gamepad2.left_stick_y * -1;

            // If the left joystick is greater than zero, it moves the left arm up
            if (leftYSignal > 0) {
                robotHardware.setClawArmPositionAndVelocity(robotHardware.getClawArmMotorPos() + MANUAL_OVERRIDE_ARM_POSITION_DELTA, leftYSignal * CLAW_ARM_VELOCITY);

            }
            // If the left joystick is less than zero, it moves the left arm down
            else if (leftYSignal < 0){
                robotHardware.setClawArmPositionAndVelocity(robotHardware.getClawArmMotorPos() - MANUAL_OVERRIDE_ARM_POSITION_DELTA, leftYSignal * CLAW_ARM_VELOCITY);
            }
        }
    }

    private void ProcessDPad() {

        //DEPENDING ON HOW THE CLAW ARM IS, THE SLIDE IS PERMITTED TO MOVE CERTAIN MAX DISTANCES.
         int maxSlidePosition = -1;

         if (robotHardware.getClawArmMotorPos() < CLAW_ARM_AUTO_HANG_SPECIMEN) { //ARM IS BEHIND ROBOT
             maxSlidePosition = MAX_SLIDE_POSITION_ARM_BACKWARDS_LOW;
         }
         else if (robotHardware.getClawArmMotorPos() > CLAW_ARM_DROP_SAMPLE_HIGH + 100) {
             maxSlidePosition = MAX_SLIDE_POSITION_ARM_FORWARDS_LOW;
         }

        if (gamepad2.dpad_up){
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
            //SLIDE POSITION CANNOT BE LESS THAN 0
            robotHardware.setSlidePosition(Math.max(robotHardware.getSlidePos() - MANUAL_OVERRIDE_SLIDE_POSITION_DELTA, 0));
        }
    }
}