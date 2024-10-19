package org.firstinspires.ftc.teamcode.drive.opmode;

import android.os.AsyncTask;

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

    //Left (Claw) Arm constants
    public static int CLAW_ARM_RESTING_BACK = 0;
    public static int CLAW_ARM_PICK_SPECIMEN_A = 4600;//4600;
    public static int CLAW_ARM_HANG_SPECIMEN_Y = 3550;
    public static int CLAW_ARM_SNAP_SPECIMEN_B = 4000;
    public static int CLAW_ARM_HORIZONTAL_X = 4200;
    public static int CLAW_ARM_HANG_START = 500;
    public static int CLAW_ARM_VELOCITY = 1500;

    //Right (Intake) Arm constants
    public static int INTAKE_ARM_RESTING_BACK = 0;
    public static int INTAKE_ARM_PICK_NEAR_SAMPLE_A = 5000; //4700
    public static int INTAKE_ARM_PICK_FAR_SAMPLE_A = 4605; //4700
    public static int INTAKE_ARM_DROP_SAMPLE_HIGH_Y = 2600;
    public static int INTAKE_ARM_DROP_SAMPLE_LOW_B = 2800; //4050
    public static int INTAKE_ARM_HORIZONTAL_X = 4000;
    public static int INTAKE_ARM_HANG_START = 300;
    public static int INTAKE_ARM_VELOCITY = 1500;
    private static boolean INTAKE_ARM_NEAR_SAMPLE_MODE = true;

    //claw positions
    public static double CLAW_OPEN_POSITION = 60.0/300.0;
    public static double CLAW_CLOSE_POSITION = 0.0;

    //Slide movement position
    public static int SLIDE_POSITION_TO_PICK_NEAR_SAMPLE = 800;
    public static int SLIDE_POSITION_TO_PICK_FAR_SAMPLE = 2893;
    public static int SLIDE_POSITION_TO_SCORE_HIGH = 4500;
    public static int SLIDE_POSITION_TO_SCORE_LOW = 0;
    public static int SLIDE_POSITION_RESTING = 0;
    public static int SLIDE_VELOCITY = 5000;

    //override...
    private boolean MANUAL_OVERRIDE = false;
    private static int MANUAL_OVERRIDE_DELTA = 150;

    public IncredibotsArmControl(Gamepad gamepad, RobotHardware robotHardware) {
        gamepad2 = gamepad;
        this.robotHardware = robotHardware;
    }

    public void ProcessInputs(Telemetry telemetry) {
        //left trigger + buttons controls left arm (claw)
        //right trigger + buttons controls right arm (intake)

        ProcessButtons();

        ProcessDPad(telemetry);

        ProcessBumpers(telemetry);

        HandleManualOverride();
    }
//functions to take gamepad inputs and turn it into movements
    private void ProcessButtons() {

        //toggle the arms to pick near samples or far samples
        if (gamepad2.back && gamepad2.start) {
            INTAKE_ARM_NEAR_SAMPLE_MODE = !INTAKE_ARM_NEAR_SAMPLE_MODE;

            if (INTAKE_ARM_NEAR_SAMPLE_MODE) {
                robotHardware.setSlidePositionAndVelocity(SLIDE_POSITION_TO_PICK_NEAR_SAMPLE, SLIDE_VELOCITY);
                robotHardware.setRightArmPositionAndVelocity(INTAKE_ARM_PICK_NEAR_SAMPLE_A, INTAKE_ARM_VELOCITY);
            } else {
                robotHardware.setSlidePositionAndVelocity(SLIDE_POSITION_TO_PICK_FAR_SAMPLE, SLIDE_VELOCITY);
                robotHardware.setRightArmPositionAndVelocity(INTAKE_ARM_PICK_FAR_SAMPLE_A, INTAKE_ARM_VELOCITY);
            }

            //start moving the intake servo for picking up samples
            robotHardware.operateIntakeServo(true, false);
        }

        //process back button
        if (gamepad2.back){
            if (gamepad2.left_trigger > 0) {
                robotHardware.operateClawServo(false, CLAW_OPEN_POSITION, CLAW_CLOSE_POSITION);
                robotHardware.setLeftArmPositionAndVelocity(CLAW_ARM_RESTING_BACK, CLAW_ARM_VELOCITY);
            }

            if (gamepad2.right_trigger > 0) {
                robotHardware.setSlidePositionAndVelocity(SLIDE_POSITION_RESTING, SLIDE_VELOCITY);
                robotHardware.setRightArmPositionAndVelocity(INTAKE_ARM_RESTING_BACK, INTAKE_ARM_VELOCITY-500);
            }
        }

        //process button A
        if (gamepad2.a){
            if (gamepad2.left_trigger > 0) {
                robotHardware.setLeftArmPositionAndVelocity(CLAW_ARM_PICK_SPECIMEN_A, CLAW_ARM_VELOCITY);

                //robotHardware.operateClawServo(true, CLAW_OPEN_POSITION, CLAW_CLOSE_POSITION);
            }

            if (gamepad2.right_trigger > 0) {
                if (INTAKE_ARM_NEAR_SAMPLE_MODE) {
                    robotHardware.setRightArmPositionAndVelocity(INTAKE_ARM_PICK_NEAR_SAMPLE_A, INTAKE_ARM_VELOCITY);
                    robotHardware.setSlidePositionAndVelocity(SLIDE_POSITION_TO_PICK_NEAR_SAMPLE, SLIDE_VELOCITY);
                }
                else {
                    robotHardware.setRightArmPositionAndVelocity(INTAKE_ARM_PICK_FAR_SAMPLE_A, INTAKE_ARM_VELOCITY);
                    robotHardware.setSlidePositionAndVelocity(SLIDE_POSITION_TO_PICK_FAR_SAMPLE, SLIDE_VELOCITY);
                }
                robotHardware.operateIntakeServo(true, false);
            }
        }

        //process button Y
        if (gamepad2.y){
            if (gamepad2.left_trigger > 0) {
                robotHardware.setLeftArmPositionAndVelocity(CLAW_ARM_HANG_SPECIMEN_Y, CLAW_ARM_VELOCITY);
            }

            if (gamepad2.right_trigger > 0) {
                robotHardware.setRightArmPositionAndVelocity(INTAKE_ARM_DROP_SAMPLE_HIGH_Y, INTAKE_ARM_VELOCITY);
                robotHardware.setSlidePositionAndVelocity(SLIDE_POSITION_TO_SCORE_HIGH, SLIDE_VELOCITY);
            }
        }

        //process button B
        if (gamepad2.b){
            if (gamepad2.left_trigger > 0) {
                robotHardware.setLeftArmPositionAndVelocity(CLAW_ARM_SNAP_SPECIMEN_B, 500);



//                try {
//                    Thread.sleep(1500);
//                } catch (InterruptedException e) {
//                    Thread.currentThread().interrupt();
//                }
                //robotHardware.operateClawServo(true, CLAW_OPEN_POSITION, CLAW_CLOSE_POSITION);
            }

            if (gamepad2.right_trigger > 0) {
                robotHardware.setRightArmPositionAndVelocity(INTAKE_ARM_DROP_SAMPLE_LOW_B, INTAKE_ARM_VELOCITY);
                robotHardware.setSlidePositionAndVelocity(SLIDE_POSITION_TO_SCORE_LOW, SLIDE_VELOCITY);
            }
        }

        //process button x
        if (gamepad2.x){
            if (gamepad2.left_trigger > 0) {
                robotHardware.setLeftArmPositionAndVelocity(CLAW_ARM_HORIZONTAL_X, CLAW_ARM_VELOCITY);
            }

            if (gamepad2.right_trigger > 0) {
                robotHardware.setRightArmPositionAndVelocity(INTAKE_ARM_HORIZONTAL_X, INTAKE_ARM_VELOCITY);
                robotHardware.setSlidePositionAndVelocity(SLIDE_POSITION_RESTING, SLIDE_VELOCITY);
            }
        }

        //when the start button is pressed it sets the left & right arms into hanging position
        if (gamepad2.start && gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0){
            robotHardware.setLeftArmPositionAndVelocity(CLAW_ARM_HANG_START, CLAW_ARM_VELOCITY);
            robotHardware.setRightArmPositionAndVelocity(INTAKE_ARM_HANG_START, INTAKE_ARM_VELOCITY);
            robotHardware.operateClawServo(true, CLAW_OPEN_POSITION, CLAW_CLOSE_POSITION); //open claw when hanging
        }

    }

    private void ProcessDPad(Telemetry telemetry) {
        //process Dpad left input to spin intake servo clockwise to pick up the peice
        if (gamepad2.dpad_left){
            robotHardware.operateIntakeServo(true, false);
        }
        //process Dpad right input to spin intake servo counter clockwise to throw the peice out
        if (gamepad2.dpad_right) {
            robotHardware.operateIntakeServo(false, true);
        }
    }

    private void ProcessBumpers(Telemetry telemetry) {
        // if the right bumper is pressed it opens the claw
        if (gamepad2.right_bumper) {
            //telemetry.addLine("right bumper pressed");
            robotHardware.operateClawServo(true, CLAW_OPEN_POSITION, CLAW_CLOSE_POSITION);
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
            float leftYSignal = gamepad2.left_stick_y * -1;
            float rightYSignal = gamepad2.right_stick_y * -1;

            // If the left joystick is greater than zero, it moves the left arm up
            if (leftYSignal > 0) {
                robotHardware.setLeftArmPositionAndVelocity(robotHardware.getClawArmMotorPos() + MANUAL_OVERRIDE_DELTA, leftYSignal * CLAW_ARM_VELOCITY);

            }
            // If the left joystick is less than zero, it moves the left arm down
            else if (leftYSignal < 0){
                robotHardware.setLeftArmPositionAndVelocity(robotHardware.getClawArmMotorPos() - MANUAL_OVERRIDE_DELTA, leftYSignal * CLAW_ARM_VELOCITY);
            }

            // If the right joystick is greater zero it moves the right arm up
            if (rightYSignal > 0) {
                robotHardware.setRightArmPositionAndVelocity(robotHardware.getSlideArmMotorPos() + MANUAL_OVERRIDE_DELTA, rightYSignal * INTAKE_ARM_VELOCITY);
            }
            // If the right joystick is less than zero it moves the right arm down
            else if (rightYSignal < 0) {
                robotHardware.setRightArmPositionAndVelocity(robotHardware.getSlideArmMotorPos() - MANUAL_OVERRIDE_DELTA, rightYSignal * INTAKE_ARM_VELOCITY);
            }

            //process Dpad up input to extend linear slide
            if (gamepad2.dpad_up){
                //SLIDE CANNOT EXPAND BEYOND THE FAR POSITION FOR IT TO BE UNDER LIMITS
                robotHardware.setSlidePositionAndVelocity(Math.min(robotHardware.getSlidePos() + MANUAL_OVERRIDE_DELTA, SLIDE_POSITION_TO_PICK_FAR_SAMPLE), SLIDE_VELOCITY);
            }

            //process Dpad down input to retract linear slide
            if (gamepad2.dpad_down){
                //SLIDE POSITION CANNOT BE LESS THAN 0
                robotHardware.setSlidePositionAndVelocity(Math.max(robotHardware.getSlidePos() - MANUAL_OVERRIDE_DELTA, 0), SLIDE_VELOCITY);
            }
        }
    }

    interface RobotHardwareFunction {
        void run(int position, int velocity);
    }

    private class myAsyncTask extends AsyncTask<Void, Void, Void>
    {
        RobotHardwareFunction function1;
        RobotHardwareFunction function2;

        public myAsyncTask(RobotHardwareFunction function1, RobotHardwareFunction function2) {
            this.function1 = function1;
            this.function2 = function2;
        }

        @Override
        protected Void doInBackground(Void... params) {
            return null;
        }

        @Override
        protected void onPostExecute(Void result) {
            try {

            } catch (Exception e) {

            }
        }
    }


}