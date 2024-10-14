package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Config
public class IncredibotsArmControl
{
    Gamepad gamepad2;
    RobotHardware robotHardware;

    //Left (Claw) Arm constants
    public static int LEFT_ARM_RESTING_A = 0;
    public static int LEFT_ARM_HORIZONTAL_B = 4200;
    public static int LEFT_ARM_HANG_SPECIMEN_X = 3639;
    public static int LEFT_ARM_SNAP_SPECIMEN_Y = 3850;
    public static int LEFT_ARM_HANG_ROBOT_START = 500;

    //Right (Intake) Arm constants
    public static int RIGHT_ARM_RESTING_A = 0;
    public static int RIGHT_ARM_HORIZONTAL_B = 0;
    public static int RIGHT_ARM_PICK_SAMPLE_X = 0;
    public static int RIGHT_ARM_DROP_SAMPLE_Y = 0;
    public static int RIGHT_ARM_HANG_ROBOT_START = 0;

    //Slide movement position
    public static int SLIDE_POSITION_TO_SCORE = 0;

    //overide...
    private boolean MANUAL_OVERRIDE = false;

    public IncredibotsArmControl(Gamepad gamepad, RobotHardware robotHardware) {
        gamepad2 = gamepad;
        this.robotHardware = robotHardware;
    }

    public void ProcessInputs() {
        //left trigger + buttons controls left arm (claw)
        //right trigger + buttons controls right arm (intake)

        ProcessButtons();

        ProcessDPad();

        ProcessBumpers();

        HandleManualOverride();
    }
//functions to take gamepad inputs and turn it into movements
    private void ProcessButtons() {
        //process button A
        if (gamepad2.a){
            if (gamepad2.left_trigger > 0) {
                robotHardware.setLeftArmPositionAndPower(LEFT_ARM_RESTING_A, 1);
            }

            if (gamepad2.right_trigger > 0) {
                robotHardware.setRightArmPositionAndPower(RIGHT_ARM_RESTING_A, 1);
            }
        }

        //process button B
        if (gamepad2.b){
            if (gamepad2.left_trigger > 0) {
                robotHardware.setLeftArmPositionAndPower(LEFT_ARM_HORIZONTAL_B, 1);
            }

            if (gamepad2.right_trigger > 0) {
                robotHardware.setRightArmPositionAndPower(RIGHT_ARM_HORIZONTAL_B, 1);
            }
        }

        //process button X
        if (gamepad2.x){
            if (gamepad2.left_trigger > 0) {
                robotHardware.setLeftArmPositionAndPower(LEFT_ARM_HANG_SPECIMEN_X, 1);
            }

            if (gamepad2.right_trigger > 0) {
                robotHardware.setRightArmPositionAndPower(RIGHT_ARM_PICK_SAMPLE_X, 1);
            }
        }

        //process button Y
        if (gamepad2.y){
            if (gamepad2.left_trigger > 0) {
                robotHardware.setLeftArmPositionAndPower(LEFT_ARM_SNAP_SPECIMEN_Y, 1);
            }

            if (gamepad2.right_trigger > 0) {
                robotHardware.setRightArmPositionAndPower(RIGHT_ARM_DROP_SAMPLE_Y, 1);
            }
        }

        //when the start button is pressed it sets the left & right arms into hanging position
        if (gamepad2.start){
            robotHardware.setLeftArmPositionAndPower(LEFT_ARM_HANG_ROBOT_START, 1);
            robotHardware.setRightArmPositionAndPower(RIGHT_ARM_HANG_ROBOT_START, 1);
        }
    }

    private void ProcessDPad() {
        //proccess Dpad up input to extend linear slide to scoring position
        if (gamepad2.dpad_up){
            robotHardware.setSlidePositionAndPower(SLIDE_POSITION_TO_SCORE, 1);
        }
        //process Dpad down input to de-extend linear slide
        if (gamepad2.dpad_down){
            robotHardware.setSlidePositionAndPower(0, 1);
        }
        //process Dpad left input to spin intake servo clockwise to pick up the peice
        if (gamepad2.dpad_left){
            robotHardware.operateIntakeServo(true, false);
        }
        //process Dpad right input to spin intake servo counter clockwise to throw the peice out
        if (gamepad2.dpad_right) {
            robotHardware.operateIntakeServo(false, true);
        }
    }

    private void ProcessBumpers() {
        // if the right bumper is pressed it opens the claw
        if (gamepad2.right_bumper){
            robotHardware.operateClawServo(true);
        }
        // if the left bumper is pressed it closes the claw
        else if (gamepad2.left_bumper){
            robotHardware.operateClawServo(false);
        }
    }

    private void HandleManualOverride() {
        // if the back button is pressed it switches manual ovverides value
        if (gamepad2.back){
            MANUAL_OVERRIDE = !MANUAL_OVERRIDE;
        }
        // if manual override is true it will allow the joysticks to control the arms
        if (MANUAL_OVERRIDE) {

            if (gamepad2.left_stick_y > 0) {
                robotHardware.setLeftArmPositionAndPower(robotHardware.getLeftArmMotorPos() + 1, gamepad2.left_stick_y);

            }
            else if (gamepad2.left_stick_y < 0){
                robotHardware.setLeftArmPositionAndPower(robotHardware.getLeftArmMotorPos() - 1, gamepad2.left_stick_y);
            }

            if (gamepad2.right_stick_y > 0) {
                robotHardware.setRightArmPositionAndPower(robotHardware.getRightArmMotorPos() + 1, gamepad2.right_stick_y);
            }
            else if (gamepad2.right_stick_y < 0) {
                robotHardware.setRightArmPositionAndPower(robotHardware.getRightArmMotorPos() - 1, gamepad2.right_stick_y);
            }
        }
    }

}