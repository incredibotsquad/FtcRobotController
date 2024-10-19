package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;

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
    public static int INTAKE_ARM_PICK_SAMPLE_A = 5000; //4700
    public static int INTAKE_ARM_DROP_SAMPLE_HIGH_Y = 2600;
    public static int INTAKE_ARM_DROP_SAMPLE_LOW_B = 2800; //4050
    public static int INTAKE_ARM_HORIZONTAL_X = 4000;
    public static int INTAKE_ARM_HANG_START = 300;
    public static int INTAKE_ARM_VELOCITY = 1500;

    //claw positions
    public static double CLAW_OPEN_POSITION = 60.0/300.0;
    public static double CLAW_CLOSE_POSITION = 0.0;

    //Slide movement position
    public static int SLIDE_POSITION_TO_PICK_SAMPLE = 800;
    public static int SLIDE_POSITION_TO_SCORE_HIGH = 4500;
    public static int SLIDE_POSITION_TO_SCORE_LOW = 0;
    public static int SLIDE_POSITION_RESTING = 0;
    public static int SLIDE_VELOCITY = 5000;

    //override...
    private boolean MANUAL_OVERRIDE = false;

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
                robotHardware.setRightArmPositionAndVelocity(INTAKE_ARM_PICK_SAMPLE_A, INTAKE_ARM_VELOCITY);
                robotHardware.setSlidePositionAndVelocity(SLIDE_POSITION_TO_PICK_SAMPLE, SLIDE_VELOCITY);
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
        if (gamepad2.start){
            if (gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0) {
                robotHardware.setLeftArmPositionAndVelocity(CLAW_ARM_HANG_START, CLAW_ARM_VELOCITY);
                robotHardware.setRightArmPositionAndVelocity(INTAKE_ARM_HANG_START, INTAKE_ARM_VELOCITY);
            }
        }
    }

    private void ProcessDPad(Telemetry telemetry) {
        //process Dpad up input to extend linear slide to scoring position
        if (gamepad2.dpad_up){
            robotHardware.setSlidePositionAndVelocity(robotHardware.getRightSlidePos() + 1, SLIDE_VELOCITY);
        }
        //process Dpad down input to de-extend linear slide
        if (gamepad2.dpad_down){
            robotHardware.setSlidePositionAndVelocity(robotHardware.getRightSlidePos() - 1, SLIDE_VELOCITY);
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

    private void ProcessBumpers(Telemetry telemetry) {
        // if the right bumper is pressed it opens the claw
        if (gamepad2.right_bumper){
            //telemetry.addLine("right bumper pressed");
            robotHardware.operateClawServo(true, CLAW_OPEN_POSITION, CLAW_CLOSE_POSITION);
        }
        // if the left bumper is pressed it closes the claw
        else if (gamepad2.left_bumper){
            //telemetry.addLine("left bumper pressed");
            robotHardware.operateClawServo(false, CLAW_OPEN_POSITION, CLAW_CLOSE_POSITION);
        }
    }

    private void HandleManualOverride() {
        // if the back button is pressed it switches manual ovverides value
        if (gamepad2.guide){
            MANUAL_OVERRIDE = !MANUAL_OVERRIDE;
        }
        // if manual override is true it will allow the joysticks to control the arms
        if (MANUAL_OVERRIDE) {
            // If the left joystick is greater than zero, it moves the left arm up
            if (gamepad2.left_stick_y > 0) {
                robotHardware.setLeftArmPositionAndVelocity(robotHardware.getLeftArmMotorPos() + 1, gamepad2.left_stick_y * CLAW_ARM_VELOCITY);

            }
            // If the left joystick is less than zero, it moves the left arm down
            else if (gamepad2.left_stick_y < 0){
                robotHardware.setLeftArmPositionAndVelocity(robotHardware.getLeftArmMotorPos() - 1, gamepad2.left_stick_y * CLAW_ARM_VELOCITY);
            }
            // If the right joystick is greater zero it moves the right arm up
            if (gamepad2.right_stick_y > 0) {
                robotHardware.setRightArmPositionAndVelocity(robotHardware.getRightArmMotorPos() + 1, gamepad2.right_stick_y * INTAKE_ARM_VELOCITY);
            }
            // If the right joystick is less than zero it moves the right arm down
            else if (gamepad2.right_stick_y < 0) {
                robotHardware.setRightArmPositionAndVelocity(robotHardware.getRightArmMotorPos() - 1, gamepad2.right_stick_y * INTAKE_ARM_VELOCITY);
            }
        }
    }

}