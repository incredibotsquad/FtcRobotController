package org.firstinspires.ftc.teamcode.drive.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Config
@Disabled
@TeleOp(name="Color Sensor Test", group="Linear OpMode")
public class ColorSensorTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public ColorRangeSensor colorSensor;

    private int proxnum = 4;
    public CRServo leftServo;
    public CRServo rightServo;
    private double prox;
    private int RightServoPower = -1;
    private int LeftServoPower = 1;
    public int counter;
    private String teamColor = "RED";
    private Boolean isNotTeamColor = false;
    private Boolean DoingSpecimen = true;


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "ColorSensor");

        leftServo = hardwareMap.get(CRServo.class, "LeftServo");
        rightServo = hardwareMap.get(CRServo.class, "RightServo");

        while (opModeInInit()) {
            // Prompt the driver to select the team color
            telemetry.addData("Team Color Selection", "Press B for RED, X for BLUE");
            telemetry.update();

            // Check if buttons are pressed and set teamColor
            if (gamepad2.b || gamepad1.b) {
                teamColor = "RED";  // Set to RED if 'A' is pressed
                telemetry.addData("Selected Color", "RED");
                telemetry.update();
            } else if (gamepad2.x || gamepad1.x) {
                teamColor = "BLUE";  // Set to BLUE if 'B' is pressed
                telemetry.addData("Selected Color", "BLUE");
                telemetry.update();
            }

            if (gamepad2.y || gamepad1.y) {
                DoingSpecimen = false;  // Set to RED if 'A' is pressed
                telemetry.addData("Selected thing", "sample");
                telemetry.update();
            } else if (gamepad2.a || gamepad1.a) {

                DoingSpecimen = true;  // Set to BLUE if 'B' is pressed
                telemetry.addData("Selected thing", "specimen");
                telemetry.update();
            }
        }

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            prox = colorSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("TEAMCOLOR: ", teamColor);
            telemetry.addData("===RIGHTSERVOPOWER===", rightServo.getPower());
            telemetry.addData("===LEFTSERVOPOWER===", leftServo.getPower());
            telemetry.addData("===PROX===", prox);
            telemetry.update();
            leftServo.setPower(LeftServoPower);
            rightServo.setPower(RightServoPower);
            if (prox <= proxnum) {
                    if (teamColor.equals("BLUE")) {
                        telemetry.addData("Checking for team color ", "BLUE OR YELLOW");
                        isNotTeamColor = (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green());
                    } else {
                        telemetry.addData("Checking for team color: ", "RED OR YELLOW");
                        isNotTeamColor = (colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green());
                    }
                }
                if (isNotTeamColor) {
                    Outtake();
                    Intake();
                    isNotTeamColor = false;
                }
            }
        }




    private void Outtake() {
        telemetry.addData("SWITCHING SERVO DIRECTION", "WRONG COLOR DETECTED");
        counter = 0;
        while (counter <= 50000) {
            telemetry.addData("Out taking teancolor: ", isNotTeamColor);
            telemetry.addData("COUNTER:", counter);
            telemetry.update();
            leftServo.setPower(-1);
            rightServo.setPower(1);
            counter += 1;
        }
    }

    private void Intake() {

        telemetry.addData("Resetting", "direction to intake");
        telemetry.update();
        leftServo.setPower(1);
        rightServo.setPower(-1);
    }


    public String DetectColor() {

        if (colorSensor.blue() > colorSensor.green() && colorSensor.blue() > colorSensor.red()) {
            return "BLUE";

        } else if (colorSensor.red() > colorSensor.green() && colorSensor.red() > colorSensor.blue()) {
            return  "RED;";
        }

        else{
            return "YELLOW";
        }

    }
}
