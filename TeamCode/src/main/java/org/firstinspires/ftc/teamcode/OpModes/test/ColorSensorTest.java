package org.firstinspires.ftc.teamcode.OpModes.test;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GameColors;


@Config
@TeleOp(name="Color Sensor Test", group="Linear OpMode")
public class ColorSensorTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public ColorRangeSensor colorSensor1;
    public ColorRangeSensor colorSensor2;

    public static double DISTANCE_THRESHOLD_IN_MM = 100;

    @Override
    public void runOpMode() {

        telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        colorSensor1 = hardwareMap.get(ColorRangeSensor.class, "IntakeColorSensor1");
        colorSensor2 = hardwareMap.get(ColorRangeSensor.class, "IntakeColorSensor2");

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            GameColors detectedColor = GameColors.NONE;
            if ((colorSensor1.getDistance(DistanceUnit.MM) < DISTANCE_THRESHOLD_IN_MM) ||
                    (colorSensor2.getDistance(DistanceUnit.MM) < DISTANCE_THRESHOLD_IN_MM)) {
                Log.i("Color Sensor Test", "Sensor 1 distance in mm: " + colorSensor1.getDistance(DistanceUnit.MM));
                Log.i("Color Sensor Test", "Sensor 2 distance in mm: " + colorSensor2.getDistance(DistanceUnit.MM));

                Log.i("Color Sensor Test", "Sensor 1 R: " + colorSensor1.red());
                Log.i("Color Sensor Test", "Sensor 1 G: " + colorSensor1.green());
                Log.i("Color Sensor Test", "Sensor 1 B: " + colorSensor1.blue());
                Log.i("Color Sensor Test", "Sensor 1 Alpha: " + colorSensor1.alpha());


                if ((colorSensor1.green() > colorSensor1.blue() && colorSensor1.green() > colorSensor1.red()) &&
                        (colorSensor2.green() > colorSensor2.blue() && colorSensor2.green() > colorSensor2.red())) {
                    detectedColor = GameColors.GREEN;

                }

                Log.i("Color Sensor Test", "Sensor 2 R: " + colorSensor2.red());
                Log.i("Color Sensor Test", "Sensor 2 G: " + colorSensor2.green());
                Log.i("Color Sensor Test", "Sensor 2 B: " + colorSensor2.blue());
                Log.i("Color Sensor Test", "Sensor 2 Alpha: " + colorSensor2.alpha());

                if ((colorSensor1.blue() > colorSensor1.green() && colorSensor1.blue() > colorSensor1.red()) &&
                        (colorSensor2.blue() > colorSensor2.green() && colorSensor2.blue() > colorSensor2.red())) {
                    detectedColor = GameColors.PURPLE;

                }

                Log.i("Color Sensor Test", "Detected color: " + detectedColor);

            }
        }
    }
}
