package org.firstinspires.ftc.teamcode.OpModes.test;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.opencv.core.Scalar;


@Config
@TeleOp(name="Color Sensor Test", group="Tests")
public class ColorSensorTest extends LinearOpMode {

    public static double COLOR_SENSOR_LIGHT_INTENSITY =  0.5;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public ColorRangeSensor colorSensor;
    private Servo Servo1;

    public static double DISTANCE_THRESHOLD_IN_MM = 30;

    @Override
    public void runOpMode() {

        telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        colorSensor = hardwareMap.get(ColorRangeSensor.class, "IntakeColorSensor");
        Servo1 = hardwareMap.get(Servo.class, "ColorSensorLight");
        Servo1.setPosition(COLOR_SENSOR_LIGHT_INTENSITY);

        runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        // Wait for the game to start (driver presses START)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (runtime.milliseconds() < 2000)
                continue;

            runtime.reset();

            //||
            //                    (colorSensor2.getDistance(DistanceUnit.MM) < DISTANCE_THRESHOLD_IN_MM)

            GameColors detectedColor = GameColors.NONE;
//            if ((colorSensor.getDistance(DistanceUnit.MM) < DISTANCE_THRESHOLD_IN_MM) ) {


//                Log.i("Color Sensor Test", "Sensor 1 distance in mm: " + colorSensor1.getDistance(DistanceUnit.MM));
//                Log.i("Color Sensor Test", "Sensor 2 distance in mm: " + colorSensor2.getDistance(DistanceUnit.MM));
//
//                Log.i("Color Sensor Test", "Sensor 1 R: " + colorSensor.red() + " G: " + colorSensor.green() + " B: " + colorSensor.blue());

            NormalizedRGBA normalizedRGBA = colorSensor.getNormalizedColors();

            Log.i("Color Sensor Test", "Sensor 1 NORMALIZED R: " + normalizedRGBA.red + " G: " + normalizedRGBA.green + " B: " + normalizedRGBA.blue);

//                Log.i("Color Sensor Test", "Sensor 1 Alpha: " + colorSensor1.alpha());
//
//
                if (normalizedRGBA.green > normalizedRGBA.blue && normalizedRGBA.green > normalizedRGBA.red)  {
                    detectedColor = GameColors.GREEN;
                }
//

//                Log.i("Color Sensor Test", "Sensor 2 Alpha: " + colorSensor2.alpha());
//
                if (normalizedRGBA.blue > normalizedRGBA.green && normalizedRGBA.blue > normalizedRGBA.red) {
                    detectedColor = GameColors.PURPLE;
                }
//
                Log.i("Color Sensor Test", "Detected color: " + detectedColor);

        }
    }

    static Scalar rgbToYCrCb(double r, double g, double b) {
        // BT.601 (OpenCV’s YCrCb)
        double Y  = 0.299*r + 0.587*g + 0.114*b;
        double Cr = (r - Y) * 0.713 + 128.0;
        double Cb = (b - Y) * 0.564 + 128.0;
        return new Scalar(Y, Cr, Cb); // <-- OpenCV’s order: Y, Cr, Cb
    }

    private GameColors classifyGreenOrPurple(double r, double g, double b){
        // Normalize RGB values to 0-1 range
        double rNorm = r / 255.0;
        double gNorm = g / 255.0;
        double bNorm = b / 255.0;

        // Find max and min values
        double max = Math.max(rNorm, Math.max(gNorm, bNorm));
        double min = Math.min(rNorm, Math.min(gNorm, bNorm));
        double delta = max - min;

        // Early exit for gray/black/white (undefined hue)
        // This rejects white plates and other low-saturation colors
        if (delta < 0.05) {
            return GameColors.NONE;
        }

        // Calculate Saturation - reject if too low (white/gray detection)
        double saturation = delta / max;
        if (saturation < 0.35) {
            return GameColors.NONE;  // Too washed out, likely white/gray
        }

        // Calculate Value (brightness) - reject if too dark
        double value = max;
        if (value < 0.2) {
            return GameColors.NONE;  // Too dark
        }

        // Determine which channel is max using index instead of equality checks
        int maxIndex = 0;  // 0=R, 1=G, 2=B
        if (gNorm > rNorm && gNorm >= bNorm) {
            maxIndex = 1;
        } else if (bNorm > rNorm && bNorm > gNorm) {
            maxIndex = 2;
        }

        // Calculate Hue (in degrees 0-360) based on max channel
        double hue;
        if (maxIndex == 0) {
            // Red is max
            hue = 60.0 * ((gNorm - bNorm) / delta);
        } else if (maxIndex == 1) {
            // Green is max
            hue = 60.0 * (((bNorm - rNorm) / delta) + 2.0);
        } else {
            // Blue is max
            hue = 60.0 * (((rNorm - gNorm) / delta) + 4.0);
        }

        // Normalize hue to [0, 360)
        while (hue < 0) {
            hue += 360.0;
        }
        while (hue >= 360.0) {
            hue -= 360.0;
        }

        Log.i("Color Sensor Test", "Algo 1: hue: " + hue);


        // Calculate color strength ratios
        // Green strength: how much green dominates over red and blue
        double greenStrength = gNorm / (rNorm + bNorm + 0.001);
        Log.i("Color Sensor Test", "Algo 1: greenStrength: " + greenStrength);

        // Purple strength: how much red and blue together dominate over green
        double purpleStrength = (rNorm + bNorm) / (2.0 * gNorm + 0.001);
        Log.i("Color Sensor Test", "Algo 1: purpleStrength: " + purpleStrength);

        // Classify based on hue and strength
        // Green: Hue 140-180 degrees (narrowed based on actual data)
        // Purple/Blue: Hue 200-260 degrees (expanded to catch actual purple ball at hue 225)
        boolean hueIsGreen = (hue >= 140 && hue <= 180);
        boolean hueIsPurple = (hue >= 200 && hue <= 260);

        Log.i("Color Sensor Test", "Algo 1: hueIsGreen: " + hueIsGreen);
        Log.i("Color Sensor Test", "Algo 1: hueIsPurple: " + hueIsPurple);


        // Strength thresholds tuned from real sensor data:
        // White plate: greenStrength ~0.69, Green ball: ~0.965
        // Purple ball: purpleStrength ~1.167
        boolean strongGreen = greenStrength > 0.85;
        boolean strongPurple = purpleStrength > 1.05;

        Log.i("Color Sensor Test", "Algo 1: strongGreen: " + strongGreen);
        Log.i("Color Sensor Test", "Algo 1: strongPurple: " + strongPurple);

        // Decision logic: both hue and strength must agree for confident classification
        if (hueIsGreen && strongGreen) {
            return GameColors.GREEN;
        } else if (hueIsPurple && strongPurple) {
            return GameColors.PURPLE;
        } else {
            return GameColors.NONE;
        }
    }

}
