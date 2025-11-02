package org.firstinspires.ftc.teamcode.OpModes.test;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GameColors;
import org.opencv.core.Scalar;


@Disabled
@Config
@TeleOp(name="Color Sensor Test", group="Linear OpMode")
public class ColorSensorTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    public ColorRangeSensor colorSensor1;
    public ColorRangeSensor colorSensor2;

    public static double DISTANCE_THRESHOLD_IN_MM = 30;

    @Override
    public void runOpMode() {

        telemetry.update();
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        colorSensor1 = hardwareMap.get(ColorRangeSensor.class, "IntakeColorSensor1");
        colorSensor2 = hardwareMap.get(ColorRangeSensor.class, "IntakeColorSensor2");

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
            if ((colorSensor1.getDistance(DistanceUnit.MM) < DISTANCE_THRESHOLD_IN_MM) ) {


//                Log.i("Color Sensor Test", "Sensor 1 distance in mm: " + colorSensor1.getDistance(DistanceUnit.MM));
//                Log.i("Color Sensor Test", "Sensor 2 distance in mm: " + colorSensor2.getDistance(DistanceUnit.MM));
//
                Log.i("Color Sensor Test", "Sensor 1 R: " + colorSensor1.red() + " G: " + colorSensor1.green() + " B: " + colorSensor1.blue());
                Log.i("Color Sensor Test", "Sensor 1 NORMALIZED R: " + colorSensor1.getNormalizedColors().red + " G: " +colorSensor1.getNormalizedColors().green + " B: " + colorSensor1.getNormalizedColors().blue);

//                Log.i("Color Sensor Test", "Sensor 1 Alpha: " + colorSensor1.alpha());
//
//
//                if ((colorSensor1.green() > colorSensor1.blue() && colorSensor1.green() > colorSensor1.red()) &&
//                        (colorSensor2.green() > colorSensor2.blue() && colorSensor2.green() > colorSensor2.red())) {
//                    detectedColor = GameColors.GREEN;
//
//                }
//

//                Log.i("Color Sensor Test", "Sensor 2 Alpha: " + colorSensor2.alpha());
//
//                if ((colorSensor1.blue() > colorSensor1.green() && colorSensor1.blue() > colorSensor1.red()) &&
//                        (colorSensor2.blue() > colorSensor2.green() && colorSensor2.blue() > colorSensor2.red())) {
//                    detectedColor = GameColors.PURPLE;
//
//                }
//
//                Log.i("Color Sensor Test", "Detected color: " + detectedColor);
//                Scalar sensor1out = rgbToYCrCb(colorSensor1.getNormalizedColors().red, colorSensor1.getNormalizedColors().green, colorSensor1.getNormalizedColors().blue);
//                Log.i("Color Sensor Test", "Sensor 1 SCALAR: " + sensor1out.toString());


//                GameColors colorSensor1Classification = classifyGreenOrPurple(colorSensor1.red(), colorSensor1.green(), colorSensor1.blue());
//                Log.i("Color Sensor Test", "Sensor 1 Classification: " + colorSensor1Classification);
//
//
//
//                Log.i("Color Sensor Test", "Sensor 2 R: " + colorSensor2.red() + " G: " +colorSensor2.green() + " B: " + colorSensor2.blue());
//                Log.i("Color Sensor Test", "Sensor 2 NORMALIZED R: " + colorSensor2.getNormalizedColors().red + " G: " + colorSensor2.getNormalizedColors().green + " B: " + colorSensor2.getNormalizedColors().blue);
//
//                Scalar sensor2out = rgbToYCrCb(colorSensor2.getNormalizedColors().red, colorSensor2.getNormalizedColors().green, colorSensor2.getNormalizedColors().blue);
//                Log.i("Color Sensor Test", "Sensor 2 SCALAR: " + sensor2out.toString());
//
//                GameColors colorSensor2Classification = classifyGreenOrPurple(colorSensor2.red(), colorSensor2.green(), colorSensor2.blue());
//                Log.i("Color Sensor Test", "Sensor 2 Classification: " + colorSensor2Classification);
            }
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

//    private GameColors classifyGreenOrPurple(double r, double g, double b) {
//        // Normalize RGB values to 0-1 range for easier calculations
//        double rNorm = r / 255.0;
//        double gNorm = g / 255.0;
//        double bNorm = b / 255.0;
//
//        // Convert RGB to HSV for hue-based detection
//        double max = Math.max(rNorm, Math.max(gNorm, bNorm));
//        double min = Math.min(rNorm, Math.min(gNorm, bNorm));
//        double delta = max - min;
//
//        // Calculate Hue (in degrees 0-360)
//        double hue = 0;
//        if (delta > 0.001) { // Avoid division by zero
//            if (max == rNorm) {
//                hue = 60 * (((gNorm - bNorm) / delta) % 6);
//            } else if (max == gNorm) {
//                hue = 60 * (((bNorm - rNorm) / delta) + 2);
//            } else {
//                hue = 60 * (((rNorm - gNorm) / delta) + 4);
//            }
//        }
//        if (hue < 0) hue += 360;
//
//        Log.i("Color Sensor Test", "hue: " + hue);
//
//        // Calculate Saturation
//        double saturation = (max < 0.001) ? 0 : (delta / max);
//        Log.i("Color Sensor Test", "saturation: " + saturation);
//
//        // Calculate Value (brightness)
//        double value = max;
//
//        // Multi-factor classification
//
//        // Factor 1: Hue-based detection
//        // Green: 90-150 degrees, Purple: 270-330 degrees
//        boolean hueIsGreen = (hue >= 90 && hue <= 150);
//        boolean hueIsPurple = (hue >= 270 && hue <= 330);
//
//        Log.i("Color Sensor Test", "Factor 1: hueIsGreen: " + hueIsGreen);
//        Log.i("Color Sensor Test", "Factor 1: hueIsPurple: " + hueIsPurple);
//
//
//        // Factor 2: RGB ratio analysis
//        // For Green: G should dominate, and G/(R+B) should be high
//        // For Purple: R and B should be similar and higher than G
//        double greenStrength = gNorm / (rNorm + bNorm + 0.001); // Avoid division by zero
//        double purpleStrength = (rNorm + bNorm) / (2.0 * gNorm + 0.001);
//
//        Log.i("Color Sensor Test", "Factor 2: greenStrength: " + greenStrength);
//        Log.i("Color Sensor Test", "Factor 2: purpleStrength: " + purpleStrength);
//
//
//        // Factor 3: Channel dominance with thresholds
//        // Green: G should be significantly higher than both R and B
//        boolean greenDominant = (g > r + 20) && (g > b + 20);
//
//        // Purple: R and B should be close to each other and both higher than G
//        boolean purpleDominant = (Math.abs(r - b) < 40) && (r > g + 10) && (b > g + 10);
//
//        Log.i("Color Sensor Test", "Factor 3: greenDominant: " + greenDominant);
//        Log.i("Color Sensor Test", "Factor 3: purpleDominant: " + purpleDominant);
//
//        // Factor 4: Saturation check (colors should be reasonably saturated)
//        boolean isSaturated = saturation > 0.2;
//        Log.i("Color Sensor Test", "Factor 4: isSaturated: " + isSaturated);
//
//        // Factor 5: Brightness check (object should be reasonably bright)
//        boolean isBright = value > 0.15;
//        Log.i("Color Sensor Test", "Factor 5: isBright: " + isBright);
//
//        // Decision logic combining multiple factors
//        int greenScore = 0;
//        int purpleScore = 0;
//
//        if (hueIsGreen) greenScore += 3;
//        if (hueIsPurple) purpleScore += 3;
//
//        if (greenStrength > 1.2) greenScore += 2;
//        if (purpleStrength > 1.1) purpleScore += 2;
//
//        if (greenDominant) greenScore += 2;
//        if (purpleDominant) purpleScore += 2;
//
//        // Additional checks for edge cases
//        // If hue is in ambiguous range but RGB ratios are clear
//        if (!hueIsGreen && !hueIsPurple) {
//            if (greenStrength > 1.5 && g > 100) greenScore += 1;
//            if (purpleStrength > 1.3 && r > 80 && b > 80) purpleScore += 1;
//        }
//
//        // Require minimum saturation and brightness
//        if (!isSaturated || !isBright) {
//            // If color is too washed out, be more conservative
//            greenScore = Math.max(0, greenScore - 1);
//            purpleScore = Math.max(0, purpleScore - 1);
//        }
//
//        Log.i("Color Sensor Test", "greenScore: " + greenScore);
//        Log.i("Color Sensor Test", "purpleScore: " + purpleScore);
//
//        // Final classification with confidence threshold
//        if ( greenScore > purpleScore) {
//            return GameColors.GREEN;
//        } else if (purpleScore > greenScore) {
//            return GameColors.PURPLE;
//        } else {
//            return GameColors.NONE;
//        }
//    }
}
