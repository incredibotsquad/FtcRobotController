package org.firstinspires.ftc.teamcode.SubSystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ColorSensorOutput;
import org.firstinspires.ftc.teamcode.GameColors;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

@Config
public class IntakeColorSensors implements Subsystem {
    private ColorRangeSensor colorSensor1;
    private ColorRangeSensor colorSensor2;
    public GameColors detectedColor;

    public static double DISTANCE_THRESHOLD_IN_MM = 40;
    public static final IntakeColorSensors INSTANCE = new IntakeColorSensors();

    private IntakeColorSensors() {
        detectedColor = GameColors.NONE;
    }

    @Override
    public void initialize() {
        colorSensor1 = ActiveOpMode.hardwareMap().get(ColorRangeSensor.class, "IntakeColorSensor1");

        colorSensor2 = ActiveOpMode.hardwareMap().get(ColorRangeSensor.class, "IntakeColorSensor2");
    }

    public void clearDetectedColor() {
        detectedColor = GameColors.NONE;
    }

    //THIS FUNCTION GETS CALLED EVERY LOOP WHILE WAITING FOR START - ITS FINE TO RUN IT THEN
    //THIS WILL ALSO GET CALLED WHILE THE OPMODE IS RUNNING - THIS IS ALSO WHAT WE NEED
    @Override
    public void periodic() {
        double sensor1Distance = colorSensor1.getDistance(DistanceUnit.MM);
        double sensor2Distance = colorSensor2.getDistance(DistanceUnit.MM);

        if ((sensor1Distance < DISTANCE_THRESHOLD_IN_MM) || (sensor2Distance < DISTANCE_THRESHOLD_IN_MM))
        {
            //NOTE: might have to set the detected color to NONE here
            Log.i("COLOR SENSORS", "PASSED DISTANCE THRESHOLD: ");
            Log.i("COLOR SENSORS", "DISTANCE 1: " + sensor1Distance);
            Log.i("COLOR SENSORS", "DISTANCE 2: " + sensor2Distance);

            detectedColor = GameColors.NONE;

            Log.i("Color Sensor Test", "Sensor 1 R: " + colorSensor1.red());
            Log.i("Color Sensor Test", "Sensor 1 G: " + colorSensor1.green());
            Log.i("Color Sensor Test", "Sensor 1 B: " + colorSensor1.blue());

            if ((colorSensor1.green() > colorSensor1.blue() && colorSensor1.green() > colorSensor1.red()) &&
                (colorSensor2.green() > colorSensor2.blue() && colorSensor2.green() > colorSensor2.red())) {
                detectedColor = GameColors.GREEN;
                Log.i("COLOR SENSORS", "DETECTED GREEN");
            }

            Log.i("Color Sensor Test", "Sensor 2 R: " + colorSensor2.red());
            Log.i("Color Sensor Test", "Sensor 2 G: " + colorSensor2.green());
            Log.i("Color Sensor Test", "Sensor 2 B: " + colorSensor2.blue());

            if ((colorSensor1.blue() > colorSensor1.green() && colorSensor1.blue() > colorSensor1.red()) &&
                    (colorSensor2.blue() > colorSensor2.green() && colorSensor2.blue() > colorSensor2.red())) {
                detectedColor = GameColors.PURPLE;
                Log.i("COLOR SENSORS", "DETECTED PURPLE");
            }
        }
    }
}
