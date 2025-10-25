package org.firstinspires.ftc.teamcode.SubSystems;

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

    public static double DISTANCE_THRESHOLD_IN_MM = 100;
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

    @Override
    public void periodic() {
        if ((colorSensor1.getDistance(DistanceUnit.MM) < DISTANCE_THRESHOLD_IN_MM) ||
            (colorSensor2.getDistance(DistanceUnit.MM) < DISTANCE_THRESHOLD_IN_MM))
        {
            //NOTE: might have to set the detected color to NONE here

            if ((colorSensor1.green() > colorSensor1.blue() && colorSensor1.green() > colorSensor1.red()) &&
                (colorSensor2.green() > colorSensor2.blue() && colorSensor2.green() > colorSensor2.red())) {
                detectedColor = GameColors.GREEN;
            }

            if ((colorSensor1.blue() > colorSensor1.green() && colorSensor1.blue() > colorSensor1.red()) &&
                    (colorSensor2.blue() > colorSensor2.green() && colorSensor2.blue() > colorSensor2.red())) {
                detectedColor = GameColors.PURPLE;
            }
        }
    }


}
