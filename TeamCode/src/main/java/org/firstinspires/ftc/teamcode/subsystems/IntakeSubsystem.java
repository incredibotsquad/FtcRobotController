package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx intakeMotor;
    private SimpleServo artifactCountLight;
    private DigitalChannel beamBreak1;
    private DigitalChannel beamBreak2;
    private DigitalChannel beamBreak3;

    public static double ZERO_BALL_COLOR = 0;
    public static double ONE_BALL_COLOR = 0.29; //RED
    public static double TWO_BALL_COLOR = 0.388; //YELLOW
    public static double THREE_BALL_COLOR = 0.5; //GREEN

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        intakeMotor = new MotorEx(hardwareMap, "intakeMotor", Motor.GoBILDA.RPM_1150);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);

        // Get the digital sensor from the hardware map and Set the channel as an input
        beamBreak1 = hardwareMap.get(DigitalChannel.class, "ballSensorLow");
        beamBreak1.setMode(DigitalChannel.Mode.INPUT);

        beamBreak2 = hardwareMap.get(DigitalChannel.class, "ballSensorMid");
        beamBreak2.setMode(DigitalChannel.Mode.INPUT);

        beamBreak3 = hardwareMap.get(DigitalChannel.class, "ballSensorHigh");
        beamBreak3.setMode(DigitalChannel.Mode.INPUT);

        artifactCountLight = hardwareMap.get(SimpleServo.class, "artifactCountLight");
    }

    public void startIntake() {
        intakeMotor.set(1);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    private void updateStatusLight(int artifactCount) {

        double color =  ZERO_BALL_COLOR;

        switch (artifactCount) {
            case 1:
                color = ONE_BALL_COLOR;
                break;
            case 2:
                color = TWO_BALL_COLOR;
                break;
            case 3:
                color = THREE_BALL_COLOR;
                break;
        }

        artifactCountLight.setPosition(color);
    }

    /*
     * This method is called periodically by the CommandScheduler. Useful for updating subsystem-specific state that you don't want to offload to a Command.
     * Teams should try to be consistent within their own codebases about which responsibilities will be handled by Commands, and which will be handled here.
     * Excellent for sending telemetry or updating dashboards, but optional!
     * */
    @Override
    public void periodic() {

        int artifactCount = 0;

        // Read the sensor state (true = HIGH, false = LOW)
        boolean stateHigh = beamBreak1.getState();
        if (stateHigh)
            artifactCount++;

        stateHigh = beamBreak2.getState();
        if (stateHigh)
            artifactCount++;

        stateHigh = beamBreak3.getState();
        if (stateHigh)
            artifactCount++;

        updateStatusLight(artifactCount);

        // (You would pass a telemetry object into the subsystem constructor to use this)
        // telemetry.addData("Launcher Ready", isReady);
        // telemetry.addData("Current Speed", getCurrentVelocity());
    }
}
