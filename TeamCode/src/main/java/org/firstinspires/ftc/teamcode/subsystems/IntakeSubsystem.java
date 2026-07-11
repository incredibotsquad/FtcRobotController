package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx intakeMotor;
    private final SimpleServo artifactCountLight;
    private final DigitalChannel beamBreakHigh;
    private final DigitalChannel beamBreakMid;
    private final DigitalChannel beamBreakLow;

    public static double ZERO_BALL_COLOR = 0;
    public static double ONE_BALL_COLOR = 0.29; //RED
    public static double TWO_BALL_COLOR = 0.388; //YELLOW
    public static double THREE_BALL_COLOR = 0.5; //GREEN

    public IntakeSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        intakeMotor = new MotorEx(hardwareMap, "intakeMotor", Motor.GoBILDA.RPM_1150);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);

        // Get the digital sensor from the hardware map and Set the channel as an input
        beamBreakHigh = hardwareMap.get(DigitalChannel.class, "ballSensorHigh");
        beamBreakHigh.setMode(DigitalChannel.Mode.INPUT);

        beamBreakMid = hardwareMap.get(DigitalChannel.class, "ballSensorMid");
        beamBreakMid.setMode(DigitalChannel.Mode.INPUT);

        beamBreakLow = hardwareMap.get(DigitalChannel.class, "ballSensorLow");
        beamBreakLow.setMode(DigitalChannel.Mode.INPUT);

        artifactCountLight = new SimpleServo(hardwareMap, "artifactCountLight", 0, 270);
    }

    public void startIntake() {
        intakeMotor.set(1);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    public void reverseIntake() {
        intakeMotor.set(-1);
    }

    public void updateStatusLight(int artifactCount) {

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

    public int getArtifactCount() {
        int artifactCount = 0;

        // Read the sensor state (true = HIGH, false = LOW)
        boolean stateHigh = isHighSensorBlocked();
//        Log.i("Intake Subsystem", " Beambreak high detected a ball: " + stateHigh);
        if (stateHigh)
            artifactCount++;

        stateHigh = isMidSensorBlocked();
//        Log.i("Intake Subsystem", " Beambreak mid detected a ball: " + stateHigh);
        if (stateHigh)
            artifactCount++;

        stateHigh = isLowSensorBlocked();
//        Log.i("Intake Subsystem", " Beambreak low detected a ball: " + stateHigh);
        if (stateHigh)
            artifactCount++;
        return artifactCount;
    }

    public boolean isHighSensorBlocked() {
        return beamBreakHigh.getState();
    }

    public boolean isMidSensorBlocked() {
        return beamBreakMid.getState();
    }

    public boolean isLowSensorBlocked() {
        return beamBreakLow.getState();
    }

    /*
     * This method is called periodically by the CommandScheduler. Useful for updating subsystem-specific state that you don't want to offload to a Command.
     * Teams should try to be consistent within their own codebases about which responsibilities will be handled by Commands, and which will be handled here.
     * Excellent for sending telemetry or updating dashboards, but optional!
     * */
    @Override
    public void periodic() {

        // (You would pass a telemetry object into the subsystem constructor to use this)
        // telemetry.addData("Launcher Ready", isReady);
        // telemetry.addData("Current Speed", getCurrentVelocity());
    }
}
