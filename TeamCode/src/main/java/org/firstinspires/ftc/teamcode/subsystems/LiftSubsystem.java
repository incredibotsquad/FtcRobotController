package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftSubsystem extends SubsystemBase {
    private final CRServo leftLiftServo;
    private final CRServo rightLiftServo;
    private final AnalogInput liftServoEncoder;
    private DigitalChannel limitSwitch;
    // Variables to track multi-turn position
    private double lastRawPosition = 0;
    private double totalRotations = 0;
    private final double MAX_VOLTAGE = 3.3; // Standard for Hub analog ports
    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftLiftServo = new CRServo(hardwareMap, "leftLiftServo");

        rightLiftServo = new CRServo(hardwareMap, "rightLiftServo");
        rightLiftServo.setInverted(true);

        liftServoEncoder = hardwareMap.get(AnalogInput.class, "liftServoEncoder");

        lastRawPosition = liftServoEncoder.getVoltage();

        limitSwitch = hardwareMap.get(DigitalChannel.class, "liftResetSensor");
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void startLiftUp() {
        leftLiftServo.set(1);
        rightLiftServo.set(1);
    }

    public void startLiftDown() {
        leftLiftServo.set(-1);
        rightLiftServo.set(-1);
    }

    public void stopLift() {
        leftLiftServo.set(0);
        rightLiftServo.set(0);
    }

    /**
     * Returns a continuous value representing the lift height.
     * Calculated as: (Full Rotations * 3.3) + Current Voltage
     */
    public double getPosition() {
        return (totalRotations * MAX_VOLTAGE) + lastRawPosition;
    }

    /**
     * Returns true if the limit switch is pressed.
     * Note: Most REV/GoBILDA switches are "Active Low" (true when NOT pressed),
     * so we negate it (!) if necessary.
     */
    public boolean isLimitSwitchPressed() {
        // If using a standard REV touch sensor:
        // getState() returns true when NOT pressed, false when pressed.
        return !limitSwitch.getState();
    }

    /*
     * This method is called periodically by the CommandScheduler. Useful for updating subsystem-specific state that you don't want to offload to a Command.
     * Teams should try to be consistent within their own codebases about which responsibilities will be handled by Commands, and which will be handled here.
     * Excellent for sending telemetry or updating dashboards, but optional!
     * */
    @Override
    public void periodic() {

        // Logic to "unwrap" the encoder (track multiple 360-degree turns)
        double currentRawPosition = liftServoEncoder.getVoltage();

        // If the jump is massive (e.g., from 3.2V to 0.1V), we passed the 0-point
        if (currentRawPosition - lastRawPosition < -MAX_VOLTAGE / 2) {
            totalRotations++;
        } else if (currentRawPosition - lastRawPosition > MAX_VOLTAGE / 2) {
            totalRotations--;
        }

        lastRawPosition = currentRawPosition;
    }
}
