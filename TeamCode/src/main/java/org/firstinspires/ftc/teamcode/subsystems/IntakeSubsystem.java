package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx intakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = new MotorEx(hardwareMap, "intakeMotor", Motor.GoBILDA.RPM_1150);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);
    }

    public void startIntake() {
        intakeMotor.set(1);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    /*
     * This method is called periodically by the CommandScheduler. Useful for updating subsystem-specific state that you don't want to offload to a Command.
     * Teams should try to be consistent within their own codebases about which responsibilities will be handled by Commands, and which will be handled here.
     * Excellent for sending telemetry or updating dashboards, but optional!
     * */
    @Override
    public void periodic() {
        //TODO: UPDATE THE STATE OF THE LIGHTS HERE BY USING THE DISTANCE SENSORS TO FIND OUT HOW MANY ARTIFACTS ARE LEFT

        // (You would pass a telemetry object into the subsystem constructor to use this)
        // telemetry.addData("Launcher Ready", isReady);
        // telemetry.addData("Current Speed", getCurrentVelocity());
    }
}
