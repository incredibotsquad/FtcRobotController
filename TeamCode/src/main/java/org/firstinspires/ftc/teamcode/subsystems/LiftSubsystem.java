package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftSubsystem extends SubsystemBase {
    private final CRServo leftLiftServo;
    private final CRServo rightLiftServo;

    ElapsedTime liftTimer;

    private static final double LIFT_DURATION = 3000;
    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftLiftServo = new CRServo(hardwareMap, "leftLiftServo");

        rightLiftServo = new CRServo(hardwareMap, "rightLiftServo");
        rightLiftServo.setInverted(true);
    }

    public void liftRobot() {
        liftTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (liftTimer.milliseconds() < LIFT_DURATION) {
            leftLiftServo.set(1);
            rightLiftServo.set(1);
        }

        leftLiftServo.set(0);
        rightLiftServo.set(0);
    }

    public void resetLift() {
        liftTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        while (liftTimer.milliseconds() < LIFT_DURATION) {
            leftLiftServo.set(-1);
            rightLiftServo.set(-1);
        }

        leftLiftServo.set(0);
        rightLiftServo.set(0);
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
