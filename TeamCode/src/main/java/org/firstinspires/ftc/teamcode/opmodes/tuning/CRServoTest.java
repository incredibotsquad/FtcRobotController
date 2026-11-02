package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Tests the intake sweeper CR servos, using the same invert flags as IntakeSubsystem,
 * so "positive" here means "sweep in" exactly as it will in the real code.
 *
 * Controls (gamepad 1):
 *   Left stick Y   - servo 1 only (up = positive / sweep in)
 *   Right stick Y  - servo 2 only (up = positive / sweep in)
 *   A              - both sweep in at test power
 *   B              - both sweep out at test power
 *   X              - stop both
 *   Dpad up/down   - test power +/- 0.1
 *
 * Sticks override the A/B mode while held. If a servo spins the wrong way on positive
 * power, flip its INVERT_INTAKE_SERVO_x flag in IntakeSubsystem.
 */
@TeleOp(name = "Test: CR Servos", group = "Test")
public class CRServoTest extends LinearOpMode {

    private static final double STICK_DEADZONE = 0.05;
    private static final double POWER_STEP = 0.1;

    private enum Mode { STOPPED, SWEEP_IN, SWEEP_OUT }

    @Override
    public void runOpMode() {
        CRServo servo1 = new CRServo(hardwareMap, "intakeServo1");
//        servo1.setInverted(IntakeSubsystem.INVERT_INTAKE_SERVO_1);

        CRServo servo2 = new CRServo(hardwareMap, "intakeServo2");
//        servo2.setInverted(IntakeSubsystem.INVERT_INTAKE_SERVO_2);

        double testPower = IntakeSubsystem.SWEEPER_POWER;
        Mode mode = Mode.STOPPED;
        boolean prevDpadUp = false;
        boolean prevDpadDown = false;

        telemetry.addLine("Ready. A = in, B = out, X = stop, sticks = individual servos.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Button modes
            if (gamepad1.a) mode = Mode.SWEEP_IN;
            if (gamepad1.b) mode = Mode.SWEEP_OUT;
            if (gamepad1.x) mode = Mode.STOPPED;

            // Power adjustment (edge-triggered)
            if (gamepad1.dpad_up && !prevDpadUp) testPower = Math.min(1.0, testPower + POWER_STEP);
            if (gamepad1.dpad_down && !prevDpadDown) testPower = Math.max(0.0, testPower - POWER_STEP);
            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;

            double modePower;
            switch (mode) {
                case SWEEP_IN:
                    modePower = testPower;
                    break;
                case SWEEP_OUT:
                    modePower = -testPower;
                    break;
                default:
                    modePower = 0;
                    break;
            }

            // Gamepad stick Y is negative when pushed up, so negate it
            double stick1 = -gamepad1.left_stick_y;
            double stick2 = -gamepad1.right_stick_y;

            double power1 = Math.abs(stick1) > STICK_DEADZONE ? stick1 : modePower;
            double power2 = Math.abs(stick2) > STICK_DEADZONE ? stick2 : modePower;

            servo1.set(power1);
            servo2.set(power2);

            telemetry.addData("Mode", mode);
            telemetry.addData("Test power", "%.1f", testPower);
            telemetry.addData("Servo 1 power", "%.2f (inverted: %b)", power1, servo1.getInverted());
            telemetry.addData("Servo 2 power", "%.2f (inverted: %b)", power2, servo2.getInverted());
            telemetry.addLine("Positive power should sweep IN on both servos.");
            telemetry.update();
        }

        servo1.stop();
        servo2.stop();
    }
}
