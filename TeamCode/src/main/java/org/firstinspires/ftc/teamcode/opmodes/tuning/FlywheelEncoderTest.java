package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LaunchSubsystem;

/**
 * Checks that each flywheel motor is paired with its own encoder (not swapped) and that each
 * encoder counts positive when its motor is driven forward. It uses LaunchSubsystem, so motor names
 * and inversion match the real code.
 *
 * Auto test (A): spins ONLY leftLaunchMotor, coasts to a stop, spins ONLY rightLaunchMotor, then
 * compares the peak speed each encoder saw:
 *   - the powered motor's own encoder moves, the other ~0   -> OK
 *   - the OTHER motor's encoder moves, its own ~0            -> SWAPPED
 *   - both encoders move about the same                      -> COUPLED: both motors turn the same
 *       shaft, so spinning one moves both encoders and this test can't tell. Take the belt/gear
 *       off one motor (or unplug one motor's power) and rerun.
 *   - neither moves                                          -> encoder unplugged / motor not spinning
 * Watch which physical motor spins in each phase: if the verdict is SWAPPED, that tells you whether
 * the encoder cables or the motor power cables are the ones crossed (both look the same to software).
 *
 * Manual (hold): LB = left motor only, RB = right motor only, live readings shown.
 *
 * Tune in Panels if needed: TEST_POWER, SPIN_MS, MOVING_RPM, DOMINANCE_RATIO.
 */
@Configurable
@Disabled
@TeleOp(name = "FlywheelEncoderTest", group = "Tests")
public class FlywheelEncoderTest extends LinearOpMode {

    /** Power for each test spin. Low is enough; this only needs the encoders to move. */
    public static double TEST_POWER = 0.4;
    public static long SPIN_MS = 1500;
    /** Coast between phases ends when both encoders are below MOVING_RPM, or after this. */
    public static long MAX_COAST_MS = 6000;
    /** An encoder faster than this counts as moving. */
    public static double MOVING_RPM = 100;
    /** The moving encoder must be this many times faster than the other to tell them apart. */
    public static double DOMINANCE_RATIO = 3.0;

    private enum Phase { IDLE, SPIN_LEFT, COAST_1, SPIN_RIGHT, COAST_2, DONE }

    /** Peak (by magnitude, sign kept) RPM each encoder reached during one spin. */
    private static class Peaks {
        double left = 0, right = 0;

        void sample(double l, double r) {
            if (Math.abs(l) > Math.abs(left)) left = l;
            if (Math.abs(r) > Math.abs(right)) right = r;
        }
    }

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        LaunchSubsystem launcher = new LaunchSubsystem(hardwareMap, panels);
        GamepadEx gp1 = new GamepadEx(gamepad1);

        Phase phase = Phase.IDLE;
        long phaseStartMs = 0;
        Peaks leftSpin = new Peaks();
        Peaks rightSpin = new Peaks();

        panels.addLine("A = run auto test. Hold LB / RB = spin left / right motor only.");
        panels.addLine("Keep hands clear of the flywheel.");
        panels.update(telemetry);
        waitForStart();

        while (opModeIsActive()) {
            gp1.readButtons();
            long now = nowMs();
            double leftRpm = launcher.getLeftFlywheelRPM();
            double rightRpm = launcher.getRightFlywheelRPM();
            boolean stopped = Math.abs(leftRpm) < MOVING_RPM && Math.abs(rightRpm) < MOVING_RPM;

            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                if (phase == Phase.IDLE || phase == Phase.DONE) {
                    leftSpin = new Peaks();
                    rightSpin = new Peaks();
                    phase = Phase.SPIN_LEFT;
                } else {
                    phase = Phase.IDLE; // A again aborts
                }
                phaseStartMs = now;
            }

            // ------------------------------------------------ auto test state machine
            double leftPower = 0, rightPower = 0;
            long elapsed = now - phaseStartMs;
            switch (phase) {
                case SPIN_LEFT:
                    leftPower = TEST_POWER;
                    leftSpin.sample(leftRpm, rightRpm);
                    if (elapsed >= SPIN_MS) { phase = Phase.COAST_1; phaseStartMs = now; }
                    break;
                case COAST_1:
                    if (stopped || elapsed >= MAX_COAST_MS) { phase = Phase.SPIN_RIGHT; phaseStartMs = now; }
                    break;
                case SPIN_RIGHT:
                    rightPower = TEST_POWER;
                    rightSpin.sample(leftRpm, rightRpm);
                    if (elapsed >= SPIN_MS) { phase = Phase.COAST_2; phaseStartMs = now; }
                    break;
                case COAST_2:
                    if (stopped || elapsed >= MAX_COAST_MS) { phase = Phase.DONE; phaseStartMs = now; }
                    break;
                default:
                    // Manual mode only when the auto test isn't running.
                    if (gp1.getButton(GamepadKeys.Button.LEFT_BUMPER)) leftPower = TEST_POWER;
                    if (gp1.getButton(GamepadKeys.Button.RIGHT_BUMPER)) rightPower = TEST_POWER;
                    break;
            }
            launcher.setFlywheelPowersForTest(leftPower, rightPower);

            // ------------------------------------------------ telemetry
            panels.addData("leftRPM", leftRpm);
            panels.addData("rightRPM", rightRpm);
            panels.addData("Phase", phaseLabel(phase));
            panels.addData("Power left / right", leftPower + " / " + rightPower);

            if (phase == Phase.DONE || leftSpin.left != 0 || leftSpin.right != 0) {
                panels.addLine("--- Left motor powered ---");
                panels.addData("left enc / right enc peak RPM", fmt(leftSpin.left) + " / " + fmt(leftSpin.right));
            }
            if (phase == Phase.DONE || rightSpin.left != 0 || rightSpin.right != 0) {
                panels.addLine("--- Right motor powered ---");
                panels.addData("left enc / right enc peak RPM", fmt(rightSpin.left) + " / " + fmt(rightSpin.right));
            }
            if (phase == Phase.DONE) {
                String leftVerdict = verdict(leftSpin.left, leftSpin.right);
                String rightVerdict = verdict(rightSpin.right, rightSpin.left);
                panels.addLine("--- Result ---");
                panels.addData("leftLaunchMotor", leftVerdict + direction(leftVerdict, leftSpin.left));
                panels.addData("rightLaunchMotor", rightVerdict + direction(rightVerdict, rightSpin.right));
                panels.addData("Overall", overall(leftVerdict, rightVerdict));
            }
            panels.update(telemetry);
        }

        launcher.setFlywheelPowersForTest(0, 0);
    }

    /** Classifies one spin from the powered motor's own encoder and the other encoder. */
    private static String verdict(double own, double other) {
        double a = Math.abs(own), b = Math.abs(other);
        if (a < MOVING_RPM && b < MOVING_RPM) return "NO READING";
        if (a >= MOVING_RPM && a >= DOMINANCE_RATIO * b) return "OK";
        if (b >= MOVING_RPM && b >= DOMINANCE_RATIO * a) return "SWAPPED";
        return "COUPLED";
    }

    /** With the motor driven forward its own encoder should read positive (the PID relies on it). */
    private static String direction(String verdict, double own) {
        if (!verdict.equals("OK")) return "";
        return own > 0 ? " (direction OK)" : " (encoder reads NEGATIVE: flip its direction)";
    }

    private static String overall(String l, String r) {
        if (l.equals("OK") && r.equals("OK")) return "Encoders match their motors.";
        if (l.equals("SWAPPED") && r.equals("SWAPPED"))
            return "SWAPPED: swap the two encoder cables (or the motor power cables, whichever spun the wrong motor).";
        if (l.equals("COUPLED") || r.equals("COUPLED"))
            return "Motors share a shaft: decouple one motor and rerun.";
        if (l.equals("NO READING") || r.equals("NO READING"))
            return "An encoder isn't reading or a motor didn't spin: check cables, then rerun.";
        return "Inconsistent: check wiring and rerun (raise TEST_POWER if readings are low).";
    }

    private static String phaseLabel(Phase p) {
        switch (p) {
            case SPIN_LEFT: return "Spinning LEFT motor only (watch which wheel turns)";
            case COAST_1: return "Coasting";
            case SPIN_RIGHT: return "Spinning RIGHT motor only (watch which wheel turns)";
            case COAST_2: return "Coasting";
            case DONE: return "Done (A = run again)";
            default: return "Idle (A = run, hold LB / RB = manual)";
        }
    }

    private static String fmt(double rpm) {
        return String.valueOf(Math.round(rpm));
    }

    private static long nowMs() {
        return System.nanoTime() / 1_000_000L;
    }
}
