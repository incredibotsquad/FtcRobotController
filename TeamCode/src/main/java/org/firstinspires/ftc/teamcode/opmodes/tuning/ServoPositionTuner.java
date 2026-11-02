package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * Find positions for any positional servo on the robot (gate open/closed,
 * turret limits, indicator light colors, ...).
 *
 * Servos are not moved until you touch them, so nothing jumps on start.
 * Servos missing from the robot configuration are skipped.
 * Inversion matches the subsystems, so the numbers shown here can be copied straight
 * into the subsystem constants.
 *
 * Controls (gamepad 1):
 *   LB / RB        - previous / next servo
 *   Dpad up / down - position +/- step
 *   B              - cycle step size
 *   A              - go to 0.5
 *   Y              - go to PANELS_POSITION (editable in Panels Configs tab)
 *   X              - toggle sweep between SWEEP_MIN and SWEEP_MAX
 */
@Configurable
@TeleOp(name = "ServoPositionTuner", group = "Tests")
public class ServoPositionTuner extends LinearOpMode {

    /** Hardware name + whether the subsystem inverts it. Keep in sync with the subsystems. */
    private static final String[] SERVO_NAMES = {
            "launchGateServo",
            "turretServo",
            "alignmentIndicatorLight",
            "artifactCountLight",
    };
    private static final boolean[] SERVO_INVERTED = {
            false,
            false,
            false,
            false,
    };

    // Live-editable in Panels
    public static double PANELS_POSITION = 0.5;
    public static double SWEEP_MIN = 0.0;
    public static double SWEEP_MAX = 1.0;
    public static double SWEEP_PERIOD_MS = 2000;

    private final double[] stepSizes = {0.1, 0.05, 0.01, 0.005, 0.001, 0.0001};

    private static class TunedServo {
        final String name;
        final Servo servo;
        double position = Double.NaN; // NaN = not yet commanded

        TunedServo(String name, Servo servo) {
            this.name = name;
            this.servo = servo;
        }

        void set(double pos) {
            position = Math.max(0.0, Math.min(1.0, pos));
            servo.setPosition(position);
        }
    }

    @Override
    public void runOpMode() {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        GamepadEx gp1 = new GamepadEx(gamepad1);

        List<TunedServo> servos = new ArrayList<>();
        List<String> missing = new ArrayList<>();
        for (int i = 0; i < SERVO_NAMES.length; i++) {
            Servo s = hardwareMap.tryGet(Servo.class, SERVO_NAMES[i]);
            if (s == null) {
                missing.add(SERVO_NAMES[i]);
                continue;
            }
            s.setDirection(SERVO_INVERTED[i] ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
            servos.add(new TunedServo(SERVO_NAMES[i], s));
        }

        if (servos.isEmpty()) {
            panels.addLine("No servos found in the robot configuration.");
            panels.update(telemetry);
            waitForStart();
            return;
        }

        int selected = 0;
        int stepIndex = 2;
        boolean sweeping = false;
        boolean sweepHigh = false;
        ElapsedTime sweepTimer = new ElapsedTime();

        panels.addLine("Ready. Servos will not move until touched.");
        if (!missing.isEmpty()) panels.addData("Skipped (not configured)", String.join(", ", missing));
        panels.update(telemetry);
        waitForStart();

        while (opModeIsActive()) {
            gp1.readButtons();
            TunedServo current = servos.get(selected);

            // Select servo (stops any sweep so it doesn't keep running on the old one)
            if (gp1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                selected = (selected + 1) % servos.size();
                sweeping = false;
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                selected = (selected - 1 + servos.size()) % servos.size();
                sweeping = false;
            }
            current = servos.get(selected);

            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                stepIndex = (stepIndex + 1) % stepSizes.length;
            }

            double base = Double.isNaN(current.position) ? 0.5 : current.position;
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                sweeping = false;
                current.set(base + stepSizes[stepIndex]);
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                sweeping = false;
                current.set(base - stepSizes[stepIndex]);
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                sweeping = false;
                current.set(0.5);
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.Y)) {
                sweeping = false;
                current.set(PANELS_POSITION);
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.X)) {
                sweeping = !sweeping;
                sweepTimer.reset();
                sweepHigh = false;
                if (sweeping) current.set(SWEEP_MIN);
            }

            // Sweep: alternate ends every half period
            if (sweeping && sweepTimer.milliseconds() >= SWEEP_PERIOD_MS / 2.0) {
                sweepTimer.reset();
                sweepHigh = !sweepHigh;
                current.set(sweepHigh ? SWEEP_MAX : SWEEP_MIN);
            }

            // Telemetry
            panels.addData("Selected", String.format("%s (%d/%d)", current.name, selected + 1, servos.size()));
            panels.addData("Position", Double.isNaN(current.position) ? "not set" : String.format("%.3f", current.position));
            panels.addData("Step", stepSizes[stepIndex]);
            panels.addData("Sweeping", sweeping);
            panels.addLine("--- All servos ---");
            for (TunedServo s : servos) {
                panels.addData(s.name, Double.isNaN(s.position) ? "not set" : String.format("%.3f", s.position));
            }
            panels.update(telemetry);
        }
    }
}
