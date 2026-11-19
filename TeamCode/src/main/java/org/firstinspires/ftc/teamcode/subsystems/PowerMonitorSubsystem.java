package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

/**
 * Power logging for chasing the main-fuse blow: each hub's input (battery) voltage and total
 * current, and each motor's current, written to logcat with tag "PowerLog".
 *
 * These reads are not part of the hubs' bulk data, so each one is a separate hub command (a few
 * ms). To keep the loop fast, periodic() reads ONE item per loop (a hub = its voltage and current,
 * or one motor) and writes a line once every item has been read. A full line takes about
 * (number of hubs + motors) loops, so its values are spread over roughly that many loops
 * (~0.2 s). A new round starts at most every LOG_INTERVAL_MS.
 *
 * Lines (see LOG_FORMAT below):
 *   PowerLog: Control Hub 12.4 V 6.1 A | Expansion Hub 2 12.3 V 9.8 A | hubs total 15.9 A
 *             | motors: frontLeftMotor 1.2 A, ... transferMotor 4.3 A | motors total 14.7 A
 * A line is a warning (Log.w, starts with "HIGH") when the hubs' total current is at least
 * HIGH_CURRENT_WARN_A or a hub's voltage is at most LOW_VOLTAGE_WARN_V.
 *
 * Registered by Incredibot in TeleOp, auto and DRIVE_ONLY_MODE, so it runs from START.
 * Pit/debug tool: turn it off with ENABLED (Panels) if loop time matters.
 */
@Configurable
public class PowerMonitorSubsystem extends SubsystemBase {

    /** Master switch, live in Panels. */
    public static boolean ENABLED = true;
    /** Minimum time between the starts of two log rounds, ms. */
    public static long LOG_INTERVAL_MS = 250;
    /** Log the line as a warning when the hubs' total current is at least this (the main fuse is 20 A). */
    public static double HIGH_CURRENT_WARN_A = 20.0;
    /** Log the line as a warning when any hub's input voltage is at or below this. */
    public static double LOW_VOLTAGE_WARN_V = 10.0;

    private static final String TAG = "PowerLog";

    /** Motors listed in this order first; any other motors in the config follow. */
    private static final List<String> MOTOR_ORDER = Arrays.asList(
            "frontLeftMotor", "frontRightMotor", "backLeftMotor", "backRightMotor",
            "leftLaunchMotor", "rightLaunchMotor", "intakeMotor", "transferMotor");

    private final List<LynxModule> hubs = new ArrayList<>();
    private final List<String> hubNames = new ArrayList<>();
    private final List<DcMotorEx> motors = new ArrayList<>();
    private final List<String> motorNames = new ArrayList<>();

    private final double[] hubVolts;
    private final double[] hubAmps;
    private final double[] motorAmps;

    /** Next item to read this round: hubs first, then motors. */
    private int next = 0;
    private long roundStartMs = -1;
    /** The last full line written (for Driver Station telemetry). */
    private String lastLine = "no reading yet";

    public PowerMonitorSubsystem(HardwareMap hardwareMap) {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hubs.add(hub);
            hubNames.add(nameOf(hardwareMap, hub, "hub " + hub.getModuleAddress()));
        }

        List<DcMotorEx> all = hardwareMap.getAll(DcMotorEx.class);
        for (String wanted : MOTOR_ORDER) {
            for (DcMotorEx m : all) {
                if (hardwareMap.getNamesOf(m).contains(wanted) && !motors.contains(m)) {
                    motors.add(m);
                    motorNames.add(wanted);
                }
            }
        }
        for (DcMotorEx m : all) {
            if (!motors.contains(m)) {
                motors.add(m);
                motorNames.add(nameOf(hardwareMap, m, "motor"));
            }
        }

        hubVolts = new double[hubs.size()];
        hubAmps = new double[hubs.size()];
        motorAmps = new double[motors.size()];
        Log.i(TAG, "monitoring hubs " + hubNames + " and motors " + motorNames);
    }

    @Override
    public void periodic() {
        if (!ENABLED) return;
        int items = hubs.size() + motors.size();
        if (items == 0) return;

        long now = System.nanoTime() / 1_000_000L;
        if (next == 0) {
            if (roundStartMs >= 0 && now - roundStartMs < LOG_INTERVAL_MS) return;
            roundStartMs = now;
        }

        if (next < hubs.size()) {
            LynxModule hub = hubs.get(next);
            hubVolts[next] = read(() -> hub.getInputVoltage(VoltageUnit.VOLTS));
            hubAmps[next] = read(() -> hub.getCurrent(CurrentUnit.AMPS));
        } else {
            int i = next - hubs.size();
            DcMotorEx motor = motors.get(i);
            motorAmps[i] = read(() -> motor.getCurrent(CurrentUnit.AMPS));
        }

        next++;
        if (next >= items) {
            next = 0;
            writeLine();
        }
    }

    private void writeLine() {
        StringBuilder b = new StringBuilder();
        double hubTotal = 0;
        boolean lowVoltage = false;
        for (int i = 0; i < hubs.size(); i++) {
            b.append(String.format("%s %.2f V %.1f A | ", hubNames.get(i), hubVolts[i], hubAmps[i]));
            if (!Double.isNaN(hubAmps[i])) hubTotal += hubAmps[i];
            if (!Double.isNaN(hubVolts[i]) && hubVolts[i] <= LOW_VOLTAGE_WARN_V) lowVoltage = true;
        }
        b.append(String.format("hubs total %.1f A | motors: ", hubTotal));
        double motorTotal = 0;
        for (int i = 0; i < motors.size(); i++) {
            if (i > 0) b.append(", ");
            b.append(String.format("%s %.1f A", motorNames.get(i), motorAmps[i]));
            if (!Double.isNaN(motorAmps[i])) motorTotal += motorAmps[i];
        }
        b.append(String.format(" | motors total %.1f A", motorTotal));

        lastLine = b.toString();
        boolean highCurrent = hubTotal >= HIGH_CURRENT_WARN_A;
        if (highCurrent || lowVoltage) {
            Log.w(TAG, "HIGH" + (highCurrent ? " CURRENT" : "") + (lowVoltage ? " LOW VOLTAGE" : "") + ": " + b);
        } else {
            Log.i(TAG, b.toString());
        }
    }

    /** The last full reading (same text as the log line), or "no reading yet". */
    public String getLastLine() {
        return lastLine;
    }

    private interface Reading {
        double get();
    }

    /** NaN if the hub doesn't answer (e.g. it browned out), so one bad read doesn't stop the logging. */
    private static double read(Reading r) {
        try {
            return r.get();
        } catch (RuntimeException e) {
            return Double.NaN;
        }
    }

    private static String nameOf(HardwareMap hardwareMap, HardwareDevice device, String fallback) {
        Iterator<String> names = hardwareMap.getNamesOf(device).iterator();
        return names.hasNext() ? names.next() : fallback;
    }
}
