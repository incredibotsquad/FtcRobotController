package org.firstinspires.ftc.teamcode.opmodes.tuning;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Guided Pinpoint drift test. Push the robot by hand (the drive motors are set to coast and never
 * powered) through four checks, each from a fixed mark, and get suggested localizer constants:
 *
 *  1. FORWARD : push exactly TEST_DISTANCE_IN straight forward  -> forward distance error, sideways creep
 *  2. STRAFE  : push exactly TEST_DISTANCE_IN straight left      -> strafe distance error, forward creep
 *  3. SPIN    : spin 180 deg counter-clockwise in place, back into the same footprint (facing the other
 *               way), SPIN_REPEATS times -> pod offset errors (an offset error shows as twice its size
 *               after 180 deg; after a full turn it cancels, so 360 deg spins can't find it)
 *  4. HEADING : spin HEADING_TURNS full turns counter-clockwise, back to the same orientation -> heading drift
 *  5. LOOP    : drive any path with turns and come back to the mark -> total drift (optional)
 *
 * It uses the localizer exactly as configured in pedro/Constants.localizerConfig, so the results are
 * the errors left with the current offsets; the suggestions are corrections to add to them.
 *
 * Gamepad 1:
 *   A        step 1: zero here at the mark / step 2: record the measurement here
 *   B        redo the current measurement (back to "zero at the mark")
 *   Dpad L/R previous / next test
 *   Y        log the summary (also shown on screen)
 *
 * Results: Driver Station, logcat tag OdometryDriftTest, /sdcard/FIRST/data/odometry_drift_test.csv.
 */
@Configurable
@Disabled // pit tool, re-enable to use (TODO.md: disabled before competition)
@TeleOp(name = "OdometryDriftTest", group = "Tests")
public class OdometryDriftTest extends LinearOpMode {

    private static final String TAG = "OdometryDriftTest";

    /** Measured straight-push distance (use a tape line). */
    public static double TEST_DISTANCE_IN = 48.0;
    public static int SPIN_REPEATS = 4;
    public static int HEADING_TURNS = 5;
    /** goBILDA 4-bar pod resolution the Pinpoint uses by default. */
    public static double DEFAULT_TICKS_PER_MM = 19.89436789;

    private enum Test { FORWARD, STRAFE, SPIN, HEADING, LOOP }

    private static final String[] INSTRUCTIONS = {
            "Put the robot on a tape line at a mark, facing along it. A = zero. Then push it exactly %.0f in straight FORWARD (no turning) and press A.",
            "Put the robot at a mark. A = zero. Then push it exactly %.0f in straight LEFT (sideways, no turning) and press A.",
            "Put the robot against a square fixture / tile corner. A = zero. Spin it 180 deg COUNTER-CLOCKWISE in place, set it back into the same footprint (now facing the other way), press A.",
            "Put the robot against a fixture. A = zero. Spin it %d full turns COUNTER-CLOCKWISE and set it back exactly as it started, press A.",
            "Put the robot at a mark. A = zero. Drive or push any path with turns, come back exactly to the mark and orientation, press A.",
    };

    private PinpointLocalizer localizer;
    private Test test = Test.FORWARD;
    private boolean zeroed = false;
    private double unwrappedHeading = 0, lastHeading = 0;

    // results
    private double[] forward, strafe, heading, loop; // {x, y, headingDeg}
    private final List<double[]> spins = new ArrayList<>();
    private File csv;
    private String lastMessage = "";

    @Override
    public void runOpMode() {
        localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        csv = new File(AppUtil.ROBOT_DATA_DIR, "odometry_drift_test.csv");

        telemetry.addLine("OdometryDriftTest: push the robot by hand; nothing is powered.");
        telemetry.addData("Current offsets", String.format(Locale.US, "xPodOffset %.3f, yPodOffset %.3f in",
                Constants.localizerConfig.xPodOffset.get(), Constants.localizerConfig.yPodOffset.get()));
        telemetry.update();
        waitForStart();
        // Let the robot be pushed easily (no motor is ever powered here).
        for (String m : new String[]{"frontLeftMotor", "frontRightMotor", "backLeftMotor", "backRightMotor"}) {
            hardwareMap.get(DcMotor.class, m).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        log("session start: xPodOffset " + Constants.localizerConfig.xPodOffset.get() + ", yPodOffset " + Constants.localizerConfig.yPodOffset.get());

        while (opModeIsActive()) {
            localizer.update();
            Pose p = localizer.pose();
            trackHeading(p.heading());

            if (gamepad1.dpadRightWasPressed()) select(Test.values()[Math.min(Test.values().length - 1, test.ordinal() + 1)]);
            if (gamepad1.dpadLeftWasPressed()) select(Test.values()[Math.max(0, test.ordinal() - 1)]);
            if (gamepad1.bWasPressed()) { zeroed = false; lastMessage = "redo: zero at the mark again"; }
            if (gamepad1.yWasPressed()) log("SUMMARY: " + summary().replace('\n', ' '));
            if (gamepad1.aWasPressed()) {
                if (!zeroed) {
                    localizer.setPose(new Pose(0, 0, 0));
                    localizer.update();
                    unwrappedHeading = 0;
                    lastHeading = 0;
                    zeroed = true;
                    lastMessage = "zeroed: now move the robot, then press A";
                } else {
                    record(p);
                    zeroed = false;
                }
            }

            telemetry.addData("TEST", (test.ordinal() + 1) + "/5 " + test + (test == Test.SPIN ? String.format(Locale.US, " (%d/%d)", spins.size(), SPIN_REPEATS) : ""));
            telemetry.addLine(instructions());
            telemetry.addData("State", zeroed ? "MEASURING: move it, then A" : "press A at the mark to zero");
            telemetry.addData("Pose now", String.format(Locale.US, "x %.2f  y %.2f  heading %.1f deg (total turned %.1f)",
                    p.x(), p.y(), Math.toDegrees(p.heading()), Math.toDegrees(unwrappedHeading)));
            telemetry.addLine(lastMessage);
            telemetry.addLine("---- results ----");
            for (String line : summary().split("\n")) telemetry.addLine(line);
            telemetry.addLine("Dpad L/R test | B redo | Y log summary");
            telemetry.update();
        }
    }

    /** Instruction text for the current test (each format takes its own argument type). */
    private String instructions() {
        String f = INSTRUCTIONS[test.ordinal()];
        switch (test) {
            case FORWARD:
            case STRAFE: return String.format(Locale.US, f, TEST_DISTANCE_IN);
            case HEADING: return String.format(Locale.US, f, HEADING_TURNS);
            default: return f;
        }
    }

    private void select(Test t) {
        test = t;
        zeroed = false;
        lastMessage = "";
    }

    private void trackHeading(double h) {
        double d = h - lastHeading;
        while (d > Math.PI) d -= 2 * Math.PI;
        while (d < -Math.PI) d += 2 * Math.PI;
        unwrappedHeading += d;
        lastHeading = h;
    }

    private void record(Pose p) {
        double[] r = {p.x(), p.y(), Math.toDegrees(unwrappedHeading)};
        Test recorded = test; // select() below moves on to the next test
        switch (test) {
            case FORWARD: forward = r; select(Test.STRAFE); break;
            case STRAFE: strafe = r; select(Test.SPIN); break;
            case SPIN:
                if (Math.abs(r[2] - 180) > 10) {
                    lastMessage = String.format(Locale.US, "spin read %.1f deg, not ~180: not used, redo it", r[2]);
                    break;
                }
                spins.add(r);
                if (spins.size() >= SPIN_REPEATS) select(Test.HEADING);
                break;
            case HEADING: heading = r; select(Test.LOOP); break;
            case LOOP: loop = r; break;
        }
        String row = String.format(Locale.US, "%d,%s,%.3f,%.3f,%.2f,%.4f,%.4f", System.currentTimeMillis(), recorded, r[0], r[1], r[2],
                Constants.localizerConfig.xPodOffset.get(), Constants.localizerConfig.yPodOffset.get());
        appendCsv(row);
        log(String.format(Locale.US, "recorded %s: x %.3f y %.3f heading %.2f deg", recorded, r[0], r[1], r[2]));
        if (lastMessage.isEmpty()) lastMessage = "recorded";
    }

    /** All results so far and the suggested constant changes. */
    private String summary() {
        StringBuilder sb = new StringBuilder();
        double xOff = Constants.localizerConfig.xPodOffset.get(), yOff = Constants.localizerConfig.yPodOffset.get();
        double fwdScale = Double.NaN, strScale = Double.NaN;
        if (forward != null) {
            fwdScale = forward[0] / TEST_DISTANCE_IN;
            sb.append(String.format(Locale.US, "FORWARD: read %.2f in for %.0f (%+.2f%%), sideways creep %.2f in, heading %.1f deg\n",
                    forward[0], TEST_DISTANCE_IN, (fwdScale - 1) * 100, forward[1], forward[2]));
        }
        if (strafe != null) {
            strScale = strafe[1] / TEST_DISTANCE_IN;
            sb.append(String.format(Locale.US, "STRAFE: read %.2f in for %.0f (%+.2f%%), forward creep %.2f in, heading %.1f deg\n",
                    strafe[1], TEST_DISTANCE_IN, (strScale - 1) * 100, strafe[0], strafe[2]));
        }
        if (!spins.isEmpty()) {
            double mx = 0, my = 0;
            for (double[] s : spins) { mx += s[0]; my += s[1]; }
            mx /= spins.size();
            my /= spins.size();
            double scatter = 0;
            for (double[] s : spins) scatter = Math.max(scatter, Math.hypot(s[0] - mx, s[1] - my));
            // After a 180 deg spin about the robot center, the remaining offset errors show as twice their
            // size: same rule Pedro's PinpointTuner uses (x offset from -y / 2, y offset from -x / 2).
            double dX = -my / 2, dY = -mx / 2;
            sb.append(String.format(Locale.US, "SPIN x%d: moved avg (%.2f, %.2f) in, scatter %.2f in%s\n", spins.size(), mx, my, scatter,
                    scatter > 0.75 ? "  (TOO SCATTERED: spin more carefully)" : ""));
            sb.append(String.format(Locale.US, "  -> xPodOffset %.3f -> %.3f, yPodOffset %.3f -> %.3f%s\n", xOff, xOff + dX, yOff, yOff + dY,
                    Math.hypot(dX, dY) < 0.15 ? " (already good)" : ""));
        }
        if (heading != null) {
            double err = heading[2] - 360.0 * HEADING_TURNS;
            sb.append(String.format(Locale.US, "HEADING: %.1f deg over %d turns (%+.2f deg per turn), moved (%.2f, %.2f) in%s\n",
                    err, HEADING_TURNS, err / HEADING_TURNS, heading[0], heading[1],
                    Math.abs(err / HEADING_TURNS) > 0.5 ? "  (heading drift: check the Pinpoint IMU / mounting)" : ""));
        }
        if (loop != null) {
            sb.append(String.format(Locale.US, "LOOP: ended %.2f in from the mark (x %.2f, y %.2f), heading %.1f deg\n",
                    Math.hypot(loop[0], loop[1]), loop[0], loop[1], loop[2] - 360 * Math.round(loop[2] / 360)));
        }
        if (!Double.isNaN(fwdScale) && !Double.isNaN(strScale)) {
            double avg = (fwdScale + strScale) / 2;
            if (Math.abs(avg - 1) > 0.005) {
                sb.append(String.format(Locale.US, "  distance scale off %+.2f%%: custom resolution %.4f ticks/in (encoderResolutionUnit INCH, ticksPerUnit)\n",
                        (avg - 1) * 100, DEFAULT_TICKS_PER_MM * 25.4 * avg));
            }
            if (Math.abs(fwdScale - strScale) > 0.01) sb.append("  forward and strafe scales differ: check a pod's wheel / spring / mounting\n");
        }
        if (sb.length() == 0) sb.append("no results yet\n");
        return sb.toString().trim();
    }

    private void appendCsv(String row) {
        boolean newFile = !csv.exists();
        try (FileWriter w = new FileWriter(csv, true)) {
            if (newFile) w.write("t_ms,test,x_in,y_in,heading_deg,x_pod_offset,y_pod_offset\n");
            w.write(row + "\n");
        } catch (IOException e) {
            Log.w(TAG, "csv write failed: " + e);
        }
    }

    private void log(String s) {
        Log.i(TAG, s);
    }
}
