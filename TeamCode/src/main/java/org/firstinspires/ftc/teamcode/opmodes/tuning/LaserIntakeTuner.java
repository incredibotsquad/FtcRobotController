package org.firstinspires.ftc.teamcode.opmodes.tuning;

import android.util.Log;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LaserBallCounter;
import org.firstinspires.ftc.teamcode.subsystems.LaserBallCounter.Params;
import org.firstinspires.ftc.teamcode.subsystems.LaunchGateSubsystem;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;

/**
 * Automated tuning of the INTAKE count (laser sensor 1). Sensor 4 is recorded too, for reference
 * and for the launch tuning that comes next. Sensors 2/3 aren't used.
 *
 * Each trial: pick how many balls you'll feed (1-4), press A, feed them. The intake and transfer
 * run as in a match (gate closed). The trial ends by itself once sensor 1 has been still for
 * QUIET_MS after the first movement (or A again, or TRIAL_TIMEOUT_MS). The raw distances of both
 * sensors are recorded at the loop rate. Then hold LT to spit the balls back out and do the next
 * trial. Feed the balls the way they come in a match: one at a time, in quick succession, touching.
 *
 * After every trial a background search replays ALL recorded trials through LaserBallCounter with
 * every combination of the GRID_* values and picks the set whose counts match the balls you fed
 * best. An undercount costs UNDER_WEIGHT, an overcount OVER_WEIGHT (an undercount can let in a 5th
 * artifact, G407). Ties go to the set closest to the current values. The real "jam = full" rule
 * (sensor 1 blocked IntakeSubsystem.JAM_MS while intaking) is replayed too, since the robot uses it;
 * a jam in a trial with fewer than MAX balls is a false "full" and is shown.
 * The best set is also checked at slower loop rates (CHECK_LOOP_MS) to show how much a slow match
 * loop would hurt (see TODO.md, loop time).
 *
 * Saved trials: every trial is written to the CSV below, so RB loads ALL trials from every earlier
 * session (and this one) and searches over them together; trials recorded afterwards add to them.
 * Trials are labelled session.trial (e.g. 3.5 = session 3, trial 5). B discards are saved too, so
 * they stay out on later loads. Discards made before loading existed (sessions before 2026-11-16)
 * weren't saved: list those trials in EXCLUDE_TRIALS (e.g. "1.4, 2.7"). Reinstalling the app doesn't delete the
 * CSV (it's on shared storage); back it up with adb pull anyway.
 *
 * Y applies the best set live (LaserBallCounter's static values, plus IntakeSubsystem's sensor-1
 * duration rule and both baselines) until the app restarts, and logs the values to copy into the
 * code (logcat tag LaserIntakeTuner). Every sample is appended to
 * /sdcard/FIRST/data/laserIntakeTrials.csv.
 *
 * Keep both sensors clear during INIT: the baselines are measured then.
 *
 * Gamepad 1:
 *   DPAD UP/DOWN  balls for the next trial (1-4)
 *   A             start a trial / end it early
 *   LT (hold)     eject (intake + transfer reverse), between trials
 *   B             discard the last trial (saved, so it stays out on later loads)
 *   RB            load all saved trials (every session) and search over them
 *   X             re-run the search
 *   Y             apply the best values live
 *   BACK          re-capture both baselines (sensors clear)
 */
@Configurable
@Disabled // pit tool, re-enable to use (TODO.md: disabled before competition)
@TeleOp(name = "LaserIntakeTuner", group = "Tests")
public class LaserIntakeTuner extends LinearOpMode {

    public static double INTAKE_POWER = 1.0;
    public static double TRANSFER_POWER = 1.0;
    public static double EJECT_POWER = 1.0;

    /** Trial ends once sensor 1 has moved less than ACTIVITY_MM for this long (after the first movement). */
    public static long QUIET_MS = 2500;
    public static double ACTIVITY_MM = 3;
    public static long TRIAL_TIMEOUT_MS = 30000;

    /**
     * Loop period (ms) the search replays at: each step takes the latest recorded sample, like a
     * robot loop. 0 = every recorded sample (this OpMode's fast loop). Set it to the match loop time
     * once that's measured.
     */
    public static long TUNE_LOOP_MS = 0;
    /** The best and current values are also scored at these loop periods. */
    public static long[] CHECK_LOOP_MS = {20, 25};

    public static double UNDER_WEIGHT = 2;
    public static double OVER_WEIGHT = 1;

    public static double[] GRID_ENTER_MM = {6, 8, 10, 12, 15};
    public static double[] GRID_EXIT_MM = {3, 4, 6, 8};
    public static double[] GRID_SEPARATION_MM = {6, 8, 10, 12, 15};
    public static double[] GRID_DEEP_MM = {10, 15, 20, 25};
    public static long[] GRID_EXIT_HOLD_MS = {20, 30, 50};
    /** 0 = duration rule off. */
    public static long[] GRID_LONG_MS = {0, 85, 100, 115, 130, 150};

    /** Saved trials to leave out when loading, as session.trial labels, comma-separated (e.g. "1.4, 2.7"). */
    public static String EXCLUDE_TRIALS = "";
    /** Threads for the search (the Control Hub has 4 cores; leave some for the OpMode). */
    public static int SEARCH_THREADS = 2;

    private static final String TAG = "LaserIntakeTuner";
    private static final int MAX_SAMPLES = 20000;
    private static final int SHOWN_TRIALS = 8;
    private static final double TRIGGER = 0.3;

    /** One recorded trial (immutable once finished). */
    private static final class Trial {
        int number;
        int session;
        String label; // session.trial
        int expected;
        double baseline1, baseline4;
        int[] t;       // ms since trial start
        float[] mm1, mm4;
        int n;
        int liveCount; // what the live values counted during the trial
        boolean s4BlockedAtEnd;
        double avgLoopMs;
    }

    /** Search output. */
    private static final class TuneResult {
        int trials;
        Params best, current;
        double bestCost, currentCost;
        int[] bestCounts, currentCounts;
        boolean[] bestJam;
        int plateau, combos;
        long tookMs;
        String[] checks;
    }

    private final List<Trial> trials = new ArrayList<>();
    private final Object trialsLock = new Object();

    private volatile boolean tuneRequested = false;
    private volatile boolean loadRequested = false;
    private volatile String loadStatus = null;
    private int currentSession = 0;
    private volatile boolean stopWorker = false;
    private volatile int progressDone, progressTotal;
    private volatile TuneResult result;

    private AnalogInput sensor1, sensor4;
    private MotorEx intakeMotor, transferMotor;
    private File csvFile;

    @Override
    public void runOpMode() {
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        sensor1 = hardwareMap.get(AnalogInput.class, "ballSensor1");
        sensor4 = hardwareMap.get(AnalogInput.class, "ballSensor4");
        intakeMotor = new MotorEx(hardwareMap, "intakeMotor", Motor.GoBILDA.BARE);
        intakeMotor.setInverted(true);
        intakeMotor.setRunMode(Motor.RunMode.RawPower);
        transferMotor = new MotorEx(hardwareMap, "transferMotor", Motor.GoBILDA.RPM_435);
        transferMotor.setInverted(true);
        transferMotor.setRunMode(Motor.RunMode.RawPower);
        LaunchGateSubsystem gate = new LaunchGateSubsystem(hardwareMap, panels);
        openCsv();

        // Baselines: average during INIT with both sensors clear.
        double sum1 = 0, sum4 = 0, sq1 = 0, sq4 = 0;
        int n = 0;
        while (opModeInInit()) {
            double a = mm(sensor1), b = mm(sensor4);
            sum1 += a; sum4 += b; sq1 += a * a; sq4 += b * b; n++;
            double m1 = sum1 / n, m4 = sum4 / n;
            panels.addLine("LaserIntakeTuner: keep sensors 1 and 4 CLEAR (measuring baselines)");
            panels.addData("Sensor 1", String.format("%.1f mm, baseline %.1f ±%.1f", a, m1, Math.sqrt(Math.max(0, sq1 / n - m1 * m1))));
            panels.addData("Sensor 4", String.format("%.1f mm, baseline %.1f ±%.1f", b, m4, Math.sqrt(Math.max(0, sq4 / n - m4 * m4))));
            panels.update(telemetry);
        }
        if (n == 0) { sum1 = mm(sensor1); sum4 = mm(sensor4); n = 1; }
        double baseline1 = sum1 / n, baseline4 = sum4 / n;
        appendCsv(String.format("# session %s baseline1=%.1f baseline4=%.1f", new java.util.Date(), baseline1, baseline4));
        appendCsv("trial,expected,t_ms,mm1,mm4");
        currentSession = countSessions();
        Log.i(TAG, String.format("baselines: sensor1 %.1f mm, sensor4 %.1f mm", baseline1, baseline4));
        if (!opModeIsActive()) return;

        gate.closeGate();

        Thread worker = new Thread(this::workerLoop, "LaserIntakeTunerSearch");
        worker.start();

        // Live monitors (live values): baseline tracking and blocked state between trials.
        LaserBallCounter mon1 = new LaserBallCounter("mon1", baseline1, null);
        LaserBallCounter mon4 = new LaserBallCounter("mon4", baseline4, null);

        int expected = 1;
        boolean recording = false;
        String message = "Pick the ball count (DPAD), press A, feed the balls.";
        long start = 0, lastActivityMs = 0, lastLoopMs = 0;
        boolean sawActivity = false;
        double activityRef = 0, loopSum = 0;
        int loops = 0;
        int trialNumber = 0;
        int[] t = new int[MAX_SAMPLES];
        float[] r1 = new float[MAX_SAMPLES], r4 = new float[MAX_SAMPLES];
        int samples = 0;
        LaserBallCounter live = null;
        double trialBase1 = 0, trialBase4 = 0;
        boolean jam = false;

        try {
            while (opModeIsActive()) {
                long now = nowMs();
                double a = mm(sensor1), b = mm(sensor4);
                mon1.step(a, now);
                mon4.step(b, now);

                if (!recording) {
                    if (gamepad1.dpadUpWasPressed()) expected = Math.min(IntakeSubsystem.MAX_ARTIFACTS, expected + 1);
                    if (gamepad1.dpadDownWasPressed()) expected = Math.max(1, expected - 1);
                    if (gamepad1.bWasPressed()) {
                        synchronized (trialsLock) {
                            if (!trials.isEmpty()) {
                                Trial removed = trials.remove(trials.size() - 1);
                                appendCsv("# discard " + removed.label);
                                message = "Discarded trial #" + removed.label;
                                Log.i(TAG, message);
                            }
                        }
                        tuneRequested = true;
                    }
                    if (gamepad1.xWasPressed()) tuneRequested = true;
                    if (gamepad1.rightBumperWasPressed()) {
                        loadRequested = true;
                        message = "Loading saved trials...";
                    }
                    if (gamepad1.yWasPressed()) message = applyBest(baseline1, baseline4);
                    if (gamepad1.backWasPressed()) {
                        baseline1 = a;
                        baseline4 = b;
                        mon1 = new LaserBallCounter("mon1", baseline1, null);
                        mon4 = new LaserBallCounter("mon4", baseline4, null);
                        message = String.format("Baselines re-captured: %.1f / %.1f mm", baseline1, baseline4);
                        Log.i(TAG, message);
                    }

                    if (gamepad1.left_trigger > TRIGGER) {
                        setMotors(-EJECT_POWER, -EJECT_POWER);
                    } else {
                        setMotors(0, 0);
                        if (gamepad1.aWasPressed()) {
                            if (mon1.isBlocked() || mon4.isBlocked()) {
                                message = "Sensor " + (mon1.isBlocked() ? "1" : "4") + " is blocked: empty the robot (LT) first.";
                            } else {
                                recording = true;
                                trialNumber++;
                                start = now;
                                lastActivityMs = now;
                                lastLoopMs = now;
                                sawActivity = false;
                                activityRef = a;
                                loopSum = 0;
                                loops = 0;
                                samples = 0;
                                jam = false;
                                trialBase1 = mon1.getBaselineMm();
                                trialBase4 = mon4.getBaselineMm();
                                live = new LaserBallCounter("live", trialBase1, null);
                                live.setDurationRule(IntakeSubsystem.SENSOR1_DURATION_RULE);
                                setMotors(INTAKE_POWER, TRANSFER_POWER);
                                message = "Trial #" + trialNumber + ": feed " + expected + " ball(s) now.";
                                Log.i(TAG, message);
                            }
                        }
                    }
                } else {
                    // Recording
                    int ms = (int) (now - start);
                    if (samples < MAX_SAMPLES) {
                        t[samples] = ms;
                        r1[samples] = (float) a;
                        r4[samples] = (float) b;
                        samples++;
                    }
                    live.step(a, ms);
                    if (!jam && live.isBlocked() && live.msInState(ms) >= IntakeSubsystem.JAM_MS) jam = true;
                    if (Math.abs(a - activityRef) >= ACTIVITY_MM) {
                        activityRef = a;
                        lastActivityMs = now;
                        sawActivity = true;
                    }
                    loopSum += now - lastLoopMs;
                    lastLoopMs = now;
                    loops++;

                    boolean quiet = sawActivity && now - lastActivityMs >= QUIET_MS;
                    boolean timedOut = now - start >= TRIAL_TIMEOUT_MS || samples >= MAX_SAMPLES;
                    if (gamepad1.aWasPressed() || quiet || timedOut) {
                        setMotors(0, 0);
                        recording = false;
                        Trial tr = new Trial();
                        tr.number = trialNumber;
                        tr.session = currentSession;
                        tr.label = currentSession + "." + trialNumber;
                        tr.expected = expected;
                        tr.baseline1 = trialBase1;
                        tr.baseline4 = trialBase4;
                        tr.n = samples;
                        tr.t = Arrays.copyOf(t, samples);
                        tr.mm1 = Arrays.copyOf(r1, samples);
                        tr.mm4 = Arrays.copyOf(r4, samples);
                        tr.liveCount = finalCount(live.getTotalSeen(), jam);
                        tr.s4BlockedAtEnd = mon4.isBlocked();
                        tr.avgLoopMs = loops > 0 ? loopSum / loops : 0;
                        writeTrial(tr); // before adding, so a load running now can't miss it and then duplicate it
                        synchronized (trialsLock) {
                            trials.add(tr);
                        }
                        message = String.format("Trial #%s done (%s): %d fed, live count %d%s. Hold LT to eject.",
                                tr.label, quiet ? "quiet" : timedOut ? "timeout" : "A", tr.expected, tr.liveCount,
                                tr.liveCount == tr.expected ? " OK" : " WRONG");
                        Log.i(TAG, message + String.format(" loop %.1f ms, %d samples", tr.avgLoopMs, tr.n));
                        tuneRequested = true;
                    }
                }

                // ------------------------------------------------ telemetry
                panels.addLine(message);
                panels.addData("Next trial", expected + " ball(s)  (DPAD)");
                if (recording) {
                    panels.addData("RECORDING", String.format("%.1f s, live count %d%s, %d samples", (now - start) / 1000.0, finalCount(live.getTotalSeen(), jam), jam ? " (jam)" : "", samples));
                }
                panels.addData("Sensor 1", String.format("%.0f mm (base %.0f) %s", a, mon1.getBaselineMm(), mon1.isBlocked() ? "BLOCKED" : "clear"));
                panels.addData("Sensor 4", String.format("%.0f mm (base %.0f) %s", b, mon4.getBaselineMm(), mon4.isBlocked() ? "BLOCKED" : "clear"));
                showResults(panels);
                if (loadStatus != null) panels.addData("Saved trials", loadStatus);
                panels.addLine("A trial | LT eject | B discard | RB load saved | X search | Y apply | BACK baselines");
                panels.update(telemetry);
            }
        } finally {
            setMotors(0, 0);
            stopWorker = true;
            worker.interrupt();
        }
    }

    // ---------------------------------------------------------------- display

    private void showResults(TelemetryManager panels) {
        List<Trial> snapshot;
        synchronized (trialsLock) {
            snapshot = new ArrayList<>(trials);
        }
        TuneResult r = result;
        int[] perCount = new int[IntakeSubsystem.MAX_ARTIFACTS + 1];
        for (Trial tr : snapshot) perCount[Math.min(tr.expected, perCount.length - 1)]++;
        panels.addData("Trials", String.format("%d  (1:%d 2:%d 3:%d 4:%d)", snapshot.size(), perCount[1], perCount[2], perCount[3], perCount[4]));

        if (tuneRequested || progressTotal > 0 && progressDone < progressTotal) {
            panels.addData("Search", String.format("running %d/%d", progressDone, progressTotal));
        }
        if (r != null) {
            panels.addData("Current", String.format("%s", r.current));
            panels.addData("  current score", score(r.currentCounts, r.trials, snapshot) + String.format(" (cost %.0f)", r.currentCost));
            panels.addData("BEST", String.format("%s", r.best));
            panels.addData("  best score", score(r.bestCounts, r.trials, snapshot) + String.format(" (cost %.0f, %d of %d sets tie, %d ms)", r.bestCost, r.plateau, r.combos, r.tookMs));
            for (String c : r.checks) panels.addData("  slower loop", c);
        }
        int from = Math.max(0, snapshot.size() - SHOWN_TRIALS);
        for (int i = snapshot.size() - 1; i >= from; i--) {
            Trial tr = snapshot.get(i);
            String best = r != null && i < r.trials
                    ? " | best " + r.bestCounts[i] + (r.bestJam[i] && tr.expected < IntakeSubsystem.MAX_ARTIFACTS ? " FALSE JAM" : "")
                    : "";
            panels.addData("#" + tr.label, String.format("%d fed: live %d%s%s | s4 end %s | loop %.1f ms", tr.expected, tr.liveCount, tr.liveCount == tr.expected ? " ok" : " X", best, tr.s4BlockedAtEnd ? "blocked" : "clear", tr.avgLoopMs));
        }
    }

    private static String score(int[] counts, int n, List<Trial> trials) {
        int exact = 0, over = 0, under = 0;
        for (int i = 0; i < n && i < trials.size(); i++) {
            int err = counts[i] - trials.get(i).expected;
            if (err == 0) exact++; else if (err > 0) over++; else under++;
        }
        return String.format("%d/%d exact, %d over, %d under", exact, n, over, under);
    }

    private String applyBest(double baseline1, double baseline4) {
        TuneResult r = result;
        if (r == null) return "No search result yet.";
        long previousLong = LaserBallCounter.LONG_EPISODE_MS;
        r.best.applyLive();
        boolean durationRule = r.best.longEpisodeMs > 0;
        // Rule off: the flag turns it off; keep the old time so turning the flag back on is sane.
        if (!durationRule) LaserBallCounter.LONG_EPISODE_MS = previousLong;
        IntakeSubsystem.SENSOR1_DURATION_RULE = durationRule;
        IntakeSubsystem.SENSOR1_BASELINE_MM = round1(baseline1);
        IntakeSubsystem.SENSOR4_BASELINE_MM = round1(baseline4);
        Params b = r.best;
        String code = String.format("LaserBallCounter: ENTER_DEPTH_MM = %.0f; EXIT_DEPTH_MM = %.0f; SEPARATION_MM = %.0f; "
                        + "DEEP_DEPTH_MM = %.0f; EXIT_HOLD_MS = %d; LONG_EPISODE_MS = %d; "
                        + "IntakeSubsystem: SENSOR1_DURATION_RULE = %b; SENSOR1_BASELINE_MM = %.1f; SENSOR4_BASELINE_MM = %.1f",
                b.enterDepthMm, b.exitDepthMm, b.separationMm, b.deepDepthMm, b.exitHoldMs,
                LaserBallCounter.LONG_EPISODE_MS, durationRule, round1(baseline1), round1(baseline4));
        Log.i(TAG, "APPLIED (copy into code): " + code);
        return "Applied live until restart. Values logged (tag " + TAG + ").";
    }

    // ---------------------------------------------------------------- search (background thread)

    private void workerLoop() {
        while (!stopWorker) {
            if (loadRequested) {
                loadRequested = false;
                try {
                    loadSavedTrials();
                } catch (Exception e) {
                    loadStatus = "load failed: " + e;
                    Log.e(TAG, "load failed", e);
                }
                tuneRequested = true;
            } else if (tuneRequested) {
                tuneRequested = false;
                try {
                    search();
                } catch (Exception e) {
                    Log.e(TAG, "search failed", e);
                }
            } else {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException e) {
                    return;
                }
            }
        }
    }

    private void search() {
        List<Trial> snapshot;
        synchronized (trialsLock) {
            snapshot = new ArrayList<>(trials);
        }
        if (snapshot.isEmpty()) {
            result = null;
            return;
        }
        long t0 = nowMs();
        Params current = Params.live();
        if (!IntakeSubsystem.SENSOR1_DURATION_RULE) current.longEpisodeMs = 0;

        List<Params> grid = new ArrayList<>();
        for (double enter : GRID_ENTER_MM)
            for (double exit : GRID_EXIT_MM) {
                if (exit >= enter) continue;
                for (double sep : GRID_SEPARATION_MM)
                    for (long hold : GRID_EXIT_HOLD_MS)
                        for (long lng : GRID_LONG_MS)
                            for (double deep : GRID_DEEP_MM) {
                                grid.add(new Params(enter, exit, deep, sep, hold, lng, current.baselineTrackRate));
                                if (lng <= 0) break; // deep only matters with the duration rule
                            }
            }
        progressTotal = grid.size();
        progressDone = 0;

        // Score every set, split over SEARCH_THREADS threads.
        final double[] costs = new double[grid.size()];
        final AtomicInteger next = new AtomicInteger(0);
        final AtomicInteger done = new AtomicInteger(0);
        final List<Trial> data = snapshot;
        Runnable scorer = () -> {
            int i;
            while ((i = next.getAndIncrement()) < costs.length) {
                if (stopWorker || tuneRequested || loadRequested) return;
                costs[i] = cost(data, grid.get(i), TUNE_LOOP_MS);
                progressDone = done.incrementAndGet();
            }
        };
        int threads = Math.max(1, SEARCH_THREADS);
        List<Thread> pool = new ArrayList<>();
        for (int k = 1; k < threads; k++) {
            Thread th = new Thread(scorer, "LaserIntakeTunerScore" + k);
            th.start();
            pool.add(th);
        }
        scorer.run();
        for (Thread th : pool) {
            try {
                th.join();
            } catch (InterruptedException e) {
                return;
            }
        }
        if (stopWorker || tuneRequested || loadRequested || done.get() < costs.length) return; // new data: start over

        Params best = null;
        double bestCost = Double.MAX_VALUE, bestDist = Double.MAX_VALUE;
        int plateau = 0;
        for (int i = 0; i < costs.length; i++) {
            Params p = grid.get(i);
            double cost = costs[i];
            double dist = distance(p, current);
            if (cost < bestCost - 1e-9) {
                bestCost = cost;
                best = p;
                bestDist = dist;
                plateau = 1;
            } else if (Math.abs(cost - bestCost) < 1e-9) {
                plateau++;
                if (dist < bestDist) {
                    best = p;
                    bestDist = dist;
                }
            }
        }

        TuneResult r = new TuneResult();
        r.trials = snapshot.size();
        r.best = best;
        r.current = current;
        r.bestCost = bestCost;
        r.currentCost = cost(snapshot, current, TUNE_LOOP_MS);
        r.bestCounts = new int[r.trials];
        r.bestJam = new boolean[r.trials];
        r.currentCounts = new int[r.trials];
        for (int i = 0; i < r.trials; i++) {
            boolean[] jam = new boolean[1];
            r.bestCounts[i] = replay(snapshot.get(i), best, TUNE_LOOP_MS, jam);
            r.bestJam[i] = jam[0];
            r.currentCounts[i] = replay(snapshot.get(i), current, TUNE_LOOP_MS, null);
        }
        r.plateau = plateau;
        r.combos = grid.size();
        r.checks = new String[CHECK_LOOP_MS.length];
        for (int k = 0; k < CHECK_LOOP_MS.length; k++) {
            long step = CHECK_LOOP_MS[k];
            int[] bc = new int[r.trials], cc = new int[r.trials];
            for (int i = 0; i < r.trials; i++) {
                bc[i] = replay(snapshot.get(i), best, step, null);
                cc[i] = replay(snapshot.get(i), current, step, null);
            }
            r.checks[k] = step + " ms: best " + score(bc, r.trials, snapshot) + "; current " + score(cc, r.trials, snapshot);
        }
        r.tookMs = nowMs() - t0;
        result = r;
        Log.i(TAG, String.format("search over %d trials, %d sets, %d ms: best [%s] %s; current [%s] %s",
                r.trials, r.combos, r.tookMs, best, score(r.bestCounts, r.trials, snapshot),
                current, score(r.currentCounts, r.trials, snapshot)));
    }

    private static double cost(List<Trial> trials, Params p, long stepMs) {
        double cost = 0;
        for (Trial tr : trials) {
            int err = replay(tr, p, stepMs, null) - tr.expected;
            cost += err > 0 ? err * OVER_WEIGHT : -err * UNDER_WEIGHT;
        }
        return cost;
    }

    /** Count the robot would end the trial with, using {@code p}, sampling every stepMs (0 = every sample). */
    private static int replay(Trial tr, Params p, long stepMs, boolean[] jamOut) {
        LaserBallCounter c = new LaserBallCounter("replay", tr.baseline1, p);
        c.setDurationRule(p.longEpisodeMs > 0);
        boolean jam = false;
        long jamMs = IntakeSubsystem.JAM_MS;
        if (stepMs <= 0) {
            for (int i = 0; i < tr.n; i++) {
                c.step(tr.mm1[i], tr.t[i]);
                if (!jam && c.isBlocked() && c.msInState(tr.t[i]) >= jamMs) jam = true;
            }
        } else {
            int i = 0;
            int end = tr.t[tr.n - 1];
            for (long t = 0; t <= end; t += stepMs) {
                while (i + 1 < tr.n && tr.t[i + 1] <= t) i++;
                c.step(tr.mm1[i], t);
                if (!jam && c.isBlocked() && c.msInState(t) >= jamMs) jam = true;
            }
        }
        if (jamOut != null) jamOut[0] = jam;
        return finalCount(c.getTotalSeen(), jam);
    }

    /** As IntakeSubsystem: clamped to MAX, and a jam at sensor 1 means full. */
    private static int finalCount(int seen, boolean jam) {
        int max = IntakeSubsystem.MAX_ARTIFACTS;
        return jam ? max : Math.min(max, seen);
    }

    /** Rough normalized distance between two sets, for breaking ties toward the current values. */
    private static double distance(Params a, Params b) {
        return Math.abs(a.enterDepthMm - b.enterDepthMm) / 10 + Math.abs(a.exitDepthMm - b.exitDepthMm) / 5
                + Math.abs(a.separationMm - b.separationMm) / 10 + Math.abs(a.deepDepthMm - b.deepDepthMm) / 15
                + Math.abs(a.exitHoldMs - b.exitHoldMs) / 30.0 + Math.abs(a.longEpisodeMs - b.longEpisodeMs) / 100.0;
    }

    // ---------------------------------------------------------------- hardware and files

    private void setMotors(double intake, double transfer) {
        if (intakeMotor.get() != intake) intakeMotor.set(intake);
        if (transferMotor.get() != transfer) transferMotor.set(transfer);
    }

    private static double mm(AnalogInput sensor) {
        return LaserBallCounter.voltsToMm(sensor.getVoltage());
    }

    private static double round1(double v) {
        return Math.round(v * 10) / 10.0;
    }

    private void openCsv() {
        try {
            File dir = AppUtil.ROBOT_DATA_DIR;
            //noinspection ResultOfMethodCallIgnored
            dir.mkdirs();
            csvFile = new File(dir, "laserIntakeTrials.csv");
        } catch (Exception e) {
            Log.w(TAG, "can't open csv: " + e);
            csvFile = null;
        }
    }

    private void writeTrial(Trial tr) {
        if (csvFile == null) return;
        try (FileWriter w = new FileWriter(csvFile, true)) {
            w.write(String.format("# trial %d expected=%d baseline1=%.1f baseline4=%.1f live=%d loop=%.1fms s4end=%d\n",
                    tr.number, tr.expected, tr.baseline1, tr.baseline4, tr.liveCount, tr.avgLoopMs, tr.s4BlockedAtEnd ? 1 : 0));
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < tr.n; i++) {
                sb.setLength(0);
                sb.append(tr.number).append(',').append(tr.expected).append(',').append(tr.t[i]).append(',')
                        .append(Math.round(tr.mm1[i] * 10) / 10.0).append(',').append(Math.round(tr.mm4[i] * 10) / 10.0).append('\n');
                w.write(sb.toString());
            }
        } catch (IOException e) {
            Log.w(TAG, "csv write failed: " + e);
        }
    }

    /** Number of "# session" headers in the CSV (this session's is the last one). */
    private int countSessions() {
        if (csvFile == null || !csvFile.exists()) return 0;
        int sessions = 0;
        try (BufferedReader r = new BufferedReader(new FileReader(csvFile))) {
            String line;
            while ((line = r.readLine()) != null) {
                if (line.startsWith("# session")) sessions++;
            }
        } catch (IOException e) {
            Log.w(TAG, "can't read csv: " + e);
        }
        return sessions;
    }

    /** Replaces the trial list with every trial in the CSV, minus saved discards and EXCLUDE_TRIALS. */
    private void loadSavedTrials() throws IOException {
        if (csvFile == null || !csvFile.exists()) {
            loadStatus = "no saved trials file";
            return;
        }
        Set<String> excluded = new HashSet<>();
        for (String label : EXCLUDE_TRIALS.split(",")) {
            if (!label.trim().isEmpty()) excluded.add(label.trim());
        }
        List<Trial> loaded = new ArrayList<>();
        int session = 0;
        Trial cur = null;
        int[] t = new int[MAX_SAMPLES];
        float[] r1 = new float[MAX_SAMPLES], r4 = new float[MAX_SAMPLES];
        int n = 0;
        try (BufferedReader r = new BufferedReader(new FileReader(csvFile))) {
            String line;
            while ((line = r.readLine()) != null) {
                if (line.isEmpty()) continue;
                if (line.charAt(0) == '#') {
                    if (line.startsWith("# discard ")) {
                        excluded.add(line.substring("# discard ".length()).trim());
                        continue;
                    }
                    if (line.startsWith("# session") || line.startsWith("# trial ")) {
                        if (finishLoaded(cur, t, r1, r4, n)) loaded.add(cur);
                        cur = null;
                        n = 0;
                    }
                    if (line.startsWith("# session")) {
                        session++;
                    } else if (line.startsWith("# trial ")) {
                        cur = parseTrialHeader(line, session);
                    }
                    continue;
                }
                if (cur == null || !Character.isDigit(line.charAt(0)) || n >= MAX_SAMPLES) continue;
                String[] f = line.split(",");
                if (f.length < 5) continue;
                try {
                    t[n] = Integer.parseInt(f[2]);
                    r1[n] = Float.parseFloat(f[3]);
                    r4[n] = Float.parseFloat(f[4]);
                    n++;
                } catch (NumberFormatException ignored) {
                    // skip a malformed row
                }
            }
        }
        if (finishLoaded(cur, t, r1, r4, n)) loaded.add(cur);

        List<Trial> kept = new ArrayList<>();
        int dropped = 0;
        for (Trial tr : loaded) {
            if (excluded.contains(tr.label)) dropped++;
            else kept.add(tr);
        }
        int[] perCount = new int[IntakeSubsystem.MAX_ARTIFACTS + 1];
        for (Trial tr : kept) perCount[Math.min(tr.expected, perCount.length - 1)]++;
        synchronized (trialsLock) {
            trials.clear();
            trials.addAll(kept);
        }
        loadStatus = String.format("loaded %d trials from %d sessions (1:%d 2:%d 3:%d 4:%d), %d discarded/excluded",
                kept.size(), session, perCount[1], perCount[2], perCount[3], perCount[4], dropped);
        Log.i(TAG, loadStatus);
    }

    private static Trial parseTrialHeader(String line, int session) {
        Trial tr = new Trial();
        tr.session = session;
        String[] words = line.substring(2).trim().split("\\s+"); // "trial N key=value ..."
        try {
            tr.number = Integer.parseInt(words[1]);
        } catch (Exception e) {
            return null;
        }
        tr.label = session + "." + tr.number;
        tr.expected = -1;
        for (String w : words) {
            int eq = w.indexOf('=');
            if (eq < 0) continue;
            String key = w.substring(0, eq), value = w.substring(eq + 1).replace("ms", "");
            try {
                switch (key) {
                    case "expected": tr.expected = Integer.parseInt(value); break;
                    case "baseline1": tr.baseline1 = Double.parseDouble(value); break;
                    case "baseline4": tr.baseline4 = Double.parseDouble(value); break;
                    case "live": tr.liveCount = Integer.parseInt(value); break;
                    case "loop": tr.avgLoopMs = Double.parseDouble(value); break;
                    case "s4end": tr.s4BlockedAtEnd = value.equals("1"); break;
                    default: break;
                }
            } catch (NumberFormatException ignored) {
                // leave the default
            }
        }
        return tr.expected > 0 && tr.baseline1 > 0 ? tr : null;
    }

    /** Copies the samples into a loaded trial; false if there's nothing usable. */
    private static boolean finishLoaded(Trial tr, int[] t, float[] r1, float[] r4, int n) {
        if (tr == null || n == 0) return false;
        tr.n = n;
        tr.t = Arrays.copyOf(t, n);
        tr.mm1 = Arrays.copyOf(r1, n);
        tr.mm4 = Arrays.copyOf(r4, n);
        return true;
    }

    private void appendCsv(String line) {
        if (csvFile == null) return;
        try (FileWriter w = new FileWriter(csvFile, true)) {
            w.write(line);
            w.write('\n');
        } catch (IOException e) {
            Log.w(TAG, "csv write failed: " + e);
        }
    }

    private static long nowMs() {
        return System.nanoTime() / 1_000_000L;
    }
}
