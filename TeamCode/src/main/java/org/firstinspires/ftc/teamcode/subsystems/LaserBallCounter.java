package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Counts balls passing a goBILDA Laser Distance Sensor in analog mode (0-3.3 V = 0-1000 mm).
 * The algorithm and starting values come from the laser tests of 2026-11-11; LaserIntakeTuner
 * re-tunes them from recorded intake trials.
 *
 * With nothing in front the sensor reads the background (baseline); a ball reads closer.
 * Depth = baseline - distance.
 *  - Episode: starts at depth >= ENTER_DEPTH_MM, ends once depth stays below EXIT_DEPTH_MM for
 *    EXIT_HOLD_MS. The first ball of an episode is counted when it starts.
 *  - Seam (peak) rule: inside an episode the distance rising SEPARATION_MM above its lowest point
 *    and then falling SEPARATION_MM again (still >= ENTER_DEPTH_MM deep) is another, touching ball.
 *  - Duration rule (per sensor, see setDurationRule; LONG_EPISODE_MS <= 0 also turns it off): time
 *    at least DEEP_DEPTH_MM deep of LONG_EPISODE_MS or more makes the episode 2 balls, never more.
 *
 * Two running totals are kept, so the owner can pick the right moment to count:
 *  - totalSeen: balls counted as they ARRIVE (episode start, each seam, the duration rule).
 *  - totalGone: balls counted once they have LEFT the beam: at a seam the ball ahead has passed;
 *    at the episode end every ball of the episode has passed.
 *
 * Baseline: given by the owner if > 0 (measure it with the sensor clear), otherwise the first
 * reading. While clear it slowly tracks drift. A clear reading more than ENTER_DEPTH_MM farther
 * than the baseline means the baseline was taken with something in front: it jumps there.
 *
 * The static fields are the live (Panels-tunable) values. A counter built with explicit Params
 * (replay constructor) uses those instead, which is how LaserIntakeTuner tries many sets offline.
 */
@Configurable
public class LaserBallCounter {

    // Live values (sensor 1 intake values; sensor 4 currently shares them).
    public static double ENTER_DEPTH_MM = 10;
    public static double EXIT_DEPTH_MM = 6;
    public static double DEEP_DEPTH_MM = 15;
    public static long EXIT_HOLD_MS = 30;
    public static double SEPARATION_MM = 10;
    public static long LONG_EPISODE_MS = 115;
    /** Baseline tracking while clear: fraction of the error applied per update. */
    public static double BASELINE_TRACK_RATE = 0.002;
    /**
     * A clear reading farther than the baseline (by more than ENTER_DEPTH_MM) must persist this long
     * before the baseline jumps to it. A single stray far sample (seen right after START, when the
     * intake bar moves) used to move the baseline 10+ mm out, after which the normal background read
     * as "blocked" forever and the jam rule called the robot full with one ball inside.
     */
    public static long BASELINE_JUMP_MS = 250;

    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;

    /** One set of counting thresholds. */
    public static final class Params {
        public double enterDepthMm, exitDepthMm, deepDepthMm, separationMm, baselineTrackRate;
        public long exitHoldMs, longEpisodeMs;

        public Params(double enterDepthMm, double exitDepthMm, double deepDepthMm, double separationMm,
                      long exitHoldMs, long longEpisodeMs, double baselineTrackRate) {
            this.enterDepthMm = enterDepthMm;
            this.exitDepthMm = exitDepthMm;
            this.deepDepthMm = deepDepthMm;
            this.separationMm = separationMm;
            this.exitHoldMs = exitHoldMs;
            this.longEpisodeMs = longEpisodeMs;
            this.baselineTrackRate = baselineTrackRate;
        }

        /** The current live (static) values. */
        public static Params live() {
            return new Params(ENTER_DEPTH_MM, EXIT_DEPTH_MM, DEEP_DEPTH_MM, SEPARATION_MM,
                    EXIT_HOLD_MS, LONG_EPISODE_MS, BASELINE_TRACK_RATE);
        }

        /** Makes these the live values. */
        public void applyLive() {
            ENTER_DEPTH_MM = enterDepthMm;
            EXIT_DEPTH_MM = exitDepthMm;
            DEEP_DEPTH_MM = deepDepthMm;
            SEPARATION_MM = separationMm;
            EXIT_HOLD_MS = exitHoldMs;
            LONG_EPISODE_MS = longEpisodeMs;
            BASELINE_TRACK_RATE = baselineTrackRate;
        }

        @Override
        public String toString() {
            return String.format("enter %.0f, exit %.0f, sep %.0f, deep %.0f, hold %d ms, long %s",
                    enterDepthMm, exitDepthMm, separationMm, deepDepthMm, exitHoldMs,
                    longEpisodeMs > 0 ? longEpisodeMs + " ms" : "off");
        }
    }

    private final String name;
    private final AnalogInput sensor;
    /** null = use the live static values every update. */
    private final Params fixedParams;
    private Params p;
    private boolean durationRule = true;

    private double baselineMm;
    private double lastMm;
    private long lastUpdateMs = -1;
    /** Since when clear readings have been farther than the baseline (see BASELINE_JUMP_MS); -1 = not now. */
    private long fartherSinceMs = -1;

    private boolean inEpisode;
    private long episodeStartMs;
    private long belowExitSinceMs = -1;
    private long deepMs;
    private int peakBalls;
    private int episodeSeen;
    private int episodeGone;
    private double minMm;
    private double maxMm;
    private boolean rising;
    private double deepestMm;
    /** When the beam last went from clear to occupied or back (for "blocked for X ms" checks). */
    private long stateSinceMs;

    private int totalSeen;
    private int totalGone;
    private int episodes;

    /** Hardware counter with the live values. @param baselineMm clear background, or <= 0 for the first reading. */
    public LaserBallCounter(HardwareMap hardwareMap, String name, double baselineMm) {
        this.name = name;
        sensor = hardwareMap.get(AnalogInput.class, name);
        fixedParams = null;
        p = Params.live();
        lastMm = readMm();
        this.baselineMm = baselineMm > 0 ? baselineMm : lastMm;
        stateSinceMs = nowMs();
        Log.i("LaserBallCounter", String.format("%s: baseline %.1f mm (%s), reading %.1f mm", name,
                this.baselineMm, baselineMm > 0 ? "configured" : "first reading", lastMm));
    }

    /**
     * Counter without hardware, fed through step(). @param params fixed thresholds, or null for the
     * live values. Doesn't log.
     */
    public LaserBallCounter(String name, double baselineMm, Params params) {
        this.name = name;
        sensor = null;
        fixedParams = params;
        p = params != null ? params : Params.live();
        this.baselineMm = baselineMm;
        lastMm = baselineMm;
        stateSinceMs = 0;
    }

    public void setDurationRule(boolean on) {
        durationRule = on;
    }

    /** Reads the sensor and advances the counter. Call once per loop. */
    public void update(long now) {
        step(readMm(), now);
    }

    /** Advances the counter with one distance sample (mm) taken at {@code now} (ms). */
    public void step(double mm, long now) {
        if (fixedParams == null) p = Params.live();
        lastMm = mm;
        long dt = lastUpdateMs < 0 ? 0 : now - lastUpdateMs;
        lastUpdateMs = now;
        double depth = baselineMm - mm;

        if (!inEpisode) {
            if (depth >= p.enterDepthMm) {
                fartherSinceMs = -1;
                startEpisode(mm, now);
            } else if (-depth > p.enterDepthMm) {
                // Farther than the baseline: only trust it once it persists (a stray sample is ignored).
                if (fartherSinceMs < 0) fartherSinceMs = now;
                if (now - fartherSinceMs >= BASELINE_JUMP_MS) {
                    if (sensor != null) {
                        Log.i("LaserBallCounter", String.format("%s: baseline %.1f -> %.1f mm (reading farther than baseline for %d ms)",
                                name, baselineMm, mm, now - fartherSinceMs));
                    }
                    baselineMm = mm;
                    fartherSinceMs = -1;
                }
            } else {
                fartherSinceMs = -1;
                baselineMm += (mm - baselineMm) * p.baselineTrackRate;
            }
            return;
        }

        if (depth >= p.deepDepthMm) deepMs += dt;
        deepestMm = Math.max(deepestMm, depth);

        // Seam rule: rise separation above the lowest point, then fall separation again.
        if (!rising) {
            minMm = Math.min(minMm, mm);
            if (mm >= minMm + p.separationMm) {
                rising = true;
                maxMm = mm;
            }
        } else {
            maxMm = Math.max(maxMm, mm);
            if (mm <= maxMm - p.separationMm && depth >= p.enterDepthMm) {
                peakBalls++;
                rising = false;
                minMm = mm;
                addGone(peakBalls - 1 - episodeGone); // the ball ahead of the seam has passed
            }
        }

        addSeen(episodeBalls() - episodeSeen);

        if (depth < p.exitDepthMm) {
            if (belowExitSinceMs < 0) belowExitSinceMs = now;
            if (now - belowExitSinceMs >= p.exitHoldMs) finishEpisode(now);
        } else {
            belowExitSinceMs = -1;
        }
    }

    /**
     * Restarts the timing of an episode in progress as a single ball that hasn't left yet (e.g. a
     * ball resting in front of the sensor when a launch starts, so its resting time doesn't trigger
     * the duration rule). No effect if the beam is clear.
     */
    public void rebase(long now) {
        if (!inEpisode) return;
        deepMs = 0;
        peakBalls = 1;
        episodeGone = 0;
        episodeSeen = Math.min(episodeSeen, 1);
        minMm = lastMm;
        rising = false;
        belowExitSinceMs = -1;
    }

    private int episodeBalls() {
        int balls = peakBalls;
        if (durationRule && p.longEpisodeMs > 0 && deepMs >= p.longEpisodeMs) balls = Math.max(balls, 2);
        return balls;
    }

    private void startEpisode(double mm, long now) {
        inEpisode = true;
        episodes++;
        episodeStartMs = now;
        stateSinceMs = now;
        belowExitSinceMs = -1;
        deepMs = 0;
        peakBalls = 1;
        episodeSeen = 0;
        episodeGone = 0;
        minMm = mm;
        rising = false;
        deepestMm = baselineMm - mm;
        addSeen(1);
    }

    private void finishEpisode(long now) {
        inEpisode = false;
        stateSinceMs = now;
        int balls = Math.max(episodeBalls(), episodeSeen);
        addGone(balls - episodeGone);
        if (sensor != null) {
            Log.i("LaserBallCounter", String.format("%s: episode %d ball(s), seams %d, %d ms, deep %d ms, deepest %.0f mm",
                    name, balls, peakBalls - 1, belowExitSinceMs - episodeStartMs, deepMs, deepestMm));
        }
    }

    private void addSeen(int n) {
        if (n <= 0) return;
        episodeSeen += n;
        totalSeen += n;
    }

    private void addGone(int n) {
        if (n <= 0) return;
        episodeGone += n;
        totalGone += n;
    }

    /** Balls counted on arrival so far (only goes up). */
    public int getTotalSeen() {
        return totalSeen;
    }

    /** Balls counted as having passed the beam so far (only goes up). */
    public int getTotalGone() {
        return totalGone;
    }

    /** Episodes started so far (one per uninterrupted blocking, touching balls included). */
    public int getEpisodes() {
        return episodes;
    }

    /** Something is in front of the sensor (an episode is in progress). */
    public boolean isBlocked() {
        return inEpisode;
    }

    /** How long the sensor has been blocked / clear. */
    public long msInState(long now) {
        return now - stateSinceMs;
    }

    /** Reads the sensor now without advancing the counter (for INIT displays). */
    public double readDistanceMmNow() {
        return sensor != null ? readMm() : lastMm;
    }

    public double getDistanceMm() {
        return lastMm;
    }

    public double getBaselineMm() {
        return baselineMm;
    }

    private double readMm() {
        return sensor.getVoltage() / MAX_VOLTS * MAX_DISTANCE_MM;
    }

    /** Converts a raw analog reading to mm (for code that reads the AnalogInput itself). */
    public static double voltsToMm(double volts) {
        return volts / MAX_VOLTS * MAX_DISTANCE_MM;
    }

    private static long nowMs() {
        return System.nanoTime() / 1_000_000L;
    }
}
