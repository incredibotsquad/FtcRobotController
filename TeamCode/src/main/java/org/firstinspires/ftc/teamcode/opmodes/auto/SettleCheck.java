package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.math.Pose;

import java.util.Locale;

/**
 * "Has the robot stopped moving?" before an auto launch (DynamicAuto.SETTLE_BEFORE_LAUNCH). Settled
 * = the pose has stayed within SETTLE_MAX_MOVE_IN and SETTLE_MAX_TURN_DEG of where it was for
 * SETTLE_MS. Gives up after SETTLE_TIMEOUT_MS (counts as settled, so a launch is never blocked).
 * Settings are DynamicAuto's static fields (shared by LeftWallAuto and RightWallLGLFP).
 */
final class SettleCheck {
    private Pose ref = null;
    private long stillSinceMs = -1;
    private long startMs = -1;
    private double lastMoveIn = 0, lastTurnDeg = 0;

    /** Start over (arrived somewhere new, or the wait restarted). */
    void reset() {
        ref = null;
        stillSinceMs = -1;
        startMs = -1;
    }

    /** Call once per loop. True once settled, or after SETTLE_TIMEOUT_MS, or with the check off. */
    boolean update(Pose p, long nowMs) {
        if (!DynamicAuto.SETTLE_BEFORE_LAUNCH) return true;
        if (startMs < 0) startMs = nowMs;
        if (ref == null) {
            ref = p;
            stillSinceMs = nowMs;
        }
        lastMoveIn = Math.hypot(p.x() - ref.x(), p.y() - ref.y());
        double d = p.heading() - ref.heading();
        lastTurnDeg = Math.abs(Math.toDegrees(Math.atan2(Math.sin(d), Math.cos(d))));
        if (lastMoveIn > DynamicAuto.SETTLE_MAX_MOVE_IN || lastTurnDeg > DynamicAuto.SETTLE_MAX_TURN_DEG) {
            ref = p; // still moving: measure from here
            stillSinceMs = nowMs;
        }
        return nowMs - stillSinceMs >= DynamicAuto.SETTLE_MS || timedOut(nowMs);
    }

    boolean timedOut(long nowMs) {
        return DynamicAuto.SETTLE_BEFORE_LAUNCH && startMs >= 0 && nowMs - startMs >= DynamicAuto.SETTLE_TIMEOUT_MS
                && nowMs - stillSinceMs < DynamicAuto.SETTLE_MS;
    }

    /** For status / log lines. */
    String describe(long nowMs) {
        if (!DynamicAuto.SETTLE_BEFORE_LAUNCH) return "settle check off";
        if (startMs < 0) return "not checked";
        if (timedOut(nowMs)) return String.format(Locale.US, "NOT settled after %d ms (SETTLE_TIMEOUT_MS), launching anyway",
                nowMs - startMs);
        return String.format(Locale.US, "still %d ms (moved %.2f in / %.1f deg), waited %d ms",
                nowMs - stillSinceMs, lastMoveIn, lastTurnDeg, nowMs - startMs);
    }
}
