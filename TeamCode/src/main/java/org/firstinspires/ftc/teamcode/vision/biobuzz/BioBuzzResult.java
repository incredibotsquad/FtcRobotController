package org.firstinspires.ftc.teamcode.vision.biobuzz;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/** Immutable snapshot. Counts describe visible blobs, not guaranteed piece counts. */
public final class BioBuzzResult {
    public enum Kind { POLLEN, RED_NECTAR, BLUE_NECTAR }
    public enum Alliance { RED, BLUE }
    public enum Mode { POLLEN_ONLY, MIXED }
    public static final class Position {
        public final double forward, left;
        public Position(double forward, double left) { this.forward = forward; this.left = left; }
        public double distance() { return Math.hypot(forward, left); }
        public double bearingRadians() { return Math.atan2(left, forward); }
        /** Field pose must correspond to captureTimeNanos; heading is CCW from field +X. */
        public FieldPoint toField(double robotX, double robotY, double headingRadians) {
            double c = Math.cos(headingRadians), s = Math.sin(headingRadians);
            return new FieldPoint(robotX + forward*c-left*s, robotY+forward*s+left*c);
        }
    }
    public static final class FieldPoint {
        public final double x, y;
        FieldPoint(double x, double y) { this.x = x; this.y = y; }
    }
    public static final class Blob {
        public final Kind kind;
        public final int x, y, width, height;
        public final double area, u, v;
        public final boolean clipped;
        /** Null if uncalibrated, outside calibrated area, clipped, or camera not ready. */
        public final Position position;
        Blob(Kind kind, int x, int y, int width, int height, double area, double u, double v,
             boolean clipped, Position position) {
            this.kind=kind; this.x=x; this.y=y; this.width=width; this.height=height;
            this.area=area; this.u=u; this.v=v; this.clipped=clipped; this.position=position;
        }
    }
    public static final class Target {
        public final Position position;
        public final double score;
        public final int blobIndex, pollenBlobs, redNectarBlobs, blueNectarBlobs;
        Target(Position p, double score, int index, int pollen, int red, int blue) {
            position=p; this.score=score; blobIndex=index;
            pollenBlobs=pollen; redNectarBlobs=red; blueNectarBlobs=blue;
        }
    }
    public final long captureTimeNanos;
    public final List<Blob> blobs;
    /** Ranked overlapping neighborhoods, each anchored on an actual desired blob. */
    public final List<Target> targets;
    public final boolean intakeReady, truncated;
    public final double processingMs;
    BioBuzzResult(long time, List<Blob> blobs, List<Target> targets, boolean ready, boolean truncated, double ms) {
        captureTimeNanos=time; this.blobs=Collections.unmodifiableList(new ArrayList<>(blobs));
        this.targets=Collections.unmodifiableList(new ArrayList<>(targets));
        intakeReady=ready; this.truncated=truncated; processingMs=ms;
    }
    public Target best() { return targets.isEmpty() ? null : targets.get(0); }
    public int count(Kind kind) { int n=0; for (Blob b:blobs) if (b.kind==kind) n++; return n; }
    public boolean isFresh(long nowNanos, int maxAgeMs) {
        long age=nowNanos-captureTimeNanos;
        return captureTimeNanos > 0 && age >= 0 && age <= maxAgeMs*1_000_000L;
    }
    static BioBuzzResult empty() {
        return new BioBuzzResult(0, Collections.emptyList(), Collections.emptyList(), false, false, 0);
    }
}
