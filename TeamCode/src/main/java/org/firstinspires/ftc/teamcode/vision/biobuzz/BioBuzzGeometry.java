package org.firstinspires.ftc.teamcode.vision.biobuzz;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import static org.firstinspires.ftc.teamcode.vision.biobuzz.BioBuzzResult.*;

/** Geometry/ranking intentionally match tools/biobuzz/core.py. */
final class BioBuzzGeometry {
    static Position project(double u, double v, BioBuzzConfig c, double aspect) {
        if (!c.isCalibrated() || Math.abs(aspect/c.calibrationAspect-1) > .01) return null;
        double sign=0;
        for (int i=0;i<4;i++) {
            double[] a=c.polygon[i], b=c.polygon[(i+1)%4];
            double cross=(b[0]-a[0])*(v-a[1])-(b[1]-a[1])*(u-a[0]);
            if (Math.abs(cross)<1e-9) continue;
            if (sign*cross<0) return null;
            sign=cross;
        }
        double[] h=c.homography;
        double z=h[6]*u+h[7]*v+h[8];
        if (!Double.isFinite(z) || Math.abs(z)<1e-9) return null;
        double f=(h[0]*u+h[1]*v+h[2])/z, l=(h[3]*u+h[4]*v+h[5])/z;
        if (!Double.isFinite(f) || !Double.isFinite(l) || f<0 || Math.hypot(f,l)>c.maxRange) return null;
        return new Position(f,l);
    }
    static List<Target> rank(List<Blob> blobs, BioBuzzConfig c, Alliance alliance, Mode mode) {
        Kind own=alliance==Alliance.RED?Kind.RED_NECTAR:Kind.BLUE_NECTAR;
        double[] weights=new double[]{c.pollenWeight,-c.opponentPenalty,-c.opponentPenalty};
        weights[own.ordinal()]=mode==Mode.MIXED?c.nectarWeight:0;
        List<Target> targets=new ArrayList<>();
        for (int i=0;i<blobs.size();i++) {
            Blob anchor=blobs.get(i);
            if (anchor.position==null || weights[anchor.kind.ordinal()]<=0) continue;
            int[] counts=new int[3];
            for (Blob b:blobs) if (b.position!=null && Math.hypot(b.position.forward-anchor.position.forward,
                    b.position.left-anchor.position.left)<=c.radius) counts[b.kind.ordinal()]++;
            double score=-c.distancePenalty*anchor.position.distance();
            for(int j=0;j<3;j++) score+=counts[j]*weights[j];
            targets.add(new Target(anchor.position,score,i,counts[0],counts[1],counts[2]));
        }
        targets.sort(Comparator.comparingDouble((Target t)->-t.score)
                .thenComparingDouble(t->t.position.distance()).thenComparingInt(t->t.blobIndex));
        return targets;
    }
}
