package org.firstinspires.ftc.teamcode.vision.biobuzz;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import static org.firstinspires.ftc.teamcode.vision.biobuzz.BioBuzzResult.*;

/** Optional adapter for export_limelight.py's 8-number protocol.
 * Own pipeline selection externally; do not let the AprilTag subsystem switch pipelines concurrently.
 * Limelight sends the best area + whole-frame counts, not the USB processor's full blob list.
 */
public final class BioBuzzLimelight {
    public static final class Summary {
        /** Estimated capture time; use pose history for field transforms on a moving robot. */
        public final long captureTimeNanos;
        public final Position position; // null when the current frame has no metric target
        public final double score;
        public final int pollenBlobs, redNectarBlobs, blueNectarBlobs;
        private Summary(long time, double[] out, boolean valid) {
            captureTimeNanos=time; position=valid?new Position(out[1],out[2]):null;
            score=valid?out[3]:0;
            pollenBlobs=(int)out[4]; redNectarBlobs=(int)out[5]; blueNectarBlobs=(int)out[6];
        }
    }
    private final Limelight3A camera;
    private final BioBuzzConfig config;
    private final int pipeline;
    private Alliance alliance=Alliance.RED;
    private Mode mode=Mode.MIXED;
    private boolean intake=false, sentReady=false;
    private long readyAfter=Long.MAX_VALUE, sequenceSeenAt=0;
    private int request=1;
    private double lastSequence=-1;

    public BioBuzzLimelight(Limelight3A camera, int pipeline, BioBuzzConfig config) {
        if(pipeline<0 || pipeline>9) throw new IllegalArgumentException("Pipeline must be 0..9");
        this.camera=camera; this.pipeline=pipeline; this.config=config;
        // Random session nonce keeps an old pipeline result from matching a new OpMode.
        request=1+(int)(System.nanoTime()%1000000000L);
        send(false);
    }
    public synchronized void setIntakePose(boolean atStop) {
        if(intake==atStop) return;
        intake=atStop;
        readyAfter=atStop?System.nanoTime()+config.settleMs*1_000_000L:Long.MAX_VALUE;
        send(false);
    }
    public synchronized void setSelection(Alliance alliance,Mode mode) {
        if(alliance==null || mode==null) throw new IllegalArgumentException("Null selection");
        if(this.alliance==alliance && this.mode==mode) return;
        this.alliance=alliance; this.mode=mode; send(intake&&System.nanoTime()>=readyAfter);
    }
    private void send(boolean ready) {
        request=request>=1000000000?1:request+1;
        sentReady=ready; lastSequence=-1; sequenceSeenAt=0;
        camera.updatePythonInputs(new double[]{request,ready?1:0,alliance==Alliance.BLUE?1:0,mode==Mode.POLLEN_ONLY?1:0,0,0,0,0});
    }
    /** Call every loop. Null means missing/stale/wrong pipeline/unacknowledged configuration. */
    public synchronized Summary getLatest() {
        long now=System.nanoTime();
        boolean ready=intake&&now>=readyAfter;
        if(ready!=sentReady) send(ready);
        LLResult r=camera.getLatestResult();
        if(r==null || r.getPipelineIndex()!=pipeline || r.getStaleness()>config.maxAgeMs) return null;
        double[] out=r.getPythonOutput();
        if(out==null || out.length!=8) return null;
        for(double v:out) if(!Double.isFinite(v)) return null;
        if(Math.abs(out[0])!=request) return null;
        if(out[7]!=lastSequence) { lastSequence=out[7]; sequenceSeenAt=now; }
        if(now-sequenceSeenAt>config.maxAgeMs*1_000_000L) return null;
        double age=r.getStaleness()+r.getCaptureLatency()+r.getTargetingLatency();
        if(!Double.isFinite(age) || age<0 || age>config.maxAgeMs) return null;
        for(int i=4;i<=6;i++) if(out[i]<0 || out[i]>config.maxBlobsPerColor || out[i]!=Math.rint(out[i])) return null;
        boolean valid=out[0]>0 && ready && out[1]>=0 && Math.hypot(out[1],out[2])<=config.maxRange;
        return new Summary(now-(long)(age*1e6),out,valid);
    }
}
