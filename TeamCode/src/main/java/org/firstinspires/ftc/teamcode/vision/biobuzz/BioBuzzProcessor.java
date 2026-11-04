package org.firstinspires.ftc.teamcode.vision.biobuzz;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import static org.firstinspires.ftc.teamcode.vision.biobuzz.BioBuzzResult.*;

/** Attach to a VisionPortal. Call setIntakePose(false) BEFORE moving the swivel. */
public final class BioBuzzProcessor implements VisionProcessor, AutoCloseable {
    private final BioBuzzConfig config;
    private final Mat hsv=new Mat(), mask=new Mat(), part=new Mat(), hierarchy=new Mat(), kernel;
    private BioBuzzResult latest=BioBuzzResult.empty();
    private Alliance alliance=Alliance.RED;
    private Mode mode=Mode.MIXED;
    private boolean intakePose=false, closed=false;
    private long readyAfterNanos=Long.MAX_VALUE, settingsAfterNanos=0;

    public BioBuzzProcessor(BioBuzzConfig config) {
        this.config=config;
        kernel=Mat.ones(config.kernel,config.kernel,CvType.CV_8U);
    }
    /** true means the mechanism has reached its calibrated intake stop; adds settle_ms. */
    public synchronized void setIntakePose(boolean atCalibratedStop) {
        if (intakePose==atCalibratedStop) return;
        intakePose=atCalibratedStop;
        readyAfterNanos=atCalibratedStop?System.nanoTime()+config.settleMs*1_000_000L:Long.MAX_VALUE;
        latest=BioBuzzResult.empty();
    }
    public synchronized void setSelection(Alliance alliance, Mode mode) {
        if (alliance==null || mode==null) throw new IllegalArgumentException("Selection cannot be null");
        if (this.alliance==alliance && this.mode==mode) return;
        this.alliance=alliance; this.mode=mode; settingsAfterNanos=System.nanoTime();
        latest=BioBuzzResult.empty();
    }
    /** Returns an empty snapshot after a camera stall, shutdown, or selection/pose transition. */
    public synchronized BioBuzzResult getLatest() {
        return latest.isFresh(System.nanoTime(),config.maxAgeMs)?latest:BioBuzzResult.empty();
    }
    @Override public void init(int width,int height,CameraCalibration calibration) { }
    @Override public synchronized Object processFrame(Mat frame,long captureTimeNanos) {
        if (closed || captureTimeNanos<settingsAfterNanos) return BioBuzzResult.empty();
        long start=System.nanoTime();
        boolean ready=intakePose && captureTimeNanos>=readyAfterNanos;
        int width=frame.cols(), height=frame.rows();
        // VisionPortal delivers RGB; the Mac/Limelight implementations receive BGR.
        Imgproc.cvtColor(frame,hsv,Imgproc.COLOR_RGB2HSV);
        int left=(int)(config.roi[0]*width), top=(int)(config.roi[1]*height);
        int right=(int)(config.roi[2]*width), bottom=(int)(config.roi[3]*height);
        List<Blob> blobs=new ArrayList<>();
        boolean truncated=false;
        for (Kind kind:Kind.values()) {
            mask.create(height,width,CvType.CV_8U); mask.setTo(Scalar.all(0));
            for (int[] r:config.hsv[kind.ordinal()]) {
                Core.inRange(hsv,new Scalar(r[0],r[1],r[2]),new Scalar(r[3],r[4],r[5]),part);
                Core.bitwise_or(mask,part,mask);
            }
            Imgproc.morphologyEx(mask,mask,Imgproc.MORPH_OPEN,kernel);
            Imgproc.morphologyEx(mask,mask,Imgproc.MORPH_CLOSE,kernel);
            zeroRect(mask,0,0,width,top); zeroRect(mask,0,bottom,width,height-bottom);
            zeroRect(mask,0,0,left,height); zeroRect(mask,right,0,width-right,height);
            List<MatOfPoint> contours=new ArrayList<>();
            List<Blob> accepted=new ArrayList<>();
            try {
                Imgproc.findContours(mask,contours,hierarchy,Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_SIMPLE);
                for (MatOfPoint contour:contours) {
                    double area=Imgproc.contourArea(contour), fraction=area/(width*(double)height);
                    if (fraction<config.minArea || fraction>config.maxArea) continue;
                    Rect r=Imgproc.boundingRect(contour);
                    if (area/(r.width*(double)r.height)<config.minFill || Math.max(r.width/(double)r.height,r.height/(double)r.width)>config.maxAspect) continue;
                    double maxY=-1, sumX=0; int count=0;
                    for (Point p:contour.toArray()) {
                        if (p.y>maxY) { maxY=p.y; sumX=p.x; count=1; }
                        else if (p.y==maxY) { sumX+=p.x; count++; }
                    }
                    double u=sumX/count/width, v=maxY/height;
                    boolean clipped=r.x<=left || r.y<=top || r.x+r.width>=right || r.y+r.height>=bottom;
                    Position position=ready&&!clipped?BioBuzzGeometry.project(u,v,config,width/(double)height):null;
                    accepted.add(new Blob(kind,r.x,r.y,r.width,r.height,area,u,v,clipped,position));
                }
            } finally { for(MatOfPoint contour:contours) contour.release(); }
            accepted.sort(Comparator.comparingDouble((Blob b)->-b.area).thenComparingInt(b->b.x).thenComparingInt(b->b.y));
            truncated|=accepted.size()>config.maxBlobsPerColor;
            blobs.addAll(accepted.subList(0,Math.min(accepted.size(),config.maxBlobsPerColor)));
        }
        latest=new BioBuzzResult(captureTimeNanos,blobs,BioBuzzGeometry.rank(blobs,config,alliance,mode),
                ready,truncated,(System.nanoTime()-start)/1e6);
        return latest;
    }
    private static void zeroRect(Mat m,int x,int y,int w,int h) {
        if(w<=0 || h<=0) return;
        Mat sub=m.submat(new Rect(x,y,w,h));
        try { sub.setTo(Scalar.all(0)); } finally { sub.release(); }
    }
    @Override public void onDrawFrame(Canvas canvas,int onscreenWidth,int onscreenHeight,
                                      float scale,float density,Object context) {
        if (!(context instanceof BioBuzzResult)) return;
        BioBuzzResult result=(BioBuzzResult)context;
        Paint paint=new Paint(); paint.setStyle(Paint.Style.STROKE); paint.setStrokeWidth(2*density);
        for (Blob b:result.blobs) {
            paint.setColor(b.kind==Kind.POLLEN?Color.YELLOW:b.kind==Kind.RED_NECTAR?Color.RED:Color.BLUE);
            canvas.drawRect(b.x*scale,b.y*scale,(b.x+b.width)*scale,(b.y+b.height)*scale,paint);
        }
        Target best=result.best();
        if (best!=null) {
            Blob b=result.blobs.get(best.blobIndex); paint.setColor(Color.GREEN);
            canvas.drawCircle((b.x+b.width/2f)*scale,(b.y+b.height)*scale,12*density,paint);
        }
    }
    /** Close the VisionPortal first, then release this processor's native buffers. */
    @Override public synchronized void close() {
        closed=true; latest=BioBuzzResult.empty();
        hsv.release(); mask.release(); part.release(); hierarchy.release(); kernel.release();
    }
}
