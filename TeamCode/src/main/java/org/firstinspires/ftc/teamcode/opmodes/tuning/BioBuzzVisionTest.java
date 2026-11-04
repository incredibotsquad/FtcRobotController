package org.firstinspires.ftc.teamcode.opmodes.tuning;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.biobuzz.*;
import org.firstinspires.ftc.vision.VisionPortal;
import java.io.ByteArrayOutputStream;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import static org.firstinspires.ftc.teamcode.vision.biobuzz.BioBuzzResult.*;

/** Stationary, vision-only diagnostic. Does not command drive motors or swivel servos. */
@TeleOp(name="BIOBuzz: Vision Test",group="Tuning")
public class BioBuzzVisionTest extends LinearOpMode {
    @Override public void runOpMode() {
        VisionPortal portal=null;
        BioBuzzProcessor processor=null;
        try {
            String json;
            try(InputStream in=hardwareMap.appContext.getAssets().open("biobuzz.json")) {
                ByteArrayOutputStream out=new ByteArrayOutputStream(); byte[] buffer=new byte[4096]; int n;
                while((n=in.read(buffer))!=-1) out.write(buffer,0,n);
                json=new String(out.toByteArray(),StandardCharsets.UTF_8);
            }
            BioBuzzConfig config=new BioBuzzConfig(json);
            processor=new BioBuzzProcessor(config);
            portal=new VisionPortal.Builder().setCamera(hardwareMap.get(WebcamName.class,"Webcam 1"))
                    .setCameraResolution(new Size(config.width,config.height)).addProcessor(processor).build();
            boolean intake=false, previousA=false, previousB=false, previousX=false;
            Alliance alliance=Alliance.RED; Mode mode=Mode.MIXED;
            while(!isStopRequested() && (opModeInInit() || opModeIsActive())) {
                if(gamepad1.a&&!previousA) { intake=!intake; processor.setIntakePose(intake); }
                if(gamepad1.b&&!previousB) alliance=alliance==Alliance.RED?Alliance.BLUE:Alliance.RED;
                if(gamepad1.x&&!previousX) mode=mode==Mode.MIXED?Mode.POLLEN_ONLY:Mode.MIXED;
                previousA=gamepad1.a; previousB=gamepad1.b; previousX=gamepad1.x;
                processor.setSelection(alliance,mode);
                BioBuzzResult result=processor.getLatest();
                telemetry.addLine("A: confirm intake pose / disable; B: alliance; X: pickup mode");
                telemetry.addData("Calibration",config.isCalibrated()?"loaded":"missing: use Mac calibration");
                telemetry.addData("Selection",alliance+" "+mode);
                telemetry.addData("Camera",portal.getCameraState());
                telemetry.addData("Intake requested / settled",intake+" / "+result.intakeReady);
                telemetry.addData("Pollen / red / blue blobs","%d / %d / %d",result.count(Kind.POLLEN),result.count(Kind.RED_NECTAR),result.count(Kind.BLUE_NECTAR));
                telemetry.addData("FPS / processing ms","%.1f / %.1f",portal.getFps(),result.processingMs);
                telemetry.addData("Blob limit reached",result.truncated);
                Target best=result.best();
                if(best==null) telemetry.addLine("No usable pickup target");
                else telemetry.addData("Best forward / left / score","%.1f in / %.1f in / %.2f",best.position.forward,best.position.left,best.score);
                telemetry.update(); sleep(30);
            }
        } catch(Exception e) {
            telemetry.addData("Vision setup error",e.toString()); telemetry.update();
            while(!isStopRequested()) sleep(50);
        } finally {
            if(portal!=null) portal.close();
            if(processor!=null) processor.close();
        }
    }
}
