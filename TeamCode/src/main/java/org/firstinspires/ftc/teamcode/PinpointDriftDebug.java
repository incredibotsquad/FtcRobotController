package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Locale;

@TeleOp(name = "Pinpoint Drift Debug", group = "Debug")
public class PinpointDriftDebug extends LinearOpMode {
    @Override
    public void runOpMode() {
        GoBildaPinpointDriver odo = hardwareMap.get(GoBildaPinpointDriver.class, "PinpointOdo");

        // Mirror the defaults used by PinpointLocalizer (change there if you change here)
        odo.setOffsets(
                PinpointLocalizer.PARAMS.xPodOffsetIn,
                PinpointLocalizer.PARAMS.yPodOffsetIn,
                DistanceUnit.INCH
        );

        odo.setEncoderDirections(
                PinpointLocalizer.PARAMS.xPodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD,
                PinpointLocalizer.PARAMS.yPodReversed ? GoBildaPinpointDriver.EncoderDirection.REVERSED : GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        telemetry.addLine("Press A to reset+calibrate IMU (hold still).\nPress B to recalibrate IMU (hold still).");
        telemetry.update();

        waitForStart();

        double lastX = 0;
        double lastY = 0;
        double lastH = 0;
        long lastNs = System.nanoTime();

        while (opModeIsActive()) {
            odo.update();

            if (gamepad1.a) {
                odo.resetPosAndIMU();
            } else if (gamepad1.b) {
                odo.recalibrateIMU();
            }

            double x = odo.getPosX(DistanceUnit.INCH);
            double y = odo.getPosY(DistanceUnit.INCH);
            double h = odo.getHeading(UnnormalizedAngleUnit.RADIANS);

            long nowNs = System.nanoTime();
            double dt = (nowNs - lastNs) * 1e-9;
            if (dt <= 0) dt = 1e-3;

            double dx = x - lastX;
            double dy = y - lastY;
            double dh = h - lastH;

            telemetry.addData("Status", odo.getDeviceStatus());
            telemetry.addData("EncX/EncY", "%d / %d", odo.getEncoderX(), odo.getEncoderY());

            telemetry.addData("Pos (in)", String.format(Locale.US, "x=%.3f y=%.3f", x, y));
            telemetry.addData("Heading (deg)", Math.toDegrees(h));

            telemetry.addData("Vel (in/s)", String.format(Locale.US,
                    "x=%.3f y=%.3f", odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH)));
            telemetry.addData("HVel (deg/s)", Math.toDegrees(odo.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)));

            telemetry.addData("Δ/loop", String.format(Locale.US,
                    "dx=%.4f dy=%.4f dhDeg=%.3f", dx, dy, Math.toDegrees(dh)));
            telemetry.addData("Δ/sec", String.format(Locale.US,
                    "dx=%.4f dy=%.4f dhDeg=%.3f", dx / dt, dy / dt, Math.toDegrees(dh) / dt));

            telemetry.addData("Offsets (in)", String.format(Locale.US,
                    "x=%.3f y=%.3f", PinpointLocalizer.PARAMS.xPodOffsetIn, PinpointLocalizer.PARAMS.yPodOffsetIn));
            telemetry.addData("Dirs", String.format(Locale.US,
                    "xReversed=%s yReversed=%s", PinpointLocalizer.PARAMS.xPodReversed, PinpointLocalizer.PARAMS.yPodReversed));

            telemetry.update();

            lastX = x;
            lastY = y;
            lastH = h;
            lastNs = nowNs;
        }
    }
}
