package org.firstinspires.ftc.teamcode.Vision;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * lf - port 0
 * rf - port 1
 * lb - port 2
 * rb - port 3
 * Chases Pollen automatically. As soon as a ball is confirmed by the PollenTracker
 * vision pipeline, the robot turns toward it and drives to it, then goes looking
 * for the next one. No start button.
 *
 * A = stop. Holding A keeps it stopped; releasing it lets the robot hunt again.
 * The sticks drive normally whenever nothing is being chased.
 *
 * No encoders are used. Camera exposure and gain come from PollenTracker, so this
 * OpMode sees exactly the colors your ranges were tuned on.
 *
 * Stale data is never driven on. The blind finish fires only when ALL of these
 * hold, so it can't lunge at nothing:
 *   - the last good reading is less than LAST_GOOD_MAX_S old
 *   - that reading was inside BLIND_START_DIST and roughly straight ahead
 *   - no other ball is currently being tracked
 * And if the ball was lost while further out than BLIND_START_DIST, the chase ends
 * right away instead of waiting (GIVEUP_WHEN_TOO_FAR).
 *
 * Robot frame: x = right (in), y = forward (in), from robot center.
 */
@Configurable
@TeleOp(name = "Move To Ball", group = "Vision")
public class moveToBall extends LinearOpMode {

    // ================= TUNING =================
    // ---- 1. Turn controller (degrees -> turn power) ----
    public static double KP_TURN = 0.015;        // power per degree of error
    public static double MAX_TURN = 0.45;        // cap
    public static double MIN_TURN = 0.10;        // floor, or the robot can't break friction
    public static double TURN_TOL_DEG = 3.0;     // aimed enough to start driving
    public static double TURN_TOL_IN = 1.5;      // or this close sideways, whichever hits first
    public static double STEER_CLAMP = 0.30;     // max steering correction while driving

    // ---- 2. Drive controller (inches -> forward power) ----
    public static double KP_DRIVE = 0.020;       // power per inch of error
    public static double MAX_DRIVE = 0.55;
    public static double MIN_DRIVE = 0.12;       // floor
    public static double FAR_DRIVE_CAP = 0.30;   // speed cap past FAR_DIST, where readings are noisy
    public static double FAR_DIST = 40.0;
    public static double STOP_DIST = 6.0;        // ROBOT CENTER to aim point when done (in)
    public static double DRIVE_TOL = 0.5;        // how close counts as arrived (in)

    // ---- 3. Targeting ----
    // Group scoring (CLUSTER_RADIUS, DIST_PENALTY, AIM_AT_CENTROID) lives in blobTrack.
    public static double MAX_CHASE_DIST = 0;     // ignore groups further than this (in); 0 = no limit
    public static int AVG_SAMPLES = 5;           // rolling average length
    public static double STALE_STOP_S = 0.15;    // no fresh reading this long -> cut power
    public static double RELOCK_S = 0.30;        // locked group gone this long -> re-pick the best one
    public static double LOST_GIVEUP_S = 1.50;   // nothing at all this long -> end the chase
    public static double TIMEOUT_S = 12.0;       // safety timeout per chase
    public static double REARM_S = 0.50;         // pause after a chase ends before hunting again

    // ---- 4. Blind final approach (timed; runs when the balls leave the view) ----
    public static boolean BLIND_FINISH = true;
    public static double BLIND_START_DIST = 24.0;  // camera loses the ball at ~21", so go blind inside this
    public static double LAST_GOOD_MAX_S = 1.0;    // and only if that reading is this fresh
    public static double BLIND_POWER = 0.25;
    public static double BLIND_SPEED_IPS = 13.0;   // in/s at BLIND_POWER  <- set by the speed test
    public static double BLIND_MAX_S = 1.50;       // hard cap; must cover ~18" at BLIND_SPEED_IPS

    // Group lost while further out than BLIND_START_DIST: end the chase immediately
    // instead of waiting LOST_GIVEUP_S. Set false to allow a longer recovery window.
    public static boolean GIVEUP_WHEN_TOO_FAR = true;
    public static double FAR_LOST_WAIT_S = 0.35;   // small grace period for a one-frame dropout

    // ---- 5. Speed test (hold both bumpers) ----
    public static double SPEED_TEST_S = 2.0;       // seconds to drive at BLIND_POWER
    public static double SPEED_TEST_STEP = 1.0;    // inches per dpad press

    // Camera
    private static final int IMG_W = 320;
    private static final int IMG_H = 240;
    // ==========================================

    private enum State { HUNTING, CHASE, BLIND, COOLDOWN, SPEED_TEST, SPEED_RESULT }

    private DcMotor frontLeft, backLeft, frontRight, backRight;

    @Override
    public void runOpMode() {
        // ---- Drivetrain ----
        frontLeft = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeft = hardwareMap.dcMotor.get("backLeftMotor");
        frontRight = hardwareMap.dcMotor.get("frontRightMotor");
        backRight = hardwareMap.dcMotor.get("backRightMotor");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // ---- Vision (blobTrack pipeline: clusters, not single balls) ----
        blobTrack.PollenProcessor processor = new blobTrack.PollenProcessor();
        blobTrack.Geometry geo = new blobTrack.Geometry();
        blobTrack.Tracker tracker = new blobTrack.Tracker();

        // The tracker's defaults assume a still robot and a moving ball. Here it's the
        // other way round, so drop velocity prediction and widen the match gate.
        blobTrack.BETA = 0.0;
        blobTrack.GATE = 8.0;

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(processor)
                .setCameraResolution(new Size(IMG_W, IMG_H))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(100);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // Same exposure as blobTrack, or the camera falls back to auto exposure and
        // the color ranges stop matching.
        String exposureStatus;
        if (blobTrack.MANUAL_EXPOSURE) {
            telemetry.addLine("Waiting for camera to set exposure...");
            telemetry.update();
            exposureStatus = setManualExposure(portal)
                    ? String.format("manual %d ms, gain %d", blobTrack.EXPOSURE_MS, blobTrack.GAIN)
                    : "FAILED to set manual exposure";
        } else {
            exposureStatus = "AUTO (blobTrack.MANUAL_EXPOSURE is false)";
        }

        telemetry.addLine("AUTO CHASE: goes for the biggest group of balls. Hold A to stop.");
        telemetry.addData("exposure", exposureStatus);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        State state = State.HUNTING;
        int lockedId = -1;                 // lead ball of the group being chased
        int lockedCount = 0;               // how many balls that group has
        long lastFrameTime = 0;
        Deque<double[]> samples = new ArrayDeque<>();
        ElapsedTime seenTimer = new ElapsedTime();
        ElapsedTime runTimer = new ElapsedTime();
        ElapsedTime blindTimer = new ElapsedTime();
        ElapsedTime speedTimer = new ElapsedTime();
        ElapsedTime lastGoodTimer = new ElapsedTime();   // age of lastGoodDist
        ElapsedTime cooldownTimer = new ElapsedTime();

        double blindDuration = 0;
        double lastGoodDist = 0, lastGoodHeading = 0;
        double measuredInches = 0;
        String note = "";

        List<blobTrack.Group> groups = new ArrayList<>();

        // Frozen readout of how the last chase ended
        double endDist = -1;
        double endBlindIn = 0;
        String endReason = "";

        boolean lastDown = false, lastLeft = false, lastRight = false, lastBumpers = false;

        while (opModeIsActive()) {
            boolean stopHeld = gamepad1.a;                       // the only chase control
            boolean dDown = gamepad1.dpad_down;
            boolean dLeft = gamepad1.dpad_left, dRight = gamepad1.dpad_right;
            boolean bumpers = gamepad1.left_bumper && gamepad1.right_bumper;

            // ---------- A: stop and stay stopped ----------
            if (stopHeld) {
                stopDrive();
                if (state != State.SPEED_TEST) {
                    state = State.COOLDOWN;
                    cooldownTimer.reset();
                    lockedId = -1;
                    lockedCount = 0;
                    samples.clear();
                    lastGoodDist = 0;
                    note = "STOPPED (holding A)";
                }
            }

            // ---------- Speed test: hold both bumpers (only while idle) ----------
            if (bumpers && !lastBumpers && !stopHeld
                    && (state == State.HUNTING || state == State.COOLDOWN || state == State.SPEED_RESULT)) {
                state = State.SPEED_TEST;
                speedTimer.reset();
                measuredInches = 0;
                note = "mark the robot's start position!";
            }

            // After the speed test: enter how far it actually went
            if (state == State.SPEED_RESULT) {
                if (dRight && !lastRight) measuredInches += SPEED_TEST_STEP;
                if (dLeft && !lastLeft) measuredInches = Math.max(0, measuredInches - SPEED_TEST_STEP);
                if (dDown && !lastDown && measuredInches > 0) {
                    BLIND_SPEED_IPS = measuredInches / SPEED_TEST_S;
                    note = String.format("BLIND_SPEED_IPS = %.1f  (copy into code)", BLIND_SPEED_IPS);
                    state = State.COOLDOWN;
                    cooldownTimer.reset();
                }
            }

            lastDown = dDown; lastLeft = dLeft; lastRight = dRight; lastBumpers = bumpers;

            // ---------- Vision ----------
            geo.syncFromStatics();
            blobTrack.Frame frame = processor.latest;
            if (frame.timeNanos != lastFrameTime) {
                lastFrameTime = frame.timeNanos;

                List<blobTrack.Measurement> meas = new ArrayList<>();
                for (blobTrack.Ball ball : frame.balls) {
                    blobTrack.Projection p = geo.project(ball.x, ball.y, ball.r);
                    if (p == null) continue;
                    if (p.sizeRatio < blobTrack.SIZE_MIN || p.sizeRatio > blobTrack.SIZE_MAX) continue;
                    meas.add(new blobTrack.Measurement(p.x, p.y, ball.x, ball.y, ball.r));
                }
                tracker.update(meas, frame.timeNanos);
                groups = tracker.getGroups();

                if (state == State.CHASE || state == State.BLIND) {
                    // Follow the group that still contains our lead ball, so the aim
                    // point keeps updating as the robot closes in.
                    blobTrack.Group g = tracker.groupContaining(lockedId);

                    // Lead ball gone a while: take the best group now and start fresh.
                    // Runs in BLIND too, so a blind lunge is interrupted the moment
                    // any ball becomes visible again.
                    if (g == null && lockedId >= 0 && seenTimer.seconds() > RELOCK_S) {
                        g = tracker.getTargetGroup(groups);
                        if (g != null) {
                            lockedId = g.leadId;
                            samples.clear();        // never blend pre-dropout readings
                            note = String.format("re-locked on %d ball(s)", g.balls.size());
                        }
                    }

                    if (g != null) {
                        lockedCount = g.balls.size();
                        samples.addLast(new double[] { g.x, g.y });
                        while (samples.size() > Math.max(1, AVG_SAMPLES)) samples.removeFirst();
                        seenTimer.reset();
                    }
                }
            }

            // ---------- Averaged target ----------
            double tx = 0, ty = 0;
            if (!samples.isEmpty()) {
                for (double[] s : samples) { tx += s[0]; ty += s[1]; }
                tx /= samples.size();
                ty /= samples.size();
            }
            double headingErr = Math.toDegrees(Math.atan2(tx, ty));   // + = target is right
            double distance = Math.hypot(tx, ty);
            double distErr = distance - STOP_DIST;
            boolean haveTarget = !samples.isEmpty();
            boolean fresh = seenTimer.seconds() <= STALE_STOP_S;

            // Fresh readings always overwrite the old ones, so lastGood can never
            // lag behind a group that is still visible.
            if (haveTarget && fresh) {
                lastGoodDist = distance;
                lastGoodHeading = headingErr;
                lastGoodTimer.reset();
            }

            // ---------- State machine ----------
            double drive = 0, turn = 0;

            switch (state) {
                case HUNTING: {
                    // Nothing to chase yet: sticks drive normally
                    manualDrive();

                    blobTrack.Group g = tracker.getTargetGroup(groups);
                    boolean inRange = g != null
                            && (MAX_CHASE_DIST <= 0 || g.distance() <= MAX_CHASE_DIST);
                    if (inRange && !stopHeld) {
                        lockedId = g.leadId;
                        lockedCount = g.balls.size();
                        samples.clear();
                        samples.addLast(new double[] { g.x, g.y });
                        seenTimer.reset();
                        lastGoodDist = 0;
                        endDist = -1;
                        endBlindIn = 0;
                        endReason = "";
                        runTimer.reset();
                        state = State.CHASE;
                        note = String.format("%d ball(s) at %.1f in, chasing",
                                lockedCount, g.distance());
                    }
                    break;
                }

                case COOLDOWN:
                    stopDrive();
                    manualDrive();                     // sticks still work
                    if (!stopHeld && cooldownTimer.seconds() > REARM_S) {
                        state = State.HUNTING;
                        note = "";
                    }
                    break;

                case SPEED_TEST:
                    if (speedTimer.seconds() >= SPEED_TEST_S || stopHeld) {
                        stopDrive();
                        state = State.SPEED_RESULT;
                        note = "measure the distance, dpad L/R, then DOWN";
                    } else {
                        drive = BLIND_POWER;
                        setDrive(drive, 0);
                    }
                    break;

                case SPEED_RESULT:
                    stopDrive();
                    manualDrive();
                    break;

                case CHASE:
                    if (runTimer.seconds() > TIMEOUT_S) {
                        stopDrive();
                        endDist = lastGoodDist;
                        endReason = "Timed out";
                        note = endReason;
                        state = State.COOLDOWN;
                        cooldownTimer.reset();
                        break;
                    }

                    if (!haveTarget || !fresh) {
                        stopDrive();                  // never drive on stale data

                        // Go blind ONLY if we just lost a group that was close, straight
                        // ahead, and nothing else is visible right now.
                        boolean canFinishBlind = BLIND_FINISH
                                && lastGoodDist > 0
                                && lastGoodTimer.seconds() < LAST_GOOD_MAX_S
                                && lastGoodDist < BLIND_START_DIST
                                && Math.abs(lastGoodHeading) < 10
                                && tracker.getConfirmed().isEmpty();

                        // Too far out for a blind finish: nothing to do but end the
                        // chase, so don't sit there waiting.
                        boolean tooFarToFinish = GIVEUP_WHEN_TOO_FAR
                                && lastGoodDist >= BLIND_START_DIST
                                && seenTimer.seconds() > FAR_LOST_WAIT_S;

                        if (canFinishBlind) {
                            endBlindIn = Math.max(0, lastGoodDist - STOP_DIST);
                            blindDuration = Math.min(BLIND_MAX_S, endBlindIn / BLIND_SPEED_IPS);
                            blindTimer.reset();
                            state = State.BLIND;
                            note = String.format("blind %.1f in (from %.1f) for %.2fs",
                                    endBlindIn, lastGoodDist, blindDuration);
                        } else if (tooFarToFinish) {
                            endDist = lastGoodDist;
                            endReason = String.format("Lost at %.1f in, too far for blind", lastGoodDist);
                            note = endReason;
                            state = State.COOLDOWN;
                            cooldownTimer.reset();
                        } else if (seenTimer.seconds() > LOST_GIVEUP_S) {
                            endDist = lastGoodDist;
                            endReason = "Lost the group";
                            note = endReason;
                            state = State.COOLDOWN;
                            cooldownTimer.reset();
                        }
                        break;
                    }

                    // Arrived?
                    if (distErr <= DRIVE_TOL) {
                        stopDrive();
                        endDist = distance;
                        endReason = String.format("Arrived (%d ball group)", lockedCount);
                        note = endReason;
                        state = State.COOLDOWN;       // then hunt for the next group
                        cooldownTimer.reset();
                        lockedId = -1;
                        lockedCount = 0;
                        samples.clear();
                        lastGoodDist = 0;
                        break;
                    }

                    // Turn in place until roughly lined up, then steer while driving
                    boolean lined = Math.abs(headingErr) <= TURN_TOL_DEG
                            || Math.abs(tx) <= TURN_TOL_IN;
                    if (!lined) {
                        turn = clampWithFloor(KP_TURN * headingErr, MIN_TURN, MAX_TURN);
                        setDrive(0, turn);
                    } else {
                        drive = clampWithFloor(KP_DRIVE * distErr, MIN_DRIVE, MAX_DRIVE);
                        if (distance > FAR_DIST) drive = clamp(drive, -FAR_DRIVE_CAP, FAR_DRIVE_CAP);
                        turn = clamp(KP_TURN * headingErr, -STEER_CLAMP, STEER_CLAMP);
                        setDrive(drive, turn);
                    }
                    break;

                case BLIND:
                    if (haveTarget && fresh) {
                        stopDrive();
                        state = State.CHASE;          // balls are back, resume normally
                        note = "reacquired";
                    } else if (blindTimer.seconds() >= blindDuration) {
                        stopDrive();
                        endDist = lastGoodDist;
                        endReason = String.format("Blind done (%.1f in asked)", endBlindIn);
                        note = endReason;
                        state = State.COOLDOWN;       // then hunt for the next group
                        cooldownTimer.reset();
                        lockedId = -1;
                        lockedCount = 0;
                        samples.clear();
                        lastGoodDist = 0;
                    } else {
                        drive = BLIND_POWER;
                        setDrive(drive, 0);
                        note = String.format("blind %.2f / %.2f s",
                                blindTimer.seconds(), blindDuration);
                    }
                    break;
            }

            // ---------- Telemetry ----------
            telemetry.addData("state", state);
            if (!note.isEmpty()) telemetry.addData("note", note);

            if (endDist >= 0) {
                telemetry.addLine("-- last chase --");
                telemetry.addData("STOPPED AT", "%.1f in (vision)", endDist);
                telemetry.addData("reason", endReason);
                if (endBlindIn > 0) telemetry.addData("blind asked for", "%.1f in", endBlindIn);
            }

            if (state == State.SPEED_TEST) {
                telemetry.addData("driving", "%.2f / %.1f s at %.2f power",
                        speedTimer.seconds(), SPEED_TEST_S, BLIND_POWER);
            } else if (state == State.SPEED_RESULT) {
                telemetry.addData("measured", "%.1f in   (dpad L/R)", measuredInches);
                telemetry.addData("would give", "%.1f in/s   (dpad DOWN)",
                        measuredInches / SPEED_TEST_S);
                telemetry.addData("BLIND_SPEED_IPS now", "%.1f", BLIND_SPEED_IPS);
            } else {
                telemetry.addData("locked", lockedId < 0 ? "none"
                        : String.format("group of %d (lead id %d)", lockedCount, lockedId));
                telemetry.addData("vision", fresh ? "fresh"
                        : String.format("STALE %.2fs", seenTimer.seconds()));
                if (haveTarget) {
                    telemetry.addData("aim x,y", "%.1f, %.1f", tx, ty);
                    telemetry.addData("distance", "%.1f in", distance);
                    telemetry.addData("heading err", "%.1f deg", headingErr);
                }
                telemetry.addData("last good", "%.1f in @ %.1f deg  (%.1fs old)",
                        lastGoodDist, lastGoodHeading, lastGoodTimer.seconds());
                telemetry.addData("power", "drive %.2f  turn %.2f", drive, turn);

                telemetry.addLine("-- groups --");
                telemetry.addLine("n    x      y     dist  score");
                for (blobTrack.Group g : groups) {
                    telemetry.addLine(String.format("%-3d %6.1f %6.1f %6.1f %6.2f%s",
                            g.balls.size(), g.x, g.y, g.distance(), g.score,
                            g.contains(lockedId) ? "  <-- locked" : ""));
                }

                telemetry.addLine("-- camera --");
                telemetry.addData("exposure", exposureStatus);
                telemetry.addData("tiltDeg", "%.2f", geo.tiltDeg);
                telemetry.addData("focalPx", "%.1f", geo.focalPx);
            }

            telemetry.addLine("A = STOP   both bumpers = speed test");
            telemetry.update();
        }

        stopDrive();
        portal.close();
    }

    // =====================================================================

    /**
     * Same exposure setup as blobTrack, reading its values so the two OpModes
     * always match. Returns false if the camera never started streaming.
     */
    private boolean setManualExposure(VisionPortal portal) {
        while (!isStopRequested()
                && portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            sleep(20);
        }
        if (isStopRequested()) return false;

        ExposureControl exp = portal.getCameraControl(ExposureControl.class);
        if (exp.getMode() != ExposureControl.Mode.Manual) {
            exp.setMode(ExposureControl.Mode.Manual);
            sleep(50);
        }
        exp.setExposure(blobTrack.EXPOSURE_MS, TimeUnit.MILLISECONDS);
        sleep(20);
        portal.getCameraControl(GainControl.class).setGain(blobTrack.GAIN);
        sleep(20);
        return true;
    }

    private void manualDrive() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;
        setPowers(y, x, rx);
    }

    private void setDrive(double forward, double turn) {
        setPowers(forward, 0, turn);
    }

    private void setPowers(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeft.setPower((y + x + rx) / denominator);
        backLeft.setPower((y - x + rx) / denominator);
        frontRight.setPower((y - x - rx) / denominator);
        backRight.setPower((y + x - rx) / denominator);
    }

    private void stopDrive() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static double clampWithFloor(double v, double min, double max) {
        double s = Math.signum(v);
        double m = Math.min(Math.abs(v), max);
        if (m < min) m = min;
        return s * m;
    }
}