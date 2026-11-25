package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static Follower create(HardwareMap h) {
        return create(h, d -> d);
    }

    /**
     * Same as {@link #create(HardwareMap)}, but lets the caller wrap the Mecanum drivetrain
     * (DriveSubsystem uses this to cap motor power during the launch hold).
     */
    public static Follower create(HardwareMap h, java.util.function.UnaryOperator<Drivetrain> wrapDrivetrain) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                wrapDrivetrain.apply(new Mecanum(h, drivetrainConfig)),
                new Foresight(foresightConfig)
        );
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeftMotor");
        c.frontRightName.set("frontRightMotor");
        c.backLeftName.set("backLeftMotor");
        c.backRightName.set("backRightMotor");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(3.194); // from OdometryDriftTest, 4 x 180 deg spins averaged (2026-11-19); was 0.5907 (Pedro autotune)
        c.yPodOffset.set(-5.431); // from OdometryDriftTest, 4 x 180 deg spins averaged (2026-11-19); was 2.2376 (Pedro autotune)
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


    // Commented out first: the 2026-11-02 tune (commit 59e45f9), the last one the auto ran smoothly with.
    // Active (second): the 2026-11-21 retune (pod offsets 3.194 / -5.431), with its heading values and strafe P
    // set back to the 11-02 tune's (its own values are kept in comments). Every tune since 11-17 fitted
    // a heading brake coefficient of 0.06-0.085 instead of 0.018. Foresight aims from pose + brake
    // displacement, so heading P x that coefficient acts as a D gain on the turn rate: 8.0 x 0.085 = ~0.68
    // in this retune, against 6.2 x 0.018 = 0.11 in the 11-02 tune, and strafe P was up 23-43%. That
    // over-reacts to Pinpoint velocity noise and loop delay: shaking during paths and at path ends.
//    public static ForesightConfig foresightConfig = new ForesightConfig(
//        c -> {
//            Controller primaryTranslationalForward = Controller.proportional(0.3795337318405364);
//            Controller secondaryTranslationalForward = Controller.proportional(0.1402275725869879);
//            Controller primaryTranslationalLateral = Controller.proportional(0.7307014297843751);
//            Controller secondaryTranslationalLateral = Controller.proportional(0.26997465360353123);
//
//            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//            c.coast.set(Controller.proportionalFeedforward(0.010699578237100073));
//            c.brake.set(Controller.proportionalFeedforward(0.009094641501535062));
//
//            c.headingFeedback.set(Controller.proportional(6.2331816930391755));
//            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.017671053733162783, 0.014059848553904316));
//
//            c.linearBrakeCoefficients.set(Matrix.diag(0.0958358589478464, 0.06803624134392508));
//            c.quadraticBrakeCoefficients.set(Matrix.diag(0.003033300254943125, 0.002181491118553669));
//
//            c.maxAchievableForwardVelocity.set(84.81930331318657);
//            c.maxAchievableStrafeVelocity.set(67.00461140790186);
//            c.naturalForwardDeceleration.set(35.874735526362706);
//            c.naturalStrafeDeceleration.set(73.51962989828517);
//        }
//    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.42104461607899013);
                Controller secondaryTranslationalForward = Controller.proportional(0.1555647351218414);
                // Strafe P set back to the 11-02 tune (2026-11-21): the autotuned values were 33% higher,
                // jitter at path ends / while holding the end pose. Autotuned values were:
                // Controller primaryTranslationalLateral = Controller.proportional(0.9735663322216968);
                // Controller secondaryTranslationalLateral = Controller.proportional(0.3597067455844101);
                Controller primaryTranslationalLateral = Controller.proportional(0.7307014297843751);
                Controller secondaryTranslationalLateral = Controller.proportional(0.26997465360353123);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01161539329281517));
                c.brake.set(Controller.proportionalFeedforward(0.009873084298892894));

                // Heading set back to the 11-02 tune (2026-11-21). Brake x 0.085 made heading P x brake x ~0.68,
                // ~6x the smooth tune's 0.11: shaking during and at the end of paths. With x at 0.018 but P 7.96
                // and y 0.0064 the heading loop was underdamped: shaking while turning. Autotuned values were:
                // c.headingFeedback.set(Controller.proportional(7.957179196950872));
                // c.headingBrakeCoefficients.set(Vector2D.cartesian(0.08499049465530635, 0.006449439442411524));
                // Tried first (fixed shaking on lines, not on turns):
                // c.headingBrakeCoefficients.set(Vector2D.cartesian(0.018, 0.006449439442411524));
                c.headingFeedback.set(Controller.proportional(6.2331816930391755));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.017671053733162783, 0.014059848553904316));

                c.linearBrakeCoefficients.set(Matrix.diag(0.10366361637569067, 0.06550708756711174));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002941774676820776, 0.0022645430363285586));

                c.maxAchievableForwardVelocity.set(81.87583170524951);
                c.maxAchievableStrafeVelocity.set(64.25362625555093);
                c.naturalForwardDeceleration.set(36.16755676305569);
                c.naturalStrafeDeceleration.set(66.9916490108159);
            }
    );

}