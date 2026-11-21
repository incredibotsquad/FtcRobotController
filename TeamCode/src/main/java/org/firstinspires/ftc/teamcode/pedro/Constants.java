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

//    public static ForesightConfig foresightConfig = new ForesightConfig(
//        c -> {
//            Controller primaryTranslationalForward = Controller.proportional(0.43534930697193375);
//            Controller secondaryTranslationalForward = Controller.proportional(0.16084993617840385);
//            Controller primaryTranslationalLateral = Controller.proportional(0.9004706333237392);
//            Controller secondaryTranslationalLateral = Controller.proportional(0.33269983799466113);
//
//            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//            c.coast.set(Controller.proportionalFeedforward(0.0113404287730719));
//            c.brake.set(Controller.proportionalFeedforward(0.009639364457111114));
//
//            c.headingFeedback.set(Controller.proportional(8.479996767035848));
//            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06174766314672536, 0.010043866541526815));
//
//            c.linearBrakeCoefficients.set(Matrix.diag(0.10910827958284812, 0.08790019618213649));
//            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0029163082754960533, 0.0017677224508885938));
//
//            c.maxAchievableForwardVelocity.set(85.48587731774418);
//            c.maxAchievableStrafeVelocity.set(66.90410822544288);
//            c.naturalForwardDeceleration.set(32.657917907919995);
//            c.naturalStrafeDeceleration.set(75.55232765376651);
//        }
//    );

    // Active: the 2026-11-02 tune (commit 59e45f9), the last one the auto ran smoothly with. Every tune
    // since 11-17 fitted a heading brake coefficient of 0.06-0.08 instead of 0.018. Foresight aims from
    // pose + brake displacement, so heading P x that coefficient acts as a D gain on the turn rate:
    // 8.5 x 0.062 or 7.6 x 0.068 = ~0.5, against 6.2 x 0.018 = 0.11 here, and strafe P was up 23-43%.
    // That over-reacts to Pinpoint velocity noise and loop delay: shaking during paths and at path ends.
    // The 2026-11-20 retune (pod offsets 3.194 / -5.431) is kept commented out below.
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.3795337318405364);
                Controller secondaryTranslationalForward = Controller.proportional(0.1402275725869879);
                Controller primaryTranslationalLateral = Controller.proportional(0.7307014297843751);
                Controller secondaryTranslationalLateral = Controller.proportional(0.26997465360353123);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.010699578237100073));
                c.brake.set(Controller.proportionalFeedforward(0.009094641501535062));

                c.headingFeedback.set(Controller.proportional(6.2331816930391755));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.017671053733162783, 0.014059848553904316));

                c.linearBrakeCoefficients.set(Matrix.diag(0.0958358589478464, 0.06803624134392508));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.003033300254943125, 0.002181491118553669));

                c.maxAchievableForwardVelocity.set(84.81930331318657);
                c.maxAchievableStrafeVelocity.set(67.00461140790186);
                c.naturalForwardDeceleration.set(35.874735526362706);
                c.naturalStrafeDeceleration.set(73.51962989828517);
            }
    );

    // 2026-11-20 retune (shook in auto, see above):
//    public static ForesightConfig foresightConfig = new ForesightConfig(
//            c -> {
//                Controller primaryTranslationalForward = Controller.proportional(0.40017350220082065);
//                Controller secondaryTranslationalForward = Controller.proportional(0.1478534162302917);
//                Controller primaryTranslationalLateral = Controller.proportional(1.0463549504448026);
//                Controller secondaryTranslationalLateral = Controller.proportional(0.38660019507014814);
//
//                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
//                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));
//
//                c.coast.set(Controller.proportionalFeedforward(0.011488917158353528));
//                c.brake.set(Controller.proportionalFeedforward(0.009765579584600498));
//
//                c.headingFeedback.set(Controller.proportional(7.554380926098339));
//                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06791183476300533, 0.008439187045035));
//
//                c.linearBrakeCoefficients.set(Matrix.diag(0.10904107668140206, 0.03724441033792372));
//                c.quadraticBrakeCoefficients.set(Matrix.diag(0.002941076547855179, 0.002733465938302611));
//
//                c.maxAchievableForwardVelocity.set(81.38520822393299);
//                c.maxAchievableStrafeVelocity.set(68.96915178345907);
//                c.naturalForwardDeceleration.set(41.18425622334019);
//                c.naturalStrafeDeceleration.set(81.17923970461624);
//            }
//    );

}