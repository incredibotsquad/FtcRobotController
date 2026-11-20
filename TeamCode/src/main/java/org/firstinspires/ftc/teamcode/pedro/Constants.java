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
        c.xPodOffset.set(0.5906894263320082);
        c.yPodOffset.set(2.2375808175154557);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
        c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.43534930697193375);
            Controller secondaryTranslationalForward = Controller.proportional(0.16084993617840385);
            Controller primaryTranslationalLateral = Controller.proportional(0.9004706333237392);
            Controller secondaryTranslationalLateral = Controller.proportional(0.33269983799466113);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.0113404287730719));
            c.brake.set(Controller.proportionalFeedforward(0.009639364457111114));

            c.headingFeedback.set(Controller.proportional(8.479996767035848));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06174766314672536, 0.010043866541526815));

            c.linearBrakeCoefficients.set(Matrix.diag(0.10910827958284812, 0.08790019618213649));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0029163082754960533, 0.0017677224508885938));

            c.maxAchievableForwardVelocity.set(85.48587731774418);
            c.maxAchievableStrafeVelocity.set(66.90410822544288);
            c.naturalForwardDeceleration.set(32.657917907919995);
            c.naturalStrafeDeceleration.set(75.55232765376651);
        }
    );
}