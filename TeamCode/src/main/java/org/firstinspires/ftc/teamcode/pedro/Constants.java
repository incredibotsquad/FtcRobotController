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
        c.xPodOffset.set(-0.5288593412384274);
        c.yPodOffset.set(0.4551008179431825);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4688490500264793);
                Controller secondaryTranslationalForward = Controller.proportional(0.17322719610743817);
                Controller primaryTranslationalLateral = Controller.proportional(0.9051421833444001);
                Controller secondaryTranslationalLateral = Controller.proportional(0.33442585090117977);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011012278781322417));
                c.brake.set(Controller.proportionalFeedforward(0.009360436964124055));

                c.headingFeedback.set(Controller.proportional(8.271184425879827));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.07907162105286057, 0.007470507780590061));

                c.linearBrakeCoefficients.set(Matrix.diag(0.13274694017891775, 0.08303596278871449));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0025701721005049354, 0.001821085488351071));

                c.maxAchievableForwardVelocity.set(80.77174816260268);
                c.maxAchievableStrafeVelocity.set(64.97543624011732);
                c.naturalForwardDeceleration.set(26.344274468686006);
                c.naturalStrafeDeceleration.set(67.31555015749073);
            }
    );
}