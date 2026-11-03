package org.firstinspires.ftc.teamcode.opmodes.Auton;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class PathConstants {
    // in the code hive 1 is the hive which is started tipped up at the start of the match
    // hive 1 contains 3 nectar so only 1 shot is needed to tip it.

    public static final PoseFactory poseFactory = PoseFactory.degrees();

    private static double maxSpeed = 0.6;
    public static final Pose startingPose = poseFactory.of(56, 10, 180);
    //    public static final Pose startingPose = poseFactory.of(72 - (multiplier * 16), 72 - (multiplier * 64), 180 - (multiplier * 90));
    public static final Pose intakeGarden = poseFactory.of(12, 10, 180);
    public static final Pose shotOnHive2 = poseFactory.of(39.5176, 118.4389, 90);
    public static final Pose intakeFlower1 = poseFactory.of(50, 130, 90);
    public static final Pose intakeFlower2 = poseFactory.of(16, 48, 180);
    public static final Pose intakeFlower2Control1 = poseFactory.of(13.46, 93.43, 0);
    public static final Pose intakeFlower2Control2 = poseFactory.of(32.58, 51.36, 0);
    public static final Pose shotOnHive1 = poseFactory.of(56, 22, 180);

    public static Path intakeGarden() {
        return Paths.line(startingPose, intakeGarden).constant(intakeGarden).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }

    public static Path moveToHive2() {
        return Paths.line(intakeGarden, shotOnHive2).linear(intakeGarden, shotOnHive2).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }

    public static Path intakeFlower1() {
        return Paths.line(shotOnHive2, intakeFlower1).constant(intakeFlower1).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }

    public static Path intakeFlower2() {
        return Paths.curve(intakeFlower1, intakeFlower2Control1, intakeFlower2Control2, intakeFlower2).linear(intakeFlower1, intakeFlower2).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }

    public static Path moveToHive1() {
        return Paths.line(intakeFlower2, shotOnHive1).constant(shotOnHive1).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }
}