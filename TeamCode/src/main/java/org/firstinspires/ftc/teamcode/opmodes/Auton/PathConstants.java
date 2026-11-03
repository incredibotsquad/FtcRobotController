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
    public static final Pose startingPoseRightHive = poseFactory.of(56, 10, 180);
    public static final Pose startingPoseLeftHive = poseFactory.of(65,133, 180);
    //    public static final Pose startingPose = poseFactory.of(72 - (multiplier * 16), 72 - (multiplier * 64), 180 - (multiplier * 90));
    public static final Pose intakeGarden = poseFactory.of(12, 10, 180);
    public static final Pose shotOnLeftHive = poseFactory.of(39.5176, 118.4389, 90);
    public static final Pose intakeFarFlower = poseFactory.of(50, 127, 90);
    public static final Pose farFlowerControlPoint = poseFactory.of(54.85, 109.79, 0);
    public static final Pose intakeCloseFlower = poseFactory.of(16, 48, 180);
    public static final Pose intakeCloseFlowerControl1 = poseFactory.of(13.46, 93.43, 0);
    public static final Pose intakeCloseFlowerControl2 = poseFactory.of(32.58, 51.36, 0);
    public static final Pose shotOnRightHive = poseFactory.of(56, 10, 180);
    public static final Pose parkPoseLeftSide = poseFactory.of(8,115,270);
    public static final Pose parkPoseRightSide = poseFactory.of(8,100, 90);

    public static Path intakeGarden() {
        return Paths.line(startingPoseRightHive, intakeGarden).constant(intakeGarden).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }

    public static Path moveToLeftHive() {
        return Paths.line(intakeGarden, shotOnLeftHive).linear(intakeGarden, shotOnLeftHive).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }

    public static Path intakeFarFlower() {
        return Paths.line(shotOnLeftHive, intakeFarFlower).constant(intakeFarFlower).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }

    public static Path intakeCloseFlower() {
        return Paths.curve(intakeFarFlower, intakeCloseFlowerControl1, intakeCloseFlowerControl2, intakeCloseFlower).linear(intakeCloseFlower, intakeCloseFlower).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }

    public static Path moveToShootRightHive() {
        return Paths.line(intakeCloseFlower, shotOnRightHive).constant(shotOnRightHive).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }
    //move from hive without nectar to flower 1 to intake
    public static Path intakeFarFlowerLeftSide(){
        return Paths.curve(startingPoseLeftHive, farFlowerControlPoint, intakeFarFlower).tangent().with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }
    public static Path parkLeftSide(){
        return Paths.line(intakeFarFlower, parkPoseLeftSide).constant(parkPoseLeftSide).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }
    public static Path parkRightSide(){
        return Paths.line(shotOnRightHive, parkPoseRightSide).constant(parkPoseRightSide).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }
}