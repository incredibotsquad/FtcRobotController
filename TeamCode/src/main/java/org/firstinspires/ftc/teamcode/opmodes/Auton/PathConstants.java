package org.firstinspires.ftc.teamcode.opmodes.Auton;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class PathConstants {
    // in the code hive 1 is the hive which is started tipped up at the start of the match
    // hive 1 contains 3 nectar so only 1 shot is needed to tip it.

    private static final double FIELD_CENTER = 72;

    public final PoseFactory poseFactory;

    private static double maxSpeed = 0.6;
    public final Pose startingPoseRightHive;
    public final Pose startingPoseLeftHive;
    //    public static final Pose startingPose = poseFactory.of(72 - (multiplier * 16), 72 - (multiplier * 64), 180 - (multiplier * 90));
    public final Pose intakeGarden;
    public final Pose shotOnLeftHive;
    public final Pose intakeFarFlowerEarly;
    public final Pose intakeFarFlower;
    public final Pose farFlowerControlPoint;
    public final Pose intakeCloseFlowerEarly;
    public final Pose intakeCloseFlower;
    public final Pose intakeCloseFlowerControl1;
    public final Pose intakeCloseFlowerControl2;
    public final Pose shotOnRightHive;
    public final Pose parkPoseLeftSide;
    public final Pose parkPoseRightSide;

    public PathConstants(AllianceColors alliance) {
        if (alliance == AllianceColors.BLUE) {
            poseFactory = PoseFactory.degrees().mirrorAroundPoint(FIELD_CENTER, FIELD_CENTER);
        } else {
            poseFactory = PoseFactory.degrees();
        }

        startingPoseRightHive = poseFactory.of(56, 10, 180);
        startingPoseLeftHive = poseFactory.of(65,133, 180);
        intakeGarden = poseFactory.of(12, 10, 180);
        shotOnLeftHive = poseFactory.of(39.5176, 118.4389, 90);
        //old pose before intake drop down (50,127)
        intakeFarFlowerEarly = poseFactory.of(50, 124, 90);
        intakeFarFlower = poseFactory.of(50,127, 90);
        farFlowerControlPoint = poseFactory.of(54.85, 109.79, 0);
        //old pose before intake drop down (16, 48)
        intakeCloseFlowerEarly = poseFactory.of(19, 48, 180);
        intakeCloseFlower = poseFactory.of(16, 48, 180);
        intakeCloseFlowerControl1 = poseFactory.of(13.46, 93.43, 0);
        intakeCloseFlowerControl2 = poseFactory.of(32.58, 51.36, 0);
        shotOnRightHive = poseFactory.of(56, 10, 180);
        parkPoseLeftSide = poseFactory.of(8,115,270);
        parkPoseRightSide = poseFactory.of(8,100, 90);
    }

    public Path intakeGarden() {
        return Paths.line(startingPoseRightHive, intakeGarden).constant(intakeGarden).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }

    public Path moveToLeftHive() {
        return Paths.line(intakeGarden, shotOnLeftHive).linear(intakeGarden, shotOnLeftHive).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }

    public Path intakeFarFlowerEarly() {
        return Paths.line(shotOnLeftHive, intakeFarFlowerEarly).constant(intakeFarFlowerEarly).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }
    public Path intakeFarFlower(){
        return Paths.line(intakeFarFlowerEarly, intakeFarFlower).constant(intakeFarFlower).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }

    public Path intakeCloseFlowerEarly() {
        return Paths.curve(intakeFarFlower, intakeCloseFlowerControl1, intakeCloseFlowerControl2, intakeCloseFlowerEarly).linear(intakeCloseFlower, intakeCloseFlower).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }
    public Path intakeCloseFlower(){
        return Paths.line(intakeCloseFlowerEarly, intakeCloseFlower).constant(intakeCloseFlower).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }

    public Path moveToShootRightHive() {
        return Paths.line(intakeCloseFlower, shotOnRightHive).constant(shotOnRightHive).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }
    //move from hive without nectar to flower 1 to intake
    public Path intakeFarFlowerEarlyLeftSide(){
        return Paths.curve(startingPoseLeftHive, farFlowerControlPoint, intakeFarFlowerEarly).tangent().with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }
    public Path parkLeftSide(){
        return Paths.line(intakeFarFlower, parkPoseLeftSide).constant(parkPoseLeftSide).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }
    public Path parkRightSide(){
        return Paths.line(shotOnRightHive, parkPoseRightSide).constant(parkPoseRightSide).with(Constants.foresightConfig.maxPathSpeed.at(maxSpeed));
    }
}
