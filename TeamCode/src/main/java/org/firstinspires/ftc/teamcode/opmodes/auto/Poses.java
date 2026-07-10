package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.Pose;

public class Poses {
    public static final Pose NEAR_START_POSE = new Pose(22, 122, Math.toRadians(324)); // Start Pose of our robot. This is against the goal facing AWAY
    public static final Pose NEAR_SCORE_POSE = new Pose(60, 84, Math.toRadians(135)); // Scoring Pose of our robot.
    public static final Pose NEAR_END_POSE = new Pose (60, 105); // Final Pose of our robot, off the starting line

    public static final Pose FAR_START_POSE = new Pose(84, 12, Math.toRadians(324)); // Start Pose of our robot. This is against the goal facing AWAY
    public static final Pose FAR_SCORE_POSE = new Pose(84, 24, Math.toRadians(135)); // Scoring Pose of our robot.
    public static final Pose FAR_END_POSE = new Pose (132, 48); // Final Pose of our robot, off the starting line

    public static final Pose PICKUP_1_POSE = new Pose(17, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static final Pose PICKUP_2_POSE = new Pose(12, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    public static final Pose PICKUP_3_POSE = new Pose(12, 36, Math.toRadians(180)); // Lowest (Third Set) of Artifacts from the Spike Mark.
    public static final Pose PICKUP_GATE_POSE = new Pose(125, 68, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static final Pose PICKUP_HUMAN_POSE = new Pose(17, 84, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.


    public static Pose mirrorPose(Pose pose) {
        return new Pose(144 - pose.getX(), pose.getY(), Math.PI - pose.getHeading());
    }
}
