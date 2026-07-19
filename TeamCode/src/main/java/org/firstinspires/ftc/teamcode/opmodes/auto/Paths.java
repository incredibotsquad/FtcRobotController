package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

public class Paths {

    private Follower follower;
    private Poses poses;

    public PathChain NEAR_PRELOADS, NEAR_GATE_1_PATH, NEAR_SCORE_GATE, NEAR_GATE, NEAR_GATE_2, NEAR_SPIKE_1, NEAR_SCORE_SPIKE_1, NEAR_SPIKE_2, NEAR_SCORE_SPIKE_2, NEAR_HUMAN_PLAYER, NEAR_SCORE_HUMAN, NEAR_SPIKE_3, NEAR_SCORE_SPIKE_3, NEAR_TUNNEL, NEAR_SCORE_TUNNEL, FAR_SCORE_HUMAN, FAR_SCORE_TUNNEL, NEAR_LEAVE;
    public PathChain FAR_HUMAN_PLAYER, FAR_TUNNEL, FAR_SPIKE_3, FAR_LEAVE, FAR_SCORE_SPIKE_3, FAR_SPIKE_2, FAR_SCORE_SPIKE_2, FAR_SPIKE_1, FAR_SCORE_SPIKE_1, FAR_GATE, FAR_GATE_2, FAR_SCORE_GATE;

    public Paths(Follower follower, Poses poses) {
        this.follower = follower;
        this.poses = poses;
    }

    public void buildNearPaths() {
        NEAR_PRELOADS = follower.pathBuilder()
                .addPath(new BezierLine(poses.NEAR_INIT, poses.NEAR_LAUNCH))
                .setLinearHeadingInterpolation(poses.NEAR_INIT.getHeading(), poses.NEAR_LAUNCH.getHeading())
                .build();

        NEAR_SPIKE_1 = follower.pathBuilder()
                .addPath(new BezierLine(poses.NEAR_LAUNCH, poses.NEAR_FIRST_SPIKE))
                .setLinearHeadingInterpolation(poses.NEAR_LAUNCH.getHeading(), poses.NEAR_FIRST_SPIKE.getHeading())
                .build();

        NEAR_SCORE_SPIKE_1 = follower.pathBuilder()
                .addPath(new BezierLine(poses.NEAR_FIRST_SPIKE, poses.NEAR_LAUNCH))
                .setLinearHeadingInterpolation(poses.NEAR_FIRST_SPIKE.getHeading(), poses.NEAR_LAUNCH.getHeading())
                .build();

        NEAR_SPIKE_2 = follower.pathBuilder()
                .addPath(new BezierCurve(poses.NEAR_LAUNCH, poses.NEAR_SECOND_SPIKE_CONTROL, poses.NEAR_SECOND_SPIKE))
                .setLinearHeadingInterpolation(poses.NEAR_LAUNCH.getHeading(), poses.NEAR_SECOND_SPIKE.getHeading())
                .build();

        NEAR_SCORE_SPIKE_2 = follower.pathBuilder()
                .addPath(new BezierLine(poses.NEAR_SECOND_SPIKE, poses.NEAR_LAUNCH))
                .setLinearHeadingInterpolation(poses.NEAR_SECOND_SPIKE.getHeading(), poses.NEAR_LAUNCH.getHeading())
                .build();

        NEAR_GATE = follower.pathBuilder()
                .addPath(new BezierLine(poses.NEAR_LAUNCH, poses.NEAR_GATE_POSE))
                .setLinearHeadingInterpolation(poses.NEAR_LAUNCH.getHeading(), poses.NEAR_GATE_POSE.getHeading())
                .build();

        NEAR_GATE_2 = follower.pathBuilder()
                .addPath(new BezierCurve(poses.NEAR_GATE_POSE, poses.NEAR_GATE_CONTROL_2nd, poses.NEAR_GATE_POSE_2))
                .setLinearHeadingInterpolation(poses.NEAR_GATE_POSE.getHeading(), poses.NEAR_GATE_POSE_2.getHeading())
                .build();

        NEAR_GATE_1_PATH = follower.pathBuilder()
                .addPath(new BezierCurve(poses.NEAR_LAUNCH, poses.NEAR_1_PATH_GATE_CONTROL_1, poses.NEAR_1_PATH_GATE_CONTROL_2, poses.NEAR_1_PATH_GATE_INTAKE))
                .setLinearHeadingInterpolation(poses.NEAR_LAUNCH.getHeading(), poses.NEAR_1_PATH_GATE_INTAKE.getHeading())
                .build();

        NEAR_SCORE_GATE = follower.pathBuilder()
                .addPath(new BezierLine(poses.NEAR_GATE_POSE_2, poses.NEAR_LAUNCH))
                .setLinearHeadingInterpolation(poses.NEAR_GATE_POSE_2.getHeading(), poses.NEAR_LAUNCH.getHeading())
                .build();

//        NEAR_HUMAN_PLAYER = follower.pathBuilder()
//                .addPath(new BezierCurve(poses.NEAR_SHOT_POSE, poses.NEAR_HUMAN_CONTROL, poses.PICKUP_HUMAN_POSE))
//                .setLinearHeadingInterpolation(poses.NEAR_SHOT_POSE.getHeading(), poses.PICKUP_HUMAN_POSE.getHeading())
//                .build();
//
//        NEAR_SCORE_HUMAN = follower.pathBuilder()
//                .addPath(new BezierLine(poses.PICKUP_HUMAN_POSE, poses.NEAR_SHOT_POSE))
//                .setLinearHeadingInterpolation(poses.PICKUP_HUMAN_POSE.getHeading(), poses.NEAR_SHOT_POSE.getHeading())
//                .build();
//
//        NEAR_SPIKE_3 = follower.pathBuilder()
//                .addPath(new BezierCurve(poses.NEAR_SHOT_POSE, poses.SPIKE_3_CONTROL_POSE, poses.FAR_INTAKE2_POSE))
//                .setLinearHeadingInterpolation(poses.NEAR_SHOT_POSE.getHeading(), poses.FAR_INTAKE2_POSE.getHeading())
//                .build();
//
//        NEAR_SCORE_SPIKE_3 = follower.pathBuilder()
//                .addPath(new BezierLine(poses.FAR_INTAKE2_POSE, poses.NEAR_SHOT_POSE))
//                .setLinearHeadingInterpolation(poses.FAR_INTAKE2_POSE.getHeading(), poses.NEAR_SHOT_POSE.getHeading())
//                .build();
//
//        NEAR_TUNNEL = follower.pathBuilder()
//                .addPath(new BezierLine(poses.NEAR_SHOT_POSE, poses.FAR_INTAKE3_POSE))
//                .setLinearHeadingInterpolation(poses.NEAR_SHOT_POSE.getHeading(), poses.FAR_INTAKE3_POSE.getHeading())
//                .build();
//
//        NEAR_SCORE_TUNNEL = follower.pathBuilder()
//                .addPath(new BezierLine(poses.FAR_INTAKE3_POSE, poses.NEAR_SHOT_POSE))
//                .setLinearHeadingInterpolation(poses.FAR_INTAKE3_POSE.getHeading(), poses.NEAR_SHOT_POSE.getHeading())
//                .build();

        NEAR_LEAVE = follower.pathBuilder()
                .addPath(new BezierLine(poses.NEAR_LAUNCH, poses.NEAR_END_POSE))
                .setLinearHeadingInterpolation(poses.NEAR_LAUNCH.getHeading(), poses.NEAR_END_POSE.getHeading())
                .build();
    }

    public void buildFarPaths() {
        FAR_HUMAN_PLAYER = follower.pathBuilder()
                .addPath(new BezierLine(poses.FAR_SCORE_POSE, poses.HUMAN_PLAYER_POSE))
                .setLinearHeadingInterpolation(poses.FAR_SCORE_POSE.getHeading(), poses.HUMAN_PLAYER_POSE.getHeading())
                .build();

        FAR_SCORE_HUMAN = follower.pathBuilder()
                .addPath(new BezierLine(poses.HUMAN_PLAYER_POSE, poses.FAR_SCORE_POSE))
                .setLinearHeadingInterpolation(poses.HUMAN_PLAYER_POSE.getHeading(), poses.FAR_SCORE_POSE.getHeading())
                .build();

        FAR_SPIKE_3 = follower.pathBuilder()
                .addPath(new BezierCurve(poses.FAR_SCORE_POSE, poses.SPIKE_3_CONTROL_POSE, poses.FAR_INTAKE2_POSE))
                .setLinearHeadingInterpolation(poses.FAR_SCORE_POSE.getHeading(), poses.FAR_INTAKE2_POSE.getHeading())
                .build();

        FAR_SCORE_SPIKE_3 = follower.pathBuilder()
                .addPath(new BezierLine(poses.FAR_INTAKE2_POSE, poses.FAR_SCORE_POSE))
                .setLinearHeadingInterpolation(poses.FAR_SCORE_POSE.getHeading(), poses.FAR_INTAKE2_POSE.getHeading())
                .build();

        FAR_TUNNEL = follower.pathBuilder()
                .addPath(new BezierLine(poses.FAR_SCORE_POSE, poses.FAR_INTAKE3_POSE))
                .setLinearHeadingInterpolation(poses.FAR_SCORE_POSE.getHeading(), poses.FAR_INTAKE3_POSE.getHeading())
                .build();

        FAR_SCORE_TUNNEL = follower.pathBuilder()
                .addPath(new BezierLine(poses.FAR_INTAKE3_POSE, poses.FAR_SCORE_POSE))
                .setLinearHeadingInterpolation(poses.FAR_INTAKE3_POSE.getHeading(), poses.FAR_SCORE_POSE.getHeading())
                .build();

        FAR_LEAVE = follower.pathBuilder()
                .addPath(new BezierLine(poses.FAR_SCORE_POSE, poses.FAR_END_POSE))
                .setLinearHeadingInterpolation(poses.FAR_SCORE_POSE.getHeading(), poses.FAR_SCORE_POSE.getHeading())
                .build();

        FAR_SPIKE_2 = follower.pathBuilder()
                .addPath(new BezierCurve(poses.FAR_SCORE_POSE, poses.FAR_SPIKE_2_CONTROL_POSE, poses.NEAR_SECOND_SPIKE))
                .setLinearHeadingInterpolation(poses.FAR_SCORE_POSE.getHeading(), poses.NEAR_SECOND_SPIKE.getHeading())
                .build();

        FAR_SCORE_SPIKE_2 = follower.pathBuilder()
                .addPath(new BezierLine(poses.NEAR_SECOND_SPIKE, poses.FAR_SCORE_POSE))
                .setLinearHeadingInterpolation(poses.NEAR_SECOND_SPIKE.getHeading(), poses.FAR_SCORE_POSE.getHeading())
                .build();

        FAR_SPIKE_1 = follower.pathBuilder()
                .addPath(new BezierCurve(poses.FAR_SCORE_POSE, poses.FAR_SPIKE_1_CONTROL_POSE, poses.NEAR_FIRST_SPIKE))
                .setLinearHeadingInterpolation(poses.FAR_SCORE_POSE.getHeading(), poses.NEAR_FIRST_SPIKE.getHeading())
                .build();

        FAR_SCORE_SPIKE_1 = follower.pathBuilder()
                .addPath(new BezierLine(poses.NEAR_FIRST_SPIKE, poses.FAR_SCORE_POSE))
                .setLinearHeadingInterpolation(poses.NEAR_FIRST_SPIKE.getHeading(), poses.FAR_SCORE_POSE.getHeading())
                .build();

        FAR_GATE = follower.pathBuilder()
                .addPath(new BezierLine(poses.FAR_SCORE_POSE, poses.NEAR_GATE_POSE))
                .setLinearHeadingInterpolation(poses.FAR_SCORE_POSE.getHeading(), poses.NEAR_GATE_POSE.getHeading())
                .build();

        FAR_GATE_2 = follower.pathBuilder()
                .addPath(new BezierCurve(poses.NEAR_GATE_POSE, poses.NEAR_GATE_CONTROL_2nd, poses.NEAR_GATE_POSE_2))
                .setLinearHeadingInterpolation(poses.NEAR_GATE_POSE.getHeading(), poses.NEAR_GATE_POSE_2.getHeading())
                .build();

        FAR_SCORE_GATE = follower.pathBuilder()
                .addPath(new BezierLine(poses.NEAR_GATE_POSE_2, poses.FAR_SCORE_POSE))
                .setLinearHeadingInterpolation(poses.NEAR_GATE_POSE_2.getHeading(), poses.FAR_SCORE_POSE.getHeading())
                .build();
    }
}