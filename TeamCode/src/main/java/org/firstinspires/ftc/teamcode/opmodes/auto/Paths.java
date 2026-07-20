package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

public class Paths {

    private Follower follower;
    private Poses poses;

    public static double slowDownPoint = 0.6;

    public static double speedUpPoint = 1;

    public PathChain NEAR_PRELOADS, NEAR_JIGGLE_GATE_PATH_1, NEAR_JIGGLE_GATE_PATH_2, NEAR_OPEN_GATE, NEAR_SCORE_LAST_GATE, NEAR_GATE_1_PATH, NEAR_SCORE_GATE, NEAR_GATE, NEAR_GATE_2, NEAR_SPIKE_1, NEAR_SCORE_SPIKE_1_AFTER_GATE, NEAR_SPIKE_2, NEAR_SCORE_SPIKE_2, NEAR_HUMAN_PLAYER, NEAR_SCORE_HUMAN, NEAR_SPIKE_3, NEAR_SCORE_SPIKE_3, NEAR_TUNNEL, NEAR_SCORE_TUNNEL, FAR_SCORE_HUMAN, NEAR_LEAVE;
    public PathChain FAR_HUMAN_PLAYER, FAR_PRELOADS, FAR_PARK, FAR_SPIKE_3, FAR_SCORE_SPIKE_3, FAR_SPIKE_2, FAR_SCORE_SPIKE_2, FAR_SPIKE_1, FAR_SCORE_SPIKE_1, FAR_GATE, FAR_GATE_2, FAR_SCORE_GATE, FAR_JIGGLE_GATE_PATH_1, FAR_JIGGLE_GATE_PATH_2;

    public Paths(Follower follower, Poses poses) {
        this.follower = follower;
        this.poses = poses;
    }

    public void buildNearPaths() {
        NEAR_OPEN_GATE = follower.pathBuilder()
                .addPath(new BezierCurve(poses.FIRST_SPIKE, poses.OPEN_GATE_CONTROL, poses.OPEN_GATE))
                .setLinearHeadingInterpolation(poses.FIRST_SPIKE.getHeading(), poses.OPEN_GATE.getHeading())
                .build();

        NEAR_PRELOADS = follower.pathBuilder()
                .addPath(new BezierLine(poses.NEAR_INIT, poses.NEAR_LAUNCH))
                .setLinearHeadingInterpolation(poses.NEAR_INIT.getHeading(), poses.NEAR_LAUNCH.getHeading())
                .build();

        NEAR_SPIKE_1 = follower.pathBuilder()
                .addPath(new BezierLine(poses.NEAR_LAUNCH, poses.FIRST_SPIKE))
                .setLinearHeadingInterpolation(poses.NEAR_LAUNCH.getHeading(), poses.FIRST_SPIKE.getHeading())
                .build();

        NEAR_SCORE_SPIKE_1_AFTER_GATE = follower.pathBuilder()
                .addPath(new BezierLine(poses.OPEN_GATE, poses.NEAR_LAUNCH))
                .setLinearHeadingInterpolation(poses.FIRST_SPIKE.getHeading(), poses.NEAR_LAUNCH.getHeading())
                .build();

        NEAR_SPIKE_2 = follower.pathBuilder()
                .addPath(new BezierCurve(poses.NEAR_LAUNCH, poses.NEAR_SPIKE_2_CONTROL, poses.SECOND_SPIKE))
                .setLinearHeadingInterpolation(poses.NEAR_LAUNCH.getHeading(), poses.SECOND_SPIKE.getHeading())
                .build();

        NEAR_SCORE_SPIKE_2 = follower.pathBuilder()
                .addPath(new BezierLine(poses.SECOND_SPIKE, poses.NEAR_LAUNCH))
                .setLinearHeadingInterpolation(poses.SECOND_SPIKE.getHeading(), poses.NEAR_LAUNCH.getHeading())
                .build();

        NEAR_GATE_1_PATH = follower.pathBuilder()
                .addPath(new BezierCurve(poses.NEAR_LAUNCH, poses.NEAR_1_PATH_GATE_CONTROL_1, poses.NEAR_1_PATH_GATE_CONTROL_2, poses.ONE_PATH_GATE_INTAKE))
                .setLinearHeadingInterpolation(poses.NEAR_LAUNCH.getHeading(), poses.ONE_PATH_GATE_INTAKE.getHeading())
                .build();

        NEAR_SCORE_GATE = follower.pathBuilder()
                .addPath(new BezierLine(poses.ONE_PATH_GATE_INTAKE, poses.NEAR_LAUNCH))
                .setLinearHeadingInterpolation(poses.ONE_PATH_GATE_INTAKE.getHeading(), poses.NEAR_LAUNCH.getHeading())
                .build();

        NEAR_SCORE_LAST_GATE = follower.pathBuilder()
                .addPath(new BezierLine(poses.ONE_PATH_GATE_INTAKE, poses.NEAR_FINAL_LAUNCH))
                .setLinearHeadingInterpolation(poses.ONE_PATH_GATE_INTAKE.getHeading(), poses.NEAR_FINAL_LAUNCH.getHeading())
                .build();

        NEAR_JIGGLE_GATE_PATH_1 = follower.pathBuilder()
                .addPath(new BezierLine(poses.ONE_PATH_GATE_INTAKE, poses.GATE_INTAKE_JIGGLE_BACKWORD_POSE))
                .setLinearHeadingInterpolation(poses.ONE_PATH_GATE_INTAKE.getHeading(), poses.GATE_INTAKE_JIGGLE_BACKWORD_POSE.getHeading())
                .build();

        NEAR_JIGGLE_GATE_PATH_2 = follower.pathBuilder()
                .addPath(new BezierLine(poses.GATE_INTAKE_JIGGLE_BACKWORD_POSE, poses.ONE_PATH_GATE_INTAKE))
                .setLinearHeadingInterpolation(poses.GATE_INTAKE_JIGGLE_BACKWORD_POSE.getHeading(), poses.ONE_PATH_GATE_INTAKE.getHeading())
                .build();

        NEAR_LEAVE = follower.pathBuilder()
                .addPath(new BezierLine(poses.NEAR_LAUNCH, poses.OPEN_GATE_CONTROL))
                .setLinearHeadingInterpolation(poses.NEAR_LAUNCH.getHeading(), poses.OPEN_GATE_CONTROL.getHeading())
                .build();
    }

    public void buildFarPaths() {
        FAR_PRELOADS = follower.pathBuilder()
                .addPath(new BezierLine(poses.FAR_INIT, poses.FAR_LAUNCH))
                .setLinearHeadingInterpolation(poses.FAR_INIT.getHeading(), poses.FAR_LAUNCH.getHeading())
                .build();

        FAR_HUMAN_PLAYER = follower.pathBuilder()
                .addPath(new BezierLine(poses.FAR_LAUNCH, poses.HUMAN_PLAYER_POSE))
                .setLinearHeadingInterpolation(poses.FAR_LAUNCH.getHeading(), poses.HUMAN_PLAYER_POSE.getHeading())
                .build();

        FAR_JIGGLE_GATE_PATH_1 = follower.pathBuilder()
                .addPath(new BezierLine(poses.HUMAN_PLAYER_POSE, poses.HUMAN_JIGGLE_BACKWARD))
                .setLinearHeadingInterpolation(poses.HUMAN_PLAYER_POSE.getHeading(), poses.HUMAN_JIGGLE_BACKWARD.getHeading())
                .build();

        FAR_JIGGLE_GATE_PATH_2 = follower.pathBuilder()
                .addPath(new BezierLine(poses.HUMAN_JIGGLE_BACKWARD, poses.HUMAN_PLAYER_POSE))
                .setLinearHeadingInterpolation(poses.HUMAN_JIGGLE_BACKWARD.getHeading(), poses.HUMAN_PLAYER_POSE.getHeading())
                .build();

        FAR_SCORE_HUMAN = follower.pathBuilder()
                .addPath(new BezierLine(poses.HUMAN_PLAYER_POSE, poses.FAR_LAUNCH))
                .setLinearHeadingInterpolation(poses.HUMAN_PLAYER_POSE.getHeading(), poses.FAR_LAUNCH.getHeading())
                .build();

        FAR_PARK = follower.pathBuilder()
                .addPath(new BezierLine(poses.FAR_LAUNCH, poses.FAR_PARK))
                .setLinearHeadingInterpolation(poses.FAR_LAUNCH.getHeading(), poses.FAR_PARK.getHeading())
                .build();

        FAR_SPIKE_3 = follower.pathBuilder()
                .addPath(new BezierCurve(poses.FAR_LAUNCH, poses.FAR_SPIKE_3_CONTROL, poses.THIRD_SPIKE))
                .setLinearHeadingInterpolation(poses.FAR_LAUNCH.getHeading(), poses.THIRD_SPIKE.getHeading())
                .build();

        FAR_SCORE_SPIKE_3 = follower.pathBuilder()
                .addPath(new BezierLine(poses.THIRD_SPIKE, poses.FAR_LAUNCH))
                .setLinearHeadingInterpolation(poses.FAR_LAUNCH.getHeading(), poses.THIRD_SPIKE.getHeading())
                .build();

        FAR_SPIKE_2 = follower.pathBuilder()
                .addPath(new BezierCurve(poses.FAR_LAUNCH, poses.FAR_SPIKE_2_CONTROL, poses.SECOND_SPIKE))
                .setLinearHeadingInterpolation(poses.FAR_LAUNCH.getHeading(), poses.SECOND_SPIKE.getHeading())
                .build();

        FAR_SCORE_SPIKE_2 = follower.pathBuilder()
                .addPath(new BezierLine(poses.SECOND_SPIKE, poses.FAR_LAUNCH))
                .setLinearHeadingInterpolation(poses.SECOND_SPIKE.getHeading(), poses.FAR_LAUNCH.getHeading())
                .build();

        FAR_SPIKE_1 = follower.pathBuilder()
                .addPath(new BezierCurve(poses.FAR_LAUNCH, poses.FAR_SPIKE_1_CONTROL, poses.FIRST_SPIKE))
                .setLinearHeadingInterpolation(poses.FAR_LAUNCH.getHeading(), poses.FIRST_SPIKE.getHeading())
                .build();

        FAR_SCORE_SPIKE_1 = follower.pathBuilder()
                .addPath(new BezierLine(poses.FIRST_SPIKE, poses.FAR_LAUNCH))
                .setLinearHeadingInterpolation(poses.FIRST_SPIKE.getHeading(), poses.FAR_LAUNCH.getHeading())
                .build();
    }
}
