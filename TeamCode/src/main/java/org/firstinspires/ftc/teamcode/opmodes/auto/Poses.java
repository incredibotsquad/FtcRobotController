package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.Pose;

public class Poses {
    public final Pose NEAR_START_POSE;
    public final Pose NEAR_SCORE_POSE;
    public final Pose NEAR_END_POSE;

    public final Pose FAR_START_POSE;
    public final Pose HUMAN_PLAYER_POSE;
    public final Pose FAR_INTAKE2_POSE;
    public final Pose SPIKE_3_CONTROL_POSE;
    public final Pose FAR_SPIKE_2_CONTROL_POSE;
    public final Pose FAR_SPIKE_1_CONTROL_POSE;
    public final Pose FAR_INTAKE3_POSE;
    public final Pose FAR_SCORE_POSE;
    public final Pose FAR_END_POSE;

    public final Pose PICKUP_1_POSE;
    public final Pose PICKUP_2_POSE;
    public final Pose PICKUP_3_POSE;
    public final Pose PICKUP_GATE_POSE;
    public final Pose PICKUP_HUMAN_POSE;

    public final Pose NEAR_INIT, NEAR_FIRST_SHOT, NEAR_FIRST_SPIKE, NEAR_SECOND_SPIKE, NEAR_SECOND_SPIKE_CONTROL, NEAR_SHOT_POSE, NEAR_GATE_POSE, NEAR_GATE_POSE_2, NEAR_GATE_CONTROL_2nd, NEAR_HUMAN_CONTROL;

    public Poses(boolean isRed) {
        NEAR_START_POSE = maybeMirror(new Pose(22, 122, Math.toRadians(324)), isRed);
        NEAR_SCORE_POSE = maybeMirror(new Pose(60, 84, Math.toRadians(135)), isRed);
        NEAR_END_POSE = maybeMirror(new Pose(60, 105), isRed);

        FAR_START_POSE = maybeMirror(new Pose(85, 9, Math.toRadians(0)), isRed);
        HUMAN_PLAYER_POSE = maybeMirror(new Pose(130, 10, Math.toRadians(0)), isRed);
        FAR_INTAKE2_POSE = maybeMirror(new Pose(130, 35, Math.toRadians(0)), isRed);
        SPIKE_3_CONTROL_POSE = maybeMirror(new Pose(100, 35, Math.toRadians(0)), isRed);
        FAR_INTAKE3_POSE = maybeMirror(new Pose(130, 45, Math.toRadians(0)), isRed);
        FAR_SCORE_POSE = maybeMirror(new Pose(85, 15, Math.toRadians(0)), isRed);
        FAR_END_POSE = maybeMirror(new Pose(132, 48), isRed);

        PICKUP_1_POSE = maybeMirror(new Pose(17, 84, Math.toRadians(180)), isRed);
        PICKUP_2_POSE = maybeMirror(new Pose(12, 60, Math.toRadians(180)), isRed);
        PICKUP_3_POSE = maybeMirror(new Pose(12, 36, Math.toRadians(180)), isRed);
        PICKUP_GATE_POSE = maybeMirror(new Pose(125, 68, Math.toRadians(180)), isRed);
        PICKUP_HUMAN_POSE = maybeMirror(new Pose(135, 10, Math.toRadians(0)), isRed);

        NEAR_INIT = maybeMirror(new Pose(122, 120, Math.toRadians(37)), isRed);
        NEAR_FIRST_SHOT = maybeMirror(new Pose(93, 83, Math.toRadians(0)), isRed);



        NEAR_FIRST_SPIKE = maybeMirror(new Pose(121, 85, Math.toRadians(0)), isRed);
        NEAR_SECOND_SPIKE = maybeMirror(new Pose(130, 50, Math.toRadians(0)), isRed);
        NEAR_SECOND_SPIKE_CONTROL = maybeMirror(new Pose(89, 50), isRed);
        NEAR_SHOT_POSE = maybeMirror(new Pose(72, 72, Math.toRadians(0)), isRed);
        NEAR_GATE_POSE = maybeMirror(new Pose(127, 68, Math.toRadians(0)), isRed);
        NEAR_GATE_POSE_2 = maybeMirror(new Pose(132, 50, Math.toRadians(45)), isRed);
        NEAR_GATE_CONTROL_2nd = maybeMirror(new Pose(125, 50), isRed);
        FAR_SPIKE_1_CONTROL_POSE = maybeMirror(new Pose(85, 89), isRed);
        FAR_SPIKE_2_CONTROL_POSE = maybeMirror(new Pose(85, 58), isRed);
        NEAR_HUMAN_CONTROL = maybeMirror(new Pose(120, 10), isRed);
    }

    private Pose maybeMirror(Pose pose, boolean isRed) {
        return isRed ? pose : pose.mirror(144);
    }
}
