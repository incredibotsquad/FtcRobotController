package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class Poses {
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

    public static Pose NEAR_INIT, NEAR_1_PATH_GATE_INTAKE, NEAR_1_PATH_GATE_CONTROL_1, NEAR_1_PATH_GATE_CONTROL_2, NEAR_LAUNCH, NEAR_FIRST_SPIKE, NEAR_SECOND_SPIKE, NEAR_SECOND_SPIKE_CONTROL, NEAR_GATE_POSE, NEAR_GATE_POSE_2, NEAR_GATE_CONTROL_2nd, NEAR_HUMAN_CONTROL;

    public Poses(boolean isRed) {
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


        // ==================== NEAR POSES ====================
        NEAR_INIT = maybeMirror(new Pose(122, 120, Math.toRadians(37)), isRed);
        NEAR_LAUNCH = maybeMirror(new Pose(91, 85, Math.toRadians(0)), isRed);

        NEAR_FIRST_SPIKE = maybeMirror(new Pose(126, 85, Math.toRadians(0)), isRed);
        NEAR_SECOND_SPIKE = maybeMirror(new Pose(130, 50, Math.toRadians(0)), isRed);
        NEAR_SECOND_SPIKE_CONTROL = maybeMirror(new Pose(89, 50), isRed);

        NEAR_GATE_POSE = maybeMirror(new Pose(126, 65, Math.toRadians(15)), isRed);
        NEAR_GATE_POSE_2 = maybeMirror(new Pose(132, 50, Math.toRadians(45)), isRed);
        NEAR_GATE_CONTROL_2nd = maybeMirror(new Pose(125, 50), isRed);

        FAR_SPIKE_1_CONTROL_POSE = maybeMirror(new Pose(85, 89), isRed);
        FAR_SPIKE_2_CONTROL_POSE = maybeMirror(new Pose(85, 58), isRed);
        NEAR_HUMAN_CONTROL = maybeMirror(new Pose(120, 10), isRed);

        NEAR_1_PATH_GATE_CONTROL_1 = maybeMirror(new Pose(138, 77, Math.toRadians(0)), isRed);
        NEAR_1_PATH_GATE_CONTROL_2 = maybeMirror(new Pose(120, 49, Math.toRadians(0)), isRed);
        NEAR_1_PATH_GATE_INTAKE = maybeMirror(new Pose(131, 52, Math.toRadians(60)), isRed);
    }

    private Pose maybeMirror(Pose pose, boolean isRed) {
        return isRed ? pose : pose.mirror(144);
    }
}