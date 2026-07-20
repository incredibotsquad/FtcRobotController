package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class Poses {

    public final Pose NEAR_PARK;

    public final Pose FAR_INIT;
    public final Pose FAR_LAUNCH;

    public final Pose HUMAN_PLAYER_POSE;
    public final Pose HUMAN_JIGGLE_BACKWARD;
    public final Pose THIRD_SPIKE;
    public final Pose FAR_SPIKE_3_CONTROL;
    public final Pose FAR_SPIKE_2_CONTROL;
    public final Pose FAR_SPIKE_1_CONTROL;
    public final Pose FAR_PARK, FAR_1_PATH_GATE_CONTROL_1, FAR_1_PATH_GATE_CONTROL_2;

    public final Pose NEAR_INIT, NEAR_FINAL_LAUNCH, ONE_PATH_GATE_INTAKE, GATE_INTAKE_JIGGLE_BACKWORD_POSE, OPEN_GATE_CONTROL, OPEN_GATE, NEAR_1_PATH_GATE_CONTROL_1, NEAR_1_PATH_GATE_CONTROL_2, NEAR_LAUNCH, FIRST_SPIKE, SECOND_SPIKE, NEAR_SPIKE_2_CONTROL, NEAR_HUMAN_CONTROL, NEAR_SPIKE_3_CONTROL;

    public Poses(boolean isRed) {

        // ==================== COMMON POSES ====================

        FIRST_SPIKE = maybeMirror(new Pose(124, 85, Math.toRadians(0)), isRed);
        SECOND_SPIKE = maybeMirror(new Pose(130, 55, Math.toRadians(0)), isRed);
        THIRD_SPIKE = maybeMirror(new Pose(130, 34, Math.toRadians(0)), isRed);

        OPEN_GATE = maybeMirror(new Pose(130, 73, Math.toRadians(0)), isRed);
        OPEN_GATE_CONTROL = maybeMirror(new Pose(115, 70, Math.toRadians(0)), isRed);

        HUMAN_PLAYER_POSE = maybeMirror(new Pose(130, 8, Math.toRadians(0)), isRed);
        HUMAN_JIGGLE_BACKWARD = maybeMirror(new Pose(125, 10, Math.toRadians(0)), isRed);

        ONE_PATH_GATE_INTAKE = maybeMirror(new Pose(127, 52, Math.toRadians(60)), isRed);
        GATE_INTAKE_JIGGLE_BACKWORD_POSE = maybeMirror(new Pose(127, 47, Math.toRadians(60)), isRed);

        // ==================== FAR POSES ====================

        FAR_INIT = maybeMirror(new Pose(85, 9, Math.toRadians(0)), isRed);
        FAR_LAUNCH = maybeMirror(new Pose(90, 15, Math.toRadians(0)), isRed);
        FAR_PARK = maybeMirror(new Pose(96, 36), isRed);

        FAR_SPIKE_1_CONTROL = maybeMirror(new Pose(85, 89), isRed);
        FAR_SPIKE_2_CONTROL = maybeMirror(new Pose(90, 66), isRed);
        FAR_SPIKE_3_CONTROL = maybeMirror(new Pose(90, 40), isRed);

        FAR_1_PATH_GATE_CONTROL_1 = maybeMirror(new Pose(145, 112, Math.toRadians(0)), isRed);
        FAR_1_PATH_GATE_CONTROL_2 = maybeMirror(new Pose(120, 49, Math.toRadians(0)), isRed);

        // ==================== NEAR POSES ====================
        NEAR_INIT = maybeMirror(new Pose(122, 120, Math.toRadians(37)), isRed);
        NEAR_LAUNCH = maybeMirror(new Pose(91, 85, Math.toRadians(0)), isRed);
        NEAR_FINAL_LAUNCH = maybeMirror(new Pose(87, 112, Math.toRadians(0)), isRed);
        NEAR_PARK = maybeMirror(new Pose(105, 70, Math.toRadians(0)), isRed);

        NEAR_SPIKE_2_CONTROL = maybeMirror(new Pose(90, 55), isRed);
        NEAR_SPIKE_3_CONTROL = maybeMirror(new Pose(83, 26), isRed);

        NEAR_1_PATH_GATE_CONTROL_1 = maybeMirror(new Pose(138, 77, Math.toRadians(0)), isRed);
        NEAR_1_PATH_GATE_CONTROL_2 = maybeMirror(new Pose(120, 49, Math.toRadians(0)), isRed);

        NEAR_HUMAN_CONTROL = maybeMirror(new Pose(80, 5), isRed);
    }

    private Pose maybeMirror(Pose pose, boolean isRed) {
        return isRed ? pose : pose.mirror(144);
    }
}
