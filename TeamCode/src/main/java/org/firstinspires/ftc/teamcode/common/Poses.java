package org.firstinspires.ftc.teamcode.common;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

/**
 * Field poses for autonomous paths, in Pedro coordinates (inches, headings in degrees).
 *
 * Poses are written once for the red alliance. For blue, the PoseFactory mirrors them across
 * the field's vertical center line: x -> 144 - x, heading -> 180 - heading.
 * Poses given without a heading (path control points) use heading 0; it has no effect on them.
 */
@Configurable
public class Poses {

    /** Mirror line for blue: mirrorX(72) maps x -> 2 * 72 - x. */
    private static final double FIELD_CENTER = 72;

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
    public final Pose RELOCALIZE_STANCE;
    public final Pose NEAR_INIT, NEAR_SPIKE_2_TO_LAUNCH_CONTROL, NEAR_FINAL_LAUNCH, NEAR_1_PATH_GATE_TO_LAUNCH_CONTROL, ONE_PATH_GATE_INTAKE, GATE_INTAKE_JIGGLE_BACKWORD_POSE, OPEN_GATE_CONTROL, OPEN_GATE, NEAR_1_PATH_GATE_CONTROL_1, NEAR_1_PATH_GATE_CONTROL_2, NEAR_LAUNCH, FIRST_SPIKE, SECOND_SPIKE, NEAR_SPIKE_2_CONTROL, NEAR_HUMAN_CONTROL, NEAR_SPIKE_3_CONTROL;

    public Poses(boolean isRed) {
        PoseFactory p = isRed
                ? PoseFactory.degrees()
                : PoseFactory.degrees().mirrorX(FIELD_CENTER);

        // ==================== COMMON POSES ====================

        RELOCALIZE_STANCE = p.of(9, 9, 180);

        FIRST_SPIKE = p.of(124, 85, 0);
        SECOND_SPIKE = p.of(130, 55, 0);
        THIRD_SPIKE = p.of(130, 34, 0);

        OPEN_GATE = p.of(126, 73, 0);
        OPEN_GATE_CONTROL = p.of(115, 70, 0);

        HUMAN_PLAYER_POSE = p.of(130, 8, 0);
        HUMAN_JIGGLE_BACKWARD = p.of(125, 8, 0);

        ONE_PATH_GATE_INTAKE = p.of(127, 52, 60);
        GATE_INTAKE_JIGGLE_BACKWORD_POSE = p.of(127, 47, 60);

        // ==================== FAR POSES ====================

        FAR_INIT = p.of(85, 9, 0);
        FAR_LAUNCH = p.of(90, 15, 0);
        FAR_PARK = p.of(96, 36, 0);

        FAR_SPIKE_1_CONTROL = p.of(85, 89, 0);
        FAR_SPIKE_2_CONTROL = p.of(90, 66, 0);
        FAR_SPIKE_3_CONTROL = p.of(90, 40, 0);

        FAR_1_PATH_GATE_CONTROL_1 = p.of(145, 112, 0);
        FAR_1_PATH_GATE_CONTROL_2 = p.of(120, 49, 0);

        // ==================== NEAR POSES ====================
        NEAR_INIT = p.of(122, 120, 37);
        NEAR_LAUNCH = p.of(91, 85, 0);
        NEAR_FINAL_LAUNCH = p.of(87, 112, 0);
        NEAR_PARK = p.of(105, 70, 0);

        NEAR_SPIKE_2_CONTROL = p.of(90, 55, 0);
        NEAR_SPIKE_2_TO_LAUNCH_CONTROL = p.of(114, 47, 0);
        NEAR_SPIKE_3_CONTROL = p.of(83, 26, 0);

        NEAR_1_PATH_GATE_CONTROL_1 = p.of(138, 77, 0);
        NEAR_1_PATH_GATE_CONTROL_2 = p.of(120, 49, 0);

        NEAR_1_PATH_GATE_TO_LAUNCH_CONTROL = p.of(112, 45, 0);

        NEAR_HUMAN_CONTROL = p.of(80, 5, 0);
    }
}
