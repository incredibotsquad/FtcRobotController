package org.firstinspires.ftc.teamcode.common;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.bylazar.configurables.annotations.Configurable;

/**
 * Simple static field serving as a storage medium for the bot's pose and alliance color.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 */
@Configurable
public class CrossOpModeStorage {

    // Launch target coordinates live in LaunchTargets (HIVE CELLS, nearest-CELL selection).

//    public static Pose2d currentPose = new Pose2d(40, 6.5, new Rotation2d(Math.toRadians(180)));
    public static Pose2d currentPose = new Pose2d(42.5, 10, new Rotation2d(Math.toRadians(90)));

    public static AllianceColors allianceColor = AllianceColors.RED;

    public static double turretPosition = 0.5; // center aligned
}