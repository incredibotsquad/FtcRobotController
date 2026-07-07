package org.firstinspires.ftc.teamcode.common;

import com.arcrobotics.ftclib.geometry.Pose2d;

/**
 * Simple static field serving as a storage medium for the bot's pose and alliance color.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 */
public class CrossOpModeStorage {

    public static final double BLUE_TARGET_X = 0.0;
    public static final double BLUE_TARGET_Y = 144.0;

    public static final double RED_TARGET_X = 144.0;
    public static final double RED_TARGET_Y = 144.0;

    public static Pose2d currentPose = new Pose2d();

    public static AllianceColors allianceColor = AllianceColors.RED;

    public static double turretPosition = 0;
}