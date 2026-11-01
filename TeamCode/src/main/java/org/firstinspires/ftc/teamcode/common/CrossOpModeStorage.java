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

    public static final double BLUE_TARGET_X = 13.25;
    public static final double BLUE_TARGET_Y = 134.85;

    public static final double RED_TARGET_X = 130.75;
    public static final double RED_TARGET_Y = 134.85;

    public static Pose2d currentPose = new Pose2d(72, 72, new Rotation2d(Math.toRadians(0)));

    public static AllianceColors allianceColor = AllianceColors.RED;

    public static double turretPosition = 0;
}