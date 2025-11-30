package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * Simple static field serving as a storage medium for the bot's pose and alliance color.
 * This allows different classes/opmodes to set and read from a central source of truth.
 * A static field allows data to persist between opmodes.
 */
public class CrossOpModeStorage {
    //0,0 and obelisk heading is the default
    public static Pose2d currentPose = new Pose2d(new Vector2d(0, 0), Math.toRadians(180));

    public static AllianceColors allianceColor = AllianceColors.BLUE;
}
