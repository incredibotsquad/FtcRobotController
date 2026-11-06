package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

/**
 * Fixed-stance relocalization in whichever field corner the robot is in.
 *
 * Push the robot square into any corner, against both walls, facing an alliance wall (the red
 * wall, heading 180, or the blue wall, heading 0), and press the button. The corner is the one
 * nearest the current odometry position, and the heading is whichever of 0 / 180 is nearest the
 * current odometry heading, so odometry only has to be roughly right (closer to this corner than
 * to any other, and within 90 deg). The pose is then reset to that corner's exact stance.
 *
 * Field frame: Pedro, inches, 144 x 144, origin at the red/audience corner. With heading 0 or 180
 * the robot's length runs along X and its width along Y, so the robot center sits
 * CORNER_X_OFFSET_IN from the red/blue wall and CORNER_Y_OFFSET_IN from the audience/far wall.
 * VERIFY both offsets on the robot (half the bumper-to-bumper length / width).
 */
@Configurable
public class RelocalizeCommand extends CommandBase {

    private static final double FIELD_SIZE = 144;

    /** Robot center to the red/blue wall when pushed against it (half the robot's length). */
    public static double CORNER_X_OFFSET_IN = 8;
    /** Robot center to the audience/far wall when pushed against it (half the robot's width). */
    public static double CORNER_Y_OFFSET_IN = 6.5;

    private final OdometrySubsystem odometry;
    private final TelemetryManager telemetry;

    public RelocalizeCommand(OdometrySubsystem odometry, TelemetryManager telemetry) {
        this.odometry = odometry;
        this.telemetry = telemetry;
        // No requirements: it only resets the pose, so it never interrupts driving or aiming.
    }

    @Override
    public void initialize() {
        Pose2d current = odometry.getPose();

        // Nearest corner: each axis independently picks the nearer wall.
        boolean nearRedWall = current.getX() < FIELD_SIZE / 2;
        boolean nearAudienceWall = current.getY() < FIELD_SIZE / 2;
        double x = nearRedWall ? CORNER_X_OFFSET_IN : FIELD_SIZE - CORNER_X_OFFSET_IN;
        double y = nearAudienceWall ? CORNER_Y_OFFSET_IN : FIELD_SIZE - CORNER_Y_OFFSET_IN;

        // Facing an alliance wall: 0 (blue wall) or 180 (red wall), whichever is nearer.
        double headingDeg = Math.cos(current.getHeading()) >= 0 ? 0 : 180;

        String corner = (nearRedWall ? "red" : "blue") + "/" + (nearAudienceWall ? "audience" : "far");
        Log.i("RelocalizeCommand", String.format("Relocalize: odometry was (%.1f, %.1f, %.0f deg) -> %s corner (%.1f, %.1f, %.0f deg)",
                current.getX(), current.getY(), Math.toDegrees(current.getHeading()), corner, x, y, headingDeg));
        telemetry.addData("Relocalized to", String.format("%s corner (%.1f, %.1f, %.0f deg)", corner, x, y, headingDeg));

        odometry.resetPose(x, y, headingDeg); // also writes CrossOpModeStorage.currentPose
    }

    @Override
    public boolean isFinished() {
        return true; // one-shot
    }
}
