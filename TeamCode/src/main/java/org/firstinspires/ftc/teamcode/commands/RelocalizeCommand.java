package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.common.AllianceColors;
import org.firstinspires.ftc.teamcode.common.CrossOpModeStorage;
import org.firstinspires.ftc.teamcode.common.DriverBanner;
import org.firstinspires.ftc.teamcode.opmodes.auto.Poses;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;

/**
 * Fixed-stance relocalization. Two modes:
 *
 * NEAREST_CORNER (operator LB): the corner nearest the current odometry position, described below.
 * Needs odometry to be roughly right, so a press away from a corner (a mis-press mid-match) can
 * leave the pose so wrong that later NEAREST_CORNER presses pick the wrong corner.
 *
 * ALLIANCE_STANCE (operator left stick button): always our own GARDEN corner, ignoring odometry
 * completely, so it recovers from a bad pose. The stance is Poses.RED_RELOCALIZE_STANCE (red:
 * red/audience corner, against both walls, facing the red alliance wall, the robot's length along
 * the GARDEN tape), rotated 180 deg about the field center for blue. The alliance is
 * CrossOpModeStorage.allianceColor (set by the auto; RED if no auto ran).
 *
 * NEAREST_CORNER details:
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

    public enum Mode { NEAREST_CORNER, ALLIANCE_STANCE }

    private final OdometrySubsystem odometry;
    private final TelemetryManager telemetry;
    private final Mode mode;

    public RelocalizeCommand(OdometrySubsystem odometry, TelemetryManager telemetry) {
        this(odometry, telemetry, Mode.NEAREST_CORNER);
    }

    public RelocalizeCommand(OdometrySubsystem odometry, TelemetryManager telemetry, Mode mode) {
        this.odometry = odometry;
        this.telemetry = telemetry;
        this.mode = mode;
        // No requirements: it only resets the pose, so it never interrupts driving or aiming.
    }

    @Override
    public void initialize() {
        Pose2d current = odometry.getPose();
        if (mode == Mode.ALLIANCE_STANCE) {
            relocalizeToAllianceStance(current);
            return;
        }

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
        DriverBanner.showRelocalized(corner + " corner", x, y, headingDeg);
    }

    /** Resets to our GARDEN-corner stance (see class comment), whatever odometry says. */
    private void relocalizeToAllianceStance(Pose2d current) {
        AllianceColors alliance = CrossOpModeStorage.allianceColor;
        // Read the alliance and the stance at the press, so Panels edits and a late alliance change apply.
        Pose stance = new Poses(alliance == AllianceColors.RED).relocalizeStance();
        double headingDeg = Math.toDegrees(stance.heading());

        Log.i("RelocalizeCommand", String.format("Relocalize (%s stance): odometry was (%.1f, %.1f, %.0f deg) -> (%.1f, %.1f, %.0f deg)",
                alliance, current.getX(), current.getY(), Math.toDegrees(current.getHeading()),
                stance.x(), stance.y(), headingDeg));
        telemetry.addData("Relocalized to", String.format("%s stance (%.1f, %.1f, %.0f deg)",
                alliance, stance.x(), stance.y(), headingDeg));

        odometry.resetPose(stance.x(), stance.y(), headingDeg); // also writes CrossOpModeStorage.currentPose
        DriverBanner.showRelocalized(alliance + " GARDEN corner", stance.x(), stance.y(), headingDeg);
    }

    @Override
    public boolean isFinished() {
        return true; // one-shot
    }
}
