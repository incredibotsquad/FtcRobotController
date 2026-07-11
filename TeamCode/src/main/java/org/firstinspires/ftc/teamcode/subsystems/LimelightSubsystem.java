package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimelightSubsystem extends SubsystemBase {
    private final Limelight3A limelight;
    private final TelemetryManager telemetry;

    // Default to an invalid ID so we don't accidentally relocalize on the wrong side
    private int targetTagId = -1;
    public LimelightSubsystem(HardwareMap hardwareMap, TelemetryManager telemetry) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // Ensure your AprilTag pipeline is index 0
        limelight.start();

        this.telemetry = telemetry;
    }

    /**
     * Sets the Alliance-specific tag to look for.
     * @param isRed Pass true for Red Alliance (24), false for Blue Alliance (20).
     */
    public void setAlliance(boolean isRed) {
        this.targetTagId = isRed ? 24 : 20;
    }

    /**
     * Returns the robot's pose on the field based on AprilTags.
     * Returns null if no tags are in view.
     */
    public Pose2d getLatestFieldPose() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Get all AprilTags currently seen by the camera
            List<LLResultTypes.ClassifierResult> targets = result.getClassifierResults();
            // Note: If using the standard 3D pipeline, use getFiducialResults()
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            boolean targetSeen = false;
            for (LLResultTypes.FiducialResult f : fiducials) {
                if (f.getFiducialId() == targetTagId) {
                    targetSeen = true;
                    break;
                }
            }

            // Only return the pose if our specific alliance tag was in the frame
            if (targetSeen) {
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    return new Pose2d(
                            botpose.getPosition().toUnit(DistanceUnit.INCH).x,
                            botpose.getPosition().toUnit(DistanceUnit.INCH).y,
                            new Rotation2d(botpose.getOrientation().getYaw(AngleUnit.RADIANS))
                    );
                }
            }
        }
        return null;
    }

    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }
}