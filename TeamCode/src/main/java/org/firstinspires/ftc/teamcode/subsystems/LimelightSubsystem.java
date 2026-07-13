package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

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

        this.telemetry = telemetry;
    }

    /**
     * Sets the Alliance-specific tag to look for.
     * @param isRed Pass true for Red Alliance (24), false for Blue Alliance (20).
     */
    public void setAlliance(boolean isRed) {
        if (isRed) {
            this.targetTagId = 24;
            limelight.pipelineSwitch(7);
        } else {
            this.targetTagId = 20;
            limelight.pipelineSwitch(6);
        }
        limelight.start();

        Log.i("Limelight subsystem", "Alliance set to red: " + isRed);
    }

    /**
     * Returns the robot's pose on the field based on AprilTags.
     * Returns null if no tags are in view.
     */
    public Pose getLatestFieldPose() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // We only care about Fiducials (AprilTags) for pose estimation
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            boolean targetSeen = false;
            for (LLResultTypes.FiducialResult f : fiducials) {
                if (f.getFiducialId() == targetTagId) {
                    Log.i("Limelight subsystem", "found FiducialResult");
                    targetSeen = true;
                    break;
                }
            }

            // Only return the pose if our specific alliance tag was in the frame
            if (targetSeen) {
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {

                    Log.i("Limelight subsystem", "found botpose from target. X: " + botpose.getPosition().toUnit(DistanceUnit.INCH).x + " y: " + botpose.getPosition().toUnit(DistanceUnit.INCH).y + " R: " + botpose.getOrientation().getYaw(AngleUnit.DEGREES));

                    return new Pose(
                            botpose.getPosition().toUnit(DistanceUnit.INCH).x,
                            botpose.getPosition().toUnit(DistanceUnit.INCH).y,
                            botpose.getOrientation().getYaw(AngleUnit.RADIANS),
                            FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                }
            }
        }
        return null;
    }

    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }
}