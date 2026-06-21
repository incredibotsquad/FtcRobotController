package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class AutoAimTurretCommand extends CommandBase {
    private final TurretSubsystem turret;
    private final OdometrySubsystem odometry;

    // Define the fixed field coordinate you want to point at (e.g., center of the backdrop or goal)

    //TODO: UPDATE THESE NUMBERS
    private static final double TARGET_X = 72.0;
    private static final double TARGET_Y = 0.0;

    public AutoAimTurretCommand(TurretSubsystem turret, OdometrySubsystem odometry) {
        this.turret = turret;
        this.odometry = odometry;
        
        // This command strictly controls the turret
        addRequirements(turret);
    }

    @Override
    public void execute() {
        // 1. Get current robot posture from odometry
        Pose2d currentPose = odometry.getPose();
        
        // 2. Calculate trigonometry delta relative to the target
        double deltaX = TARGET_X - currentPose.getX();
        double deltaY = TARGET_Y - currentPose.getY();
        
        // Math.atan2 returns radians; convert it to degrees for your turret
        double absoluteTargetAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));
        
        // 3. Compensate for the robot's own chassis heading
        double robotHeading = currentPose.getRotation().getDegrees();
        double localizedTargetAngle = absoluteTargetAngle - robotHeading;

        // 4. Update the turret PID controller
        turret.alignToAngle(turret.getTurretAngle(), localizedTargetAngle);
    }

    @Override
    public boolean isFinished() {
        return false; // Returns false so it runs continuously in the background
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}