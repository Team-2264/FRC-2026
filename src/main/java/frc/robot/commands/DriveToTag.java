package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;

/**
 * DriveToTag - Simple command to drive straight toward a visible AprilTag.
 * Uses camera-relative control - drives FORWARD only toward the tag.
 * Hold the button to drive toward tag, release to stop.
 */
public class DriveToTag extends Command {

    private final Swerve swerve;
    private final Vision2025 vision;
    private final double targetDistance;
    
    // Speed constants
    private static final double MAX_FORWARD_SPEED = 1.5;
    private static final double SLOWDOWN_START = 1.5;

    public DriveToTag(Swerve swerve, Vision2025 vision, double targetDistance) {
        this.swerve = swerve;
        this.vision = vision;
        this.targetDistance = targetDistance;
        addRequirements(swerve);
    }

    public DriveToTag(Swerve swerve, Vision2025 vision) {
        this(swerve, vision, 0.5);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("DriveToTag/Active", true);
    }

    @Override
    public void execute() {
        Optional<Transform3d> targetTransform = vision.getTargetTransform();
        
        if (targetTransform.isPresent()) {
            Transform3d transform = targetTransform.get();
            double tagX = transform.getX();
            double distanceToGo = tagX - targetDistance;
            
            double forwardSpeed = 0;
            if (distanceToGo > 0.05) {
                double speedRatio = Math.min(distanceToGo / SLOWDOWN_START, 1.0);
                forwardSpeed = speedRatio * MAX_FORWARD_SPEED;
                forwardSpeed = Math.max(forwardSpeed, 0.2);
            }
            
            swerve.setFieldRelative(false);
            swerve.drive(new Translation2d(forwardSpeed, 0), 0);
            
            SmartDashboard.putNumber("DriveToTag/Distance", tagX);
            SmartDashboard.putNumber("DriveToTag/Speed", forwardSpeed);
        } else {
            swerve.drive(new Translation2d(0, 0), 0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0);
        swerve.setFieldRelative(true);
        SmartDashboard.putBoolean("DriveToTag/Active", false);
    }
}
