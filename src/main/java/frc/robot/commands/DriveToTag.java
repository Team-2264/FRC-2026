package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.ElasticDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;

/**
 * DriveToTag - Drives toward a visible AprilTag and aligns to end up in front of it.
 * Rotates to face the tag, strafes to center it, and drives forward.
 * Press button once to start, auto-stops when 0.5m from tag.
 */
public class DriveToTag extends Command {

    private final Swerve swerve;
    private final Vision2025 vision;
    private final double targetDistance;
    private boolean reachedTarget;
    
    // Speed constants
    private static final double MAX_FORWARD_SPEED = 1.5;
    private static final double MAX_STRAFE_SPEED = 0.8;
    private static final double MAX_ROT_SPEED = 1.5;
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
        reachedTarget = false;
    ElasticDashboard.putBoolean("DriveToTag/Active", true);
    }

    @Override
    public void execute() {
        Optional<Transform3d> targetTransform = vision.getTargetTransform();
        
        if (targetTransform.isPresent()) {
            Transform3d transform = targetTransform.get();
            
            // Tag position relative to camera
            double tagX = transform.getX();  // Forward distance
            double tagY = transform.getY();  // Left/right offset (positive = left)
            
            // Calculate angle to tag (positive = tag is to the left)
            double angleToTag = Math.atan2(tagY, tagX);
            
            double distanceToGo = tagX - targetDistance;
            
            // Check if we've reached the target and are flat to the tag
            if (distanceToGo <= 0.1 && Math.abs(tagY) < 0.15) {
                reachedTarget = true;
                swerve.drive(new Translation2d(0, 0), 0);
                swerve.setFieldRelative(true);
                ElasticDashboard.putString("DriveToTag/Status", "REACHED");
                return;
            }
            
            // Forward speed - proportional to distance
            double forwardSpeed = 0;
            if (distanceToGo > 0.05) {
                double speedRatio = Math.min(distanceToGo / SLOWDOWN_START, 1.0);
                forwardSpeed = speedRatio * MAX_FORWARD_SPEED;
                forwardSpeed = Math.max(forwardSpeed, 0.15);
            }
            
            // Strafe speed - move sideways to center on tag
            double strafeSpeed = tagY * 1.0;
            strafeSpeed = clamp(strafeSpeed, -MAX_STRAFE_SPEED, MAX_STRAFE_SPEED);
            
            // Rotation speed - drive tagY to zero so robot ends up FLAT (perpendicular) to the tag
            // When tagY is 0, we are directly in front of the tag = flat/square alignment
            double rotSpeed = tagY * 2.0;
            rotSpeed = clamp(rotSpeed, -MAX_ROT_SPEED, MAX_ROT_SPEED);
            
            swerve.setFieldRelative(false);
            swerve.drive(new Translation2d(forwardSpeed, strafeSpeed), rotSpeed);
            
            ElasticDashboard.putNumber("DriveToTag/Distance", tagX);
            ElasticDashboard.putNumber("DriveToTag/Offset", tagY);
            ElasticDashboard.putNumber("DriveToTag/Angle", Math.toDegrees(angleToTag));
            ElasticDashboard.putNumber("DriveToTag/Speed", forwardSpeed);
            ElasticDashboard.putString("DriveToTag/Status", "DRIVING");
        } else {
            // No tag visible - stop but keep command running briefly
            swerve.drive(new Translation2d(0, 0), 0);
            ElasticDashboard.putString("DriveToTag/Status", "NO TAG");
        }
    }
    
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public boolean isFinished() {
        return reachedTarget;
    }

    @Override
    public void end(boolean interrupted) {
        // Make sure robot is fully stopped and reset
        swerve.drive(new Translation2d(0, 0), 0);
        swerve.setFieldRelative(true);
    ElasticDashboard.putBoolean("DriveToTag/Active", false);
    ElasticDashboard.putString("DriveToTag/Status", interrupted ? "INTERRUPTED" : "COMPLETE");
    }
}
