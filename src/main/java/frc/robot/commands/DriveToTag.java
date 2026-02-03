package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        SmartDashboard.putBoolean("DriveToTag/Active", true);
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
            
            // Check if we've reached the target
            if (distanceToGo <= 0.1 && Math.abs(tagY) < 0.15) {
                reachedTarget = true;
                swerve.drive(new Translation2d(0, 0), 0);
                swerve.setFieldRelative(true);  // Reset field relative immediately
                SmartDashboard.putString("DriveToTag/Status", "REACHED");
                return;
            }
            
            // Forward speed - proportional to distance
            double forwardSpeed = 0;
            if (distanceToGo > 0.05) {
                double speedRatio = Math.min(distanceToGo / SLOWDOWN_START, 1.0);
                forwardSpeed = speedRatio * MAX_FORWARD_SPEED;
                forwardSpeed = Math.max(forwardSpeed, 0.15);
            }
            
            // Strafe speed - move sideways to center on tag (reduced gain)
            double strafeSpeed = tagY * 1.0;  // Proportional to offset
            strafeSpeed = clamp(strafeSpeed, -MAX_STRAFE_SPEED, MAX_STRAFE_SPEED);
            
            // Rotation speed - turn to face the tag (reduced gain)
            double rotSpeed = angleToTag * 1.5;  // Proportional to angle
            rotSpeed = clamp(rotSpeed, -MAX_ROT_SPEED, MAX_ROT_SPEED);
            
            swerve.setFieldRelative(false);
            swerve.drive(new Translation2d(forwardSpeed, strafeSpeed), rotSpeed);
            
            SmartDashboard.putNumber("DriveToTag/Distance", tagX);
            SmartDashboard.putNumber("DriveToTag/Offset", tagY);
            SmartDashboard.putNumber("DriveToTag/Angle", Math.toDegrees(angleToTag));
            SmartDashboard.putNumber("DriveToTag/Speed", forwardSpeed);
            SmartDashboard.putString("DriveToTag/Status", "DRIVING");
        } else {
            // No tag visible - stop but keep command running briefly
            swerve.drive(new Translation2d(0, 0), 0);
            SmartDashboard.putString("DriveToTag/Status", "NO TAG");
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
        SmartDashboard.putBoolean("DriveToTag/Active", false);
        SmartDashboard.putString("DriveToTag/Status", interrupted ? "INTERRUPTED" : "COMPLETE");
    }
}
