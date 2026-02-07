package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision2025;

/**
 * ShootBall command - adjusts shooter speed based on distance to AprilTag.
 * Bind to triangle button - hold to shoot.
 */
public class ShootBall extends Command {

    private final Shooter shooter;
    private final Vision2025 vision;

    /**
     * Creates a ShootBall command with distance-based speed.
     * @param shooter The shooter subsystem
     * @param vision The vision subsystem for distance measurement
     */
    public ShootBall(Shooter shooter, Vision2025 vision) {
        this.shooter = shooter;
        this.vision = vision;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("ShootBall/Active", true);
    }

    @Override
    public void execute() {
        // Get distance to tag and adjust speed accordingly
        Optional<Transform3d> targetTransform = vision.getTargetTransform();
        
        if (targetTransform.isPresent()) {
            // Get distance (X component is forward distance to tag)
            double distance = targetTransform.get().getX();
            shooter.shootForDistance(distance);
            SmartDashboard.putNumber("ShootBall/Distance", distance);
            SmartDashboard.putBoolean("ShootBall/TagVisible", true);
        } else {
            // No tag visible - use default speed
            shooter.shoot();
            SmartDashboard.putBoolean("ShootBall/TagVisible", false);
        }
    }

    @Override
    public boolean isFinished() {
        // Command runs until button is released (whileTrue) or interrupted
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        SmartDashboard.putBoolean("ShootBall/Active", false);
    }
}
