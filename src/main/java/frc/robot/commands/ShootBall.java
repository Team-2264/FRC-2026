package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ElasticDashboard;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision2025;

import java.util.Optional;

/**
 * ShootBall command - adjusts shooter speed based on distance to AprilTag 10 or
 * 26
 * (nearest visible), so it works on both alliances.
 */
public class ShootBall extends Command {

    private final Shooter shooter;
    private final Hopper hopper;
    private final Vision2025 vision; // Add Vision2025 instance

    private static final double SHOOTER_SPEED_THRESHOLD = 0.97; // 95% of target speed
    private static double lastDistanceMeters = -1.0; // Last distance of the robot from tag

    /**
     * Creates a ShootBall command with distance-based speed.
     *
     * @param shooter The shooter subsystem
     * @param hopper  The hopper subsystem
     * @param vision  The vision subsystem
     */
    public ShootBall(Shooter shooter, Hopper hopper, Vision2025 vision) {
        this.shooter = shooter;
        this.hopper = hopper;
        this.vision = vision; // Initialize Vision2025
        lastDistanceMeters = -1.0;
        addRequirements(shooter, hopper, vision); // Add all requirements in one call
    }

    @Override
    public void initialize() {
        ElasticDashboard.putBoolean("ShootBall/Active", true);
    }

    @Override
    public void execute() {
        Optional<Transform3d> tagTransform = vision.getTransformToAny(10, 26);
        double distance = lastDistanceMeters;
        if (tagTransform.isPresent()) {
            Transform3d t = tagTransform.get();
            distance = Math.hypot(t.getX(), t.getY());
            lastDistanceMeters = distance;
        }

        if (lastDistanceMeters < 0) {
            shooter.shoot();
        }

        // Vision-independent: run shooter immediately; feed only when up to speed
        shooter.shootForDistance(distance); // Pass distance to shooter

        double targetRpm = shooter.getCurrentRPM();
        double targetRps = Math.abs(targetRpm) / 60.0;
        boolean upToSpeed = Math.abs(shooter.getVelocity()) >= SHOOTER_SPEED_THRESHOLD * targetRpm;

        if (upToSpeed) {
            hopper.startConveyor(Constants.HopperConveyorConfig.intakeDuty);
            shooter.startFeeder();
        } else {
            hopper.stopConveyor();
            shooter.stopFeeder();
        }

        ElasticDashboard.putBoolean("ShootBall/ShooterUpToSpeed", upToSpeed);
        ElasticDashboard.putNumber("ShootBall/TargetRps", targetRps);
        ElasticDashboard.putBoolean("ShootBall/TagVisible", tagTransform.isPresent());
        ElasticDashboard.putNumber("ShootBall/Distance", distance);
        ElasticDashboard.putNumber("ShootBall/ThresholdRPM", SHOOTER_SPEED_THRESHOLD * targetRpm);
    }

    @Override
    public boolean isFinished() {
        // Command runs until button is released (whileTrue) or interrupted
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hopper.stopConveyor();
        ElasticDashboard.putBoolean("ShootBall/Active", false);
    }
}
