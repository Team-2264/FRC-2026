package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.ElasticDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision2025;

/**
 * ShootBall command - adjusts shooter speed based on distance to AprilTag 10.
 * If tag 10 is not visible, the motor does not run at all.
 * Further from tag → faster motor (up to 75%).
 * At 1m and below → proportionally lower (down to ~10%).
 * Bind to triangle button - hold to shoot.
 */
public class ShootBall extends Command {

    private final Shooter shooter;
    private final Vision2025 vision;

    // Delay before starting feeder so flywheel can spin up
    private static final double FEED_DELAY_SEC = 0.5;
    private final Timer feedDelayTimer = new Timer();
    private boolean hasTarget = false;

    /**
     * Creates a ShootBall command with distance-based speed.
     * 
     * @param shooter The shooter subsystem
     * @param vision  The vision subsystem for distance measurement
     */
    public ShootBall(Shooter shooter, Vision2025 vision) {
        this.shooter = shooter;
        this.vision = vision;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        feedDelayTimer.reset();
        feedDelayTimer.start();
        hasTarget = false;
        ElasticDashboard.putBoolean("ShootBall/Active", true);
    }

    @Override
    public void execute() {
        // Get distance to tag 10 specifically
        Optional<Transform3d> targetTransform = vision.getTransformToTag(10);

        if (targetTransform.isPresent()) {
            if (!hasTarget) {
                // First frame seeing the tag — restart delay timer
                feedDelayTimer.reset();
                feedDelayTimer.start();
            }
            hasTarget = true;

            // Get distance (X component is forward distance to tag)
            double distance = targetTransform.get().getX();
            shooter.shootForDistance(distance);
            ElasticDashboard.putNumber("ShootBall/Distance", distance);
            ElasticDashboard.putBoolean("ShootBall/TagVisible", true);

            // Start feeder after delay so shooter can spin up
            if (feedDelayTimer.hasElapsed(FEED_DELAY_SEC)) {
                shooter.startFeeder();
            } else {
                shooter.stopFeeder();
            }
            ElasticDashboard.putNumber("ShootBall/FeedDelayRemaining",
                    Math.max(0, FEED_DELAY_SEC - feedDelayTimer.get()));
        } else {
            // No tag 10 visible — still shoot at default speed so driver can fire
            if (!hasTarget) {
                feedDelayTimer.reset();
                feedDelayTimer.start();
            }
            hasTarget = false;

            shooter.shoot();
            ElasticDashboard.putBoolean("ShootBall/TagVisible", false);
            ElasticDashboard.putNumber("ShootBall/Distance", -1);

            if (feedDelayTimer.hasElapsed(FEED_DELAY_SEC)) {
                shooter.startFeeder();
            } else {
                shooter.stopFeeder();
            }
            ElasticDashboard.putNumber("ShootBall/FeedDelayRemaining",
                    Math.max(0, FEED_DELAY_SEC - feedDelayTimer.get()));
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
        feedDelayTimer.stop();
        feedDelayTimer.reset();
        hasTarget = false;
        ElasticDashboard.putBoolean("ShootBall/Active", false);
    }
}
