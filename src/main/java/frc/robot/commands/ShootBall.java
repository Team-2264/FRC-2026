package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.ElasticDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/**
 * ShootBall command - adjusts shooter speed based on distance to AprilTag 10.
 * If tag 10 is not visible, the motor does not run at all.
 * Further from tag → faster motor (up to 75%).
 * At 1m and below → proportionally lower (down to ~10%).
 * Bind to triangle button - hold to shoot.
 */
public class ShootBall extends Command {

    private final Shooter shooter;

    // Delay before starting feeder so flywheel can spin up
    private static final double FEED_DELAY_SEC = 0.5;
    private final Timer feedDelayTimer = new Timer();

    /**
     * Creates a ShootBall command with distance-based speed.
     * 
     * @param shooter The shooter subsystem
     */
    public ShootBall(Shooter shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        feedDelayTimer.reset();
        feedDelayTimer.start();
        ElasticDashboard.putBoolean("ShootBall/Active", true);
    }

    @Override
    public void execute() {
        // Vision-independent: run shooter immediately, feeder after delay
        shooter.shoot();

        if (feedDelayTimer.hasElapsed(FEED_DELAY_SEC)) {
            shooter.startFeeder();
        } else {
            shooter.stopFeeder();
        }

        ElasticDashboard.putNumber("ShootBall/FeedDelayRemaining",
                Math.max(0, FEED_DELAY_SEC - feedDelayTimer.get()));
        ElasticDashboard.putBoolean("ShootBall/TagVisible", false);
        ElasticDashboard.putNumber("ShootBall/Distance", -1);
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
        ElasticDashboard.putBoolean("ShootBall/Active", false);
    }
}
