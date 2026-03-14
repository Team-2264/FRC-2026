package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.ElasticDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
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
    private final Hopper hopper;

    private static final double SHOOTER_FREE_RPS = 95.0; // Approx free speed of a NEO in rev/s
    private static final double SHOOTER_SPEED_THRESHOLD = 0.95; // 95% of target speed

    /**
     * Creates a ShootBall command with distance-based speed.
     * 
     * @param shooter The shooter subsystem
     * @param hopper The hopper subsystem
     */
    public ShootBall(Shooter shooter, Hopper hopper) {
        this.shooter = shooter;
        this.hopper = hopper;

        addRequirements(shooter);
        addRequirements(hopper);
    }

    @Override
    public void initialize() {
        ElasticDashboard.putBoolean("ShootBall/Active", true);
    }

    @Override
    public void execute() {
        // Vision-independent: run shooter immediately; feed only when up to speed
        shooter.shoot();

        double targetDuty = shooter.getCurrentSpeedDutyCycle();
        double targetRps = Math.abs(targetDuty) * SHOOTER_FREE_RPS;
        boolean upToSpeed = Math.abs(shooter.getVelocity()) >= SHOOTER_SPEED_THRESHOLD * targetRps;

        if (upToSpeed) {
            hopper.startConveyor(Constants.HopperConveyorConfig.intakeDuty);
            shooter.startFeeder();
        } else {
            hopper.stopConveyor();
            shooter.stopFeeder();
        }

        ElasticDashboard.putBoolean("ShootBall/ShooterUpToSpeed", upToSpeed);
        ElasticDashboard.putNumber("ShootBall/TargetRps", targetRps);
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
        hopper.stopConveyor();
        ElasticDashboard.putBoolean("ShootBall/Active", false);
    }
}
