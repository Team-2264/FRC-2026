package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.ElasticDashboard;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision2025;

import java.util.Optional;

/**
 * ShootBall command - adjusts shooter speed based on distance to the nearest hub tag.
 */
public class ShootBallNoVision extends Command {

    private final Shooter shooter;
    private final Hopper hopper;
    private final Intake intake;

    private static final double SHOOTER_SPEED_THRESHOLD = 0.97; // 97% of target speed
    private static final double INTAKE_PULSE_PERIOD_SECONDS = 1.0;
    private static double lastDistanceMeters = -1.0; // Last distance of the robot from tag
    private double lastIntakePulseTime = 0.0;

    /**
     * Creates a ShootBall command with distance-based speed.
     *
     * @param shooter The shooter subsystem
     * @param hopper  The hopper subsystem
     * @param vision  The vision subsystem
     */
    public ShootBallNoVision(Shooter shooter, Hopper hopper, Intake intake) {
        this.shooter = shooter;
        this.hopper = hopper;
        this.intake = intake;
        lastDistanceMeters = -1.0;
        addRequirements(shooter, hopper, intake); // Add all requirements in one call
    }

    @Override
    public void initialize() {
        ElasticDashboard.putBoolean("ShootBallNoVision/Active", true);
        lastIntakePulseTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        /* Optional<Transform3d> tagTransform = vision.getNearestHubTagTransform();
        double distance = lastDistanceMeters;
        if (tagTransform.isPresent()) {
            Transform3d t = tagTransform.get();
            distance = Math.hypot(t.getX(), t.getY());
            lastDistanceMeters = distance;
        }           

        if(lastDistanceMeters<0){
            shooter.shoot();
        } */

        // Vision-independent: run shooter immediately; feed only when up to speed
        shooter.shootForDistance(2); // Pass distance to shooter

        double targetRpm = shooter.getCurrentRPM();
        double targetRps = Math.abs(targetRpm) / 60.0;
        boolean upToSpeed = Math.abs(shooter.getVelocity()) >= SHOOTER_SPEED_THRESHOLD * Math.abs(targetRpm);

        if (upToSpeed) {
            hopper.startConveyor(Constants.HopperConveyorConfig.intakeDuty);
            shooter.startFeeder();

            double now = Timer.getFPGATimestamp();
            if ((now - lastIntakePulseTime) >= INTAKE_PULSE_PERIOD_SECONDS) {
                new InstantCommand(() -> intake.startRoller(1.0))
                        .alongWith(new InstantCommand(() -> intake.deployIntake()))
                        .andThen(new WaitCommand(1))
                        .andThen(new InstantCommand(() -> intake.retractIntake()))
                        .schedule();
                lastIntakePulseTime = now;
            }
        } else {
            hopper.stopConveyor();
            shooter.stopFeeder();
        }

        ElasticDashboard.putBoolean("ShootBallNoVision/ShooterUpToSpeed", upToSpeed);
        ElasticDashboard.putNumber("ShootBallNoVision/TargetRps", targetRps);
        //ElasticDashboard.putBoolean("ShootBallNoVision/TagVisible", tagTransform.isPresent());
        //ElasticDashboard.putNumber("ShootBallNoVision/Distance", distance);
        ElasticDashboard.putNumber("ShootBallNoVision/ThresholdRPM", SHOOTER_SPEED_THRESHOLD * targetRpm);
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
        //intake.stopRoller();
        intake.retractIntake();
        ElasticDashboard.putBoolean("ShootBallNoVision/Active", false);
        lastDistanceMeters = -1.0;
    }
}
