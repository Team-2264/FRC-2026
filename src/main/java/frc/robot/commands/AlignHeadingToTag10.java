package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;

/**
 * Rotate the robot to face AprilTag 10 (central hub reference) using vision
 * pose.
 * Positive vision error means the tag is to the left; the robot turns left to
 * center it.
 */
public class AlignHeadingToTag10 extends Command {
    private final Swerve swerve;
    private final Vision2025 vision;

    // Tunables
    private static final double kP = 3.0; // rad/s per rad of heading error
    private static final double toleranceDeg = 2.0;
    private static final double omegaLimit = Constants.Swerve.maxAngularVelocity * 0.45; // cap turn speed

    public AlignHeadingToTag10(Swerve swerve, Vision2025 vision) {
        this.swerve = swerve;
        this.vision = vision;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // Ensure field-relative orientation control
        swerve.setFieldRelative(true);
    }

    @Override
    public void execute() {
        Optional<Double> errorOpt = vision.headingErrorToTag10Deg();
        if (errorOpt.isEmpty()) {
            // No vision: hold still
            swerve.drive(new Translation2d(), 0.0);
            return;
        }

        double errorDeg = errorOpt.get();
        if (Math.abs(errorDeg) < toleranceDeg) {
            swerve.drive(new Translation2d(), 0.0);
            return;
        }

        double omega = MathUtil.clamp(Math.toRadians(errorDeg) * kP, -omegaLimit, omegaLimit);
        swerve.drive(new Translation2d(), omega);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(), 0.0);
    }

    @Override
    public boolean isFinished() {
        // Use whileTrue binding; command ends when input released.
        return false;
    }
}
