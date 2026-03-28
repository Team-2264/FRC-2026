package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.ElasticDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;

/**
 * DriveToHub - Uses the vision-fused pose estimator (odometry + AprilTags)
 * to drive the robot to a point 5 feet directly in front of the nearest hub tag.
 *
 * The robot uses swerve.driveToPose() which runs PID on the X, Y, and
 * rotation axes in field-relative coordinates. The command finishes
 * automatically when the robot is within a small tolerance of the goal.
 *
 * Bind to a button press (e.g. d-pad left) with
 * controller.pov(270).onTrue(...).
 */
public class DriveToHub extends Command {

    private final Swerve swerve;
    private final Vision2025 vision;

    /** Distance to stand in front of hub tag (meters). 5 ft ≈ 1.524 m */
    private static final double STANDOFF_METERS = Units.feetToMeters(5.0);

    /** Position tolerance - command finishes when within this radius (meters) */
    private static final double POS_TOLERANCE = 0.10; // 10 cm
    /** Heading tolerance - command finishes when within this angle (radians) */
    private static final double ROT_TOLERANCE = Math.toRadians(3.0);

    /** Safety: stop after this many seconds even if not at goal */
    private static final double TIMEOUT_SECONDS = 8.0;

    private Pose2d targetPose;
    private double startTime;

    public DriveToHub(Swerve swerve, Vision2025 vision) {
        this.swerve = swerve;
        this.vision = vision;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        // Compute target pose: 5 ft in front of nearest hub tag
        Optional<Pose2d> goal = vision.getNearestHubTargetPose(swerve.getPose(), STANDOFF_METERS);
        if (goal.isPresent()) {
            targetPose = goal.get();
            ElasticDashboard.putString("DriveToHub/Goal",
                    String.format("(%.2f, %.2f) @ %.1f\u00B0",
                            targetPose.getX(), targetPose.getY(),
                            targetPose.getRotation().getDegrees()));
        } else {
            // No hub tag found 
            targetPose = null;
            ElasticDashboard.putString("DriveToHub/Goal", "HUB TAGS NOT FOUND");
        }

        swerve.setFieldRelative(true);
        swerve.unlock(); // Clear any previous lock-on so it doesn't override rotation
        ElasticDashboard.putBoolean("DriveToHub/Active", true);
    }

    @Override
    public void execute() {
        if (targetPose == null) {
            return;
        }

        // driveToPose runs PID each call
        swerve.driveToPose(targetPose);

        // Publish debug info
        Pose2d current = swerve.getPose();
        double distErr = current.getTranslation().getDistance(targetPose.getTranslation());
        double rotErr = Math.abs(current.getRotation().minus(targetPose.getRotation()).getRadians());
        ElasticDashboard.putNumber("DriveToHub/DistError", distErr);
        ElasticDashboard.putNumber("DriveToHub/RotError", Math.toDegrees(rotErr));
    }

    @Override
    public boolean isFinished() {
        if (targetPose == null) {
            return true; // nothing to do
        }

        // Timeout safety
        if (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime > TIMEOUT_SECONDS) {
            ElasticDashboard.putString("DriveToHub/Status", "TIMEOUT");
            return true;
        }

        // Check position + heading tolerance
        Pose2d current = swerve.getPose();
        double distErr = current.getTranslation().getDistance(targetPose.getTranslation());
        double rotErr = Math.abs(current.getRotation().minus(targetPose.getRotation()).getRadians());

        if (distErr < POS_TOLERANCE && rotErr < ROT_TOLERANCE) {
            ElasticDashboard.putString("DriveToHub/Status", "REACHED");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0);
        ElasticDashboard.putBoolean("DriveToHub/Active", false);
        ElasticDashboard.putString("DriveToHub/Status", interrupted ? "INTERRUPTED" : "COMPLETE");
    }
}
