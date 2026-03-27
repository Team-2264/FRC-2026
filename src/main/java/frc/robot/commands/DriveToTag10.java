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
 * DriveToTag10 – Uses the vision-fused pose estimator (odometry + AprilTags)
 * to drive the robot to a point 5 feet directly in front of the nearest target
 * tag (10 or 26), so it works on both alliances.
 *
 * The robot uses swerve.driveToPose() which runs PID on the X, Y, and
 * rotation axes in field-relative coordinates. The command finishes
 * automatically when the robot is within a small tolerance of the goal.
 *
 * Bind to a button press (e.g. d-pad left) with
 * controller.pov(270).onTrue(...).
 */
public class DriveToTag10 extends Command {

    private final Swerve swerve;
    private final Vision2025 vision;

    private static final int[] TARGET_TAGS = { 10, 26 };
    /** Distance to stand in front of tag (meters). 5 ft ≈ 1.524 m */
    private static final double STANDOFF_METERS = Units.feetToMeters(5.0);

    /** Position tolerance – command finishes when within this radius (meters) */
    private static final double POS_TOLERANCE = 0.10; // 10 cm
    /** Heading tolerance – command finishes when within this angle (radians) */
    private static final double ROT_TOLERANCE = Math.toRadians(3.0);

    /** Safety: stop after this many seconds even if not at goal */
    private static final double TIMEOUT_SECONDS = 8.0;

    private Pose2d targetPose;
    private double startTime;

    public DriveToTag10(Swerve swerve, Vision2025 vision) {
        this.swerve = swerve;
        this.vision = vision;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        startTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

        // Pick nearest target tag (10 or 26) and drive in front of it
        Optional<Pose2d> goal = pickNearestTagTargetPose(STANDOFF_METERS);
        if (goal.isPresent()) {
            targetPose = goal.get();
            ElasticDashboard.putString("DriveToTag10/Goal",
                    String.format("(%.2f, %.2f) @ %.1f°",
                            targetPose.getX(), targetPose.getY(),
                            targetPose.getRotation().getDegrees()));
        } else {
            // Tag not in the field layout (should never happen) – cancel immediately
            targetPose = null;
            ElasticDashboard.putString("DriveToTag10/Goal", "TAG NOT FOUND");
        }

        swerve.setFieldRelative(true);
        swerve.unlock(); // Clear any previous lock-on so it doesn't override rotation
        ElasticDashboard.putBoolean("DriveToTag10/Active", true);
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
        ElasticDashboard.putNumber("DriveToTag10/DistError", distErr);
        ElasticDashboard.putNumber("DriveToTag10/RotError", Math.toDegrees(rotErr));
    }

    @Override
    public boolean isFinished() {
        if (targetPose == null) {
            return true; // nothing to do
        }

        // Timeout safety
        if (edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - startTime > TIMEOUT_SECONDS) {
            ElasticDashboard.putString("DriveToTag10/Status", "TIMEOUT");
            return true;
        }

        // Check position + heading tolerance
        Pose2d current = swerve.getPose();
        double distErr = current.getTranslation().getDistance(targetPose.getTranslation());
        double rotErr = Math.abs(current.getRotation().minus(targetPose.getRotation()).getRadians());

        if (distErr < POS_TOLERANCE && rotErr < ROT_TOLERANCE) {
            ElasticDashboard.putString("DriveToTag10/Status", "REACHED");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0);
        ElasticDashboard.putBoolean("DriveToTag10/Active", false);
        ElasticDashboard.putString("DriveToTag10/Status",
                interrupted ? "INTERRUPTED" : "COMPLETE");
    }

    private Optional<Pose2d> pickNearestTagTargetPose(double distanceMeters) {
        Pose2d robotPose = swerve.getPose();
        double bestDist = Double.POSITIVE_INFINITY;
        Integer bestTag = null;

        for (int id : TARGET_TAGS) {
            Optional<edu.wpi.first.math.geometry.Pose3d> poseOpt = vision.getFieldLayout().getTagPose(id);
            if (poseOpt.isEmpty()) {
                continue;
            }
            Pose2d tagPose = poseOpt.get().toPose2d();
            double dist = robotPose.getTranslation().getDistance(tagPose.getTranslation());
            if (dist < bestDist) {
                bestDist = dist;
                bestTag = id;
            }
        }

        if (bestTag == null) {
            return Optional.empty();
        }

        return vision.getTagTargetPose(bestTag, distanceMeters);
    }
}
