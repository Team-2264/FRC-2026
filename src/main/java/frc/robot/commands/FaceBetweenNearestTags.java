package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ElasticDashboard;
import frc.robot.subsystems.Swerve;

/**
 * Faces the midpoint between the nearest tunnel tag and nearest hub tag
 * (from provided tag ID sets). Works on either alliance side because it picks
 * the closest tag in each set based on current robot pose.
 */
public class FaceBetweenNearestTags extends Command {
    private final Swerve swerve;
    private final int[] tunnelTagIds;
    private final int[] hubTagIds;
    private final AprilTagFieldLayout layout;

    private Command delegate;

    public FaceBetweenNearestTags(Swerve swerve, int[] tunnelTagIds, int[] hubTagIds) {
        this.swerve = swerve;
        this.tunnelTagIds = tunnelTagIds;
        this.hubTagIds = hubTagIds;
        this.layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        delegate = null;

        Pose2d robotPose = swerve.getPose();
        Optional<Pose3d> tunnel = findNearestTag(tunnelTagIds, robotPose);
        Optional<Pose3d> hub = findNearestTag(hubTagIds, robotPose);

        if (tunnel.isEmpty() || hub.isEmpty()) {
            ElasticDashboard.putString("FaceBetweenTags/Status", "Tag pose missing");
            return;
        }

        double midX = (tunnel.get().getX() + hub.get().getX()) / 2.0;
        double midY = (tunnel.get().getY() + hub.get().getY()) / 2.0;

        delegate = new FaceGoal(swerve, midX, midY, 0.0);
        delegate.initialize();
        ElasticDashboard.putString("FaceBetweenTags/Status", "Facing midpoint");
        ElasticDashboard.putNumber("FaceBetweenTags/MidX", midX);
        ElasticDashboard.putNumber("FaceBetweenTags/MidY", midY);
    }

    @Override
    public void execute() {
        if (delegate != null) {
            delegate.execute();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if (delegate != null) {
            delegate.end(interrupted);
        }
    }

    @Override
    public boolean isFinished() {
        return delegate == null || delegate.isFinished();
    }

    private Optional<Pose3d> findNearestTag(int[] tagIds, Pose2d robotPose) {
        double bestDist = Double.POSITIVE_INFINITY;
        Pose3d bestPose = null;

        for (int id : tagIds) {
            Optional<Pose3d> pose = layout.getTagPose(id);
            if (pose.isEmpty()) {
                continue;
            }
            double dx = pose.get().getX() - robotPose.getX();
            double dy = pose.get().getY() - robotPose.getY();
            double distSq = dx * dx + dy * dy;
            if (distSq < bestDist) {
                bestDist = distSq;
                bestPose = pose.get();
            }
        }

        return Optional.ofNullable(bestPose);
    }
}
