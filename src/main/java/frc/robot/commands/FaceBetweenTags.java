package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ElasticDashboard;
import frc.robot.subsystems.Swerve;

/**
 * Faces the midpoint between two AprilTags (e.g., tunnel and hub).
 * Uses the official 2026 field layout to look up tag poses, then delegates to
 * {@link FaceGoal} with a custom target point. Tag IDs are placeholders—tune as
 * needed for your field setup.
 */
public class FaceBetweenTags extends Command {
    // Placeholder IDs: update to the actual tunnel and hub tag IDs
    private final int tunnelTagId;
    private final int hubTagId;

    private final Swerve swerve;
    private final AprilTagFieldLayout layout;
    private Command delegate;

    public FaceBetweenTags(Swerve swerve, int tunnelTagId, int hubTagId) {
        this.swerve = swerve;
        this.tunnelTagId = tunnelTagId;
        this.hubTagId = hubTagId;
        this.layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        Optional<Pose3d> tunnelPose = layout.getTagPose(tunnelTagId);
        Optional<Pose3d> hubPose = layout.getTagPose(hubTagId);

        if (tunnelPose.isEmpty() || hubPose.isEmpty()) {
            ElasticDashboard.putString("FaceBetweenTags/Status", "Tag pose missing");
            delegate = null;
            return;
        }

        double midX = (tunnelPose.get().getX() + hubPose.get().getX()) / 2.0;
        double midY = (tunnelPose.get().getY() + hubPose.get().getY()) / 2.0;

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
}
