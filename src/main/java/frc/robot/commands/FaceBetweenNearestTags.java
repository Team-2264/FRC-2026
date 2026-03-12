package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
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

    // rotation control (mirrors FaceGoal)
    private static final double ROT_KP = 3.0;
    private static final double MAX_ROT = 2.0; // rad/s — full speed
    private static final double SLOW_KP = 1.2;
    private static final double SLOW_MAX_ROT = 0.5; // rad/s — slow speed
    private static final double SLOW_THRESHOLD = 10.0; // degrees — switch to slow below this
    private static final double ANGLE_TOLERANCE = 1.0; // degrees

    public FaceBetweenNearestTags(Swerve swerve, int[] tunnelTagIds, int[] hubTagIds) {
        this.swerve = swerve;
        this.tunnelTagIds = tunnelTagIds;
        this.hubTagIds = hubTagIds;
        this.layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        ElasticDashboard.putString("FaceBetweenTags/Status", "ACTIVE");
    }

    @Override
    public void execute() {
        Pose2d robotPose = swerve.getPose();
        Pose3d tunnel = findNearestTag(tunnelTagIds, robotPose);
        Pose3d hub = findNearestTag(hubTagIds, robotPose);

        if (tunnel == null || hub == null) {
            swerve.drive(new Translation2d(0, 0), 0);
            ElasticDashboard.putString("FaceBetweenTags/Status", "Tag pose missing");
            return;
        }

        double midX = (tunnel.getX() + hub.getX()) / 2.0;
        double midY = (tunnel.getY() + hub.getY()) / 2.0;

        double dx = midX - robotPose.getX();
        double dy = midY - robotPose.getY();
        double targetAngleRad = Math.atan2(dy, dx);
        double errorRad = targetAngleRad - robotPose.getRotation().getRadians();
        while (errorRad > Math.PI)
            errorRad -= 2 * Math.PI;
        while (errorRad < -Math.PI)
            errorRad += 2 * Math.PI;
        double errorDeg = Math.toDegrees(errorRad);

        double kp = (Math.abs(errorDeg) < SLOW_THRESHOLD) ? SLOW_KP : ROT_KP;
        double cap = (Math.abs(errorDeg) < SLOW_THRESHOLD) ? SLOW_MAX_ROT : MAX_ROT;
        double rot = Math.max(-cap, Math.min(cap, errorRad * kp));

        if (Math.abs(errorDeg) < ANGLE_TOLERANCE) {
            rot = 0.0;
        }

        swerve.drive(new Translation2d(0, 0), rot);

        ElasticDashboard.putNumber("FaceBetweenTags/MidX", midX);
        ElasticDashboard.putNumber("FaceBetweenTags/MidY", midY);
        ElasticDashboard.putNumber("FaceBetweenTags/ErrorDeg", errorDeg);
        ElasticDashboard.putString("FaceBetweenTags/Status",
                (Math.abs(errorDeg) < SLOW_THRESHOLD) ? "ROTATING_SLOW" : "ROTATING");
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0);
        ElasticDashboard.putString("FaceBetweenTags/Status", interrupted ? "CANCELLED" : "DONE");
    }

    @Override
    public boolean isFinished() {
        return false; // intended for whileTrue/held usage
    }

    private Pose3d findNearestTag(int[] tagIds, Pose2d robotPose) {
        double bestDist = Double.POSITIVE_INFINITY;
        Pose3d bestPose = null;

        for (int id : tagIds) {
            var pose = layout.getTagPose(id);
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

        return bestPose;
    }
}
