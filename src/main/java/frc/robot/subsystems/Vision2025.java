package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.ElasticDashboard;

/**
 * Subsystem for vision processing using PhotonVision.
 * 
 */
public class Vision2025 extends SubsystemBase {
    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;
    private PhotonPipelineResult latestResult;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private Field2d field;

    private DoubleLogEntry distancelog;
    private IntegerLogEntry tagNumberLogger;
    private DoubleLogEntry robotPoseXLogger;
    private DoubleLogEntry robotPoseYLogger;
    private DoubleLogEntry robotPoseRotLogger;
    private DoubleLogEntry bestTagXLogger;
    private DoubleLogEntry bestTagYLogger;
    private DoubleLogEntry bestTagZLogger;
    private DoubleLogEntry bestTagYawLogger;
    private DoubleLogEntry bestTagPitchLogger;

    // ── AdvantageScope 3D struct publishers ──
    private final StructPublisher<Pose3d> visionRobotPose3dPub;
    private final StructPublisher<Pose3d> cameraPose3dPub;
    private final StructArrayPublisher<Pose3d> detectedTagPosesPub;

    // Cycle counter for throttling SmartDashboard debug text
    private int periodicCounter = 0;

    /**
     * Constructs a new Vision instance.
     * 
     */
    public Vision2025() {
        camera = new PhotonCamera(Constants.Vision.cameraName);
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // Construct the estimator with PoseStrategy.LOWEST_AMBIGUITY
        // The 3-argument constructor takes (layout, strategy, robotToCamera)
        estimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY,
                Constants.Vision.robotToCamera);

        // Set initial pose to middle of field (for coprocessor multi-tag sync if
        // enabled)
        // 2025 field is 16.54m x 8.23m, blue origin at (0,0)
        // Center is approximately at (8.27, 4.11) in field coordinates
        estimator.setLastPose(new Pose3d(8.27, 4.11, 0.35, new Rotation3d()));

        field = new Field2d();

        // Start data logging and create entries
        DataLogManager.start();
        DataLog log = DataLogManager.getLog();
        distancelog = new DoubleLogEntry(log, "Vision/Distance");
        tagNumberLogger = new IntegerLogEntry(log, "Vision/TagNumber");

        // Robot pose log entries for visualization (AdvantageScope)
        robotPoseXLogger = new DoubleLogEntry(log, "Vision/RobotPose/X");
        robotPoseYLogger = new DoubleLogEntry(log, "Vision/RobotPose/Y");
        robotPoseRotLogger = new DoubleLogEntry(log, "Vision/RobotPose/RotDeg");

        // Best-target (camera->target) transform entries
        bestTagXLogger = new DoubleLogEntry(log, "Vision/BestTarget/X");
        bestTagYLogger = new DoubleLogEntry(log, "Vision/BestTarget/Y");
        bestTagZLogger = new DoubleLogEntry(log, "Vision/BestTarget/Z");
        bestTagYawLogger = new DoubleLogEntry(log, "Vision/BestTarget/YawDeg");
        bestTagPitchLogger = new DoubleLogEntry(log, "Vision/BestTarget/PitchDeg");

        // ── AdvantageScope 3D struct publishers ──
        var nt = NetworkTableInstance.getDefault();
        visionRobotPose3dPub = nt.getStructTopic("SmartDashboard/Vision/RobotPose3d", Pose3d.struct).publish();
        cameraPose3dPub = nt.getStructTopic("SmartDashboard/Vision/CameraPose3d", Pose3d.struct).publish();
        detectedTagPosesPub = nt.getStructArrayTopic("SmartDashboard/Vision/DetectedTagPoses", Pose3d.struct).publish();

    }

    /**
     * Get the latest result from the camera.
     * 
     */
    public void get() {
        try {
            var results = camera.getAllUnreadResults();
            if (!results.isEmpty()) {
                // take the last (most recent) unread result
                latestResult = results.get(results.size() - 1);
            }
            // if there are no unread results, latestResult stays as-is
        } catch (Exception e) {
            ElasticDashboard.putString("Vision/GetError", e.toString());
        }
    }

    /**
     * Estimate Pose2d of the robot.
     * 
     * @return estimated Pose2d
     */
    public Optional<Pose2d> getEstimatedPose2d() {
        var estimatedPose = getEstimatedPose();

        if (estimatedPose.isPresent()) {
            Pose2d pose = new Pose2d(
                    estimatedPose.get().estimatedPose.getX(),
                    estimatedPose.get().estimatedPose.getY(),
                    estimatedPose.get().estimatedPose.getRotation().toRotation2d());
            return Optional.of(pose);
        } else {
            return Optional.empty();
        }
    }

    /**
     * Estimate the pose of the robot.
     * 
     */
    private Optional<EstimatedRobotPose> lastEstimatedRobotBose = Optional.empty();

    public Optional<EstimatedRobotPose> getEstimatedPose() {
        if (null == latestResult || !latestResult.hasTargets()) {
            return lastEstimatedRobotBose;
        }

        try {
            var robot_pose = estimator.update(latestResult);
            if (robot_pose.isPresent()) {
                lastEstimatedRobotBose = Optional.of(robot_pose.get());
                return Optional.of(robot_pose.get());
            }
        } catch (Exception e) {
        }

        return lastEstimatedRobotBose;
    }

    /**
     * 
     * @return Transform3d of a specific tag
     */
    public Optional<Transform3d> getTargetTransform() {
        if (latestResult == null || !latestResult.hasTargets()) {
            return Optional.empty();
        }

        return Optional.of(latestResult.getBestTarget().getBestCameraToTarget());
    }

    /**
     * Get the fiducial ID of the best visible target.
     * 
     * @return Optional containing the tag ID, or empty if no target visible
     */
    public Optional<Integer> getBestTargetId() {
        if (latestResult == null || !latestResult.hasTargets()) {
            return Optional.empty();
        }
        return Optional.of(latestResult.getBestTarget().getFiducialId());
    }

    /**
     * Returns the yaw angle (degrees) of the best visible target, if present.
     * Positive = left of optical axis.
     */
    public java.util.OptionalDouble getBestTargetYawDeg() {
        if (latestResult == null || !latestResult.hasTargets()) {
            return java.util.OptionalDouble.empty();
        }
        try {
            double yaw = latestResult.getBestTarget().getYaw();
            return java.util.OptionalDouble.of(yaw);
        } catch (Exception e) {
            ElasticDashboard.putString("Vision/BestYawError", e.toString());
            return java.util.OptionalDouble.empty();
        }
    }

    /**
     * Get the transform to a specific AprilTag by ID.
     * 
     * @param tagId The fiducial ID of the tag to find
     * @return Optional containing the Transform3d to that tag, or empty if not
     *         visible
     */
    public Optional<Transform3d> getTransformToTag(int tagId) {
        if (latestResult == null || !latestResult.hasTargets()) {
            return Optional.empty();
        }
        for (var target : latestResult.targets) {
            if (target.getFiducialId() == tagId) {
                return Optional.of(target.getBestCameraToTarget());
            }
        }
        return Optional.empty();
    }

    /**
     * Calculate the target pose for the robot to be positioned in front of a tag.
     * 
     * @param tagPose  The field pose of the AprilTag
     * @param distance How far in front of the tag to position (meters)
     * @return The target Pose2d for the robot
     */
    public Pose2d calculateTargetPose(Pose2d tagPose, double distance) {
        // AprilTag rotation points OUTWARD from the surface it's mounted on.
        // To place the robot IN FRONT of the tag (facing it), we subtract the
        // offset so the target is on the field side of the tag, not behind it.
        Rotation2d tagRotation = tagPose.getRotation();
        double offsetX = tagPose.getX() - distance * tagRotation.getCos();
        double offsetY = tagPose.getY() - distance * tagRotation.getSin();
        // Robot should face toward the tag (opposite of the tag's outward direction)
        Rotation2d robotRotation = tagRotation.plus(Rotation2d.fromDegrees(180));
        return new Pose2d(offsetX, offsetY, robotRotation);
    }

    /**
     * Returns the Pose2d the robot should drive to in order to be a given distance
     * directly in front of AprilTag 10, facing the tag.
     *
     * Uses the field layout to look up tag 10's 3D pose, then offsets along the
     * tag's facing direction so the robot ends up in front of it.
     *
     * @param distanceMeters how far in front of the tag (meters)
     * @return Optional containing the target Pose2d, or empty if tag 10 is not in
     *         the layout
     */
    public Optional<Pose2d> getTag10TargetPose(double distanceMeters) {
        Optional<Pose3d> tag10Pose3d = aprilTagFieldLayout.getTagPose(10);
        if (tag10Pose3d.isEmpty()) {
            return Optional.empty();
        }
        Pose2d tag10Pose = tag10Pose3d.get().toPose2d();
        return Optional.of(calculateTargetPose(tag10Pose, distanceMeters));
    }

    /**
     * Returns the Pose2d the robot should drive to in order to be a given distance
     * directly in front of the specified tag, facing the tag.
     */
    public Optional<Pose2d> getTagTargetPose(int tagId, double distanceMeters) {
        Optional<Pose3d> tagPose3d = aprilTagFieldLayout.getTagPose(tagId);
        if (tagPose3d.isEmpty()) {
            return Optional.empty();
        }
        Pose2d tagPose = tagPose3d.get().toPose2d();
        return Optional.of(calculateTargetPose(tagPose, distanceMeters));
    }

    /**
     * Return the first visible transform to any of the given tag IDs. If multiple
     * are visible, chooses the closest in camera frame.
     */
    public Optional<Transform3d> getTransformToAny(int... tagIds) {
        if (latestResult == null || !latestResult.hasTargets()) {
            return Optional.empty();
        }
        Transform3d best = null;
        double bestRange = Double.POSITIVE_INFINITY;
        for (var target : latestResult.targets) {
            int id = target.getFiducialId();
            for (int wanted : tagIds) {
                if (id == wanted) {
                    Transform3d tf = target.getBestCameraToTarget();
                    double range = Math.hypot(tf.getX(), tf.getY());
                    if (range < bestRange) {
                        bestRange = range;
                        best = tf;
                    }
                    break;
                }
            }
        }
        return Optional.ofNullable(best);
    }

    /**
     * track IDs of detected April Tags
     */
    public int[] targetIds;

    /**
     * Periodically update SmartDashboard and log data for AdvantageScope 3D.
     * 
     * NOTE: The pose estimator is updated via getEstimatedPose() which is called
     * from RobotContainer.robotPeriodic(). We use the cached result here so we
     * don't call estimator.update() twice per cycle.
     */
    @Override
    public void periodic() {
        // NOTE: vision.get() is called from RobotContainer.robotPeriodic() already.
        // Do NOT call get() here — double-consuming camera frames causes jitter.

        if (null == latestResult) {
            return;
        }

        boolean hasTargets = latestResult.hasTargets();

        // Track detected tag IDs
        targetIds = new int[latestResult.targets.size()];
        for (int i = 0; i < latestResult.targets.size(); i++) {
            targetIds[i] = latestResult.targets.get(i).getFiducialId();
        }

        // Use the cached estimated pose from getEstimatedPose() which was already
        // called in RobotContainer.robotPeriodic(). Do NOT call estimator.update()
        // again — a second update produces a slightly different pose and causes
        // the 3D model to flicker between two estimates.
        Optional<EstimatedRobotPose> estimated = lastEstimatedRobotBose;

        if (estimated.isPresent()) {
            Pose3d meterPose = estimated.get().estimatedPose;
            field.setRobotPose(meterPose.toPose2d());

            // Vision-estimated robot pose (shows as "ghost" in AdvantageScope)
            visionRobotPose3dPub.set(meterPose);

            // Camera pose in field coordinates
            Pose3d cameraPoseField = meterPose.transformBy(Constants.Vision.robotToCamera);
            cameraPose3dPub.set(cameraPoseField);
        }

        // Detected tag poses (AdvantageScope renders these as highlighted tags)
        if (hasTargets) {
            Pose3d[] tagPoses = latestResult.targets.stream()
                    .map(t -> aprilTagFieldLayout.getTagPose(t.getFiducialId()))
                    .filter(Optional::isPresent)
                    .map(Optional::get)
                    .toArray(Pose3d[]::new);
            detectedTagPosesPub.set(tagPoses);
        } else {
            detectedTagPosesPub.set(new Pose3d[0]);
        }

        // Data log entries (every cycle — cheap, goes to .wpilog file)
        if (hasTargets) {
            try {
                var bestTarget = latestResult.getBestTarget();
                tagNumberLogger.append(bestTarget.getFiducialId());
                distancelog.append(bestTarget.getBestCameraToTarget().getX());

                if (estimated.isPresent()) {
                    Pose3d estPose = estimated.get().estimatedPose;
                    robotPoseXLogger.append(estPose.getX());
                    robotPoseYLogger.append(estPose.getY());
                    robotPoseRotLogger.append(Math.toDegrees(estPose.getRotation().getZ()));
                }

                Transform3d camToTarget = bestTarget.getBestCameraToTarget();
                bestTagXLogger.append(camToTarget.getTranslation().getX());
                bestTagYLogger.append(camToTarget.getTranslation().getY());
                bestTagZLogger.append(camToTarget.getTranslation().getZ());
                bestTagYawLogger.append(bestTarget.getYaw());
                bestTagPitchLogger.append(bestTarget.getPitch());
            } catch (Exception e) {
                // Logged below with SmartDashboard if throttle cycle allows
            }
        }

        // ── Throttle SmartDashboard debug text to every 5th cycle (~100ms) ──
        // Keeps NT bandwidth free for the struct publishers above.
        periodicCounter++;
        if (periodicCounter >= 5) {
            periodicCounter = 0;

            ElasticDashboard.putData("Vision - Field", field);
            ElasticDashboard.putBoolean("Vision - Has Targets", hasTargets);
            ElasticDashboard.putString("Vision - Tracked Targets", Arrays.toString(targetIds));
            ElasticDashboard.putBoolean("vision-estimated:", estimated.isPresent());

            if (estimated.isPresent()) {
                ElasticDashboard.putString("Rotation - vision:",
                        estimated.get().estimatedPose.getRotation().toString());

                Pose3d estPose = estimated.get().estimatedPose;
                ElasticDashboard.putNumber("Est/Robot/X", estPose.getX());
                ElasticDashboard.putNumber("Est/Robot/Y", estPose.getY());
                ElasticDashboard.putNumber("Est/Robot/Z", estPose.getZ());
                ElasticDashboard.putNumber("Est/Robot/RotDeg",
                        Math.toDegrees(estPose.getRotation().getZ()));
            }

            if (hasTargets) {
                try {
                    var bestTarget = latestResult.getBestTarget();
                    Transform3d camToTarget = bestTarget.getBestCameraToTarget();
                    ElasticDashboard.putNumber("3D/X_Meters", camToTarget.getX());
                    ElasticDashboard.putNumber("3D/Y_Meters", camToTarget.getY());
                    ElasticDashboard.putNumber("3D/Z_Meters", camToTarget.getZ());
                    ElasticDashboard.putNumber("3D/Yaw_Degrees",
                            Math.toDegrees(camToTarget.getRotation().getZ()));
                    ElasticDashboard.putNumber("3D/Ambiguity", bestTarget.getPoseAmbiguity());
                } catch (Exception e) {
                    ElasticDashboard.putString("Vision/LoggingError", e.toString());
                }
            }
        }
    }

    public Optional<Pose3d> getTagPose(int tagId) {
        return aprilTagFieldLayout.getTagPose(tagId);
    }

    public AprilTagFieldLayout getFieldLayout() {
        return aprilTagFieldLayout;
    }
}