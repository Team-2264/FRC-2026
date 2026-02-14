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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    // Struct-based logging for proper 3D visualization in AdvantageScope
    private StructLogEntry<Pose3d> robotPose3dLogger;
    private StructLogEntry<Transform3d> targetTransformLogger;
    
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

        // Robot pose log entries for visualization (AdvantageScope / AdvantageKit)
        robotPoseXLogger = new DoubleLogEntry(log, "Vision/RobotPose/X");
        robotPoseYLogger = new DoubleLogEntry(log, "Vision/RobotPose/Y");
        robotPoseRotLogger = new DoubleLogEntry(log, "Vision/RobotPose/RotDeg");

        // Struct loggers for proper 3D visualization
        robotPose3dLogger = StructLogEntry.create(log, "Vision/RobotPose3d", Pose3d.struct);
        targetTransformLogger = StructLogEntry.create(log, "Vision/TargetTransform3d", Transform3d.struct);

        // Best-target (camera->target) transform entries
        bestTagXLogger = new DoubleLogEntry(log, "Vision/BestTarget/X");
        bestTagYLogger = new DoubleLogEntry(log, "Vision/BestTarget/Y");
        bestTagZLogger = new DoubleLogEntry(log, "Vision/BestTarget/Z");
        bestTagYawLogger = new DoubleLogEntry(log, "Vision/BestTarget/YawDeg");
        bestTagPitchLogger = new DoubleLogEntry(log, "Vision/BestTarget/PitchDeg");
    }

    /**
     *  Get the latest result from the camera.
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
            SmartDashboard.putString("Vision/GetError", e.toString());
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
            SmartDashboard.putString("Vision/GetEstimatedPoseError", e.toString());
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
     * track IDs of detected April Tags
     */
    public int[] targetIds;

    /**
     * Periodicly update SmartDashboard and track april tags.
     */
    @Override
    public void periodic() {
        get(); // Query the camera for the latest result

        // Has targets
        if (null == latestResult) {
            return;
        }

        SmartDashboard.putBoolean("Vision - Has Targets", latestResult.hasTargets());

        // Tracked targets ids
        targetIds = new int[latestResult.targets.size()];
        for (int i = 0; i < latestResult.targets.size(); i++) {
            targetIds[i] = latestResult.targets.get(i).getFiducialId();
        }
        SmartDashboard.putString("Vision - Tracked Targets", Arrays.toString(targetIds));
       // SmartDashboard.putNumber("Vision - Closest Target", targetIds[getClosestTarget()]);

        if (targetIds.length != 0 && latestResult.hasTargets()) {
            try {
                Transform3d targetTrans = latestResult.targets.get(0).getBestCameraToTarget();
                SmartDashboard.putString("PhotonTarget", targetTrans.toString());
            } catch (Exception e) {
                SmartDashboard.putString("Vision/TargetAccessError", e.toString());
            }
        }


        // Estimated pose (non-blocking) — use estimator.update which does local estimation
        Optional<EstimatedRobotPose> estimated = Optional.empty();
        try {
            if (latestResult != null && latestResult.hasTargets()) {
                estimated = estimator.update(latestResult);
            }
        } catch (Exception e) {
            SmartDashboard.putString("Vision/EstimatorError", e.toString());
        }

        SmartDashboard.putBoolean("vision-estimated:", estimated.isPresent());

        if (estimated.isPresent()) {
            Pose3d meterPose = estimated.get().estimatedPose;
            // Update Field2d with a Pose2d (use Pose2d constructor)
            field.setRobotPose(new Pose2d(meterPose.getX(), meterPose.getY(), meterPose.getRotation().toRotation2d()));

            Pose3d inchPose = new Pose3d(
                    meterPose.getTranslation().getX() * 39.3701,
                    meterPose.getTranslation().getY() * 39.3701,
                    meterPose.getTranslation().getZ() * 39.3701,
                    meterPose.getRotation());

            SmartDashboard.putString("Vision - Estimated Pose (in)", inchPose.toString());
            SmartDashboard.putString("Rotation - vision: ", meterPose.getRotation().toString());
        }
        
        // Field
        SmartDashboard.putData("Vision - Field", field);

        // Log every periodic cycle for maximum smoothness
        if ((latestResult != null) && latestResult.hasTargets()) {
            try {
                var target = latestResult.getBestTarget();
                int tagNumber = target.getFiducialId();
                double dist = target.getBestCameraToTarget().getX();

                tagNumberLogger.append(tagNumber);
                distancelog.append(dist);

                // Log robot pose (if available) so AdvantageScope can show robot on field
                // PhotonPoseEstimator returns pose in FIELD frame (meters):
                // - X: forward (0 = red wall, 16.54 = blue wall)
                // - Y: left (0 = red wall, 8.23 = blue wall)
                // - Z: height (0 = ground, bumper ~0.35m)
                if (estimated.isPresent()) {
                    Pose3d meterPose = estimated.get().estimatedPose;
                    robotPoseXLogger.append(meterPose.getTranslation().getX());
                    robotPoseYLogger.append(meterPose.getTranslation().getY());
                    // log rotation in degrees (yaw)
                    double yaw = meterPose.getRotation().getZ(); // radians
                    robotPoseRotLogger.append(Math.toDegrees(yaw));

                    // Log full Pose3d struct with only yaw (heading) for proper 3D visualization
                    Rotation3d robotYawRotation = new Rotation3d(0, 0, yaw);
                    Pose3d robotPose3d = new Pose3d(
                            meterPose.getTranslation().getX(),
                            meterPose.getTranslation().getY(),
                            0.35, // bumper height in meters
                            robotYawRotation);
                    robotPose3dLogger.append(robotPose3d);
                }

                // IMPORTANT: Log camera->target transform (distance/bearing to tag)
                // Camera frame coordinates (from PhotonVision):
                // - X: forward distance (away from camera, along optical axis)
                // - Y: left offset (horizontal, left of optical axis)
                // - Z: up offset (vertical, above optical axis)
                // AdvantageScope can use this to show tag position relative to camera
                Transform3d camToTarget = target.getBestCameraToTarget();
                bestTagXLogger.append(camToTarget.getTranslation().getX());
                bestTagYLogger.append(camToTarget.getTranslation().getY());
                bestTagZLogger.append(camToTarget.getTranslation().getZ());
                // get yaw/pitch from tracked target
                bestTagYawLogger.append(target.getYaw());
                bestTagPitchLogger.append(target.getPitch());

                // Log full Transform3d struct for proper camera->target 3D visualization
                targetTransformLogger.append(camToTarget);

                // SmartDashboard 3D values for debugging
                Transform3d cameraToTarget = target.getBestCameraToTarget();
                Translation3d translation = cameraToTarget.getTranslation();
                double x = translation.getX();
                double y = translation.getY();
                double z = translation.getZ();

                Rotation3d rotation = cameraToTarget.getRotation();
                double yaw = Math.toDegrees(rotation.getZ());

                SmartDashboard.putNumber("3D/X_Meters", x);
                SmartDashboard.putNumber("3D/Y_Meters", y);
                SmartDashboard.putNumber("3D/Z_Meters", z);
                SmartDashboard.putNumber("3D/Yaw_Degrees", yaw);
                SmartDashboard.putNumber("3D/Ambiguity", target.getPoseAmbiguity());

                // Debug: log estimated robot position for AdvantageScope
                if (estimated.isPresent()) {
                    Pose3d est = estimated.get().estimatedPose;
                    SmartDashboard.putNumber("Est/Robot/X", est.getX());
                    SmartDashboard.putNumber("Est/Robot/Y", est.getY());
                    SmartDashboard.putNumber("Est/Robot/Z", est.getZ());
                    SmartDashboard.putNumber("Est/Robot/RotDeg", Math.toDegrees(est.getRotation().getZ()));
                }

            } catch (Exception e) {
                SmartDashboard.putString("Vision/LoggingError", e.toString());
            }
        }
    }
}