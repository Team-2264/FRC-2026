package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
// removed unused imports
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Transform3d;
// removed unused import
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.IntegerLogEntry;
// DoubleLogEntry already imported above

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
    private double lastLogTime = 0.0;
    private IntegerLogEntry tagNumberLogger;
    private DoubleLogEntry robotPoseXLogger;
    private DoubleLogEntry robotPoseYLogger;
    private DoubleLogEntry robotPoseRotLogger;
    private DoubleLogEntry bestTagXLogger;
    private DoubleLogEntry bestTagYLogger;
    private DoubleLogEntry bestTagZLogger;
    private DoubleLogEntry bestTagYawLogger;
    private DoubleLogEntry bestTagPitchLogger;

    /**
     * Constructs a new Vision instance.
     * 
     */
    public Vision2025() {
        camera = new PhotonCamera(Constants.Vision.cameraName);
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        // Construct the estimator with a pose strategy and the camera transform
        estimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                Constants.Vision.robotToCamera);

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

        // Best-target (camera->target) transform entries
        bestTagXLogger = new DoubleLogEntry(log, "Vision/BestTarget/X");
        bestTagYLogger = new DoubleLogEntry(log, "Vision/BestTarget/Y");
        bestTagZLogger = new DoubleLogEntry(log, "Vision/BestTarget/Z");
        bestTagYawLogger = new DoubleLogEntry(log, "Vision/BestTarget/YawDeg");
        bestTagPitchLogger = new DoubleLogEntry(log, "Vision/BestTarget/PitchDeg");

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
            SmartDashboard.putString("Vision/GetError", e.toString());
        }
    }

    /**
     * This returns the Pose2d of the setpoint the robot is desired to drive to
     * based on the closest april tag detected.
     * Used for auto aligning to the right reef.
     *
     * See https://www.desmos.com/calculator/sib8enmabx
     * 
     * @param tagID of the april tag detected
     * @return Pose2d the robot is desired to drive to
     */
    public Pose2d rightReefPose(int tagID) {
        var _tagPose = tagPose(tagID);

        // Guard against empty Optional
        if (_tagPose.isEmpty()) {
            SmartDashboard.putString("Vision/Error", "rightReefPose: tagID " + tagID + " not found");
            return null;
        }

        switch (tagID) {
            // Red Alliance
            case 1:
                // return new Pose2d(Units.inchesToMeters(657.37), Units.inchesToMeters(25.8),
                // Rotation2d.fromDegrees(126));
                return new Pose2d(_tagPose.get().getX() - 1.0, _tagPose.get().getY() - 1.0,
                        new Rotation2d(_tagPose.get().getRotation().getRadians()));
            case 2:
                return new Pose2d(Units.inchesToMeters(657.37), Units.inchesToMeters(291.20),
                        Rotation2d.fromDegrees(234));
            case 6:
                return new Pose2d(Units.inchesToMeters(536.49), Units.inchesToMeters(119.7777),
                        Rotation2d.fromDegrees(240));
            case 7:
                return new Pose2d(Units.inchesToMeters(558.87), Units.inchesToMeters(158.5),
                        Rotation2d.fromDegrees(180));
            case 8:
                return new Pose2d(Units.inchesToMeters(536.49), Units.inchesToMeters(197.2223),
                        Rotation2d.fromDegrees(180));
            case 9:
                return new Pose2d(Units.inchesToMeters(491.77), Units.inchesToMeters(197.2223),
                        Rotation2d.fromDegrees(60));
            case 10:
                return new Pose2d(Units.inchesToMeters(469.39), Units.inchesToMeters(158.5), Rotation2d.fromDegrees(0));
            case 11:
                return new Pose2d(Units.inchesToMeters(491.7777), Units.inchesToMeters(119.777),
                        Rotation2d.fromDegrees(300));

            // Blue Alliance
            // case 12:
            // return new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0),
            // Rotation2d.fromDegrees(54));
            case 13:
                return new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), Rotation2d.fromDegrees(306));
            case 17:
                return new Pose2d(Units.inchesToMeters(154.39), Units.inchesToMeters(119.7777),
                        Rotation2d.fromDegrees(240));
            case 18:
                return new Pose2d(Units.inchesToMeters(144), Units.inchesToMeters(158.5), Rotation2d.fromDegrees(180));
            case 19:
                return new Pose2d(Units.inchesToMeters(144), Units.inchesToMeters(158.5), Rotation2d.fromDegrees(180));
            case 20:
                return new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83),
                        Rotation2d.fromDegrees(60));
            case 21:
                return new Pose2d(Units.inchesToMeters(221.49), Units.inchesToMeters(158.5), Rotation2d.fromDegrees(0));
            case 22:
                return new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17),
                        Rotation2d.fromDegrees(300));

            case 12:
                return new Pose2d(Units.inchesToMeters(50), Units.inchesToMeters(50), Rotation2d.fromDegrees(54));
            default:
                return null;
        }
    }

    /**
     * Returns the Pose2d of a tag based on tagID
     * 
     * @param tagID
     * @return Pose2d of a specific april tag
     */
    public Optional<Pose2d> tagPose(int tagID) {
        try {
            Pose2d tagPose = switch (tagID) {
                case 1 -> new Pose2d(
                        Units.inchesToMeters(Constants.TagLocation.tag1.get().getX()),
                        Units.inchesToMeters(Constants.TagLocation.tag1.get().getY()),
                        Constants.TagLocation.tag1.get().getRotation().toRotation2d());
                case 2 -> new Pose2d(
                        Units.inchesToMeters(Constants.TagLocation.tag2.get().getX()),
                        Units.inchesToMeters(Constants.TagLocation.tag2.get().getY()),
                        Constants.TagLocation.tag2.get().getRotation().toRotation2d());
                case 3 -> new Pose2d(
                        Units.inchesToMeters(Constants.TagLocation.tag3.get().getX()),
                        Units.inchesToMeters(Constants.TagLocation.tag3.get().getY()),
                        Constants.TagLocation.tag3.get().getRotation().toRotation2d());
                case 4 -> new Pose2d(
                        Units.inchesToMeters(Constants.TagLocation.tag4.get().getX()),
                        Units.inchesToMeters(Constants.TagLocation.tag4.get().getY()),
                        Constants.TagLocation.tag4.get().getRotation().toRotation2d());
                case 5 -> new Pose2d(
                        Units.inchesToMeters(Constants.TagLocation.tag5.get().getX()),
                        Units.inchesToMeters(Constants.TagLocation.tag5.get().getY()),
                        Constants.TagLocation.tag5.get().getRotation().toRotation2d());
                case 6 -> new Pose2d(
                        Units.inchesToMeters(Constants.ReefTagLocation.tag6.get().getX()),
                        Units.inchesToMeters(Constants.ReefTagLocation.tag6.get().getY()),
                        Constants.ReefTagLocation.tag6.get().getRotation().toRotation2d());
                case 7 -> new Pose2d(
                        Units.inchesToMeters(Constants.ReefTagLocation.tag7.get().getX()),
                        Units.inchesToMeters(Constants.ReefTagLocation.tag7.get().getY()),
                        Constants.ReefTagLocation.tag7.get().getRotation().toRotation2d());
                case 8 -> new Pose2d(
                        Units.inchesToMeters(Constants.ReefTagLocation.tag8.get().getX()),
                        Units.inchesToMeters(Constants.ReefTagLocation.tag8.get().getY()),
                        Constants.ReefTagLocation.tag8.get().getRotation().toRotation2d());
                case 9 -> new Pose2d(
                        Units.inchesToMeters(Constants.ReefTagLocation.tag9.get().getX()),
                        Units.inchesToMeters(Constants.ReefTagLocation.tag9.get().getY()),
                        Constants.ReefTagLocation.tag9.get().getRotation().toRotation2d());
                case 10 -> new Pose2d(
                        Units.inchesToMeters(Constants.ReefTagLocation.tag10.get().getX()),
                        Units.inchesToMeters(Constants.ReefTagLocation.tag10.get().getY()),
                        Constants.ReefTagLocation.tag10.get().getRotation().toRotation2d());
                case 11 -> new Pose2d(
                        Units.inchesToMeters(Constants.ReefTagLocation.tag11.get().getX()),
                        Units.inchesToMeters(Constants.ReefTagLocation.tag11.get().getY()),
                        Constants.ReefTagLocation.tag11.get().getRotation().toRotation2d());
                case 12 -> new Pose2d(
                        Units.inchesToMeters(Constants.TagLocation.tag12.get().getX()),
                        Units.inchesToMeters(Constants.TagLocation.tag12.get().getY()),
                        Constants.TagLocation.tag12.get().getRotation().toRotation2d());
                case 13 -> new Pose2d(
                        Units.inchesToMeters(Constants.TagLocation.tag13.get().getX()),
                        Units.inchesToMeters(Constants.TagLocation.tag13.get().getY()),
                        Constants.TagLocation.tag13.get().getRotation().toRotation2d());
                case 14 -> new Pose2d(
                        Units.inchesToMeters(Constants.TagLocation.tag14.get().getX()),
                        Units.inchesToMeters(Constants.TagLocation.tag14.get().getY()),
                        Constants.TagLocation.tag14.get().getRotation().toRotation2d());
                case 15 -> new Pose2d(
                        Units.inchesToMeters(Constants.TagLocation.tag15.get().getX()),
                        Units.inchesToMeters(Constants.TagLocation.tag15.get().getY()),
                        Constants.TagLocation.tag15.get().getRotation().toRotation2d());
                case 16 -> new Pose2d(
                        Units.inchesToMeters(Constants.TagLocation.tag16.get().getX()),
                        Units.inchesToMeters(Constants.TagLocation.tag16.get().getY()),
                        Constants.TagLocation.tag16.get().getRotation().toRotation2d());

                case 17 -> new Pose2d(
                        Units.inchesToMeters(Constants.ReefTagLocation.tag17.get().getX()),
                        Units.inchesToMeters(Constants.ReefTagLocation.tag17.get().getY()),
                        Constants.ReefTagLocation.tag17.get().getRotation().toRotation2d());
                case 18 -> new Pose2d(
                        Units.inchesToMeters(Constants.ReefTagLocation.tag18.get().getX()),
                        Units.inchesToMeters(Constants.ReefTagLocation.tag18.get().getY()),
                        Constants.ReefTagLocation.tag18.get().getRotation().toRotation2d());
                case 19 -> new Pose2d(
                        Units.inchesToMeters(Constants.ReefTagLocation.tag19.get().getX()),
                        Units.inchesToMeters(Constants.ReefTagLocation.tag19.get().getY()),
                        Constants.ReefTagLocation.tag19.get().getRotation().toRotation2d());
                case 20 -> new Pose2d(
                        Units.inchesToMeters(Constants.ReefTagLocation.tag20.get().getX()),
                        Units.inchesToMeters(Constants.ReefTagLocation.tag20.get().getY()),
                        Constants.ReefTagLocation.tag20.get().getRotation().toRotation2d());
                case 21 -> new Pose2d(
                        Units.inchesToMeters(Constants.ReefTagLocation.tag21.get().getX()),
                        Units.inchesToMeters(Constants.ReefTagLocation.tag21.get().getY()),
                        Constants.ReefTagLocation.tag21.get().getRotation().toRotation2d());
                case 22 -> new Pose2d(
                        Units.inchesToMeters(Constants.ReefTagLocation.tag22.get().getX()),
                        Units.inchesToMeters(Constants.ReefTagLocation.tag22.get().getY()),
                        Constants.ReefTagLocation.tag22.get().getRotation().toRotation2d());

                default -> null;
            };

            if (tagPose != null) {
                return Optional.of(tagPose);
            } else {
                return Optional.empty();
            }
        } catch (Exception e) {
            SmartDashboard.putString("Vision/TagPoseError", "tagID " + tagID + ": " + e.toString());
            return Optional.empty();
        }

    }

    // /**
    // *
    // * @return array index of the closest tag
    // */
    // public int getClosestTarget(){
    // var estimatedPose = getEstimatedPose2d();
    // if (estimatedPose.isEmpty()){
    // return -1;
    // }

    // if (targetIds.length == 0){
    // return -1;
    // }

    // int closest = 0;
    // double closestDistance = PhotonUtils.getDistanceToPose(estimatedPose.get(),
    // tagPose(targetIds[0]).get());

    // for (int i=1; i<targetIds.length; i++){
    // double compare = PhotonUtils.getDistanceToPose(estimatedPose.get(),
    // tagPose(targetIds[i]).get());
    // if (compare<closestDistance){
    // closestDistance = compare;
    // closest = i;
    // }
    // }

    // return closest;
    // }

    // public int getClosestID() {
    // if (null == targetIds) {
    // return -1;
    // }

    // int closestTartet = getClosestTarget();
    // if ((targetIds.length > 0) && (closestTartet >= 0)) {
    // return targetIds[closestTartet];
    // }
    // else {
    // return -1;
    // }
    // }

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

    // /**
    // * Gets the Pose2d of the closest AprilTag, if it exists.
    // * @return Pose2d of closest april tag
    // */
    // public Pose2d getPoseOfClosestReef() {
    // int closest = getClosestID();

    // if (closest != -1){
    // switch (closest) {
    // case 17:
    // return new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17),
    // Rotation2d.fromDegrees(240));
    // case 18:
    // return new Pose2d(Units.inchesToMeters(144), Units.inchesToMeters(158.5),
    // Rotation2d.fromDegrees(180));
    // case 19:
    // return new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83),
    // Rotation2d.fromDegrees(120));
    // case 20:
    // return new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83),
    // Rotation2d.fromDegrees(60));
    // case 21:
    // return new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.5),
    // Rotation2d.fromDegrees(0));
    // case 22:
    // return new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17),
    // Rotation2d.fromDegrees(300));
    // default:
    // return null;
    // }
    // }
    // else {
    // return null;
    // }
    // }

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
        // SmartDashboard.putNumber("Vision - Closest Target",
        // targetIds[getClosestTarget()]);

        if (targetIds.length != 0 && latestResult.hasTargets()) {
            try {
                Transform3d targetTrans = latestResult.targets.get(0).getBestCameraToTarget();
                SmartDashboard.putString("PhotonTarget", targetTrans.toString());
            } catch (Exception e) {
                SmartDashboard.putString("Vision/TargetAccessError", e.toString());
            }
        }

        // Estimated pose (non-blocking) — use estimator.update which does local
        // estimation
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

        // Throttled logging at 5 Hz (once every 0.2 seconds)
        double now = Timer.getFPGATimestamp();
        if (now - lastLogTime >= 0.2) {
            // Log only if we have targets (all guarded by null/hasTargets checks)
            if ((latestResult != null) && latestResult.hasTargets()) {
                try {
                    var target = latestResult.getBestTarget();
                    int tagNumber = target.getFiducialId();
                    double dist = target.getBestCameraToTarget().getX();

                    tagNumberLogger.append(tagNumber);
                    distancelog.append(dist);

                    // Log robot pose (if available) so AdvantageScope can show robot on field
                    if (estimated.isPresent()) {
                        Pose3d meterPose = estimated.get().estimatedPose;
                        robotPoseXLogger.append(meterPose.getTranslation().getX());
                        robotPoseYLogger.append(meterPose.getTranslation().getY());
                        // log rotation in degrees (yaw)
                        robotPoseRotLogger.append(Math.toDegrees(meterPose.getRotation().getZ()));
                    }

                    // Log best-target transform (camera -> target)
                    Transform3d camToTarget = target.getBestCameraToTarget();
                    bestTagXLogger.append(camToTarget.getTranslation().getX());
                    bestTagYLogger.append(camToTarget.getTranslation().getY());
                    bestTagZLogger.append(camToTarget.getTranslation().getZ());
                    // get yaw/pitch from tracked target
                    bestTagYawLogger.append(target.getYaw());
                    bestTagPitchLogger.append(target.getPitch());

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

                } catch (Exception e) {
                    SmartDashboard.putString("Vision/LoggingError", e.toString());
                }
            }

            lastLogTime = now;
        }

    }

}