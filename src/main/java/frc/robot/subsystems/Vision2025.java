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
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;

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
    private double lastLogTime = 0.0;
    private IntegerLogEntry tagNumberLogger;
    private DoubleLogEntry distancelog;

    /**
     * Constructs a new Vision instance.
     * 
     */
    public Vision2025() {
        camera = new PhotonCamera(Constants.Vision.cameraName);
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        estimator = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                Constants.Vision.robotToCamera);

        estimator.setRobotToCameraTransform(Constants.Vision.robotToCamera);

        field = new Field2d();

        DataLogManager.start();
        DataLog log = DataLogManager.getLog();
        tagNumberLogger = new IntegerLogEntry(log, "/Vision/TagNumber");
        distancelog = new DoubleLogEntry(DataLogManager.getLog(), "/Vision/TagDistance");

    }

    /**
     * Get the latest result from the camera.
     * 
     */
    public void get() {
        // var results = camera.getAllUnreadResults();
        // if (!results.isEmpty()) {
        // // Camera processed a new frame since last
        // // Get the last one in the list.
        // latestResult = results.get(results.size() - 1);
        // }
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // take the last (most recent) unread result
            latestResult = results.get(results.size() - 1);
        }
        // if there are no unread results, latestResult stays as-is
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
        if (null == latestResult) {
            return lastEstimatedRobotBose;
        }

        var robot_pose = estimator.estimateCoprocMultiTagPose(latestResult);
        if (robot_pose.isPresent()) {
            lastEstimatedRobotBose = Optional.of(robot_pose.get());
            return Optional.of(robot_pose.get());
        }

        return lastEstimatedRobotBose;
    }

    /**
     * 
     * @return Transform3d of a specific tag
     */
    public Optional<Transform3d> getTargetTransform() {
        if (latestResult == null || !latestResult.hasTargets() || latestResult.targets.isEmpty()) {
            return Optional.empty();
        }

        return Optional.of(latestResult.getBestTarget().getBestCameraToTarget());
    }

    /**
     * Heading error (degrees) from the robot to AprilTag 10 (hub reference).
     * Positive -> tag10 is to the left; Negative -> tag10 is to the right.
     * Use this to drive a turn controller: turn left if > 0, right if < 0, stop
     * when |error| < threshold.
     */
    public Optional<Double> headingErrorToTag10Deg() {
        Optional<EstimatedRobotPose> est = getEstimatedPose();
        if (est.isEmpty()) {
            return Optional.empty();
        }

        Optional<Pose3d> tag10PoseOpt = aprilTagFieldLayout.getTagPose(10);
        if (tag10PoseOpt.isEmpty()) {
            return Optional.empty();
        }

        Pose2d robotPose = est.get().estimatedPose.toPose2d();
        Pose2d tag10Pose2d = tag10PoseOpt.get().toPose2d();

        Translation2d toTag = tag10Pose2d.getTranslation().minus(robotPose.getTranslation());
        double targetHeading = Math.atan2(toTag.getY(), toTag.getX()); // radians
        double headingError = MathUtil.angleModulus(targetHeading - robotPose.getRotation().getRadians());

        return Optional.of(Math.toDegrees(headingError));
    }

    /**
     * track IDs of detected April Tags
     */
    public int[] targetIds;

    // ========== DriveToTag Helper Methods ==========

    /**
     * Returns the fiducial ID of the best (most centered) visible target.
     * 
     * @return Optional containing the best target's ID, or empty if no targets
     *         visible
     */
    public Optional<Integer> getBestTargetId() {
        if (latestResult == null || !latestResult.hasTargets()) {
            return Optional.empty();
        }
        return Optional.of(latestResult.getBestTarget().getFiducialId());
    }

    /**
     * Returns the Pose2d of the best visible AprilTag using the field layout.
     * 
     * @return Optional containing the tag's field pose, or empty if no targets
     *         visible
     */
    public Optional<Pose2d> getBestTagPose2d() {
        Optional<Integer> bestId = getBestTargetId();
        if (bestId.isEmpty()) {
            return Optional.empty();
        }
        return tagPose(bestId.get());
    }

    /**
     * Finds the closest AprilTag to the robot's current position.
     * Only considers reef tags (6-11 for red, 17-22 for blue).
     * 
     * @param robotPose the robot's current pose from odometry
     * @return Optional containing the closest tag's ID, or empty if no reef tags
     *         exist
     */
    public Optional<Integer> getClosestTagId(Pose2d robotPose) {
        // Reef tag IDs: Red alliance 6-11, Blue alliance 17-22
        int[] reefTags = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };

        int closestId = -1;
        double closestDistance = Double.MAX_VALUE;

        for (int tagId : reefTags) {
            Optional<Pose2d> tagPoseOpt = tagPose(tagId);
            if (tagPoseOpt.isPresent()) {
                double distance = robotPose.getTranslation().getDistance(tagPoseOpt.get().getTranslation());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestId = tagId;
                }
            }
        }

        if (closestId == -1) {
            return Optional.empty();
        }
        return Optional.of(closestId);
    }

    /**
     * Returns the Pose2d of the closest AprilTag to the robot.
     * 
     * @param robotPose the robot's current pose from odometry
     * @return Optional containing the closest tag's field pose, or empty if none
     *         found
     */
    public Optional<Pose2d> getClosestTagPose2d(Pose2d robotPose) {
        Optional<Integer> closestId = getClosestTagId(robotPose);
        if (closestId.isEmpty()) {
            return Optional.empty();
        }
        return tagPose(closestId.get());
    }

    /**
     * Calculates the target pose for the robot to drive to, positioned in front of
     * the tag.
     * The robot will face the tag at the specified offset distance.
     * 
     * @param tagPose        the AprilTag's field pose
     * @param offsetDistance distance in meters to stop in front of the tag
     * @return the target Pose2d for the robot
     */
    public Pose2d calculateTargetPose(Pose2d tagPose, double offsetDistance) {
        // Get the tag's rotation (the direction the tag is facing)
        Rotation2d tagRotation = tagPose.getRotation();

        // Calculate offset position (in front of the tag)
        // The tag faces outward, so we add offset in the direction the tag faces
        double offsetX = tagPose.getX() + offsetDistance * tagRotation.getCos();
        double offsetY = tagPose.getY() + offsetDistance * tagRotation.getSin();

        // Robot should face the tag (opposite of tag's facing direction)
        Rotation2d robotRotation = tagRotation.plus(Rotation2d.fromDegrees(180));

        return new Pose2d(offsetX, offsetY, robotRotation);
    }

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

        if (targetIds.length != 0) {
            // double targetYawMeters = latestResult.targets.get(0).getYaw();
            // double targetPitchMeters = latestResult.targets.get(0).getPitch();
            // double targetSkewMeters = latestResult.targets.get(0).getSkew();
            // SmartDashboard.putNumber("Yaw", targetYawMeters);
            // SmartDashboard.putNumber("Pitch", targetPitchMeters);
            // SmartDashboard.putNumber("Skew", targetSkewMeters);
            Transform3d targetTrans = latestResult.targets.get(0).getBestCameraToTarget();
            SmartDashboard.putString("PhotonTarget", targetTrans.toString());
        }

        // Estimated pose
        Optional<EstimatedRobotPose> estimated = estimator.estimateCoprocMultiTagPose(latestResult);

        if (estimated.isEmpty()) {
            estimated = estimator.estimateLowestAmbiguityPose(latestResult);
        }
        SmartDashboard.putBoolean("vision-estimated:", estimated.isPresent());

        if (estimated.isPresent()) {
            Pose3d meterPose = estimated.get().estimatedPose;
            field.setRobotPose(meterPose.getX(), meterPose.getY(), meterPose.getRotation().toRotation2d());

            Pose3d inchPose = new Pose3d(
                    meterPose.getTranslation().getX() * 39.3701,
                    meterPose.getTranslation().getY() * 39.3701,
                    meterPose.getTranslation().getZ() * 39.3701,
                    meterPose.getRotation());
            SmartDashboard.putString("Vision - Estimated Pose (in)", inchPose.toString());
            // SmartDashboard.putNumber("Vision - x: ", inchPose.getX());
            // SmartDashboard.putNumber("Vision - y: ", inchPose.getY());

            SmartDashboard.putString("Rotation - vision: ", meterPose.getRotation().toString());

            // SmartDashboard.putNumber("Closest April Tag: ", getClosestID());

        }

        // Field
        SmartDashboard.putData("Vision - Field", field);

        double now = Timer.getFPGATimestamp();

        if (now - lastLogTime >= 1.0) {

            if (latestResult.hasTargets()) {
                var target = latestResult.getBestTarget();
                int tagNumber = target.getFiducialId();
                double dist = latestResult.getBestTarget().getBestCameraToTarget().getX();

                tagNumberLogger.append(tagNumber); // log the tag number
                distancelog.append(dist); // log the distance from the tag

            }

            lastLogTime = now;
        }

    }

}