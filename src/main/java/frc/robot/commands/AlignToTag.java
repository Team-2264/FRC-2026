package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.ElasticDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;

/**
 * AlignToTag - Auto-aligns to an AprilTag using field-relative control.
 * Rotates to face the tag, strafes to center, and drives forward.
 * Uses the tag's known field position for accurate alignment.
 */
public class AlignToTag extends Command {

    private final Swerve swerve;
    private final Vision2025 vision;
    private final double targetDistance;
    
    // PID Controllers
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotController;
    
    // State
    private Pose2d targetPose;
    private int currentTagId;
    
    // Tuning constants
    private static final double MAX_SPEED = 1.5;
    private static final double MAX_ROT_SPEED = 2.0;

    public AlignToTag(Swerve swerve, Vision2025 vision, double targetDistance) {
        this.swerve = swerve;
        this.vision = vision;
        this.targetDistance = targetDistance;
        
        // PID for X (forward/back)
        xController = new PIDController(1.5, 0, 0.05);
        xController.setTolerance(0.05);
        
        // PID for Y (left/right)
        yController = new PIDController(1.5, 0, 0.05);
        yController.setTolerance(0.05);
        
        // PID for rotation
        rotController = new PIDController(2.0, 0, 0.1);
        rotController.setTolerance(Math.toRadians(2.0));
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        
        addRequirements(swerve);
    }

    public AlignToTag(Swerve swerve, Vision2025 vision) {
        this(swerve, vision, 0.5);
    }

    @Override
    public void initialize() {
        targetPose = null;
        currentTagId = -1;
        xController.reset();
        yController.reset();
        rotController.reset();
    ElasticDashboard.putBoolean("AlignToTag/Active", true);
    }

    @Override
    public void execute() {
        // Try to get a visible tag
        Optional<Integer> bestTargetId = vision.getBestTargetId();
        Optional<Transform3d> targetTransform = vision.getTargetTransform();
        
        if (bestTargetId.isPresent() && targetTransform.isPresent()) {
            currentTagId = bestTargetId.get();
            
            // Get the tag's field position
            Optional<Pose2d> tagPoseOpt = vision.tagPose(currentTagId);
            if (tagPoseOpt.isPresent()) {
                // Calculate where we want to be (in front of tag, facing it)
                targetPose = vision.calculateTargetPose(tagPoseOpt.get(), targetDistance);
                driveToTarget();
            } else {
                // Tag not in field layout - fall back to camera-relative
                driveCameraRelative(targetTransform.get());
            }
        } else {
            // No tag visible - stop
            swerve.drive(new Translation2d(0, 0), 0);
            ElasticDashboard.putNumber("AlignToTag/TagId", -1);
        }
    }

    private void driveToTarget() {
        if (targetPose == null) {
            swerve.drive(new Translation2d(0, 0), 0);
            return;
        }

        Pose2d robotPose = swerve.getPose();
        
        // Calculate errors
        double xError = targetPose.getX() - robotPose.getX();
        double yError = targetPose.getY() - robotPose.getY();
        double rotError = targetPose.getRotation().getRadians() - robotPose.getRotation().getRadians();
        
        // Normalize rotation error
        while (rotError > Math.PI) rotError -= 2 * Math.PI;
        while (rotError < -Math.PI) rotError += 2 * Math.PI;
        
        double distance = Math.sqrt(xError * xError + yError * yError);
        
        // PID outputs
        double xSpeed = xController.calculate(0, -xError);
        double ySpeed = yController.calculate(0, -yError);
        double rotSpeed = rotController.calculate(0, -rotError);
        
        // Slowdown when close
        double speedScale = Math.min(distance / 1.0, 1.0);
        speedScale = Math.max(speedScale, 0.15);
        
        // Clamp speeds
        xSpeed = clamp(xSpeed * speedScale, -MAX_SPEED, MAX_SPEED);
        ySpeed = clamp(ySpeed * speedScale, -MAX_SPEED, MAX_SPEED);
        rotSpeed = clamp(rotSpeed, -MAX_ROT_SPEED, MAX_ROT_SPEED);
        
        // Drive field-relative
        swerve.setFieldRelative(true);
        swerve.drive(new Translation2d(xSpeed, ySpeed), rotSpeed);
        
        // Debug
    ElasticDashboard.putNumber("AlignToTag/TagId", currentTagId);
    ElasticDashboard.putNumber("AlignToTag/Distance", distance);
    ElasticDashboard.putNumber("AlignToTag/RotError", Math.toDegrees(rotError));
    }

    private void driveCameraRelative(Transform3d transform) {
        double tagX = transform.getX();
        double tagY = transform.getY();
        double angleToTag = Math.atan2(tagY, tagX);
        
        double distanceToGo = tagX - targetDistance;
        
        // Forward proportional to distance
        double forwardSpeed = clamp(distanceToGo * 1.0, -MAX_SPEED, MAX_SPEED);
        
        // Strafe to center tag
        double strafeSpeed = clamp(-tagY * 1.0, -0.5, 0.5);
        
        // Rotate to face tag
        double rotSpeed = clamp(-angleToTag * 1.5, -MAX_ROT_SPEED, MAX_ROT_SPEED);
        
        swerve.setFieldRelative(false);
        swerve.drive(new Translation2d(forwardSpeed, strafeSpeed), rotSpeed);
        
    ElasticDashboard.putNumber("AlignToTag/TagId", currentTagId);
    ElasticDashboard.putNumber("AlignToTag/CamDistance", tagX);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0);
        swerve.setFieldRelative(true);
    ElasticDashboard.putBoolean("AlignToTag/Active", false);
    }
}
