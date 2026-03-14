package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.ElasticDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/**
 * FaceGoal — rotation-only command that turns the robot to face the goal.
 * Auto-selects reef based on alliance. Includes shoot-while-moving heading compensation.
 * Press once, auto-finishes when aimed.
 */
public class FaceGoal extends Command {

    private final Swerve swerve;
    private final double angleOffsetRad;

    private static final double RED_GOAL_X = 12.004;
    private static final double RED_GOAL_Y = 4.035;
    private static final double BLUE_GOAL_X = 4.537;
    private static final double BLUE_GOAL_Y = 4.035;

    private static final double ROT_KP = 3.0;
    private static final double MAX_ROT = 2.0;
    private static final double SLOW_KP = 1.2;
    private static final double SLOW_MAX_ROT = 0.5;
    private static final double SLOW_THRESHOLD = 10.0;
    private static final double ANGLE_TOLERANCE = 1.0;

    private static final int TIMEOUT_CYCLES = 150;
    private int cycleCount = 0;
    private boolean isDone = false;
    private double goalX;
    private double goalY;

    private final boolean useCustomTarget;
    private final double customGoalX;
    private final double customGoalY;

    /** Faces directly at goal center. */
    public FaceGoal(Swerve swerve) {
        this(swerve, 0.0);
    }

    /** Faces goal with an angular offset (degrees). */
    public FaceGoal(Swerve swerve, double angleOffsetDeg) {
        this.swerve = swerve;
        this.angleOffsetRad = Math.toRadians(angleOffsetDeg);
        this.useCustomTarget = false;
        this.customGoalX = 0;
        this.customGoalY = 0;
        addRequirements(swerve);
    }

    /** Faces a specific field position (x, y) with angular offset (degrees). */
    public FaceGoal(Swerve swerve, double targetX, double targetY, double angleOffsetDeg) {
        this.swerve = swerve;
        this.angleOffsetRad = Math.toRadians(angleOffsetDeg);
        this.useCustomTarget = true;
        this.customGoalX = targetX;
        this.customGoalY = targetY;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        cycleCount = 0;
        isDone = false;

        if (useCustomTarget) {
            // Face a specific field position
            goalX = customGoalX;
            goalY = customGoalY;
            ElasticDashboard.putString("FaceGoal/Alliance", "CUSTOM TARGET");
        } else {
            // Pick the correct reef based on alliance
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
                goalX = BLUE_GOAL_X;
                goalY = BLUE_GOAL_Y;
                ElasticDashboard.putString("FaceGoal/Alliance", "BLUE");
            } else {
                goalX = RED_GOAL_X;
                goalY = RED_GOAL_Y;
                ElasticDashboard.putString("FaceGoal/Alliance", "RED");
            }
        }

    ElasticDashboard.putBoolean("FaceGoal/Active", true);
    ElasticDashboard.putString("FaceGoal/Status", "ROTATING");
    }

    @Override
    public void execute() {
        cycleCount++;
        if (cycleCount > TIMEOUT_CYCLES) {
            isDone = true;
            swerve.drive(new Translation2d(0, 0), 0);
            ElasticDashboard.putString("FaceGoal/Status", "TIMEOUT");
            return;
        }

        Pose2d currentPose = swerve.getPose();
        double robotX = currentPose.getX();
        double robotY = currentPose.getY();
        double robotHeadingRad = currentPose.getRotation().getRadians();

        // Raw bearing to goal
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double goalAngle = Math.atan2(dy, dx);
        double distance = Math.hypot(dx, dy);

        // Shoot-while-moving compensation
        ChassisSpeeds fieldVel = swerve.getFieldRelativeSpeeds();
        double vx = fieldVel.vxMetersPerSecond;
        double vy = fieldVel.vyMetersPerSecond;

        double vPerp = vx * Math.sin(goalAngle) - vy * Math.cos(goalAngle);
        double vPar  = vx * Math.cos(goalAngle) + vy * Math.sin(goalAngle);

        double muzzleSpeed = Constants.ShooterConfig.Shooter.MUZZLE_SPEED_MPS;
        double ratio = Math.max(-1.0, Math.min(1.0, -vPerp / muzzleSpeed));
        double headingOffset = Math.asin(ratio);

        // Compensated target heading
        double targetAngleRad = goalAngle + headingOffset + angleOffsetRad;

        double flightTime = (muzzleSpeed > 0) ? distance / muzzleSpeed : 0;
        double effectiveDistance = distance - vPar * flightTime;

        // Rotation error (compensated)
        double errorRad = targetAngleRad - robotHeadingRad;

        // Normalize to -PI..PI
        while (errorRad > Math.PI) errorRad -= 2 * Math.PI;
        while (errorRad < -Math.PI) errorRad += 2 * Math.PI;

        double errorDeg = Math.toDegrees(errorRad);

    ElasticDashboard.putNumber("FaceGoal/ErrorDeg", errorDeg);
    ElasticDashboard.putNumber("FaceGoal/TargetAngleDeg", Math.toDegrees(targetAngleRad));
    ElasticDashboard.putNumber("FaceGoal/HeadingOffsetDeg", Math.toDegrees(headingOffset));
    ElasticDashboard.putNumber("FaceGoal/EffectiveDistance", effectiveDistance);
    ElasticDashboard.putNumber("FaceGoal/vPerp", vPerp);
    ElasticDashboard.putNumber("FaceGoal/vPar", vPar);

        // Shoot gate: check compensated heading error
        if (Math.abs(errorDeg) < ANGLE_TOLERANCE) {
            isDone = true;
            swerve.drive(new Translation2d(0, 0), 0);
            ElasticDashboard.putString("FaceGoal/Status", "FACING GOAL!");
            return;
        }

        // Two-speed rotation: full speed when far, slow when within threshold
        double kp = (Math.abs(errorDeg) < SLOW_THRESHOLD) ? SLOW_KP : ROT_KP;
        double cap = (Math.abs(errorDeg) < SLOW_THRESHOLD) ? SLOW_MAX_ROT : MAX_ROT;

        double rot = errorRad * kp;
        rot = Math.max(-cap, Math.min(cap, rot));

        // Drive with zero translation, just rotation
        swerve.drive(new Translation2d(0, 0), rot);
    ElasticDashboard.putString("FaceGoal/Status",
            (Math.abs(errorDeg) < SLOW_THRESHOLD) ? "ROTATING (SLOW)" : "ROTATING");
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0);
    ElasticDashboard.putBoolean("FaceGoal/Active", false);
    ElasticDashboard.putString("FaceGoal/Status", interrupted ? "CANCELLED" : "DONE");
    }
}
