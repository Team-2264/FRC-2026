package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/**
 * FaceGoal - Rotation-only command that turns the robot to face the reef (goal).
 * 
 * Automatically picks the correct reef based on alliance color.
 * Uses the robot's current field position (from odometry/vision) to calculate
 * the angle to the reef center and rotates to face it. Works from anywhere on the field.
 * 
 * Press once, auto-finishes when facing the goal.
 */
public class FaceGoal extends Command {

    private final Swerve swerve;
    private final double angleOffsetRad; // offset in radians applied to the target angle

    // 2026 reef/cage centers (average of reef tag positions, in meters)
    // Red reef: tags 2,3,4,5,8,9,10,11
    private static final double RED_GOAL_X = 12.004;  // meters
    private static final double RED_GOAL_Y = 4.035;

    // Blue reef: tags 18,19,20,21,24,25,26,27
    private static final double BLUE_GOAL_X = 4.537;
    private static final double BLUE_GOAL_Y = 4.035;

    // Rotation control
    private static final double ROT_KP = 3.0;
    private static final double MAX_ROT = 2.0;        // rad/s — full speed
    private static final double SLOW_KP = 1.2;
    private static final double SLOW_MAX_ROT = 0.5;   // rad/s — slow speed for fine alignment
    private static final double SLOW_THRESHOLD = 10.0; // degrees — switch to slow below this
    private static final double ANGLE_TOLERANCE = 1.0; // degrees

    // Safety
    private static final int TIMEOUT_CYCLES = 150; // 3 seconds
    private int cycleCount = 0;
    private boolean isDone = false;
    private double goalX;
    private double goalY;

    // When true, goalX/goalY are set explicitly and not from alliance
    private final boolean useCustomTarget;
    private final double customGoalX;
    private final double customGoalY;

    /**
     * Constructor with 0° offset (faces directly at goal center).
     */
    public FaceGoal(Swerve swerve) {
        this(swerve, 0.0);
    }

    /**
     * Constructor with custom angle offset.
     * @param swerve Swerve subsystem
     * @param angleOffsetDeg Offset in degrees added to the "face goal" angle.
     *                       Negative = rotated right of goal, 0 = directly at goal,
     *                       Positive = rotated left of goal.
     */
    public FaceGoal(Swerve swerve, double angleOffsetDeg) {
        this.swerve = swerve;
        this.angleOffsetRad = Math.toRadians(angleOffsetDeg);
        this.useCustomTarget = false;
        this.customGoalX = 0;
        this.customGoalY = 0;
        addRequirements(swerve);
    }

    /**
     * Constructor that faces a specific field position instead of the reef center.
     * @param swerve Swerve subsystem
     * @param targetX Field X coordinate to face (meters)
     * @param targetY Field Y coordinate to face (meters)
     * @param angleOffsetDeg Offset in degrees
     */
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
            SmartDashboard.putString("FaceGoal/Alliance", "CUSTOM TARGET");
        } else {
            // Pick the correct reef based on alliance
            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
                goalX = BLUE_GOAL_X;
                goalY = BLUE_GOAL_Y;
                SmartDashboard.putString("FaceGoal/Alliance", "BLUE");
            } else {
                goalX = RED_GOAL_X;
                goalY = RED_GOAL_Y;
                SmartDashboard.putString("FaceGoal/Alliance", "RED");
            }
        }

        SmartDashboard.putBoolean("FaceGoal/Active", true);
        SmartDashboard.putString("FaceGoal/Status", "ROTATING");
    }

    @Override
    public void execute() {
        cycleCount++;
        if (cycleCount > TIMEOUT_CYCLES) {
            isDone = true;
            swerve.drive(new Translation2d(0, 0), 0);
            SmartDashboard.putString("FaceGoal/Status", "TIMEOUT");
            return;
        }

        Pose2d currentPose = swerve.getPose();
        double robotX = currentPose.getX();
        double robotY = currentPose.getY();
        double robotHeadingRad = currentPose.getRotation().getRadians();

        // Angle from robot to the goal, plus the user-specified offset
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double targetAngleRad = Math.atan2(dy, dx) + angleOffsetRad;

        // Error = how far we need to rotate
        double errorRad = targetAngleRad - robotHeadingRad;

        // Normalize to -PI..PI
        while (errorRad > Math.PI) errorRad -= 2 * Math.PI;
        while (errorRad < -Math.PI) errorRad += 2 * Math.PI;

        double errorDeg = Math.toDegrees(errorRad);

        SmartDashboard.putNumber("FaceGoal/ErrorDeg", errorDeg);
        SmartDashboard.putNumber("FaceGoal/TargetAngleDeg", Math.toDegrees(targetAngleRad));

        if (Math.abs(errorDeg) < ANGLE_TOLERANCE) {
            isDone = true;
            swerve.drive(new Translation2d(0, 0), 0);
            SmartDashboard.putString("FaceGoal/Status", "FACING GOAL!");
            return;
        }

        // Two-speed rotation: full speed when far, slow when within threshold
        double kp = (Math.abs(errorDeg) < SLOW_THRESHOLD) ? SLOW_KP : ROT_KP;
        double cap = (Math.abs(errorDeg) < SLOW_THRESHOLD) ? SLOW_MAX_ROT : MAX_ROT;

        double rot = errorRad * kp;
        rot = Math.max(-cap, Math.min(cap, rot));

        // Drive with zero translation, just rotation
        swerve.drive(new Translation2d(0, 0), rot);
        SmartDashboard.putString("FaceGoal/Status",
            (Math.abs(errorDeg) < SLOW_THRESHOLD) ? "ROTATING (SLOW)" : "ROTATING");
    }

    @Override
    public boolean isFinished() {
        return isDone;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0);
        SmartDashboard.putBoolean("FaceGoal/Active", false);
        SmartDashboard.putString("FaceGoal/Status", interrupted ? "CANCELLED" : "DONE");
    }
}
