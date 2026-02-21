package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.ElasticDashboard;
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

    // Red reef center (average of tags 6-11)
    private static final double RED_GOAL_X = 514.13 * 0.0254;  // meters
    private static final double RED_GOAL_Y = 158.50 * 0.0254;

    // Blue reef center (average of tags 17-22)
    private static final double BLUE_GOAL_X = 176.75 * 0.0254;
    private static final double BLUE_GOAL_Y = 158.50 * 0.0254;

    // Rotation control
    private static final double ROT_KP = 3.0;
    private static final double MAX_ROT = 2.0;       // rad/s
    private static final double ANGLE_TOLERANCE = 2.0; // degrees

    // Safety
    private static final int TIMEOUT_CYCLES = 150; // 3 seconds
    private int cycleCount = 0;
    private boolean isDone = false;
    private double goalX;
    private double goalY;

    public FaceGoal(Swerve swerve) {
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        cycleCount = 0;
        isDone = false;

        // Pick the correct reef based on alliance
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            goalX = BLUE_GOAL_X;
            goalY = BLUE_GOAL_Y;
            ElasticDashboard.putString("FaceGoal/Alliance", "BLUE");
        } else {
            // Default to red if unknown
            goalX = RED_GOAL_X;
            goalY = RED_GOAL_Y;
            ElasticDashboard.putString("FaceGoal/Alliance", "RED");
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

        // Angle from robot to the goal
        double dx = goalX - robotX;
        double dy = goalY - robotY;
        double targetAngleRad = Math.atan2(dy, dx);

        // Error = how far we need to rotate
        double errorRad = targetAngleRad - robotHeadingRad;

        // Normalize to -PI..PI
        while (errorRad > Math.PI) errorRad -= 2 * Math.PI;
        while (errorRad < -Math.PI) errorRad += 2 * Math.PI;

        double errorDeg = Math.toDegrees(errorRad);

    ElasticDashboard.putNumber("FaceGoal/ErrorDeg", errorDeg);
    ElasticDashboard.putNumber("FaceGoal/TargetAngleDeg", Math.toDegrees(targetAngleRad));

        if (Math.abs(errorDeg) < ANGLE_TOLERANCE) {
            isDone = true;
            swerve.drive(new Translation2d(0, 0), 0);
            ElasticDashboard.putString("FaceGoal/Status", "FACING GOAL!");
            return;
        }

        // Proportional rotation — field-relative drive with zero translation
        double rot = errorRad * ROT_KP;
        rot = Math.max(-MAX_ROT, Math.min(MAX_ROT, rot));

        // Drive with zero translation, just rotation (field-relative stays on so heading is correct)
        swerve.drive(new Translation2d(0, 0), rot);
    ElasticDashboard.putString("FaceGoal/Status", "ROTATING");
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
