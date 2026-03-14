package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.ElasticDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

/**
 * ShootBall — hold to spin flywheel at distance-compensated RPM and auto-aim at goal.
 * Translation stays on sticks, rotation is overridden to face goal with shoot-while-moving compensation.
 * Bound to triangle (whileTrue).
 */
public class ShootBall extends Command {

    private final Shooter shooter;
    private final Swerve swerve;

    private static final double RED_GOAL_X = 12.004;
    private static final double RED_GOAL_Y = 4.035;
    private static final double BLUE_GOAL_X = 4.537;
    private static final double BLUE_GOAL_Y = 4.035;

    public ShootBall(Shooter shooter, Swerve swerve) {
        this.shooter = shooter;
        this.swerve = swerve;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // Determine goal based on alliance and activate auto-aim
        double goalX, goalY;
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            goalX = BLUE_GOAL_X;
            goalY = BLUE_GOAL_Y;
        } else {
            goalX = RED_GOAL_X;
            goalY = RED_GOAL_Y;
        }
        swerve.setAutoAimGoal(new Translation2d(goalX, goalY));
        ElasticDashboard.putBoolean("ShootBall/Active", true);
    }

    @Override
    public void execute() {
        // Re-derive goal from alliance for RPM calculation
        double goalX, goalY;
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            goalX = BLUE_GOAL_X;
            goalY = BLUE_GOAL_Y;
        } else {
            goalX = RED_GOAL_X;
            goalY = RED_GOAL_Y;
        }

        Pose2d pose = swerve.getPose();
        double dx = goalX - pose.getX();
        double dy = goalY - pose.getY();
        double distance = Math.hypot(dx, dy);
        double goalAngle = Math.atan2(dy, dx);

        ChassisSpeeds fieldVel = swerve.getFieldRelativeSpeeds();
        double vPar = fieldVel.vxMetersPerSecond * Math.cos(goalAngle)
                    + fieldVel.vyMetersPerSecond * Math.sin(goalAngle);

        double muzzleSpeed = Constants.ShooterConfig.Shooter.MUZZLE_SPEED_MPS;
        double flightTime = (muzzleSpeed > 0) ? distance / muzzleSpeed : 0;
        double effectiveDistance = Math.max(0, distance - vPar * flightTime);

        // Spin flywheel at RPM for the effective distance
        shooter.shootForDistance(effectiveDistance);

        ElasticDashboard.putNumber("ShootBall/TargetRPM", shooter.getTargetRPM());
        ElasticDashboard.putNumber("ShootBall/ActualRPM", shooter.getVelocityRPM());
        ElasticDashboard.putNumber("ShootBall/Distance", distance);
        ElasticDashboard.putNumber("ShootBall/EffectiveDistance", effectiveDistance);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        swerve.clearAutoAim();
        ElasticDashboard.putBoolean("ShootBall/Active", false);
    }
}
