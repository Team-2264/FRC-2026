package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.ElasticDashboard;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motors.Neo;
import frc.lib.motors.NeoConfiguration;

/**
 * Shooter subsystem — flywheel with closed-loop RPM control + feeder motor.
 * Distance→RPM is interpolated via {@link InterpolatingDoubleTreeMap}.
 */
public class Shooter extends SubsystemBase {

    private final Neo motor;
    private final Neo feederMotor;
    private boolean isShooting = false;

    private double targetRPM = 0;
    private double currentFeederSpeed = 0;

    private static final double FEEDER_SPEED = 0.50;
    private static final double UP_TO_SPEED_THRESHOLD = 0.95;

    private final InterpolatingDoubleTreeMap distanceToRpmMap =
            Constants.ShooterConfig.Shooter.distanceToRpmMap;

    /**
     * Creates a new Shooter subsystem.
     */
    public Shooter() {
        // Shooter motor — config comes from Constants (includes PID + output range)
        motor = new Neo(Constants.ShooterConfig.Shooter.neoConfig);

        // Feeder motor
        NeoConfiguration feederConfig = new NeoConfiguration(Constants.ShooterConfig.Feeder.motorId)
                .withBrakeMode(true)
                .withMotorInvert(true)
                .withSmartCurrentLimit(30)
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0);
        feederMotor = new Neo(feederConfig);
    }

    @Override
    public void periodic() {
        ElasticDashboard.putBoolean("Shooter/Active", isShooting);
        ElasticDashboard.putNumber("Shooter/VelocityRPS", motor.getVelocity());
        ElasticDashboard.putNumber("Shooter/VelocityRPM", getVelocityRPM());
        ElasticDashboard.putNumber("Shooter/TargetRPM", targetRPM);
        ElasticDashboard.putNumber("Shooter/Current", motor.getOutputCurrent());
        ElasticDashboard.putNumber("Shooter/FeederSpeed", currentFeederSpeed);
        ElasticDashboard.putNumber("Shooter/FeederCurrent", feederMotor.getOutputCurrent());
        ElasticDashboard.putBoolean("Shooter/UpToSpeed", isUpToSpeed());
    }

    // ── RPM-based shooting ──

    /** Look up RPM for distance and spin flywheel. */
    public void shootForDistance(double distanceMeters) {
        double rpm = distanceToRpmMap.get(distanceMeters);
        shootAtRPM(rpm);
    }

    /**
     * Command the flywheel to the given RPM via closed-loop velocity control.
     */
    public void shootAtRPM(double rpm) {
        isShooting = true;
        targetRPM = rpm;
        double targetRPS = rpm / 60.0;
        motor.rotateAtVelocity(targetRPS);
    }

    /** Start flywheel at default RPM. */
    public void shoot() {
        shootAtRPM(Constants.ShooterConfig.Shooter.DEFAULT_SHOOTER_RPM);
    }

    // ── Feeder ──

    public void startFeeder() {
        currentFeederSpeed = FEEDER_SPEED;
        feederMotor.rotateAtSpeed(FEEDER_SPEED);
    }

    /** Stop the feeder motor. */
    public void stopFeeder() {
        currentFeederSpeed = 0;
        feederMotor.stop();
    }

    // ── Stop ──

    public void stop() {
        isShooting = false;
        targetRPM = 0;
        motor.stop();
        stopFeeder();
    }

    // ── Queries ──

    public boolean isShooting() {
        return isShooting;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    /** Current flywheel velocity in RPS. */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /** Current flywheel velocity in RPM. */
    public double getVelocityRPM() {
        return motor.getVelocity() * 60.0;
    }

    /** True when flywheel is within 95% of target RPM. */
    public boolean isUpToSpeed() {
        if (targetRPM == 0) return false;
        return Math.abs(getVelocityRPM()) >= UP_TO_SPEED_THRESHOLD * Math.abs(targetRPM);
    }

    /** Current feeder motor velocity (RPS). */
    public double getFeederVelocity() {
        return feederMotor.getVelocity();
    }
}
