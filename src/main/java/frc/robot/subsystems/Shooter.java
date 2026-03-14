package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.ElasticDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motors.Neo;
import frc.lib.motors.NeoConfiguration;
import edu.wpi.first.math.util.Units;

/**
 * Shooter subsystem for the 2026 FRC game.
 * Uses a single Neo motor (CAN ID 21) for shooting balls.
 * Speed adjusts based on distance to target.
 */
public class Shooter extends SubsystemBase {

    private final Neo motor;
    private final Neo feederMotor;
    private boolean isShooting = false;
    private double currentSpeed = 0;
    private double currentFeederSpeed = 0;

    // Distance-based speed now uses InterpolatingDoubleTreeMap in Constants.
    // See Constants.ShooterConfig.Shooter.speedTable for the table entries.
    private static final double DEFAULT_SPEED = Constants.ShooterConfig.Shooter.DefaultShooterDutyCycle;

    // Feeder motor configuration (placeholder ID — set real CAN ID when known)
    private static final int FEEDER_MOTOR_ID = 32;
    private static final int SHOOTER_MOTOR_ID = 30;
    private static final double FEEDER_SPEED = -0.50; // duty cycle for feeding

    /**
     * Creates a new Shooter subsystem using CAN ID 21.
     */
    public Shooter() {
        // Configure the shooter motor using same style as ElevatorConfig
        NeoConfiguration config = new NeoConfiguration(SHOOTER_MOTOR_ID)
                .withBrakeMode(false) // Coast mode for flywheel
                .withMotorInvert(false)
                .withSmartCurrentLimit(40)
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0);

        motor = new Neo(config);

        // Feeder motor: coast disabled so it stops quickly when button released
        NeoConfiguration feederConfig = new NeoConfiguration(FEEDER_MOTOR_ID)
                .withBrakeMode(true)
                .withMotorInvert(false)
                .withSmartCurrentLimit(30)
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0);
        feederMotor = new Neo(feederConfig);
    }

    @Override
    public void periodic() {
        ElasticDashboard.putBoolean("Shooter/Active", isShooting);
        ElasticDashboard.putNumber("Shooter/Velocity", motor.getVelocity());
        ElasticDashboard.putNumber("Shooter/Current", motor.getOutputCurrent());
        ElasticDashboard.putNumber("Shooter/Speed", currentSpeed);
        ElasticDashboard.putNumber("Shooter/FeederSpeed", currentFeederSpeed);
        ElasticDashboard.putNumber("Shooter/FeederCurrent", feederMotor.getOutputCurrent());
        double shooterVelocity = motor.getVelocity();
    }

    /**
     * Look up shooter duty cycle for a given distance using the
     * {@link Constants.ShooterConfig.Shooter#speedTable InterpolatingDoubleTreeMap}.
     *
     * @param distanceInFeet Distance to target in <b>feet</b>.
     * @return Duty cycle (negative values = correct motor direction).
     */
    public double calculateSpeedForDistance(double distanceInFeet) {
        double speed = Constants.ShooterConfig.Shooter.speedTable.get(Units.feetToMeters(distanceInFeet));
        ElasticDashboard.putNumber("Shooter/TargetDistInches", Units.feetToMeters(distanceInFeet));
        ElasticDashboard.putNumber("Shooter/CalculatedSpeed", speed);

        return speed;
    }

    /**
     * Start the shooter motor at default speed.
     */
    public void shoot() {
        shoot(DEFAULT_SPEED);
    }

    /**
     * Start the shooter motor at a specific speed.
     * 
     * @param speed Duty cycle from -1.0 to 1.0
     */
    public void shoot(double speed) {
        isShooting = true;
        currentSpeed = speed;
        motor.rotateAtSpeed(speed);
    }

    /**
     * Shoot based on distance to target.
     * Looks up duty cycle from the interpolation table.
     *
     * @param distanceInches Distance to target in <b>inches</b>.
     */
    public void shootForDistance(double distanceInches) {
        double speed = calculateSpeedForDistance(distanceInches);
        shoot(speed);
    }

    /**
     * Start the feeder motor at the configured speed.
     */
    public void startFeeder() {
        currentFeederSpeed = FEEDER_SPEED;
        feederMotor.rotateAtSpeed(FEEDER_SPEED);
    }

    /**
     * Stop the feeder motor.
     */
    public void stopFeeder() {
        currentFeederSpeed = 0;
        feederMotor.stop();
    }

    /**
     * Stop the shooter motor.
     */
    public void stop() {
        isShooting = false;
        currentSpeed = 0;
        motor.stop();
        stopFeeder();
    }

    /**
     * Check if the shooter is currently running.
     */
    public boolean isShooting() {
        return isShooting;
    }

    /**
     * Get the current commanded shooter duty cycle (last set in
     * {@link #shoot(double)}).
     */
    public double getCurrentSpeedDutyCycle() {
        return currentSpeed;
    }

    /**
     * Get the current velocity of the shooter motor.
     */
    public double getVelocity() {
        return motor.getVelocity();
    }

    /**
     * Get the current feeder motor velocity.
     */
    public double getFeederVelocity() {
        return feederMotor.getVelocity();
    }
}
