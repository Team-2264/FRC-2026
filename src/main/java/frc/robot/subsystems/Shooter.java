package frc.robot.subsystems;

import frc.robot.ElasticDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motors.Neo;
import frc.lib.motors.NeoConfiguration;

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

    // Distance-based speed constants
    private static final double MIN_DISTANCE = 0.0; // meters - closest shot
    private static final double MAX_DISTANCE = 5.0; // meters - farthest shot
    private static final double MIN_SPEED = 0.10; // 10% speed at closest distance
    private static final double MAX_SPEED = 0.75; // 75% speed at farthest distance
    private static final double DEFAULT_SPEED = 0.5; // fallback if no tag visible

    // Feeder motor configuration (placeholder ID — set real CAN ID when known)
    private static final int FEEDER_MOTOR_ID = 22;
    private static final double FEEDER_SPEED = 0.25; // duty cycle for feeding

    /**
     * Creates a new Shooter subsystem using CAN ID 21.
     */
    public Shooter() {
        // Configure the shooter motor using same style as ElevatorConfig
        NeoConfiguration config = new NeoConfiguration(21)
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
    }

    /**
     * Calculate shooter speed based on distance to target.
     * Closer = slower, farther = faster.
     * 
     * @param distanceMeters Distance to target in meters
     * @return Duty cycle speed (0.0 to 1.0)
     */
    public double calculateSpeedForDistance(double distanceMeters) {
        // Clamp distance to valid range
        double clampedDistance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distanceMeters));

        // Linear interpolation between MIN_SPEED and MAX_SPEED based on distance
        double ratio = (clampedDistance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
        double speed = MIN_SPEED + ratio * (MAX_SPEED - MIN_SPEED);

        ElasticDashboard.putNumber("Shooter/TargetDistance", distanceMeters);
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
     * 
     * @param distanceMeters Distance to target in meters
     */
    public void shootForDistance(double distanceMeters) {
        double speed = calculateSpeedForDistance(distanceMeters);
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
