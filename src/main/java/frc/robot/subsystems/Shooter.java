package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private boolean isShooting = false;
    private double currentSpeed = 0;
    
    // Distance-based speed constants
    private static final double MIN_DISTANCE = 0.5;   // meters - closest shot
    private static final double MAX_DISTANCE = 4.0;   // meters - farthest shot
    private static final double MIN_SPEED = 0.3;      // speed at closest distance
    private static final double MAX_SPEED = 0.8;      // speed at farthest distance
    private static final double DEFAULT_SPEED = 0.5;  // fallback if no tag visible

    /**
     * Creates a new Shooter subsystem using CAN ID 21.
     */
    public Shooter() {
        // Configure the shooter motor using same style as ElevatorConfig
        NeoConfiguration config = new NeoConfiguration(21)
            .withBrakeMode(false)  // Coast mode for flywheel
            .withMotorInvert(false)
            .withSmartCurrentLimit(40)
            .withKP(0.0)
            .withKI(0.0)
            .withKD(0.0);
            
        motor = new Neo(config);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Shooter/Active", isShooting);
        SmartDashboard.putNumber("Shooter/Velocity", motor.getVelocity());
        SmartDashboard.putNumber("Shooter/Current", motor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Speed", currentSpeed);
    }

    /**
     * Calculate shooter speed based on distance to target.
     * Closer = slower, farther = faster.
     * @param distanceMeters Distance to target in meters
     * @return Duty cycle speed (0.0 to 1.0)
     */
    public double calculateSpeedForDistance(double distanceMeters) {
        // Clamp distance to valid range
        double clampedDistance = Math.max(MIN_DISTANCE, Math.min(MAX_DISTANCE, distanceMeters));
        
        // Linear interpolation between MIN_SPEED and MAX_SPEED based on distance
        double ratio = (clampedDistance - MIN_DISTANCE) / (MAX_DISTANCE - MIN_DISTANCE);
        double speed = MIN_SPEED + ratio * (MAX_SPEED - MIN_SPEED);
        
        SmartDashboard.putNumber("Shooter/TargetDistance", distanceMeters);
        SmartDashboard.putNumber("Shooter/CalculatedSpeed", speed);
        
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
     * @param speed Duty cycle from -1.0 to 1.0
     */
    public void shoot(double speed) {
        isShooting = true;
        currentSpeed = speed;
        motor.rotateAtSpeed(speed);
    }

    /**
     * Shoot based on distance to target.
     * @param distanceMeters Distance to target in meters
     */
    public void shootForDistance(double distanceMeters) {
        double speed = calculateSpeedForDistance(distanceMeters);
        shoot(speed);
    }

    /**
     * Stop the shooter motor.
     */
    public void stop() {
        isShooting = false;
        currentSpeed = 0;
        motor.stop();
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
}
