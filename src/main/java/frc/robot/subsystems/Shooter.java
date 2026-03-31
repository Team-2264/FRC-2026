package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.ElasticDashboard;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
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
    private double currentRPM = 0;
    private double currentFeederDutyCycle = 0;

    // Distance-based speed now uses InterpolatingDoubleTreeMap in Constants.
    // See Constants.ShooterConfig.Shooter.speedTable for the table entries.
    private static final double DEFAULT_RPM = Constants.ShooterConfig.Shooter.DefaultShooterRPM;

    // Feeder motor configuration (placeholder ID — set real CAN ID when known)
    private static final int FEEDER_MOTOR_ID = 32;
    private static final int SHOOTER_MOTOR_ID = 30;
    private static final double FEEDER_DUTY_CYCLE = -1; // RPM for feeding

    /**
     * Creates a new Shooter subsystem using CAN ID 21.
     */
    public Shooter() {
        // Configure the shooter motor using same style as ElevatorConfig
        NeoConfiguration config = new NeoConfiguration(SHOOTER_MOTOR_ID)
                .withBrakeMode(false) // Coast mode for flywheel
                .withMotorInvert(true)
                .withSmartCurrentLimit(40)
                .withKFF(0.002114)
                .withOutputRange(-5000,5000)
                .withKP(0.00035)
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

        SendableRegistry.addLW(this, "Shooter");
    }

    @Override
    public void periodic() {
        ElasticDashboard.putBoolean("Shooter/Active", isShooting);
        ElasticDashboard.putNumber("Shooter/Velocity", motor.getVelocity());
        ElasticDashboard.putNumber("Shooter/Current", motor.getOutputCurrent());
        ElasticDashboard.putNumber("Shooter/InputRPM", currentRPM);
        ElasticDashboard.putNumber("Shooter/FeederTargetRPM", currentFeederDutyCycle);
        ElasticDashboard.putNumber("Shooter/FeederCurrent", feederMotor.getOutputCurrent());
    }

    /**
     * Look up shooter RPM for a given distance using the
     * {@link Constants.ShooterConfig.Shooter #speedTable InterpolatingDoubleTreeMap}.
     *
     * @param distanceInMeters Distance to target in <b>meters</b>.
     * @return Target RPM (negative values = correct motor direction).
     */
    public double calculateRPMForDistance(double distanceInMeters) {
        double rpm = Constants.ShooterConfig.Shooter.speedTable.get(distanceInMeters);
        if(distanceInMeters<0){
            rpm=Constants.ShooterConfig.Shooter.ShootFromCenterRPM;
        }
        ElasticDashboard.putNumber("Shooter/TargetDistInches", distanceInMeters);
        ElasticDashboard.putNumber("Shooter/CalculatedRPM", rpm);
        

        return rpm;
    }

    private double rpmFromDashboard;

    public double getRpmFromDashboard(){
        return rpmFromDashboard;
    }

    public void setRpmFromDashboard(double rpm){
        rpmFromDashboard=rpm;
    }

    @Override
    public void initSendable(SendableBuilder builder){
        builder.addDoubleProperty("Shooter/RpmFromDashboard", this::getRpmFromDashboard, this::setRpmFromDashboard);
    }

    public void shootAtSpeedFromDashboard(){
        shoot(rpmFromDashboard);
    }

    /**
     * Start the shooter motor at default RPM.
     */
    public void shoot() {
        shoot(DEFAULT_RPM);
    }

    /**
     * Start the shooter motor at a specific RPM.
     * 
     * @param rpm Target velocity in RPM
     */
    public void shoot(double rpm) {
        isShooting = true;
        currentRPM = rpm;
        motor.setVelocityRPM(rpm);
    }

    /**
     * Shoot based on distance to target.
     * Looks up RPM from the interpolation table.
     *
     * @param distanceInches Distance to target in <b>inches</b>.
     */
    public void shootForDistance(double distanceInches) {
        double rpm = calculateRPMForDistance(distanceInches);
        shoot(rpm);
    }

    /**
     * Start the feeder motor at the configured RPM.
     */
    public void startFeeder() {
        currentFeederDutyCycle = FEEDER_DUTY_CYCLE;
        feederMotor.rotateAtSpeed(currentFeederDutyCycle);
    }

    public void startFeederReverse() {
        currentFeederDutyCycle = -FEEDER_DUTY_CYCLE;
        feederMotor.rotateAtSpeed(currentFeederDutyCycle);
    }

    /**
     * Stop the feeder motor.
     */
    public void stopFeeder() {
        currentFeederDutyCycle = 0;
        feederMotor.stop();
    }

    /**
     * Stop the shooter motor.
     */
    public void stop() {
        isShooting = false;
        currentRPM = 0;
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
     * Get the current commanded shooter RPM (last set in
     * {@link #shoot(double)}).
     */
    public double getCurrentRPM() {
        return currentRPM;
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
