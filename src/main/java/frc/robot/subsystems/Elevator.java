package frc.robot.subsystems;

import java.util.OptionalDouble;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.ElasticDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.LimitSwitch;
import frc.lib.Math2264;
import frc.lib.motors.Neo;
import frc.robot.Constants;

/**
 * Subsystem for controlling the Elevator.
 * 
 */
public class Elevator extends SubsystemBase {

    /**
     * Enum for the Elevator status.
     * 
     */
    private enum HomingState {
        UNKNWON,
        MOVING_OFF_OF_LIMIT_SWITCH,
        MOVING_ON_TO_LIMIT_SWITCH,
        COMPLETE,
    }

    private Neo[] motors;
    private LimitSwitch homeLimitSwitch;
    private ElevatorFeedforward feedforward;
    private ProfiledPIDController pidController;

    private HomingState homingState = HomingState.UNKNWON;
    private OptionalDouble targetPositionInches = OptionalDouble.empty();

    private OptionalDouble targetPositionRequestWhileHoming = OptionalDouble.empty();

    private MedianFilter motorCurrentDrawMedianFilter = new MedianFilter(25);

    /**
     * Crate a new Elevator object
     * 
     * @param neoConfigurations Configuration for the NEO motors used to move the elevator
     */
    public Elevator() {
        // create the limit switch input
        this.homeLimitSwitch = new LimitSwitch(Constants.ElevatorConfig.homeLimitSwitchId, Constants.ElevatorConfig.homeLimitSwitchActiveHigh);

        this.feedforward = new ElevatorFeedforward(Constants.ElevatorConfig.feedForwardKs, 
                                                   Constants.ElevatorConfig.feedForwardKg, 
                                                   Constants.ElevatorConfig.feedForwardKv);

        this.pidController = new ProfiledPIDController(Constants.ElevatorConfig.pidKp,
                                                       Constants.ElevatorConfig.pidKi,
                                                       Constants.ElevatorConfig.pidKd,
                                                       new Constraints(Constants.ElevatorConfig.maxVelocityIps, 
                                                                       Constants.ElevatorConfig.accelerationIpsPs));

        // create motors
        motors = new Neo[Constants.ElevatorConfig.neoConfigs.length];
        for(int i = 0; i < Constants.ElevatorConfig.neoConfigs.length; i++) {
            motors[i] = new Neo(Constants.ElevatorConfig.neoConfigs[i]);
        }

        findHome(); // Shuold automatically start once enabled.
    }

    /**
     * Find the home position for the elevator by moving down until the limit switch is activated
     */
    public void findHome() {
        switch (homingState) {
            case UNKNWON:   // Fallthrough...
            case COMPLETE:
                homingState = HomingState.MOVING_OFF_OF_LIMIT_SWITCH;
                break;
            case MOVING_OFF_OF_LIMIT_SWITCH:
            case MOVING_ON_TO_LIMIT_SWITCH:
            default:
                break;
        }
    }

    /**
     * Get if the elevator home switch is activated
     * 
     * @return true if the switch is active, otherwise false
     */
    public boolean isHome() {
        return homeLimitSwitch.get();
    }

    /**
     * Get the current position of the elevator in inches
     */
    public double getPositionInches() {
        double position_revs = motors[0].getPosition();

        return position_revs * Constants.ElevatorConfig.final_inches_per_motor_rev;
    }

    /**
     * Moves the elevator at a constant speed
     * 
     * @param   speedIps    Speed to move at in Inches Per Second
     */
    public void moveAtSpeed(double speedIps) {
        double speedRps = speedIps / Constants.ElevatorConfig.final_inches_per_motor_rev;
        motors[0].rotateAtSpeed(speedRps);

        if (Math.abs(speedRps) < 0.01) {
            resetElevator();
            moveToPosition(getPositionInches());
        }
        else {
            targetPositionInches = OptionalDouble.empty();
        }
    }

    /**
     * Move to a specific position in inches
     * 
     * @param   inches  Position to move to in inches
     */
    public void moveToPosition(double inches) {
        if (homingState != HomingState.COMPLETE)
        {
            targetPositionRequestWhileHoming = OptionalDouble.of(inches);
            return;
        }

        if (targetPositionInches.isEmpty()) {
            pidController.reset(getPositionInches());
        }

        targetPositionInches = OptionalDouble.of(inches);

    }

    public void resetElevator(){
        targetPositionInches = OptionalDouble.empty();
    }

    /**
     * Called periodically to update information
     */
    @Override
    public void periodic() {
        double currentPosition = this.getPositionInches();
        double currentVelocityRps = motors[0].getVelocity();
        double currentVelocityIpsps = currentVelocityRps * Constants.ElevatorConfig.final_inches_per_motor_rev;

    ElasticDashboard.putBoolean("Elevator Position Known", homingState == HomingState.COMPLETE);
    ElasticDashboard.putNumber("Elevator Position (Inches)", currentPosition);
        // if (targetPositionInches.isPresent()) {
        //     SmartDashboard.putNumber("Elevator Target (Inches)", targetPositionInches.getAsDouble());
        // }

        // if (targetPositionRequestWhileHoming.isPresent()) {
        //     SmartDashboard.putNumber("Elevator request (homing)", targetPositionRequestWhileHoming.getAsDouble());
        // }
        // else {
        //     SmartDashboard.putNumber("Elevator request (homing)", -1000);
        // }

        //SmartDashboard.putNumber("Elevator Velocity (IPS)", currentVelocityIpsps);

        double motorCurrent = motors[0].getOutputCurrent();
        double filteredMotorCurrent = motorCurrentDrawMedianFilter.calculate(motorCurrent);

        //SmartDashboard.putNumber("Elevator Filtered Ams", filteredMotorCurrent);
        switch (homingState) {
            case UNKNWON:
                break;
            case MOVING_OFF_OF_LIMIT_SWITCH:
                homingState = HomingState.MOVING_ON_TO_LIMIT_SWITCH;
                break;
            case MOVING_ON_TO_LIMIT_SWITCH:
                motors[0].setVoltage(-Constants.ElevatorConfig.homingVoltage);
                if (filteredMotorCurrent > Constants.ElevatorConfig.homingCurrentThresholdAmps) {
                    motors[0].setPosition(Constants.ElevatorConfig.homePositionInches / Constants.ElevatorConfig.final_inches_per_motor_rev);
                    resetElevator();

                    double target = getPositionInches();
                    if (targetPositionRequestWhileHoming.isPresent())
                    {
                        target = targetPositionRequestWhileHoming.getAsDouble();
                        targetPositionRequestWhileHoming = OptionalDouble.empty();
                    }

                    homingState = HomingState.COMPLETE;
                    moveToPosition(target);
                }
                break;
            case COMPLETE:
                break;
            default:
                break;
        }

        if (HomingState.COMPLETE != homingState) {
            return;
        }

        if (targetPositionInches.isPresent()) {
            double targetInches = targetPositionInches.getAsDouble();

            if (targetInches < Constants.ElevatorConfig.minPositionInches) {
                targetInches = Constants.ElevatorConfig.minPositionInches;
                targetPositionInches = OptionalDouble.of(targetInches);
            }
    
            if (targetInches > Constants.ElevatorConfig.maxPositionInches) {
                targetInches = Constants.ElevatorConfig.maxPositionInches;
                targetPositionInches = OptionalDouble.of(targetInches);
            }

            double ff = feedforward.calculateWithVelocities(currentVelocityIpsps, pidController.getSetpoint().velocity);
            double feedback = pidController.calculate(currentPosition, targetInches);
            double voltage = Math2264.limitMagnitude(ff + feedback, Constants.ElevatorConfig.maxVoltage);

        // ElasticDashboard.putNumber("Elevator FF", ff);
        // ElasticDashboard.putNumber("Elevator Feedback", feedback);
        // ElasticDashboard.putNumber("Elevator Voltage", voltage);

            motors[0].setVoltage(voltage);
        }
    }
}
