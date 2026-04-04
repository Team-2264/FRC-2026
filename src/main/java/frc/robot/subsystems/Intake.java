package frc.robot.subsystems;

import java.util.OptionalDouble;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Math2264;
import frc.lib.motors.Neo;

import frc.robot.Constants;
import frc.robot.ElasticDashboard;

public class Intake extends SubsystemBase {
    private Neo rollerMotor;
    private Neo deployMotor;

    private ProfiledPIDController deployPidController;
    private SimpleMotorFeedforward deployFeedforward;

    /**
     * Enum for the Intake Rotation homing status.
     * 
     */
    private enum HomingState {
        UNKNWON,
        MOVING_OFF_OF_LIMIT,
        MOVING_ON_TO_LIMIT,
        COMPLETE,
    }

    private HomingState deployHomingState = HomingState.UNKNWON;
    private OptionalDouble deployTargetPositionRevs = OptionalDouble.empty();
    private OptionalDouble deployTargetPositionRequestWhileHoming = OptionalDouble.empty();

    private MedianFilter deployMotorCurrentDrawMedianFilter = new MedianFilter(25);

    public Intake() {
        rollerMotor = new Neo(Constants.IntakeConfig.Roller.neoConfig);
        deployMotor = new Neo(Constants.IntakeConfig.Deploy.neoConfig);

        this.deployFeedforward = new SimpleMotorFeedforward(Constants.IntakeConfig.Deploy.feedForwardKs, 
                                                            Constants.IntakeConfig.Deploy.feedForwardKv,
                                                            Constants.IntakeConfig.Deploy.feedForwardKa);

        this.deployPidController = new ProfiledPIDController(Constants.IntakeConfig.Deploy.pidKp,
                                                       Constants.IntakeConfig.Deploy.pidKi,
                                                       Constants.IntakeConfig.Deploy.pidKd,
                                                       new Constraints(Constants.IntakeConfig.Deploy.maxRevsPerSecond, 
                                                                       Constants.IntakeConfig.Deploy.accelRevsPerSecondPerSecond));

        deployFindHome();
    }

       /**
     * Find the home position for the elevator by moving down until the limit switch is activated
     */
    public void deployFindHome() {
        switch (deployHomingState) {
            case UNKNWON:   // Fallthrough...
            case COMPLETE:
                deployHomingState = HomingState.MOVING_OFF_OF_LIMIT;
                break;
            case MOVING_OFF_OF_LIMIT:
            case MOVING_ON_TO_LIMIT:
            default:
                break;
        }
    }

    public void deployIntake() {
        if (deployHomingState != HomingState.COMPLETE)
        {
            deployTargetPositionRequestWhileHoming = OptionalDouble.of(Constants.IntakeConfig.Deploy.extendedPosition);
            return;
        }

        if (deployTargetPositionRevs.isEmpty())
        {
            deployPidController.reset(deployGetPosition());
        }

        deployTargetPositionRevs = OptionalDouble.of(Constants.IntakeConfig.Deploy.extendedPosition);
    }

    public void retractIntake() {
        if (deployHomingState != HomingState.COMPLETE)
        {
            deployTargetPositionRequestWhileHoming = OptionalDouble.of(Constants.IntakeConfig.Deploy.retractedPosition);
            return;
        }

        if (deployTargetPositionRevs.isEmpty())
        {
            deployPidController.reset(deployGetPosition());
        }

        deployTargetPositionRevs = OptionalDouble.of(Constants.IntakeConfig.Deploy.retractedPosition);
    }

    public double deployGetPosition() {
        return deployMotor.getPosition();
    }

    public void deployMoveToPosition(double position) {

    }

    public void resetIntake(){
        deployTargetPositionRevs = OptionalDouble.empty();
    }

    public void deployManualMove(double voltage) {
        deployMotor.setVoltage(voltage);
    }

    public void startRoller(double dutycycle) {
        rollerMotor.rotateAtSpeed(dutycycle);
    }

    public void stopRoller() {
        rollerMotor.rotateAtSpeed(0);
    }

    /**
     * Called periodically to update information
     */
    @Override
    public void periodic() {
        double currentDeployPosition = this.deployGetPosition();

        ElasticDashboard.putBoolean("Intake/Deploy/PositionKnown", deployHomingState == HomingState.COMPLETE);
        ElasticDashboard.putNumber("Intake/Deploy/Position", currentDeployPosition);


        double deployMotorCurrent = deployMotor.getOutputCurrent();
        double filteredMotorCurrent = deployMotorCurrentDrawMedianFilter.calculate(deployMotorCurrent);

        ElasticDashboard.putNumber("Intake/Deploy/MotorCurrent", filteredMotorCurrent);

        switch (deployHomingState) {
            case UNKNWON:
                break;
            case MOVING_OFF_OF_LIMIT:
                deployHomingState = HomingState.MOVING_ON_TO_LIMIT;
                break;
            case MOVING_ON_TO_LIMIT:
                deployMotor.setVoltage(-Constants.IntakeConfig.Deploy.homingVoltage);
                if (filteredMotorCurrent > Constants.IntakeConfig.Deploy.homingCurrentThresholdAmps) {
                    deployMotor.setPosition(Constants.IntakeConfig.Deploy.homePositionRevs / Constants.IntakeConfig.Deploy.gearRatio);
                    resetIntake();

                    double target = deployGetPosition();
                    if (deployTargetPositionRequestWhileHoming.isPresent())
                    {
                        target = deployTargetPositionRequestWhileHoming.getAsDouble();
                        deployTargetPositionRequestWhileHoming = OptionalDouble.empty();
                    }

                    deployHomingState = HomingState.COMPLETE;
                    deployMoveToPosition(target);
                }
                break;
            case COMPLETE:
                break;
            default:
                break;
        }

        if (HomingState.COMPLETE != deployHomingState) {
            return;
        }

        if (deployTargetPositionRevs.isPresent()) {
            double targetRevs = deployTargetPositionRevs.getAsDouble();

            if (targetRevs < Constants.IntakeConfig.Deploy.minAngle) {
                targetRevs = Constants.IntakeConfig.Deploy.minAngle;
                deployTargetPositionRevs = OptionalDouble.of(targetRevs);
            }
    
            if (targetRevs > Constants.IntakeConfig.Deploy.maxAngle) {
                targetRevs = Constants.IntakeConfig.Deploy.maxAngle;
                deployTargetPositionRevs = OptionalDouble.of(targetRevs);
            }

            double ff = deployFeedforward.calculate(targetRevs * 2 * Math.PI, Constants.IntakeConfig.Deploy.maxRevsPerSecond);
            double feedback = deployPidController.calculate(currentDeployPosition, targetRevs);
            double voltage = Math2264.limitMagnitude(ff + feedback, Constants.IntakeConfig.Deploy.maxVoltage);
            deployMotor.setVoltage(voltage);

            ElasticDashboard.putNumber("Intake/Deploy/DriveVoltage", voltage);
            ElasticDashboard.putBoolean("Intake/Deploy/Extended", currentDeployPosition>3.0);
        }
    }

}
