package frc.robot.subsystems;

import java.util.Optional;
import java.util.OptionalDouble;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.ElasticDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Math2264;
import frc.lib.motors.Neo;
import frc.robot.RobotContainer;
import frc.robot.enums.CoralIntakeStatus;
import frc.robot.enums.CoralShoulderStatus;
import frc.robot.Constants;

/**
 * Subsystem for controlling the Coral Arm. Controls both the End Effector and Shoulder.
 * 
 */
public class CoralArm extends SubsystemBase{

    // End Effector
    private Neo intakeMotor;
    private CoralIntakeStatus coralIntakeStatus;

    // Shoulder
    private Neo angleMotor;
    private OptionalDouble desiredAngle = OptionalDouble.empty();
    private CoralShoulderStatus coralAngleStatus;

    private ProfiledPIDController pidController;
    private ArmFeedforward feedforward;

    private boolean coralPresent = false;

    // Should be updated every ~20ms, so this will be be a 500mS filter
    private MedianFilter intakeMotorCurrentMedianFilter = new MedianFilter(15);


    /**
     * Constructs a new CoralIntake instance.
     * 
     */
    public CoralArm(RobotContainer container) {
        intakeMotor = new Neo(Constants.CoralConfig.intakeNeoConfig);
        angleMotor = new Neo(Constants.CoralConfig.angleNeoConfig);

        this.feedforward = new ArmFeedforward(Constants.CoralConfig.angleFeedForwardKs, 
                                              Constants.CoralConfig.angleFeedForwardKg, 
                                              Constants.CoralConfig.angleFeedForwardKv);

        this.pidController = new ProfiledPIDController(Constants.CoralConfig.anglePidKp,
                                                       Constants.CoralConfig.anglePidKi,
                                                       Constants.CoralConfig.anglePidKd,
                                                       new Constraints(Constants.CoralConfig.angleMaxDegreesPerSecond, 
                                                                       Constants.CoralConfig.angleAccelDegreesPerSecondPerSecond));
        
        coralAngleStatus = CoralShoulderStatus.HOME;

        angleMotor.setPosition(0);
        setMotorPositionDegrees();

        // Coral should be present at startup since we start with a piece loaded.
        coralPresent = true;
    }

    /**
     * intake the coral to the End Effector.
     */
    public void intakeCoral(){
        intakeMotor.rotateAtSpeed(Constants.CoralConfig.intakeMotorIntakeDuty);
        coralIntakeStatus = CoralIntakeStatus.INTAKING;
    }

    /**
     * Release the coral from the End Effector.
     * 
     * @param dutyCycle
     */
    public void releaseCoral(double dutyCycle){
        coralIntakeStatus = CoralIntakeStatus.RELEASING;
        intakeMotor.rotateAtSpeed(dutyCycle);
        coralPresent = false;
    }

    /**
     * Stops the intake motor.
     */
    public void stopIntake() {
        intakeMotor.stop();
        coralIntakeStatus = CoralIntakeStatus.STOPPED;
    }

    /**
     * Set the intake status of the coral End Effector (enum).
     * 
     * @param status intake status
     */
    public void setCoralIntakeStatus(CoralIntakeStatus status){
        this.coralIntakeStatus = status;
    }

    /**
     * 
     * @return if coral is present
     */
    public boolean isCoralPresent() {
        return coralPresent;
    }



    /**
     * Gets the angle of the shoulder relative to its bottom position.
     * 0 is pointing directly toward ground, 90 is horozontal, 180 is directly up
     * 
     * @return The angle of the shoulder in degrees.
     */
    public double getAbsEncoderDegrees() {
        double encoderRaw = angleMotor.getAbsEncoderPosition();
        double encoderPos = encoderRaw + Constants.CoralConfig.angleEncoderOffsetZeroToOne;
        double relDegrees = encoderPos * 360.0;
        double wrappedDegrees = relDegrees;
        if (relDegrees > 180.0) {
            wrappedDegrees -= 360.0;
        }
        else if (relDegrees < -180.0) {
            wrappedDegrees += 360.0;
        }

        // SmartDashboard.putNumber("Coral Intake Angle (direct)", encoderRaw);
        // SmartDashboard.putNumber("Coral Intake Angle (enc)", encoderPos);
        // SmartDashboard.putNumber("Coral Intake Angle (rel)", relDegrees);
        
        return wrappedDegrees;
    }

    /**
     * Get the motor position in 
     * 
     * @return motor position
     */
    public double getMotorPositonDegrees()
    {
        return (angleMotor.getPosition() * 360.0) / Constants.CoralConfig.angleGearRatio;
    }

    /**
     * Set the position of the angle motor. Should only be called at startup or when
     * rehoming the angle.
     */
    public void setMotorPositionDegrees() {
        angleMotor.setPosition((getAbsEncoderDegrees() / 360) * Constants.CoralConfig.angleGearRatio);
    }


    /**
     * Sets the angle that we aim to rotate to.
     * Does not trigger actual rotation.
     * Angle in rotations (not degrees)
     * 
     * @param angle The angle that we aim to rotate to.
     */
    public void rotateTo(double angle) {
        if(desiredAngle.isEmpty()) {
            pidController.reset(getAbsEncoderDegrees());
        }

        desiredAngle = OptionalDouble.of(angle);
    ElasticDashboard.putNumber("coral angle target", desiredAngle.getAsDouble());
    }

    
    /**
     * Rotates the coral arm using open-loop voltage control.
     * Resets PID if voltage is near zero.
     *
     * @param volts voltage
     */
    public void rotateOpenLoop(double volts) {
        angleMotor.setVoltage(volts);
    ElasticDashboard.putNumber("coral angle volts", volts);

        if (Math.abs(volts) < 0.01) {
            pidController.reset(getAbsEncoderDegrees());
            rotateTo(getAbsEncoderDegrees());
        }
        else {
            desiredAngle = OptionalDouble.empty();
        }
    }

    /**
     * Sets the state of the coral arm shoulder.
     * Rotates the shoulder to a given angle.
     * 
     * @param state
     */
    public void setShoulderState(CoralShoulderStatus status) {
        // if (status != coralAngleStatus){
        //     this.coralAngleStatus = status;
        //     rotateTo(status.shoulderAngle());
        // }
    }

    /**
     * 
     * @return shoulderStatus enum
     */
    public CoralShoulderStatus getShoulderStatus(){
        return coralAngleStatus;
    }

    /**
     * 
     * @return intakeStatus enum
     */
    public CoralIntakeStatus getIntakeStatus(){
        return coralIntakeStatus;
    }

    public void resetShoulderStatus(){
        //desiredAngle = OptionalDouble.empty();
    }

    @Override
    public void periodic(){
        // End Effector
        //SmartDashboard.putString("Coral Intake Status: ", coralIntakeStatus.toString());

        double intakeMotorCurrent = intakeMotor.getOutputCurrent();
        double intakeMotorFilteredCurrent = intakeMotorCurrentMedianFilter.calculate(intakeMotorCurrent);

        // shoulder
        double currentAngle = this.getAbsEncoderDegrees();
        //double currentMotorDegrees = this.getMotorPositonDegrees();

    ElasticDashboard.putNumber("Coral Intake Angle", currentAngle);
        //SmartDashboard.putNumber("Coral Intake pos (motor)", angleMotor.getPosition());
        //SmartDashboard.putNumber("Coral Intake pos (motor deg)", getMotorPositonDegrees());
        //SmartDashboard.putNumber("Coral Intake voltage", angleMotor.getAppliedOutput());

        // stop intaking if coral is present
        if (coralIntakeStatus == CoralIntakeStatus.INTAKING) {
            if (Math.abs(intakeMotorFilteredCurrent) > Constants.CoralConfig.intakeMotorIntakeStallThresholdAmps) {
                coralPresent = true;
                stopIntake();
            }
        }

        //SmartDashboard.putNumber("Coral End Effector Amps", intakeMotorCurrent);
        //SmartDashboard.putNumber("Coral End Effector Filtered Ams", intakeMotorFilteredCurrent);
    ElasticDashboard.putBoolean("Coral Present", isCoralPresent());

        double targetAngle = 1000;
        //double targetMotorDegrees = 1000;

        if(desiredAngle.isPresent()) {
            targetAngle = this.desiredAngle.getAsDouble();
            //targetMotorDegrees = targetAngle * Constants.CoralIntake.angleGearRatio;

            if (targetAngle < Constants.CoralConfig.angleMinAngle) {
                targetAngle = Constants.CoralConfig.angleMinAngle;
                desiredAngle = OptionalDouble.of(targetAngle);
            }
    
            if (targetAngle > Constants.CoralConfig.angleMaxAngle) {
                targetAngle = Constants.CoralConfig.angleMaxAngle;
                desiredAngle = OptionalDouble.of(targetAngle);
            }

            double ff = feedforward.calculate(targetAngle * 2 * Math.PI, Constants.CoralConfig.angleMaxDegreesPerSecond);
            double feedback = pidController.calculate(currentAngle, targetAngle);
            double voltage = Math2264.limitMagnitude(ff + feedback, Constants.CoralConfig.angleMaxVoltage);
            angleMotor.setVoltage(voltage);
        }

        //SmartDashboard.putNumber("Coral Intake Target", targetAngle);
    }
}
