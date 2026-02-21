package frc.robot.subsystems;


import edu.wpi.first.math.filter.MedianFilter;
import frc.robot.ElasticDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motors.Neo;
import frc.robot.Constants;

/**
 * Subsystem that controls the Algae End Effector.
 * 
 */
public class AlgaeManipulator extends SubsystemBase {

    private Neo motor;

    /**
     * Enum for algae end effector status.
     * 
     */
    enum algaeManipulatorState {
        IDLE, INTAKE, LOADED, OUTTAKE
    }

    private algaeManipulatorState currentState = algaeManipulatorState.IDLE;
    private double targetMotorDutyCycle = 0.0;

    private boolean algaePresent = false;

    // Should be updated every ~20ms, so this will be be a 500mS filter
    private MedianFilter motorCurrentMedianFilter = new MedianFilter(25);

    /**
     * Constructs a new AlgaeManipulator instance.
     * 
     */
    public AlgaeManipulator() {
        motor = new Neo(Constants.AlgaeConfig.neoConfigs);

    }

    /**
     * Starts the intake.
     * 
     */
    public void intake() {
        currentState = algaeManipulatorState.INTAKE;
    }
    
    /**
     * Stops the Algae Motors.
     * 
     */
    public void stop() {
        if (algaePresent) {
            currentState = algaeManipulatorState.LOADED;
        }
        else {
            currentState = algaeManipulatorState.IDLE;
        }
    }
    
    /**
     * Reverses the intake. Referred to as "outtaking".
     * 
     */
    public void outtake() {
        currentState = algaeManipulatorState.OUTTAKE;
    }

    @Override
    public void periodic() {
        double motorCurrent = motor.getOutputCurrent();
        double motorFilteredCurrent = motorCurrentMedianFilter.calculate(motorCurrent);

        // motor actions based on desired state
        switch (currentState)
        {
            case INTAKE:
                targetMotorDutyCycle = Constants.AlgaeConfig.intakeDuty;

                if (Math.abs(motorFilteredCurrent) > Constants.AlgaeConfig.motorIntakeStallThresholdAmps) {
                    algaePresent = true;
                    currentState = algaeManipulatorState.LOADED;
                }
                break;
            case LOADED:
                targetMotorDutyCycle = Constants.AlgaeConfig.holdDuty;
                break;
            case OUTTAKE:
                algaePresent = false;
                targetMotorDutyCycle = Constants.AlgaeConfig.outtakeDuty;
                break;
            default:
            case IDLE:
                algaePresent = false;
                targetMotorDutyCycle = 0;
                break;
        }

        motor.rotateAtSpeed(targetMotorDutyCycle);

    ElasticDashboard.putBoolean("Algae Present", algaePresent);
    ElasticDashboard.putNumber("Aglae Intake Current", motorFilteredCurrent);
    ElasticDashboard.putNumber("Aglae Intake Duty", targetMotorDutyCycle);
    }

}
    