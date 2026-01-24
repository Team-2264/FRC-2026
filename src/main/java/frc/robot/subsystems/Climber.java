package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motors.Neo;
import frc.robot.Constants;

/**
 * Subsystem for controling the climber.
 * 
 */
public class Climber extends SubsystemBase {
    private Neo motor;

    private boolean deployed = false;

    /**
     * Constructs a new Climber instance.
     * 
     */
    public Climber (){
        motor = new Neo(Constants.ClimberConfig.neoConfig);
    }

    /**
     * Moves the climber up.
     */
    public void moveUp(){
        motor.rotateAtSpeed(Constants.ClimberConfig.moveUpDrive);
    }

    /**
     * Moves the climber down.
     */
    public void moveDown(){
        motor.rotateAtSpeed(Constants.ClimberConfig.moveDownDrive);
    }

    /**
     * set the deployed status to true.
     */
    public void setDeployed(){
        deployed = true;
    }

    /**
     * 
     * @return if the climber is deployed
     */
    public boolean isDeployed(){
        return deployed;
    }

    /**
     * stop the climber motor
     */
    public void stop(){
        motor.stop();
    }
}
