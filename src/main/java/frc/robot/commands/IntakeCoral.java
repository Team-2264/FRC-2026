package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.enums.CoralIntakeStatus;
import frc.robot.subsystems.CoralArm;

/**
 * Command for intaking coral.
 * 
 */
public class IntakeCoral extends Command {
    private final CoralArm coralArm;
    private double startTime;

    public IntakeCoral (CoralArm coralArm){
        this.coralArm = coralArm;

    }

    @Override
    public void initialize() {

        startTime = Timer.getFPGATimestamp();
        coralArm.intakeCoral();
        coralArm.setCoralIntakeStatus(CoralIntakeStatus.INTAKING);
        
    }

    @Override
    public void end(boolean interrupted){
        coralArm.stopIntake();
    }


    @Override
    public boolean isFinished() {
        return (Timer.getFPGATimestamp() - startTime)> 0.5;
    }
    
}
