package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CoralArm;

/**
 * Command for releasing coral from the end effector.
 * 
 */
public class ReleaseCoral extends Command {
    private double m_startTime;
    private boolean m_finished = false;
    private CoralArm m_coralArm;

    public ReleaseCoral(CoralArm coralArm) {
        m_coralArm = coralArm;
    }

    @Override
    public void initialize() {
        m_startTime = Timer.getFPGATimestamp();
        m_finished = false;

        m_coralArm.releaseCoral(Constants.CoralConfig.intakeMotorReleaseDuty);
    }

    @Override
    public void execute() {
        if ((Timer.getFPGATimestamp() - m_startTime) > 0.5)
        {
            m_coralArm.rotateTo(Constants.CoralConfig.angleIntake);
            m_coralArm.stopIntake();
            m_finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
