package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/**
 * Command for releasing coral from the end effector.
 * 
 */
public class RetractAndStopIntake extends Command {
    private static final double STOP_ROLLER_DELAY_TIME = 1.5;
    
    private double m_startTime;
    private boolean m_finished = false;
    private Intake m_intake;

    public RetractAndStopIntake(Intake intake) {
        m_intake = intake;
    }

    @Override
    public void initialize() {
        m_startTime = Timer.getFPGATimestamp();
        m_finished = false;

        m_intake.retractIntake();
    }

    @Override
    public void execute() {
        if ((Timer.getFPGATimestamp() - m_startTime) > STOP_ROLLER_DELAY_TIME)
        {
            m_intake.stopRoller();
            m_finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
