package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motors.Neo;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private Neo rollerMotor;
    private Neo deployMotor;

    public Intake() {
        rollerMotor = new Neo(Constants.IntakeConfig.Roller.neoConfig);
        deployMotor = new Neo(Constants.IntakeConfig.Deploy.neoConfig);
    }

    public void startRoller(double dutycycle) {
        rollerMotor.rotateAtSpeed(dutycycle);
    }

    public void stopRoller() {
        rollerMotor.rotateAtSpeed(0);
    }

}
