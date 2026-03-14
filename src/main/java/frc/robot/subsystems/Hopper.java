package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motors.Neo;

import frc.robot.Constants;

public class Hopper extends SubsystemBase {
    private Neo conveyorMotor;

    public Hopper() {
        conveyorMotor = new Neo(Constants.HopperConveyorConfig.neoConfig);
    }

    public void startConveyor(double dutycycle) {
        conveyorMotor.rotateAtSpeed(dutycycle);
    }

    public void stopConveyor() {
        conveyorMotor.rotateAtSpeed(0);
    }

}
