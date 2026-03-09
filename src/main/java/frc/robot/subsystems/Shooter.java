package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.motors.Neo;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private Neo shooterMotor;
    private Neo feedMotor;
    private Neo conveyorMotor;

    public Shooter() {
        shooterMotor = new Neo(Constants.ShooterConfig.Shooter.neoConfig);
        feedMotor = new Neo(Constants.ShooterConfig.Feeder.neoConfig);
        conveyorMotor = new Neo(Constants.HopperConveyorConfig.neoConfig);
    }

    public void startShooting() {
        conveyorMotor.rotateAtSpeed(0.75);
        shooterMotor.rotateAtSpeed(0.6);
        feedMotor.rotateAtSpeed(0.6);
    }

    public void stopShooting() {
        conveyorMotor.rotateAtSpeed(0);
        shooterMotor.rotateAtSpeed(0);
        feedMotor.rotateAtSpeed(0);
    }
}