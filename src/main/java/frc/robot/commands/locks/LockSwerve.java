package frc.robot.commands.locks;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;


public class LockSwerve extends Command {
    private final Swerve swerve;

    public LockSwerve(Swerve swerve){
        this.swerve = swerve;    

    }

    @Override
    public void initialize() {
        Optional<Alliance> alliance2 = DriverStation.getAlliance();
        if(alliance2.isEmpty()) {
            throw new RuntimeException("Failed to get alliance: Alliance is not set in Driverstation");
        }

        // Alliance alliance = alliance2.get();
        // Translation2d speakerTranslation = switch (alliance) {
        //     // TODO: Update this to account for the reef having multiple tags. NMP 1/29/2025
        //     case Blue -> Constants.Targeting.BlueReefTag1.getWpiBlue().getTranslation().toTranslation2d();
        //     case Red -> Constants.Targeting.RedReefTag1.getWpiBlue().getTranslation().toTranslation2d();

        // };
        
        // swerve.lockOnto(speakerTranslation);

    }

    @Override
    public boolean isFinished() {
        return true;

    }

}
