package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;

/**
 * Command to rotate the robot to face a specific tag.
 * 
 */

public class FaceTag extends Command {
    private final Swerve swerve;
    private final Vision2025 vision;
    private final int tagId;

    public FaceTag(Swerve swerve, Vision2025 vision, int tagId){
        this.swerve = swerve;
        this.vision = vision;
        this.tagId = tagId;

    }

    @Override
    public void initialize(){
        // Guard against null or empty targetIds array to prevent crash
        if (vision.targetIds == null || vision.targetIds.length == 0 || tagId >= vision.targetIds.length) {
            return;  // No targets visible, do nothing
        }
        
        Translation2d tagtranslation = switch(vision.targetIds[tagId]){ //vision.targetIds[vision.getClosestTarget()]
            // 2026: tags 6, 7 are now coral station tags (in TagLocation)
            case 6 -> Constants.TagLocation.tag6.getWpiBlue().getTranslation().toTranslation2d();
            case 7 -> Constants.TagLocation.tag7.getWpiBlue().getTranslation().toTranslation2d();
            // Reef tags remain in ReefTagLocation
            case 8 -> Constants.ReefTagLocation.tag8.getWpiBlue().getTranslation().toTranslation2d();
            case 9 -> Constants.ReefTagLocation.tag9.getWpiBlue().getTranslation().toTranslation2d();
            case 10 -> Constants.ReefTagLocation.tag10.getWpiBlue().getTranslation().toTranslation2d();
            case 11 -> Constants.ReefTagLocation.tag11.getWpiBlue().getTranslation().toTranslation2d();

            // 2026: tags 17, 22 are now coral station tags (in TagLocation)
            case 17 -> Constants.TagLocation.tag17.getWpiBlue().getTranslation().toTranslation2d();
            case 18 -> Constants.ReefTagLocation.tag18.getWpiBlue().getTranslation().toTranslation2d();
            case 19 -> Constants.ReefTagLocation.tag19.getWpiBlue().getTranslation().toTranslation2d();
            case 20 -> Constants.ReefTagLocation.tag20.getWpiBlue().getTranslation().toTranslation2d();
            case 21 -> Constants.ReefTagLocation.tag21.getWpiBlue().getTranslation().toTranslation2d();
            case 22 -> Constants.TagLocation.tag22.getWpiBlue().getTranslation().toTranslation2d();


            default -> null;
        };
        swerve.lockOnto(tagtranslation);
    }


    @Override
    public boolean isFinished() {
        return true;

    }
}
