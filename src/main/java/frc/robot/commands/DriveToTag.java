package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionOld;

/**
 * Command to drive the motion to a specific distance from the tag.
 * 
 */
public class DriveToTag extends Command{

    private final Swerve swerve;
    private final VisionOld vision;
    private Pose2d targetPose;
    private Pose2d tagPose;
    //private Optional<Pose2d> tagPose;

    /*
     * This code is not finalized.
     * For autoaligning based on robot translation, take a look at the translation_based_autoalign branch.
     */

    public DriveToTag(Swerve swerve, VisionOld vision){
        this.swerve = swerve;
        this.vision = vision;
        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        //swerve.setGyro();
        if (vision.targetIds.length>0){
            int closestID = vision.targetIds[0];
            targetPose = new Pose2d(0.1, 0.05, new Rotation2d(0.0)); //vision.rightReefPose(closestID);
            //tagPose = vision.tagPose(closestID);
            //Translation2d rotationLock = tagPose.getTranslation().
        } else {
            targetPose = null;
        }


        // swerve.lockOnto(Translation2d)
    }

    @Override
    public void execute(){
        //swerve.setGyro();
        swerve.setFieldRelative(false);

        Optional<Transform3d> tagTransform = vision.getTargetTransform();
        if (tagTransform.isPresent()) {
            Transform3d xform = tagTransform.get();
            //Pose2d tagPose = Pose2d(Meters.of(xform.getMeasureX()), xform.getMeasureY());
            //Pose2d currentPose = swerve.getPose();
            Translation2d error= targetPose.getTranslation().minus(tagPose.getTranslation());
            Translation2d driveVector = error.times(1.0);
            swerve.drive(driveVector, 0);

            SmartDashboard.putString("Move to tag err", error.toString());
        }
    }

    @Override
    public boolean isFinished(){
        if (vision.targetIds.length == 0) {
            return true;
        }

        Pose2d currentPose = swerve.getPose();
        double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());

        return distance < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0,0), 0);
        swerve.setFieldRelative(true);
        swerve.zeroGyro();
        //swerve.unlock();
    }
    
}