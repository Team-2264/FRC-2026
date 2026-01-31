package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;
import frc.robot.Constants;

import java.util.Optional;

/**
 * Command to drive the motion to a specific distance from the tag.
 * 
 */
public class DriveToTag extends Command{

    private final Swerve swerve;
    private final Vision2025 vision;
    private Optional<Pose2d> targetPose = Optional.empty();
    private final Optional<Integer> targetTagId;
    private final Transform2d tagOffset;
    //private Optional<Pose2d> tagPose;

    /*
     * This code is not finalized.
     * For autoaligning based on robot translation, take a look at the translation_based_autoalign branch.
     */

    public DriveToTag(Swerve swerve, Vision2025 vision){
        this(swerve, vision, Optional.empty(),
            new Transform2d(
                new Translation2d(-Units.inchesToMeters(Constants.gamePieces.reefDistance), 0.0),
                new Rotation2d()
            )
        );
    }

    public DriveToTag(Swerve swerve, Vision2025 vision, Optional<Integer> targetTagId, Transform2d tagOffset){
        this.swerve = swerve;
        this.vision = vision;
        this.targetTagId = targetTagId;
        this.tagOffset = tagOffset;
        addRequirements(swerve);
    }

    @Override
    public void initialize(){
        swerve.setFieldRelative(true);
        Optional<Pose2d> targetTagPose = targetTagId.isPresent()
            ? vision.tagPose(targetTagId.get())
            : vision.getBestTagPose2d();

        targetPose = targetTagPose.map(pose -> pose.transformBy(tagOffset));
    }

    @Override
    public void execute(){
        if (targetPose.isPresent()) {
            swerve.driveToPose(targetPose.get());
            Translation2d error = targetPose.get().getTranslation().minus(swerve.getPose().getTranslation());
            SmartDashboard.putString("Move to tag err", error.toString());
        }
    }

    @Override
    public boolean isFinished(){
        if (targetPose.isEmpty()) {
            return true;
        }

        Pose2d currentPose = swerve.getPose();
        double distance = currentPose.getTranslation().getDistance(targetPose.get().getTranslation());

        return distance < 0.1;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0,0), 0);
        swerve.setFieldRelative(true);
        //swerve.unlock();
    }
    
}