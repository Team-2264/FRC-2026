package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;

/**
 * AlignToShoot - Rotation-only alignment.
 * Turns the robot so the visible tag is dead-center in the camera.
 * Press button once — rotates until aligned, then auto-stops.
 */
public class AlignToShoot extends Command {

    private final Swerve swerve;
    private final Vision2025 vision;

    // Proportional gain for rotation
    private static final double ROT_KP = 2.5;

    // Max rotation speed (rad/s)
    private static final double MAX_ROT = 1.5;

    // "Good enough" threshold
    private static final double ANGLE_TOLERANCE = 2.0;   // degrees
    
    // Track if we're done
    private boolean isAligned = false;
    private int noTagCount = 0;

    public AlignToShoot(Swerve swerve, Vision2025 vision) {
        this.swerve = swerve;
        this.vision = vision;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        isAligned = false;
        noTagCount = 0;
        swerve.setFieldRelative(false);  // robot-relative for camera alignment
        SmartDashboard.putBoolean("AlignToShoot/Active", true);
        SmartDashboard.putBoolean("AlignToShoot/Aligned", false);
    }

    @Override
    public void execute() {
        Optional<Transform3d> tf = vision.getTargetTransform();

        if (tf.isEmpty()) {
            // Can't see a tag — hold still, count how long
            swerve.drive(new Translation2d(0, 0), 0);
            SmartDashboard.putString("AlignToShoot/Status", "NO TAG");
            noTagCount++;
            // If no tag for 50 cycles (~1 sec), give up
            if (noTagCount > 50) {
                isAligned = true;  // End the command
            }
            return;
        }
        
        noTagCount = 0;  // Reset counter when we see a tag

        Transform3d transform = tf.get();
        double tagY = transform.getY();   // lateral offset (+ = tag is LEFT of center)
        double tagX = transform.getX();   // forward distance
        
        // Angle to tag: positive means tag is to the LEFT
        double angleDeg = Math.toDegrees(Math.atan2(tagY, tagX));

        // Check if we're aligned
        if (Math.abs(angleDeg) < ANGLE_TOLERANCE) {
            isAligned = true;
            swerve.drive(new Translation2d(0, 0), 0);
            SmartDashboard.putString("AlignToShoot/Status", "ALIGNED!");
            SmartDashboard.putBoolean("AlignToShoot/Aligned", true);
            return;
        }

        // Rotation: if tag is LEFT (positive angleDeg), we need to rotate LEFT (positive rotation)
        // to bring the tag to center. Try POSITIVE sign first.
        double rot = angleDeg * ROT_KP * (Math.PI / 180.0);
        rot = clamp(rot, -MAX_ROT, MAX_ROT);

        swerve.drive(new Translation2d(0, 0), rot);
        SmartDashboard.putString("AlignToShoot/Status", "TURNING");
        SmartDashboard.putNumber("AlignToShoot/AngleDeg", angleDeg);
        SmartDashboard.putNumber("AlignToShoot/Rotation", rot);
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    @Override
    public boolean isFinished() {
        return isAligned;  // Finish when aligned (or gave up)
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0);
        swerve.setFieldRelative(true);
        SmartDashboard.putBoolean("AlignToShoot/Active", false);
        SmartDashboard.putString("AlignToShoot/Status", interrupted ? "CANCELLED" : "DONE");
    }
}
