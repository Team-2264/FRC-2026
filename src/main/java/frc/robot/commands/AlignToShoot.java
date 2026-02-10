package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;

/**
 * AlignToShoot - Strafe left from tag 12 until tag 9 or 10 is visible, then align to tag 10.
 * 
 * Only activates when robot sees tag 12. Strafes LEFT until camera picks up
 * tag 9 or 10, then rotates to center tag 10. Press X to start, X again to cancel.
 */
public class AlignToShoot extends Command {

    private final Swerve swerve;
    private final Vision2025 vision;

    // Strafe speed (m/s) — NEGATIVE = left in robot-relative
    private static final double STRAFE_SPEED = -0.3;

    // Rotation gains for fine alignment
    private static final double ROT_KP = 2.5;
    private static final double MAX_ROT = 1.5;
    private static final double ANGLE_TOLERANCE = 2.0; // degrees

    // Timeout (cycles at 50Hz)
    private static final int TIMEOUT_CYCLES = 250; // 5 seconds

    // State machine
    private enum AlignState { STRAFE_LEFT, ALIGN_TO_10, DONE }
    private AlignState state;
    private int cycleCount = 0;
    private int noTagCount = 0;

    public AlignToShoot(Swerve swerve, Vision2025 vision) {
        this.swerve = swerve;
        this.vision = vision;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        cycleCount = 0;
        noTagCount = 0;
        swerve.setFieldRelative(false); // Robot-relative so left is always left

        // Only start if we currently see tag 12
        Optional<Integer> tagId = vision.getBestTargetId();
        if (tagId.isPresent() && tagId.get() == 12) {
            state = AlignState.STRAFE_LEFT;
            SmartDashboard.putString("AlignToShoot/Status", "SEES 12 - STRAFING LEFT");
        } else {
            // Don't do anything if we don't see tag 12
            state = AlignState.DONE;
            SmartDashboard.putString("AlignToShoot/Status", "NO TAG 12 - ABORTED");
        }

        SmartDashboard.putBoolean("AlignToShoot/Active", true);
    }

    @Override
    public void execute() {
        cycleCount++;

        // Global timeout
        if (cycleCount > TIMEOUT_CYCLES) {
            state = AlignState.DONE;
            swerve.drive(new Translation2d(0, 0), 0);
            SmartDashboard.putString("AlignToShoot/Status", "TIMEOUT");
            return;
        }

        if (state == AlignState.STRAFE_LEFT) {
            // Check if we can see tag 9 or 10
            Optional<Integer> tagId = vision.getBestTargetId();
            boolean seesTarget = tagId.isPresent() && (tagId.get() == 9 || tagId.get() == 10);

            if (seesTarget) {
                // Found tag 9 or 10! Switch to alignment phase
                state = AlignState.ALIGN_TO_10;
                SmartDashboard.putString("AlignToShoot/Status", "FOUND TAG " + tagId.get() + " - ALIGNING");
            } else {
                // Keep strafing left
                swerve.drive(new Translation2d(0, STRAFE_SPEED), 0);
                SmartDashboard.putString("AlignToShoot/Status", "STRAFING LEFT");
            }
        } else if (state == AlignState.ALIGN_TO_10) {
            // Try to get transform specifically to tag 10
            Optional<Transform3d> tf = vision.getTransformToTag(10);

            if (tf.isEmpty()) {
                // Can't see tag 10 specifically — hold still briefly
                swerve.drive(new Translation2d(0, 0), 0);
                noTagCount++;
                SmartDashboard.putString("AlignToShoot/Status", "LOOKING FOR TAG 10");
                if (noTagCount > 50) {
                    state = AlignState.DONE; // Give up after ~1s
                }
                return;
            }

            noTagCount = 0;
            Transform3d transform = tf.get();
            double tagY = transform.getY();
            double tagX = transform.getX();
            double angleDeg = Math.toDegrees(Math.atan2(tagY, tagX));

            SmartDashboard.putNumber("AlignToShoot/AngleDeg", angleDeg);

            if (Math.abs(angleDeg) < ANGLE_TOLERANCE) {
                // Aligned!
                state = AlignState.DONE;
                swerve.drive(new Translation2d(0, 0), 0);
                SmartDashboard.putString("AlignToShoot/Status", "ALIGNED!");
            } else {
                // Rotate to center tag 10
                double rot = angleDeg * ROT_KP * (Math.PI / 180.0);
                rot = Math.max(-MAX_ROT, Math.min(MAX_ROT, rot));
                swerve.drive(new Translation2d(0, 0), rot);
                SmartDashboard.putString("AlignToShoot/Status", "ROTATING TO TAG 10");
            }
        }
    }

    @Override
    public boolean isFinished() {
        return state == AlignState.DONE;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0);
        swerve.setFieldRelative(true);
        SmartDashboard.putBoolean("AlignToShoot/Active", false);
        SmartDashboard.putString("AlignToShoot/Status", interrupted ? "CANCELLED" : "DONE");
    }
}
