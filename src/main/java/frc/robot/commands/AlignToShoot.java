package frc.robot.commands;

import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;

/**
 * AlignToShoot - Alliance-aware command to move from a coral station to a
 * shooting position aligned with the center reef tags.
 *
 * 2026 field layout:
 *   Red coral stations:  Tag 7  (Y=0.645, bottom)  Tag 12 (Y=7.425, top)
 *   Red center reef:     Tag 9  (Y=3.679)           Tag 10 (Y=4.035)
 *   Blue coral stations: Tag 17 (Y=0.645, bottom)  Tag 22 (Y=7.425, top)
 *                        Tag 23 (Y=7.425, top)      Tag 28 (Y=0.645, bottom)
 *   Blue center reef:    Tag 19 (Y=3.679)           Tag 20 (Y=4.035)
 *
 * Behavior:
 *   1. Must see a coral station tag to start (7/12 for Red, 17/22/23/28 for Blue)
 *   2. Strafes sideways toward the center reef (direction depends on which station)
 *   3. When a center reef tag appears (9/10 Red, 19/20 Blue), stops and rotates to align
 *   4. Finishes when the reef tag is centered within tolerance
 *
 * Bind to left D-pad. Press once to start, press again to cancel.
 */
public class AlignToShoot extends Command {

    private final Swerve swerve;
    private final Vision2025 vision;

    // Strafe speed magnitude (m/s) — sign chosen per coral station tag
    private static final double STRAFE_SPEED = 0.3;

    // Rotation gains for fine alignment
    private static final double ROT_KP = 1.8;          // softer P gain so it doesn't overshoot
    private static final double MAX_ROT = 0.8;          // hard cap on rotation speed (rad/s)
    private static final double SLOW_ROT = 0.4;         // even slower when tag is nearly centered
    private static final double SLOW_THRESHOLD = 8.0;   // degrees — switch to slow mode below this
    private static final double ANGLE_TOLERANCE = 2.0;   // degrees — done when within this

    // Timeout (cycles at 50Hz)
    private static final int TIMEOUT_CYCLES = 350; // 7 seconds (longer for cross-field strafe)

    // 2026 tag sets
    // Red coral station tags the robot might see when picking up coral
    private static final Set<Integer> RED_STATION_TAGS = Set.of(7, 12);
    // Blue coral station tags
    private static final Set<Integer> BLUE_STATION_TAGS = Set.of(17, 22, 23, 28);
    // Red center reef tags (the shooting targets)
    private static final Set<Integer> RED_REEF_CENTER = Set.of(9, 10);
    // Blue center reef tags
    private static final Set<Integer> BLUE_REEF_CENTER = Set.of(19, 20);

    // Tags at the BOTTOM of the field (low Y) — strafe must go UP (+Y in field, which is +strafe in robot-relative)
    private static final Set<Integer> BOTTOM_STATION_TAGS = Set.of(7, 17, 28);
    // Tags at the TOP of the field (high Y) — strafe must go DOWN (-Y in field, which is -strafe in robot-relative)
    private static final Set<Integer> TOP_STATION_TAGS = Set.of(12, 22, 23);

    // State machine
    private enum AlignState { STRAFING, ALIGNING, DONE }
    private AlignState state;
    private int cycleCount = 0;
    private int noTagCount = 0;
    private double strafeDirection = 0; // +1 or -1
    private Set<Integer> targetReefTags;  // which reef tags to look for
    private int alignTagId; // which specific tag to align to (10 for red, 20 for blue)

    public AlignToShoot(Swerve swerve, Vision2025 vision) {
        this.swerve = swerve;
        this.vision = vision;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        cycleCount = 0;
        noTagCount = 0;
        swerve.setFieldRelative(false); // Robot-relative so strafe direction is consistent

        // Determine alliance
        boolean isBlue = false;
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            isBlue = true;
        }

        Set<Integer> stationTags = isBlue ? BLUE_STATION_TAGS : RED_STATION_TAGS;
        targetReefTags = isBlue ? BLUE_REEF_CENTER : RED_REEF_CENTER;
        alignTagId = isBlue ? 20 : 10; // primary alignment tag (center-facing reef tag)

        // Check if we see a coral station tag
        Optional<Integer> tagId = vision.getBestTargetId();
        if (tagId.isPresent() && stationTags.contains(tagId.get())) {
            int seenTag = tagId.get();

            // Determine strafe direction based on which station tag we see
            if (BOTTOM_STATION_TAGS.contains(seenTag)) {
                // Station is at bottom (low Y), reef is above → strafe LEFT in robot-relative
                // (robot is facing the wall, so left = toward center of field = +Y)
                strafeDirection = -1.0;
            } else if (TOP_STATION_TAGS.contains(seenTag)) {
                // Station is at top (high Y), reef is below → strafe RIGHT in robot-relative
                strafeDirection = 1.0;
            } else {
                state = AlignState.DONE;
                return;
            }

            state = AlignState.STRAFING;
            SmartDashboard.putString("AlignToShoot/Status",
                "SEES TAG " + seenTag + " - STRAFING " + (strafeDirection < 0 ? "LEFT" : "RIGHT"));
        } else {
            // No station tag visible — abort
            state = AlignState.DONE;
            SmartDashboard.putString("AlignToShoot/Status", "NO STATION TAG - ABORTED");
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

        if (state == AlignState.STRAFING) {
            // Check if we can see a center reef tag
            Optional<Integer> tagId = vision.getBestTargetId();
            boolean seesReef = tagId.isPresent() && targetReefTags.contains(tagId.get());

            if (seesReef) {
                // Found a reef tag! Stop motion, then switch to alignment phase
                swerve.drive(new Translation2d(0, 0), 0);
                state = AlignState.ALIGNING;
                SmartDashboard.putString("AlignToShoot/Status",
                    "FOUND TAG " + tagId.get() + " - ALIGNING");
            } else {
                // Keep strafing toward the reef
                swerve.drive(new Translation2d(0, strafeDirection * STRAFE_SPEED), 0);
                SmartDashboard.putString("AlignToShoot/Status", "STRAFING");
            }
        } else if (state == AlignState.ALIGNING) {
            // Try to get transform to the primary alignment tag
            Optional<Transform3d> tf = vision.getTransformToTag(alignTagId);

            // If we can't see the primary tag, try the secondary
            if (tf.isEmpty()) {
                int secondaryTag = (alignTagId == 10) ? 9 : 19;
                tf = vision.getTransformToTag(secondaryTag);
            }

            if (tf.isEmpty()) {
                // Can't see either reef tag — hold still briefly
                swerve.drive(new Translation2d(0, 0), 0);
                noTagCount++;
                SmartDashboard.putString("AlignToShoot/Status", "LOOKING FOR REEF TAG");
                if (noTagCount > 75) {
                    state = AlignState.DONE; // Give up after ~1.5s
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
                // Rotate to center the reef tag — use a lower cap when close so
                // the tag stays in frame and we don't overshoot
                double capRot = (Math.abs(angleDeg) < SLOW_THRESHOLD) ? SLOW_ROT : MAX_ROT;
                double rot = angleDeg * ROT_KP * (Math.PI / 180.0);
                rot = Math.max(-capRot, Math.min(capRot, rot));
                swerve.drive(new Translation2d(0, 0), rot);
                SmartDashboard.putString("AlignToShoot/Status",
                    "ROTATING " + (Math.abs(angleDeg) < SLOW_THRESHOLD ? "(SLOW)" : "(FAST)"));
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
