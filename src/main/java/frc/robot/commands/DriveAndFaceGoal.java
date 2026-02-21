package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;

/**
 * DriveAndFaceGoal — 3-phase sequential command:
 *
 *   Phase 1 (FACE):   Reuses FaceGoal — rotates toward the reef center.
 *   Phase 2 (DRIVE):  Reuses DriveToTag — drives toward the nearest visible tag.
 *   Phase 3 (ALIGN):  Custom AlignToHub — rotates heading toward reef center
 *                      (odometry) while strafing to stay centered on the tag (vision).
 *
 * Bind to D-pad LEFT. Press once to start, press again to cancel.
 */
public class DriveAndFaceGoal extends SequentialCommandGroup {

    // Tag 10 field position (2026 field, WPI Blue origin, in meters)
    // 481.39 inches = 12.227m, 158.50 inches = 4.026m
    private static final int TARGET_TAG_ID = 10;
    private static final double TAG10_X = 12.227;
    private static final double TAG10_Y = 4.026;

    public DriveAndFaceGoal(Swerve swerve, Vision2025 vision) {
        this(swerve, vision, 3.0);
    }

    public DriveAndFaceGoal(Swerve swerve, Vision2025 vision, double targetDistance) {
        addCommands(
            // Phase 1: Face tag 10's field position (works even if seeing a different tag)
            new FaceGoal(swerve, TAG10_X, TAG10_Y, 0.0),

            // Phase 1.5: Keep rotating slowly until camera actually SEES tag 10
            new SearchForTag(swerve, vision),

            // Phase 2: Drive forward until ~targetDistance from tag 10 (NO centering/strafing)
            new DriveToDistance(swerve, vision, targetDistance),

            // Phase 3: Align heading to reef center + strafe to center on tag
            new AlignToHub(swerve, vision, targetDistance)
        );
    }

    /**
     * Phase 1.5 — Rotate slowly until tag 10 is visible in the camera.
     * FaceGoal uses odometry, but that might be slightly off. This phase
     * continues rotating in the same direction until vision actually sees tag 10.
     * If it already sees tag 10, finishes immediately.
     */
    private static class SearchForTag extends Command {

        private final Swerve swerve;
        private final Vision2025 vision;

        private static final double SEARCH_ROT_SPEED = 0.4; // rad/s — gentle rotation
        private static final int TIMEOUT_CYCLES = 200;      // 4 seconds max
        private static final int CONFIRM_CYCLES = 3;        // need 3 consecutive frames with tag 10

        private int cycleCount;
        private int confirmCount;
        private boolean isDone;
        private double rotDirection; // +1 or -1

        public SearchForTag(Swerve swerve, Vision2025 vision) {
            this.swerve = swerve;
            this.vision = vision;
            addRequirements(swerve);
        }

        @Override
        public void initialize() {
            cycleCount = 0;
            confirmCount = 0;
            isDone = false;

            // Initial direction guess from odometry (will be overridden if we see a known neighbor tag)
            Pose2d pose = swerve.getPose();
            double dx = TAG10_X - pose.getX();
            double dy = TAG10_Y - pose.getY();
            double angleToTag = Math.atan2(dy, dx);
            double headingError = angleToTag - pose.getRotation().getRadians();
            while (headingError >  Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;
            rotDirection = (headingError >= 0) ? 1.0 : -1.0;

            swerve.setFieldRelative(false);
            SmartDashboard.putString("DriveAndFace/Phase", "SEARCH");
        }

        /**
         * Decide rotation direction based on which tag is currently visible.
         * Tag 10 is to the LEFT of tag 12, so seeing 12 → rotate left (+1).
         * Tag 10 is to the RIGHT of tag 7, so seeing 7 → rotate right (-1).
         * For any other tag, fall back to the odometry-based guess.
         */
        private void updateRotDirectionFromVisibleTag() {
            Optional<Integer> visibleId = vision.getBestTargetId();
            if (visibleId.isPresent()) {
                int id = visibleId.get();
                if (id == 12) {
                    rotDirection = 1.0;   // tag 10 is to the left of tag 12
                } else if (id == 7) {
                    rotDirection = -1.0;  // tag 10 is to the right of tag 7
                }
                // For other tags, keep the current rotDirection (odometry guess)
                SmartDashboard.putNumber("DriveAndFace/SearchSeesTag", id);
            }
        }

        @Override
        public void execute() {
            cycleCount++;

            if (cycleCount > TIMEOUT_CYCLES) {
                // Timeout — proceed to DriveToTag anyway (it will handle no-tag)
                isDone = true;
                swerve.drive(new Translation2d(0, 0), 0);
                SmartDashboard.putString("DriveAndFace/Phase", "SEARCH TIMEOUT");
                return;
            }

            Optional<Transform3d> tf10 = vision.getTransformToTag(TARGET_TAG_ID);
            if (tf10.isPresent()) {
                confirmCount++;
                if (confirmCount >= CONFIRM_CYCLES) {
                    // Tag 10 confirmed visible — stop and proceed
                    isDone = true;
                    swerve.drive(new Translation2d(0, 0), 0);
                    SmartDashboard.putString("DriveAndFace/Phase", "TAG 10 FOUND");
                    return;
                }
            } else {
                confirmCount = 0;
                // Tag 10 not visible — use whatever tag IS visible to pick rotation direction
                updateRotDirectionFromVisibleTag();
            }

            // Keep rotating slowly to find tag 10
            swerve.drive(new Translation2d(0, 0), rotDirection * SEARCH_ROT_SPEED);
            SmartDashboard.putNumber("DriveAndFace/SearchCycles", cycleCount);
        }

        @Override
        public boolean isFinished() {
            return isDone;
        }

        @Override
        public void end(boolean interrupted) {
            swerve.drive(new Translation2d(0, 0), 0);
            SmartDashboard.putString("DriveAndFace/Phase",
                interrupted ? "SEARCH CANCELLED" : "SEARCH DONE");
        }
    }

    /**
     * Phase 2 — Drive forward/backward until the robot is approximately targetDistance
     * meters from tag 10.  This command does NOT strafe or rotate to center on the tag.
     * It only adjusts the forward distance so the approach works identically from both
     * the tag-7 side and the tag-12 side.  Centering is left entirely to AlignToHub.
     */
    private static class DriveToDistance extends Command {

        private final Swerve swerve;
        private final Vision2025 vision;
        private final double targetDistance;

        private static final double MAX_FORWARD_SPEED = 1.2;
        private static final double SLOWDOWN_START = 4.0;   // proportional slowdown zone
        private static final double DIST_TOL = 0.20;        // meters — "close enough"
        private static final double MIN_SPEED = 0.12;       // prevent creep-to-zero
        private static final int TIMEOUT_CYCLES = 250;      // 5 seconds

        // Slew-rate limiting
        private static final double MAX_ACCEL = 0.06;
        private double prevForward = 0;
        private int cycleCount;
        private int noTagCount;
        private boolean isDone;

        public DriveToDistance(Swerve swerve, Vision2025 vision, double targetDistance) {
            this.swerve = swerve;
            this.vision = vision;
            this.targetDistance = targetDistance;
            addRequirements(swerve);
        }

        @Override
        public void initialize() {
            prevForward = 0;
            cycleCount = 0;
            noTagCount = 0;
            isDone = false;
            swerve.setFieldRelative(false);
            SmartDashboard.putString("DriveAndFace/Phase", "DRIVE");
        }

        @Override
        public void execute() {
            cycleCount++;
            if (cycleCount > TIMEOUT_CYCLES) {
                isDone = true;
                swerve.drive(new Translation2d(0, 0), 0);
                SmartDashboard.putString("DriveAndFace/Phase", "DRIVE TIMEOUT");
                return;
            }

            Optional<Transform3d> tf10 = vision.getTransformToTag(TARGET_TAG_ID);
            if (tf10.isPresent()) {
                noTagCount = 0;
                double tagX = tf10.get().getX(); // forward distance to tag

                double distError = tagX - targetDistance; // positive = too far, negative = too close
                SmartDashboard.putNumber("DriveAndFace/DriveDistError", distError);
                SmartDashboard.putNumber("DriveAndFace/DriveTagDist", tagX);

                if (Math.abs(distError) < DIST_TOL) {
                    // Close enough — done
                    isDone = true;
                    swerve.drive(new Translation2d(0, 0), 0);
                    SmartDashboard.putString("DriveAndFace/Phase", "DRIVE REACHED");
                    return;
                }

                // Proportional forward speed (positive = drive toward tag, negative = back away)
                double speedRatio = Math.min(Math.abs(distError) / SLOWDOWN_START, 1.0);
                double forwardSpeed = Math.copySign(speedRatio * MAX_FORWARD_SPEED, distError);
                // Enforce minimum speed so we don't stall
                if (Math.abs(forwardSpeed) < MIN_SPEED) {
                    forwardSpeed = Math.copySign(MIN_SPEED, distError);
                }

                // Slew-rate limit for smooth acceleration
                forwardSpeed = rampLimit(forwardSpeed, prevForward, MAX_ACCEL);
                prevForward = forwardSpeed;

                // Drive forward/back only — zero strafe, zero rotation
                swerve.drive(new Translation2d(forwardSpeed, 0), 0);
                SmartDashboard.putString("DriveAndFace/Phase", "DRIVING");
            } else {
                // Lost tag 10 — stop and wait
                noTagCount++;
                swerve.drive(new Translation2d(0, 0), 0);
                prevForward = 0;
                SmartDashboard.putString("DriveAndFace/Phase", "DRIVE (no tag)");

                if (noTagCount > 100) {
                    // Lost tag for 2 seconds — proceed to AlignToHub which can search
                    isDone = true;
                }
            }
        }

        private double rampLimit(double target, double prev, double maxDelta) {
            double delta = target - prev;
            if (delta >  maxDelta) delta =  maxDelta;
            if (delta < -maxDelta) delta = -maxDelta;
            return prev + delta;
        }

        @Override
        public boolean isFinished() {
            return isDone;
        }

        @Override
        public void end(boolean interrupted) {
            swerve.drive(new Translation2d(0, 0), 0);
            SmartDashboard.putString("DriveAndFace/Phase",
                interrupted ? "DRIVE CANCELLED" : "DRIVE DONE");
        }
    }

    /**
     * Phase 3 — the only custom piece.
     * Rotates heading toward reef center using odometry,
     * strafes to stay centered on TAG 10 ONLY using vision.
     * If tag 10 is not visible but other tags are, rotates slowly until tag 10 appears.
     */
    private static class AlignToHub extends Command {

        private final Swerve swerve;
        private final Vision2025 vision;
        private final double targetDistance;

        // Reef centers (2026 field)
        private static final double RED_GOAL_X = 12.004;
        private static final double RED_GOAL_Y = 4.035;
        private static final double BLUE_GOAL_X = 4.537;
        private static final double BLUE_GOAL_Y = 4.035;

        // Rotation
        private static final double ROT_KP = 2.0;
        private static final double ROT_MAX = 0.8;
        private static final double ROT_SLOW_KP = 1.0;
        private static final double ROT_SLOW_MAX = 0.4;
        private static final double ROT_SLOW_THRESH = 8.0;   // degrees
        // Slow search rotation when tag 10 not visible but other tags are
        private static final double SEARCH_ROT_SPEED = 0.3;  // rad/s — gentle spin to find tag 10
        // Strafe
        private static final double STRAFE_KP = 1.5;
        private static final double STRAFE_MAX = 0.5;
        // Forward distance hold
        private static final double DIST_KP = 0.8;
        private static final double DIST_MAX_SPEED = 0.4;
        private static final double DIST_TOL = 0.15;         // meters — OK within 15cm of target
        // Tolerances
        private static final double HEADING_TOL = 2.0;       // degrees
        private static final double CENTER_TOL = 0.12;       // meters — relaxed for 3m range
        private static final int TIMEOUT_CYCLES = 250;       // 5 seconds

        private double goalX, goalY;
        private int cycleCount;
        private int noTagCount;
        private double lastTagY;
        private boolean isDone;

        public AlignToHub(Swerve swerve, Vision2025 vision, double targetDistance) {
            this.swerve = swerve;
            this.vision = vision;
            this.targetDistance = targetDistance;
            addRequirements(swerve);
        }

        @Override
        public void initialize() {
            cycleCount = 0;
            noTagCount = 0;
            lastTagY = 0;
            isDone = false;

            Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
                goalX = BLUE_GOAL_X;
                goalY = BLUE_GOAL_Y;
            } else {
                goalX = RED_GOAL_X;
                goalY = RED_GOAL_Y;
            }

            swerve.setFieldRelative(false);
            SmartDashboard.putString("DriveAndFace/Phase", "ALIGN");
        }

        @Override
        public void execute() {
            cycleCount++;
            if (cycleCount > TIMEOUT_CYCLES) {
                isDone = true;
                swerve.drive(new Translation2d(0, 0), 0);
                SmartDashboard.putString("DriveAndFace/Phase", "TIMEOUT");
                return;
            }

            // --- Rotation: odometry-based toward reef center ---
            Pose2d currentPose = swerve.getPose();
            double robotHeadingRad = currentPose.getRotation().getRadians();

            double dx = goalX - currentPose.getX();
            double dy = goalY - currentPose.getY();
            double targetAngleRad = Math.atan2(dy, dx);

            double headingErrorRad = targetAngleRad - robotHeadingRad;
            while (headingErrorRad > Math.PI)  headingErrorRad -= 2 * Math.PI;
            while (headingErrorRad < -Math.PI) headingErrorRad += 2 * Math.PI;
            double headingErrorDeg = Math.toDegrees(headingErrorRad);

            // --- Strafe: vision-based centering on TAG 10 ONLY ---
            Optional<Transform3d> tf10 = vision.getTransformToTag(TARGET_TAG_ID);
            double tagY;
            double tagX = targetDistance;
            boolean hasTag10 = false;

            if (tf10.isPresent()) {
                // We can see tag 10 — use it for strafing
                Transform3d transform = tf10.get();
                tagY = transform.getY();
                tagX = transform.getX();
                lastTagY = tagY;
                hasTag10 = true;
                noTagCount = 0;
            } else {
                // Tag 10 NOT visible
                tagY = lastTagY;
                noTagCount++;

                // Check if we can see ANY tag (probably tag 12 or 7)
                Optional<Integer> visibleId = vision.getBestTargetId();

                if (visibleId.isPresent()) {
                    // We see a different tag but not 10 — rotate toward tag 10.
                    // Use the visible tag ID to pick the correct direction:
                    //   Tag 12 → tag 10 is to the left  → rotate left (+)
                    //   Tag 7  → tag 10 is to the right → rotate right (-)
                    //   Other  → fall back to odometry heading error
                    int id = visibleId.get();
                    double searchRot;
                    if (id == 12) {
                        searchRot = SEARCH_ROT_SPEED;   // rotate left
                    } else if (id == 7) {
                        searchRot = -SEARCH_ROT_SPEED;  // rotate right
                    } else {
                        searchRot = (headingErrorRad > 0) ? SEARCH_ROT_SPEED : -SEARCH_ROT_SPEED;
                    }
                    swerve.drive(new Translation2d(0, 0), searchRot);
                    SmartDashboard.putString("DriveAndFace/Phase",
                        "ALIGN (searching for tag 10, sees tag " + id + ")");
                    SmartDashboard.putNumber("DriveAndFace/AlignHeadingError", headingErrorDeg);
                    SmartDashboard.putBoolean("DriveAndFace/AlignHasTag", false);
                    return; // skip normal align logic — just search
                }

                if (noTagCount > 150) {
                    // Lost ALL tags for 3 seconds — finish anyway
                    isDone = true;
                    swerve.drive(new Translation2d(0, 0), 0);
                    SmartDashboard.putString("DriveAndFace/Phase", "DONE (tag lost)");
                    return;
                }
            }

            SmartDashboard.putNumber("DriveAndFace/AlignHeadingError", headingErrorDeg);
            SmartDashboard.putNumber("DriveAndFace/AlignOffset", tagY);
            SmartDashboard.putBoolean("DriveAndFace/AlignHasTag", hasTag10);

            boolean headingOk = Math.abs(headingErrorDeg) < HEADING_TOL;
            boolean centerOk = Math.abs(tagY) < CENTER_TOL;
            boolean distanceOk = Math.abs(tagX - targetDistance) < DIST_TOL;

            if (headingOk && centerOk && distanceOk && hasTag10) {
                isDone = true;
                swerve.drive(new Translation2d(0, 0), 0);
                SmartDashboard.putString("DriveAndFace/Phase", "DONE!");
                return;
            }

            // Two-speed rotation
            boolean slowRot = Math.abs(headingErrorDeg) < ROT_SLOW_THRESH;
            double rotKp  = slowRot ? ROT_SLOW_KP  : ROT_KP;
            double rotCap = slowRot ? ROT_SLOW_MAX  : ROT_MAX;
            double rot = clamp(headingErrorRad * rotKp, -rotCap, rotCap);

            // Strafe — live tag 10 offset, or last known if briefly lost
            double strafe = clamp(tagY * STRAFE_KP, -STRAFE_MAX, STRAFE_MAX);

            // Hold target distance with proportional control (only with live tag 10 vision)
            double forward = 0;
            if (hasTag10) {
                double distError = tagX - targetDistance;
                if (Math.abs(distError) > DIST_TOL) {
                    forward = clamp(distError * DIST_KP, -DIST_MAX_SPEED, DIST_MAX_SPEED);
                }
            }

            swerve.drive(new Translation2d(forward, strafe), rot);
            SmartDashboard.putString("DriveAndFace/Phase",
                hasTag10 ? (slowRot ? "ALIGN (SLOW)" : "ALIGN") : "ALIGN (no tag 10, strafing)");
            SmartDashboard.putNumber("DriveAndFace/AlignDistError",
                hasTag10 ? (tagX - targetDistance) : 0);
        }

        private double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }

        @Override
        public boolean isFinished() {
            return isDone;
        }

        @Override
        public void end(boolean interrupted) {
            swerve.drive(new Translation2d(0, 0), 0);
            swerve.setFieldRelative(true);
            SmartDashboard.putBoolean("DriveAndFace/Active", false);
            SmartDashboard.putString("DriveAndFace/Phase", interrupted ? "CANCELLED" : "COMPLETE");
        }
    }
}
