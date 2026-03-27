package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.ElasticDashboard;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;

/**
 * ArcDrive — bound to L2 (whileTrue).
 *
 * <h3>Behaviour:</h3>
 * <p>
 * While L2 is held, the robot is locked on a circular arc around AprilTag 10.
 * The radius is captured on initialisation from the current distance to the
 * tag.
 * Hold L1 to move left along the arc, R1 to move right along the arc.
 * The robot continuously rotates to keep tag 10 centred in the camera.
 * If tag 10 is lost, the robot stops immediately and waits for re-acquisition.
 * </p>
 *
 * <h3>Prerequisites:</h3>
 * <p>
 * The driver should first use D-pad LEFT (DriveAndFaceGoal) to align with
 * tag 10 before activating this command.
 * </p>
 */
public class ArcDrive extends Command {

    private final Swerve swerve;
    private final Vision2025 vision;
    private final CommandPS4Controller controller;

    // ---- Targets ----
    private static final int[] TARGET_TAG_IDS = { 10, 26 };

    // ---- Arc movement speed ----
    private static final double ARC_SPEED = 0.5; // m/s — fixed slow speed along the arc

    // ---- Rotation P-gain (keep tag centred in camera) ----
    private static final double KP_ROT = 3.0; // rad bearing error → rad/s
    private static final double MAX_ROT = 2.0; // rad/s cap

    // ---- Radial correction ----
    private static final double RADIUS_DEADZONE = 0.05; // metres — no correction inside this band
    private static final double KP_RADIAL = 1.0; // P-gain for radius correction
    private static final double MAX_RADIAL = 0.3; // m/s cap for radial correction

    // ---- Slew-rate limits (per 20 ms cycle) ----
    private static final double MAX_SPEED_DELTA = 0.06;
    private static final double MAX_ROT_DELTA = 0.10;

    // ---- State ----
    private double arcRadius; // captured on init (metres)
    private double prevArcSpeed; // slew-rate state for tangential speed
    private double prevRot; // slew-rate state for rotation
    private int lockedTagId; // which tag we locked to for this run

    public ArcDrive(Swerve swerve, Vision2025 vision, CommandPS4Controller controller) {
        this.swerve = swerve;
        this.vision = vision;
        this.controller = controller;
        addRequirements(swerve);
    }

    // ===================================================================
    // Lifecycle
    // ===================================================================

    @Override
    public void initialize() {
        prevArcSpeed = 0;
        prevRot = 0;

        // Use robot-relative: forward = toward tag, left/right = along arc
        swerve.setFieldRelative(false);

        // Choose nearest target tag based on current pose and field layout
        Pose2d robotPose = swerve.getPose();
        double bestDist = Double.POSITIVE_INFINITY;
        lockedTagId = TARGET_TAG_IDS[0];

        for (int id : TARGET_TAG_IDS) {
            Optional<edu.wpi.first.math.geometry.Pose3d> tagPoseOpt = vision.getFieldLayout().getTagPose(id);
            if (tagPoseOpt.isEmpty())
                continue;
            Pose2d tagPose = tagPoseOpt.get().toPose2d();
            double dist = robotPose.getTranslation().getDistance(tagPose.getTranslation());
            if (dist < bestDist) {
                bestDist = dist;
                lockedTagId = id;
            }
        }

        // Capture radius from current robot position to chosen tag
        Optional<edu.wpi.first.math.geometry.Pose3d> lockedTagPose = vision.getFieldLayout().getTagPose(lockedTagId);
        double dx = 0;
        double dy = 0;
        if (lockedTagPose.isPresent()) {
            Pose2d tagPose2d = lockedTagPose.get().toPose2d();
            dx = tagPose2d.getX() - robotPose.getX();
            dy = tagPose2d.getY() - robotPose.getY();
        }
        arcRadius = Math.hypot(dx, dy);

        // Clamp radius to sensible bounds
        if (arcRadius < 0.5)
            arcRadius = 0.5;
        if (arcRadius > 6.0)
            arcRadius = 6.0;

        ElasticDashboard.putBoolean("ArcDrive/Active", true);
        ElasticDashboard.putNumber("ArcDrive/Radius", arcRadius);
        ElasticDashboard.putNumber("ArcDrive/LockedTag", lockedTagId);
    }

    @Override
    public void execute() {
        // ---- Read L1 / R1 for arc direction ----
        boolean l1 = controller.getHID().getL1Button();
        boolean r1 = controller.getHID().getR1Button();

        // L1 = move left along arc, R1 = move right along arc
        // If both or neither are pressed, target speed is 0
        double desiredStrafeSpeed = 0;
        if (l1 && !r1) {
            desiredStrafeSpeed = ARC_SPEED; // strafe left (robot-relative)
        } else if (r1 && !l1) {
            desiredStrafeSpeed = -ARC_SPEED; // strafe right (robot-relative)
        }

        // ---- Check for the chosen tag (either 10 or 26) ----
        Optional<Transform3d> opt = vision.getTransformToAny(TARGET_TAG_IDS);

        if (!opt.isPresent()) {
            // Tag lost — coast to stop
            prevArcSpeed = slewLimit(0, prevArcSpeed, MAX_SPEED_DELTA);
            prevRot = slewLimit(0, prevRot, MAX_ROT_DELTA);
            swerve.drive(new Translation2d(0, 0), 0);
            ElasticDashboard.putString("ArcDrive/Status", "TAG_LOST");
            return;
        }

        // ---- Rotation: keep tag centred using camera bearing ----
        Transform3d t = opt.get();
        double tagCamX = t.getX(); // forward in camera frame
        double tagCamY = t.getY(); // left in camera frame
        double range = Math.hypot(tagCamX, tagCamY);
        double bearingRad = Math.atan2(tagCamY, tagCamX);
        double rotCmd = clamp(bearingRad * KP_ROT, -MAX_ROT, MAX_ROT);
        rotCmd = slewLimit(rotCmd, prevRot, MAX_ROT_DELTA);
        prevRot = rotCmd;

        // ---- Radial correction (forward/backward in robot frame) ----
        // Use camera-measured range directly instead of odometry
        double radiusError = range - arcRadius; // positive = too far from tag
        double fwdCmd = 0;
        if (Math.abs(radiusError) > RADIUS_DEADZONE) {
            double correctable = radiusError - Math.signum(radiusError) * RADIUS_DEADZONE;
            fwdCmd = clamp(correctable * KP_RADIAL, -MAX_RADIAL, MAX_RADIAL);
        }

        // ---- Tangential speed (strafe in robot frame, slew-rate limited) ----
        double strafeSpeed = slewLimit(desiredStrafeSpeed, prevArcSpeed, MAX_SPEED_DELTA);
        prevArcSpeed = strafeSpeed;

        // ---- Drive (robot-relative) ----
        // X = forward (toward tag), Y = left (along arc)
        swerve.drive(new Translation2d(fwdCmd, strafeSpeed), rotCmd);

        // ---- Telemetry ----
        ElasticDashboard.putNumber("ArcDrive/Radius", arcRadius);
        ElasticDashboard.putNumber("ArcDrive/CameraRange", range);
        ElasticDashboard.putNumber("ArcDrive/RadiusError", radiusError);
        ElasticDashboard.putNumber("ArcDrive/FwdCmd", fwdCmd);
        ElasticDashboard.putNumber("ArcDrive/StrafeSpeed", strafeSpeed);
        ElasticDashboard.putBoolean("ArcDrive/L1", l1);
        ElasticDashboard.putBoolean("ArcDrive/R1", r1);
        ElasticDashboard.putNumber("ArcDrive/BearingDeg", Math.toDegrees(bearingRad));
        ElasticDashboard.putNumber("ArcDrive/RotCmd", rotCmd);
        ElasticDashboard.putString("ArcDrive/Status",
                Math.abs(radiusError) < RADIUS_DEADZONE ? "ON_ARC" : "CORRECTING");
    }

    // ===================================================================
    // End
    // ===================================================================

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0);
        swerve.setFieldRelative(true);
        prevArcSpeed = 0;
        prevRot = 0;
        ElasticDashboard.putBoolean("ArcDrive/Active", false);
        ElasticDashboard.putString("ArcDrive/Status", interrupted ? "CANCELLED" : "DONE");
    }

    // ===================================================================
    // Helpers
    // ===================================================================

    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private static double slewLimit(double target, double prev, double maxDelta) {
        double delta = target - prev;
        if (Math.abs(delta) > maxDelta) {
            delta = Math.signum(delta) * maxDelta;
        }
        return prev + delta;
    }
}
