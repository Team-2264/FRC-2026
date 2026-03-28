package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ElasticDashboard;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;

/**
 * DriveAndFaceGoal — single-command approach bound to D-pad LEFT.
 */
public class DriveAndFaceGoal extends Command {

    private final Swerve swerve;
    private final Vision2025 vision;

    // ---- Distance band ----
    private static final double MAX_RADIUS = 4.0;   // outer edge — drive in if farther
    private static final double MIN_RADIUS = 1.0;   // inner edge — back up if closer

    // ---- Speed caps ----
    private static final double MAX_FWD       = 1.5;  // m/s — approach speed cap
    private static final double SLOW_FWD      = 0.5;  // m/s — near-band-edge cap
    private static final double SLOW_ZONE     = 0.8;  // metres from band edge → start slowing
    private static final double MAX_BACK      = 0.6;  // m/s — backing-up cap
    private static final double MAX_ROT       = 2.0;  // rad/s — fast rotation cap
    private static final double SEARCH_ROT    = 0.5;  // rad/s — gentle search spin

    // ---- Proportional gains ----
    private static final double KP_FWD  = 1.0;   // m error → m/s
    private static final double KP_BACK = 0.8;
    private static final double KP_ROT  = 3.0;   // rad bearing error → rad/s

    // ---- Slew-rate limits (per 20 ms cycle) ----
    private static final double MAX_FWD_DELTA = 0.08;
    private static final double MAX_ROT_DELTA = 0.12;

    // ---- Finish tolerances ----
    private static final double DIST_TOL       = 0.25;  // metres — within-band tolerance
    private static final double ANGLE_TOL_DEG  = 2.5;   // degrees — tag centred in camera
    private static final int    REQUIRED_STABLE = 8;     // consecutive OK cycles

    // ---- Safety ----
    private static final int SEARCH_TIMEOUT  = 200;  // 4 s max searching for tag 10
    private static final int TOTAL_TIMEOUT   = 500;  // 10 s overall timeout

    // ---- State ----
    private enum Phase { SEARCH, ALIGN }
    private Phase phase;
    private double prevFwd, prevRot;
    private int stableCount;
    private int cycles;
    private int searchCycles;
    private double searchDirection; // +1 or -1

    public DriveAndFaceGoal(Swerve swerve, Vision2025 vision) {
        this.swerve = swerve;
        this.vision = vision;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        prevFwd = 0;
        prevRot = 0;
        stableCount = 0;
        cycles = 0;
        searchCycles = 0;
        phase = Phase.SEARCH;

        // Guess initial search direction from odometry
        Pose2d pose = swerve.getPose();
        
        Optional<Pose3d> exactTagPose = vision.getNearestHubTagPose3d();
        if (exactTagPose.isPresent()) {
            double angleToTag = Math.atan2(exactTagPose.get().getY() - pose.getY(), exactTagPose.get().getX() - pose.getX());
            double headingErr = angleToTag - pose.getRotation().getRadians();
            headingErr = wrapAngle(headingErr);
            searchDirection = (headingErr >= 0) ? 1.0 : -1.0;
        } else {
            searchDirection = 1.0;
        }

        swerve.setFieldRelative(false);
        ElasticDashboard.putBoolean("DriveFace/Active", true);
        ElasticDashboard.putString("DriveFace/Phase", "SEARCH");
    }

    @Override
    public void execute() {
        cycles++;

        // Always try to see hub tag
        Optional<Transform3d> opt = vision.getNearestHubTagTransform();

        if (phase == Phase.SEARCH) {
            executeSearch(opt);
        } else {
            executeAlign(opt);
        }
    }

    // ===================================================================
    //  SEARCH — rotate until tag 10 is visible
    // ===================================================================

    private void executeSearch(Optional<Transform3d> opt) {
        searchCycles++;

        if (opt.isPresent()) {
            // Found a hub tag -> transition to ALIGN
            phase = Phase.ALIGN;
            ElasticDashboard.putString("DriveFace/Phase", "ALIGN");
            executeAlign(opt);
            return;
        }

        if (searchCycles > SEARCH_TIMEOUT) {
            // Give up searching -> transition anyway
            phase = Phase.ALIGN;
            ElasticDashboard.putString("DriveFace/Phase", "ALIGN (search timed out)");
            return;
        }

        // Gentle rotation to find hub tag
        double rot = slewLimit(searchDirection * SEARCH_ROT, prevRot, MAX_ROT_DELTA);
        prevRot = rot;
        prevFwd = slewLimit(0, prevFwd, MAX_FWD_DELTA);
        swerve.drive(new Translation2d(prevFwd, 0), rot);

        ElasticDashboard.putNumber("DriveFace/SearchCycles", searchCycles);
    }

    // ===================================================================
    //  ALIGN — center tag in camera + maintain distance band
    // ===================================================================

    private void executeAlign(Optional<Transform3d> opt) {

        if (!opt.isPresent()) {
            // Hub tag lost -> coast to stop smoothly
            prevFwd = slewLimit(0, prevFwd, MAX_FWD_DELTA);
            prevRot = slewLimit(0, prevRot, MAX_ROT_DELTA);
            swerve.drive(new Translation2d(prevFwd, 0), prevRot);
            stableCount = 0;
            ElasticDashboard.putString("DriveFace/Status", "NO_TAG");
            return;
        }

        // Because of camera offset and PnP instability, camera-relative angles can over-compensate.
        // We use the fused Odometry (which accurately tracks our global position using the tags)
        // to aim the physical center of the robot directly at the hub's coordinates.
        Pose2d robotPose = swerve.getPose();

        double tagFieldX = robotPose.getX();
        double tagFieldY = robotPose.getY();

        // Approximate hub location (if tag isn't yet retrieved from layout)
        double hubFieldX = tagFieldX;
        double hubFieldY = tagFieldY;

        // Refine with the exact field layout position for the nearest hub tag
        Optional<Pose3d> exactTagPose = vision.getNearestHubTagPose3d();
        if (exactTagPose.isPresent()) {
            Pose3d tagPose = exactTagPose.get();
            tagFieldX = tagPose.getX();
            tagFieldY = tagPose.getY();

            // The center of the hub is 23.5 inches behind the tag in the tag's local coordinate system.
            // A tag's X axis points outwards (into the field). We want to go backwards (into the hub).
            // So we translate -23.5 inches along the tag's local X axis.
            Pose3d hubPose = tagPose.transformBy(
                new Transform3d(
                    new edu.wpi.first.math.geometry.Translation3d(Units.inchesToMeters(-23.5), 0, 0),
                    new edu.wpi.first.math.geometry.Rotation3d()
                )
            );

            hubFieldX = hubPose.getX();
            hubFieldY = hubPose.getY();
        }

        // Maintain distance band using the TAG face distance, so we don't crash into the hub
        double tagDx = tagFieldX - robotPose.getX();
        double tagDy = tagFieldY - robotPose.getY();
        double range = Math.hypot(tagDx, tagDy);

        // Absolute field angle we need to point our center towards (aim at the HUB, not the tag)
        double hubDx = hubFieldX - robotPose.getX();
        double hubDy = hubFieldY - robotPose.getY();
        double targetAngleRad = Math.atan2(hubDy, hubDx);

        // Relative error between our current heading and the target heading
        // Positive means we need to rotate LEFT.
        double bearingRad = wrapAngle(targetAngleRad - robotPose.getRotation().getRadians());
        
        // P-control using the bearing error
        double rotCmd = clamp(bearingRad * KP_ROT, -MAX_ROT, MAX_ROT);

        // ---- Forward: enforce distance band [MIN_RADIUS .. MAX_RADIUS] ----
        double fwdCmd = 0;
        if (range > MAX_RADIUS) {
            // Too far → drive forward
            double err = range - MAX_RADIUS;
            double cap = (err < SLOW_ZONE) ? SLOW_FWD : MAX_FWD;
            fwdCmd = clamp(err * KP_FWD, 0, cap);
        } else if (range < MIN_RADIUS) {
            // Too close → back up
            double err = MIN_RADIUS - range;
            fwdCmd = -clamp(err * KP_BACK, 0, MAX_BACK);
        }
        // else: inside 1–3 m band → forward stays 0, only rotate

        // ---- Slew-rate limiting ----
        fwdCmd = slewLimit(fwdCmd, prevFwd, MAX_FWD_DELTA);
        rotCmd = slewLimit(rotCmd, prevRot, MAX_ROT_DELTA);
        prevFwd = fwdCmd;
        prevRot = rotCmd;

        // ---- Drive (no strafe) ----
        swerve.drive(new Translation2d(fwdCmd, 0), rotCmd);

        // ---- Telemetry ----
        ElasticDashboard.putNumber("DriveFace/range", range);
        ElasticDashboard.putNumber("DriveFace/hubDx", hubDx);
        ElasticDashboard.putNumber("DriveFace/hubDy", hubDy);
        ElasticDashboard.putNumber("DriveFace/bearingDeg", Math.toDegrees(bearingRad));
        ElasticDashboard.putNumber("DriveFace/fwdCmd", fwdCmd);
        ElasticDashboard.putNumber("DriveFace/rotCmd", rotCmd);
        ElasticDashboard.putString("DriveFace/Status",
                (range > MAX_RADIUS) ? "APPROACHING" :
                (range < MIN_RADIUS) ? "BACKING UP" : "IN BAND");

        // ---- Stable check ----
        boolean distOk  = range >= (MIN_RADIUS - DIST_TOL) && range <= (MAX_RADIUS + DIST_TOL);
        boolean angleOk = Math.abs(Math.toDegrees(bearingRad)) <= ANGLE_TOL_DEG;
        if (distOk && angleOk) {
            stableCount++;
        } else {
            stableCount = 0;
        }
    }

    // ===================================================================
    //  Finish / End
    // ===================================================================

    @Override
    public boolean isFinished() {
        return stableCount >= REQUIRED_STABLE || cycles > TOTAL_TIMEOUT;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0);
        swerve.setFieldRelative(true);
        ElasticDashboard.putBoolean("DriveFace/Active", false);
        ElasticDashboard.putString("DriveFace/Phase",
                interrupted ? "CANCELLED" : "COMPLETE");
    }

    // ===================================================================
    //  Helpers
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

    private static double wrapAngle(double rad) {
        while (rad >  Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }
}
