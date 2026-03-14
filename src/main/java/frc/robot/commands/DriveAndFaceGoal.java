package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ElasticDashboard;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;

/**
 * DriveAndFaceGoal — single-command approach bound to D-pad LEFT.
 *
 * <h3>Behaviour:</h3>
 * <ol>
 *   <li><b>Rotate to find tag 10</b> — uses odometry to guess direction, then
 *       searches with a slow spin until tag 10 appears in the camera.</li>
 *   <li><b>Center tag 10 in the camera</b> — pure rotation (P-control on the
 *       camera-frame bearing angle).  No strafe, so there is no jitter.</li>
 *   <li><b>If &gt; 3 m away:</b> drive forward toward the tag while keeping it
 *       centered, until the robot reaches 3 m.</li>
 *   <li><b>If 1–3 m away:</b> remain at the current distance; only rotate to
 *       keep the tag centred.</li>
 *   <li><b>If &lt; 1 m:</b> back up gently to 1 m while keeping tag centred.</li>
 * </ol>
 *
 * Finishes when the tag is centred and the range is inside the 1–3 m band for
 * several consecutive cycles, or on timeout.
 *
 * Press once to start; press again (toggle binding) to cancel.
 */
public class DriveAndFaceGoal extends Command {

    private final Swerve swerve;
    private final Vision2025 vision;

    // ---- Target tag ----
    private static final int TARGET_TAG_ID = 10;

    // Tag 10 approximate field position (for initial rotation guess)
    private static final double TAG10_X = 12.227; // metres (2026 field, blue-origin)
    private static final double TAG10_Y = 4.026;

    // ---- Distance band ----
    private static final double MAX_RADIUS = 3.0;   // outer edge — drive in if farther
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
        double angleToTag = Math.atan2(TAG10_Y - pose.getY(), TAG10_X - pose.getX());
        double headingErr = angleToTag - pose.getRotation().getRadians();
        headingErr = wrapAngle(headingErr);
        searchDirection = (headingErr >= 0) ? 1.0 : -1.0;

        swerve.setFieldRelative(false);
        ElasticDashboard.putBoolean("DriveFace/Active", true);
        ElasticDashboard.putString("DriveFace/Phase", "SEARCH");
    }

    @Override
    public void execute() {
        cycles++;

        // Always try to see tag 10
        Optional<Transform3d> opt = vision.getTransformToTag(TARGET_TAG_ID);

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
            // Found tag 10 → transition to ALIGN
            phase = Phase.ALIGN;
            ElasticDashboard.putString("DriveFace/Phase", "ALIGN");
            executeAlign(opt);
            return;
        }

        if (searchCycles > SEARCH_TIMEOUT) {
            // Give up searching — transition anyway
            phase = Phase.ALIGN;
            ElasticDashboard.putString("DriveFace/Phase", "ALIGN (search timed out)");
            return;
        }

        // Refine search direction from any visible tag
        Optional<Integer> visibleId = vision.getBestTargetId();
        if (visibleId.isPresent()) {
            int id = visibleId.get();
            if (id == 12) searchDirection = 1.0;       // tag 10 is left of 12
            else if (id == 7) searchDirection = -1.0;   // tag 10 is right of 7
        }

        // Gentle rotation to find tag 10
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
            // Tag 10 lost — coast to stop smoothly
            prevFwd = slewLimit(0, prevFwd, MAX_FWD_DELTA);
            prevRot = slewLimit(0, prevRot, MAX_ROT_DELTA);
            swerve.drive(new Translation2d(prevFwd, 0), prevRot);
            stableCount = 0;
            ElasticDashboard.putString("DriveFace/Status", "NO_TAG");
            return;
        }

        Transform3d t = opt.get();
        double tagX = t.getX();   // forward (positive = tag in front)
        double tagY = t.getY();   // lateral (positive = tag is left of camera center)
        double range = Math.hypot(tagX, tagY);

        // ---- Rotation: centre the tag in the camera ----
        // bearingRad > 0 means tag is to the LEFT of center
        // We want to rotate LEFT (positive) to center it → rotCmd = +bearing * KP
        double bearingRad = Math.atan2(tagY, tagX);
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
        ElasticDashboard.putNumber("DriveFace/tagX", tagX);
        ElasticDashboard.putNumber("DriveFace/tagY", tagY);
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
