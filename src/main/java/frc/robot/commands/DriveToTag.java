package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.ElasticDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;

/**
 * DriveToTag — move onto a distance band around an AprilTag and rotate to
 * center the tag in the camera.
 *
 * <p><b>Distance band:</b> The robot must stay between {@code MIN_RADIUS} and
 * {@code maxRadius} metres from the tag.  If it is farther than {@code maxRadius}
 * it drives forward; if it is closer than {@code MIN_RADIUS} it backs up.
 * Inside the band it only rotates to centre the tag.
 *
 * <p><b>Speed profile:</b> Forward speed scales with distance error — fast when
 * far away, progressively slower as it approaches the band edge.  A separate
 * "slow zone" near the band edge caps the speed further so arrivals are gentle.
 *
 * <p><b>Rotation:</b> P-control on {@code atan2(tagY, tagX)} centres the tag
 * in the camera.  No strafing — rotation alone sweeps the robot along an arc,
 * making the path smooth and identical from both the tag-7 and tag-12 sides.
 *
 * <p>Finishes when range is inside the band AND tag centred for several
 * consecutive cycles.
 */
public class DriveToTag extends Command {

    private final Swerve swerve;
    private final Vision2025 vision;
    private final int tagId;          // -1 = best visible tag
    private final double maxRadius;   // metres — outer edge of acceptable band

    // ---- Distance band ----
    private static final double MIN_RADIUS = 1.0;   // metres — too close → back up

    // ---- Speed caps ----
    private static final double MAX_FWD       = 2.5;  // m/s — full speed when far away
    private static final double SLOW_FWD      = 0.6;  // m/s — cap when within SLOW_ZONE of band edge
    private static final double SLOW_ZONE     = 0.8;  // metres — distance from band edge to start slowing
    private static final double MAX_BACK      = 0.8;  // m/s — backing-up speed cap (gentle)
    private static final double MAX_ROT       = 2.0;  // rad/s

    // ---- Proportional gains ----
    private static final double KP_FWD  = 1.2;   // m error → m/s  (approach)
    private static final double KP_BACK = 1.0;   // m error → m/s  (back-up)
    private static final double KP_ROT  = 3.0;   // rad error → rad/s

    // ---- Slew-rate limits (per 20 ms cycle) ----
    private static final double MAX_FWD_DELTA = 0.12;   // m/s per cycle  (≈6 m/s²)
    private static final double MAX_ROT_DELTA = 0.14;   // rad/s per cycle

    // ---- Finish tolerances ----
    private static final double DIST_TOL      = 0.25;  // metres — how far past band edges is OK
    private static final double ANGLE_TOL_DEG = 2.5;   // degrees — tag centred in camera
    private static final int    REQUIRED_STABLE = 5;    // consecutive OK cycles

    // ---- Safety ----
    private static final int MAX_CYCLES = 500;  // 10 s timeout

    // ---- Runtime state ----
    private double prevFwd, prevRot;
    private int stableCount;
    private int cycles;

    // ================== Constructors ==================

    /** Default: tag 10, max radius 3.0 m. */
    public DriveToTag(Swerve swerve, Vision2025 vision) {
        this(swerve, vision, 3.0, 10);
    }

    /** Specify max radius, default tag 10. */
    public DriveToTag(Swerve swerve, Vision2025 vision, double maxRadiusMeters) {
        this(swerve, vision, maxRadiusMeters, 10);
    }

    /** Full constructor — max radius + tag ID (-1 = best visible). */
    public DriveToTag(Swerve swerve, Vision2025 vision, double maxRadiusMeters, int tagId) {
        this.swerve    = swerve;
        this.vision    = vision;
        this.maxRadius = maxRadiusMeters;
        this.tagId     = tagId;
        addRequirements(swerve);
    }

    // ================== Command lifecycle ==================

    @Override
    public void initialize() {
        prevFwd = 0;
        prevRot = 0;
        stableCount = 0;
        cycles = 0;
        swerve.setFieldRelative(false);
        ElasticDashboard.putBoolean("DriveToTag/Active", true);
        ElasticDashboard.putNumber("DriveToTag/maxRadius", maxRadius);
        ElasticDashboard.putNumber("DriveToTag/tagId", tagId);
    }

    @Override
    public void execute() {
        cycles++;

        // ---- Get tag transform ----
        Optional<Transform3d> opt = (tagId >= 0)
                ? vision.getTransformToTag(tagId)
                : vision.getTargetTransform();

        if (!opt.isPresent()) {
            // No tag visible — coast to zero smoothly
            prevFwd = slewLimit(0, prevFwd, MAX_FWD_DELTA);
            prevRot = slewLimit(0, prevRot, MAX_ROT_DELTA);
            swerve.drive(new Translation2d(prevFwd, 0), prevRot);
            stableCount = 0;
            ElasticDashboard.putString("DriveToTag/Status", "NO_TAG");
            return;
        }

        Transform3d t = opt.get();
        double tagX = t.getX();   // forward distance  (positive = in front)
        double tagY = t.getY();   // lateral offset    (positive = tag is left)

        // True range to the tag (hypotenuse, not just forward component)
        double range = Math.hypot(tagX, tagY);

        // ---- Rotation: centre the tag in camera ----
        // bearingRad > 0 means tag is LEFT of center → rotate LEFT (positive) to center
        double angleRad = Math.atan2(tagY, tagX);
        double rotCmd   = clamp(angleRad * KP_ROT, -MAX_ROT, MAX_ROT);

        // ---- Forward: maintain distance within band [MIN_RADIUS .. maxRadius] ----
        double fwdCmd = 0;
        if (range > maxRadius) {
            // Too far — drive forward, fast at first, slower near the band edge
            double distErr = range - maxRadius;                    // positive
            double rawSpeed = distErr * KP_FWD;
            double cap = (distErr < SLOW_ZONE) ? SLOW_FWD : MAX_FWD;
            fwdCmd = clamp(rawSpeed, 0, cap);
        } else if (range < MIN_RADIUS) {
            // Too close — back up gently
            double distErr = MIN_RADIUS - range;                   // positive
            double rawSpeed = distErr * KP_BACK;
            fwdCmd = -clamp(rawSpeed, 0, MAX_BACK);               // negative = backwards
        }
        // else: inside band → fwdCmd stays 0, only rotate

        // ---- Slew-rate limiting ----
        fwdCmd = slewLimit(fwdCmd, prevFwd, MAX_FWD_DELTA);
        rotCmd = slewLimit(rotCmd, prevRot, MAX_ROT_DELTA);
        prevFwd = fwdCmd;
        prevRot = rotCmd;

        // ---- Drive (no strafe) ----
        swerve.drive(new Translation2d(fwdCmd, 0), rotCmd);

        // ---- Telemetry ----
        ElasticDashboard.putNumber("DriveToTag/range", range);
        ElasticDashboard.putNumber("DriveToTag/tagX", tagX);
        ElasticDashboard.putNumber("DriveToTag/tagY", tagY);
        ElasticDashboard.putNumber("DriveToTag/angleDeg", Math.toDegrees(angleRad));
        ElasticDashboard.putNumber("DriveToTag/fwdCmd", fwdCmd);
        ElasticDashboard.putNumber("DriveToTag/rotCmd", rotCmd);
        ElasticDashboard.putString("DriveToTag/Status", "DRIVING");

        // ---- Stable check: inside band AND tag centred ----
        boolean distOk  = range >= (MIN_RADIUS - DIST_TOL) && range <= (maxRadius + DIST_TOL);
        boolean angleOk = Math.abs(Math.toDegrees(angleRad)) <= ANGLE_TOL_DEG;

        if (distOk && angleOk) {
            stableCount++;
        } else {
            stableCount = 0;
        }
    }

    @Override
    public boolean isFinished() {
        return stableCount >= REQUIRED_STABLE || cycles > MAX_CYCLES;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0);
        swerve.setFieldRelative(true);
    ElasticDashboard.putBoolean("DriveToTag/Active", false);
    ElasticDashboard.putString("DriveToTag/Status", interrupted ? "INTERRUPTED" : "COMPLETE");
    }

    // ---- Small helpers ----
    /**
     * Clamp value to [min, max].
     */
    private static double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    /**
     * Simple slew-rate limiter: move previous value toward target by at most
     * maxDelta (absolute) and return the new value.
     */
    private static double slewLimit(double target, double prev, double maxDelta) {
        double delta = target - prev;
        if (Math.abs(delta) > maxDelta) {
            delta = Math.signum(delta) * maxDelta;
        }
        return prev + delta;
    }
}
