// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AlignToShoot;
import frc.robot.commands.ArcDrive;
import frc.robot.commands.DriveAndFaceGoal;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.FaceGoal;
import frc.robot.commands.ShootBall;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ToggleTurbo;
import frc.robot.commands.locks.LockSwerve;
import frc.robot.commands.locks.UnlockSwerve;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;
import frc.robot.subsystems.ManualMotors;
import frc.robot.commands.RotateAndSpinWhileHeld;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.ElasticDashboard;
import edu.wpi.first.wpilibj.RobotController;

import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.photonvision.EstimatedRobotPose;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared.
 */
public class RobotContainer {
    // Subsystems
    public final Swerve swerve = new Swerve();
    public final Leds leds = new Leds(Constants.LedStrip.pwmPort, Constants.LedStrip.numLeds,
            Constants.LedStrip.scaleFactor);
    public final Vision2025 vision = new Vision2025();
    public final Shooter shooter = new Shooter();
    // Manual motors: rotator (CAN 23) and spinner (CAN 24) — updated CAN IDs
    public final ManualMotors manualMotors = new ManualMotors(23, 24);

    // Controllers
    private final CommandPS4Controller controller = new CommandPS4Controller(Constants.Operator.controllerPort);
    private final CommandPS4Controller controller2 = new CommandPS4Controller(Constants.Operator.controller2Port);

    // transient LED override (millis since epoch)
    private long ledOverrideUntil = 0;
    private int ledOverrideR = 0;
    private int ledOverrideG = 0;
    private int ledOverrideB = 0;
    private boolean hadVisibleTag = false;
    private long visionLostBlinkUntilMs = 0;
    private long visionPosePulseUntilMs = 0;
    private double lastVisionPoseTimestampSec = -1.0;

    // Autonomous
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("lockSwerve", new LockSwerve(swerve));
        NamedCommands.registerCommand("unlockSwerve", new UnlockSwerve(swerve));

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        ElasticDashboard.putData("Auto Mode", autoChooser);
    }

    /**
     * Configures all button bindings.
     */
    private void configureBindings() {
        // ======== Swerve ========
        swerve.setDefaultCommand(new TeleopSwerve(swerve, controller));
        controller.options().onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        controller.share().onTrue(new ToggleTurbo(swerve));
        controller.share().onFalse(new ToggleTurbo(swerve));

        // ======== Face Buttons ========

        // Triangle - HOLD to shoot (vision-independent now)
        controller.triangle().whileTrue(new ShootBall(shooter));

        // Cross (X) - currently unused
        // (AlignToShoot moved to left D-pad)

        // Square - Press once to drive within 3.0m of tag 10 and center it in camera
        DriveToTag driveToTagCommand = new DriveToTag(swerve, vision, 3.0, 10);
        controller.square().onTrue(driveToTagCommand);

    // Cross (X) - while held: rotate CAN-1 +30° and spin CAN-2 at 0.3.
    controller.cross().whileTrue(new RotateAndSpinWhileHeld(manualMotors, 0.3, 30.0));

        // Circle - Press to CANCEL DriveToTag
        controller.circle().onTrue(new InstantCommand(() -> {
            edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().cancel(driveToTagCommand);
        }));

        // ======== D-Pad ========

        // D-pad DOWN - Face Goal: rotate to face the reef center from any position
        controller.pov(180).onTrue(new FaceGoal(swerve));

        // D-pad LEFT - DriveAndFaceGoal: face goal, drive to tag, align + re-center
        // Press to start, press again to cancel (toggleOnTrue)
        DriveAndFaceGoal driveAndFaceCommand = new DriveAndFaceGoal(swerve, vision);
        controller.pov(270).toggleOnTrue(driveAndFaceCommand);

        // ======== Triggers ========

        // L2 (Left Trigger) - Arc Drive: while held, orbit tag 10 at current radius
        // Left stick X moves along the arc. Robot always faces tag.
        controller.L2().whileTrue(new ArcDrive(swerve, vision, controller));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Called every robot loop in both autonomous and teleop.
     */
    public void robotPeriodic() {
        vision.get();
        Optional<EstimatedRobotPose> visionPose = vision.getEstimatedPose();
        ElasticDashboard.putBoolean("vision pose", visionPose.isPresent());

        if (visionPose.isPresent()) {
            swerve.addVisionMeasurement(visionPose.get());

            double ts = visionPose.get().timestampSeconds;
            if (Math.abs(ts - lastVisionPoseTimestampSec) > 1e-6) {
                lastVisionPoseTimestampSec = ts;
                visionPosePulseUntilMs = System.currentTimeMillis() + 120;
            }
        }

        // Debug: show whether a transient LED override is active
        long now = System.currentTimeMillis();
        long remainingMs = Math.max(0, ledOverrideUntil - now);
        ElasticDashboard.putNumber("LEDs/OverrideRemainingMs", remainingMs);
        ElasticDashboard.putString("LEDs/Priority",
            "OVERRIDE > BATTERY > ENDGAME > SHOOTER > TURBO > VISION_LOST > VISION_POSE > DISABLED > TAG10 > IDLE");

        Optional<edu.wpi.first.math.geometry.Transform3d> tag10Tf = vision.getTransformToTag(10);
        Optional<Integer> bestTagId = vision.getBestTargetId();

        boolean hasAnyVisibleTag = tag10Tf.isPresent() || bestTagId.isPresent();
        if (hasAnyVisibleTag) {
            hadVisibleTag = true;
        } else if (hadVisibleTag) {
            hadVisibleTag = false;
            visionLostBlinkUntilMs = now + 600;
        }

        // Priority 1: explicit override always wins, including when Tag 10 is visible.
        boolean overrideActive = now < ledOverrideUntil;
        if (overrideActive) {
            ElasticDashboard.putString("LEDs/Mode", "OVERRIDE");
            leds.setRGB(ledOverrideR, ledOverrideG, ledOverrideB);
            return;
        }

        // Priority 2: low battery warning.
        double batteryVolts = RobotController.getBatteryVoltage();
        ElasticDashboard.putNumber("LEDs/BatteryVolts", batteryVolts);
        if (batteryVolts < 11.8) {
            ElasticDashboard.putString("LEDs/Mode", "LOW_BATTERY");
            boolean pulseOn = ((now / 200) % 2) == 0;
            leds.setRGB(pulseOn ? 255 : 0, 0, 0);
            return;
        }

        // Priority 3: endgame warning in teleop (<=30s remaining).
        double matchTime = DriverStation.getMatchTime();
        ElasticDashboard.putNumber("LEDs/MatchTimeSec", matchTime);
        if (DriverStation.isTeleopEnabled() && matchTime > 0.0 && matchTime <= 30.0) {
            ElasticDashboard.putString("LEDs/Mode", "ENDGAME");
            boolean flashOn = ((now / 150) % 2) == 0;
            leds.setRGB(flashOn ? 255 : 0, flashOn ? 180 : 0, 0);
            return;
        }

        // Priority 4: shooter active.
        if (shooter.isShooting()) {
            ElasticDashboard.putString("LEDs/Mode", "SHOOTER_ACTIVE");
            leds.setRGB(255, 80, 0);
            return;
        }

        // Priority 5: turbo mode active.
        if (swerve.getTurboStatus()) {
            ElasticDashboard.putString("LEDs/Mode", "TURBO");
            leds.setRGB(0, 255, 255);
            return;
        }

        // Priority 6: vision-lost warning blink after losing all tags.
        if (now < visionLostBlinkUntilMs) {
            ElasticDashboard.putString("LEDs/Mode", "VISION_LOST");
            boolean blinkOn = ((now / 120) % 2) == 0;
            leds.setRGB(blinkOn ? 255 : 0, blinkOn ? 120 : 0, 0);
            return;
        }

        // Priority 7: pulse when a new vision pose measurement is accepted.
        if (now < visionPosePulseUntilMs) {
            ElasticDashboard.putString("LEDs/Mode", "VISION_POSE");
            leds.setRGB(0, 80, 255);
            return;
        }

        // Priority 8: disabled standby pattern.
        if (DriverStation.isDisabled()) {
            ElasticDashboard.putString("LEDs/Mode", "DISABLED_BREATHE");
            double phase = (now % 2000L) / 2000.0;
            double wave = 0.5 * (1.0 + Math.sin(2.0 * Math.PI * phase));
            int v = 10 + (int) Math.round(45.0 * wave);
            leds.setRGB(v, v, v);
            return;
        }

        // Priority 9: Tag 10 progressive centering feedback.
        if (tag10Tf.isPresent()) {
            try {
                ElasticDashboard.putString("LEDs/Mode", "TAG10_PROGRESS");
                var tf = tag10Tf.get();
                // Debug both rotation-based yaw and translation-based bearing.
                double yawDegRot = Math.toDegrees(tf.getRotation().getZ());
                double tx = tf.getTranslation().getX();
                double ty = tf.getTranslation().getY();
                double yawDegXY = Math.toDegrees(Math.atan2(ty, tx));
                double yawDeg = Math.abs(yawDegXY); // prefer translation-based bearing
                ElasticDashboard.putNumber("LEDs/Tag10YawDeg_Rot", Math.abs(yawDegRot));
                ElasticDashboard.putNumber("LEDs/Tag10YawDeg_XY", yawDegXY);
                ElasticDashboard.putNumber("LEDs/Tag10DistMeters", tx);
                ElasticDashboard.putNumber("LEDs/Tag10YawDeg", yawDeg);

                final double MAX_YAW_DEG = 15.0; // adjust as needed
                double frac = 1.0 - Math.min(yawDeg / MAX_YAW_DEG, 1.0);
                int numLeds = Constants.LedStrip.numLeds;
                int lit = (int) Math.round(frac * numLeds);
                ElasticDashboard.putNumber("LEDs/Tag10Lit", lit);
                if (lit <= 0) {
                    int center = numLeds / 2;
                    leds.setRGBRange(0, 255, 0, center, center);
                } else if (lit >= numLeds) {
                    leds.setRGB(0, 255, 0);
                } else {
                    int center = numLeds / 2;
                    int half = lit / 2;
                    int start = center - half;
                    int end = start + lit - 1;
                    if (start < 0)
                        start = 0;
                    if (end >= numLeds)
                        end = numLeds - 1;
                    leds.setRGBRange(0, 255, 0, start, end);
                }
            } catch (Exception e) {
                ElasticDashboard.putString("LEDs/Error", e.toString());
                leds.setRGB(0, 255, 0);
            }
        } else {
            // Priority 5: no non-Tag10 indicator requested; fall through to idle.
            if (bestTagId.isPresent()) {
                int id = bestTagId.get();
                ElasticDashboard.putNumber("LEDs/BestTagId", id);
            }
            ElasticDashboard.putString("LEDs/Mode", "IDLE");
            leds.setRGB(0, 0, 0);
        }

        // hub alignment is triggered by the DriveToTag10 binding (on completion)
    }

    public Leds getLeds() {
        return leds;
    }

    /**
     * Request a transient LED override color for a duration in seconds.
     */
    private void setLedOverride(int r, int g, int b, double durationSeconds) {
        ledOverrideR = r;
        ledOverrideG = g;
        ledOverrideB = b;
        ledOverrideUntil = System.currentTimeMillis() + (long) (durationSeconds * 1000.0);
    }

    public void requestLedOverride(int r, int g, int b, double durationSeconds) {
        setLedOverride(r, g, b, durationSeconds);
    }

    public void clearLedOverride() {
        ledOverrideUntil = 0;
    }

    public boolean isLedOverrideActive() {
        return System.currentTimeMillis() < ledOverrideUntil;
    }
}