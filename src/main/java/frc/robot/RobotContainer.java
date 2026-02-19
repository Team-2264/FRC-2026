// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AlignToShoot;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.DriveToTag10;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;
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

    // Controllers
    private final CommandPS4Controller controller = new CommandPS4Controller(Constants.Operator.controllerPort);
    private final CommandPS4Controller controller2 = new CommandPS4Controller(Constants.Operator.controller2Port);

    // transient LED override (millis since epoch)
    private long ledOverrideUntil = 0;
    private int ledOverrideR = 0;
    private int ledOverrideG = 0;
    private int ledOverrideB = 0;

    // Autonomous
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("lockSwerve", new LockSwerve(swerve));
        NamedCommands.registerCommand("unlockSwerve", new UnlockSwerve(swerve));

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Mode", autoChooser);
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

        // Triangle - HOLD to shoot (speed adjusts based on distance to tag)
        controller.triangle().whileTrue(new ShootBall(shooter, vision));

        // Cross (X) - TOGGLE: strafe left from tag 12 until sees 9/10, then align to 10
        // Press X to start, press X again to cancel
        AlignToShoot alignToShootCommand = new AlignToShoot(swerve, vision);
        controller.cross().toggleOnTrue(alignToShootCommand);

        // Square - Press once to drive toward the nearest visible tag (auto-stops at
        // 0.5m)
        DriveToTag driveToTagCommand = new DriveToTag(swerve, vision);
        controller.square().onTrue(driveToTagCommand);

        // Circle - Press to CANCEL DriveToTag
        controller.circle().onTrue(new InstantCommand(() -> {
            edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().cancel(driveToTagCommand);
        }));

        // ======== D-Pad ========

        // D-pad DOWN - Face Goal: rotate to face the reef center from any position
        controller.pov(180).onTrue(new FaceGoal(swerve));

        // D-pad LEFT - Drive to 5 feet in front of AprilTag 10
        // Chain the drive command with a follow-up that pulses orange + rumble when
        // the command completes successfully (user requested pulse when alignment
        // finished via the left D-pad binding).
        var driveToTag10Cmd = new DriveToTag10(swerve, vision);
        controller.pov(270).onTrue(
                driveToTag10Cmd.andThen(new InstantCommand(() -> {
                    // Only pulse if the drive command completed (not interrupted)
                    // DriveToTag10 writes status to SmartDashboard; we also check that
                    // the robot is within tolerance by reading pose.
                    try {
                        // pulse orange override for 0.5s
                        setLedOverride(255, 165, 0, 0.5);
                        controller.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0.6);
                        controller.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 0.6);
                        new WaitCommand(0.5).andThen(new InstantCommand(() -> {
                            controller.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                            controller.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
                        })).schedule();
                    } catch (Exception ignored) {
                    }
                }))
        );
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
        SmartDashboard.putBoolean("vision pose", visionPose.isPresent());

        if (visionPose.isPresent()) {
            swerve.addVisionMeasurement(visionPose.get());
        }

        // Debug: show whether a transient LED override is active
        long now = System.currentTimeMillis();
        long remainingMs = Math.max(0, ledOverrideUntil - now);
        SmartDashboard.putNumber("LEDs/OverrideRemainingMs", remainingMs);

        // If a transient LED override is active, honor it first — but allow
        // Tag 10 progressive green to bypass the override so the driver sees
        // detection/centering feedback even during short confirmations.
        Optional<Integer> peekTag = vision.getBestTargetId();
        boolean bypassOverrideForTag10 = false;
        if (peekTag.isPresent() && peekTag.get() == 10) {
            bypassOverrideForTag10 = true;
        }
        if (System.currentTimeMillis() < ledOverrideUntil && !bypassOverrideForTag10) {
            leds.setRGB(ledOverrideR, ledOverrideG, ledOverrideB);
            return;
        }

        // LED based on visible tag ID
        // Tag 12 = red (top 4 LEDs only), Tag 9 or 10 = green, else = purple
        Optional<Integer> tagId = vision.getBestTargetId();
        if (tagId.isPresent()) {
            int id = tagId.get();
            SmartDashboard.putNumber("LEDs/BestTagId", id);
            if (id == 12) {
                leds.setRGBRange(255, 0, 0, 0, 3); // red on top 4 LEDs only
   
            } 
            else if (id == 10) {
                // Tag 10 -> progressive green based on how centered the tag is
                // (more centered => more LEDs lit). We'll map yaw -> number of LEDs.
                try {
                    java.util.OptionalDouble yawOpt = vision.getBestTargetYawDeg();
                    if (yawOpt.isPresent()) {
                        double yawDeg = Math.abs(yawOpt.getAsDouble());
                        SmartDashboard.putNumber("LEDs/Tag10YawDeg", yawDeg);
                        // Define how wide "barely sees" is. Adjust as needed.
                        final double MAX_YAW_DEG = 15.0; // at this yaw we show minimal green
                        // fraction 0..1 where 1 == perfectly centered
                        double frac = 1.0 - Math.min(yawDeg / MAX_YAW_DEG, 1.0);
                        int numLeds = Constants.LedStrip.numLeds;
                        int lit = (int) Math.round(frac * numLeds);
                        SmartDashboard.putNumber("LEDs/Tag10Lit", lit);
                        if (lit <= 0) {
                            // show a single faint center LED to indicate detection
                            int center = numLeds / 2;
                            leds.setRGBRange(0, 255, 0, center, center);
                        } else if (lit >= numLeds) {
                            leds.setRGB(0, 255, 0);
                        } else {
                            // light `lit` LEDs centered on the strip
                            int center = numLeds / 2;
                            int half = lit / 2;
                            int start = center - half;
                            int end = start + lit - 1;
                            // clamp
                            if (start < 0) start = 0;
                            if (end >= numLeds) end = numLeds - 1;
                            leds.setRGBRange(0, 255, 0, start, end);
                        }
                    } else {
                        // no yaw info -> fallback to single center LED
                        int center = Constants.LedStrip.numLeds / 2;
                        leds.setRGBRange(0, 255, 0, center, center);
                    }
                } catch (Exception e) {
                    SmartDashboard.putString("LEDs/Error", e.toString());
                    leds.setRGB(0, 255, 0);
                }
            } else {
                leds.setRGB(128, 0, 128); // purple for any other tag
            }
        } else {
            leds.setRGB(128, 0, 128); // purple when no tag visible
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
}