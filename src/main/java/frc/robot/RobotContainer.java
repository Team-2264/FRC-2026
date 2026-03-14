// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.ArcDrive;
import frc.robot.commands.DriveAndFaceGoal;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.FaceGoal;
import frc.robot.commands.ShootBall;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ToggleTurbo;
import frc.robot.commands.locks.LockSwerve;
import frc.robot.commands.locks.UnlockSwerve;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
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

import java.util.Optional;
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
    public final Intake intake = new Intake();
    private final Hopper hopper = new Hopper();

    // Controllers
    private final CommandPS4Controller controller = new CommandPS4Controller(Constants.Operator.controllerPort);
    //private final CommandPS4Controller controller2 = new CommandPS4Controller(Constants.Operator.controller2Port);

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

        // Triangle - HOLD to shoot (vision-independent now)
        controller.L1().whileTrue(new ShootBall(shooter, hopper, vision));

        controller.povUp().onTrue(new InstantCommand(() -> hopper.startConveyor(0.6)));
        controller.povDown().onTrue(new InstantCommand(() -> hopper.stopConveyor()));

        controller.R1().onTrue(new InstantCommand(() -> intake.startRoller(0.9))
            .alongWith(new InstantCommand(() -> intake.deployIntake())));
        controller.R1().onFalse(new InstantCommand(() -> intake.stopRoller())
            .alongWith(new InstantCommand(() -> intake.retractIntake())));

        // D-pad LEFT - DriveAndFaceGoal: face goal, drive to tag, align + re-center
        DriveAndFaceGoal driveAndFaceCommand = new DriveAndFaceGoal(swerve, vision);
        controller.pov(270).toggleOnTrue(driveAndFaceCommand);

        controller.square().onTrue(new InstantCommand(() -> intake.deployManualMove(Constants.IntakeConfig.Deploy.motorOpenLoopMoveVolts)));
        controller.square().onFalse(new InstantCommand(() -> intake.deployManualMove(0)));
        controller.circle().onTrue(new InstantCommand(() -> intake.deployManualMove(-Constants.IntakeConfig.Deploy.motorOpenLoopMoveVolts)));
        controller.circle().onFalse(new InstantCommand(() -> intake.deployManualMove(0)));
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
        }

        // Debug: show whether a transient LED override is active
        long now = System.currentTimeMillis();
        long remainingMs = Math.max(0, ledOverrideUntil - now);
        ElasticDashboard.putNumber("LEDs/OverrideRemainingMs", remainingMs);

        // If a transient LED override is active, honor it first — but allow
        // Tag 10 progressive green to bypass the override so the driver sees
        // detection/centering feedback even during short confirmations.
        Optional<edu.wpi.first.math.geometry.Transform3d> tag10Tf = vision.getTransformToTag(10);
        boolean bypassOverrideForTag10 = tag10Tf.isPresent();
        if (System.currentTimeMillis() < ledOverrideUntil && !bypassOverrideForTag10) {
            leds.setRGB(ledOverrideR, ledOverrideG, ledOverrideB);
            return;
        }

        // Prefer Tag 10 progressive green if Tag 10 is visible at all (even if
        // Photon reports another 'best' target). This prevents tag 9 from
        // overriding tag 10 centering feedback.
        if (tag10Tf.isPresent()) {
            try {
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
            // Fallback: use best-target logic (red for 12, purple otherwise)
            Optional<Integer> tagId = vision.getBestTargetId();
            if (tagId.isPresent()) {
                int id = tagId.get();
                ElasticDashboard.putNumber("LEDs/BestTagId", id);
                if (id == 12) {
                    leds.setRGBRange(255, 0, 0, 0, 3); // red on top 4 LEDs only
                } else {
                    leds.setRGB(128, 0, 128); // purple for any other tag
                }
            } else {
                leds.setRGB(128, 0, 128); // purple when no tag visible
            }
        }

        // hub alignment is triggered by the DriveToTag10 binding (on completion)
    }

    public Leds getLeds() {
        return leds;
    }

    public CommandPS4Controller getDriverController() {
        return controller;
    }

    /**
     * Request a transient LED override color for a duration in seconds.
     */
    // private void setLedOverride(int r, int g, int b, double durationSeconds) {
    //     ledOverrideR = r;
    //     ledOverrideG = g;
    //     ledOverrideB = b;
    //     ledOverrideUntil = System.currentTimeMillis() + (long) (durationSeconds * 1000.0);
    // }
}