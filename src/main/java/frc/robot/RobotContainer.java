// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AlignToTag;
import frc.robot.commands.AlignHeadingToTag10;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ToggleTurbo;
import frc.robot.commands.locks.LockSwerve;
import frc.robot.commands.locks.UnlockSwerve;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.subsystems.CoralArm;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    public final Swerve swerve = new Swerve();
    public final Leds leds = new Leds(Constants.LedStrip.pwmPort, Constants.LedStrip.numLeds,
            Constants.LedStrip.scaleFactor);
    public final Vision2025 vision = new Vision2025();
    private final CoralArm m_intake = new CoralArm(this);

    // Controllers
    private final CommandPS4Controller controller = new CommandPS4Controller(Constants.Operator.controllerPort);

    // Autonomous
    private final SendableChooser<Command> autoChooser;

    // vision trigger
    private Trigger hasVisionTarget = new Trigger(() -> vision.getEstimatedPose().isPresent());

    // LED override (transient confirmations) — timestamp in ms until which override
    // is active
    private volatile long ledOverrideUntil = 0;
    private volatile int ledOverrideR = 0;
    private volatile int ledOverrideG = 0;
    private volatile int ledOverrideB = 0;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Register named commands for autonomous - pathplanner

        NamedCommands.registerCommand("lockSwerve", new LockSwerve(swerve));
        NamedCommands.registerCommand("unlockSwerve", new UnlockSwerve(swerve));

        // Configure the button bindings
        configureBindings();

        // Set up autonomous
        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);

    }

    /**
     * Configures all button bindings.
     */
    private void configureBindings() {
        // ======== Swerve ========
        swerve.setDefaultCommand(new TeleopSwerve(swerve, controller));
        /**
         * zero gyro with verification and LED/rumble confirmation
         * Option button is the small button on the right side of the controller
         */
        controller.options().onTrue(
                new InstantCommand(() -> swerve.zeroGyro())
                        .andThen(new WaitCommand(0.12))
                        .andThen(new InstantCommand(() -> {
                            double yawDeg = swerve.getGyroAngle().getDegrees();
                            if (Math.abs(yawDeg) < 3.0) {
                                // success
                                setLedOverride(0, 255, 255, 0.25); // cyan
                                controller.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0.4);
                                controller.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 0.4);
                                // stop rumble after short delay
                                new WaitCommand(0.2).andThen(new InstantCommand(() -> {
                                    controller.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                                    controller.getHID().setRumble(GenericHID.RumbleType.kRightRumble, 0.0);
                                })).schedule();
                            } else {
                                // failure
                                setLedOverride(255, 128, 0, 0.6); // amber
                                edu.wpi.first.wpilibj.DriverStation
                                        .reportWarning("Gyro zero check failed: yaw=" + yawDeg, false);
                            }
                        })));

        controller.share().onTrue(new ToggleTurbo(swerve));
        controller.share().onFalse(new ToggleTurbo(swerve));

        // DriveToTag - Press Square once, drives to tag and auto-stops 0.5m away
        controller.square().onTrue(new DriveToTag(swerve, vision));

        // AlignToTag - HOLD Circle to auto-align (rotate + strafe + drive to face tag)
        controller.circle().whileTrue(new AlignToTag(swerve, vision));

        // PS5 D-pad Up - HOLD to rotate until AprilTag 10 is centered (hub reference)
        controller.povUp().whileTrue(new AlignHeadingToTag10(swerve, vision));

        // on target acquired -> short green pulse + small rumble
        hasVisionTarget.onTrue(new InstantCommand(() -> {
            setLedOverride(0, 255, 0, 0.25); // green transient
            controller.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0.35);
        }).andThen(new WaitCommand(0.15)).andThen(new InstantCommand(() -> {
            controller.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
        })));

        // on target lost -> short red pulse
        hasVisionTarget.onFalse(new InstantCommand(() -> setLedOverride(255, 0, 0, 0.3)));

        // NOTE: don't set a default LED command (we control LEDs from robotPeriodic()
        // to enforce priorities)
    }

    /**
     * Returns the command to run in autonomous mode.
     *
     * @return the command to run in autonomous
     */
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

        // add vision pose to
        if (visionPose.isPresent()) {
            swerve.addVisionMeasurement(visionPose.get());
        }

        // --- LED priority handling ---
        long now = System.currentTimeMillis();
        // If a transient override is active, apply it
        if (now < ledOverrideUntil) {
            leds.setRGB(ledOverrideR, ledOverrideG, ledOverrideB);
            return;
        }

        // Low battery has highest priority for this set
        double battery = edu.wpi.first.wpilibj.RobotController.getBatteryVoltage();
        if (battery < 7.5) {
            leds.setRGB(255, 0, 0); // critical red
            return;
        }

        // Intake status
        try {
            var intakeStatus = m_intake.getIntakeStatus();
            if (intakeStatus == frc.robot.enums.CoralIntakeStatus.INTAKING) {
                leds.setRGB(0, 0, 255); // blue while intaking
                return;
            }
            if (m_intake.isCoralPresent()) {
                leds.setRGB(128, 0, 128); // purple when holding
                return;
            }
        } catch (Exception e) {
            // if intake subsystem not ready, continue to default
        }

        // Default team color
        leds.setRGB(255, 255, 0);

    }

    /**
     * Request a transient LED override color for a duration in seconds.
     * This is used for confirmations like gyro-zero or vision acquire.
     */
    private void setLedOverride(int r, int g, int b, double durationSeconds) {
        ledOverrideR = r;
        ledOverrideG = g;
        ledOverrideB = b;
        ledOverrideUntil = System.currentTimeMillis() + (long) (durationSeconds * 1000.0);
    }

    public Leds getLeds() {
        return leds;
    }

}