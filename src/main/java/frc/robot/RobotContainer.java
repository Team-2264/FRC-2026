// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AlignToShoot;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.ShootBall;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ToggleTurbo;
import frc.robot.commands.locks.LockSwerve;
import frc.robot.commands.locks.UnlockSwerve;
import frc.robot.enums.HeldButton;
import frc.robot.subsystems.HeldButtons;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision2025;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.lang.annotation.ElementType;
import java.util.Map;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


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
    public final Leds leds = new Leds(Constants.LedStrip.pwmPort, Constants.LedStrip.numLeds, Constants.LedStrip.scaleFactor);
    public final Vision2025 vision = new Vision2025();
    public final Shooter shooter = new Shooter();  // Shooter with Neo motor (CAN ID 20)
    private final HeldButtons heldButtons = new HeldButtons();

    // Controllers
    private final CommandPS4Controller controller = new CommandPS4Controller(Constants.Operator.controllerPort);
    private final CommandPS4Controller controller2 = new CommandPS4Controller(Constants.Operator.controller2Port);

    // Autonomous
    private final SendableChooser<Command> autoChooser;

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
        controller.options().onTrue( new InstantCommand(() -> swerve.zeroGyro()));

        controller.share().onTrue(new ToggleTurbo(swerve));
        controller.share().onFalse(new ToggleTurbo(swerve)); 

        // ======== Shooter ========
        // Triangle button - HOLD to shoot (speed adjusts based on distance to tag)
        controller.triangle().whileTrue(new ShootBall(shooter, vision));
        
        // X button - Press to align (rotates until tag is centered, then stops)
        controller.cross().onTrue(new AlignToShoot(swerve, vision));

        // ======== DriveToTag ========
        // Create the DriveToTag command so we can reference it for cancellation
        DriveToTag driveToTagCommand = new DriveToTag(swerve, vision);
        
        // Square button - Press to drive toward the nearest visible tag (auto-stops at 0.5m)
        controller.square().onTrue(driveToTagCommand);

        // Circle button - Press to CANCEL DriveToTag and return control to driver
        controller.circle().onTrue(new InstantCommand(() -> {
            edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().cancel(driveToTagCommand);
        }));
    }
        // on target acquired -> short green pulse + small rumble
        hasVisionTarget.onTrue(new InstantCommand(() -> {
            setLedOverride(0, 255, 0, 1.0); // green transient (1s)
            controller.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0.35);
        }).andThen(new WaitCommand(0.15)).andThen(new InstantCommand(() -> {
            controller.getHID().setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
        })));

        // on target lost -> short red pulse
        hasVisionTarget.onFalse(new InstantCommand(() -> setLedOverride(255, 0, 0, 0.3)));

        // NOTE: don't set a default LED command (we control LEDs from robotPeriodic() to enforce priorities)
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
        if(visionPose.isPresent()) {
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

        // Vision has next priority: if we have an estimated pose, show green while present
        if (visionPose.isPresent()) {
            leds.setRGB(0, 255, 0);
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
        ledOverrideUntil = System.currentTimeMillis() + (long)(durationSeconds * 1000.0);
    }

    public Leds getLeds() {
        return leds;
    }

}