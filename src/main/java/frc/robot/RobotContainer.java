// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AlignToShoot;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    // Controllers
    private final CommandPS4Controller controller = new CommandPS4Controller(Constants.Operator.controllerPort);
    private final CommandPS4Controller controller2 = new CommandPS4Controller(Constants.Operator.controller2Port);

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

        // Cross (X) - currently unused
        // (AlignToShoot moved to left D-pad)

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

        // D-pad LEFT - AlignToShoot: strafe from coral station to reef center, then align
        //              Press to start, press again to cancel (toggleOnTrue)
        AlignToShoot alignToShootCommand = new AlignToShoot(swerve, vision);
        controller.pov(270).toggleOnTrue(alignToShootCommand);
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

        // LED based on visible tag ID (2026 field layout)
        // Coral station tags (7,12 Red / 17,22,23,28 Blue) = red (top 4 LEDs)
        // Center reef tags (9,10 Red / 19,20 Blue) = green (all LEDs)
        // Any other tag = purple
        Optional<Integer> tagId = vision.getBestTargetId();
        if (tagId.isPresent()) {
            int id = tagId.get();
            if (id == 7 || id == 12 || id == 17 || id == 22 || id == 23 || id == 28) {
                leds.setRGBRange(255, 0, 0, 0, 3); // red on top 4 LEDs — coral station
            } else if (id == 9 || id == 10 || id == 19 || id == 20) {
                leds.setRGB(0, 255, 0); // green all LEDs — reef center (shooting position)
            } else {
                leds.setRGB(128, 0, 128); // purple for any other tag
            }
        } else {
            leds.setRGB(128, 0, 128); // purple when no tag visible
        }
    }

    public Leds getLeds() {
        return leds;
    }
}