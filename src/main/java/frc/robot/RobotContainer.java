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
    // private final CommandPS4Controller controller2 = new
    // CommandPS4Controller(Constants.Operator.controller2Port);

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

        // controller.R1().onTrue(new InstantCommand(() -> intake.startRoller(0.9))
        // .alongWith(new InstantCommand(() -> intake.deployIntake())));
        // controller.R1().onFalse(new InstantCommand(() -> intake.stopRoller())
        // .alongWith(new InstantCommand(() -> intake.retractIntake())));
        controller.R1().onTrue(new InstantCommand(() -> intake.deployIntake()));
        controller.R1().onFalse(new InstantCommand(() -> intake.retractIntake()));
        controller.R2().onTrue(new InstantCommand(() -> intake.startRoller(1.0)));
        controller.R2().onFalse(new InstantCommand(() -> intake.stopRoller()));

        controller.triangle().onTrue(new InstantCommand(() -> shooter.shootAtSpeedFromDashboard()));
        controller.triangle().onFalse(new InstantCommand(() -> shooter.stop()));

        // D-pad LEFT - DriveAndFaceGoal: face goal, drive to tag, align + re-center
        DriveAndFaceGoal driveAndFaceCommand = new DriveAndFaceGoal(swerve, vision);
        controller.pov(270).toggleOnTrue(driveAndFaceCommand);

        controller.square().onTrue(new InstantCommand(
                () -> intake.deployManualMove(Constants.IntakeConfig.Deploy.motorOpenLoopMoveVolts)));
        controller.square().onFalse(new InstantCommand(() -> intake.deployManualMove(0)));
        controller.circle().onTrue(new InstantCommand(
                () -> intake.deployManualMove(-Constants.IntakeConfig.Deploy.motorOpenLoopMoveVolts)));
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
        // progressive green for hub-facing tags (10 or 26) to bypass so the driver sees
        // detection/centering feedback even during short confirmations.
        Optional<edu.wpi.first.math.geometry.Transform3d> tagTf = vision.getTransformToAny(10, 26);
        boolean bypassOverrideForHubTag = tagTf.isPresent();
        if (System.currentTimeMillis() < ledOverrideUntil && !bypassOverrideForHubTag) {
            leds.setRGB(ledOverrideR, ledOverrideG, ledOverrideB);
            return;
        }

        // Prefer hub tag progressive green if a hub tag is visible at all (even if
        // Photon reports another 'best' target). This prevents tag 9 from
        // overriding hub centering feedback.
        if (tagTf.isPresent()) {
            try {
                edu.wpi.first.math.geometry.Pose2d robotPose = swerve.getPose();
                double hubFieldX = 0;
                double hubFieldY = 0;

                // Choose nearest hub tag (10 or 26) from field layout
                int[] hubTags = { 10, 26 };
                int chosenTag = 10;
                double bestDist = Double.POSITIVE_INFINITY;
                for (int id : hubTags) {
                    Optional<edu.wpi.first.math.geometry.Pose3d> poseOpt = vision.getTagPose(id);
                    if (poseOpt.isEmpty())
                        continue;
                    edu.wpi.first.math.geometry.Pose2d tagPose2d = poseOpt.get().toPose2d();
                    double dist = robotPose.getTranslation().getDistance(tagPose2d.getTranslation());
                    if (dist < bestDist) {
                        bestDist = dist;
                        chosenTag = id;
                    }
                }

                Optional<edu.wpi.first.math.geometry.Pose3d> exactTagPose = vision.getTagPose(chosenTag);
                if (exactTagPose.isPresent()) {
                    edu.wpi.first.math.geometry.Pose3d tagPose = exactTagPose.get();
                    edu.wpi.first.math.geometry.Pose3d hubPose = tagPose.transformBy(
                            new edu.wpi.first.math.geometry.Transform3d(
                                    new edu.wpi.first.math.geometry.Translation3d(
                                            edu.wpi.first.math.util.Units.inchesToMeters(-23.5), 0, 0),
                                    new edu.wpi.first.math.geometry.Rotation3d()));
                    hubFieldX = hubPose.getX();
                    hubFieldY = hubPose.getY();
                }

                double hubDx = hubFieldX - robotPose.getX();
                double hubDy = hubFieldY - robotPose.getY();
                double targetAngleRad = Math.atan2(hubDy, hubDx);

                // Wrap the bearing to [-pi, pi]
                double bearingRad = targetAngleRad - robotPose.getRotation().getRadians();
                while (bearingRad > Math.PI)
                    bearingRad -= 2 * Math.PI;
                while (bearingRad < -Math.PI)
                    bearingRad += 2 * Math.PI;

                double yawDeg = Math.abs(Math.toDegrees(bearingRad));

                ElasticDashboard.putNumber("LEDs/HubYawDeg", yawDeg);

                final double MAX_YAW_DEG = 15.0; // adjust as needed
                double frac = 1.0 - Math.min(yawDeg / MAX_YAW_DEG, 1.0);
                int numLeds = Constants.LedStrip.numLeds;
                int lit = (int) Math.round(frac * numLeds);
                ElasticDashboard.putNumber("LEDs/HubLit", lit);
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
    // ledOverrideR = r;
    // ledOverrideG = g;
    // ledOverrideB = b;
    // ledOverrideUntil = System.currentTimeMillis() + (long) (durationSeconds *
    // 1000.0);
    // }
}