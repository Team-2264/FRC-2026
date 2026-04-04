// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveAndFaceGoal;
import frc.robot.commands.RetractAndStopIntake;
import frc.robot.commands.ShootBall;
import frc.robot.commands.ShootBallNoVision;
//import frc.robot.commands.DriveToTag;
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
        NamedCommands.registerCommand("AlignWithHubTag", new DriveAndFaceGoal(swerve, vision).withTimeout(2));
        NamedCommands.registerCommand("ShootBallFinal", new ShootBall(shooter, hopper, intake, vision).withTimeout(9.0));
        NamedCommands.registerCommand("ShootBallInitial", new ShootBall(shooter, hopper, intake, vision).withTimeout(4.0));
        //NamedCommands.registerCommand("IntakeOut", new InstantCommand(() -> intake.deployIntake()));
        //NamedCommands.registerCommand("IntakeIn", new InstantCommand(() -> intake.retractIntake()));
        //NamedCommands.registerCommand("RollerStart", new InstantCommand(() -> intake.startRoller(1.0)));
        //NamedCommands.registerCommand("RollerStop", new InstantCommand(() -> intake.stopRoller()));
        NamedCommands.registerCommand("ShooterSpinUp", new InstantCommand(() -> shooter.shoot(2500)));


        NamedCommands.registerCommand("InitiateIntake",new InstantCommand(() -> intake.startRoller(1.0))
            .alongWith(new InstantCommand(() -> intake.deployIntake())));
        NamedCommands.registerCommand("RetractAndStopIntake",new RetractAndStopIntake(intake));


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

        // L2 - HOLD to shoot (vision-independent now)
        controller.L1().whileTrue(new ShootBall(shooter, hopper, intake, vision));

        controller.L2().whileTrue(new InstantCommand(() -> {
            if(!controller.L1().getAsBoolean()){
                shooter.shoot(2500);
            }
        }));

        controller.L2().onFalse(new InstantCommand(() -> {
            if(!controller.L1().getAsBoolean()){
                shooter.stop();
            }
        }));

        controller.R1().onTrue(new InstantCommand(() -> intake.startRoller(1.0))
            .alongWith(new InstantCommand(() -> intake.deployIntake())));
        controller.R1().onFalse(new RetractAndStopIntake(intake));

        controller.R2().onTrue(new InstantCommand(() -> intake.startRoller(1.0)));
        controller.R2().onFalse(new InstantCommand(() -> intake.stopRoller()));

        // D-pad LEFT - DriveAndFaceGoal: face goal, drive to tag, align + re-center
        DriveAndFaceGoal driveAndFaceCommand = new DriveAndFaceGoal(swerve, vision);
        //DriveToTag driveToTag = new DriveToTag(swerve, vision);

        controller.povLeft().toggleOnTrue(driveAndFaceCommand);
        //controller.povUp().toggleOnTrue(driveToTag);


        // Codriver controller

        // R2 - Reverse the intake, hopper and feed motor in case we need to try 
        // and unjam a ball that is stuck between the feeder and shooter
        controller2.R2().onTrue(new InstantCommand(() -> intake.deployIntake())
            .alongWith(new InstantCommand(() -> intake.startRoller(-1.0)))
            .alongWith(new InstantCommand(() -> hopper.startConveyor(-1.0)))
            .alongWith(new InstantCommand(() -> shooter.startFeederReverse())));
        controller2.R2().onFalse(new InstantCommand(() -> shooter.stopFeeder())
            .alongWith(new InstantCommand(() -> hopper.stopConveyor()))
            .alongWith(new RetractAndStopIntake(intake)));

        controller2.povUp().onTrue(new InstantCommand(() -> intake.deployManualMove(Constants.IntakeConfig.Deploy.motorOpenLoopMoveVolts)));
        controller2.povUp().onFalse(new InstantCommand(() -> intake.deployManualMove(0)));
        controller2.povDown().onTrue(new InstantCommand(() -> intake.deployManualMove(-Constants.IntakeConfig.Deploy.motorOpenLoopMoveVolts)));
        controller2.povDown().onFalse(new InstantCommand(() -> intake.deployManualMove(0)));
        controller2.povLeft().onTrue(new InstantCommand(() -> intake.deployFindHome()));

        controller2.triangle().onTrue(new InstantCommand(() -> hopper.startConveyor(1.0)));
        controller2.triangle().onFalse(new InstantCommand(() -> hopper.stopConveyor()));
        controller2.cross().onTrue(new InstantCommand(() -> hopper.startConveyor(-1.0)));
        controller2.cross().onFalse(new InstantCommand(() -> hopper.stopConveyor()));

        controller2.L1().whileTrue(new ShootBallNoVision(shooter, hopper, intake));
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

        // If a transient LED override is active, honor it first - but allow
        // Hub tag progressive green to bypass the override so the driver sees
        // detection/centering feedback even during short confirmations.
        Optional<edu.wpi.first.math.geometry.Transform3d> hubTf = vision.getNearestHubTagTransform();
        boolean bypassOverrideForHub = hubTf.isPresent();

        if (System.currentTimeMillis() < ledOverrideUntil && !bypassOverrideForHub) {
            leds.setRGB(ledOverrideR, ledOverrideG, ledOverrideB);
            return;
        }

        // Prefer Hub tag progressive green if any Hub tag is visible at all (even if
        // Photon reports another 'best' target).
        if (hubTf.isPresent()) {
            try {
                edu.wpi.first.math.geometry.Pose2d robotPose = swerve.getPose();
                
                // Get the target pose directly
                Optional<edu.wpi.first.math.geometry.Pose2d> exactTagPose = vision.getNearestHubTargetPose(robotPose, 0);

                if (exactTagPose.isPresent()) {
                    edu.wpi.first.math.geometry.Pose2d tagPose = exactTagPose.get();
                    double hubDx = tagPose.getX() - robotPose.getX();
                    double hubDy = tagPose.getY() - robotPose.getY();
                    double targetAngleRad = Math.atan2(hubDy, hubDx);

                    // Wrap the bearing to [-pi, pi]
                    double bearingRad = targetAngleRad - robotPose.getRotation().getRadians();
                    while (bearingRad > Math.PI) bearingRad -= 2 * Math.PI;
                    while (bearingRad < -Math.PI) bearingRad += 2 * Math.PI;

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

        // hub alignment is triggered by the DriveToHub binding (on completion)
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