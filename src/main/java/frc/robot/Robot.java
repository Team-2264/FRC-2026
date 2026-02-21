// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.ElasticDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.PS5Controller;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;
    BooleanLogEntry myBooleanLog;
    DoubleLogEntry myBatteryLog;
    StringLogEntry myStringLog;
    Alert alertTest = new Alert("This is a test!", AlertType.kError);
    DoubleLogEntry joystickLog;
    PS5Controller driver = new PS5Controller(0);
    DoubleLogEntry joystickLog;
    PS5Controller driver = new PS5Controller(0);

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // ── Start WPILib data logging (writes .wpilog files to USB / roboRIO) ──
        DataLogManager.start();
        DataLogManager.log("Testing Wayzata");

        // ── Instantiate robot container (subsystems, commands, button bindings) ──
        m_robotContainer = new RobotContainer();

        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        alertTest.set(true);

        // Custom log entries
        DataLogManager.start();
        DataLog log = DataLogManager.getLog();
        myBooleanLog = new BooleanLogEntry(log, "/my/boolean");
        myBatteryLog = new DoubleLogEntry(log, "/my/BatteryVoltage");
        myStringLog = new StringLogEntry(log, "/my/string");
        joystickLog = new DoubleLogEntry(log, "/driver/leftStick");

        addPeriodic(() -> {
            myBatteryLog.update(RobotController.getBatteryVoltage(), RobotController.getFPGATime());
        }, 2.00, 0.00);
        DataLogManager.start();
        DataLog Log = DataLogManager.getLog();
        joystickLog = new DoubleLogEntry(Log, "/driver/leftStick");

    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and
     * SmartDashboard integrated updating.
     */
    long oldMicrosecondTimestamp = RobotController.getFPGATime();

    @Override
    public void robotPeriodic() {
        // ── Vision pipeline FIRST — so pose corrections are applied before Swerve
        // publishes ──
        // This eliminates the 1-cycle (20ms) lag where Swerve.periodic() would publish
        // a stale pose because vision hadn't been injected yet this cycle.
        try {
            if (m_robotContainer != null) {
                m_robotContainer.robotPeriodic();
            }
        } catch (Exception e) {
            ElasticDashboard.putString("Robot/visionPeriodic Error", e.toString());
        }

        // ── Then run the scheduler (which calls subsystem periodics → Swerve
        // publishes) ──
        try {
            CommandScheduler.getInstance().run();
        } catch (Exception e) {
            ElasticDashboard.putString("Robot/robotPeriodic Error", e.toString());
        }

        /*
         * double batteryVoltage = RobotController.getBatteryVoltage();
         * long microsecondTimestamp = RobotController.getFPGATime();
         * if(microsecondTimestamp-)
         * myBatteryLog.update(batteryVoltage,microsecondTimestamp);
         */

    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        m_robotContainer.leds.setRGB(255, 0, 0);
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);

        }

    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();

        }

    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        try {
            // Get current pose safely
            if (m_robotContainer != null && m_robotContainer.swerve != null) {
                // Pose2d currentPose = m_robotContainer.swerve.getPose();
                // Pose2d tagPose = m_robotContainer.vision.getPoseOfClosestReef();
                // ... rest of pose handling ...
            }

            // Get joystick input safely
            if (driver != null) {
                double speed = driver.getLeftY();
                if (joystickLog != null) {
                    joystickLog.append(speed);
                }
            }
        } catch (Exception e) {
            ElasticDashboard.putString("Robot/teleopPeriodic Error", e.toString());

        }

    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
        m_robotContainer.leds.setRGB(0, 255, 0);
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        try {
            if (m_robotContainer != null && m_robotContainer.getLeds() != null) {
                m_robotContainer.getLeds().Rainbow();
            }
        } catch (Exception e) {
            ElasticDashboard.putString("Robot/testPeriodic Error", e.toString());
        }
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
    }


}
