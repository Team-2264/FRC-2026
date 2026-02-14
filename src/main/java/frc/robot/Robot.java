// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;

import org.littletonrobotics.junction.Logger;

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

    // Log entries
    private DoubleLogEntry joystickLog;
    private DoubleLogEntry batteryVoltageLog;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();

        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        // Record metadata
        Logger.recordMetadata("ProjectName", "Wayzata2026Test");
        Logger.recordMetadata("RobotMACAddress", "10");
        Logger.recordMetadata("RobotInstance", "Trojans");
        Logger.recordMetadata("BuildDate", "11/12/25");

        DataLogManager.start();
        // Set up custom log entries
        DataLog log = DataLogManager.getLog();
        DataLogManager.log("Testing Wayzata");

        batteryVoltageLog = new DoubleLogEntry(log, "/BatteryVoltage");
        addPeriodic(() -> {
                batteryVoltageLog.update(RobotController.getBatteryVoltage(), RobotController.getFPGATime()); 
            }, 2.00, 0.00);

        joystickLog = new DoubleLogEntry(log, "/driver/leftStick");
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
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler. This is responsible for polling buttons, adding
        // newly-scheduled
        // commands, running already-scheduled commands, removing finished or
        // interrupted commands,
        // and running subsystem periodic() methods. This must be called from the
        // robot's periodic
        // block in order for anything in the Command-based framework to work.
        try {
            CommandScheduler.getInstance().run();
            if (m_robotContainer != null) {
                m_robotContainer.robotPeriodic();
            }
        } catch (Exception e) {
            SmartDashboard.putString("Robot/robotPeriodic Error", e.toString());
        }
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
                // ... rest of pose handling ...
            }

            // Get joystick input safely
            var driverController = m_robotContainer.getDriverController();
            if (driverController != null) {
                double speed = driverController.getLeftY();
                if (joystickLog != null) {
                    joystickLog.append(speed);
                }
            }
        } catch (Exception e) {
            SmartDashboard.putString("Robot/teleopPeriodic Error", e.toString());
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
            SmartDashboard.putString("Robot/testPeriodic Error", e.toString());
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
