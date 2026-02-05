// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.PS5Controller;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

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
    private SparkMax m_motor;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Start Logging
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.
        m_robotContainer = new RobotContainer();
        m_motor = new SparkMax(5, MotorType.kBrushless);

        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());

        // Record metadata
        Logger.recordMetadata("ProjectName", "Wayzata2026Test");
        Logger.recordMetadata("RobotMACAddress", "10");
        Logger.recordMetadata("RobotInstance", "Trojans");
        Logger.recordMetadata("BuildDate", "11/12/25");

        alertTest.set(true); // Show alertTest

        DataLogManager.start();
        // Set up custom log entries
        DataLog log = DataLogManager.getLog();
        DataLogManager.log("Testing Wayzata");
        myBooleanLog = new BooleanLogEntry(log, "/my/boolean");
        myBatteryLog = new DoubleLogEntry(log, "/my/BatteryVoltage");
        myStringLog = new StringLogEntry(log, "/my/string");

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
            SmartDashboard.putString("Robot/teleopPeriodic Error", e.toString());
            m_motor.set(0.5);
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
