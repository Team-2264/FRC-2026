// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.lib.FieldPose;
import frc.lib.motors.NeoConfiguration;
import frc.lib.motors.TalonFxConfiguration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other purpose.
 * All constants should be declared globally (i.e. public static). 
 * Do not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    /**
     * Operator class holds constants related to operator input.
     */


    public static class Operator {
        public static final int controllerPort = 0;
        public static final int controller2Port = 1;
        public static final double stickDeadband = 0.1;

    }
    
    /**
     * Vision class holds constants related to the vision system.
     */
    public static final class Vision {
        public static final String cameraName = "apriltag";

        public static final Matrix<N3, N1> visionStandardDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(17));
        public static double singleTargetMultiplier = 0;
        
        // Robot to Camera Transform
        public static final Transform3d robotToCamera = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(11.75), 
                Units.inchesToMeters(0), 
                Units.inchesToMeters(7.25)
            ),
            new Rotation3d(0, 0, 0)
            
        );

    }

    /**
     * LedStrip class holds constants related to the LED strip.
     */
    public static final class LedStrip {
        public static final int pwmPort = 0;
        public static final int numLeds = 8;
        public static final float scaleFactor = 0.5f;
    }

    /**
     * Swerve class holds constants related to the swerve drive system.
     */
    public static final class Swerve {
        public static final int pigeonID = 15;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final Pose2d initialPose = new Pose2d(
            Units.inchesToMeters(39.3701 * 2), // x
            Units.inchesToMeters(218.42), // y
            new Rotation2d()
        );

        //new PIDConstants(0.220019, 0.0, 0.0), // Translation PID constants
        //new PIDConstants(1.201173, 0.0, 0.024023) // Rotation PID constants

        public static final double translationKp = 2.0;
        public static final double translationKd = 0.0;
        public static final double translationKi = 0.0;

        public static final double rotationKp = 0.001;//0.22019;
        public static final double rotationKd = 0.0;
        public static final double rotationKi = 0.0;

     
        // Drivetrain Constants
        public static final double trackWidth = Units.inchesToMeters(27);
        public static final double wheelBase = Units.inchesToMeters(32.5);
        public static final double wheelDiameter = Units.inchesToMeters(4.05);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.5;
        public static final double closedLoopRamp = 0.5;

        public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0)
            
        );

        public static final frc.lib.motors.TalonFxConfiguration angleMotorConfig = new TalonFxConfiguration()
            .withContinuousCurrentLimit(25)
            .withPeakCurrentLimit(40)
            .withPeakCurrentDuration(0.1)
            .withPID(
                1.201173, // kP
                0.0, // kI
                0.024023, // kD
                0.0 // kF
            )
            .withoutNeutralModeBrake();

        public static final frc.lib.motors.TalonFxConfiguration driveMotorConfig = new TalonFxConfiguration()
            .withContinuousCurrentLimit(35)
            .withPeakCurrentLimit(60)
            .withPeakCurrentDuration(0.1)
            .withPID(
                0.20019, // kP
                0.0, // kI
                0.0, // kD
                0.0 // kF
            )
            .withNeutralModeBrake()
            .withOpenLoopRamp(0.5)
            .withClosedLoopRamp(0.5);

        // Drive Motor Characterization Values
        public static final double driveKS = (0.667 / 12); // divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        // Swerve Profiling Values
        public static final double maxSpeed = 1.0; //2.5; // meters per second
        public static final double maxAngularVelocity = Math.PI / 2; // Math.PI; // radians per second (rad/s)
        
        // Turbo mode values
        public static final double turboMaxSpeed = 4.5;
        public static final double turboMaxAngularVelocity = 0.9 * Math.PI;

        // Module Specific Constants
        // Front Left Module - Module 0
        public static final class Mod0 {
            public static final int driveMotorID = 40;
            public static final int angleMotorID = 41;
            public static final int angleEncoderID = 48;

            public static final double angleOffset = 95; //-0.4384765625 * 360 + 3; // 0.244873 * 360;

        }

        // Front Right Module - Module 1
        public static final class Mod1 {
            public static final int driveMotorID = 42;
            public static final int angleMotorID = 43;
            public static final int angleEncoderID = 49;

            public static final double angleOffset = -0.079833984375 * 360 + 42 + 14 + 180; // 0.300781 * 360;

        }

        // Back Right Module - Module 2
        public static final class Mod2 {
            public static final int driveMotorID = 46;
            public static final int angleMotorID = 47;
            public static final int angleEncoderID = 50;

            public static final double angleOffset =  0.156005859375 * 360 + 55 + 13 ; // 0.210693 * 360;
        }

        // Back Left Module - Module 3
        public static final class Mod3 {
            public static final int driveMotorID = 44;
            public static final int angleMotorID = 45;
            public static final int angleEncoderID = 51;

            public static final double angleOffset = -0.424072265625 * 360 - 55 + 180; // 0.008057 * 360;
        }

        // Autonomous 
        public static final PPHolonomicDriveController pathFollowerConfig = 
            new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(0.220019, 0.0, 0.0), // Translation PID constants
                new PIDConstants(1.201173, 0.0, 0.024023) // Rotation PID constants
                //10, // Max module speed, in m/s - previously 4.5
                //0.395 // Drive base radius in meters. Distance from robot center to furthest module. - previously 0.4
                // pathplanner removed replanning
                // new ReplanningConfig() // Default path replanning config. See the API for the options here
            );

        public static RobotConfig config; 
        static{
            try {
                config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                // Handle exception as needed
                e.printStackTrace();
            }
        }
        

        // Robot rotation locking
        public static final double rotationLockKP = 2.0;
        public static final double rotationLockKI = 0.01;
        public static final double rotationLockKD = 0;
    }

    /**
     * measurements for game pieces.
     */
    public static final class gamePieces{
        public static final double coralStationHeight = 37.44;
        public static final double reefDistance = 12.94;
        public static final double L1Height = 17.88;
        public static final double L2Height = 31.72;
        public static final double L3Height = 47.59;
        public static final double L4Height = 71.87;


    }

    /**
     * 2026 AprilTag field poses (meters, from 2026-rebuilt-welded.json).
     * Coral station tags: 1,6,7,12 (Red), 17,22,23,28 (Blue)
     * Barge tags: 13,14,15,16 (Red wall), 29,30,31,32 (Blue wall)
     */
    public static final class TagLocation{
        //--------------Red Side Coral Stations--------------

        /** Tag 1 - Red Coral Station (top, faces 180°) */
        public static FieldPose tag1 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(11.8779798, 7.4247756, 0.889),
            new Rotation3d(0, 0, Math.PI) // faces -X (180°)
        ));

        /** Tag 6 - Red Coral Station (bottom, faces 180°) */
        public static FieldPose tag6 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(11.8779798, 0.6444996, 0.889),
            new Rotation3d(0, 0, Math.PI)
        ));

        /** Tag 7 - Red Coral Station (bottom, faces 0°) */
        public static FieldPose tag7 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(11.9528844, 0.6444996, 0.889),
            new Rotation3d(0, 0, 0) // faces +X (0°)
        ));

        /** Tag 12 - Red Coral Station (top, faces 0°) */
        public static FieldPose tag12 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(11.9528844, 7.4247756, 0.889),
            new Rotation3d(0, 0, 0)
        ));

        //--------------Blue Side Coral Stations--------------

        /** Tag 17 - Blue Coral Station (bottom, faces 0°) */
        public static FieldPose tag17 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(4.6630844, 0.6444996, 0.889),
            new Rotation3d(0, 0, 0)
        ));

        /** Tag 22 - Blue Coral Station (top, faces 0°) */
        public static FieldPose tag22 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(4.6630844, 7.4247756, 0.889),
            new Rotation3d(0, 0, 0)
        ));

        /** Tag 23 - Blue Coral Station (top, faces 180°) */
        public static FieldPose tag23 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(4.5881798, 7.4247756, 0.889),
            new Rotation3d(0, 0, Math.PI)
        ));

        /** Tag 28 - Blue Coral Station (bottom, faces 180°) */
        public static FieldPose tag28 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(4.5881798, 0.6444996, 0.889),
            new Rotation3d(0, 0, Math.PI)
        ));

        //--------------Red Barge Tags--------------

        /** Tag 13 - Red Barge (far wall, faces 180°) */
        public static FieldPose tag13 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(16.5333172, 7.4033126, 0.55245),
            new Rotation3d(0, 0, Math.PI)
        ));

        /** Tag 14 - Red Barge (far wall, faces 180°) */
        public static FieldPose tag14 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(16.5333172, 6.9715126, 0.55245),
            new Rotation3d(0, 0, Math.PI)
        ));

        /** Tag 15 - Red Barge (far wall, faces 180°) */
        public static FieldPose tag15 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(16.5329616, 4.3235626, 0.55245),
            new Rotation3d(0, 0, Math.PI)
        ));

        /** Tag 16 - Red Barge (far wall, faces 180°) */
        public static FieldPose tag16 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(16.5329616, 3.8917626, 0.55245),
            new Rotation3d(0, 0, Math.PI)
        ));

        //--------------Blue Barge Tags--------------

        /** Tag 29 - Blue Barge (near wall, faces 0°) */
        public static FieldPose tag29 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(0.0077470, 0.6659626, 0.55245),
            new Rotation3d(0, 0, 0)
        ));

        /** Tag 30 - Blue Barge (near wall, faces 0°) */
        public static FieldPose tag30 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(0.0077470, 1.0977626, 0.55245),
            new Rotation3d(0, 0, 0)
        ));

        /** Tag 31 - Blue Barge (near wall, faces 0°) */
        public static FieldPose tag31 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(0.0080772, 3.7457126, 0.55245),
            new Rotation3d(0, 0, 0)
        ));

        /** Tag 32 - Blue Barge (near wall, faces 0°) */
        public static FieldPose tag32 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(0.0080772, 4.1775126, 0.55245),
            new Rotation3d(0, 0, 0)
        ));
    }

    /**
     * 2026 AprilTag field poses for reef/cage (meters, from 2026-rebuilt-welded.json).
     * Red reef: tags 2-5 (inner), 8-11 (outer)
     * Blue reef: tags 18-21 (inner), 24-27 (outer)
     */
    public static final class ReefTagLocation{

        //--------------Red Reef (inner cage)--------------

        /** Tag 2 - Red Reef inner (faces 90°, +Y) */
        public static FieldPose tag2 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(11.9154194, 4.6380400, 1.12395),
            new Rotation3d(0, 0, Math.PI / 2) // 90°
        ));

        /** Tag 3 - Red Reef inner (faces 180°) */
        public static FieldPose tag3 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(11.3118646, 4.3902376, 1.12395),
            new Rotation3d(0, 0, Math.PI) // 180°
        ));

        /** Tag 4 - Red Reef inner (faces 180°) */
        public static FieldPose tag4 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(11.3118646, 4.0346376, 1.12395),
            new Rotation3d(0, 0, Math.PI) // 180°
        ));

        /** Tag 5 - Red Reef inner (faces -90° / 270°) */
        public static FieldPose tag5 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(11.9154194, 3.4312352, 1.12395),
            new Rotation3d(0, 0, -Math.PI / 2) // -90°
        ));

        //--------------Red Reef (outer cage)--------------

        /** Tag 8 - Red Reef outer (faces -90° / 270°) */
        public static FieldPose tag8 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(12.2710194, 3.4312352, 1.12395),
            new Rotation3d(0, 0, -Math.PI / 2) // -90°
        ));

        /** Tag 9 - Red Reef outer (faces 0°) — CENTER REEF SHOOTING TAG */
        public static FieldPose tag9 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(12.5191774, 3.6790376, 1.12395),
            new Rotation3d(0, 0, 0) // 0°
        ));

        /** Tag 10 - Red Reef outer (faces 0°) — CENTER REEF SHOOTING TAG */
        public static FieldPose tag10 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(12.5191774, 4.0346376, 1.12395),
            new Rotation3d(0, 0, 0) // 0°
        ));

        /** Tag 11 - Red Reef outer (faces 90°, +Y) */
        public static FieldPose tag11 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(12.2710194, 4.6380400, 1.12395),
            new Rotation3d(0, 0, Math.PI / 2) // 90°
        ));

        //--------------Blue Reef (inner cage)--------------

        /** Tag 18 - Blue Reef inner (faces -90° / 270°) */
        public static FieldPose tag18 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(4.6256194, 3.4312352, 1.12395),
            new Rotation3d(0, 0, -Math.PI / 2) // -90°
        ));

        /** Tag 19 - Blue Reef inner (faces 0°) */
        public static FieldPose tag19 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(5.2291742, 3.6790376, 1.12395),
            new Rotation3d(0, 0, 0) // 0°
        ));

        /** Tag 20 - Blue Reef inner (faces 0°) */
        public static FieldPose tag20 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(5.2291742, 4.0346376, 1.12395),
            new Rotation3d(0, 0, 0) // 0°
        ));

        /** Tag 21 - Blue Reef inner (faces 90°, +Y) */
        public static FieldPose tag21 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(4.6256194, 4.6380400, 1.12395),
            new Rotation3d(0, 0, Math.PI / 2) // 90°
        ));

        //--------------Blue Reef (outer cage)--------------

        /** Tag 24 - Blue Reef outer (faces 90°, +Y) */
        public static FieldPose tag24 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(4.2700194, 4.6380400, 1.12395),
            new Rotation3d(0, 0, Math.PI / 2) // 90°
        ));

        /** Tag 25 - Blue Reef outer (faces 180°) */
        public static FieldPose tag25 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(4.0218614, 4.3902376, 1.12395),
            new Rotation3d(0, 0, Math.PI) // 180°
        ));

        /** Tag 26 - Blue Reef outer (faces 180°) */
        public static FieldPose tag26 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(4.0218614, 4.0346376, 1.12395),
            new Rotation3d(0, 0, Math.PI) // 180°
        ));

        /** Tag 27 - Blue Reef outer (faces -90° / 270°) */
        public static FieldPose tag27 = FieldPose.fromWpiBlue(new Pose3d(
            new Translation3d(4.2700194, 3.4312352, 1.12395),
            new Rotation3d(0, 0, -Math.PI / 2) // -90°
        ));

    }


    // Elevator
    public static final class ElevatorConfig {
        public static final NeoConfiguration[] neoConfigs = new NeoConfiguration[] {
            new NeoConfiguration(20)
                .withBrakeMode(true)
                .withMotorInvert(false)
                .withSmartCurrentLimit(40)
                .withKP(0.0)
                .withKI(0.0)
                .withKD(0.0),
            new NeoConfiguration(21)
                .withBrakeMode(true)
                .withSmartCurrentLimit(40)
                .withFollowMotor(20, true)
                // The remaining parameters don't need to be set due to being a follower.
        };

        public static final int homeLimitSwitchId = 3;
        public static final boolean homeLimitSwitchActiveHigh = true;

        public static final double homingCurrentThresholdAmps = 12.0;

        public static final double chain_pitch_inches = 0.25;
        public static final int chain_sprocket_teeth = 22;
        public static final double chain_inches_per_rev = chain_pitch_inches * chain_sprocket_teeth;    // 5.5 inches

        public static final double gearbox_ratio = 3.0 * 4.0;   // Gearbox is a MaxPlanetary with a 3:1 and 4:1 cartridge -> 12:1 gear ratio

        public static final int num_stages = 2;

        public static final double final_inches_per_motor_rev = (chain_inches_per_rev / gearbox_ratio) * num_stages;   // 0.9164 inches per motor revolution

        public final static double feedForwardKs = 0; 
        public final static double feedForwardKg = 0.0325 * 12.0;
        public final static double feedForwardKv = 0;

        public final static double pidKp = 0.095 * 12.0;
        public final static double pidKi = 0.001 * 12.0;
        public final static double pidKd = 0;

        public final static double maxVelocityIps = 50.0;
        public final static double accelerationIpsPs = 800.0;

        public static final double homingVoltage = 0.05 * 12;
        public static final double homePositionInches = 0.1;     
        public static final double minPositionInches = 0.0;
        public static final double maxPositionInches = 52.0;

        public static final double maxVoltage = 12.0;
    }

    public final static class CoralConfig {
        public static final int intakeMotorID = 31;
        public static final int angleMotorID = 30;

        public static final double intakeMotorIntakeDuty = 0.2;
        public static final double intakeMotorReleaseDuty = -0.15; // -0.50;
        public static final double intakeMotorReleaseDutyL1 = -0.50;

        public static final double intakeMotorIntakeStallThresholdAmps = 15.0;

        public static final double angleOpenLoopMoveVolts = 1.0;

        public static final double angleGearRatio = 25.0;

        public static final NeoConfiguration intakeNeoConfig = new NeoConfiguration(intakeMotorID)
            .withKP(0.0)
            .withKI(0.0)
            .withKD(0.0)
            .withBrakeMode(false);

        public static final NeoConfiguration angleNeoConfig = new NeoConfiguration(angleMotorID)
            .withKP(0.0)
            .withKI(0.0)
            .withKD(0.0)
            .withBrakeMode(true)
            .withMotorInvert(true)
            .withAbsoluteEncoder(true);

        public final static double angleFeedForwardKs = 0; 
        public final static double angleFeedForwardKg = 0.250;
        public final static double angleFeedForwardKv = 0;

        public final static double anglePidKp = 0.15;
        public final static double anglePidKi = 0.00;
        public final static double anglePidKd = 0;

        public final static double angleMaxDegreesPerSecond = 100.0;
        public final static double angleAccelDegreesPerSecondPerSecond = 7200.0;

        public final static double angleMotorMaxDegreesPerSecond = angleMaxDegreesPerSecond * angleGearRatio;
        public final static double angleMotorAccelDegreesPerSecondPerSecond = angleAccelDegreesPerSecondPerSecond * angleGearRatio;

        public static final double angleEncoderOffsetZeroToOne = -0.07;
        public static final double angleEncoderOffsetDegrees = 23.0;

        public static final double angleMaxVoltage = 6.0;
        public static final double angleMinAngle = -85.0;
        public static final double angleMaxAngle = 65.0;

        // Coral elevator positions
        public static final double elevatorL2 = 12.0;
        public static final double elevatorL3 = 27.0;
        public static final double elevatorL4 = 52.0;
        public static final double elevatorIntake = 10.0;
        public static final double elevatorHome = 0.25;

        // Coral arm angles
        public static final double angleL2 = -20.0;
        public static final double angleL3 = -20.0;
        public static final double angleL4 = -17.0;
        public static final double angleIntake = 30.0;
        public static final double angleHome = 55.0;
        
    }

    public static final class AlgaeConfig {
        public static final int motorId = 32;
        public static final double intakeDuty = -0.50;
        public static final double outtakeDuty = 0.50;
        public static final double holdDuty = -0.10;

        public static final double motorIntakeStallThresholdAmps = 7.0; // TODO: will need to be adjusted

        public static final NeoConfiguration neoConfigs = 
            new NeoConfiguration(motorId)
            .withBrushedMotor(true)
            .withBrakeMode(true)
            .withMotorInvert(false)
            .withSmartCurrentLimit(20)
            .withKP(0.0)
            .withKI(0.0)
            .withKD(0.0);

        // High algae
        public static final double elevatorHighA = 41.25;

    }

    public static class ClimberConfig {
        public static final int motorId = 25;

        public static final double deployTimeSeconds = 4.0;

        public static final NeoConfiguration neoConfig = new NeoConfiguration(motorId).withBrakeMode(true);

        public static final double moveUpDrive = 0.50;
        public static final double moveDownDrive = -0.75;
    }
}
