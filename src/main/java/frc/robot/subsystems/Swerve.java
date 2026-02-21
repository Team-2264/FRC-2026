package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.enums.SwerveStatus;
import frc.lib.Math2264;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.util.datalog.DataLog;

/**
 * The Swerve class represents a swerve drive subsystem for a robot.
 * This class handles swerve modules, odometry, and gyro integration.
 * 
 */
public class Swerve extends SubsystemBase {
    private final Field2d field;
    private Pigeon2 pidgey;

    private SwerveModule[] swerveModules;

    private boolean turboModeStatus = false;
    private SwerveStatus swerveStatus = SwerveStatus.TELEOP;

    private boolean fieldRelative = true;

    private PIDController xPID = new PIDController(Constants.Swerve.translationKp, Constants.Swerve.translationKi,
            Constants.Swerve.translationKd);
    private PIDController yPID = new PIDController(Constants.Swerve.translationKp, Constants.Swerve.translationKi,
            Constants.Swerve.translationKd);
    private PIDController rotationController = new PIDController(Constants.Swerve.rotationKp,
            Constants.Swerve.rotationKi, Constants.Swerve.rotationKd);

    private PIDController rotationLockController = new PIDController(Constants.Swerve.rotationLockKP,
            Constants.Swerve.rotationLockKI, Constants.Swerve.rotationLockKD);
    private Optional<Translation2d> lockedOnto = Optional.empty();

    private final SwerveDrivePoseEstimator poseEstimator;
    DoubleLogEntry Module0SpeedLog;
    DoubleLogEntry Module0AngleLog;
    DoubleLogEntry Module1SpeedLog;
    DoubleLogEntry Module1AngleLog;
    DoubleLogEntry Module2SpeedLog;
    DoubleLogEntry Module2AngleLog;
    DoubleLogEntry Module3SpeedLog;
    DoubleLogEntry Module3AngleLog;

    private DoubleLogEntry gyroYawLog;
    private DoubleLogEntry gyroPitchLog;
    private DoubleLogEntry gyroRollLog;
    private DoubleLogEntry gyroYawRateLog;

    // ── AdvantageScope 3D struct publishers (published to NetworkTables) ──
    private final StructPublisher<Pose3d> robotPose3dPub;
    private final StructArrayPublisher<SwerveModuleState> measuredStatesPub;
    private final StructArrayPublisher<SwerveModuleState> desiredStatesPub;

    // Store desired states for logging
    private SwerveModuleState[] lastDesiredStates = new SwerveModuleState[4];

    // Cached Pigeon2 status signals (batched refresh is faster & more consistent)
    private StatusSignal<?> yawSignal;
    private StatusSignal<?> pitchSignal;
    private StatusSignal<?> rollSignal;
    private StatusSignal<?> yawRateSignal;

    // Cycle counter for throttling SmartDashboard (debug text every 5th cycle =
    // 100ms)
    private int periodicCounter = 0;

    // Alert Mod0Alert = new Alert()

    /**
     * Constructs a Swerve subsystem instance.
     */
    public Swerve() {
        // Init pigeon
        pidgey = new Pigeon2(Constants.Swerve.pigeonID);
        pidgey.getConfigurator().apply(new Pigeon2Configuration());
        zeroGyro();

        // Init Modules
        swerveModules = new SwerveModule[] {
                new SwerveModule(0,
                        Constants.Swerve.Mod0.driveMotorID,
                        Constants.Swerve.Mod0.angleMotorID,
                        Constants.Swerve.Mod0.angleEncoderID,
                        Constants.Swerve.Mod0.angleOffset),

                new SwerveModule(1,
                        Constants.Swerve.Mod1.driveMotorID,
                        Constants.Swerve.Mod1.angleMotorID,
                        Constants.Swerve.Mod1.angleEncoderID,
                        Constants.Swerve.Mod1.angleOffset),

                new SwerveModule(2,
                        Constants.Swerve.Mod2.driveMotorID,
                        Constants.Swerve.Mod2.angleMotorID,
                        Constants.Swerve.Mod2.angleEncoderID,
                        Constants.Swerve.Mod2.angleOffset),

                new SwerveModule(3,
                        Constants.Swerve.Mod3.driveMotorID,
                        Constants.Swerve.Mod3.angleMotorID,
                        Constants.Swerve.Mod3.angleEncoderID,
                        Constants.Swerve.Mod3.angleOffset),

        };

        DataLogManager.start();
        DataLog log = DataLogManager.getLog();
        Module0SpeedLog = new DoubleLogEntry(log, "/my/SpeedModule0");

        poseEstimator = new SwerveDrivePoseEstimator(Constants.Swerve.swerveKinematics, getGyroAngle(),
                getModulePositions(), Constants.Swerve.initialPose);

        gyroYawLog = new DoubleLogEntry(log, "/swerve/gyro/yaw");
        gyroPitchLog = new DoubleLogEntry(log, "/swerve/gyro/pitch");
        gyroRollLog = new DoubleLogEntry(log, "/swerve/gyro/roll");
        gyroYawRateLog = new DoubleLogEntry(log, "/swerve/gyro/yawRate");

        // Cache Pigeon2 status signals for batched refresh (lower latency, less CAN bus
        // chatter)
        yawSignal = pidgey.getYaw();
        pitchSignal = pidgey.getPitch();
        rollSignal = pidgey.getRoll();
        yawRateSignal = pidgey.getAngularVelocityZWorld();

        // ── AdvantageScope 3D: publish structs to NetworkTables ──
        var nt = NetworkTableInstance.getDefault();
        robotPose3dPub = nt.getStructTopic("SmartDashboard/Swerve/RobotPose3d", Pose3d.struct).publish();
        measuredStatesPub = nt.getStructArrayTopic("SmartDashboard/Swerve/MeasuredStates", SwerveModuleState.struct)
                .publish();
        desiredStatesPub = nt.getStructArrayTopic("SmartDashboard/Swerve/DesiredStates", SwerveModuleState.struct)
                .publish();

        // Init desired states to zero
        for (int i = 0; i < 4; i++) {
            lastDesiredStates[i] = new SwerveModuleState();
        }

        // Configure AutoBuilder
        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeeds,
                (speeds, feedforwards) -> drive(speeds),
                Constants.Swerve.pathFollowerConfig, // controller
                Constants.Swerve.config, // add robot configuration here
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);

        // Init Field
        field = new Field2d();

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((Pose2d pose) -> {
            field.setRobotPose(pose);

        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((Pose2d pose) -> {
            field.getObject("target pose").setPose(pose);

        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((List<Pose2d> poses) -> {
            field.getObject("path").setPoses(poses);

        });

    }

    public void lockOnto(Translation2d translation2d) {
        rotationLockController.reset();

        lockedOnto = Optional.of(translation2d);

    }

    public void unlock() {
        lockedOnto = Optional.empty();

    }

    public boolean locked() {
        return lockedOnto.isPresent();

    }

    /**
     * Drives the swerve drive based on field-relative {@link ChassisSpeeds}.
     */
    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        setModuleStates(swerveModuleStates);
    }

    public void setFieldRelative(boolean value) {
        fieldRelative = value;
    }

    public boolean getFieldRelative() {
        return fieldRelative;
    }

    /**
     * Drives the swerve drive based on translation and rotation inputs.
     *
     * @param translation The translation vector representing movement in x and y
     *                    directions.
     * @param rotation    The rotation value for the swerve drive.
     */
    public void drive(Translation2d translation, double rotation) {
        if (lockedOnto.isPresent()) {
            swerveStatus = SwerveStatus.LOCKED;

            Pose2d robot_pose = getPose();

            Translation2d look_vector = lockedOnto.get().minus(robot_pose.getTranslation()); // Vector from robot to
                                                                                             // target
            Rotation2d look_rotation = new Rotation2d(look_vector.getX(), look_vector.getY())
                    .rotateBy(new Rotation2d(Math.PI)); // Rotation of vector

            double targetRotationOffest = MathUtil
                    .angleModulus(look_rotation.minus(robot_pose.getRotation()).getRadians());

            double targetRotation = robot_pose.getRotation().getRadians() + targetRotationOffest;

            if (Math.abs(targetRotationOffest) < Math.toRadians(0.5)) {
                rotation = 0;
            } else {
                rotation = rotationLockController.calculate(robot_pose.getRotation().getRadians(), targetRotation) * 3;
            }

        } else {
            swerveStatus = SwerveStatus.TELEOP;

        }

        rotation = Math2264.limitMagnitude(rotation, Constants.Swerve.maxAngularVelocity);
        Translation2d limitedTranslation = Math2264.limitMagnitude(translation, maximumSpeed());

        ChassisSpeeds chassisSpeeds = fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(limitedTranslation.getX(), limitedTranslation.getY(), rotation,
                        getGyroAngle())
                : new ChassisSpeeds(limitedTranslation.getX(), limitedTranslation.getY(), rotation);

        drive(chassisSpeeds);
    }

    /**
     * Will drive to a supplied Pose2d. Must be called repeatedly, in the
     * corresponding command.
     * 
     * @param targetPose The desired Pose2d to drive to.
     */
    public void driveToPose(Pose2d targetPose) {

        Pose2d currentPose = getPose();
        // Translation2d currentTranslation = currentPose.getTranslation();
        // Translation2d newTranslation = targetPose.getTranslation();
        // Rotation2d currentRotation = currentPose.getRotation();
        // Rotation2d newRotation = targetPose.getRotation();

        // // calculate new x and y movement
        // double xSpeed = xPID.calculate(currentTranslation.getX(),
        // newTranslation.getX());
        // double ySpeed = yPID.calculate(currentTranslation.getY(),
        // newTranslation.getY());

        // // calculate new rotation
        // double rotationSpeed =
        // rotationController.calculate(currentRotation.getRadians(),
        // newRotation.getRadians());

        // // limit the speed, so we don't break anything
        // xSpeed = Math2264.limitMagnitude(xSpeed, maximumSpeed());
        // ySpeed = Math2264.limitMagnitude(ySpeed, maximumSpeed());
        // rotationSpeed = Math2264.limitMagnitude(rotationSpeed,
        // Constants.Swerve.maxAngularVelocity);

        // //make chassisSpeeds, and drive!!
        // ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed,
        // rotationSpeed);
        // drive(chassisSpeeds);

        if (targetPose == null) {
            return;
        }

        double xSpeed = xPID.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yPID.calculate(currentPose.getY(), targetPose.getY());

        double goalSpeed = rotationLockController.calculate(currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians());

        double[] errors = { currentPose.getX() - targetPose.getX(),
                currentPose.getY() - targetPose.getY(),
                currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians() };
        double[] speeds = { xSpeed, ySpeed, goalSpeed };
        SmartDashboard.putString("LockOnErrors", Arrays.toString(errors));
        SmartDashboard.putString("LockOnSpeeds", Arrays.toString(speeds));

        drive(new Translation2d(xSpeed, ySpeed), goalSpeed);
    }

    /**
     * Get maximum speed based on turbo mode status
     */
    public double maximumSpeed() {
        if (turboModeStatus == true) {
            return Constants.Swerve.turboMaxSpeed;
        } else {
            return Constants.Swerve.maxSpeed;
        }
    }

    /**
     * Sets the desired states for swerve modules.
     *
     * @param desiredStates The array of desired swerve module states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        for (SwerveModule mod : swerveModules) {
            mod.setDesiredState(desiredStates[mod.moduleNumber]);
        }
        // Capture for AdvantageScope 3D logging
        lastDesiredStates = desiredStates;
    }

    /**
     * Gets the current pose (position and orientation) of the robot.
     *
     * @return The current robot pose.
     */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();

    }

    /**
     * Sets the pose (position and orientation) of the robot.
     *
     * @param pose The desired pose for the robot.
     */
    public void setPose(Pose2d pose) {
        poseEstimator.resetPosition(getGyroAngle(), getModulePositions(), pose);

    }

    /**
     * Gets the field-relative {@link ChassisSpeeds} for the robot.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Gets the current states of swerve modules.
     *
     * @return The array of current swerve module states.
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : swerveModules) {
            states[mod.moduleNumber] = mod.getState();
        }

        return states;

    }

    /**
     * Gets the positions of swerve modules.
     *
     * @return The array of swerve module positions.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : swerveModules) {
            positions[mod.moduleNumber] = mod.getPosition();
        }

        return positions;

    }

    /**
     * Resets the gyroscope's yaw to zero.
     */
    public void zeroGyro() {
        pidgey.setYaw(0);

    }

    public void setGyro() {
        pidgey.setYaw(180);
    }

    /**
     * Gets the current gyro angle (yaw) of the robot.
     *
     * @return The current gyro angle.
     */
    public Rotation2d getGyroAngle() {
        double yaw = pidgey.getYaw().refresh().getValueAsDouble();
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - yaw)
                : Rotation2d.fromDegrees(yaw);

    }

    /**
     * Generates a command that will follow a specified path.
     * 
     * @param pathName
     * @return
     */

    public Command followPathCommand(PathPlannerPath path) {
        return new FollowPathCommand(
                path,
                this::getPose, // Robot pose supplier
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                         // ChassisSpeeds
                Constants.Swerve.pathFollowerConfig, // PathFollowerConfig
                Constants.Swerve.config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;

                },
                this // Reference to this subsystem to set requirements
        );

    }

    /**
     * Turns on/off turbo mode
     */
    public void toggleTurboMode(boolean condition) {
        turboModeStatus = condition;
    }

    /**
     * Returns if turbo mode is on
     */
    public boolean getTurboStatus() {
        return turboModeStatus;
    }

    public boolean addedVision;

    /**
     * Adds a vision measurement to the pose estimator. This will correct the
     * robot's pose.
     * 
     * @param pose The estimated robot pose.
     */
    public void addVisionMeasurement(EstimatedRobotPose pose) {
        Pose2d measuredPose = pose.estimatedPose.toPose2d();

        double totalDistance = 0;
        for (PhotonTrackedTarget target : pose.targetsUsed) {
            totalDistance += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        double averageDistance = totalDistance / pose.targetsUsed.size();

        double devMultiplier = (1.0 / 50.0) * averageDistance;
        Matrix<N3, N1> standardDevs = Constants.Vision.visionStandardDevs.times(devMultiplier);

        poseEstimator.addVisionMeasurement(measuredPose, pose.timestampSeconds, standardDevs);
        addedVision = true;
    }

    @Override
    public void periodic() {
        // Update odometry every cycle (this is the core 50Hz loop — must stay fast)
        poseEstimator.update(getGyroAngle(), getModulePositions());

        Pose2d currentPose = getPose();

        // ── Batch-refresh all Pigeon2 signals in one CAN frame ──
        // This is faster than 4 separate .refresh() calls and ensures all values
        // come from the same CAN timestamp (no inter-sample skew = smoother 3D).
        BaseStatusSignal.refreshAll(yawSignal, pitchSignal, rollSignal, yawRateSignal);
        double yaw = yawSignal.getValueAsDouble();
        double pitch = pitchSignal.getValueAsDouble();
        double roll = rollSignal.getValueAsDouble();
        double yawRate = yawRateSignal.getValueAsDouble();

        // ── AdvantageScope 3D: Publish robot Pose3d (every cycle = smooth) ──
        Pose3d robotPose3d = new Pose3d(
                currentPose.getX(),
                currentPose.getY(),
                0.0,
                new Rotation3d(
                        Math.toRadians(roll),
                        Math.toRadians(pitch),
                        currentPose.getRotation().getRadians()));
        robotPose3dPub.set(robotPose3d);

        // ── AdvantageScope 3D: Publish swerve module states (every cycle) ──
        measuredStatesPub.set(getModuleStates());
        desiredStatesPub.set(lastDesiredStates);

        // Field2d for 2D field view (every cycle)
        field.setRobotPose(currentPose);
        SmartDashboard.putData(field);

        // ── Flush NT immediately so AdvantageScope gets data ASAP ──
        // Without this, data sits in the NT buffer up to 100ms before being sent.
        NetworkTableInstance.getDefault().flush();

        // Gyro data log (every cycle — these go to .wpilog file, cheap)
        gyroYawLog.append(yaw);
        gyroPitchLog.append(pitch);
        gyroRollLog.append(roll);
        gyroYawRateLog.append(yawRate);

        // ── Throttle SmartDashboard debug text to every 5th cycle (~100ms) ──
        // These string/number puts compete with struct data for NT bandwidth.
        periodicCounter++;
        if (periodicCounter >= 5) {
            periodicCounter = 0;

            SmartDashboard.putString("Swerve Status", swerveStatus.toString());
            SmartDashboard.putBoolean("added vision in swerve:", addedVision);

            SmartDashboard.putNumber("Gyro Yaw", yaw);
            SmartDashboard.putNumber("Gyro Pitch", pitch);
            SmartDashboard.putNumber("Gyro Roll", roll);
            SmartDashboard.putNumber("Gyro Yaw Rate", yawRate);

            for (SwerveModule mod : swerveModules) {
                SmartDashboard.putNumber("Mod " + mod.moduleNumber, mod.getState().angle.getDegrees());
            }
        }
    }
}