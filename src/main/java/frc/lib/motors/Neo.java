// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.motors;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;

/**
 * A wrapper around the NEO motors.
 */
public class Neo {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController pidNeo;
  private final SparkMaxConfig config;

  /**
   * Creates a Neo motor with the specified configuration
   */
  public Neo(NeoConfiguration c) {
    MotorType motorType = MotorType.kBrushless;

    if (c.brushedMotor) {
      motorType = MotorType.kBrushed;
    }

    motor = new SparkMax(c.canId, motorType);
    encoder = motor.getEncoder();
    pidNeo = motor.getClosedLoopController();
    config = new SparkMaxConfig();

    config.inverted(c.motorInvertDirection);

    if(c.breakMode) {
      config.idleMode(IdleMode.kBrake);
    } else {
      config.idleMode(IdleMode.kCoast);
    }

    if (c.smartCurrentLimit.isPresent()) {
      config.smartCurrentLimit(c.smartCurrentLimit.getAsInt());
    }


  
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(c.kP, c.kI, c.kD);
    config.closedLoop.feedForward.kV(c.kFF);
    config.closedLoop.outputRange(c.outputRangeLower, c.outputRangeUpper);
    config.closedLoop.iZone(0);

    if (c.hasAbsoluteEncoderConfig) {
      config.absoluteEncoder.setSparkMaxDataPortConfig();
      config.absoluteEncoder.inverted(c.InvertAbsoluteEncoder);
    }

    //config.encoder.velocityConversionFactor(1.0/60.0);
    if(c.followMotor.isPresent()) {
        config.follow(c.followMotor.getAsInt(), c.followInvert);
    }

    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /**
   * Starts rotating the motor at a specific speed.
   * @param speed The speed in the range of -1 to 1.
   */
  public void rotateAtSpeed(double speed) {
    motor.set(speed);
  }

  /**
   * Stops rotating the motor.
   */
  public void stop(){
    motor.stopMotor();
  }

  /**
   * Gets the position of the motor. This is reset to 0 when the robot is turned on.
   * @return The current rotation of the motor in rotations.
   */
  public double getPosition() {
    return encoder.getPosition();
  }

  /**
   * Sets the current position of the motor in revolutions
   */
  public void setPosition(double position_revs) {
    encoder.setPosition(position_revs);
  }

  /**
   * Gets the absolute encoder position, range is 0.0 - 1.0
   * @return absolute encoder position, range of 0.0 - 1.0
   */
  public double getAbsEncoderPosition() {
    return motor.getAbsoluteEncoder().getPosition();
  }

  /**
   * Gets the velocity of the motor.
   * @return The current velocity of the motor in rotations per second.
   */
  public double getVelocity() {
    return encoder.getVelocity();
  }

  /**
   * Sets the motor voltage in volts.
   */
  public void setVoltage(double volts) {
     motor.setVoltage(volts);
  }

  /**
   * Sets the target velocity for closed-loop control in RPM.
   * @param rpm The target RPM.
   */
  public void setVelocityRPM(double rpm) {
    pidNeo.setSetpoint(rpm, ControlType.kVelocity);
  }

  /**
   * Rotates the motor to a specific position relative to where it started.
   * @param position The angle to rotate to in rotations
   */
  public void rotateTo(double position){

    pidNeo.setSetpoint(position, ControlType.kPosition);
  }

  /**
   * Get tthe voltage that is being applied to the motor
   */
  public double getAppliedOutput() {
    return motor.getAppliedOutput();
  }

  /**
   * Get the current being applied to the motor
   */
  public double getOutputCurrent() {
    return motor.getOutputCurrent();
  }
  
}
