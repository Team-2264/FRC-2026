package frc.lib.motors;

import java.util.OptionalInt;

public class NeoConfiguration {
    public int canId;
    
    public double kP = 0.5;
    public double kI = 0.001;
    public double kD = 0;

    public double outputRangeLower = -0.05;
    public double outputRangeUpper = 0.05;

    public boolean motorInvertDirection = false;
    public boolean breakMode = false;
    public boolean brushedMotor = false;

    public OptionalInt followMotor = OptionalInt.empty();
    public boolean followInvert = false;

    public OptionalInt smartCurrentLimit = OptionalInt.empty();

    public boolean hasAbsoluteEncoderConfig = false;
    public boolean InvertAbsoluteEncoder = false;

    /**
     * Creates a Neo Motor configuration using default values.
     */
    public NeoConfiguration(int canId) {
        this.canId = canId;
    }

    /**
     * Configure the spark max to drive a brushed motor (default is brushless)
     */
    public NeoConfiguration withBrushedMotor(boolean brushedMotor) {
        this.brushedMotor = brushedMotor;
        return this;
    }

    public NeoConfiguration withMotorInvert(boolean invert) {
        this.motorInvertDirection = invert;
        return this;
    }

    /**
     * Sets the Neo to brake when it isn't being controlled.
     */
    public NeoConfiguration withBrakeMode(boolean brake_mode) {
        this.breakMode = brake_mode;
        return this;
    }

    /**
     * Sets the voltage of the Neo to the voltage of a different Neo.
     */
    public NeoConfiguration withFollowMotor(int canId) {
        this.followMotor = OptionalInt.of(canId);
        return this;
    }

    /**
     * Sets the voltage of the Neo to the voltage of a different Neo.
     */
    public NeoConfiguration withFollowMotor(int canId, boolean invert) {
        this.followMotor = OptionalInt.of(canId);
        this.followInvert = invert;
        return this;
    }

    /**
     * Sets the PID kP value.
     */
    public NeoConfiguration withKP(double kP) {
        this.kP = kP;
        return this;
    }

    /**
     * Sets the PID kI value.
     */
    public NeoConfiguration withKI(double kI) {
        this.kI = kI;
        return this;
    }

    /**
     * Sets the PID kD value.
     */
    public NeoConfiguration withKD(double kD) {
        this.kD = kD;
        return this;
    }

    /**
     * Sets allowable output range of the PID loop.
     */
    public NeoConfiguration withOutputRange(double lower, double upper) {
        this.outputRangeUpper = upper;
        this.outputRangeLower = lower;
        return this;
    }

    /**
     * Sets current limit for motor
     */
    public NeoConfiguration withSmartCurrentLimit(int currentLimitAmps) {
        this.smartCurrentLimit = OptionalInt.of(currentLimitAmps);
        return this;
    }

    /**
     * Sets the absolute encoder config
     */
    public NeoConfiguration withAbsoluteEncoder(boolean inverted)
    {
        this.hasAbsoluteEncoderConfig = true;
        this.InvertAbsoluteEncoder = inverted;
        return this;
    }
}
