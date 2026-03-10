package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ManualMotors;

/**
 * Command: while held, rotate the rotator motor +30° and spin the other motor at a
 * constant speed. On release, stop the spinner and command the rotator back to
 * its original position.
 */
public class RotateAndSpinWhileHeld extends Command {
    private final ManualMotors motors;
    private final double spinDuty;
    private final double degrees;
    private double originalRevs;

    public RotateAndSpinWhileHeld(ManualMotors motors, double spinDuty, double degrees) {
        this.motors = motors;
        this.spinDuty = spinDuty;
        this.degrees = degrees;
        addRequirements(motors);
    }

    @Override
    public void initialize() {
        // record original motor position (in rotations) and start actions
        originalRevs = motors.getRotatorPositionRevs();
        // rotate immediately to +degrees (converted to rotations)
        motors.rotateToPositionRevs(originalRevs + (degrees / 360.0));
        // start spinner at duty
        motors.startSpinner(spinDuty);
    }

    @Override
    public void execute() {
        // nothing to do continuously; motors handle the motion
    }

    @Override
    public void end(boolean interrupted) {
        // Stop spinner immediately
        motors.stopSpinner();
        // Command rotator back to original position; allow it to finish
        motors.rotateToPositionRevs(originalRevs);
    }

    @Override
    public boolean isFinished() {
        // This command is intended to be used with whileTrue/whileHeld and should
        // therefore not finish on its own.
        return false;
    }
}
